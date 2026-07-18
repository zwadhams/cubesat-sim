/*
 * EPS flight software — C port of the power distribution unit.
 *
 * Style notes, bare-metal PDU flavored: one superloop, static fixed-size
 * buffers, no heap at all (the OBC's getline mallocs; a PDU micro would
 * not), a fixed load-switch table walked in a fixed order, and no
 * dependence on wall-clock time or randomness. Behavior must be
 * bit-identical to the Python reference EPS (subsystems/eps.py): the
 * equivalence test in tests/test_remote_eps.py compares whole flights.
 *
 * Logic: EPS is the single authority over the cmd/loads switches. The OBC and
 * thermal control *request* load states; EPS honors requests unless its
 * own shed threshold trips on the voltage-derived state-of-charge
 * estimate, in which case non-essential loads are vetoed until the
 * battery recovers.
 *
 * Number formatting: values that land exactly on an integer (the SoC
 * estimate clamps to 1.0 at full charge) must print as "1.0", not "1" —
 * the bridge JSON-parses the reply, and an int where the reference emits
 * a float would diverge in the flight recording.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_MAX_LEN 65536

static const double V_EMPTY = 6.0;
static const double V_FULL = 8.4;
static const double SHED_SOC = 0.15;
static const double RESTORE_SOC = 0.30;

#define N_LOADS 3
static const char *LOAD_NAMES[N_LOADS] = {"adcs", "payload", "bat_heater"};
/* every load the EPS can veto is non-essential; table order is the
 * publish order and must match the reference dict order */

static char line[LINE_MAX_LEN];

/* -- state (the PDU's entire RAM footprint) ------------------------------- */
static int has_v = 0;
static double v_meas = 0.0;
static double solar_w = 0.0;
static double load_w = 0.0;
static int shedding = 0;
static int desired[N_LOADS] = {1, 0, 0};    /* adcs on until told otherwise */
static int commanded[N_LOADS] = {-1, -1, -1};  /* -1: never commanded */

static void emit_num(double v)
{
    if (fabs(v) < 1e15 && v == (double)(long long)v) {
        printf("%.1f", v);
    } else {
        printf("%.17g", v);
    }
}

static int find_double(const char *seg, const char *key, double *out)
{
    const char *p = strstr(seg, key);
    if (p == NULL) {
        return 0;
    }
    *out = strtod(p + strlen(key), NULL);
    return 1;
}

static int find_bool(const char *seg, const char *key, int *out)
{
    const char *p = strstr(seg, key);
    if (p == NULL) {
        return 0;
    }
    *out = strncmp(p + strlen(key), "true", 4) == 0;
    return 1;
}

/* read one newline-terminated line into the static buffer; returns 0 on
 * EOF. Oversized lines are truncated but consumed to the newline, so the
 * loop never desynchronizes from the frame stream. */
static int read_line(void)
{
    int ch;
    size_t n = 0;
    while ((ch = getchar()) != EOF && ch != '\n') {
        if (n < LINE_MAX_LEN - 1) {
            line[n++] = (char)ch;
        }
    }
    line[n] = '\0';
    return !(ch == EOF && n == 0);
}

static void step(void)
{
    /* walk the frame's messages in delivery order */
    const char *anchor = "\"topic\":\"";
    char *seg = strstr(line, anchor);
    while (seg != NULL) {
        seg += strlen(anchor);
        char *end = strstr(seg, anchor);
        char saved = 0;
        if (end != NULL) {
            saved = *end;
            *end = '\0';
        }

        double v;
        int b;
        if (strncmp(seg, "sensors/eps/battery_voltage\"", 28) == 0) {
            if (find_double(seg, "\"volts\":", &v)) {
                v_meas = v;
                has_v = 1;
            }
        } else if (strncmp(seg, "sensors/eps/solar_power\"", 24) == 0) {
            if (find_double(seg, "\"watts\":", &v)) {
                solar_w = v;
            }
        } else if (strncmp(seg, "sensors/eps/load_power\"", 23) == 0) {
            if (find_double(seg, "\"watts\":", &v)) {
                load_w = v;
            }
        } else if (strncmp(seg, "obc/request/loads\"", 18) == 0) {
            if (find_bool(seg, "\"adcs\":", &b)) {
                desired[0] = b;
            }
            if (find_bool(seg, "\"payload\":", &b)) {
                desired[1] = b;
            }
        } else if (strncmp(seg, "thermal/request/heater\"", 23) == 0) {
            if (find_bool(seg, "\"on\":", &b)) {
                desired[2] = b;
            }
        }

        if (end != NULL) {
            *end = saved;
        }
        seg = end;
    }

    if (!has_v) {  /* no telemetry yet: say nothing, like the reference */
        printf("{\"type\":\"out\",\"pub\":[],\"telemetry\":{},\"events\":[]}\n");
        fflush(stdout);
        return;
    }

    double soc_est = (v_meas - V_EMPTY) / (V_FULL - V_EMPTY);
    if (soc_est < 0.0) {
        soc_est = 0.0;
    }
    if (soc_est > 1.0) {
        soc_est = 1.0;
    }

    int shed_event = 0;   /* +1 shed, -1 restore */
    if (!shedding && soc_est < SHED_SOC) {
        shedding = 1;
        shed_event = 1;
    } else if (shedding && soc_est > RESTORE_SOC) {
        shedding = 0;
        shed_event = -1;
    }

    printf("{\"type\":\"out\",\"pub\":[");
    int first = 1;
    for (int i = 0; i < N_LOADS; ++i) {
        int target = shedding ? 0 : desired[i];
        if (commanded[i] != target) {
            if (!first) {
                printf(",");
            }
            first = 0;
            printf("{\"topic\":\"cmd/loads/%s\",\"data\":{\"on\":%s}}",
                   LOAD_NAMES[i], target ? "true" : "false");
            commanded[i] = target;
        }
    }
    if (!first) {
        printf(",");
    }
    printf("{\"topic\":\"eps/status\",\"data\":{\"soc_est\":");
    emit_num(soc_est);
    printf(",\"battery_v\":");
    emit_num(v_meas);
    printf(",\"solar_w\":");
    emit_num(solar_w);
    printf(",\"load_w\":");
    emit_num(load_w);
    printf(",\"shedding\":%s}}", shedding ? "true" : "false");

    printf("],\"telemetry\":{\"soc_est\":");
    emit_num(soc_est);
    printf(",\"shedding\":%.1f}", shedding ? 1.0 : 0.0);

    if (shed_event != 0) {
        printf(",\"events\":[{\"kind\":\"%s\",\"detail\":{\"soc_est\":",
               shed_event > 0 ? "load_shed" : "load_restore");
        emit_num(soc_est);
        printf("}}]");
    }
    printf("}\n");
    fflush(stdout);
}

int main(void)
{
    while (read_line()) {
        if (strstr(line, "\"type\":\"init\"") != NULL) {
            printf("{\"type\":\"ready\",\"subscribe\":[\"sensors/eps/*\","
                   "\"obc/request/loads\",\"thermal/request/heater\"]}\n");
            fflush(stdout);
        } else if (strstr(line, "\"type\":\"shutdown\"") != NULL) {
            break;
        } else if (strstr(line, "\"type\":\"step\"") != NULL) {
            step();
        }
    }
    return 0;
}
