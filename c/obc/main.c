/*
 * OBC flight software — C port of the mode manager + FDIR.
 *
 * Style notes, cFS-flavored: a superloop that blocks on the bus (here,
 * stdin), fixed message formats parsed with targeted scanners instead of a
 * general JSON library, no dynamic allocation in the loop body, and no
 * dependence on wall-clock time or randomness. Behavior must be
 * bit-identical to the Python reference OBC (subsystems/obc.py): the
 * equivalence test in tests/test_remote_obc.py holds us to that.
 *
 * Mode logic: NOMINAL runs the payload; SAFE sheds it. Transitions are
 * hysteresis-banded on the EPS's estimated state of charge.
 *
 * FDIR: gyro health watchdog. A latched sensor repeats its output word
 * exactly (healthy noise never does); an insane sensor reads rates no
 * real flight can reach. Either one, sustained, triggers an ADCS power
 * cycle — MAX_CYCLES per flight, then give up and leave it powered.
 *
 * Event details that are conceptually integers must print as integers
 * (%d): the bridge JSON-parses them, and 1 vs 1.0 would diverge from the
 * Python reference in the flight recording.
 */

#define _POSIX_C_SOURCE 200809L  /* getline */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MODE_NOMINAL 0
#define MODE_SAFE 1

static const double SAFE_ENTER_SOC = 0.25;
static const double SAFE_EXIT_SOC = 0.45;
static const double REASSERT_EVERY_S = 30.0;

static const double RATE2_LIMIT = 0.1225;  /* (0.35 rad/s)^2 ~ 20 deg/s */
static const int STUCK_TRIGGER = 5;
static const int INSANE_TRIGGER = 3;
static const double CYCLE_OFF_S = 20.0;
static const int MAX_CYCLES = 3;

/* Parse the number after the first occurrence of `key`, at or after `start`. */
static int find_double_at(const char *start, const char *key, double *out)
{
    const char *p = strstr(start, key);
    if (p == NULL) {
        return 0;
    }
    *out = strtod(p + strlen(key), NULL);
    return 1;
}

/* Parse the number after the last occurrence of `key` — when several
 * messages arrive in one step frame, the freshest value wins, matching the
 * reference implementation's drain loop. */
static int find_last_double(const char *line, const char *key, double *out)
{
    const char *p = line;
    const char *last = NULL;
    size_t klen = strlen(key);

    while ((p = strstr(p, key)) != NULL) {
        last = p + klen;
        p += klen;
    }
    if (last == NULL) {
        return 0;
    }
    *out = strtod(last, NULL);
    return 1;
}

/* Pointer to the last occurrence of `pat`, or NULL. */
static const char *find_last_str(const char *line, const char *pat)
{
    const char *p = line;
    const char *last = NULL;
    size_t plen = strlen(pat);

    while ((p = strstr(p, pat)) != NULL) {
        last = p;
        p += plen;
    }
    return last;
}

int main(void)
{
    char *line = NULL;
    size_t cap = 0;

    int mode = MODE_NOMINAL;
    int has_soc = 0;
    double soc_est = 0.0;
    int has_last_assert = 0;
    double last_assert = 0.0;

    /* FDIR state */
    int cycling = 0;
    double cycle_started = 0.0;
    int cycles_used = 0;
    int gave_up = 0;
    int stuck_run = 0;
    int insane_run = 0;
    int has_prev = 0;
    double px = 0.0, py = 0.0, pz = 0.0;

    /* per-step event buffer: at most 3 short events, well under 1 KiB */
    char ev[1024];

    while (getline(&line, &cap, stdin) != -1) {
        if (strstr(line, "\"type\":\"init\"") != NULL) {
            printf("{\"type\":\"ready\",\"subscribe\":"
                   "[\"eps/status\",\"sensors/adcs/gyro\"]}\n");
            fflush(stdout);
            continue;
        }
        if (strstr(line, "\"type\":\"shutdown\"") != NULL) {
            break;
        }
        if (strstr(line, "\"type\":\"step\"") == NULL) {
            continue;
        }

        double t = 0.0;
        (void)find_double_at(line, "\"t\":", &t);

        double v;
        if (find_last_double(line, "\"soc_est\":", &v)) {
            soc_est = v;
            has_soc = 1;
        }
        int has_gyro = 0;
        double gx = 0.0, gy = 0.0, gz = 0.0;
        const char *g = find_last_str(line, "\"topic\":\"sensors/adcs/gyro\"");
        if (g != NULL) {
            has_gyro = find_double_at(g, "\"x\":", &gx)
                    && find_double_at(g, "\"y\":", &gy)
                    && find_double_at(g, "\"z\":", &gz);
        }

        size_t evn = 0;
        ev[0] = '\0';

        int changed = 0;
        if (has_soc) {
            if (mode == MODE_NOMINAL && soc_est < SAFE_ENTER_SOC) {
                mode = MODE_SAFE;
                changed = 1;
            } else if (mode == MODE_SAFE && soc_est > SAFE_EXIT_SOC) {
                mode = MODE_NOMINAL;
                changed = 1;
            }
        }
        const char *mode_str = (mode == MODE_SAFE) ? "SAFE" : "NOMINAL";
        if (changed) {
            evn += (size_t)snprintf(ev + evn, sizeof ev - evn,
                "{\"kind\":\"mode_change\",\"detail\":"
                "{\"to\":\"%s\",\"soc_est\":%.17g}}", mode_str, soc_est);
        }

        /* -- FDIR: gyro health watchdog -> ADCS power cycle -------------- */
        int fdir_changed = 0;
        int masked = cycling;  /* data from an unpowered rail proves nothing */
        if (cycling && t - cycle_started >= CYCLE_OFF_S) {
            cycling = 0;
            fdir_changed = 1;
            evn += (size_t)snprintf(ev + evn, sizeof ev - evn,
                "%s{\"kind\":\"fdir_adcs_repower\",\"detail\":"
                "{\"cycles_used\":%d}}", evn ? "," : "", cycles_used);
        }
        if (has_gyro && !masked) {
            if (has_prev && gx == px && gy == py && gz == pz) {
                stuck_run++;
            } else {
                stuck_run = 0;
            }
            double rate2 = gx * gx + gy * gy + gz * gz;
            if (rate2 > RATE2_LIMIT) {
                insane_run++;
                if (insane_run == 1) {
                    evn += (size_t)snprintf(ev + evn, sizeof ev - evn,
                        "%s{\"kind\":\"gyro_anomaly\",\"detail\":{}}",
                        evn ? "," : "");
                }
            } else {
                insane_run = 0;
            }
            px = gx;
            py = gy;
            pz = gz;
            has_prev = 1;
            if (stuck_run >= STUCK_TRIGGER || insane_run >= INSANE_TRIGGER) {
                if (cycles_used >= MAX_CYCLES) {
                    if (!gave_up) {
                        gave_up = 1;
                        evn += (size_t)snprintf(ev + evn, sizeof ev - evn,
                            "%s{\"kind\":\"fdir_giveup\",\"detail\":"
                            "{\"cycles_used\":%d}}", evn ? "," : "",
                            cycles_used);
                    }
                } else {
                    cycling = 1;
                    cycle_started = t;
                    cycles_used++;
                    stuck_run = 0;
                    insane_run = 0;
                    has_prev = 0;
                    fdir_changed = 1;
                    evn += (size_t)snprintf(ev + evn, sizeof ev - evn,
                        "%s{\"kind\":\"fdir_adcs_power_cycle\",\"detail\":"
                        "{\"n\":%d}}", evn ? "," : "", cycles_used);
                }
            }
        }

        const char *adcs_on = cycling ? "false" : "true";
        const char *payload_on = (mode == MODE_NOMINAL) ? "true" : "false";
        int due = !has_last_assert || (t - last_assert >= REASSERT_EVERY_S);

        printf("{\"type\":\"out\",\"pub\":[");
        int first = 1;
        if (changed) {
            printf("{\"topic\":\"obc/mode\",\"data\":{\"mode\":\"%s\"}}", mode_str);
            first = 0;
        }
        if (changed || fdir_changed || due) {
            if (!first) {
                printf(",");
            }
            printf("{\"topic\":\"obc/request/loads\",\"data\":"
                   "{\"loads\":{\"adcs\":%s,\"payload\":%s}}}",
                   adcs_on, payload_on);
            last_assert = t;
            has_last_assert = 1;
        }
        printf("],\"telemetry\":{\"safe_mode\":%.1f,\"fdir_cycles\":%.1f}",
               (mode == MODE_SAFE) ? 1.0 : 0.0, (double)cycles_used);
        if (evn > 0) {
            printf(",\"events\":[%s]", ev);
        }
        printf("}\n");
        fflush(stdout);
    }

    free(line);
    return 0;
}
