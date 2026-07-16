/*
 * OBC flight software — C port of the mode manager.
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

/* Parse the number after the first occurrence of `key`. */
static int find_double(const char *line, const char *key, double *out)
{
    const char *p = strstr(line, key);
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

int main(void)
{
    char *line = NULL;
    size_t cap = 0;

    int mode = MODE_NOMINAL;
    int has_soc = 0;
    double soc_est = 0.0;
    int has_last_assert = 0;
    double last_assert = 0.0;

    while (getline(&line, &cap, stdin) != -1) {
        if (strstr(line, "\"type\":\"init\"") != NULL) {
            printf("{\"type\":\"ready\",\"subscribe\":[\"eps/status\"]}\n");
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
        (void)find_double(line, "\"t\":", &t);

        double v;
        if (find_last_double(line, "\"soc_est\":", &v)) {
            soc_est = v;
            has_soc = 1;
        }

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
        const char *payload_on = (mode == MODE_NOMINAL) ? "true" : "false";
        int due = !has_last_assert || (t - last_assert >= REASSERT_EVERY_S);

        printf("{\"type\":\"out\",\"pub\":[");
        int first = 1;
        if (changed) {
            printf("{\"topic\":\"obc/mode\",\"data\":{\"mode\":\"%s\"}}", mode_str);
            first = 0;
        }
        if (changed || due) {
            if (!first) {
                printf(",");
            }
            printf("{\"topic\":\"obc/request/loads\",\"data\":"
                   "{\"loads\":{\"adcs\":true,\"payload\":%s}}}", payload_on);
            last_assert = t;
            has_last_assert = 1;
        }
        printf("],\"telemetry\":{\"safe_mode\":%s}",
               (mode == MODE_SAFE) ? "1.0" : "0.0");
        if (changed) {
            printf(",\"events\":[{\"kind\":\"mode_change\",\"detail\":"
                   "{\"to\":\"%s\",\"soc_est\":%.17g}}]", mode_str, soc_est);
        }
        printf("}\n");
        fflush(stdout);
    }

    free(line);
    return 0;
}
