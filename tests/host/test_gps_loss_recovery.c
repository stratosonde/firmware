/* GPS-loss same-wake recovery wiring gate (Phase A6/A7 of the 2026-08-15
 * next-step handoff; tracker #285/H-09).
 *
 * RED-FIRST: the recovery scans are red against the current tree for the
 * intended reason; the suite is added to `all` by the A7 implementation
 * commit.
 *
 * Scenario pinned (lora_app.c cannot be host-linked; wiring scans per the
 * A1/A4 precedent):
 *  1. a FLIGHT wake whose 24-hour GPS-loss budget has expired opens silent
 *     with VETO_GPS_LOSS (GREEN pin: the silence is still set);
 *  2. an ACCEPTED fix in the same wake (gnss_result ==
 *     GNSS_ACQUIRE_FRESH_GOOD_FIX, itself gated by the A5 configured
 *     predicate) clears the GPS-loss veto BEFORE region selection and TX;
 *  3. the clear is guarded by plan.veto == VETO_GPS_LOSS - it is the ONLY
 *     VETO_NONE assignment in the file, so no-session, restricted-region,
 *     and prelaunch-quiet vetoes are never cleared by it;
 *  4. a below-quality fix does not clear silence (the guard conjunct
 *     requires FRESH_GOOD_FIX);
 *  5. region selection still runs afterwards and may independently reapply
 *     VETO_RESTRICTED_REGION / VETO_RF_SILENCE (GREEN pin).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int checks = 0, failures = 0;
#define CHECK(cond) do { checks++; if (!(cond)) { \
    failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { printf("FATAL: cannot open %s\n", path); exit(2); }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)n + 1);
    if (!buf) exit(2);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) exit(2);
    buf[n] = '\0';
    fclose(f);
    /* Firmware sources are CRLF; scan anchors are written LF. */
    {
        size_t r = 0, w = 0;
        while (r < n) {
            if (buf[r] != '\r') buf[w++] = buf[r];
            r++;
        }
        buf[w] = '\0';
    }
    return buf;
}

/* Strip block and line comments so a scan asserts on CODE, not prose. */
static char *strip_comments(const char *src)
{
    size_t n = strlen(src);
    char *out = (char *)malloc(n + 1);
    size_t o = 0;
    for (size_t i = 0; i < n; i++) {
        if (src[i] == '/' && src[i + 1] == '*') {
            i += 2;
            while (i < n && !(src[i] == '*' && src[i + 1] == '/')) i++;
            i++;
            out[o++] = ' ';
        } else if (src[i] == '/' && src[i + 1] == '/') {
            while (i < n && src[i] != '\n') i++;
            out[o++] = '\n';
        } else {
            out[o++] = src[i];
        }
    }
    out[o] = '\0';
    return out;
}

/* s occurs inside the window [from, to). */
static int in_window(const char *from, const char *to, const char *s)
{
    const char *p = strstr(from, s);
    return p != NULL && p < to;
}

/* occurrences of s in the whole buffer */
static int count_occ(const char *buf, const char *s)
{
    int n = 0;
    const char *p = buf;
    while ((p = strstr(p, s)) != NULL) { n++; p += strlen(s); }
    return n;
}

int main(void)
{
    char *app = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));

    printf("-- A6: same-wake GPS-loss recovery (RED pre-A7)\n");

    /* The window: after acquisition returns, before region selection. */
    const char *acquire = strstr(app, "gnss_result = AcquireGnssFix(");
    CHECK(acquire != NULL);
    const char *select = acquire ? strstr(acquire, "SelectRegionAndSession(&rf_silence, &plan);") : NULL;
    CHECK(select != NULL);
    if (!acquire || !select) {
        printf("%d checks, %d failures\n", checks, failures);
        return 1;
    }

    /* GREEN pin: the GPS-loss silence itself is unchanged and still opens
     * the wake dark before acquisition. */
    const char *gpsloss = strstr(app, "RegionPolicy_Silence(&plan, &rf_silence, VETO_GPS_LOSS);");
    CHECK(gpsloss != NULL && gpsloss < acquire);

    /* RED: the guarded same-wake clear exists in the window - an ACCEPTED
     * fix (A5 predicate upstream) while GPS loss is the recorded veto. */
    CHECK(in_window(acquire, select, "gnss_result == GNSS_ACQUIRE_FRESH_GOOD_FIX &&"));
    CHECK(in_window(acquire, select, "plan.veto == VETO_GPS_LOSS"));
    CHECK(in_window(acquire, select, "plan.veto = VETO_NONE;"));
    CHECK(in_window(acquire, select, "rf_silence = false;"));

    /* RED: the clear is the ONLY veto-clear in the file - no-session,
     * restricted-region, and prelaunch-quiet vetoes can never be cleared
     * by this path (and a below-quality fix fails the guard conjunct). */
    CHECK(count_occ(app, "plan.veto = VETO_NONE;") == 1);

    /* GREEN pin: region selection (the `select` anchor itself) still runs
     * after the window, and the restricted-region silence sites it and the
     * stale-position paths carry are intact (>= 2 VETO_RESTRICTED_REGION
     * applications file-wide). */
    CHECK(count_occ(app, "VETO_RESTRICTED_REGION") >= 2);

    printf("%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
