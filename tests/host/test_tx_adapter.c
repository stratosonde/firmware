/* TX adapter wiring assertions (Phase A1/A2 of the 2026-08-15 next-step
 * handoff; finding TX-ADAPTER-01 in FINDINGS.md's open findings log).
 *
 * lora_app.c cannot be linked into the host harness, so these are narrow
 * source-wiring assertions on the DESIGNATED FIELD INITIALIZER that maps
 * MissionState_Get() into TxFsmConfirmInput_t.mission_ascent - the exact
 * mapping whose inversion produced the TX-ADAPTER-01 regression (ASCENT
 * permitted archive recovery, FLOAT blocked it). A source-wiring assertion
 * is the accepted mechanism here per the handoff ("Because lora_app.c
 * cannot presently be linked into the host harness, a narrow source-wiring
 * assertion is acceptable ... It must check the designated field
 * initializer, not merely the presence of MISSION_ASCENT somewhere in the
 * function").
 *
 * Semantics pinned end to end:
 *   MISSION_ASCENT -> mission_ascent == true
 *     -> TxFsm_OnTxConfirm evaluates !mission_ascent == false
 *     -> archive recovery BLOCKED during ascent
 *   MISSION_FLOAT  -> mission_ascent == false
 *     -> archive recovery PERMITTED when the ACK/battery/cache gates pass
 *
 * The pure-FSM halves are proven behaviourally elsewhere: test_burst_fsm.c
 * T-C6 (ascent inhibits the archive opportunity) and the open-burst
 * scenarios (float permits it), plus the 734,060-check shadow suite. This
 * file pins the adapter mapping those tests cannot see - the defect class
 * that produced TX-ADAPTER-01.
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
        while (r < (size_t)n) {
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

/* s occurs inside the initializer window [init, call). */
static int in_window(const char *init, const char *call, const char *s)
{
    const char *p = strstr(init, s);
    return p != NULL && p < call;
}

int main(void)
{
    char *app   = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));
    char *txfsm = strip_comments(slurp("../../Core/Src/tx_fsm.c"));

    printf("-- TX-ADAPTER-01: confirm-input mission_ascent polarity\n");

    /* The window under test: the confirm-input initializer, bounded by the
     * struct declaration and the TxFsm_OnTxConfirm call it feeds. */
    const char *init = strstr(app, "TxFsmConfirmInput_t fsm_in");
    CHECK(init != NULL);
    const char *call = init ? strstr(init, "TxFsm_OnTxConfirm(&g_tx_fsm") : NULL;
    CHECK(call != NULL);
    if (!init || !call) {
        printf("%d checks, %d failures\n", checks, failures);
        return 1;
    }

    /* IN_WINDOW: s occurs inside the initializer (before the consuming call) */
#define IN_WINDOW(s) in_window(init, call, (s))

    /* The positive mapping: MISSION_ASCENT -> mission_ascent == true.
     * RED on the reviewed baseline b958a95 (positional, negated form). */
    CHECK(IN_WINDOW(".mission_ascent = (MissionState_Get() == MISSION_ASCENT)"));

    /* The inverted form must be gone from the initializer. RED on b958a95. */
    {
        const char *inv = strstr(init, "MissionState_Get() != MISSION_ASCENT");
        CHECK(inv == NULL || inv >= call);
    }

    /* Every field of the touched initializer is designated, so no future
     * edit can silently reintroduce positional/polarity ambiguity. */
    CHECK(IN_WINDOW(".now_ms ="));
    CHECK(IN_WINDOW(".status_ok ="));
    CHECK(IN_WINDOW(".ack_received ="));
    CHECK(IN_WINDOW(".battery_good ="));
    CHECK(IN_WINDOW(".has_unsent ="));
    CHECK(IN_WINDOW(".max_bulk_packets ="));

    /* Consumption-side contract (GREEN pin): the module opens recovery on
     * !mission_ascent, i.e. a positively-named field consumed with positive
     * polarity. If the module's polarity ever changes, this adapter mapping
     * must be re-reviewed in the same commit. */
    CHECK(strstr(txfsm, "!in->mission_ascent") != NULL);

    printf("%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
