/* TX adapter wiring assertions (Phase A1/A2 of the 2026-08-15 next-step
 * handoff; finding TX-ADAPTER-01 in FINDINGS.md's open findings log).
 *
 * RETIRED 2026-08-16 (MAINT-01, Phase 5): the designated-field-initializer
 * scans pinned the inline marshal that mapped MissionState_Get() into
 * TxFsmConfirmInput_t.mission_ascent. That mapping now lives in the
 * production adapter module (Core/Src/lora_app_adapters.c) and is proven
 * BEHAVIORALLY by the linked suite tests/host/test_lora_app_adapters.c
 * (both polarities, mutation-gated). What remains here is the
 * consumption-side contract plus the delegation anchor:
 *
 *   MISSION_ASCENT -> mission_ascent == true (adapter, linked-tested)
 *     -> TxFsm_OnTxConfirm evaluates !mission_ascent == false
 *     -> archive recovery BLOCKED during ascent
 *   MISSION_FLOAT  -> mission_ascent == false
 *     -> archive recovery PERMITTED when the ACK/battery/cache gates pass
 *
 * The pure-FSM halves are proven behaviourally elsewhere: test_burst_fsm.c
 * T-C6 (ascent inhibits the archive opportunity) and the open-burst
 * scenarios (float permits it), plus the 734,060-check shadow suite.
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

int main(void)
{
    char *app   = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));
    char *txfsm = strip_comments(slurp("../../Core/Src/tx_fsm.c"));

    printf("-- TX-ADAPTER-01: delegation + consumption-side polarity\n");

    /* Delegation anchor (MAINT-01): lora_app.c builds the confirm input via
     * the linked-tested adapter builder and feeds it to the FSM. */
    const char *build = strstr(app, "AppAdapters_BuildTxConfirm(&snap)");
    CHECK(build != NULL);
    const char *call = build ? strstr(build, "TxFsm_OnTxConfirm(&g_tx_fsm") : NULL;
    CHECK(call != NULL);

    /* The raw enum - not a pre-derived bool - crosses the boundary. */
    CHECK(strstr(app, ".mission_state = MissionState_Get()") != NULL);

    /* The inverted form must never return (RED on b958a95; would re-invert
     * R3-03). */
    CHECK(strstr(app, "MissionState_Get() != MISSION_ASCENT") == NULL);

    /* Consumption-side contract (GREEN pin): the module opens recovery on
     * !mission_ascent, i.e. a positively-named field consumed with positive
     * polarity. If the module's polarity ever changes, this adapter mapping
     * must be re-reviewed in the same commit. */
    CHECK(strstr(txfsm, "!in->mission_ascent") != NULL);

    printf("%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
