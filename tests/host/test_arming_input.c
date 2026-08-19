/* Module contract suite for Core/Src/arming_input.c (PRETEST-DEC-01, #142).
 * The unit is #included (same-TU pattern, like test_multiregion.c) so the
 * host GPIO read-state double is shared with the module. The mission door
 * functions are test-owned doubles (the real mission_state.h is included
 * for the prototypes; this suite's include order puts Core/Inc first). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arming_input.h"
#include "mission_state.h"
#include "stm32wlxx_hal.h"

static int checks;
static int failures;
#define CHECK(x) do { checks++; if (!(x)) { failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); } } while (0)

/* ---- test doubles for the mission door ---- */
static int g_commissioning = 1;
static int g_enter_flight_calls = 0;
bool MissionState_IsCommissioning(void) { return g_commissioning != 0; }
void MissionState_EnterFlight(void) { g_enter_flight_calls++; }

#include "../../Core/Src/arming_input.c"

static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc((size_t)n + 1U);
    if (buf) {
        if (fread(buf, 1, (size_t)n, f) != (size_t)n) { free(buf); buf = NULL; }
        else { buf[n] = '\0'; }
    }
    fclose(f);
    return buf;
}
/* lora_app.c declares OnTxData in its prototype block before defining it:
 * take the first occurrence of sig whose parameter list is followed by '{'
 * and not ';' (the test_lt_20260813.c pattern). String literals are skipped
 * so a brace inside a log format cannot unbalance the count. */
static char *function_body(const char *src, const char *sig) {
  const char *start = NULL;
  const char *p = NULL;
  for (const char *cand = strstr(src, sig); cand != NULL;
       cand = strstr(cand + 1, sig)) {
    const char *q = cand + strlen(sig);
    while (*q != '\0' && *q != '{' && *q != ';')
      q++;
    if (*q == '{') {
      start = cand;
      p = q;
      break;
    }
  }
  if (!start || !p)
    return NULL;
  int depth = 0;
  const char *end = NULL;
  for (; *p != '\0'; p++) {
    if (*p == '"' || *p == '\'') {
      char qc = *p;
      p++;
      while (*p != '\0' && *p != qc) {
        if (*p == '\\' && p[1] != '\0')
          p++;
        p++;
      }
      if (*p == '\0')
        break;
      continue;
    }
    if (*p == '{')
      depth++;
    else if (*p == '}') {
      depth--;
      if (depth == 0) {
        end = p + 1;
        break;
      }
    }
  }
  if (!end)
    end = src + strlen(src);
  size_t len = (size_t)(end - start);
  char *body = malloc(len + 1);
  memcpy(body, start, len);
  body[len] = '\0';
  return body;
}

int main(void) {
  /* ---- pure debounce ---- */
  uint8_t st = 0;
  CHECK(!ArmingInput_Debounce(&st, false, ARMING_CONFIRM_WAKES));
  CHECK(!ArmingInput_Debounce(&st, true, ARMING_CONFIRM_WAKES));
  CHECK(st == 1);
  CHECK(ArmingInput_Debounce(&st, true, ARMING_CONFIRM_WAKES)); /* 2nd consecutive */
  CHECK(!ArmingInput_Debounce(NULL, true, ARMING_CONFIRM_WAKES));
  CHECK(!ArmingInput_Debounce(&st, true, 0U));
  st = 1;
  CHECK(!ArmingInput_Debounce(&st, false, ARMING_CONFIRM_WAKES) && st == 0); /* release resets */

  /* ---- poll path (GPIO + mission doubles) ---- */
  g_commissioning = 1;
  g_enter_flight_calls = 0;
  g_host_gpio_read_state = GPIO_PIN_SET; /* released (pull-up reads high) */
  ArmingInput_Poll();
  CHECK(g_enter_flight_calls == 0);
  g_host_gpio_read_state = GPIO_PIN_RESET; /* pressed (button to GND) */
  ArmingInput_Poll();                      /* wake 1: streak only */
  CHECK(g_enter_flight_calls == 0);
  ArmingInput_Poll(); /* wake 2: confirmed -> arm */
  CHECK(g_enter_flight_calls == 1);

  /* a release between presses resets the streak */
  g_host_gpio_read_state = GPIO_PIN_SET;
  ArmingInput_Poll();
  g_host_gpio_read_state = GPIO_PIN_RESET;
  ArmingInput_Poll();
  CHECK(g_enter_flight_calls == 1); /* single press after release: no arm */
  ArmingInput_Poll();
  CHECK(g_enter_flight_calls == 2);

  /* never arms outside commissioning (and never samples as armed there) */
  g_commissioning = 0;
  g_host_gpio_read_state = GPIO_PIN_RESET;
  ArmingInput_Poll();
  ArmingInput_Poll();
  ArmingInput_Poll();
  CHECK(g_enter_flight_calls == 2);

  /* ---- scan pins: the shared-net dance is complete ---- */
  char *src = slurp("../../Core/Src/arming_input.c");
  CHECK(src != NULL);
  if (src) {
    /* samples as input with pull-up (active-low button) */
    CHECK(strstr(src, "GPIO_MODE_INPUT") != NULL);
    CHECK(strstr(src, "GPIO_PULLUP") != NULL);
    /* restores the SPI2_SCK alternate function afterwards */
    CHECK(strstr(src, "GPIO_MODE_AF_PP") != NULL);
    CHECK(strstr(src, "GPIO_AF5_SPI2") != NULL);
    CHECK(strstr(src, "GPIO_PIN_13") != NULL);
    /* commissioning gate before any sampling */
    CHECK(strstr(src, "MissionState_IsCommissioning") != NULL);
    free(src);
  }

  /* the flight door stays out of lora_app.c (R6/#192) - the poll call is
   * the only arming reference there */
  char *app = slurp("../../LoRaWAN/App/lora_app.c");
  CHECK(app != NULL);
  if (app) {
    CHECK(strstr(app, "ArmingInput_Poll();") != NULL);
    CHECK(strstr(app, "MissionState_EnterFlight") == NULL); /* COMM-TX (2026-08-18, DDR-0002 §7 / BR-LIFE-004): commissioning now
                                                             * transmits privacy-safe telemetry, but the shared-net constraint is
                                                             * unchanged - arming is still polled FIRST and the commissioning
                                                             * cycle must not touch the W25Q/flash archive (SPI2_SCK = PB13). */
    const char *poll = strstr(app, "ArmingInput_Poll();");
    const char *call = strstr(app, "CommissioningTelemetryCycle(&sensor_data");
    CHECK(poll != NULL && call != NULL);
    CHECK(poll < call);
    char *cycle = function_body(app, "static void CommissioningTelemetryCycle(");
    CHECK(cycle != NULL);
    if (cycle) {
      CHECK(strstr(cycle, "EncodeCompactBinaryPacket(&compact_packet") != NULL);
      CHECK(strstr(cycle, "compact_packet.latitude_100m = 0;") != NULL);
      CHECK(strstr(cycle, "compact_packet.longitude_100m = 0;") != NULL);
      CHECK(strstr(cycle, "EncodeCommissioningLiveRecord(") != NULL);
      CHECK(strstr(cycle, "DatarateFromSF(10)") != NULL); /* SF10 probe */
      CHECK(strstr(cycle, "FlashLog_") == NULL);          /* SPI2 stays idle */
      CHECK(strstr(cycle, "W25Q") == NULL);
      free(cycle);
    }
    /* The SF7 live-record follow-up rides the probe's confirm in
     * OnTxData, gated BEFORE the archive FSM (commissioning never opens
     * the archive opportunity). */
    char *otx = function_body(app, "static void OnTxData(LmHandlerTxParams_t *params)");
    CHECK(otx != NULL);
    if (otx) {
      const char *gate = strstr(otx, "if (MissionState_IsCommissioning()) {");
      const char *fsmstep = strstr(otx, "TxFsm_OnTxConfirm(&g_tx_fsm");
      CHECK(gate != NULL && fsmstep != NULL);
      CHECK(gate < fsmstep);
      CHECK(strstr(otx, "s_comm_live_buf") != NULL);
      CHECK(strstr(otx, "DatarateFromSF(7)") != NULL);
      free(otx);
    }
    /* D1 superseded 2026-08-18: no per-region SF9 probe carve-out. */
    CHECK(strstr(app, "? 9 : 10") == NULL);
    free(app);
  }

  printf("%d checks, %d failures\n", checks, failures);
  return failures != 0;
}
