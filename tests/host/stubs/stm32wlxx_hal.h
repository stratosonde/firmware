/* Host-test stub: STM32 HAL (R49 + #57 GNSS parser). Types + no-op HAL
 * surfaces needed by the units under test. */
#ifndef STM32WLXX_HAL_H_STUB
#define STM32WLXX_HAL_H_STUB
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

/* ---- GPIO ---- */
typedef struct { int _host_stub; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
#define GPIOB               ((GPIO_TypeDef *)0x1)
#define GPIO_PIN_6          (1u << 6)
#define GPIO_PIN_7          (1u << 7)
#define GPIO_MODE_INPUT     0u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_ANALOG    3u
#define GPIO_NOPULL         0u
#define GPIO_SPEED_FREQ_LOW 0u
static inline void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i) { (void)p; (void)i; }
/* R9 (#194) observability: GPIO writes are recorded so transactional-cleanup
 * tests can assert final pin state. Per-TU statics (unused attribute) -
 * suites that never read them are unaffected. */
#define HOST_GPIO_LOG_MAX 32
typedef struct { const void *port; uint16_t pin; GPIO_PinState state; } HostGpioWrite_t;
static HostGpioWrite_t g_host_gpio_log[HOST_GPIO_LOG_MAX] __attribute__((unused));
static int g_host_gpio_log_n __attribute__((unused));
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s) {
    if (g_host_gpio_log_n < HOST_GPIO_LOG_MAX) {
        g_host_gpio_log[g_host_gpio_log_n].port = p;
        g_host_gpio_log[g_host_gpio_log_n].pin = pin;
        g_host_gpio_log[g_host_gpio_log_n].state = s;
        g_host_gpio_log_n++;
    }
}
static inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) { (void)p; (void)pin; return GPIO_PIN_RESET; }

/* ---- UART / DMA ---- */
typedef struct { volatile uint32_t CNDTR; } HostDMA_Regs_t;
typedef struct { HostDMA_Regs_t *Instance; } DMA_HandleTypeDef;
typedef struct {
    void *Instance;                 /* compared against USART1 */
    DMA_HandleTypeDef *hdmarx;
    uint32_t ErrorCode;
    int _host_stub;
} UART_HandleTypeDef;
#define USART1              ((void *)0x2)
extern uint16_t g_host_dma_cndtr;   /* test-controlled DMA counter (defined in test_main.c) */
#define __HAL_DMA_GET_COUNTER(hdma) (g_host_dma_cndtr)
#define UART_CLEAR_PEF      0x01u
#define UART_CLEAR_FEF      0x02u
#define UART_CLEAR_NEF      0x04u
#define UART_CLEAR_OREF     0x08u
#define HAL_UART_ERROR_NONE 0u
#define __HAL_UART_CLEAR_FLAG(h, f)     do { (void)(h); (void)(f); } while (0)
#define __HAL_UART_FLUSH_DRREGISTER(h)  do { (void)(h); } while (0)
static inline HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n, uint32_t t) { (void)h; (void)d; (void)n; (void)t; return HAL_OK; }
/* R9 (#194): DMA-start failure injection (per-TU static, defaults OK). */
static HAL_StatusTypeDef g_host_uart_dma_rc __attribute__((unused)) = HAL_OK;
/* SP-01 (#244): count DMA (re)arm calls so the UART error-recovery test can
 * assert the callback re-armed exactly once (and not during teardown). */
static uint32_t g_host_uart_rx_dma_calls __attribute__((unused));
static inline HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) { (void)h; (void)d; (void)n; g_host_uart_rx_dma_calls++; return g_host_uart_dma_rc; }
static inline HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }

/* ---- misc ---- */
typedef struct { int _host_stub; } SPI_HandleTypeDef;
typedef struct { int _host_stub; } I2C_HandleTypeDef;
typedef struct { int _host_stub; } ADC_HandleTypeDef;
/* ---- RTC (F-002/#201: timer_if.c behavioral suite) ---- */
typedef struct { volatile uint32_t SSR; } RTC_TypeDef;
extern RTC_TypeDef g_host_rtc_regs;   /* test-defined; SSR drives the tick */
#define RTC                 (&g_host_rtc_regs)
typedef struct {
    RTC_TypeDef *Instance;
    struct { uint32_t RtcFeatures; } IsEnabled;
} RTC_HandleTypeDef;
typedef struct {
    uint32_t BinaryAutoClr;
    struct { uint32_t SubSeconds; } AlarmTime;
    uint32_t AlarmMask;
    uint32_t AlarmSubSecondMask;
    uint32_t Alarm;
} RTC_AlarmTypeDef;
#define RTC_ALARM_A                        0u
#define RTC_FLAG_ALRAF                     0x1u
#define RTC_FORMAT_BCD                     0u
#define RTC_ALARMMASK_NONE                 0u
#define RTC_ALARMSUBSECONDBIN_AUTOCLR_NO   0u
#define RTC_ALARMSUBSECONDBINMASK_NONE     0u
#define RTC_BKP_DR0                        0u
#define RTC_BKP_DR1                        1u
#define RTC_BKP_DR2                        2u
#define RTC_BKP_DR3                        3u
#define RTC_BKP_DR4                        4u
#define RTC_BKP_DR5                        5u
#define RTC_BKP_DR6                        6u
#define RTC_BKP_DR7                        7u
#define RTC_BKP_DR8                        8u
#define RTC_BKP_DR9                        9u
#define RTC_BKP_DR10                       10u
#define RTC_BKP_DR11                       11u
#define RTC_BKP_DR12                       12u
#define RTC_BKP_DR13                       13u
#define RTC_BKP_DR14                       14u
#define RTC_BKP_DR15                       15u
#define __NOP()                            do { } while (0)
#define __HAL_RTC_ALARM_CLEAR_FLAG(h, f)   do { (void)(h); (void)(f); } while (0)
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);
/* ---- IWDG (multiregion_context.c joins) ---- */
typedef struct { void *Instance; } IWDG_HandleTypeDef;
static inline HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_GetTick(void);   /* test-defined (test_main.c) */
#endif
