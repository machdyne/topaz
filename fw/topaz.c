/*
 * Topaz Firmware
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "ch32fun.h"

#define LED_PIN 3   /* PC3 */
#define MCO_PIN 4   /* PC4 */
#define RST_PIN 1   /* PA1 -> ASIC RST_N */

/* Select the MCO output frequency, in Hz. Change this one line.
 *
 *   0         -- PC4 driven high-Z (floating input); use this when an
 *                external clock source is supplied on PC4 instead
 *   24000000  -- native MCO peripheral, source = HSI
 *   48000000  -- native MCO peripheral, source = SYSCLK
 *
 * Only these three values are supported. All of them are a single native
 * MCO register write (or, for 0, just switching PC4 to an input) -- no
 * timer is ever involved, so the output is always a clean edge straight
 * from the MCO hardware, and TIM1 is always free for the LED.
 */
#ifndef MCO_FREQ_HZ
//#define MCO_FREQ_HZ 0
#define MCO_FREQ_HZ 24000000
#endif

/* How long to hold RST_N low before releasing it, after MCO is running */
#define RST_PULSE_MS 100

#if MCO_FREQ_HZ == 0
	#define MCO_MODE_EXTERNAL 1
#elif MCO_FREQ_HZ == 24000000
	#define MCO_MODE_NATIVE_HSI 1
#elif MCO_FREQ_HZ == 48000000
	#define MCO_MODE_NATIVE_SYSCLK 1
#else
	#error "MCO_FREQ_HZ must be 0, 24000000, or 48000000."
#endif

/* ── LED (PC3, TIM1 Channel 3 PWM, "breathing" fade) ──────────────────────
 *
 * PC3 is TIM1_CH3 in the default (no-remap) pin mapping. MCO never uses
 * TIM1 now (only the native SYSCLK/HSI mux, or high-Z), so TIM1 is always
 * free for the LED -- a full 256-step, ~3.9 kHz PWM every time, no
 * shared-timer caveats. Brightness follows volume^2 as volume ramps
 * 0..255..0, matching the eye's roughly power-law brightness perception
 * so the fade looks smooth rather than linear/robotic.
 */

#define LED_PWM_PERIOD 256UL

static void led_pwm_init(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;

    /* PC3 as TIM1_CH3 alternate function push-pull (default mapping) */
    GPIOC->CFGLR &= ~(0xFu << (4 * LED_PIN));
    GPIOC->CFGLR |=  (GPIO_CNF_OUT_PP_AF | GPIO_Speed_10MHz) << (4 * LED_PIN);

    /* CH3 output compare: PWM mode 1 (OC3M = 110) -- high while CNT < CCR */
    TIM1->CHCTLR2 &= ~(TIM_OC3M | TIM_CC3S);
    TIM1->CHCTLR2 |=  (TIM_OC3M_2 | TIM_OC3M_1);
    TIM1->CH3CVR   = 0;               /* start fully off */
    TIM1->CCER    |= TIM_CC3E;

    /* PSC=47 -> 48MHz/48 = 1MHz timer clock; ATRLR=255 -> ~3.9kHz PWM */
    TIM1->PSC     = 47;
    TIM1->ATRLR   = LED_PWM_PERIOD - 1;
    TIM1->SWEVGR |= TIM_UG;
    TIM1->CTLR1  |= TIM_CEN;
    TIM1->BDTR   |= TIM_MOE;

    printf("led_pwm_init: PC3=TIM1_CH3, PWM period=%lu counts\n",
           (unsigned long)LED_PWM_PERIOD);
}

/* Advance the breathing animation by one step. Call this periodically
 * (e.g. every ~8ms) from the main loop. */
static void led_breathe_step(void)
{
    static uint16_t volume = 0;
    static int8_t   dir = 1;

    uint32_t duty = ((uint32_t)volume * volume * LED_PWM_PERIOD) / (256UL * 256UL);
    TIM1->CH3CVR = duty;

    if (dir > 0) {
        if (volume >= 255) dir = -1; else volume++;
    } else {
        if (volume == 0)   dir = 1;  else volume--;
    }
}

/* ── ASIC reset (RST_N, PA1, active-low, push-pull) ───────────────────────
 *
 * NOTE: PA1/PA2 default to normal GPIO function (AFIO_PCFR1.PA1PA2_RM = 0
 * on reset), so no remap is needed here. If this board ever grows an HSE
 * crystal on PA1/PA2, that bit would need to stay cleared for this to work.
 */

static inline void rst_assert(void)  { GPIOA->BSHR = (1u << (16 + RST_PIN)); } /* drive low  */
static inline void rst_release(void) { GPIOA->BSHR = (1u << RST_PIN); }        /* drive high */

static void rst_init(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA;

    /* Assert reset before the pin becomes an output, so there's no glitch */
    rst_assert();

    GPIOA->CFGLR &= ~(0xFu << (4 * RST_PIN));
    GPIOA->CFGLR |=  (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4 * RST_PIN);

    printf("rst_init: GPIOA CFGLR=0x%08lx OUTDR=0x%08lx (RST_N should read 0/low)\n",
           (unsigned long)GPIOA->CFGLR, (unsigned long)GPIOA->OUTDR);
}

/* ── MCO (clock output on PC4) ────────────────────────────────────────────
 *
 * Native MCO peripheral only -- see the MCO_FREQ_HZ block above. Always a
 * single register write (or a GPIO mode change for the high-Z/external
 * case), never a timer.
 */

static void mco_init(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

#if defined(MCO_MODE_EXTERNAL)

    /* External clock supplied on PC4 -- put the pin in floating input
     * (genuinely high-Z, no pull resistor) so the MCU doesn't drive it. */
    GPIOC->CFGLR &= ~(0xFu << (4 * MCO_PIN));
    GPIOC->CFGLR |=  (GPIO_CNF_IN_FLOATING) << (4 * MCO_PIN); /* MODE=00 (input) */

    printf("mco_init: MCO_FREQ_HZ=0 -> PC4 set to floating input (high-Z), "
           "expecting an external clock. GPIOC CFGLR=0x%08lx\n",
           (unsigned long)GPIOC->CFGLR);
    return;

#endif

    /* CNF=10 (mux push-pull output), MODE=11 (30 MHz max speed) */
    GPIOC->CFGLR &= ~(0xFu << (4 * MCO_PIN));
    GPIOC->CFGLR |=  (GPIO_CNF_OUT_PP_AF | GPIO_Speed_30MHz) << (4 * MCO_PIN);

#if defined(MCO_MODE_NATIVE_HSI)

    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_CFGR0_MCO) | RCC_CFGR0_MCO_HSI;
    printf("mco_init: native HSI -> PC4, RCC CFGR0=0x%08lx\n", (unsigned long)RCC->CFGR0);

#elif defined(MCO_MODE_NATIVE_SYSCLK)

    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_CFGR0_MCO) | RCC_CFGR0_MCO_SYSCLK;
    printf("mco_init: native SYSCLK -> PC4, RCC CFGR0=0x%08lx\n", (unsigned long)RCC->CFGR0);

#endif
}

/* ── Main ───────────────────────────────────────────────────────────────── */

int main(void)
{
	SystemInit();
	Delay_Ms(1);

	printf("\n--- Topaz boot ---\n");

	rst_init();     /* RST_N already held low from here */

	mco_init();     /* bring up the clock to the ASIC */
	led_pwm_init(); /* LED breathing PWM               */
	Delay_Ms(RST_PULSE_MS);

	rst_release();  /* let the ASIC come out of reset with a clock present */
	printf("rst_release: GPIOA OUTDR=0x%08lx (RST_N should read 1/high)\n",
	       (unsigned long)GPIOA->OUTDR);

	uint32_t heartbeat = 0;
	while (1) {
		led_breathe_step();
		Delay_Ms(8);
		if ((heartbeat % 128) == 0) {
			printf("heartbeat %lu\n", (unsigned long)(heartbeat / 128));
		}
		heartbeat++;
	}

}
