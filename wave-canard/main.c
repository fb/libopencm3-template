#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdint.h>
#include <canard.h>

static volatile uint32_t systick_millis;

void usleep(void);

// called with usleep(1000) from canard.c
extern void usleep(void)
{
    // TODO implement 1 ms delay
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Enable GPIOC clock. */
    rcc_periph_clock_enable(RCC_GPIOA);
}

static void gpio_setup(void)
{
    /* Set GPIO to 'output push-pull'. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
}

void sys_tick_handler(void)
{
    systick_millis++;
}

int main(void)
{
    clock_setup();
    gpio_setup();

    /* 72MHz / 8 => 9000000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    uint32_t counts_per_ms = rcc_ahb_frequency / 1000U;

    /* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(counts_per_ms - 1);
    systick_interrupt_enable();
    systick_counter_enable();

    while (1) {
        {
            static uint32_t blink = 0U;
            if(systick_millis > blink)
            {
                gpio_toggle(GPIOA, GPIO4); /* Toggle LED. */
                blink += 1000U;
            }
        }
    }

    return 0;
}
