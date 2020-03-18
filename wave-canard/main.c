#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <stdint.h>
#include <canard.h>

void usleep(void);

// called with usleep(1000) from canard.c
extern void usleep(void)
{
    // TODO implement 1 ms delay
}

/* Set STM32 to 24 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_24mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
}

static void gpio_setup(void)
{
	/* Set GPIO to 'output push-pull'. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
}

int main(void)
{
	clock_setup();
	gpio_setup();

	/* Blink the LEDs (PC8 and PC9) on the board. */
	while (1) {
		gpio_toggle(GPIOA, GPIO4); /* Toggle LEDs. */
		for (uint32_t i = 0; i < 2000000; i++)      /* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
