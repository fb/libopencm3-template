#include <stdint.h>
#include <canard.h>

void usleep(void);

// called with usleep(1000) from canard.c
extern void usleep(void)
{
    // TODO implement 1 ms delay
}

int main(void) {
	/* add your own code */
	uint32_t rev = 0xaabbccdd;
	return 0;
}
