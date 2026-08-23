/*
 * Bare-metal LED blink — 0x08005000
 * Zero dependencies: no HAL, no RT-Thread, no SDK headers.
 */

typedef unsigned int uint32_t;

/* ---- Cortex-M4 core ---- */
#define SCB_CPACR   (*(volatile uint32_t *)0xE000ED88U)
#define SCB_VTOR    (*(volatile uint32_t *)0xE000ED08U)

/* ---- RCC ---- */
#define RCC_BASE    0x40023800U
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30U))

/* ---- GPIOA ---- */
#define GPIOA_BASE  0x40020000U
#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_BSRR  (*(volatile uint32_t *)(GPIOA_BASE + 0x18U))

#define RCC_AHB1ENR_GPIOAEN  (1U << 0)

int main(void);

int main(void)
{
    SCB_VTOR  = 0x08005000U;          /* vector table at APP base */
    SCB_CPACR |= (0xFU << 20);        /* enable FPU */

    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  /* GPIOA clock on */
    (void)RCC_AHB1ENR;

    GPIOA_MODER &= ~(3U << (1 * 2));  /* PA1 output */
    GPIOA_MODER |=  (1U << (1 * 2));

    while (1) {
        GPIOA_BSRR = (1U << 1);                 /* LED ON  */
        for (volatile int i = 0; i < 8000000; i++) {}
        GPIOA_BSRR = (1U << 1U) << 16;          /* LED OFF */
        for (volatile int i = 0; i < 8000000; i++) {}
    }
}
