#include "stm32f401xc.h"

// Function prototypes
void timer11_init();              // Initialize TIM11 for microsecond timing
void Delay_us(uint16_t us);       // Delay for a specified number of microseconds
void Delay_ms(uint16_t ms);       // Delay for a specified number of milliseconds
void uart_send_msg(char*);        // Send a string over UART1

int main()
{
    timer11_init();     // Configure TIM11 for delay functions

    // Enable clocks for USART1 and GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set PA9 (TX) and PA10 (RX) to alternate function mode for USART1
    GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
    GPIOA->AFR[1] |= (7 << 4) | (7 << 8); // AF7 for USART1

    // Configure USART1: baud rate 9600, enable transmitter and USART
    USART1->BRR = 16000000 / 9600;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;

    uart_send_msg("\n\n\rRunning application 2 from 0x08008000\n\n\r");

    // Enable clock for GPIOB (for LED output)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 

    // Configure PB8 as output (clear bits, then set as output)
    GPIOB->MODER &= ~(3 << 16);     // Clear mode bits for PB8
    GPIOB->MODER |= (1 << 16);     // Set PB8 as general purpose output

    while (1)
    {
        GPIOB->ODR ^= (1 << 8);     // Toggle PB8 output (LED blink)
        Delay_ms(1000);            // 1 second delay
    }
}

/**
 * @brief Initialize TIM11 as a basic timer for microsecond delays. Prescaler set so timer ticks at 1 MHz (1 tick = 1 us).
 */
void timer11_init()
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;    // Enable TIM11 clock
    TIM11->PSC = 15;                        // Prescaler: 16 MHz / (15+1) = 1 MHz
    TIM11->ARR = 0xFFFF;                    // Auto-reload value (max)
    TIM11->CR1 |= TIM_CR1_CEN;              // Enable timer
}

/**
 * @brief Delay for a specified number of microseconds using TIM11.
 *
 * @param us Number of microseconds to delay
 */
void Delay_us(uint16_t us)
{
    TIM11->CNT = 0;             // Reset timer counter
    while (TIM11->CNT < us);    // Wait until desired count is reached
}

/**
 * @brief Delay for a specified number of milliseconds using TIM11.
 *
 * @param ms Number of milliseconds to delay
 */
void Delay_ms(uint16_t ms)
{
    for (uint16_t i = 0; i < ms; i++)
    {
        Delay_us(1000);     // 1000 microseconds = 1 millisecond
    }
}

/**
 * @brief Send a null-terminated string over USART1.
 *
 * @param s Pointer to string to send
 */
void uart_send_msg(char* s)
{
    while(*s)
    {
        while (!(USART1->SR & USART_SR_TXE));   // Wait for transmit buffer to be empty
        USART1->DR = (uint8_t)*s;               // Send character
        s++;
    }
}