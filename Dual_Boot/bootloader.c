#include "stm32f401xc.h"

// External symbols defined in linker script for memory section addresses
extern uint32_t _sidata;  // Start of .data initial values in Flash
extern uint32_t _sdata;   // Start of .data section in RAM
extern uint32_t _edata;   // End of .data section in RAM
extern uint32_t _sbss;    // Start of .bss section in RAM
extern uint32_t _ebss;    // End of .bss section in RAM
extern uint32_t _estack;  // Top of stack

void uart_send_msg(char*);

/**
 * @brief Bootloader entry point - handles memory initialization, peripheral setup, and application selection logic with timeout functionality
 */
void bootloader_Reset_Handler()
{
    __disable_irq();  // Disable interrupts during initialization

    // Initialize .data section (copy from Flash to RAM)
    uint32_t* src = &_sidata;
    uint32_t* dest = &_sdata;
    while (dest < &_edata)
    {
        *dest++ = *src++;
    }

    // Initialize .bss section (zero-initialized data)
    dest = &_sbss;
    while (dest < &_ebss)
    {
        *dest++ = 0;
    }

    // UART1 Peripheral Configuration
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   // Enable USART1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // Enable GPIOA clock
    
    // Configure PA9 (TX) and PA10 (RX) for alternate function mode
    GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
    GPIOA->AFR[1] |= (7 << 4) | (7 << 8);   // AF7 for USART1
    
    // Configure baud rate (16MHz clock / 9600 baud)
    USART1->BRR = 16000000 / 9600;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;     // Enable transmitter and USART

    uart_send_msg("\n\rRunning Bootloader");

    // GPIO Configuration for Boot Mode Selection
    RCC->AHB1ENR |= 0x1;          // Enable GPIOA clock
    GPIOA->MODER &= ~(0x3);       // PA0 input mode
    GPIOA->PUPDR |= 0x1;          // PA0 pull-up

    // Timeout-based Application Selection Logic
    uint32_t timeout = 0;
    uint8_t flag = 1;       // Default to secondary application
    while(GPIOA->IDR & 1)   // Wait for button press (PA0 low)
    {
        timeout += 2;        
        if (timeout >= 4000000)     // 3-4 second timeout
        {
            flag = 0;   // Switch to primary application
            break;
        }
    }

    // Application Address Selection
    uint32_t APP_ADDR;
    if (!flag) {
        APP_ADDR = 0x08004000;      // Sector 2 (128KB starting at 0x08000000)
    } else {
        APP_ADDR = 0x08008000;      // Sector 3 (256KB starting at 0x08000000)
    }
  
    // Vector Table Relocation
    SCB->VTOR = APP_ADDR;  // Set Vector Table Offset Register
    __set_MSP(*(volatile uint32_t*)(APP_ADDR));     // Initialize Main Stack Pointer

    // Get application reset handler from vector table (second entry)
    void (*app_reset_handler)(void) = (void (*)(void))(*(volatile uint32_t*)(APP_ADDR + 4));
    
    app_reset_handler();  // Jump to selected application
}

/**
 * @brief Transmit null-terminated string via UART1
 *
 * @param s Pointer to null-terminated string
 */
void uart_send_msg(char* s)
{
    while(*s)
    {
        while (!(USART1->SR & USART_SR_TXE));   // Wait for transmit buffer empty
        USART1->DR = (uint8_t)*s;               // Send character
        s++;
    }
}

// Bootloader Interrupt Vector Table
__attribute__((section(".isr_vector"))) uint32_t bootloader_vector_table[] = {
    (uint32_t)&_estack,                     // Initial stack pointer value
    (uint32_t)&bootloader_Reset_Handler     // Reset handler pointer
};