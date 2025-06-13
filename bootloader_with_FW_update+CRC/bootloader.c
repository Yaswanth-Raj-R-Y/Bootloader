#include "stm32f401xc.h"

// Define memory addresses for the application and CRC
#define FLASH_START_ADDR 0x08004000             // Start address of user application in flash
#define FLASH_END_ADDR   0x08007FFF             // End address of user application in flash
#define CRC_ADDR         (FLASH_END_ADDR - 4)   // Address to store CRC value
#define APP_END_ADDR     (FLASH_END_ADDR - 8)   // Address to store application size

// Symbols defined in the linker script for memory sections
extern uint32_t _sidata;    // Start address of .data section in flash
extern uint32_t _sdata;     // Start address of .data section in SRAM
extern uint32_t _edata;     // End address of .data section in SRAM
extern uint32_t _sbss;      // Start address of .bss section in SRAM
extern uint32_t _ebss;      // End address of .bss section in SRAM
extern uint32_t _estack;    // Top of stack

// Function prototypes
void uart_send_msg(char*);
void update_firmware();
void erase_sector();
void uart_init();
uint16_t uart_receive(uint8_t*, uint16_t);
uint8_t check_crc();

/**
 * @brief Bootloader reset handler. Initializes .data and .bss sections, peripherals, and handles firmware update or CRC check.
 */
void bootloader_Reset_Handler()
{
    // Copy initialized data from flash to SRAM
    uint32_t* src = &_sidata;
    uint32_t* dest = &_sdata;
    while (dest < &_edata)
    {
        *dest++ = *src++;
    }

    // Zero initialize the .bss section in SRAM
    dest = &_sbss;
    while (dest < &_ebss)
    {
        *dest++ = 0;
    }

    uart_init(); // Initialize UART peripheral

    uart_send_msg("\n\rRunning Bootloader: v2.0\n\r");

    // Configure GPIOA pin 0 as input with pull-up for mode selection
    RCC->AHB1ENR |= 0x1;        // Enable GPIOA clock
    GPIOA->MODER &= ~(0x3);     // Set pin 0 as input
    GPIOA->PUPDR |= 0x1;        // Enable pull-up resistor

    // Check if button is pressed (pin 0 low) to enter firmware update mode
    if (!(GPIOA->IDR & 1))
    {
        uart_send_msg("Entered firmware update mode:\n\r");
        update_firmware();

        uart_send_msg("Jumping to application program.\n\r");
        uart_send_msg("\n\rRunning blink...\n\r");
    }
    else
    {
        // Run CRC check for firmware integrity
        uart_send_msg("Running firmware integrity test: - - - ");
        if (!check_crc())
        {
            uart_send_msg("test failed!\n\r");
            return;
        }
        uart_send_msg("test passed!:\n\r");
    }
    
    // Set up jump to user application
    uint32_t APP_ADDR = FLASH_START_ADDR;
    SCB->VTOR = APP_ADDR;                           // Relocate vector table to application start
    __set_MSP(*(volatile uint32_t*)(APP_ADDR));     // Set main stack pointer from application vector table

    // Get application's reset handler address and jump to it
    void (*app_reset_handler)(void) = (void (*)(void)) (*(volatile uint32_t*) (APP_ADDR + 4));
    uart_send_msg("Jumping to application program.\n\r");
    app_reset_handler();    // Execute application firmware
}

// Bootloader interrupt vector table
__attribute__((section(".isr_vector"))) uint32_t bootloader_vector_table[] = {
    (uint32_t)&_estack,                     // Initial stack pointer
    (uint32_t)&bootloader_Reset_Handler     // Bootloader reset handler
};

/**
 * @brief Initialize UART1 for serial communication.
 */
void uart_init()
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;    // Enable USART1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // Enable GPIOA clock

    GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1; // Set PA9, PA10 to alternate function
    GPIOA->AFR[1] |= (7 << 4) | (7 << 8);    // Set alternate function 7 (USART1) for PA9, PA10

    USART1->BRR = 16000000 / 9600;           // Set baud rate to 9600 (assuming 16 MHz clock)
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable transmitter, receiver, and USART
}

/**
 * @brief Send a null-terminated string over UART1.
 *
 * @param s Pointer to the string to send.
 */
void uart_send_msg(char* s)
{
    while(*s)
    {
        while (!(USART1->SR & USART_SR_TXE)); // Wait until transmit data register is empty
        USART1->DR = (uint8_t)*s;             // Send character
        s++;
    }
}

/**
 * @brief Firmware update routine. Receives new firmware via UART and writes it to flash, computes and stores CRC.
 */
void update_firmware()
{
    // Unlock flash memory for writing
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    erase_sector(); // Erase sector 1 before programming

    uint32_t addr = 0x08004000;     // Start address for new firmware
    uint8_t buffer[256];            // Data buffer for UART reception

    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;      // Enable CRC peripheral clock
    CRC->CR = CRC_CR_RESET;                 // Reset CRC calculation

    FLASH->CR |= FLASH_CR_PSIZE_1;      // Set programming size to 32 bits (4 bytes)
    FLASH->CR |= FLASH_CR_PG;           // Enable programming mode

    uart_send_msg("Flash new firmware - - - > ");
    while (1)
    {
        uint16_t received_byte = uart_receive(buffer, 256);     // Receive firmware chunk

        // Check if received data is a multiple of 4 bytes
        if (received_byte > 0 && received_byte % 4 == 0)
        {
            for (uint16_t i=0; i<received_byte; i += 4)
            {   
                uint32_t receivedData = *(uint32_t*)&buffer[i];
                *(uint32_t*)addr = receivedData;        // Write data to flash
                while (FLASH->SR & FLASH_SR_BSY);       // Wait for write to complete
                CRC->DR = receivedData;                 // Update CRC with new data
                addr += 4;
            }
        }
        else
        {
            // Store firmware size at APP_END_ADDR
            *(uint32_t*)APP_END_ADDR = addr - FLASH_START_ADDR;
            uart_send_msg("completed :)\n\r");
            break;
        }
    }

    // Store final CRC value at CRC_ADDR
    *(uint32_t*)CRC_ADDR = CRC->DR;
    while (FLASH->SR & FLASH_SR_BSY);   // Wait for flash to be ready

    FLASH->CR &= ~FLASH_CR_PG;      // Disable programming mode
    FLASH->CR |= FLASH_CR_LOCK;     // Lock flash memory
}

/**
 * @brief Erase flash sector 1.
 */
void erase_sector()
{
    uart_send_msg("Erasing Sector 1 - - - > ");

    while (FLASH->SR & FLASH_SR_BSY); // Wait if flash is busy

    FLASH->CR |= FLASH_CR_SER;        // Enable sector erase mode
    FLASH->CR |= FLASH_CR_SNB_0;      // Select sector 1
    FLASH->CR |= FLASH_CR_STRT;       // Start erase operation

    while (FLASH->SR & FLASH_SR_BSY); // Wait for erase to complete

    FLASH->CR &= ~FLASH_CR_SER;       // Disable sector erase mode

    uart_send_msg("completed\n\r");
}

/**
 * @brief Receive data from UART1 with a timeout.
 *
 * @param buffer Pointer to buffer to store received data.
 * @param size   Maximum number of bytes to receive.
 * @return       Number of bytes actually received.
 */
uint16_t uart_receive(uint8_t* buffer, uint16_t size)
{
    uint16_t return_size = 0;
    while (return_size < size)
    {
        uint32_t timeout = 0;
        while (!(USART1->SR & USART_SR_RXNE))
        {
            timeout++;
            if (timeout >= 1600000)     // Timeout after ~3 seconds
                return return_size;     // Return bytes received so far
        }

        buffer[return_size] = USART1->DR;   // Store received byte
        return_size++;
    }

    return return_size; // Return total bytes received
}

/**
 * @brief Check firmware CRC for integrity.
 *
 * @return 1 if CRC matches or not computed, 0 if CRC check fails.
 */
uint8_t check_crc()
{
    // If CRC not computed (flash default), treat as valid
    if (*(uint32_t*)CRC_ADDR == 0xFFFFFFFF)
    {
        return 1;
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;      // Enable CRC peripheral clock
    CRC->CR = CRC_CR_RESET;                 // Reset CRC calculation

    uint32_t size = *(uint32_t*)APP_END_ADDR;   // Get firmware size

    // Recalculate CRC for the firmware in flash
    for (uint32_t i=FLASH_START_ADDR; i<FLASH_START_ADDR + size; i += 4)
    {
        uint32_t data = *(uint32_t*)i;
        CRC->DR = data;
    }

    uint32_t calculatedCRC = CRC->DR;           // Calculated CRC value
    uint32_t storedCRC = *(uint32_t*)CRC_ADDR;  // Stored CRC value

    return (calculatedCRC == storedCRC);        // Return 1 if CRC matches, 0 otherwise
}
