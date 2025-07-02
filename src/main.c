#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>

// Define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DS3231_ADDR 0x68
#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I 0x75  // Device ID register
#define MPU6050_PWR_MGMT_1 0x6B // Power management register
#define MPU6050_GYRO_CONFIG 0x1B // Gyroscope configuration
#define MPU6050_GYRO_ZOUT_H 0x47 // Gyroscope Z-axis high byte
#define MPU6050_GYRO_ZOUT_L 0x48 // Gyroscope Z-axis low byte
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_XOUT_H  0x43

// Constants from original Espruino code
#define PUCK_GYRO_CONSTANT 600000.0f  // Magic constant for gyro to revolutions conversion
#define PULL_REV_THRESHOLD 0.1f       // Minimum revolution change to register movement
#define PULL_TIMEOUT_MS 1000          // Timeout for pull end detection
#define SESSION_TIMEOUT_MS 120000     // 2 minutes session timeout (like original)

/* Function Declarations */
void process_command(const char* cmd);
float get_total_rotations(void);
void print_session_history(void);
int i2c1_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data);
int i2c1_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);
int i2c2_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data);
int i2c2_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);
int i2c2_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data);

/* Logger */
typedef enum { LOG_OFF, LOG_ERROR, LOG_INFO, LOG_DEBUG, LOG_SILLY } LogLevel;
LogLevel log_level = LOG_INFO;

void logger(LogLevel level, const char* msg) {
    if (level <= log_level) {
        printf("%s\n", msg);
    }
}

// Event types
typedef enum { EVENT_ROLL_CHANGE, EVENT_DISPENSE } EventType;

typedef struct {
    EventType type;
    char date[16]; // YYMMDDhhmmss
    float revs;
} Event;

#define MAX_EVENTS 32
Event eventArray[MAX_EVENTS];
int eventCount = 0;

/* Internal clock set to 84 MHz */
void internal_clock(void)
{
    // Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);

    // Configure Flash prefetch, Instruction cache, Data cache and wait state
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

    // Configure PLL: PLLCLK = (HSI/16) * 336 / 4 = 84 MHz
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) | (336 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSI) |
                   (7 << RCC_PLLCFGR_PLLQ_Pos);

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);

    // Select PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* RTC Logic */
volatile uint32_t msTicks = 0;
void SysTick_Handler(void) { msTicks++; }
uint32_t getTimeMs(void) { return msTicks; }

// Helper: Convert BCD to binary
static uint8_t bcd2bin(uint8_t val) {
    return (val >> 4) * 10 + (val & 0x0F);
}

// Helper: Convert binary to BCD
static uint8_t bin2bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

int set_ds3231_time(uint16_t year, uint8_t month, uint8_t day, uint8_t weekday,
                    uint8_t hour, uint8_t min, uint8_t sec) {
    uint8_t data[7];
    data[0] = bin2bcd(sec);
    data[1] = bin2bcd(min);
    data[2] = bin2bcd(hour);      // 24h mode
    data[3] = bin2bcd(weekday);   // 1=Sunday, 7=Saturday
    data[4] = bin2bcd(day);
    data[5] = bin2bcd(month);
    data[6] = bin2bcd(year % 100);

    // Write all 7 bytes starting at register 0x00
    for (int i = 0; i < 7; ++i) {
        if (i2c2_write_reg(DS3231_ADDR, 0x00 + i, data[i]) != 0)
            return -1; // Error
    }
    return 0; // Success
}

// RTC read
void rtc_get_datetime(char *buf, size_t len) {
    uint8_t data[7] = {0};
    if (i2c2_read_bytes(DS3231_ADDR, 0x00, data, 7) == 0) {
        uint8_t sec  = bcd2bin(data[0]);
        uint8_t min  = bcd2bin(data[1]);
        uint8_t hour = bcd2bin(data[2] & 0x3F); // 24h mode
        uint8_t day  = bcd2bin(data[4]);
        uint8_t mon  = bcd2bin(data[5] & 0x1F);
        uint16_t year = 2000 + bcd2bin(data[6]);
        snprintf(buf, len, "%04u-%02u-%02u %02u:%02u:%02u", year, mon, day, hour, min, sec);
    } else {
        snprintf(buf, len, "RTC ERROR");
    }
}

/* Temporary memory handling */
// In-memory session log structure
typedef struct {
    char datetime[32];
    float rotations;
    uint32_t timestamp;
} SessionLog;

#define MAX_SESSION_LOGS 20  // Store last 20 sessions in volatile memory
SessionLog sessionHistory[MAX_SESSION_LOGS];
int sessionLogCount = 0;
int sessionLogIndex = 0;  // Circular buffer index

// Function to add a session to memory log
void add_session_to_memory(const char* datetime, float rotations) {
    // Use circular buffer approach to keep most recent sessions
    strncpy(sessionHistory[sessionLogIndex].datetime, datetime, sizeof(sessionHistory[0].datetime));
    sessionHistory[sessionLogIndex].rotations = rotations;
    sessionHistory[sessionLogIndex].timestamp = getTimeMs();
    
    sessionLogIndex = (sessionLogIndex + 1) % MAX_SESSION_LOGS;
    
    // Increase count until buffer is full
    if (sessionLogCount < MAX_SESSION_LOGS) {
        sessionLogCount++;
    }
}

// Function to print all sessions in memory
void print_session_history(void) {
    char header[] = "\r\n--- Session History ---\r\n";
    for (const char *p = header; *p; ++p) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *p;
    }
    
    // Start with oldest entry (when buffer is full)
    int start = (sessionLogCount == MAX_SESSION_LOGS) ? sessionLogIndex : 0;
    for (int i = 0; i < sessionLogCount; i++) {
        int idx = (start + i) % MAX_SESSION_LOGS;
        char entry[80];
        snprintf(entry, sizeof(entry), "[%s] Rotations: %.2f\r\n",
                sessionHistory[idx].datetime,
                sessionHistory[idx].rotations);
        
        for (const char *p = entry; *p; ++p) {
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = *p;
        }
    }
}

/* Rotation logic */
// Function to calculate total rotations across all sessions
float get_total_rotations(void) {
    float total = 0.0f;
    for (int i = 0; i < sessionLogCount; i++) {
        total += sessionHistory[i].rotations;
    }
    return total;
}

/* UART Terminal commands */
#define UART_RX_BUFFER_SIZE 64
volatile char uartRxBuffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uartRxIndex = 0;
volatile uint8_t cmdReady = 0;

// Command definitions
#define CMD_DUMP_DATA "dump"
#define CMD_CLEAR_DATA "clear"
#define CMD_HELP "help"

// Initialize UART with receive capability
void uart_init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    // Set PA2 (TX) to alternate function (AF7)
    GPIOA->MODER &= ~(0x3 << (2 * 2));
    GPIOA->MODER |= (0x2 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |= (0x7 << (2 * 4));
    
    // Set PA3 (RX) to alternate function (AF7)
    GPIOA->MODER &= ~(0x3 << (3 * 2));
    GPIOA->MODER |= (0x2 << (3 * 2));
    GPIOA->AFR[0] &= ~(0xF << (3 * 4));
    GPIOA->AFR[0] |= (0x7 << (3 * 4));
    
    // APB1 is 16 MHz by default
    USART2->BRR = (uint16_t)(16000000 / 9600);
    
    // Enable transmitter, receiver, and RXNE interrupt
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    
    // Enable USART2 interrupt in NVIC
    NVIC_EnableIRQ(USART2_IRQn);
}

// Print a string to UART
void uart_print(const char *str) {
    for (const char *p = str; *p; ++p) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *p;
    }
}

// Process received command
void process_command(const char* cmd) {
    if (strcmp(cmd, "dump") == 0) {
        print_session_history();
        
        char total[64];
        snprintf(total, sizeof(total), "Total rotations: %.2f\r\n", get_total_rotations());
        for (const char *p = total; *p; ++p) {
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = *p;
        }
    }

    // Command: clear - reset session history
    else if (strcmp(cmd, "clear") == 0) {
        sessionLogCount = 0;
        sessionLogIndex = 0;
        
        char msg[] = "Session history cleared\r\n";
        for (const char *p = msg; *p; ++p) {
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = *p;
        }
    }
    // Command: help - show available commands
    else if (strcmp(cmd, "help") == 0) {
        char help[] = "\r\nAvailable commands:\r\n"
                      "  dump  - Print session history\r\n"
                      "  clear - Clear session history\r\n"
                      "  help  - Show this help message\r\n";
        for (const char *p = help; *p; ++p) {
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = *p;
        }
    }
    else {
        char unknown[] = "Unknown command. Type 'help' for available commands.\r\n";
        for (const char *p = unknown; *p; ++p) {
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = *p;
        }
    }
}

// USART2 IRQ handler for receiving commands
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char c = USART2->DR;
        
        // Echo character back to terminal
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = c;
        
        // Process character
        if (c == '\r' || c == '\n') {
            // Command terminator received
            uartRxBuffer[uartRxIndex] = '\0';  // Null-terminate
            cmdReady = 1;  // Set command ready flag

            // Echo newline
            uart_print("\r\n");
            
            // Process command
            process_command((char*)uartRxBuffer);
            
            // Reset buffer
            uartRxIndex = 0;
        }
        else if (c == 8 || c == 127) {  // Backspace or Delete
            if (uartRxIndex > 0) {
                uartRxIndex--;
                
                // Echo backspace sequence to clear character
                uart_print("\b \b");
            }
        }
        else if (uartRxIndex < UART_RX_BUFFER_SIZE - 1) {
            uartRxBuffer[uartRxIndex++] = c;
        }
    }
}



/* Flash memory handling */
#define FLASH_LOG_SECTOR   7  // Use sector 7 (last 128KB of 512KB flash)
#define FLASH_LOG_ADDRESS  0x08060000  // Start of sector 7
#define FLASH_LOG_SIZE     (128 * 1024) // 128KB
#define FLASH_LOG_ENTRY_SIZE sizeof(Event)
#define FLASH_LOG_MAX_ENTRIES (FLASH_LOG_SIZE / FLASH_LOG_ENTRY_SIZE)

// STM32F4xx flash sector erase function
void flash_erase_sector(uint32_t sector) {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_SER;
}

// STM32F4xx flash write function (writes one Event)
void flash_write_event(uint32_t address, Event *event) {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR |= FLASH_CR_PG;
    uint32_t *src = (uint32_t*)event;
    uint32_t *dst = (uint32_t*)address;
    for (size_t i = 0; i < sizeof(Event)/4; ++i) {
        dst[i] = src[i];
        while (FLASH->SR & FLASH_SR_BSY);
    }
    FLASH->CR &= ~FLASH_CR_PG;
}

// Find next free log slot (0xFFFFFFFF means empty)
uint32_t flash_log_find_next(void) {
    for (uint32_t i = 0; i < FLASH_LOG_MAX_ENTRIES; ++i) {
        uint32_t *entry = (uint32_t*)(FLASH_LOG_ADDRESS + i * FLASH_LOG_ENTRY_SIZE);
        if (*entry == 0xFFFFFFFF) {
            return FLASH_LOG_ADDRESS + i * FLASH_LOG_ENTRY_SIZE;
        }
    }
    return 0; // Full
}

// Append event to flash log (erase sector if full)
void flash_log_append(Event *event) {
    uint32_t addr = flash_log_find_next();
    if (addr == 0) {
        // Erase sector and start over
        flash_erase_sector(FLASH_LOG_SECTOR);
        addr = FLASH_LOG_ADDRESS;
    }
    flash_write_event(addr, event);
}

// Example: log an event (call this from your interrupt/session handler) - REWRITE
void log_session_event(EventType type, const char *date, float revs) {
    Event ev;
    ev.type = type;
    strncpy(ev.date, date, sizeof(ev.date));
    ev.revs = revs;
    flash_log_append(&ev);
}

// --- I2C1 (PB8=SCL, PB9=SDA) setup for MPU6050 gyroscope ---
void i2c1_init(void) {
    // Reset I2C1 peripheral
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    // Enable GPIOB and I2C1 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // Set PB8, PB9 to alternate function (AF4)
    GPIOB->MODER &= ~((0x3 << (8 * 2)) | (0x3 << (9 * 2)));
    GPIOB->MODER |= (0x2 << (8 * 2)) | (0x2 << (9 * 2));
    GPIOB->AFR[1] &= ~((0xF << ((8-8) * 4)) | (0xF << ((9-8) * 4)));
    GPIOB->AFR[1] |= (0x4 << ((8-8) * 4)) | (0x4 << ((9-8) * 4));
    // Open-drain, pull-up
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);
    GPIOB->PUPDR &= ~((0x3 << (8 * 2)) | (0x3 << (9 * 2)));
    GPIOB->PUPDR |= (0x1 << (8 * 2)) | (0x1 << (9 * 2));
    // Add: Set ODR high to release the I2C lines (critical)
    GPIOB->ODR |= (1 << 8) | (1 << 9);
    // Configure I2C1: 100kHz standard mode (assuming 16MHz PCLK1)
    I2C1->CR2 = 16; // 16 MHz
    I2C1->CCR = 80; // 100kHz
    I2C1->TRISE = 17; // 1000ns / (1/16MHz) + 1
    I2C1->CR1 |= I2C_CR1_PE;
}

// --- I2C1 Write/Read helpers ---
int i2c1_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    const uint32_t MAX_TIMEOUT = 50000; // Same timeout as in read function
    uint32_t timeout;
    
    // Generate start condition
    I2C1->CR1 |= I2C_CR1_START;
    
    // Wait for start condition with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP; // Send stop to release the bus
        return -1;  // Timeout error
    }
    
    (void)I2C1->SR1; // Clear SB flag
    I2C1->DR = (dev_addr << 1); // Send device address (write mode)
    
    // Wait for address sent with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return -2;  // Address error
    }
    
    (void)I2C1->SR2; // Clear ADDR flag
    
    // Send register address
    I2C1->DR = reg;
    
    // Wait for TXE with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return -3;  // Register address error
    }
    
    // Send data
    I2C1->DR = data;
    
    // Wait for TXE with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return -4;  // Data write error
    }
    
    // Generate stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
    
    return 0;  // Success
}

int i2c1_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    const uint32_t MAX_TIMEOUT = 50000; // Adjust based on your system speed
    uint32_t timeout;
    
    // Write register address
    I2C1->CR1 |= I2C_CR1_START;
    
    // Wait for start condition with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP; // Send stop to release the bus
        return -1;  // Timeout error
    }
    
    (void)I2C1->SR1;
    I2C1->DR = (dev_addr << 1);
    
    // Wait for address sent with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return -2;  // Address error
    }
    
    (void)I2C1->SR2;
    I2C1->DR = reg;
    
    // Wait for register address sent with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return -3;  // Register address error
    }
    
    // Restart for read
    I2C1->CR1 |= I2C_CR1_START;
    
    // Wait for restart with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return -4;  // Restart error
    }
    
    (void)I2C1->SR1;
    I2C1->DR = (dev_addr << 1) | 1;  // Read mode
    
    // Wait for address in read mode with timeout
    timeout = MAX_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout) timeout--;
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return -5;  // Read address error
    }
    
    (void)I2C1->SR2;
    
    // For single byte read, disable ACK before clearing ADDR
    if (len == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
    } else {
        I2C1->CR1 |= I2C_CR1_ACK;
    }
    
    // Read data bytes
    for (uint8_t i = 0; i < len; ++i) {
        // For last byte, we need to send NACK
        if (i == len - 1 && len > 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }
        
        // Wait for data with timeout
        timeout = MAX_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout) timeout--;
        if (!timeout) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return -6;  // Data receive error
        }
        
        buf[i] = I2C1->DR;
    }
    
    // Generate stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
    
    // Re-enable ACK for future transfers
    I2C1->CR1 |= I2C_CR1_ACK;
    
    return 0;  // Success
}

/* I2C2 Bus Initialization */

void i2c2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    // Set PB10 (D6) to AF4 (I2C2_SCL)
    GPIOB->MODER &= ~(0x3 << (10*2));
    GPIOB->MODER |=  (0x2 << (10*2));
    GPIOB->AFR[1] &= ~(0xF << ((10-8)*4));
    GPIOB->AFR[1] |=  (0x4 << ((10-8)*4));
    // Set PB3 (D3) to AF9 (I2C2_SDA)
    GPIOB->MODER &= ~(0x3 << (3*2));
    GPIOB->MODER |=  (0x2 << (3*2));
    GPIOB->AFR[0] &= ~(0xF << (3*4));
    GPIOB->AFR[0] |=  (0x9 << (3*4));
    // Open-drain and pull-up
    GPIOB->OTYPER |= (1 << 10) | (1 << 3);
    GPIOB->PUPDR &= ~((0x3 << (10*2)) | (0x3 << (3*2)));
    GPIOB->PUPDR |=  (0x1 << (10*2)) | (0x1 << (3*2));
    GPIOB->ODR |= (1 << 10) | (1 << 3);
    // I2C2 peripheral setup
    I2C2->CR2 = 16;
    I2C2->CCR = 80;
    I2C2->TRISE = 17;
    I2C2->CR1 |= I2C_CR1_PE;
}

int i2c2_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data) {
    uint32_t timeout;
    I2C2->CR1 |= I2C_CR1_START;
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_SB) && --timeout);
    if (!timeout) return -1;
    (void)I2C2->SR1;
    I2C2->DR = (dev_addr << 1);
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_ADDR) && --timeout);
    if (!timeout) return -2;
    (void)I2C2->SR2;
    I2C2->DR = reg;
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_TXE) && --timeout);
    if (!timeout) return -3;
    I2C2->CR1 |= I2C_CR1_START;
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_SB) && --timeout);
    if (!timeout) return -4;
    (void)I2C2->SR1;
    I2C2->DR = (dev_addr << 1) | 1;
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_ADDR) && --timeout);
    if (!timeout) return -5;
    (void)I2C2->SR2;
    I2C2->CR1 &= ~I2C_CR1_ACK;
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_RXNE) && --timeout);
    if (!timeout) return -6;
    *data = I2C2->DR;
    I2C2->CR1 |= I2C_CR1_STOP;
    I2C2->CR1 |= I2C_CR1_ACK;
    return 0;
}

int i2c2_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    const uint32_t MAX_TIMEOUT = 50000;
    uint32_t timeout;

    // Write register address
    I2C2->CR1 |= I2C_CR1_START;
    timeout = MAX_TIMEOUT;
    while (!(I2C2->SR1 & I2C_SR1_SB) && timeout) timeout--;
    if (!timeout) {
        I2C2->CR1 |= I2C_CR1_STOP;
        return -1;
    }

    (void)I2C2->SR1;
    I2C2->DR = (dev_addr << 1);

    timeout = MAX_TIMEOUT;
    while (!(I2C2->SR1 & I2C_SR1_ADDR) && timeout) timeout--;
    if (!timeout) {
        I2C2->CR1 |= I2C_CR1_STOP;
        return -2;
    }

    (void)I2C2->SR2;
    I2C2->DR = reg;

    timeout = MAX_TIMEOUT;
    while (!(I2C2->SR1 & I2C_SR1_TXE) && timeout) timeout--;
    if (!timeout) {
        I2C2->CR1 |= I2C_CR1_STOP;
        return -3;
    }

    // Restart for read
    I2C2->CR1 |= I2C_CR1_START;
    timeout = MAX_TIMEOUT;
    while (!(I2C2->SR1 & I2C_SR1_SB) && timeout) timeout--;
    if (!timeout) {
        I2C2->CR1 |= I2C_CR1_STOP;
        return -4;
    }

    (void)I2C2->SR1;
    I2C2->DR = (dev_addr << 1) | 1;

    timeout = MAX_TIMEOUT;
    while (!(I2C2->SR1 & I2C_SR1_ADDR) && timeout) timeout--;
    if (!timeout) {
        I2C2->CR1 |= I2C_CR1_STOP;
        return -5;
    }

    (void)I2C2->SR2;

    // For single byte read, disable ACK before clearing ADDR
    if (len == 1) {
        I2C2->CR1 &= ~I2C_CR1_ACK;
    } else {
        I2C2->CR1 |= I2C_CR1_ACK;
    }

    for (uint8_t i = 0; i < len; ++i) {
        // For last byte, send NACK
        if (i == len - 1 && len > 1) {
            I2C2->CR1 &= ~I2C_CR1_ACK;
        }

        timeout = MAX_TIMEOUT;
        while (!(I2C2->SR1 & I2C_SR1_RXNE) && timeout) timeout--;
        if (!timeout) {
            I2C2->CR1 |= I2C_CR1_STOP;
            return -6;
        }

        buf[i] = I2C2->DR;
    }

    I2C2->CR1 |= I2C_CR1_STOP;
    I2C2->CR1 |= I2C_CR1_ACK; // Re-enable ACK for future transfers

    return 0;
}

int i2c2_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    uint32_t timeout;
    I2C2->CR1 |= I2C_CR1_START;
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_SB) && --timeout);
    if (!timeout) return -1;
    (void)I2C2->SR1;
    I2C2->DR = (dev_addr << 1);
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_ADDR) && --timeout);
    if (!timeout) return -2;
    (void)I2C2->SR2;
    I2C2->DR = reg;
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_TXE) && --timeout);
    if (!timeout) return -3;
    I2C2->DR = data;
    timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_TXE) && --timeout);
    if (!timeout) return -4;
    I2C2->CR1 |= I2C_CR1_STOP;
    return 0;
}


void i2c_bus_recovery(void) {
    // Temporarily configure pins as GPIO
    GPIOB->MODER &= ~((0x3 << (8 * 2)) | (0x3 << (9 * 2)));
    GPIOB->MODER |= (0x1 << (8 * 2)) | (0x1 << (9 * 2));
    
    // Toggle SCL to free any stuck devices
    for (int i = 0; i < 9; i++) {
        GPIOB->ODR |= (1 << 8);   // SCL high
        for (volatile int j = 0; j < 1000; j++);
        GPIOB->ODR &= ~(1 << 8);  // SCL low
        for (volatile int j = 0; j < 1000; j++);
    }
    
    // Generate STOP condition
    GPIOB->ODR &= ~(1 << 9);  // SDA low
    for (volatile int j = 0; j < 1000; j++);
    GPIOB->ODR |= (1 << 8);   // SCL high
    for (volatile int j = 0; j < 1000; j++);
    GPIOB->ODR |= (1 << 9);   // SDA high
    
    // Reconfigure for I2C
    GPIOB->MODER &= ~((0x3 << (8 * 2)) | (0x3 << (9 * 2)));
    GPIOB->MODER |= (0x2 << (8 * 2)) | (0x2 << (9 * 2));
}

// --- MPU6050 Initialization ---
void mpu6050_init(void) {
    // Wake up the MPU6050 (default is sleep mode)
    i2c1_write_reg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    
    // Verify device ID (should be 0x69 when AD0=1)
    uint8_t devid = 0;
    if (i2c1_read_bytes(MPU6050_ADDR, MPU6050_WHO_AM_I, &devid, 1) == 0) {
        if (devid != 0x69) {
            // Device ID is wrong - try bus recovery
            i2c_bus_recovery();
            i2c1_init();
        }
    }
    
    // Configure gyroscope range: +/- 250 degrees/sec (most sensitive)
    i2c1_write_reg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);
    
    // Additional delay for stability
    for (volatile int i = 0; i < 100000; i++);
}

// --- Read gyroscope Z-axis data ---
typedef struct { 
    int16_t z;     // Only Z-axis needed for rotation tracking
    float z_dps;   // Z-axis in degrees per second
} mpu6050_gyro_t;

void mpu6050_read_gyro_z(mpu6050_gyro_t *gyro) {
    uint8_t buf[2];
    int result = i2c1_read_bytes(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, buf, 2);
    if (result != 0) {
        // On error, set values to zero
        gyro->z = 0;
        gyro->z_dps = 0.0f;
        return;
    }
    
    // Combine high and low bytes
    gyro->z = (int16_t)(buf[0] << 8 | buf[1]);
    
    // Convert to degrees per second (for ±250°/s range: LSB = 131 LSB/°/s)
    gyro->z_dps = (float)gyro->z / 131.0f;
}

// --- Gyroscope INT pin (e.g., PC0) as external interrupt ---
void gyro_int_init(void) {
    // Enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    // Set PC0 as input (default)
    GPIOC->MODER &= ~(0x3 << (0 * 2));
    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // Map EXTI0 to PC0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |= (0x2 << SYSCFG_EXTICR1_EXTI0_Pos); // 0x2 = Port C
    // Unmask EXTI0
    EXTI->IMR |= EXTI_IMR_MR0;
    // Rising edge trigger (or falling, depending on sensor)
    EXTI->RTSR |= EXTI_RTSR_TR0;
    // Enable EXTI0 interrupt in NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);
}

// --- EXTI0 IRQ Handler (called on gyroscope interrupt) ---
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0; // Clear pending bit
        // Handle gyroscope event (read data, log, etc.)
        logger(LOG_INFO, "Gyroscope interrupt!");
        // Read and print gyro Z-axis
        mpu6050_gyro_t gyro;
        mpu6050_read_gyro_z(&gyro);
        char msg[64];
        snprintf(msg, sizeof(msg), "MPU6050 Gyro Z: %d (%.2f dps)", gyro.z, gyro.z_dps);
        logger(LOG_INFO, msg);
    }
}

// --- Test: Read and print MPU6050 WHO_AM_I register ---
uint8_t mpu6050_read_devid(void) {
    uint8_t devid = 0;
    if (i2c1_read_bytes(MPU6050_ADDR, MPU6050_WHO_AM_I, &devid, 1) != 0) {
        // Communication error - try recovery
        i2c_bus_recovery();
        i2c1_init();
        i2c1_read_bytes(MPU6050_ADDR, MPU6050_WHO_AM_I, &devid, 1);
    }
    return devid;
}

/* Light sensor initialization */


// --- Main function ---
int main(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Set PA5 as output (LED)
    GPIOA->MODER &= ~(0x3 << (5 * 2));
    GPIOA->MODER |= (0x1 << (5 * 2));
    
    // Initialize UART with receive capability
    uart_init();

    // Blink LED a few times to show startup
    for (int i = 0; i < 5; ++i) {
        GPIOA->ODR ^= (1 << 5);
        for (volatile int j = 0; j < 800000; ++j);
    }

    // I2C and MPU6050 init
    i2c1_init();
    i2c2_init();
    for (volatile int i = 0; i < 1600000; ++i);
    mpu6050_init();
    for (volatile int i = 0; i < 1600000; ++i);

    set_ds3231_time(2025, 6, 24, 2, 14, 30, 0);

    // Check device ID - should be 0x69 for MPU6050 (when AD0=1)
    uint8_t devid = mpu6050_read_devid();
    char init_msg[50];
    snprintf(init_msg, sizeof(init_msg), "MPU6050 ID: 0x%02X (expect 0x69 with AD0=1)\r\n", devid);
    uart_print(init_msg);

    // Systick timer for millisecond timing
    SysTick_Config(SystemCoreClock / 1000); // 1 ms tick

    // Rotation tracking variables
    float pullRevolutions = 0.0f;         // Current pull revolutions
    // float previousPullRevolutions = 0.0f; // Previous pull revolutions  
    float sessionRevolutions = 0.0f;      // Total session revolutions
    int session_active = 0;               // Session state
    uint32_t last_motion_time = 0;        // Last motion timestamp
    uint32_t current_time = 0;            // Current time
    uint32_t pull_timeout_start = 0;      // Pull timeout tracking

    // Print startup message
    char log_msg[128];
    char datetime[32];
    rtc_get_datetime(datetime, sizeof(datetime));
    snprintf(log_msg, sizeof(log_msg), "[%s] Smart Spindle Ready (Gyroscope Mode)\r\n", datetime);
    uart_print(log_msg);

    // Print command help
    uart_print("Type 'help' for available commands\r\n\r\n");

    static uint32_t last_sample_time = 0;
    // static uint32_t last_debug = 0;

    while (1) {
        mpu6050_gyro_t gyro;
        mpu6050_read_gyro_z(&gyro);

        // Get current time for timeout detection
        current_time = getTimeMs();

        // Sensor error recovery
        if (gyro.z == 0 && gyro.z_dps == 0.0f) {
            static uint32_t last_recovery = 0;
            if (current_time - last_recovery > 5000) {
                i2c_bus_recovery();
                i2c1_init();
                mpu6050_init();
                last_recovery = current_time;
            }
            continue;
        }

        // Time delta for integration
        if (last_sample_time == 0) last_sample_time = current_time;
        float dt = (current_time - last_sample_time) / 1000.0f; // ms to seconds
        last_sample_time = current_time;

        // Deadband to ignore gyro noise
        if (fabsf(gyro.z_dps) < 1.0f) gyro.z_dps = 0.0f;

        // Integrate gyro Z to get revolutions
        float delta_revs = (gyro.z_dps * dt) / 360.0f;
        pullRevolutions += delta_revs;

        // Print each movement (delta in revolutions)
        if (fabsf(delta_revs) > 0.0001f) { // Only print if there was movement
            snprintf(log_msg, sizeof(log_msg),
                "Gyro Z: %d (%.2f dps), Delta revs: %.5f, Pull revs: %.3f, Session revs: %.3f\r\n",
                gyro.z, gyro.z_dps, delta_revs, pullRevolutions, sessionRevolutions);
            uart_print(log_msg);
        }

        // Detect movement
        int spindleIsMoving = (fabsf(delta_revs) > PULL_REV_THRESHOLD);

        // Session logic
        if (spindleIsMoving) {
            if (!session_active) {
                session_active = 1;
                sessionRevolutions = 0.0f;
                rtc_get_datetime(datetime, sizeof(datetime));
                snprintf(log_msg, sizeof(log_msg), "[%s] Session started\r\n", datetime);
                uart_print(log_msg);
            }
            last_motion_time = current_time;
            pull_timeout_start = current_time;
        }

        // Pull timeout handling (like onPullEnd in old.js)
        if (session_active && pull_timeout_start > 0 &&
            (current_time - pull_timeout_start > PULL_TIMEOUT_MS)) {

            // Round revolutions to nearest quarter
            float roundedRevolutions = roundf(pullRevolutions * 4.0f) / 4.0f;

            if (fabsf(roundedRevolutions) > 0.25f) {
                snprintf(log_msg, sizeof(log_msg),
                        "Pull end: %.2f revolutions\r\n", roundedRevolutions);
                uart_print(log_msg);

                sessionRevolutions += roundedRevolutions;
            }

            // Reset pull tracking
            pullRevolutions = 0.0f;
            pull_timeout_start = 0;
        }

        // Session timeout handling (like onSessionEnd in old.js)
        if (session_active && (current_time - last_motion_time > SESSION_TIMEOUT_MS)) {
            session_active = 0;

            float absSessionRevs = fabsf(sessionRevolutions);
            if (absSessionRevs > 0.25f) {
                rtc_get_datetime(datetime, sizeof(datetime));
                snprintf(log_msg, sizeof(log_msg),
                        "[%s] Session ended. Total revolutions: %.2f\r\n",
                        datetime, absSessionRevs);
                uart_print(log_msg);

                add_session_to_memory(datetime, absSessionRevs);
                log_session_event(EVENT_DISPENSE, datetime, absSessionRevs);
            } else {
                uart_print("Session timeout (< 0.25 revolutions)\r\n");
            }

            // Reset all counters
            sessionRevolutions = 0.0f;
            pullRevolutions = 0.0f;
            pull_timeout_start = 0;
        }

        // Short delay to prevent overwhelming the I2C bus
        for (volatile int i = 0; i < 500; ++i);
    }
}