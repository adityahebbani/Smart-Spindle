/*
 * Smart Spindle - Motion Detection and Session Tracking System
 * 
 * This system tracks paper roll rotations using an MPU6050 gyroscope and manages
 * session data with automatic sleep/wake functionality. Features include:
 * - Motion-based session tracking with gyroscope Z-axis monitoring
 * - Light sensor for roll change detection
 * - UART command interface for diagnostics and data retrieval
 * - Flash memory logging with session history
 * - Low-power sleep mode with motion interrupt wake-up
 * 
 * Hardware:
 * - STM32F4xx microcontroller
 * - MPU6050 gyroscope/accelerometer (I2C1: PB8/PB9)
 * - DS3231 RTC (I2C2: PB10/PB11)
 * - TSL2591 light sensor
 * - UART2 debug interface (PA2/PA3)
 */

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "core_cm4.h"

// I2C Device Addresses
#define DS3231_ADDR 0x68
#define MPU6050_ADDR 0x68

// MPU6050 Register Addresses
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_ZOUT_H 0x47

// System Configuration
#define SIGNIFICANT_GYRO_THRESHOLD 0.1f
#define PULL_TIMEOUT_MS 1000
#define SESSION_TIMEOUT_MS 10000
#define MPU_WAKE_THRESHOLD 10
#define WAKE_DPS_THRESHOLD 1.5f
#define GYRO_PRINT_INTERVAL_MS 2000


// ================================
// TYPE DEFINITIONS
// ================================

// Logger levels
typedef enum { 
    LOG_OFF, 
    LOG_ERROR, 
    LOG_INFO, 
    LOG_DEBUG, 
    LOG_SILLY 
} LogLevel;

// Event types
typedef enum { 
    EVENT_ROLL_CHANGE, 
    EVENT_DISPENSE 
} EventType;

// MPU6050 gyroscope data
typedef struct { 
    int16_t z;
    float z_dps;
} mpu6050_gyro_t;

// Event structure
typedef struct {
    EventType type;
    char date[16];
    float revs;
} Event;

// Session log structure
typedef struct {
    char datetime[32];
    float rotations;
    uint32_t timestamp;
} SessionLog;

// ================================
// GLOBAL VARIABLES
// ================================

LogLevel log_level = LOG_INFO;

#define MAX_EVENTS 32
Event eventArray[MAX_EVENTS];
int eventCount = 0;

#define MAX_SESSION_LOGS 20
SessionLog sessionHistory[MAX_SESSION_LOGS];
int sessionLogCount = 0;
int sessionLogIndex = 0;

// System state variables
volatile int session_active = 0;
volatile int wakeup_requested = 0;
volatile uint32_t last_wakeup_time = 0;
volatile uint32_t last_significant_motion = 0;
volatile uint32_t last_gyro_print_time = 0;

#define MIN_WAKE_TIME_MS 5000

#define UART_RX_BUFFER_SIZE 64
volatile char uartRxBuffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uartRxIndex = 0;
volatile uint8_t cmdReady = 0;

// ================================
// FUNCTION DECLARATIONS
// ================================

// System functions
void internal_clock(void);
void systick_init(void);
uint32_t getTimeMs(void);

// Logger
void logger(LogLevel level, const char* msg);

// UART functions
void uart_init(void);
void uart_print(const char *str);
void process_command(const char* cmd);

// I2C functions
void i2c1_init(void);
void i2c2_init(void);
void i2c_bus_recovery(void);
int i2c1_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data);
int i2c1_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);
int i2c2_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data);
int i2c2_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);

// MPU6050 functions
void mpu6050_init(void);
void mpu6050_read_gyro_z(mpu6050_gyro_t *gyro);
void mpu6050_enable_motion_interrupt(void);
void mpu6050_diagnostic(void);
uint8_t mpu6050_read_devid(void);
void gyro_int_init(void);

// RTC functions
void rtc_get_datetime(char *buf, size_t len);
uint8_t bcd2bin(uint8_t bcd);

// Session management
void add_session_to_memory(const char* datetime, float rotations);
void print_session_history(void);
float get_total_rotations(void);

// Light sensor
int is_light_on(void);

// Flash memory
void flash_log_append(Event *event);
void log_session_event(EventType type, const char *date, float revs);

// Utility functions
void print_float(float val, int decimals);
// ================================
// UTILITY FUNCTIONS
// ================================

void logger(LogLevel level, const char* msg) {
    if (level <= log_level) {
        printf("%s\n", msg);
    }
}

void print_float(float val, int decimals) {
    char buf[32];
    float scale = 1.0f;
    
    for (int i = 0; i < decimals; i++) {
        scale *= 10.0f;
    }
    
    int isNegative = (val < 0);
    if (isNegative) val = -val;

    int wholePart = (int)val;
    int fracPart = (int)((val - wholePart) * scale + 0.5f);
    
    if (isNegative) {
        snprintf(buf, sizeof(buf), "-%d.%0*d", wholePart, decimals, fracPart);
    } else {
        snprintf(buf, sizeof(buf), "%d.%0*d", wholePart, decimals, fracPart);
    }

    uart_print(buf);
}

uint8_t bcd2bin(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

// ================================
// SYSTEM FUNCTIONS
// ================================

void internal_clock(void) {
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

// SysTick for millisecond timing
volatile uint32_t msTicks = 0;
void SysTick_Handler(void) { 
    msTicks++; 
}

uint32_t getTimeMs(void) { 
    return msTicks; 
}

// ================================
// RTC FUNCTIONS
// ================================

static uint8_t bin2bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

int set_ds3231_time(uint16_t year, uint8_t month, uint8_t day, uint8_t weekday,
                    uint8_t hour, uint8_t min, uint8_t sec) {
    uint8_t data[7];
    data[0] = bin2bcd(sec);
    data[1] = bin2bcd(min);
    data[2] = bin2bcd(hour);
    data[3] = bin2bcd(weekday);
    data[4] = bin2bcd(day);
    data[5] = bin2bcd(month);
    data[6] = bin2bcd(year % 100);

    for (int i = 0; i < 7; ++i) {
        if (i2c2_write_reg(DS3231_ADDR, 0x00 + i, data[i]) != 0)
            return -1;
    }
    return 0;
}

void rtc_get_datetime(char *buf, size_t len) {
    uint8_t data[7] = {0};
    if (i2c2_read_bytes(DS3231_ADDR, 0x00, data, 7) == 0) {
        uint8_t sec  = bcd2bin(data[0]);
        uint8_t min  = bcd2bin(data[1]);
        uint8_t hour = bcd2bin(data[2] & 0x3F);
        uint8_t day  = bcd2bin(data[4]);
        uint8_t mon  = bcd2bin(data[5] & 0x1F);
        uint16_t year = 2000 + bcd2bin(data[6]);
        snprintf(buf, len, "%04u-%02u-%02u %02u:%02u:%02u", year, mon, day, hour, min, sec);
    } else {
        snprintf(buf, len, "RTC ERROR");
    }
}

// ================================
// SESSION MANAGEMENT
// ================================

void add_session_to_memory(const char* datetime, float rotations) {
    strncpy(sessionHistory[sessionLogIndex].datetime, datetime, sizeof(sessionHistory[0].datetime));
    sessionHistory[sessionLogIndex].rotations = rotations;
    sessionHistory[sessionLogIndex].timestamp = getTimeMs();
    
    sessionLogIndex = (sessionLogIndex + 1) % MAX_SESSION_LOGS;
    
    if (sessionLogCount < MAX_SESSION_LOGS) {
        sessionLogCount++;
    }
}

void print_session_history(void) {
    uart_print("\r\n--- Session History ---\r\n");
    
    int start = (sessionLogCount == MAX_SESSION_LOGS) ? sessionLogIndex : 0;
    for (int i = 0; i < sessionLogCount; i++) {
        int idx = (start + i) % MAX_SESSION_LOGS;
        char entry[80];
        snprintf(entry, sizeof(entry), "[%s] Rotations: %.2f\r\n",
                sessionHistory[idx].datetime,
                sessionHistory[idx].rotations);
        uart_print(entry);
    }
}

float get_total_rotations(void) {
    float total = 0.0f;
    for (int i = 0; i < sessionLogCount; i++) {
        total += sessionHistory[i].rotations;
    }
    return total;
}

// ================================
// UART FUNCTIONS
// ================================
void uart_init(void) {
    // Enable GPIOA and USART2 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    // Configure PA2 (TX) and PA3 (RX) as alternate function AF7
    GPIOA->MODER &= ~((0x3 << (2 * 2)) | (0x3 << (3 * 2)));
    GPIOA->MODER |= (0x2 << (2 * 2)) | (0x2 << (3 * 2));
    GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
    GPIOA->AFR[0] |= (0x7 << (2 * 4)) | (0x7 << (3 * 4));
    
    // Configure USART2: 9600 baud, enable TX/RX and RXNE interrupt
    USART2->BRR = (uint16_t)(16000000 / 9600);
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    
    // Enable USART2 interrupt in NVIC
    NVIC_EnableIRQ(USART2_IRQn);
}

void uart_print(const char *str) {
    for (const char *p = str; *p; ++p) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *p;
    }
}

void process_command(const char* cmd) {
    if (strcmp(cmd, "dump") == 0) {
        print_session_history();
        char total[64];
        snprintf(total, sizeof(total), "Total rotations: %.2f\r\n", get_total_rotations());
        uart_print(total);
    }
    else if (strcmp(cmd, "clear") == 0) {
        sessionLogCount = 0;
        sessionLogIndex = 0;
        uart_print("Session history cleared\r\n");
    }
    else if (strcmp(cmd, "help") == 0) {
        uart_print("\r\nAvailable commands:\r\n"
                   "  dump     - Print session history\r\n"
                   "  clear    - Clear session history\r\n"
                   "  diag     - Run MPU6050 diagnostic\r\n"
                   "  gyro     - Read raw gyro values\r\n" 
                   "  help     - Show this help message\r\n");
    }
    else if (strcmp(cmd, "diag") == 0) {
        uart_print("\r\nRunning MPU6050 diagnostic...\r\n");
        mpu6050_diagnostic();
    }
    else if (strcmp(cmd, "gyro") == 0) {
        uart_print("\r\nReading gyro values...\r\n");
        for (int i = 0; i < 10; i++) {
            mpu6050_gyro_t gyro;
            mpu6050_read_gyro_z(&gyro);
            
            char msg[80];
            snprintf(msg, sizeof(msg), "Sample %d: Raw=%d, DPS=%.2f\r\n", i+1, gyro.z, gyro.z_dps);
            uart_print(msg);
            
            // Small delay between readings
            for (volatile int j = 0; j < 100000; j++);
        }
    }
    else {
        uart_print("Unknown command. Type 'help' for available commands.\r\n");
    }
}

// USART2 interrupt handler
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char c = USART2->DR;
        
        // Echo character back to terminal
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = c;
        
        // Any UART activity should prevent sleep
        last_significant_motion = getTimeMs();
        
        if (c == '\r' || c == '\n') {
            uartRxBuffer[uartRxIndex] = '\0';
            uart_print("\r\n");
            process_command((char*)uartRxBuffer);
            uartRxIndex = 0;
        }
        else if (c == 8 || c == 127) {  // Backspace or Delete
            if (uartRxIndex > 0) {
                uartRxIndex--;
                uart_print("\b \b");
            }
        }
        else if (uartRxIndex < UART_RX_BUFFER_SIZE - 1) {
            uartRxBuffer[uartRxIndex++] = c;
        }
    }
}

// ================================
// FLASH MEMORY FUNCTIONS
// ================================
#define FLASH_LOG_SECTOR   7
#define FLASH_LOG_ADDRESS  0x08060000
#define FLASH_LOG_SIZE     (128 * 1024)
#define FLASH_LOG_ENTRY_SIZE sizeof(Event)
#define FLASH_LOG_MAX_ENTRIES (FLASH_LOG_SIZE / FLASH_LOG_ENTRY_SIZE)

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

uint32_t flash_log_find_next(void) {
    for (uint32_t i = 0; i < FLASH_LOG_MAX_ENTRIES; ++i) {
        uint32_t *entry = (uint32_t*)(FLASH_LOG_ADDRESS + i * FLASH_LOG_ENTRY_SIZE);
        if (*entry == 0xFFFFFFFF) {
            return FLASH_LOG_ADDRESS + i * FLASH_LOG_ENTRY_SIZE;
        }
    }
    return 0;
}

void flash_log_append(Event *event) {
    uint32_t addr = flash_log_find_next();
    if (addr == 0) {
        flash_erase_sector(FLASH_LOG_SECTOR);
        addr = FLASH_LOG_ADDRESS;
    }
    flash_write_event(addr, event);
}

void log_session_event(EventType type, const char *date, float revs) {
    Event ev;
    ev.type = type;
    strncpy(ev.date, date, sizeof(ev.date));
    ev.revs = revs;
    flash_log_append(&ev);
}

// ================================
// I2C FUNCTIONS
// ================================
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
    
    // Configure accelerometer range: +/- 2g (most sensitive, 0x00)
    // This is important for motion detection sensitivity
    i2c1_write_reg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);
    
    // Additional delay for stability
    for (volatile int i = 0; i < 100000; i++);
}

// --- Read gyroscope Z-axis data ---
void mpu6050_read_gyro_z(mpu6050_gyro_t *gyro) {
    uint8_t buf[2];
    int result = i2c1_read_bytes(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, buf, 2);
    if (result != 0) {
        // On error, set values to zero
        gyro->z = 0;
        gyro->z_dps = 0.0f;
        
        // Debug: Log I2C read error
        char msg[64];
        snprintf(msg, sizeof(msg), "Gyro read error: %d", result);
        logger(LOG_ERROR, msg);
        return;
    }
    
    // Combine high and low bytes
    gyro->z = (int16_t)(buf[0] << 8 | buf[1]);
    
    // Convert to degrees per second (for ±250°/s range: LSB = 131 LSB/°/s)
    gyro->z_dps = (float)gyro->z / 131.0f;
    
    // Debug: Uncomment the next 3 lines if you want to see raw values
    // char debug_msg[80];
    // snprintf(debug_msg, sizeof(debug_msg), "Raw gyro: 0x%02X 0x%02X = %d = %.2f dps", buf[0], buf[1], gyro->z, gyro->z_dps);
    // logger(LOG_DEBUG, debug_msg);
}

// --- Gyroscope INT pin (e.g., PA10) as external interrupt ---
void gyro_int_init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // Set PA10 as input (default)
    GPIOA->MODER &= ~(0x3 << (10 * 2));
    
    // Enable an internal pull-up on PA10 (needed for open-drain MPU6050 output)
    GPIOA->PUPDR &= ~(0x3 << (10 * 2));
    GPIOA->PUPDR |= (0x1 << (10 * 2));   // 01: Pull-up
    
    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    // Map EXTI10 to PA10
    SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI10;
    SYSCFG->EXTICR[2] |= (0x0 << SYSCFG_EXTICR3_EXTI10_Pos); // Port A
    
    // Unmask EXTI10
    EXTI->IMR |= EXTI_IMR_MR10;
    
    // Configure ONLY falling edge trigger (active low interrupt from MPU6050)
    // MPU6050 pulls the line LOW when motion is detected (open drain output)
    EXTI->RTSR &= ~EXTI_RTSR_TR10;  // Disable rising edge
    EXTI->FTSR |= EXTI_FTSR_TR10;   // Enable falling edge only
    
    // Enable EXTI15_10 interrupt in NVIC
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// --- EXTI15 IRQ Handler (called on gyroscope interrupt) ---
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR10) {
        EXTI->PR = EXTI_PR_PR10; // Clear pending bit
        
        // Read interrupt status to see what triggered it
        uint8_t int_status = 0;
        int result = i2c1_read_bytes(MPU6050_ADDR, 0x3A, &int_status, 1);
        
        // Only process if we can read the interrupt status and motion was detected
        if (result == 0 && (int_status & 0x40)) {  // Bit 6 = motion interrupt
            // Read both gyro and accel data for diagnostic
            mpu6050_gyro_t gyro;
            mpu6050_read_gyro_z(&gyro);
            
            // Read accelerometer data (all three axes for better motion detection)
            uint8_t accel_data[6];
            if (i2c1_read_bytes(MPU6050_ADDR, 0x3B, accel_data, 6) == 0) {
                int16_t accel_x = (int16_t)(accel_data[0] << 8 | accel_data[1]);
                int16_t accel_y = (int16_t)(accel_data[2] << 8 | accel_data[3]);
                int16_t accel_z = (int16_t)(accel_data[4] << 8 | accel_data[5]);
                
                // Calculate magnitude of acceleration
                float accel_mag = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
                float accel_g = accel_mag / 16384.0f; // Convert to g (at ±2g range)
                
                char msg[96];
                snprintf(msg, sizeof(msg), "Motion Int: 0x%02X | Gyro: %.2f dps | Accel: %.2f g", 
                         int_status, gyro.z_dps, accel_g);
                logger(LOG_INFO, msg);
                
                // Set wake flags and update timing
                uint32_t current_time = getTimeMs();
                wakeup_requested = 1;
                last_significant_motion = current_time;
                last_wakeup_time = current_time;
                uart_print("Motion detected - waking up\r\n");
            }
        } else {
            // This was likely a spurious interrupt (wire connection, noise, etc.)
            char msg[64];
            snprintf(msg, sizeof(msg), "Spurious interrupt: status=0x%02X, result=%d", int_status, result);
            logger(LOG_INFO, msg);
        }
        
        // Always clear the MPU6050 interrupt by reading the status register again
        uint8_t dummy;
        i2c1_read_bytes(MPU6050_ADDR, 0x3A, &dummy, 1);
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

void mpu6050_enable_motion_interrupt(void) {
    // Keep power management consistent with mpu6050_init (use internal 8MHz oscillator)
    i2c1_write_reg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00); // Wake up, use internal oscillator
    
    // Ensure gyroscope and accelerometer are both configured properly
    i2c1_write_reg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);  // ±250°/s gyro range
    i2c1_write_reg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00); // ±2g accel range
    
    // Set motion threshold (MOT_THR, register 0x1F)
    // Each bit = 2mg (at ±2g scale). Lower value = more sensitive
    i2c1_write_reg(MPU6050_ADDR, 0x1F, MPU_WAKE_THRESHOLD); 

    // Set motion duration (MOT_DUR, register 0x20)
    // Value is in ms, 1 = 1ms (minimum duration above threshold)
    i2c1_write_reg(MPU6050_ADDR, 0x20, 1); // Very short duration for responsiveness

    // Configure INT_PIN_CFG (register 0x37)
    // 0x30 = INT is active low, open drain, latch until interrupt status read
    i2c1_write_reg(MPU6050_ADDR, 0x37, 0x30); 

    // Enable motion detection on INT_ENABLE (register 0x38)
    // Bit 6 = Motion detection interrupt enable
    i2c1_write_reg(MPU6050_ADDR, 0x38, 0x40);

    // Enable motion detection in MOT_DETECT_CTRL (register 0x69)
    // 0x15: Enable accelerometer intelligence, 1ms delay, low-power mode with 20Hz rate
    i2c1_write_reg(MPU6050_ADDR, 0x69, 0x15);
    
    // Set accelerometer low power wake control
    i2c1_write_reg(MPU6050_ADDR, 0x6C, 0x07); // Wake frequency = 1.25Hz for better power

    // Reset motion detection status by reading INT_STATUS (register 0x3A)
    uint8_t dummy;
    i2c1_read_bytes(MPU6050_ADDR, 0x3A, &dummy, 1);
    
    // Small delay to let settings take effect
    for (volatile int i = 0; i < 50000; i++);
}

/* Light sensor initialization */
// --- TSL2591 Light Sensor Definitions ---
#define TSL2591_ADDR 0x29
#define TSL2591_COMMAND_BIT 0xA0
#define TSL2591_REG_ENABLE 0x00
#define TSL2591_REG_CONTROL 0x01
#define TSL2591_REG_C0DATAL 0x14
#define TSL2591_REG_C0DATAH 0x15
#define TSL2591_REG_C1DATAL 0x16
#define TSL2591_REG_C1DATAH 0x17
#define TSL2591_ENABLE_POWERON 0x01
#define TSL2591_ENABLE_AEN 0x02
#define TSL2591_ENABLE_AIEN 0x10
#define TSL2591_ENABLE_NPIEN 0x80
#define TSL2591_INTEGRATION_100MS 0x00
#define TSL2591_GAIN_MED 0x10

// --- TSL2591 I2C helpers (on I2C1) ---
int tsl2591_write_reg(uint8_t reg, uint8_t value) {
    return i2c1_write_reg(TSL2591_ADDR, TSL2591_COMMAND_BIT | reg, value);
}

int tsl2591_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len) {
    return i2c1_read_bytes(TSL2591_ADDR, TSL2591_COMMAND_BIT | reg, buf, len);
}

void tsl2591_init(void) {
    // Power on and enable ALS
    tsl2591_write_reg(TSL2591_REG_ENABLE, TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN);
    // Set integration time and gain
    tsl2591_write_reg(TSL2591_REG_CONTROL, TSL2591_INTEGRATION_100MS | TSL2591_GAIN_MED);
}

uint32_t tsl2591_read_lux(void) {
    uint8_t buf[4];
    if (tsl2591_read_bytes(TSL2591_REG_C0DATAL, buf, 4) != 0) {
        return 0;
    }
    uint16_t ch0 = (uint16_t)(buf[1] << 8 | buf[0]);
    // uint16_t ch1 = (uint16_t)(buf[3] << 8 | buf[2]); // Unused for simple calculation
    // Simple lux calculation (not calibrated, for thresholding)
    uint32_t lux = ch0;
    return lux;
}

// --- Light Sensing Logic (like old.js) ---
int is_light_on(void) {
    uint32_t lux = tsl2591_read_lux();
    // Threshold: adjust as needed for your environment
    return lux > 1000; // Example threshold
}

// ================================
// MAIN FUNCTION
// ================================

int main(void) {
    // System state variables
    uint32_t current_time = 0;
    uint32_t last_sample_time = 0;
    uint32_t pull_timeout_start = 0;
    float pullRevolutions = 0.0f;
    float previousPullRevolutions = 0.0f;
    float sessionRevolutions = 0.0f;
    int last_light_state = is_light_on();
    
    char datetime[32];
    char log_msg[128];
    
    // Initialize peripherals
    tsl2591_init();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER = (GPIOA->MODER & ~(0x3 << (5 * 2))) | (0x1 << (5 * 2));
    
    uart_init();
    
    // Startup LED blink
    for (int i = 0; i < 5; i++) {
        GPIOA->ODR ^= (1 << 5);
        for (volatile int j = 0; j < 800000; j++);
    }
    
    // Initialize I2C and sensors
    i2c1_init();
    i2c2_init();
    for (volatile int i = 0; i < 1600000; i++);
    
    mpu6050_init();
    mpu6050_enable_motion_interrupt();
    gyro_int_init();
    
    // Set initial RTC time
    set_ds3231_time(2025, 6, 24, 2, 14, 30, 0);
    
    // Setup system timing
    SysTick_Config(SystemCoreClock / 1000);
    
    // Print startup message
    uint8_t devid = mpu6050_read_devid();
    snprintf(log_msg, sizeof(log_msg), "MPU6050 ID: 0x%02X (expect 0x69)\r\n", devid);
    uart_print(log_msg);
    
    rtc_get_datetime(datetime, sizeof(datetime));
    snprintf(log_msg, sizeof(log_msg), "[%s] Smart Spindle Ready\r\n", datetime);
    uart_print(log_msg);
    uart_print("Type 'help' for available commands\r\n\r\n");
    
    // Initialize timing variables to prevent premature sleep
    current_time = getTimeMs();
    last_significant_motion = current_time;
    last_wakeup_time = current_time;
    last_gyro_print_time = current_time;
    
    // Main loop
    while (1) {
        current_time = getTimeMs();
        
        // Sleep management - check all conditions
        uint32_t time_since_motion = current_time - last_significant_motion;
        uint32_t time_since_wakeup = current_time - last_wakeup_time;
        
        if (!session_active &&
            (time_since_motion > SESSION_TIMEOUT_MS) &&
            (time_since_wakeup > MIN_WAKE_TIME_MS) &&
            (uartRxIndex == 0)) {
            
            // Debug: Print sleep reason
            snprintf(log_msg, sizeof(log_msg), 
                     "Sleep conditions met: motion=%lu, wakeup=%lu, session=%d, uart=%d\r\n",
                     time_since_motion, time_since_wakeup, session_active, uartRxIndex);
            uart_print(log_msg);
            uart_print("Going to sleep...\r\n");
            
            SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
            __WFI();
            SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
            uart_print("Woke up!\r\n");
            
            // Reset timing variables on wake
            current_time = getTimeMs();
            last_wakeup_time = current_time;
            last_significant_motion = current_time;
            wakeup_requested = 0;
            last_gyro_print_time = 0;
            for (volatile int i = 0; i < 200000; i++);
            continue;
        }
        
        // Regular gyro value printing
        if ((current_time - last_gyro_print_time) > GYRO_PRINT_INTERVAL_MS) {
            // Read fresh gyro data
            mpu6050_gyro_t fresh_gyro;
            mpu6050_read_gyro_z(&fresh_gyro);
            
            uart_print("Gyro Z: ");
            print_float(fresh_gyro.z_dps, 2);
            uart_print(" dps\r\n");
            
            last_gyro_print_time = current_time;
            
            // If we're reading the gyro, consider this activity
            if (fabsf(fresh_gyro.z_dps) > 0.5f) {
                last_significant_motion = current_time;
            }
        }
        
        // --- Light sensor detection (roll change event) ---
        int light_now = is_light_on();
        if (light_now && !last_light_state) {
            rtc_get_datetime(datetime, sizeof(datetime));
            uart_print("[ROLL CHANGE DETECTED] Light ON at ");
            uart_print(datetime);
            uart_print("\r\n");
            log_session_event(EVENT_ROLL_CHANGE, datetime, 0.0f);
            // Reset session-related variables on roll change
            sessionRevolutions = 0.0f;
            pullRevolutions = 0.0f;
            previousPullRevolutions = 0.0f;
            session_active = 0;
            pull_timeout_start = 0;
            last_significant_motion = current_time;
        }
        last_light_state = light_now;
        
        // --- MPU6050 gyro reading and error recovery ---
        mpu6050_gyro_t gyro;
        mpu6050_read_gyro_z(&gyro);
        current_time = getTimeMs();
        if (gyro.z == 0 && gyro.z_dps == 0.0f) {
            // If sensor returns zero, attempt bus recovery every 5 seconds
            static uint32_t last_recovery = 0;
            if (current_time - last_recovery > 5000) {
                i2c_bus_recovery();
                i2c1_init();
                mpu6050_init();
                last_recovery = current_time;
            }
            continue;
        }
        
        // --- Time delta calculation ---
        if (last_sample_time == 0) last_sample_time = current_time;
        float dt = (current_time - last_sample_time) / 1000.0f;
        last_sample_time = current_time;
        
        // --- Gyro deadband filtering ---
        if (fabsf(gyro.z_dps) < 1.0f)
            gyro.z_dps = 0.0f;
        
        // --- Integrate gyro reading into revolutions ---
        float delta_revs = (gyro.z_dps * dt) / 360.0f;
        pullRevolutions += delta_revs;
        
        // --- Significant movement detection ---
        if (fabsf(pullRevolutions - previousPullRevolutions) > SIGNIFICANT_GYRO_THRESHOLD) {
            previousPullRevolutions = pullRevolutions;
            last_significant_motion = current_time;  // Reset the timeout counter on significant movement
            
            // Start a session if not already active
            if (!session_active) {
                wakeup_requested = 0;
                session_active = 1;
                sessionRevolutions = 0.0f;
                pullRevolutions = 0.0f;
                previousPullRevolutions = 0.0f;
                rtc_get_datetime(datetime, sizeof(datetime));
                snprintf(log_msg, sizeof(log_msg), "[%s] Session started\r\n", datetime);
                uart_print(log_msg);
            }
            pull_timeout_start = current_time;
        }
        // Also update motion timer for any gyro activity above deadband
        else if (fabsf(gyro.z_dps) > 1.0f) {
            last_significant_motion = current_time;
        }
        
        // --- Handle pull timeout (1 second of inactivity) ---
        if (session_active && pull_timeout_start &&
            ((current_time - pull_timeout_start) > PULL_TIMEOUT_MS)) {
            
            float roundedRevs = roundf(pullRevolutions * 4.0f) / 4.0f;
            if (fabsf(roundedRevs) > 0.25f) {
                uart_print("Pull end: ");
                print_float(roundedRevs, 2);
                uart_print(" revolutions\r\n");
                sessionRevolutions += roundedRevs;
            }
            pullRevolutions = 0.0f;
            previousPullRevolutions = 0.0f;
            pull_timeout_start = 0;
        }
        
        // --- Handle session timeout (10 seconds after last motion) ---
        if (session_active && (current_time - last_significant_motion > SESSION_TIMEOUT_MS)) {
            session_active = 0;
            float absSessionRevs = fabsf(sessionRevolutions);
            if (absSessionRevs > 0.25f) {
                rtc_get_datetime(datetime, sizeof(datetime));
                uart_print("[");
                uart_print(datetime);
                uart_print("] Session ended. Total revolutions: ");
                print_float(absSessionRevs, 2);
                uart_print("\r\n");
                add_session_to_memory(datetime, absSessionRevs);
                log_session_event(EVENT_DISPENSE, datetime, absSessionRevs);
            } else {
                uart_print("Session timeout (< 0.25 revolutions)\r\n");
            }
            sessionRevolutions = 0.0f;
            pullRevolutions = 0.0f;
            previousPullRevolutions = 0.0f;
            pull_timeout_start = 0;
        }
        
        // --- (Optional) Debug print to monitor variables ---
        // char dbg[64];
        // snprintf(dbg, sizeof(dbg), "dt=%.3f, gyro.z_dps=%.2f, delta_revs=%.3f, pullRevs=%.3f\r\n",
        //         dt, gyro.z_dps, delta_revs, pullRevolutions);
        // uart_print(dbg);
        
        // --- (Optional) Short delay to ease I2C bus load ---
        // for (volatile int i = 0; i < 500; i++);
    }
    
    return 0;
}

// --- MPU6050 diagnostic function ---
void mpu6050_diagnostic(void) {
    char msg[128];
    
    // Check device ID
    uint8_t devid = mpu6050_read_devid();
    snprintf(msg, sizeof(msg), "MPU6050 Device ID: 0x%02X (expect 0x69 with AD0=1)\r\n", devid);
    uart_print(msg);
    
    // Read power management register
    uint8_t pwr_mgmt = 0;
    i2c1_read_bytes(MPU6050_ADDR, MPU6050_PWR_MGMT_1, &pwr_mgmt, 1);
    snprintf(msg, sizeof(msg), "Power Management: 0x%02X (bit 6 = sleep mode)\r\n", pwr_mgmt);
    uart_print(msg);
    
    // Read interrupt config
    uint8_t int_cfg = 0;
    i2c1_read_bytes(MPU6050_ADDR, 0x37, &int_cfg, 1);
    snprintf(msg, sizeof(msg), "INT Pin Config: 0x%02X (0x30 = active low, latched)\r\n", int_cfg);
    uart_print(msg);
    
    // Read interrupt enable status
    uint8_t int_enable = 0;
    i2c1_read_bytes(MPU6050_ADDR, 0x38, &int_enable, 1);
    snprintf(msg, sizeof(msg), "INT Enable: 0x%02X (bit 6 = motion interrupt)\r\n", int_enable);
    uart_print(msg);
    
    // Read interrupt status
    uint8_t int_status = 0;
    i2c1_read_bytes(MPU6050_ADDR, 0x3A, &int_status, 1);
    snprintf(msg, sizeof(msg), "INT Status: 0x%02X (bit 6 = motion detected)\r\n", int_status);
    uart_print(msg);
    
    // Check motion threshold
    uint8_t mot_thr = 0;
    i2c1_read_bytes(MPU6050_ADDR, 0x1F, &mot_thr, 1);
    snprintf(msg, sizeof(msg), "Motion Threshold: %d (lower = more sensitive, each = 2mg)\r\n", mot_thr);
    uart_print(msg);
    
    // Check motion duration
    uint8_t mot_dur = 0;
    i2c1_read_bytes(MPU6050_ADDR, 0x20, &mot_dur, 1);
    snprintf(msg, sizeof(msg), "Motion Duration: %d ms\r\n", mot_dur);
    uart_print(msg);
    
    // Check motion detection control
    uint8_t mot_detect_ctrl = 0;
    i2c1_read_bytes(MPU6050_ADDR, 0x69, &mot_detect_ctrl, 1);
    snprintf(msg, sizeof(msg), "Motion Detect Ctrl: 0x%02X\r\n", mot_detect_ctrl);
    uart_print(msg);
    
    // Check GPIO pin state (PA10)
    uint8_t pin_state = (GPIOA->IDR & (1 << 10)) ? 1 : 0;
    snprintf(msg, sizeof(msg), "INT Pin State: %d (should be 1 when idle, 0 during motion)\r\n", pin_state);
    uart_print(msg);
    
    // Check EXTI configuration
    uint32_t exti_rtsr = EXTI->RTSR & EXTI_RTSR_TR10;
    uint32_t exti_ftsr = EXTI->FTSR & EXTI_FTSR_TR10;
    snprintf(msg, sizeof(msg), "EXTI Config: Rising=%d, Falling=%d (should be 0,1)\r\n", 
             exti_rtsr ? 1 : 0, exti_ftsr ? 1 : 0);
    uart_print(msg);
    
    // Read current accelerometer values
    uint8_t accel_data[6];
    i2c1_read_bytes(MPU6050_ADDR, 0x3B, accel_data, 6);
    int16_t accel_x = (int16_t)(accel_data[0] << 8 | accel_data[1]);
    int16_t accel_y = (int16_t)(accel_data[2] << 8 | accel_data[3]);
    int16_t accel_z = (int16_t)(accel_data[4] << 8 | accel_data[5]);
    
    snprintf(msg, sizeof(msg), "Accel X: %d, Y: %d, Z: %d\r\n", accel_x, accel_y, accel_z);
    uart_print(msg);
    
    // Calculate accelerometer magnitude in g's
    float accel_mag = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    float accel_g = accel_mag / 16384.0f;
    snprintf(msg, sizeof(msg), "Accel Magnitude: %.2f g\r\n", accel_g);
    uart_print(msg);
    
    // Read current gyro values
    mpu6050_gyro_t gyro;
    mpu6050_read_gyro_z(&gyro);
    snprintf(msg, sizeof(msg), "Gyro Z: Raw=%d, DPS=%.2f\r\n", gyro.z, gyro.z_dps);
    uart_print(msg);
    
    // Read gyro config register
    uint8_t gyro_config = 0;
    i2c1_read_bytes(MPU6050_ADDR, MPU6050_GYRO_CONFIG, &gyro_config, 1);
    snprintf(msg, sizeof(msg), "Gyro Config: 0x%02X (should be 0x00 for ±250°/s)\r\n", gyro_config);
    uart_print(msg);
    
    // Test direct register read for gyro Z
    uint8_t gyro_raw[2];
    int gyro_result = i2c1_read_bytes(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, gyro_raw, 2);
    snprintf(msg, sizeof(msg), "Direct Gyro Z read: result=%d, bytes=0x%02X 0x%02X\r\n", 
             gyro_result, gyro_raw[0], gyro_raw[1]);
    uart_print(msg);
    
    uart_print("------- End of MPU6050 Diagnostic -------\r\n");
}