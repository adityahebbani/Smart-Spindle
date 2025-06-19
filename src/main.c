#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

/* Logger */
typedef enum { LOG_OFF, LOG_ERROR, LOG_INFO, LOG_DEBUG, LOG_SILLY } LogLevel;
LogLevel log_level = LOG_INFO;

void log(LogLevel level, const char* msg) {
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

// Flash log configuration
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

// Example: log an event (call this from your interrupt/session handler)
void log_session_event(EventType type, const char *date, float revs) {
    Event ev;
    ev.type = type;
    strncpy(ev.date, date, sizeof(ev.date));
    ev.revs = revs;
    flash_log_append(&ev);
}

// --- I2C1 (PB6=SCL, PB7=SDA) setup for accelerometer ---
void i2c1_init(void) {
    // Enable GPIOB and I2C1 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // Set PB6, PB7 to alternate function (AF4)
    GPIOB->MODER &= ~((0x3 << (6 * 2)) | (0x3 << (7 * 2)));
    GPIOB->MODER |= (0x2 << (6 * 2)) | (0x2 << (7 * 2));
    GPIOB->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->AFR[0] |= (0x4 << (6 * 4)) | (0x4 << (7 * 4));
    // Open-drain, pull-up
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);
    GPIOB->PUPDR &= ~((0x3 << (6 * 2)) | (0x3 << (7 * 2)));
    GPIOB->PUPDR |= (0x1 << (6 * 2)) | (0x1 << (7 * 2));
    // Configure I2C1: 100kHz standard mode (assuming 16MHz PCLK1)
    I2C1->CR2 = 16; // 16 MHz
    I2C1->CCR = 80; // 100kHz
    I2C1->TRISE = 17; // 1000ns / (1/16MHz) + 1
    I2C1->CR1 |= I2C_CR1_PE;
}

// --- ADXL345 I2C address (SDO=GND) ---
#define ADXL345_ADDR 0x53

// --- I2C1 Write/Read helpers ---
void i2c1_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
    I2C1->DR = (dev_addr << 1);
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->CR1 |= I2C_CR1_STOP;
}

void i2c1_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    // Write register address
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
    I2C1->DR = (dev_addr << 1);
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    // Restart for read
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
    I2C1->DR = (dev_addr << 1) | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    for (uint8_t i = 0; i < len; ++i) {
        if (i == len - 1) I2C1->CR1 &= ~I2C_CR1_ACK;
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        buf[i] = I2C1->DR;
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 |= I2C_CR1_ACK;
}

// --- ADXL345 Initialization ---
void adxl345_init(void) {
    // Wake up (POWER_CTL = 0x2D, value = 0x08)
    i2c1_write_reg(ADXL345_ADDR, 0x2D, 0x08);
    // Set data format: full resolution, +/- 2g (DATA_FORMAT = 0x31, value = 0x08)
    i2c1_write_reg(ADXL345_ADDR, 0x31, 0x08);
    // Set BW_RATE to 100 Hz (0x0A)
    i2c1_write_reg(ADXL345_ADDR, 0x2C, 0x0A);
    // Enable measurement (POWER_CTL = 0x2D, value = 0x08)
    i2c1_write_reg(ADXL345_ADDR, 0x2D, 0x08);
    // Enable DATA_READY interrupt on INT1
    i2c1_write_reg(ADXL345_ADDR, 0x2E, 0x80); // INT_ENABLE: DATA_READY
    i2c1_write_reg(ADXL345_ADDR, 0x2F, 0x80); // INT_MAP: DATA_READY to INT1 (default)
}

// --- Read acceleration data (X, Y, Z) ---
typedef struct { int16_t x, y, z; } adxl345_axes_t;

void adxl345_read_axes(adxl345_axes_t *axes) {
    uint8_t buf[6];
    i2c1_read_bytes(ADXL345_ADDR, 0x32, buf, 6);
    axes->x = (int16_t)(buf[1] << 8 | buf[0]);
    axes->y = (int16_t)(buf[3] << 8 | buf[2]);
    axes->z = (int16_t)(buf[5] << 8 | buf[4]);
}

// --- Accelerometer INT pin (e.g., PC0) as external interrupt ---
void accel_int_init(void) {
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

// --- EXTI0 IRQ Handler (called on accelerometer interrupt) ---
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0; // Clear pending bit
        // TODO: Handle accelerometer event (read data, log, etc.)
        log(LOG_INFO, "Accelerometer interrupt!");
        // Read and print axes
        adxl345_axes_t axes;
        adxl345_read_axes(&axes);
        char msg[64];
        snprintf(msg, sizeof(msg), "ADXL345: X=%d Y=%d Z=%d", axes.x, axes.y, axes.z);
        log(LOG_INFO, msg);
    }
}

// --- Test: Read and print ADXL345 DEVID register ---
uint8_t adxl345_read_devid(void) {
    uint8_t devid = 0;
    // Read DEVID register (0x00)
    i2c1_read_bytes(ADXL345_ADDR, 0x00, &devid, 1);
    return devid;
}

/* Main */
int main(void) {
    // 1. Set up internal clock
    internal_clock();
    // 2. Initialize I2C1 for ADXL345
    i2c1_init();
    // 3. Initialize ADXL345
    adxl345_init();
    // 4. Initialize accelerometer interrupt
    accel_int_init();

    // In main(), after initializing I2C and before reading DEVID:
    // Add a delay (~100ms) to allow ADXL345 to power up
    for (volatile int i = 0; i < 16000000; ++i); // ~100ms at 16MHz
    uint8_t devid = adxl345_read_devid();
    char msg[32];
    snprintf(msg, sizeof(msg), "ADXL345 DEVID: 0x%02X", devid);
    log(LOG_INFO, msg);

    while (1) {
        // Main loop
    }
}
