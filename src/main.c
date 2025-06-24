#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define DS3231_ADDR 0x68
#define ROTATION_THRESHOLD  1000    // Adjust this threshold for your application
#define SESSION_TIMEOUT_MS  10000   // 10 seconds

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

// Example: log an event (call this from your interrupt/session handler)
void log_session_event(EventType type, const char *date, float revs) {
    Event ev;
    ev.type = type;
    strncpy(ev.date, date, sizeof(ev.date));
    ev.revs = revs;
    flash_log_append(&ev);
}

// --- I2C1 (PB8=SCL, PB9=SDA) setup for accelerometer ---
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

// --- ADXL345 I2C address (SDO=GND) ---
#define ADXL345_ADDR 0x53

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

// --- ADXL345 Initialization ---
void adxl345_init(void) {
    // Reset the device
    i2c1_write_reg(ADXL345_ADDR, 0x2D, 0x00); // Reset POWER_CTL
    
    // Verify device ID (should be 0xE5)
    uint8_t devid = 0;
    if (i2c1_read_bytes(ADXL345_ADDR, 0x00, &devid, 1) == 0) {
        if (devid != 0xE5) {
            // Device ID is wrong - try bus recovery
            i2c_bus_recovery();
            i2c1_init();
        }
    }
    
    // Configure data format: full resolution, +/- 8g (DATA_FORMAT = 0x31, value = 0x0B)
    i2c1_write_reg(ADXL345_ADDR, 0x31, 0x0B);
    
    // Set BW_RATE to 100 Hz (0x0A)
    i2c1_write_reg(ADXL345_ADDR, 0x2C, 0x0A);
    
    // Enable measurement mode (POWER_CTL = 0x2D, value = 0x08)
    i2c1_write_reg(ADXL345_ADDR, 0x2D, 0x08);
    
    // Optionally enable FIFO in stream mode for better reading stability
    i2c1_write_reg(ADXL345_ADDR, 0x38, 0x80); // FIFO_CTL: Stream mode
}

// --- Read acceleration data (X, Y, Z) ---
typedef struct { int16_t x, y, z; } adxl345_axes_t;

void adxl345_read_axes(adxl345_axes_t *axes) {
    uint8_t buf[6];
    int result = i2c1_read_bytes(ADXL345_ADDR, 0x32, buf, 6);
    if (result != 0) {
        // On error, set values to a recognizable pattern
        axes->x = -9999;
        axes->y = -9999;
        axes->z = -9999;
        return;
    }
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
    if (i2c1_read_bytes(ADXL345_ADDR, 0x00, &devid, 1) != 0) {
        // Communication error - try recovery
        i2c_bus_recovery();
        i2c1_init();
        i2c1_read_bytes(ADXL345_ADDR, 0x00, &devid, 1);
    }
    return devid;
}

/* RTC Logic */
volatile uint32_t msTicks = 0;
void SysTick_Handler(void) { msTicks++; }
uint32_t getTimeMs(void) { return msTicks; }

// Helper: Convert BCD to binary
static uint8_t bcd2bin(uint8_t val) {
    return (val >> 4) * 10 + (val & 0x0F);
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
        if (i2c1_write_reg(DS3231_ADDR, 0x00 + i, data[i]) != 0)
            return -1; // Error
    }
    return 0; // Success
}

// Dummy RTC function (replace with your actual RTC read)
void rtc_get_datetime(char *buf, size_t len) {
    uint8_t data[7] = {0};
    if (i2c1_read_bytes(DS3231_ADDR, 0x00, data, 7) == 0) {
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

// --- Main function ---
int main(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Set PA5 as output (LED)
    GPIOA->MODER &= ~(0x3 << (5 * 2));
    GPIOA->MODER |= (0x1 << (5 * 2));
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // Set PA2 to alternate function (AF7)
    GPIOA->MODER &= ~(0x3 << (2 * 2));
    GPIOA->MODER |= (0x2 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |= (0x7 << (2 * 4));
    // Set PA3 to alternate function (AF7)
    GPIOA->MODER &= ~(0x3 << (3 * 2));
    GPIOA->MODER |= (0x2 << (3 * 2));
    GPIOA->AFR[0] &= ~(0xF << (3 * 4));
    GPIOA->AFR[0] |= (0x7 << (3 * 4));
    // APB1 is 16 MHz by default
    USART2->BRR = (uint16_t)(16000000 / 9600);
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;

    // Blink LED a few times to show startup
    for (int i = 0; i < 5; ++i) {
        GPIOA->ODR ^= (1 << 5);
        for (volatile int j = 0; j < 800000; ++j);
    }

    // --- I2C1/ADXL345 minimal test ---
    // i2c_bus_recovery();
    i2c1_init();

    // Add delay
    for (volatile int i = 0; i < 1600000; ++i);

    adxl345_init();
    for (volatile int i = 0; i < 1600000; ++i); // ~100ms delay

    set_ds3231_time(2025, 6, 24, 2, 14, 30, 0);

    // Check device ID - should be 0xE5 for ADXL345
    uint8_t devid = adxl345_read_devid();
    char init_msg[40];
    snprintf(init_msg, sizeof(init_msg), "ADXL345 ID: 0x%02X (expect 0xE5)\r\n", devid);
    for (const char *p = init_msg; *p; ++p) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *p;
    }

    // Systick timer for millisecond timing
    SysTick_Config(SystemCoreClock / 1000); // 1 ms tick
    int session_active = 0;
    float session_rotations = 0.0f;
    float prev_z = 0.0f;
    uint32_t last_motion_time = 0;

    // Main data reading loop
    while (1) {
        // Read and print accelerometer axes
        adxl345_axes_t axes;
        adxl345_read_axes(&axes);
        
    //     // Check if axes are valid (not our error code)
    //     if (axes.x == -9999) {
    //         // Try recovery
    //         i2c_bus_recovery();
    //         i2c1_init();
    //         adxl345_init();
    //         continue;
    //     }
        
    //     char axes_msg[64];
    //     snprintf(axes_msg, sizeof(axes_msg), "X=%5d Y=%5d Z=%5d\r\n", axes.x, axes.y, axes.z);
    //     for (const char *p = axes_msg; *p; ++p) {
    //         while (!(USART2->SR & USART_SR_TXE));
    //         USART2->DR = *p;
    //     }

    //     // Shorter delay for more responsive readings
    //     GPIOA->ODR ^= (1 << 5);
    //     for (volatile int i = 0; i < 200000; ++i); // 1/4 of your original delay for more frequent updates
    // 

    // Use Z axis for rotation (replace with your actual calculation if needed)
        float z = (float)axes.z;

        // Calculate delta
        float dz = fabsf(z - prev_z);
        prev_z = z;

        // Detect significant motion
        if (dz > ROTATION_THRESHOLD) {
            if (!session_active) {
                session_active = 1;
                session_rotations = 0.0f;
                char msg[64];
                snprintf(msg, sizeof(msg), "Session started\r\n");
                for (const char *p = msg; *p; ++p) {
                    while (!(USART2->SR & USART_SR_TXE));
                    USART2->DR = *p;
                }
            }
            session_rotations += dz; // Or use your own conversion to revolutions
            last_motion_time = getTimeMs();
        }

        // If session is active, check for timeout
        if (session_active && (getTimeMs() - last_motion_time > SESSION_TIMEOUT_MS)) {
            session_active = 0;
            char datetime[32];
            rtc_get_datetime(datetime, sizeof(datetime));
            char msg[128];
            snprintf(msg, sizeof(msg), "[%s] Session ended. Rotations: %.2f\r\n", datetime, session_rotations / ROTATION_THRESHOLD);
            for (const char *p = msg; *p; ++p) {
                while (!(USART2->SR & USART_SR_TXE));
                USART2->DR = *p;
            }
        }

        // Short delay
        for (volatile int i = 0; i < 20000; ++i);
    }

}

