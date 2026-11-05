/**
 * @file s7_226_plc.h
 * @brief Master Header for SIMATIC S7-226 PLC Firmware (APM32E103 / STM32F103HD Cortex-M3)
 *
 * Reconstructed from binary analysis of APM32E103_backup.bin (512 KB Flash, 64 KB SRAM).
 * Toolchain: Keil MDK-ARM (armcc v5) with STM32F10x Standard Peripheral Library (SPL).
 *
 * Contents:
 *   1. Standard types, Ghidra compatibility typedefs, and Cortex-M3 Bit-Band macros
 *   2. STM32F103HD / APM32E103 peripheral register structures & memory-mapped addresses
 *   3. Flash memory layout constants (0x08000000 - 0x0807FFFF) & System Memory UID
 *   4. PLC Memory Area Descriptor Table definitions (0x20000250 - 0x2000027B)
 *   5. Complete SRAM variable map (0x20000000 - 0x2000CE38) with symbolic names
 *   6. Firmware subsystem function prototypes with exact Flash addresses (0x080xxxxx)
 */

#ifndef S7_226_PLC_H
#define S7_226_PLC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 1. COMPILER PRIMITIVES, GHIDRA COMPATIBILITY TYPEDEFS & BIT-BAND MACROS
 * ============================================================================ */

#define __I     volatile const  /*!< Read-only hardware register              */
#define __O     volatile        /*!< Write-only hardware register             */
#define __IO    volatile        /*!< Read/write hardware register             */

typedef uint8_t   undefined;
typedef uint8_t   undefined1;
typedef uint16_t  undefined2;
typedef uint32_t  undefined4;
typedef uint64_t  undefined8;
typedef uint8_t   byte;
typedef uint8_t   uchar;
typedef uint16_t  ushort;
typedef uint32_t  uint;
typedef int32_t    sword;
typedef void      (*code)(void);

/** Direct memory-mapped register/variable access macros */
#define REG8(addr)              (*(volatile uint8_t  *)(uintptr_t)(addr))
#define REG16(addr)             (*(volatile uint16_t *)(uintptr_t)(addr))
#define REG32(addr)             (*(volatile uint32_t *)(uintptr_t)(addr))
#define SREG8(addr)             (*(volatile int8_t   *)(uintptr_t)(addr))
#define SREG16(addr)            (*(volatile int16_t  *)(uintptr_t)(addr))
#define SREG32(addr)            (*(volatile int32_t  *)(uintptr_t)(addr))
#define PTR8(addr)              ((uint8_t  *)(uintptr_t)(addr))
#define PTR16(addr)             ((uint16_t *)(uintptr_t)(addr))
#define PTR32(addr)             ((uint32_t *)(uintptr_t)(addr))

/**
 * Cortex-M3 Bit-Band Alias Calculation:
 *   bit_word_addr = bit_band_base + (byte_offset * 32) + (bit_number * 4)
 */
#define SRAM_BASE_ADDR          0x20000000UL
#define SRAM_BB_BASE_ADDR       0x22000000UL
#define PERIPH_BASE_ADDR        0x40000000UL
#define PERIPH_BB_BASE_ADDR     0x42000000UL

#define SRAM_BB_ADDR(byte_addr, bit) \
    (SRAM_BB_BASE_ADDR + (((uint32_t)(byte_addr) - SRAM_BASE_ADDR) * 32UL) + ((uint32_t)(bit) * 4UL))
#define PERIPH_BB_ADDR(reg_addr, bit) \
    (PERIPH_BB_BASE_ADDR + (((uint32_t)(reg_addr) - PERIPH_BASE_ADDR) * 32UL) + ((uint32_t)(bit) * 4UL))

#define SRAM_BB(byte_addr, bit)   REG32(SRAM_BB_ADDR(byte_addr, bit))
#define PERIPH_BB(reg_addr, bit)  REG32(PERIPH_BB_ADDR(reg_addr, bit))

/* ============================================================================
 * 2. STM32F103HD / APM32E103 PERIPHERAL REGISTER DEFINITIONS
 * ============================================================================ */

/* --- Bus Base Addresses --- */
#define APB1PERIPH_BASE         (PERIPH_BASE_ADDR + 0x00000000UL) /* 0x40000000 */
#define APB2PERIPH_BASE         (PERIPH_BASE_ADDR + 0x00010000UL) /* 0x40010000 */
#define AHBPERIPH_BASE          (PERIPH_BASE_ADDR + 0x00018000UL) /* 0x40018000 */
#define SCS_BASE_ADDR           0xE000E000UL                      /* Cortex-M3 System Control Space */

/* --- APB1 Peripherals (0x40000000 - 0x40007FFF) --- */
#define TIM2_BASE               0x40000000UL  /* 100 us high-res tick timer (TIM2_IRQHandler 0x0803f58e) */
#define TIM3_BASE               0x40000400UL  /* Timed interrupt 0 SMB34 (TIM3_IRQHandler 0x0803f680)   */
#define TIM4_BASE               0x40000800UL  /* Timed interrupt 1 SMB35 (TIM4_IRQHandler 0x0803f6b2)   */
#define TIM5_BASE               0x40000C00UL  /* High-speed counter timer (TIM5_IRQHandler 0x0803f6e4)  */
#define TIM6_BASE               0x40001000UL  /* Port 0 (USART1) frame timer (TIM6_IRQHandler 0x0803f7fc)*/
#define TIM7_BASE               0x40001400UL  /* Port 1 (USART2) frame timer (TIM7_IRQHandler 0x0803f84a)*/
#define RTC_BASE                0x40002800UL  /* Real-Time Clock */
#define WWDG_BASE               0x40002C00UL  /* Window Watchdog */
#define IWDG_BASE               0x40003000UL  /* Independent Watchdog */
#define SPI2_BASE               0x40003800UL  /* SPI2 (W5500 Ethernet controller interface) */
#define SPI3_BASE               0x40003C00UL  /* SPI3 */
#define USART2_BASE             0x40004400UL  /* PLC Comm Port 1 (RS-485 PPI / Freeport / Modbus) */
#define USART3_BASE             0x40004800UL  /* PLC Comm Port 2 / Internal Expansion Bus UART */
#define UART4_BASE              0x40004C00UL  /* UART4 */
#define UART5_BASE              0x40005000UL  /* UART5 */
#define I2C1_BASE               0x40005400UL  /* I2C1 */
#define I2C2_BASE               0x40005800UL  /* I2C2 */
#define CAN1_BASE               0x40006400UL  /*bxCAN1 controller (used in CAN helper 0x08016ae8) */
#define BKP_BASE                0x40006C00UL  /* Backup Registers (DR1..DR42, RTC calibration) */
#define PWR_BASE                0x40007000UL  /* Power Control (PWR_CR at 0x40007000, DBP bit) */
#define DAC_BASE                0x40007400UL  /* Dual-channel 12-bit DAC */

/* --- APB2 Peripherals (0x40010000 - 0x40017FFF) --- */
#define AFIO_BASE               0x40010000UL  /* Alternate Function I/O & EXTI mapping */
#define EXTI_BASE               0x40010400UL  /* External Interrupt/Event Controller */
#define GPIOA_BASE              0x40010800UL  /* GPIO Port A */
#define GPIOB_BASE              0x40010C00UL  /* GPIO Port B */
#define GPIOC_BASE              0x40011000UL  /* GPIO Port C */
#define GPIOD_BASE              0x40011400UL  /* GPIO Port D */
#define GPIOE_BASE              0x40011800UL  /* GPIO Port E */
#define GPIOF_BASE              0x40011C00UL  /* GPIO Port F */
#define GPIOG_BASE              0x40012000UL  /* GPIO Port G */
#define ADC1_BASE               0x40012400UL  /* ADC1 */
#define ADC2_BASE               0x40012800UL  /* ADC2 */
#define TIM1_BASE               0x40012C00UL  /* Advanced Timer 1 (PTO0 / PWM0 output Q0.0) */
#define SPI1_BASE               0x40013000UL  /* SPI1 */
#define TIM8_BASE               0x40013400UL  /* Advanced Timer 8 (PTO1 / PWM1 output Q0.1) */
#define USART1_BASE             0x40013800UL  /* PLC Comm Port 0 (RS-485 PPI / Freeport / Modbus) */
#define ADC3_BASE               0x40013C00UL  /* ADC3 */

/* --- AHB Peripherals (0x40020000 - 0x40023FFF) --- */
#define DMA1_BASE               0x40020000UL  /* DMA1 Controller */
#define DMA2_BASE               0x40020400UL  /* DMA2 Controller */
#define RCC_BASE                0x40021000UL  /* Reset and Clock Control */
#define FLASH_R_BASE            0x40022000UL  /* Flash Memory Interface Registers */
#define CRC_BASE                0x40023000UL  /* CRC Calculation Unit */

/* --- Cortex-M3 Core Peripherals (0xE000E000) --- */
#define SYSTICK_BASE            0xE000E010UL  /* SysTick 24-bit Timer (1 ms system tick) */
#define NVIC_BASE               0xE000E100UL  /* Nested Vectored Interrupt Controller */
#define SCB_BASE                0xE000ED00UL  /* System Control Block */

/* --- Peripheral Register Structs --- */

typedef struct {
    __IO uint32_t CRL;          /* 0x00: Port configuration register low      */
    __IO uint32_t CRH;          /* 0x04: Port configuration register high     */
    __I  uint32_t IDR;          /* 0x08: Port input data register             */
    __IO uint32_t ODR;          /* 0x0C: Port output data register            */
    __O  uint32_t BSRR;         /* 0x10: Port bit set/reset register          */
    __O  uint32_t BRR;          /* 0x14: Port bit reset register              */
    __IO uint32_t LCKR;         /* 0x18: Port configuration lock register     */
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t EVCR;         /* 0x00: Event control register               */
    __IO uint32_t MAPR;         /* 0x04: AF remap and debug I/O config        */
    __IO uint32_t EXTICR[4];    /* 0x08..0x14: External interrupt config 1..4 */
    uint32_t      RESERVED0;    /* 0x18 */
    __IO uint32_t MAPR2;        /* 0x1C: AF remap and debug I/O config 2      */
} AFIO_TypeDef;

typedef struct {
    __IO uint32_t IMR;          /* 0x00: Interrupt mask register              */
    __IO uint32_t EMR;          /* 0x04: Event mask register                  */
    __IO uint32_t RTSR;         /* 0x08: Rising trigger selection register    */
    __IO uint32_t FTSR;         /* 0x0C: Falling trigger selection register   */
    __IO uint32_t SWIER;        /* 0x10: Software interrupt event register    */
    __IO uint32_t PR;           /* 0x14: Pending register                     */
} EXTI_TypeDef;

typedef struct {
    __IO uint32_t CR;           /* 0x00: Clock control register               */
    __IO uint32_t CFGR;         /* 0x04: Clock configuration register         */
    __IO uint32_t CIR;          /* 0x08: Clock interrupt register             */
    __IO uint32_t APB2RSTR;     /* 0x0C: APB2 peripheral reset register       */
    __IO uint32_t APB1RSTR;     /* 0x10: APB1 peripheral reset register       */
    __IO uint32_t AHBENR;       /* 0x14: AHB peripheral clock enable register */
    __IO uint32_t APB2ENR;      /* 0x18: APB2 peripheral clock enable register*/
    __IO uint32_t APB1ENR;      /* 0x1C: APB1 peripheral clock enable register*/
    __IO uint32_t BDCR;         /* 0x20: Backup domain control register       */
    __IO uint32_t CSR;          /* 0x24: Control/status register              */
} RCC_TypeDef;

typedef struct {
    __IO uint32_t ACR;          /* 0x00: Flash access control register        */
    __IO uint32_t KEYR;         /* 0x04: FPEC key register                    */
    __IO uint32_t OPTKEYR;      /* 0x08: Flash OPTKEY register                */
    __IO uint32_t SR;           /* 0x0C: Flash status register                */
    __IO uint32_t CR;           /* 0x10: Flash control register               */
    __IO uint32_t AR;           /* 0x14: Flash address register               */
    __IO uint32_t RESERVED;     /* 0x18 */
    __IO uint32_t OBR;          /* 0x1C: Option byte register                 */
    __IO uint32_t WRPR;         /* 0x20: Write protection register            */
} FLASH_TypeDef;

typedef struct {
    __IO uint16_t CR1;          /* 0x00: Control register 1                   */
    uint16_t      RESERVED0;
    __IO uint16_t CR2;          /* 0x04: Control register 2                   */
    uint16_t      RESERVED1;
    __IO uint16_t SMCR;         /* 0x08: Slave mode control register          */
    uint16_t      RESERVED2;
    __IO uint16_t DIER;         /* 0x0C: DMA/interrupt enable register        */
    uint16_t      RESERVED3;
    __IO uint16_t SR;           /* 0x10: Status register                      */
    uint16_t      RESERVED4;
    __IO uint16_t EGR;          /* 0x14: Event generation register            */
    uint16_t      RESERVED5;
    __IO uint16_t CCMR1;        /* 0x18: Capture/compare mode register 1      */
    uint16_t      RESERVED6;
    __IO uint16_t CCMR2;        /* 0x1C: Capture/compare mode register 2      */
    uint16_t      RESERVED7;
    __IO uint16_t CCER;         /* 0x20: Capture/compare enable register      */
    uint16_t      RESERVED8;
    __IO uint16_t CNT;          /* 0x24: Counter                              */
    uint16_t      RESERVED9;
    __IO uint16_t PSC;          /* 0x28: Prescaler                            */
    uint16_t      RESERVED10;
    __IO uint16_t ARR;          /* 0x2C: Auto-reload register                 */
    uint16_t      RESERVED11;
    __IO uint16_t RCR;          /* 0x30: Repetition counter register          */
    uint16_t      RESERVED12;
    __IO uint16_t CCR1;         /* 0x34: Capture/compare register 1           */
    uint16_t      RESERVED13;
    __IO uint16_t CCR2;         /* 0x38: Capture/compare register 2           */
    uint16_t      RESERVED14;
    __IO uint16_t CCR3;         /* 0x3C: Capture/compare register 3           */
    uint16_t      RESERVED15;
    __IO uint16_t CCR4;         /* 0x40: Capture/compare register 4           */
    uint16_t      RESERVED16;
    __IO uint16_t BDTR;         /* 0x44: Break and dead-time register         */
    uint16_t      RESERVED17;
    __IO uint16_t DCR;          /* 0x48: DMA control register                 */
    uint16_t      RESERVED18;
    __IO uint16_t DMAR;         /* 0x4C: DMA address for full transfer        */
    uint16_t      RESERVED19;
} TIM_TypeDef;

typedef struct {
    __IO uint16_t SR;           /* 0x00: Status register                      */
    uint16_t      RESERVED0;
    __IO uint16_t DR;           /* 0x04: Data register                        */
    uint16_t      RESERVED1;
    __IO uint16_t BRR;          /* 0x08: Baud rate register                   */
    uint16_t      RESERVED2;
    __IO uint16_t CR1;          /* 0x0C: Control register 1                   */
    uint16_t      RESERVED3;
    __IO uint16_t CR2;          /* 0x10: Control register 2                   */
    uint16_t      RESERVED4;
    __IO uint16_t CR3;          /* 0x14: Control register 3                   */
    uint16_t      RESERVED5;
    __IO uint16_t GTPR;         /* 0x18: Guard time and prescaler register    */
    uint16_t      RESERVED6;
} USART_TypeDef;

typedef struct {
    __IO uint16_t CR1;          /* 0x00: SPI control register 1               */
    uint16_t      RESERVED0;
    __IO uint16_t CR2;          /* 0x04: SPI control register 2               */
    uint16_t      RESERVED1;
    __IO uint16_t SR;           /* 0x08: SPI status register                  */
    uint16_t      RESERVED2;
    __IO uint16_t DR;           /* 0x0C: SPI data register                    */
    uint16_t      RESERVED3;
    __IO uint16_t CRCPR;        /* 0x10: SPI CRC polynomial register          */
    uint16_t      RESERVED4;
    __IO uint16_t RXCRCR;       /* 0x14: SPI RX CRC register                  */
    uint16_t      RESERVED5;
    __IO uint16_t TXCRCR;       /* 0x18: SPI TX CRC register                  */
    uint16_t      RESERVED6;
    __IO uint16_t I2SCFGR;      /* 0x1C: I2S configuration register           */
    uint16_t      RESERVED7;
    __IO uint16_t I2SPR;        /* 0x20: I2S prescaler register               */
    uint16_t      RESERVED8;
} SPI_TypeDef;

typedef struct {
    __IO uint32_t KR;           /* 0x00: Key register (0x5555, 0xAAAA, 0xCCCC)*/
    __IO uint32_t PR;           /* 0x04: Prescaler register                   */
    __IO uint32_t RLR;          /* 0x08: Reload register                      */
    __IO uint32_t SR;           /* 0x0C: Status register                      */
} IWDG_TypeDef;

typedef struct {
    __IO uint16_t CRH;          /* 0x00: RTC control register high            */
    uint16_t      RESERVED0;
    __IO uint16_t CRL;          /* 0x04: RTC control register low             */
    uint16_t      RESERVED1;
    __IO uint16_t PRLH;         /* 0x08: RTC prescaler load register high     */
    uint16_t      RESERVED2;
    __IO uint16_t PRLL;         /* 0x0C: RTC prescaler load register low      */
    uint16_t      RESERVED3;
    __IO uint16_t DIVH;         /* 0x10: RTC prescaler divider register high  */
    uint16_t      RESERVED4;
    __IO uint16_t DIVL;         /* 0x14: RTC prescaler divider register low   */
    uint16_t      RESERVED5;
    __IO uint16_t CNTH;         /* 0x18: RTC counter register high            */
    uint16_t      RESERVED6;
    __IO uint16_t CNTL;         /* 0x1C: RTC counter register low             */
    uint16_t      RESERVED7;
    __IO uint16_t ALRH;         /* 0x20: RTC alarm register high              */
    uint16_t      RESERVED8;
    __IO uint16_t ALRL;         /* 0x24: RTC alarm register low               */
    uint16_t      RESERVED9;
} RTC_TypeDef;

typedef struct {
    uint32_t      RESERVED0;    /* 0x00 */
    __IO uint16_t DR1;          /* 0x04: Backup data register 1 (RTC magic 0xA5A5) */
    uint16_t      RESERVED1;
    __IO uint16_t DR2;          /* 0x08: Backup data register 2               */
    uint16_t      RESERVED2;
    __IO uint16_t DR3;          /* 0x0C: Backup data register 3               */
    uint16_t      RESERVED3;
    __IO uint16_t DR4;          /* 0x10: Backup data register 4               */
    uint16_t      RESERVED4;
    __IO uint16_t DR5;          /* 0x14: Backup data register 5               */
    uint16_t      RESERVED5;
    __IO uint16_t DR6;          /* 0x18: Backup data register 6               */
    uint16_t      RESERVED6;
    __IO uint16_t DR7;          /* 0x1C: Backup data register 7               */
    uint16_t      RESERVED7;
    __IO uint16_t DR8;          /* 0x20: Backup data register 8               */
    uint16_t      RESERVED8;
    __IO uint16_t DR9;          /* 0x24: Backup data register 9               */
    uint16_t      RESERVED9;
    __IO uint16_t DR10;         /* 0x28: Backup data register 10              */
    uint16_t      RESERVED10;
    __IO uint16_t RTCCR;        /* 0x2C: RTC clock calibration register       */
    uint16_t      RESERVED11;
    __IO uint16_t CR;           /* 0x30: Backup control register              */
    uint16_t      RESERVED12;
    __IO uint16_t CSR;          /* 0x34: Backup control/status register       */
    uint16_t      RESERVED13;
} BKP_TypeDef;

typedef struct {
    __IO uint32_t CR;           /* 0x00: Power control register               */
    __IO uint32_t CSR;          /* 0x04: Power control/status register        */
} PWR_TypeDef;

typedef struct {
    __IO uint32_t MCR;          /* 0x00: CAN master control register          */
    __IO uint32_t MSR;          /* 0x04: CAN master status register           */
    __IO uint32_t TSR;          /* 0x08: CAN transmit status register         */
    __IO uint32_t RF0R;         /* 0x0C: CAN receive FIFO 0 register          */
    __IO uint32_t RF1R;         /* 0x10: CAN receive FIFO 1 register          */
    __IO uint32_t IER;          /* 0x14: CAN interrupt enable register        */
    __IO uint32_t ESR;          /* 0x18: CAN error status register            */
    __IO uint32_t BTR;          /* 0x1C: CAN bit timing register              */
} CAN_TypeDef;

typedef struct {
    __IO uint32_t CTRL;         /* 0x00: SysTick control and status register  */
    __IO uint32_t LOAD;         /* 0x04: SysTick reload value register        */
    __IO uint32_t VAL;          /* 0x08: SysTick current value register       */
    __I  uint32_t CALIB;        /* 0x0C: SysTick calibration value register   */
} SysTick_TypeDef;

/* --- Peripheral Pointer Instances --- */
#define TIM2                    ((TIM_TypeDef *)     TIM2_BASE)
#define TIM3                    ((TIM_TypeDef *)     TIM3_BASE)
#define TIM4                    ((TIM_TypeDef *)     TIM4_BASE)
#define TIM5                    ((TIM_TypeDef *)     TIM5_BASE)
#define TIM6                    ((TIM_TypeDef *)     TIM6_BASE)
#define TIM7                    ((TIM_TypeDef *)     TIM7_BASE)
#define RTC                     ((RTC_TypeDef *)     RTC_BASE)
#define IWDG                    ((IWDG_TypeDef *)    IWDG_BASE)
#define SPI2                    ((SPI_TypeDef *)     SPI2_BASE)
#define USART2                  ((USART_TypeDef *)   USART2_BASE)
#define USART3                  ((USART_TypeDef *)   USART3_BASE)
#define CAN1                    ((CAN_TypeDef *)     CAN1_BASE)
#define BKP                     ((BKP_TypeDef *)     BKP_BASE)
#define PWR                     ((PWR_TypeDef *)     PWR_BASE)
#define AFIO                    ((AFIO_TypeDef *)    AFIO_BASE)
#define EXTI                    ((EXTI_TypeDef *)    EXTI_BASE)
#define GPIOA                   ((GPIO_TypeDef *)    GPIOA_BASE)
#define GPIOB                   ((GPIO_TypeDef *)    GPIOB_BASE)
#define GPIOC                   ((GPIO_TypeDef *)    GPIOC_BASE)
#define GPIOD                   ((GPIO_TypeDef *)    GPIOD_BASE)
#define GPIOE                   ((GPIO_TypeDef *)    GPIOE_BASE)
#define GPIOF                   ((GPIO_TypeDef *)    GPIOF_BASE)
#define GPIOG                   ((GPIO_TypeDef *)    GPIOG_BASE)
#define TIM1                    ((TIM_TypeDef *)     TIM1_BASE)
#define SPI1                    ((SPI_TypeDef *)     SPI1_BASE)
#define TIM8                    ((TIM_TypeDef *)     TIM8_BASE)
#define USART1                  ((USART_TypeDef *)   USART1_BASE)
#define RCC                     ((RCC_TypeDef *)     RCC_BASE)
#define FLASH                   ((FLASH_TypeDef *)   FLASH_R_BASE)
#define SysTick                 ((SysTick_TypeDef *) SYSTICK_BASE)

/* --- Notable Bit-Band & Register Aliases Used Directly by main() --- */
/* GPIOA..GPIOE CRL/CRH/ODR/IDR raw register macros */
#define GPIOA_CRL               REG32(GPIOA_BASE + 0x00)
#define GPIOA_CRH               REG32(GPIOA_BASE + 0x04)
#define GPIOA_IDR               REG32(GPIOA_BASE + 0x08)
#define GPIOA_ODR               REG32(GPIOA_BASE + 0x0C)
#define GPIOB_CRL               REG32(GPIOB_BASE + 0x00)
#define GPIOB_CRH               REG32(GPIOB_BASE + 0x04)
#define GPIOB_IDR               REG32(GPIOB_BASE + 0x08)
#define GPIOB_ODR               REG32(GPIOB_BASE + 0x0C)
#define GPIOC_CRL               REG32(GPIOC_BASE + 0x00)
#define GPIOC_CRH               REG32(GPIOC_BASE + 0x04)
#define GPIOC_IDR               REG32(GPIOC_BASE + 0x08)
#define GPIOC_ODR               REG32(GPIOC_BASE + 0x0C)
#define GPIOD_CRL               REG32(GPIOD_BASE + 0x00)
#define GPIOD_CRH               REG32(GPIOD_BASE + 0x04)
#define GPIOD_IDR               REG32(GPIOD_BASE + 0x08)
#define GPIOD_ODR               REG32(GPIOD_BASE + 0x0C)
#define GPIOE_CRL               REG32(GPIOE_BASE + 0x00)
#define GPIOE_CRH               REG32(GPIOE_BASE + 0x04)
#define GPIOE_IDR               REG32(GPIOE_BASE + 0x08)
#define GPIOE_ODR               REG32(GPIOE_BASE + 0x0C)

/* Bit-band GPIO pins accessed in main() & ISRs */
#define BB_PA8_OUT              REG32(0x422101A0UL) /* PERIPH_BB(GPIOA_ODR, 8): USART1 RS485 DE/RE */
#define BB_PA12_OUT             REG32(0x422101B0UL) /* PERIPH_BB(GPIOA_ODR, 12): Status / Comm LED */
#define BB_PD4_OUT              REG32(0x42228190UL) /* PERIPH_BB(GPIOD_ODR, 4): RUN LED (active low)*/
#define BB_PD5_OUT              REG32(0x42228194UL) /* PERIPH_BB(GPIOD_ODR, 5): STOP/DIAG LED       */
#define BB_PD7_OUT              REG32(0x4222819CUL) /* PERIPH_BB(GPIOD_ODR, 7): USART2 RS485 DE/RE */
#define BB_PE4_IN               REG32(0x42230110UL) /* PERIPH_BB(GPIOE_IDR, 4): RUN/STOP switch in */
#define BB_PE6_IN               REG32(0x42230118UL) /* PERIPH_BB(GPIOE_IDR, 6): Power-fail sense in*/
#define BB_PE7_OUT              REG32(0x4223019CUL) /* PERIPH_BB(GPIOE_ODR, 7): SF/DIAG red LED    */
#define BB_PWR_CR_DBP           REG32(0x420E0020UL) /* PERIPH_BB(PWR_CR, 8): Disable Backup Write Protection */
#define BB_RCC_BDCR_BDRST       REG32(0x42420440UL) /* PERIPH_BB(RCC_BDCR, 16): Backup domain software reset */

/* RCC Flag constant used in Reset Cause Check */
#define RCC_FLAG_IWDGRST        0x7DU               /* RCC->CSR bit 29 (Independent Watchdog Reset) */

/* ============================================================================
 * 3. FLASH MEMORY LAYOUT CONSTANTS (0x08000000 - 0x0807FFFF) & UID
 * ============================================================================ */

#define FLASH_BASE_ADDR             0x08000000UL    /* Vector table & firmware code start            */
#define FLASH_VECTOR_TABLE_ADDR     0x08000000UL    /* 76-entry Cortex-M3 vector table (0x130 bytes) */
#define FLASH_REGION_TABLE_ADDR     0x0804A960UL    /* Keil Region$$Table (2 entries, 32 bytes)      */
#define FLASH_UID_LICENSE_BASE      0x08052000UL    /* Factory UID / license verification page       */
#define FLASH_UID_HASH_C1_ADDR      0x08052010UL    /* Stored UID hash constant 1                    */
#define FLASH_UID_HASH_C2_ADDR      0x08052030UL    /* Stored UID hash constant 2                    */
#define FLASH_UID_HASH_C3_ADDR      0x08052058UL    /* Stored UID hash constant 3                    */
#define FLASH_DB1_BASE              0x08064800UL    /* User Data Block DB1 ("pp", type 0x13 0x0A)    */
#define FLASH_RETENTIVE_CFG_BASE    0x08068000UL    /* PLC retentive configuration sector            */
#define FLASH_RETENTIVE_FLAG_ADDR   0x08068524UL    /* Retentive status word (0x5555 = valid)        */
#define FLASH_RETENTIVE_RAM_BASE    0x08069000UL    /* Retentive V/M/C/T RAM backup page 0           */
#define FLASH_RETENTIVE_T_C_BASE    0x0806B000UL    /* Retentive T/C & S7 flags backup page 1        */
#define FLASH_OB1_EXEC_BASE         0x0806D800UL    /* Compiled/Executable User Program OB1          */
#define FLASH_OB1_RAW_BASE          0x08075800UL    /* Raw STEP 7-Micro/WIN OB1 block ("pp", 0x1308) */
#define FLASH_SCATTER_DATA_BASE     0x0807D800UL    /* Keil __decompress1 compressed .data stream    */
#define FLASH_SCATTER_ZERO_BASE     0x0807D868UL    /* Keil __scatterload_zeroinit descriptor        */
#define FLASH_SDB_BASE              0x0807F800UL    /* System Data Block SDB ("pp", type 0x13 0x0B)  */

/* STM32F103 / APM32E103 96-bit Unique Device ID (System Memory) */
#define STM32_UID_BASE              0x1FFFF7E8UL
#define STM32_UID_WORD0             REG32(0x1FFFF7E8UL)
#define STM32_UID_WORD1             REG32(0x1FFFF7ECUL)
#define STM32_UID_WORD2             REG32(0x1FFFF7F0UL)

/* ============================================================================
 * 4. PLC MEMORY AREA DESCRIPTOR TABLE (0x20000250 - 0x2000027B)
 * ============================================================================
 * Each 32-bit entry encodes:
 *   - Upper byte [31:24]: Area Type ID (or Flash bank offset for OB1)
 *   - Lower 24 bits [23:0]: Offset in SRAM (0x20000000 + offset) or Flash
 */
#define PLC_AREA_DESC_V_MEM         0x1800108CUL    /* Area 0x18: V memory (DB1) at 0x2000108C       */
#define PLC_AREA_DESC_T_CUR         0x1E001000UL    /* Area 0x1E: T status/flags at 0x20001000       */
#define PLC_AREA_DESC_C_CUR         0x1D001024UL    /* Area 0x1D: C status/flags at 0x20001024       */
#define PLC_AREA_DESC_HC_CUR        0x1F001046UL    /* Area 0x1F: HC current values at 0x20001046    */
#define PLC_AREA_DESC_SM_MEM        0x20005BD0UL    /* Area 0x20: I/Q/SM base at 0x20005BD0          */
#define PLC_AREA_DESC_S_MEM         0x14005E1AUL    /* Area 0x14: S (SCR) memory at 0x20005E1A       */
#define PLC_AREA_DESC_AI_AQ         0x13005E98UL    /* Area 0x13: AI/AQ/SM image at 0x20005E98       */
#define PLC_AREA_DESC_M_MEM         0x1C005F28UL    /* Area 0x1C: M memory at 0x20005F28             */
#define PLC_AREA_DESC_T_STATE       0x17004000UL    /* Area 0x17: Timer current words at 0x20004000  */
#define PLC_AREA_DESC_C_STATE       0x16004440UL    /* Area 0x16: Counter current words at 0x20004440*/
#define PLC_AREA_DESC_OB1_FLASH     0x0306D800UL    /* Area 0x03: Compiled OB1 at Flash 0x0806D800   */

/* ============================================================================
 * 5. COMPLETE SRAM VARIABLE MAP (0x20000000 - 0x2000CE38)
 * ============================================================================
 * Regions:
 *   0x20000000 - 0x200007F3 : Initialized .data section (2,036 bytes, decompressed)
 *   0x200007F4 - 0x2000CE37 : Zero-initialized .bss & stack (50,756 bytes)
 *   0x2000CE38              : Top of Main Stack (__initial_sp)
 */

/* --- 5.1 Initialized .data Section (0x20000000 - 0x200007F3) --- */
#define g_adc_presc_table           PTR8(0x20000000UL)  /* uint8_t[4] = {2, 4, 6, 8}                 */
#define g_apb_ahb_presc_table       PTR8(0x20000004UL)  /* uint8_t[16] SPL RCC prescaler table       */
#define g_system_core_clock         REG32(0x20000014UL) /* SystemCoreClock = 96000000 (96 MHz)       */
#define g_ahb_presc_table           PTR8(0x2000001CUL)  /* uint8_t[16] SPL SystemInit AHB table      */

/* Port 0 (USART1) / Port 1 (USART2) / Port 2 (USART3) TX state & framing flags */
#define g_port0_tx_busy             REG8(0x200000EEUL)  /* Port 0 RS485 transmit in progress         */
#define g_port0_tx_done_flag        REG8(0x200000F2UL)  /* Port 0 RS485 transmit complete flag       */
#define g_port1_tx_busy             REG8(0x200000F8UL)  /* Port 1 RS485 transmit in progress         */
#define g_port1_tx_done_flag        REG8(0x200000FCUL)  /* Port 1 RS485 transmit complete flag       */
#define g_port2_tx_done_flag        REG8(0x20000100UL)  /* Port 2 (USART3) transmit complete flag    */
#define g_default_ob1_template      PTR8(0x20000100UL)  /* 56-byte default OB1 header in .data       */

/* PLC Scan Cycle, Timer, and Mode Control State (0x20000160 - 0x2000024F) */
#define g_rtc_read_Fail_cnt         REG8(0x2000016CUL)  /* Consecutive RTC validation errors         */
#define g_plc_run_mode              REG8(0x2000016FUL)  /* PLC operating mode: 0=STOP, 1=RUN         */
#define g_prev_plc_run_mode         REG8(0x20000170UL)  /* Previous scan operating mode              */
#define g_mode_switch_debounce      REG8(0x20000171UL)  /* RUN/STOP switch debounce counter          */
#define g_force_active_flag         REG8(0x20000172UL)  /* I/O or variable force active flag         */
#define g_first_scan_done_flag      REG8(0x20000173UL)  /* 0 on first scan after STOP->RUN, then 1   */
#define g_comm_led_Hold_cnt         REG8(0x20000174UL)  /* Comm activity LED hold counter            */
#define g_power_loss_debounce       REG8(0x20000175UL)  /* PE6 power-loss input debounce counter     */
#define g_warm_reset_gate           REG8(0x20000176UL)  /* Warm reset gate flag                      */
#define g_rtc_update_req            REG8(0x20000177UL)  /* Request to write new TOD to RTC           */
#define g_retentive_save_done       REG8(0x20000178UL)  /* Power-down retentive flash save completed */
#define g_rtc_init_needed           REG8(0x2000017CUL)  /* 1 if RTC needs cold initialization        */
#define g_tod_bcd_buf               PTR8(0x2000017DUL)  /* 8-byte BCD TOD buffer (YY MM DD HH MM SS) */
#define g_tod_bcd_year              REG8(0x2000017DUL)  /* BCD Year (00..99, default 0x87)           */
#define g_tod_bcd_month             REG8(0x2000017EUL)  /* BCD Month (01..12, default 0x11)          */
#define g_tod_bcd_day               REG8(0x2000017FUL)  /* BCD Day (01..31, default 0x25)            */
#define g_tod_bcd_hour              REG8(0x20000180UL)  /* BCD Hour (00..23, default 0x15)           */
#define g_tod_bcd_minute            REG8(0x20000181UL)  /* BCD Minute (00..59, default 0x32)         */
#define g_tod_bcd_second            REG8(0x20000182UL)  /* BCD Second (00..59, default 0x21)         */
#define g_tod_bcd_weekday           REG8(0x20000184UL)  /* BCD Day of Week (1..7, default 0x03)      */
#define g_plc_error_code            REG8(0x20000188UL)  /* Fatal/non-fatal error code (5=SDB err)    */
#define g_comm_idle_counter         REG16(0x2000018AUL) /* Comm inactivity watchdog counter          */
#define g_scan_100us_accum          REG16(0x2000018CUL) /* Scan time 100us accumulator               */
#define g_scan_last_100us_snap      REG16(0x2000018EUL) /* Previous TIM2 100us counter snapshot      */
#define g_tod_100ms_last_snap       REG16(0x20000194UL) /* Snapshot of 1ms tick for 100ms RTC update */
#define g_led_blink_1ms_snap        REG16(0x20000196UL) /* Snapshot of 1ms tick for SF LED blink     */
#define g_retentive_save_holdoff    REG16(0x200001A2UL) /* Holdoff timer after retentive Flash write */
#define g_em_bus_Check_timer        REG16(0x200001A4UL) /* Expansion module bus check timer          */
#define g_em_config_change_cnt      REG16(0x200001A6UL) /* Expansion module config mismatch counter  */
#define g_systick_1ms_counter       REG16(0x200001ACUL) /* Free-running 1 ms counter (SysTick ISR)   */
#define g_timer_1ms_accum           REG16(0x200001B0UL) /* 1 ms accumulator for S7 timers            */
#define g_timer_1ms_last_snap       REG16(0x200001B2UL) /* Last 1ms tick snapshot for timer update   */
#define g_timer_10ms_accum          REG16(0x200001B4UL) /* 10 ms accumulator for S7 10ms/100ms timers*/
#define g_timer_10ms_last_snap      REG16(0x200001B6UL) /* Last 1ms tick snapshot for 10ms timers    */
#define g_sm0_4_30s_counter         REG16(0x200001B8UL) /* 30,000 ms half-period counter for SM0.4   */
#define g_sm0_4_last_snap           REG16(0x200001BAUL) /* Last 1ms snapshot for SM0.4 (1 min clock) */
#define g_sm0_5_500ms_counter       REG16(0x200001BCUL) /* 500 ms half-period counter for SM0.5      */
#define g_sm0_5_last_snap           REG16(0x200001BEUL) /* Last 1ms snapshot for SM0.5 (1 sec clock) */
#define g_warm_reset_flag           REG32(0x200001D0UL) /* 0=Cold power-on boot, non-zero=IWDG warm  */
#define g_sdb_valid_flag            REG8(0x200001E0UL)  /* 1 if SDB magic/header is valid            */
#define g_ob1_valid_flag            REG8(0x200001ECUL)  /* 1 if OB1 user program in Flash is valid   */
#define g_user_prog_Executing       REG8(0x200001EEUL)  /* 1 while VM is executing user program      */
#define g_compile_err_flag          REG8(0x200001F4UL)  /* 1 if OB1 compilation failed (error 0x82)  */
#define g_tim2_100us_counter        REG16(0x20000224UL) /* Free-running 100 us counter (TIM2 ISR)    */
#define g_timed_int0_interval       REG8(0x20000226UL)  /* Cached SMB34 timed interrupt 0 interval   */
#define g_timed_int1_interval       REG8(0x20000227UL)  /* Cached SMB35 timed interrupt 1 interval   */
#define g_plc_mem_area_table        PTR32(0x20000250UL) /* 11-entry PLC Memory Area Descriptor Table */

/* Interrupt Queue & High-Speed Counter / PTO Control (0x20000370 - 0x200007F3) */
#define g_int_enable_global         REG8(0x20000378UL)  /* PLC user interrupt global enable (ENI/DISI)*/
#define g_rtc_days_before_month     PTR16(0x200004FCUL) /* uint16_t[13] cumulative days per month    */

/* --- 5.2 Zero-Initialized .bss Section & PLC Memory Areas (0x200007F4 - 0x2000CE37) --- */
#define g_plc_T_status_base         PTR8(0x20001000UL)  /* Area 0x1E: Timer status bits (T0..T255)   */
#define g_plc_C_status_base         PTR8(0x20001024UL)  /* Area 0x1D: Counter status bits (C0..C255) */
#define g_plc_HC_value_base         PTR32(0x20001046UL) /* Area 0x1F: High-Speed Counters HC0..HC5   */
#define g_plc_db1_header_ram        PTR8(0x20001068UL)  /* 0x24-byte DB1 header copied from Flash    */
#define g_plc_V_mem_base            PTR8(0x2000108CUL)  /* Area 0x18: Variable Memory VB0..VB10239   */
#define g_plc_diag_info_base        PTR8(0x20003890UL)  /* PLC internal diagnostics / UID hash block */
#define g_plc_T_current_base        PTR16(0x20004000UL) /* Area 0x17: Timer current words (T0..T255) */
#define g_plc_C_current_base        PTR16(0x20004440UL) /* Area 0x16: Counter current words (C0..C255)*/
#define g_plc_author_sig_ram        PTR8(0x200046ACUL)  /* 32-byte developer signature in SRAM       */
#define g_plc_uid_word0             REG32(0x2000511CUL) /* Cached MCU UID[31:0]                      */
#define g_plc_uid_word1             REG32(0x20005120UL) /* Cached MCU UID[63:32]                     */
#define g_plc_uid_word2             REG32(0x20005124UL) /* Cached MCU UID[95:64]                     */
#define g_plc_uid_checksum          REG32(0x20005128UL) /* Computed UID checksum word                */
#define g_plc_sdb_ram_buf           PTR8(0x200058CCUL)  /* 0x180-byte System Data Block (SDB) buffer */
#define g_plc_I_image_base          PTR8(0x20005BD0UL)  /* Process-Image Input Register IB0..IB15    */
#define g_plc_Q_image_base          PTR8(0x20005BE0UL)  /* Process-Image Output Register QB0..QB15   */
#define g_plc_AI_image_base         PTR8(0x20005BF0UL)  /* Analog Input Image AIW0..AIW62            */
#define g_plc_AQ_image_base         PTR8(0x20005C30UL)  /* Analog Output Image AQW0..AQW62           */
#define g_plc_S_mem_base            PTR8(0x20005E1AUL)  /* Area 0x14: Sequential Control Relays S0.. */
#define g_plc_SM_mem_base           PTR8(0x20005F28UL)  /* Special Memory SMB0..SMB549 (M/SM region) */

/* Top of Stack */
#define INITIAL_MSP_ADDR            0x2000CE38UL

/* ============================================================================
 * 5.3 GHIDRA SYMBOL ALIASES & HELPER MACROS (DAT_xxxxxxxx / CONCAT44)
 * ============================================================================ */


/* Cortex-M3 GPIO ODR Bit-Band Alias Symbols (uRam422xxxxx) */
#define uRam4221019c            REG32(0x4221019CUL) /* PERIPH_BB(GPIOA_ODR, 7)  */
#define uRam422101a0            REG32(0x422101A0UL) /* PERIPH_BB(GPIOA_ODR, 8)  */
#define uRam422101ac            REG32(0x422101ACUL) /* PERIPH_BB(GPIOA_ODR, 11) */
#define uRam422101b0            REG32(0x422101B0UL) /* PERIPH_BB(GPIOA_ODR, 12) */
#define uRam422101bc            REG32(0x422101BCUL) /* PERIPH_BB(GPIOA_ODR, 15) */
#define uRam42220180            REG32(0x42220180UL) /* PERIPH_BB(GPIOC_ODR, 0)  */
#define uRam42220184            REG32(0x42220184UL) /* PERIPH_BB(GPIOC_ODR, 1)  */
#define uRam422201b0            REG32(0x422201B0UL) /* PERIPH_BB(GPIOC_ODR, 12) */
#define uRam422201b4            REG32(0x422201B4UL) /* PERIPH_BB(GPIOC_ODR, 13) */
#define uRam4223018c            REG32(0x4223018CUL) /* PERIPH_BB(GPIOE_ODR, 3)  */


/* Cortex-M3 Core Intrinsics (CPSID I / DSB) */
static inline void disableIRQinterrupts(void) {
#if defined(__ARM_ARCH) || defined(__arm__) || defined(__thumb__)
    __asm volatile ("cpsid i" : : : "memory");
#endif
}
static inline void DataSynchronizationBarrier(uint32_t opt) {
    (void)opt;
#if defined(__ARM_ARCH) || defined(__arm__) || defined(__thumb__)
    __asm volatile ("dsb 0xF" : : : "memory");
#endif
}

#ifndef CONCAT44
#define CONCAT44(hi, lo) ((((uint64_t)(uint32_t)(hi)) << 32) | ((uint64_t)(uint32_t)(lo)))
#endif
typedef uint64_t ulonglong;
typedef int64_t  longlong;

#ifndef DAT_0804a2a2
#define DAT_0804a2a2             REG8(0x0804A2A2UL)
#endif
#ifndef DAT_08064800
#define DAT_08064800             REG8(0x08064800UL)
#endif
#ifndef DAT_0807f800
#define DAT_0807f800             REG8(0x0807F800UL)
#endif
#ifndef DAT_1ffff7e8
#define DAT_1ffff7e8             REG32(0x1FFFF7E8UL)
#endif
#ifndef DAT_1ffff7ec
#define DAT_1ffff7ec             REG32(0x1FFFF7ECUL)
#endif
#ifndef DAT_1ffff7f0
#define DAT_1ffff7f0             REG32(0x1FFFF7F0UL)
#endif
#ifndef DAT_20000039
#define DAT_20000039             REG8(0x20000039UL)
#endif
#ifndef DAT_2000003c
#define DAT_2000003c             REG8(0x2000003CUL)
#endif
#ifndef DAT_2000003d
#define DAT_2000003d             REG8(0x2000003DUL)
#endif
#ifndef DAT_2000003e
#define DAT_2000003e             REG8(0x2000003EUL)
#endif
#ifndef DAT_2000003f
#define DAT_2000003f             REG8(0x2000003FUL)
#endif
#ifndef DAT_20000040
#define DAT_20000040             REG8(0x20000040UL)
#endif
#ifndef DAT_20000041
#define DAT_20000041             REG8(0x20000041UL)
#endif
#ifndef DAT_20000042
#define DAT_20000042             REG8(0x20000042UL)
#endif
#ifndef DAT_20000043
#define DAT_20000043             REG8(0x20000043UL)
#endif
#ifndef DAT_20000084
#define DAT_20000084             REG8(0x20000084UL)
#endif
#ifndef DAT_20000088
#define DAT_20000088             REG8(0x20000088UL)
#endif
#ifndef DAT_20000090
#define DAT_20000090             REG8(0x20000090UL)
#endif
#ifndef DAT_20000098
#define DAT_20000098             REG8(0x20000098UL)
#endif
#ifndef DAT_2000009c
#define DAT_2000009c             REG8(0x2000009CUL)
#endif
#ifndef DAT_200000a0
#define DAT_200000a0             REG8(0x200000A0UL)
#endif
#ifndef DAT_200000a8
#define DAT_200000a8             REG8(0x200000A8UL)
#endif
#ifndef DAT_200000b0
#define DAT_200000b0             REG8(0x200000B0UL)
#endif
#ifndef DAT_200000bc
#define DAT_200000bc             (*(uint8_t * volatile *)(uintptr_t)0x200000BCUL)
#endif
#ifndef DAT_200000c0
#define DAT_200000c0             REG8(0x200000C0UL)
#endif
#ifndef DAT_200000c4
#define DAT_200000c4             (*(uint8_t * volatile *)(uintptr_t)0x200000C4UL)
#endif
#ifndef DAT_200000c8
#define DAT_200000c8             REG8(0x200000C8UL)
#endif
#ifndef DAT_200000cc
#define DAT_200000cc             (*(uint8_t * volatile *)(uintptr_t)0x200000CCUL)
#endif
#ifndef DAT_200000d0
#define DAT_200000d0             REG8(0x200000D0UL)
#endif
#ifndef DAT_20000100
#define DAT_20000100             REG8(0x20000100UL)
#endif
#ifndef DAT_20000140
#define DAT_20000140             REG8(0x20000140UL)
#endif
#ifndef DAT_20000141
#define DAT_20000141             REG8(0x20000141UL)
#endif
#ifndef DAT_20000142
#define DAT_20000142             REG8(0x20000142UL)
#endif
#ifndef DAT_20000143
#define DAT_20000143             REG8(0x20000143UL)
#endif
#ifndef DAT_20000144
#define DAT_20000144             REG8(0x20000144UL)
#endif
#ifndef DAT_20000145
#define DAT_20000145             REG8(0x20000145UL)
#endif
#ifndef DAT_20000146
#define DAT_20000146             REG8(0x20000146UL)
#endif
#ifndef DAT_20000147
#define DAT_20000147             REG8(0x20000147UL)
#endif
#ifndef DAT_20000148
#define DAT_20000148             REG8(0x20000148UL)
#endif
#ifndef DAT_20000149
#define DAT_20000149             REG8(0x20000149UL)
#endif
#ifndef DAT_2000014a
#define DAT_2000014a             REG8(0x2000014AUL)
#endif
#ifndef DAT_2000014b
#define DAT_2000014b             REG8(0x2000014BUL)
#endif
#ifndef DAT_2000014c
#define DAT_2000014c             REG8(0x2000014CUL)
#endif
#ifndef DAT_2000014d
#define DAT_2000014d             REG8(0x2000014DUL)
#endif
#ifndef DAT_2000014e
#define DAT_2000014e             REG8(0x2000014EUL)
#endif
#ifndef DAT_2000014f
#define DAT_2000014f             REG8(0x2000014FUL)
#endif
#ifndef DAT_20000150
#define DAT_20000150             REG8(0x20000150UL)
#endif
#ifndef DAT_20000151
#define DAT_20000151             REG8(0x20000151UL)
#endif
#ifndef DAT_20000152
#define DAT_20000152             REG8(0x20000152UL)
#endif
#ifndef DAT_20000153
#define DAT_20000153             REG8(0x20000153UL)
#endif
#ifndef DAT_20000154
#define DAT_20000154             REG8(0x20000154UL)
#endif
#ifndef DAT_20000155
#define DAT_20000155             REG8(0x20000155UL)
#endif
#ifndef DAT_20000156
#define DAT_20000156             REG8(0x20000156UL)
#endif
#ifndef DAT_20000157
#define DAT_20000157             REG8(0x20000157UL)
#endif
#ifndef DAT_20000158
#define DAT_20000158             REG8(0x20000158UL)
#endif
#ifndef DAT_20000159
#define DAT_20000159             REG8(0x20000159UL)
#endif
#ifndef DAT_2000015a
#define DAT_2000015a             REG8(0x2000015AUL)
#endif
#ifndef DAT_2000015b
#define DAT_2000015b             REG8(0x2000015BUL)
#endif
#ifndef DAT_2000015c
#define DAT_2000015c             REG8(0x2000015CUL)
#endif
#ifndef DAT_2000015d
#define DAT_2000015d             REG8(0x2000015DUL)
#endif
#ifndef DAT_2000015e
#define DAT_2000015e             REG8(0x2000015EUL)
#endif
#ifndef DAT_2000015f
#define DAT_2000015f             REG8(0x2000015FUL)
#endif
#ifndef DAT_20000160
#define DAT_20000160             REG8(0x20000160UL)
#endif
#ifndef DAT_20000161
#define DAT_20000161             REG8(0x20000161UL)
#endif
#ifndef DAT_20000162
#define DAT_20000162             REG8(0x20000162UL)
#endif
#ifndef DAT_20000163
#define DAT_20000163             REG8(0x20000163UL)
#endif
#ifndef DAT_20000164
#define DAT_20000164             REG8(0x20000164UL)
#endif
#ifndef DAT_20000165
#define DAT_20000165             REG8(0x20000165UL)
#endif
#ifndef DAT_20000166
#define DAT_20000166             REG8(0x20000166UL)
#endif
#ifndef DAT_20000167
#define DAT_20000167             REG8(0x20000167UL)
#endif
#ifndef DAT_20000168
#define DAT_20000168             REG8(0x20000168UL)
#endif
#ifndef DAT_20000169
#define DAT_20000169             REG8(0x20000169UL)
#endif
#ifndef DAT_2000016a
#define DAT_2000016a             REG8(0x2000016AUL)
#endif
#ifndef DAT_2000016b
#define DAT_2000016b             REG8(0x2000016BUL)
#endif
#ifndef DAT_2000016c
#define DAT_2000016c             REG8(0x2000016CUL)
#endif
#ifndef DAT_2000016d
#define DAT_2000016d             REG8(0x2000016DUL)
#endif
#ifndef DAT_2000016e
#define DAT_2000016e             REG8(0x2000016EUL)
#endif
#ifndef DAT_2000016f
#define DAT_2000016f             REG8(0x2000016FUL)
#endif
#ifndef DAT_20000170
#define DAT_20000170             REG8(0x20000170UL)
#endif
#ifndef DAT_20000171
#define DAT_20000171             REG8(0x20000171UL)
#endif
#ifndef DAT_20000172
#define DAT_20000172             REG8(0x20000172UL)
#endif
#ifndef DAT_20000173
#define DAT_20000173             REG8(0x20000173UL)
#endif
#ifndef DAT_20000174
#define DAT_20000174             REG8(0x20000174UL)
#endif
#ifndef DAT_20000175
#define DAT_20000175             REG8(0x20000175UL)
#endif
#ifndef DAT_20000176
#define DAT_20000176             REG8(0x20000176UL)
#endif
#ifndef DAT_20000177
#define DAT_20000177             REG8(0x20000177UL)
#endif
#ifndef DAT_20000178
#define DAT_20000178             REG8(0x20000178UL)
#endif
#ifndef DAT_20000179
#define DAT_20000179             REG8(0x20000179UL)
#endif
#ifndef DAT_2000017a
#define DAT_2000017a             REG8(0x2000017AUL)
#endif
#ifndef DAT_2000017b
#define DAT_2000017b             REG8(0x2000017BUL)
#endif
#ifndef DAT_2000017c
#define DAT_2000017c             REG8(0x2000017CUL)
#endif
#ifndef DAT_2000017d
#define DAT_2000017d             REG8(0x2000017DUL)
#endif
#ifndef DAT_2000017e
#define DAT_2000017e             REG8(0x2000017EUL)
#endif
#ifndef DAT_2000017f
#define DAT_2000017f             REG8(0x2000017FUL)
#endif
#ifndef DAT_20000180
#define DAT_20000180             REG8(0x20000180UL)
#endif
#ifndef DAT_20000181
#define DAT_20000181             REG8(0x20000181UL)
#endif
#ifndef DAT_20000182
#define DAT_20000182             REG8(0x20000182UL)
#endif
#ifndef DAT_20000183
#define DAT_20000183             REG8(0x20000183UL)
#endif
#ifndef DAT_20000184
#define DAT_20000184             REG8(0x20000184UL)
#endif
#ifndef DAT_20000185
#define DAT_20000185             REG8(0x20000185UL)
#endif
#ifndef DAT_20000186
#define DAT_20000186             REG8(0x20000186UL)
#endif
#ifndef DAT_20000187
#define DAT_20000187             REG8(0x20000187UL)
#endif
#ifndef DAT_20000188
#define DAT_20000188             REG8(0x20000188UL)
#endif
#ifndef DAT_20000189
#define DAT_20000189             REG8(0x20000189UL)
#endif
#ifndef DAT_2000018a
#define DAT_2000018a             REG16(0x2000018AUL)
#endif
#ifndef DAT_2000018b
#define DAT_2000018b             REG8(0x2000018BUL)
#endif
#ifndef DAT_2000018c
#define DAT_2000018c             REG16(0x2000018CUL)
#endif
#ifndef DAT_2000018d
#define DAT_2000018d             REG8(0x2000018DUL)
#endif
#ifndef DAT_2000018e
#define DAT_2000018e             REG16(0x2000018EUL)
#endif
#ifndef DAT_2000018f
#define DAT_2000018f             REG8(0x2000018FUL)
#endif
#ifndef DAT_20000190
#define DAT_20000190             REG8(0x20000190UL)
#endif
#ifndef DAT_20000191
#define DAT_20000191             REG8(0x20000191UL)
#endif
#ifndef DAT_20000192
#define DAT_20000192             REG8(0x20000192UL)
#endif
#ifndef DAT_20000193
#define DAT_20000193             REG8(0x20000193UL)
#endif
#ifndef DAT_20000194
#define DAT_20000194             REG16(0x20000194UL)
#endif
#ifndef DAT_20000195
#define DAT_20000195             REG8(0x20000195UL)
#endif
#ifndef DAT_20000196
#define DAT_20000196             REG16(0x20000196UL)
#endif
#ifndef DAT_20000197
#define DAT_20000197             REG8(0x20000197UL)
#endif
#ifndef DAT_20000198
#define DAT_20000198             REG8(0x20000198UL)
#endif
#ifndef DAT_20000199
#define DAT_20000199             REG8(0x20000199UL)
#endif
#ifndef DAT_2000019a
#define DAT_2000019a             REG8(0x2000019AUL)
#endif
#ifndef DAT_2000019b
#define DAT_2000019b             REG8(0x2000019BUL)
#endif
#ifndef DAT_2000019c
#define DAT_2000019c             REG8(0x2000019CUL)
#endif
#ifndef DAT_2000019d
#define DAT_2000019d             REG8(0x2000019DUL)
#endif
#ifndef DAT_2000019e
#define DAT_2000019e             REG8(0x2000019EUL)
#endif
#ifndef DAT_2000019f
#define DAT_2000019f             REG8(0x2000019FUL)
#endif
#ifndef DAT_200001a0
#define DAT_200001a0             REG8(0x200001A0UL)
#endif
#ifndef DAT_200001a2
#define DAT_200001a2             REG16(0x200001A2UL)
#endif
#ifndef DAT_200001a3
#define DAT_200001a3             REG8(0x200001A3UL)
#endif
#ifndef DAT_200001a4
#define DAT_200001a4             REG16(0x200001A4UL)
#endif
#ifndef DAT_200001a5
#define DAT_200001a5             REG8(0x200001A5UL)
#endif
#ifndef DAT_200001a6
#define DAT_200001a6             REG16(0x200001A6UL)
#endif
#ifndef DAT_200001a7
#define DAT_200001a7             REG8(0x200001A7UL)
#endif
#ifndef DAT_200001a8
#define DAT_200001a8             REG8(0x200001A8UL)
#endif
#ifndef DAT_200001a9
#define DAT_200001a9             REG8(0x200001A9UL)
#endif
#ifndef DAT_200001aa
#define DAT_200001aa             REG8(0x200001AAUL)
#endif
#ifndef DAT_200001ab
#define DAT_200001ab             REG8(0x200001ABUL)
#endif
#ifndef DAT_200001ac
#define DAT_200001ac             REG16(0x200001ACUL)
#endif
#ifndef DAT_200001af
#define DAT_200001af             REG8(0x200001AFUL)
#endif
#ifndef DAT_200001b0
#define DAT_200001b0             REG16(0x200001B0UL)
#endif
#ifndef DAT_200001b1
#define DAT_200001b1             REG8(0x200001B1UL)
#endif
#ifndef DAT_200001b2
#define DAT_200001b2             REG16(0x200001B2UL)
#endif
#ifndef DAT_200001b3
#define DAT_200001b3             REG8(0x200001B3UL)
#endif
#ifndef DAT_200001b4
#define DAT_200001b4             REG16(0x200001B4UL)
#endif
#ifndef DAT_200001b5
#define DAT_200001b5             REG8(0x200001B5UL)
#endif
#ifndef DAT_200001b6
#define DAT_200001b6             REG16(0x200001B6UL)
#endif
#ifndef DAT_200001b7
#define DAT_200001b7             REG8(0x200001B7UL)
#endif
#ifndef DAT_200001b8
#define DAT_200001b8             REG16(0x200001B8UL)
#endif
#ifndef DAT_200001b9
#define DAT_200001b9             REG8(0x200001B9UL)
#endif
#ifndef DAT_200001ba
#define DAT_200001ba             REG16(0x200001BAUL)
#endif
#ifndef DAT_200001bb
#define DAT_200001bb             REG8(0x200001BBUL)
#endif
#ifndef DAT_200001bc
#define DAT_200001bc             REG16(0x200001BCUL)
#endif
#ifndef DAT_200001bd
#define DAT_200001bd             REG8(0x200001BDUL)
#endif
#ifndef DAT_200001be
#define DAT_200001be             REG16(0x200001BEUL)
#endif
#ifndef DAT_200001bf
#define DAT_200001bf             REG8(0x200001BFUL)
#endif
#ifndef DAT_200001c0
#define DAT_200001c0             REG32(0x200001C0UL)
#endif
#ifndef DAT_200001c1
#define DAT_200001c1             REG8(0x200001C1UL)
#endif
#ifndef DAT_200001c2
#define DAT_200001c2             REG8(0x200001C2UL)
#endif
#ifndef DAT_200001c3
#define DAT_200001c3             REG8(0x200001C3UL)
#endif
#ifndef DAT_200001c4
#define DAT_200001c4             REG32(0x200001C4UL)
#endif
#ifndef DAT_200001c5
#define DAT_200001c5             REG8(0x200001C5UL)
#endif
#ifndef DAT_200001c6
#define DAT_200001c6             REG8(0x200001C6UL)
#endif
#ifndef DAT_200001c7
#define DAT_200001c7             REG8(0x200001C7UL)
#endif
#ifndef DAT_200001c8
#define DAT_200001c8             REG32(0x200001C8UL)
#endif
#ifndef DAT_200001c9
#define DAT_200001c9             REG8(0x200001C9UL)
#endif
#ifndef DAT_200001ca
#define DAT_200001ca             REG8(0x200001CAUL)
#endif
#ifndef DAT_200001cb
#define DAT_200001cb             REG8(0x200001CBUL)
#endif
#ifndef DAT_200001cc
#define DAT_200001cc             REG32(0x200001CCUL)
#endif
#ifndef DAT_200001cd
#define DAT_200001cd             REG8(0x200001CDUL)
#endif
#ifndef DAT_200001ce
#define DAT_200001ce             REG8(0x200001CEUL)
#endif
#ifndef DAT_200001cf
#define DAT_200001cf             REG8(0x200001CFUL)
#endif
#ifndef DAT_200001d0
#define DAT_200001d0             REG32(0x200001D0UL)
#endif
#ifndef DAT_200001d2
#define DAT_200001d2             REG16(0x200001D2UL)
#endif
#ifndef DAT_200001d4
#define DAT_200001d4             REG8(0x200001D4UL)
#endif
#ifndef DAT_200001d6
#define DAT_200001d6             REG8(0x200001D6UL)
#endif
#ifndef DAT_200001d8
#define DAT_200001d8             REG8(0x200001D8UL)
#endif
#ifndef DAT_200001d9
#define DAT_200001d9             REG8(0x200001D9UL)
#endif
#ifndef DAT_200001da
#define DAT_200001da             REG8(0x200001DAUL)
#endif
#ifndef DAT_200001db
#define DAT_200001db             REG8(0x200001DBUL)
#endif
#ifndef DAT_200001dc
#define DAT_200001dc             REG8(0x200001DCUL)
#endif
#ifndef DAT_200001dd
#define DAT_200001dd             REG8(0x200001DDUL)
#endif
#ifndef DAT_200001de
#define DAT_200001de             REG8(0x200001DEUL)
#endif
#ifndef DAT_200001e0
#define DAT_200001e0             REG8(0x200001E0UL)
#endif
#ifndef DAT_200001e2
#define DAT_200001e2             REG8(0x200001E2UL)
#endif
#ifndef DAT_200001e4
#define DAT_200001e4             REG16(0x200001E4UL)
#endif
#ifndef DAT_200001e6
#define DAT_200001e6             REG16(0x200001E6UL)
#endif
#ifndef DAT_200001e8
#define DAT_200001e8             REG8(0x200001E8UL)
#endif
#ifndef DAT_200001ea
#define DAT_200001ea             REG8(0x200001EAUL)
#endif
#ifndef DAT_200001ec
#define DAT_200001ec             REG8(0x200001ECUL)
#endif
#ifndef DAT_200001ee
#define DAT_200001ee             REG8(0x200001EEUL)
#endif
#ifndef DAT_200001f0
#define DAT_200001f0             REG8(0x200001F0UL)
#endif
#ifndef DAT_200001f2
#define DAT_200001f2             REG8(0x200001F2UL)
#endif
#ifndef DAT_200001f4
#define DAT_200001f4             REG8(0x200001F4UL)
#endif
#ifndef DAT_200001f6
#define DAT_200001f6             REG16(0x200001F6UL)
#endif
#ifndef DAT_200001f8
#define DAT_200001f8             REG8(0x200001F8UL)
#endif
#ifndef DAT_200001fa
#define DAT_200001fa             REG16(0x200001FAUL)
#endif
#ifndef DAT_200001fc
#define DAT_200001fc             REG8(0x200001FCUL)
#endif
#ifndef DAT_200001fe
#define DAT_200001fe             REG16(0x200001FEUL)
#endif
#ifndef DAT_20000200
#define DAT_20000200             REG16(0x20000200UL)
#endif
#ifndef DAT_20000204
#define DAT_20000204             REG16(0x20000204UL)
#endif
#ifndef DAT_20000206
#define DAT_20000206             REG8(0x20000206UL)
#endif
#ifndef DAT_20000208
#define DAT_20000208             REG8(0x20000208UL)
#endif
#ifndef DAT_2000020a
#define DAT_2000020a             REG16(0x2000020AUL)
#endif
#ifndef DAT_2000020c
#define DAT_2000020c             REG16(0x2000020CUL)
#endif
#ifndef DAT_20000210
#define DAT_20000210             REG16(0x20000210UL)
#endif
#ifndef DAT_20000214
#define DAT_20000214             REG16(0x20000214UL)
#endif
#ifndef DAT_20000218
#define DAT_20000218             REG8(0x20000218UL)
#endif
#ifndef DAT_2000021c
#define DAT_2000021c             REG8(0x2000021CUL)
#endif
#ifndef DAT_2000021d
#define DAT_2000021d             REG8(0x2000021DUL)
#endif
#ifndef DAT_2000021e
#define DAT_2000021e             REG8(0x2000021EUL)
#endif
#ifndef DAT_2000021f
#define DAT_2000021f             REG8(0x2000021FUL)
#endif
#ifndef DAT_20000220
#define DAT_20000220             REG32(0x20000220UL)
#endif
#ifndef DAT_20000224
#define DAT_20000224             REG16(0x20000224UL)
#endif
#ifndef DAT_20000228
#define DAT_20000228             REG8(0x20000228UL)
#endif
#ifndef DAT_2000022c
#define DAT_2000022c             REG8(0x2000022CUL)
#endif
#ifndef DAT_20000230
#define DAT_20000230             REG16(0x20000230UL)
#endif
#ifndef DAT_20000234
#define DAT_20000234             REG16(0x20000234UL)
#endif
#ifndef DAT_20000238
#define DAT_20000238             REG16(0x20000238UL)
#endif
#ifndef DAT_2000023c
#define DAT_2000023c             REG32(0x2000023CUL)
#endif
#ifndef DAT_20000240
#define DAT_20000240             REG32(0x20000240UL)
#endif
#ifndef DAT_20000244
#define DAT_20000244             REG32(0x20000244UL)
#endif
#ifndef DAT_20000248
#define DAT_20000248             REG32(0x20000248UL)
#endif
#ifndef DAT_2000024c
#define DAT_2000024c             REG32(0x2000024CUL)
#endif
#ifndef DAT_20000250
#define DAT_20000250             REG32(0x20000250UL)
#endif
#ifndef DAT_20000254
#define DAT_20000254             REG32(0x20000254UL)
#endif
#ifndef DAT_20000258
#define DAT_20000258             REG32(0x20000258UL)
#endif
#ifndef DAT_2000025c
#define DAT_2000025c             REG32(0x2000025CUL)
#endif
#ifndef DAT_20000260
#define DAT_20000260             REG32(0x20000260UL)
#endif
#ifndef DAT_20000264
#define DAT_20000264             REG32(0x20000264UL)
#endif
#ifndef DAT_20000268
#define DAT_20000268             REG32(0x20000268UL)
#endif
#ifndef DAT_2000026c
#define DAT_2000026c             REG32(0x2000026CUL)
#endif
#ifndef DAT_20000270
#define DAT_20000270             REG32(0x20000270UL)
#endif
#ifndef DAT_20000274
#define DAT_20000274             REG32(0x20000274UL)
#endif
#ifndef DAT_20000278
#define DAT_20000278             REG32(0x20000278UL)
#endif
#ifndef DAT_2000027c
#define DAT_2000027c             REG16(0x2000027CUL)
#endif
#ifndef DAT_20000280
#define DAT_20000280             REG8(0x20000280UL)
#endif
#ifndef DAT_20000284
#define DAT_20000284             REG16(0x20000284UL)
#endif
#ifndef DAT_20000288
#define DAT_20000288             REG8(0x20000288UL)
#endif
#ifndef DAT_2000028c
#define DAT_2000028c             REG8(0x2000028CUL)
#endif
#ifndef DAT_20000290
#define DAT_20000290             REG8(0x20000290UL)
#endif
#ifndef DAT_20000294
#define DAT_20000294             REG8(0x20000294UL)
#endif
#ifndef DAT_20000298
#define DAT_20000298             REG8(0x20000298UL)
#endif
#ifndef DAT_2000029c
#define DAT_2000029c             REG16(0x2000029CUL)
#endif
#ifndef DAT_200002a0
#define DAT_200002a0             REG16(0x200002A0UL)
#endif
#ifndef DAT_200002a4
#define DAT_200002a4             REG32(0x200002A4UL)
#endif
#ifndef DAT_200002a8
#define DAT_200002a8             REG32(0x200002A8UL)
#endif
#ifndef DAT_200002ac
#define DAT_200002ac             REG32(0x200002ACUL)
#endif
#ifndef DAT_200002b0
#define DAT_200002b0             REG32(0x200002B0UL)
#endif
#ifndef DAT_200002b4
#define DAT_200002b4             REG8(0x200002B4UL)
#endif
#ifndef DAT_200002b8
#define DAT_200002b8             REG8(0x200002B8UL)
#endif
#ifndef DAT_200002bc
#define DAT_200002bc             REG8(0x200002BCUL)
#endif
#ifndef DAT_200002c0
#define DAT_200002c0             REG8(0x200002C0UL)
#endif
#ifndef DAT_200002c4
#define DAT_200002c4             REG8(0x200002C4UL)
#endif
#ifndef DAT_200002c8
#define DAT_200002c8             REG8(0x200002C8UL)
#endif
#ifndef DAT_200002cc
#define DAT_200002cc             REG8(0x200002CCUL)
#endif
#ifndef DAT_200002d0
#define DAT_200002d0             REG8(0x200002D0UL)
#endif
#ifndef DAT_200002d4
#define DAT_200002d4             REG8(0x200002D4UL)
#endif
#ifndef DAT_200002d8
#define DAT_200002d8             REG8(0x200002D8UL)
#endif
#ifndef DAT_200002dc
#define DAT_200002dc             REG8(0x200002DCUL)
#endif
#ifndef DAT_200002e0
#define DAT_200002e0             REG8(0x200002E0UL)
#endif
#ifndef DAT_200002e4
#define DAT_200002e4             REG8(0x200002E4UL)
#endif
#ifndef DAT_200002e8
#define DAT_200002e8             REG8(0x200002E8UL)
#endif
#ifndef DAT_200002ec
#define DAT_200002ec             REG8(0x200002ECUL)
#endif
#ifndef DAT_200002f0
#define DAT_200002f0             REG8(0x200002F0UL)
#endif
#ifndef DAT_200002f4
#define DAT_200002f4             REG8(0x200002F4UL)
#endif
#ifndef DAT_200002f8
#define DAT_200002f8             REG8(0x200002F8UL)
#endif
#ifndef DAT_200002fc
#define DAT_200002fc             REG8(0x200002FCUL)
#endif
#ifndef DAT_20000300
#define DAT_20000300             REG8(0x20000300UL)
#endif
#ifndef DAT_20000304
#define DAT_20000304             REG8(0x20000304UL)
#endif
#ifndef DAT_20000308
#define DAT_20000308             REG8(0x20000308UL)
#endif
#ifndef DAT_2000030c
#define DAT_2000030c             REG8(0x2000030CUL)
#endif
#ifndef DAT_20000310
#define DAT_20000310             REG8(0x20000310UL)
#endif
#ifndef DAT_20000314
#define DAT_20000314             REG8(0x20000314UL)
#endif
#ifndef DAT_20000318
#define DAT_20000318             REG8(0x20000318UL)
#endif
#ifndef DAT_2000031c
#define DAT_2000031c             REG8(0x2000031CUL)
#endif
#ifndef DAT_20000320
#define DAT_20000320             REG8(0x20000320UL)
#endif
#ifndef DAT_20000324
#define DAT_20000324             REG8(0x20000324UL)
#endif
#ifndef DAT_20000328
#define DAT_20000328             REG8(0x20000328UL)
#endif
#ifndef DAT_2000032c
#define DAT_2000032c             REG8(0x2000032CUL)
#endif
#ifndef DAT_20000330
#define DAT_20000330             REG8(0x20000330UL)
#endif
#ifndef DAT_20000334
#define DAT_20000334             REG8(0x20000334UL)
#endif
#ifndef DAT_2000033a
#define DAT_2000033a             REG8(0x2000033AUL)
#endif
#ifndef DAT_20000340
#define DAT_20000340             REG8(0x20000340UL)
#endif
#ifndef DAT_20000342
#define DAT_20000342             REG8(0x20000342UL)
#endif
#ifndef DAT_20000344
#define DAT_20000344             REG8(0x20000344UL)
#endif
#ifndef DAT_20000347
#define DAT_20000347             REG8(0x20000347UL)
#endif
#ifndef DAT_2000034c
#define DAT_2000034c             REG8(0x2000034CUL)
#endif
#ifndef DAT_20000352
#define DAT_20000352             REG8(0x20000352UL)
#endif
#ifndef DAT_20000356
#define DAT_20000356             REG8(0x20000356UL)
#endif
#ifndef DAT_2000035d
#define DAT_2000035d             REG8(0x2000035DUL)
#endif
#ifndef DAT_20000364
#define DAT_20000364             REG8(0x20000364UL)
#endif
#ifndef DAT_2000036b
#define DAT_2000036b             REG8(0x2000036BUL)
#endif
#ifndef DAT_20000372
#define DAT_20000372             REG8(0x20000372UL)
#endif
#ifndef DAT_20000380
#define DAT_20000380             REG8(0x20000380UL)
#endif
#ifndef DAT_20000384
#define DAT_20000384             REG8(0x20000384UL)
#endif
#ifndef DAT_20000388
#define DAT_20000388             REG8(0x20000388UL)
#endif
#ifndef DAT_2000038c
#define DAT_2000038c             REG8(0x2000038CUL)
#endif
#ifndef DAT_2000047c
#define DAT_2000047c             REG8(0x2000047CUL)
#endif
#ifndef DAT_20000486
#define DAT_20000486             REG8(0x20000486UL)
#endif
#ifndef DAT_2000048c
#define DAT_2000048c             REG8(0x2000048CUL)
#endif
#ifndef DAT_200004d8
#define DAT_200004d8             REG8(0x200004D8UL)
#endif
#ifndef DAT_200004da
#define DAT_200004da             REG8(0x200004DAUL)
#endif
#ifndef DAT_200004e4
#define DAT_200004e4             REG8(0x200004E4UL)
#endif
#ifndef DAT_20000504
#define DAT_20000504             REG8(0x20000504UL)
#endif
#ifndef DAT_20000518
#define DAT_20000518             REG8(0x20000518UL)
#endif
#ifndef DAT_2000051d
#define DAT_2000051d             REG8(0x2000051DUL)
#endif
#ifndef DAT_200005ed
#define DAT_200005ed             REG8(0x200005EDUL)
#endif
#ifndef DAT_200005f1
#define DAT_200005f1             REG8(0x200005F1UL)
#endif
#ifndef DAT_200005f2
#define DAT_200005f2             REG8(0x200005F2UL)
#endif
#ifndef DAT_200005f3
#define DAT_200005f3             REG8(0x200005F3UL)
#endif
#ifndef DAT_200005f4
#define DAT_200005f4             REG8(0x200005F4UL)
#endif
#ifndef DAT_200005f5
#define DAT_200005f5             REG8(0x200005F5UL)
#endif
#ifndef DAT_200005f7
#define DAT_200005f7             REG8(0x200005F7UL)
#endif
#ifndef DAT_200006d2
#define DAT_200006d2             REG16(0x200006D2UL)
#endif
#ifndef DAT_200006d4
#define DAT_200006d4             REG8(0x200006D4UL)
#endif
#ifndef DAT_200006d6
#define DAT_200006d6             REG8(0x200006D6UL)
#endif
#ifndef DAT_200006d8
#define DAT_200006d8             REG8(0x200006D8UL)
#endif
#ifndef DAT_20000764
#define DAT_20000764             REG32(0x20000764UL)
#endif
#ifndef DAT_200007e8
#define DAT_200007e8             REG8(0x200007E8UL)
#endif
#ifndef DAT_200007e9
#define DAT_200007e9             REG8(0x200007E9UL)
#endif
#ifndef DAT_200007ea
#define DAT_200007ea             REG8(0x200007EAUL)
#endif
#ifndef DAT_200007eb
#define DAT_200007eb             REG8(0x200007EBUL)
#endif
#ifndef DAT_200007ec
#define DAT_200007ec             REG8(0x200007ECUL)
#endif
#ifndef DAT_2000080d
#define DAT_2000080d             REG8(0x2000080DUL)
#endif
#ifndef DAT_2000080e
#define DAT_2000080e             REG8(0x2000080EUL)
#endif
#ifndef DAT_2000081c
#define DAT_2000081c             REG8(0x2000081CUL)
#endif
#ifndef DAT_2000081d
#define DAT_2000081d             REG8(0x2000081DUL)
#endif
#ifndef DAT_2000081e
#define DAT_2000081e             REG8(0x2000081EUL)
#endif
#ifndef DAT_2000081f
#define DAT_2000081f             REG8(0x2000081FUL)
#endif
#ifndef DAT_20000820
#define DAT_20000820             REG8(0x20000820UL)
#endif
#ifndef DAT_20000821
#define DAT_20000821             REG8(0x20000821UL)
#endif
#ifndef DAT_20000822
#define DAT_20000822             REG8(0x20000822UL)
#endif
#ifndef DAT_20000823
#define DAT_20000823             REG8(0x20000823UL)
#endif
#ifndef DAT_20000824
#define DAT_20000824             REG8(0x20000824UL)
#endif
#ifndef DAT_20000825
#define DAT_20000825             REG8(0x20000825UL)
#endif
#ifndef DAT_20000826
#define DAT_20000826             REG8(0x20000826UL)
#endif
#ifndef DAT_20000827
#define DAT_20000827             REG8(0x20000827UL)
#endif
#ifndef DAT_20000828
#define DAT_20000828             REG8(0x20000828UL)
#endif
#ifndef DAT_20000829
#define DAT_20000829             REG8(0x20000829UL)
#endif
#ifndef DAT_2000082a
#define DAT_2000082a             REG8(0x2000082AUL)
#endif
#ifndef DAT_2000082b
#define DAT_2000082b             REG8(0x2000082BUL)
#endif
#ifndef DAT_2000082c
#define DAT_2000082c             REG8(0x2000082CUL)
#endif
#ifndef DAT_2000082d
#define DAT_2000082d             REG8(0x2000082DUL)
#endif
#ifndef DAT_2000082e
#define DAT_2000082e             REG8(0x2000082EUL)
#endif
#ifndef DAT_2000090c
#define DAT_2000090c             REG8(0x2000090CUL)
#endif
#ifndef DAT_2000090d
#define DAT_2000090d             REG8(0x2000090DUL)
#endif
#ifndef DAT_2000091b
#define DAT_2000091b             REG8(0x2000091BUL)
#endif
#ifndef DAT_2000091c
#define DAT_2000091c             REG8(0x2000091CUL)
#endif
#ifndef DAT_2000091d
#define DAT_2000091d             REG8(0x2000091DUL)
#endif
#ifndef DAT_2000091e
#define DAT_2000091e             REG8(0x2000091EUL)
#endif
#ifndef DAT_2000091f
#define DAT_2000091f             REG8(0x2000091FUL)
#endif
#ifndef DAT_20000920
#define DAT_20000920             REG8(0x20000920UL)
#endif
#ifndef DAT_20000921
#define DAT_20000921             REG8(0x20000921UL)
#endif
#ifndef DAT_20000922
#define DAT_20000922             REG8(0x20000922UL)
#endif
#ifndef DAT_20000923
#define DAT_20000923             REG8(0x20000923UL)
#endif
#ifndef DAT_20000924
#define DAT_20000924             REG8(0x20000924UL)
#endif
#ifndef DAT_20000925
#define DAT_20000925             REG8(0x20000925UL)
#endif
#ifndef DAT_20000926
#define DAT_20000926             REG8(0x20000926UL)
#endif
#ifndef DAT_20000927
#define DAT_20000927             REG8(0x20000927UL)
#endif
#ifndef DAT_20000928
#define DAT_20000928             REG8(0x20000928UL)
#endif
#ifndef DAT_20000929
#define DAT_20000929             REG8(0x20000929UL)
#endif
#ifndef DAT_2000092a
#define DAT_2000092a             REG8(0x2000092AUL)
#endif
#ifndef DAT_2000092b
#define DAT_2000092b             REG8(0x2000092BUL)
#endif
#ifndef DAT_2000092c
#define DAT_2000092c             REG8(0x2000092CUL)
#endif
#ifndef DAT_2000092d
#define DAT_2000092d             REG8(0x2000092DUL)
#endif
#ifndef DAT_20000a0b
#define DAT_20000a0b             REG8(0x20000A0BUL)
#endif
#ifndef DAT_20000a0c
#define DAT_20000a0c             REG8(0x20000A0CUL)
#endif
#ifndef DAT_20000a1a
#define DAT_20000a1a             REG8(0x20000A1AUL)
#endif
#ifndef DAT_20000a1b
#define DAT_20000a1b             REG8(0x20000A1BUL)
#endif
#ifndef DAT_20000a1c
#define DAT_20000a1c             REG8(0x20000A1CUL)
#endif
#ifndef DAT_20000a1d
#define DAT_20000a1d             REG8(0x20000A1DUL)
#endif
#ifndef DAT_20000a1e
#define DAT_20000a1e             REG8(0x20000A1EUL)
#endif
#ifndef DAT_20000a1f
#define DAT_20000a1f             REG8(0x20000A1FUL)
#endif
#ifndef DAT_20000a20
#define DAT_20000a20             REG8(0x20000A20UL)
#endif
#ifndef DAT_20000a21
#define DAT_20000a21             REG8(0x20000A21UL)
#endif
#ifndef DAT_20000a22
#define DAT_20000a22             REG8(0x20000A22UL)
#endif
#ifndef DAT_20000a23
#define DAT_20000a23             REG8(0x20000A23UL)
#endif
#ifndef DAT_20000a24
#define DAT_20000a24             REG8(0x20000A24UL)
#endif
#ifndef DAT_20000a25
#define DAT_20000a25             REG8(0x20000A25UL)
#endif
#ifndef DAT_20000a26
#define DAT_20000a26             REG8(0x20000A26UL)
#endif
#ifndef DAT_20000a27
#define DAT_20000a27             REG8(0x20000A27UL)
#endif
#ifndef DAT_20000a28
#define DAT_20000a28             REG8(0x20000A28UL)
#endif
#ifndef DAT_20000a29
#define DAT_20000a29             REG8(0x20000A29UL)
#endif
#ifndef DAT_20000a2a
#define DAT_20000a2a             REG8(0x20000A2AUL)
#endif
#ifndef DAT_20000a2b
#define DAT_20000a2b             REG8(0x20000A2BUL)
#endif
#ifndef DAT_20000a2c
#define DAT_20000a2c             REG8(0x20000A2CUL)
#endif
#ifndef DAT_20000b9c
#define DAT_20000b9c             REG8(0x20000B9CUL)
#endif
#ifndef DAT_20000b9d
#define DAT_20000b9d             REG8(0x20000B9DUL)
#endif
#ifndef DAT_20000b9e
#define DAT_20000b9e             REG8(0x20000B9EUL)
#endif
#ifndef DAT_20000b9f
#define DAT_20000b9f             REG8(0x20000B9FUL)
#endif
#ifndef DAT_20000ba0
#define DAT_20000ba0             REG8(0x20000BA0UL)
#endif
#ifndef DAT_20000ba1
#define DAT_20000ba1             REG8(0x20000BA1UL)
#endif
#ifndef DAT_20000ba2
#define DAT_20000ba2             REG8(0x20000BA2UL)
#endif
#ifndef DAT_20000ba3
#define DAT_20000ba3             REG8(0x20000BA3UL)
#endif
#ifndef DAT_20000ba4
#define DAT_20000ba4             REG8(0x20000BA4UL)
#endif
#ifndef DAT_20000ba5
#define DAT_20000ba5             REG8(0x20000BA5UL)
#endif
#ifndef DAT_20000ba6
#define DAT_20000ba6             REG8(0x20000BA6UL)
#endif
#ifndef DAT_20000ba7
#define DAT_20000ba7             REG8(0x20000BA7UL)
#endif
#ifndef DAT_20000ba8
#define DAT_20000ba8             REG8(0x20000BA8UL)
#endif
#ifndef DAT_20000ba9
#define DAT_20000ba9             REG8(0x20000BA9UL)
#endif
#ifndef DAT_20000baa
#define DAT_20000baa             REG8(0x20000BAAUL)
#endif
#ifndef DAT_20000bab
#define DAT_20000bab             REG8(0x20000BABUL)
#endif
#ifndef DAT_20000bac
#define DAT_20000bac             REG8(0x20000BACUL)
#endif
#ifndef DAT_20000bad
#define DAT_20000bad             REG8(0x20000BADUL)
#endif
#ifndef DAT_20000bae
#define DAT_20000bae             REG8(0x20000BAEUL)
#endif
#ifndef DAT_20000baf
#define DAT_20000baf             REG8(0x20000BAFUL)
#endif
#ifndef DAT_20000bb0
#define DAT_20000bb0             REG8(0x20000BB0UL)
#endif
#ifndef DAT_20000bb1
#define DAT_20000bb1             REG8(0x20000BB1UL)
#endif
#ifndef DAT_20000bb2
#define DAT_20000bb2             REG8(0x20000BB2UL)
#endif
#ifndef DAT_20000bb3
#define DAT_20000bb3             REG8(0x20000BB3UL)
#endif
#ifndef DAT_20000bb4
#define DAT_20000bb4             REG8(0x20000BB4UL)
#endif
#ifndef DAT_20000bb5
#define DAT_20000bb5             REG8(0x20000BB5UL)
#endif
#ifndef DAT_20000bb6
#define DAT_20000bb6             REG8(0x20000BB6UL)
#endif
#ifndef DAT_20000bb7
#define DAT_20000bb7             SREG8(0x20000BB7UL)
#endif
#ifndef DAT_20000bb8
#define DAT_20000bb8             REG8(0x20000BB8UL)
#endif
#ifndef DAT_20000bb9
#define DAT_20000bb9             REG8(0x20000BB9UL)
#endif
#ifndef DAT_20000bba
#define DAT_20000bba             REG8(0x20000BBAUL)
#endif
#ifndef DAT_20000bbb
#define DAT_20000bbb             REG8(0x20000BBBUL)
#endif
#ifndef DAT_20000bbc
#define DAT_20000bbc             REG8(0x20000BBCUL)
#endif
#ifndef DAT_20000bbd
#define DAT_20000bbd             REG8(0x20000BBDUL)
#endif
#ifndef DAT_20000bbe
#define DAT_20000bbe             REG8(0x20000BBEUL)
#endif
#ifndef DAT_20000bbf
#define DAT_20000bbf             REG8(0x20000BBFUL)
#endif
#ifndef DAT_20000c9c
#define DAT_20000c9c             REG8(0x20000C9CUL)
#endif
#ifndef DAT_20000d9c
#define DAT_20000d9c             REG8(0x20000D9CUL)
#endif
#ifndef DAT_20000e9c
#define DAT_20000e9c             REG8(0x20000E9CUL)
#endif
#ifndef DAT_20001000
#define DAT_20001000             REG8(0x20001000UL)
#endif
#ifndef DAT_20001001
#define DAT_20001001             REG8(0x20001001UL)
#endif
#ifndef DAT_20001024
#define DAT_20001024             REG8(0x20001024UL)
#endif
#ifndef DAT_20001046
#define DAT_20001046             REG8(0x20001046UL)
#endif
#ifndef DAT_20001047
#define DAT_20001047             REG8(0x20001047UL)
#endif
#ifndef DAT_20001068
#define DAT_20001068             REG8(0x20001068UL)
#endif
#ifndef DAT_2000108c
#define DAT_2000108c             REG8(0x2000108CUL)
#endif
#ifndef DAT_2000108d
#define DAT_2000108d             REG8(0x2000108DUL)
#endif
#ifndef DAT_2000108e
#define DAT_2000108e             REG8(0x2000108EUL)
#endif
#ifndef DAT_2000108f
#define DAT_2000108f             REG8(0x2000108FUL)
#endif
#ifndef DAT_2000388a
#define DAT_2000388a             REG8(0x2000388AUL)
#endif
#ifndef DAT_20003892
#define DAT_20003892             REG8(0x20003892UL)
#endif
#ifndef DAT_20003893
#define DAT_20003893             REG8(0x20003893UL)
#endif
#ifndef DAT_20003894
#define DAT_20003894             REG8(0x20003894UL)
#endif
#ifndef DAT_20003895
#define DAT_20003895             SREG8(0x20003895UL)
#endif
#ifndef DAT_20003896
#define DAT_20003896             REG8(0x20003896UL)
#endif
#ifndef DAT_200038a2
#define DAT_200038a2             REG8(0x200038A2UL)
#endif
#ifndef DAT_200038a8
#define DAT_200038a8             REG8(0x200038A8UL)
#endif
#ifndef DAT_200038a9
#define DAT_200038a9             REG8(0x200038A9UL)
#endif
#ifndef DAT_200038aa
#define DAT_200038aa             REG8(0x200038AAUL)
#endif
#ifndef DAT_200038ae
#define DAT_200038ae             REG8(0x200038AEUL)
#endif
#ifndef DAT_200038af
#define DAT_200038af             REG8(0x200038AFUL)
#endif
#ifndef DAT_200038b0
#define DAT_200038b0             REG8(0x200038B0UL)
#endif
#ifndef DAT_200038b1
#define DAT_200038b1             REG8(0x200038B1UL)
#endif
#ifndef DAT_200038b6
#define DAT_200038b6             REG8(0x200038B6UL)
#endif
#ifndef DAT_200038b7
#define DAT_200038b7             REG8(0x200038B7UL)
#endif
#ifndef DAT_200038b8
#define DAT_200038b8             REG8(0x200038B8UL)
#endif
#ifndef DAT_200038b9
#define DAT_200038b9             REG8(0x200038B9UL)
#endif
#ifndef DAT_200038ba
#define DAT_200038ba             REG8(0x200038BAUL)
#endif
#ifndef DAT_200038bb
#define DAT_200038bb             REG8(0x200038BBUL)
#endif
#ifndef DAT_200038bc
#define DAT_200038bc             REG8(0x200038BCUL)
#endif
#ifndef DAT_200038bd
#define DAT_200038bd             REG8(0x200038BDUL)
#endif
#ifndef DAT_200038be
#define DAT_200038be             REG8(0x200038BEUL)
#endif
#ifndef DAT_200038cb
#define DAT_200038cb             REG8(0x200038CBUL)
#endif
#ifndef DAT_200038cc
#define DAT_200038cc             REG8(0x200038CCUL)
#endif
#ifndef DAT_200038cd
#define DAT_200038cd             REG8(0x200038CDUL)
#endif
#ifndef DAT_200038ce
#define DAT_200038ce             REG8(0x200038CEUL)
#endif
#ifndef DAT_200038cf
#define DAT_200038cf             REG8(0x200038CFUL)
#endif
#ifndef DAT_200038d1
#define DAT_200038d1             REG8(0x200038D1UL)
#endif
#ifndef DAT_200038d2
#define DAT_200038d2             REG8(0x200038D2UL)
#endif
#ifndef DAT_200038d3
#define DAT_200038d3             REG8(0x200038D3UL)
#endif
#ifndef DAT_200038d4
#define DAT_200038d4             REG8(0x200038D4UL)
#endif
#ifndef DAT_200038d5
#define DAT_200038d5             REG8(0x200038D5UL)
#endif
#ifndef DAT_20003a08
#define DAT_20003a08             REG8(0x20003A08UL)
#endif
#ifndef DAT_20003b07
#define DAT_20003b07             REG8(0x20003B07UL)
#endif
#ifndef DAT_20004000
#define DAT_20004000             REG16(0x20004000UL)
#endif
#ifndef DAT_2000422c
#define DAT_2000422c             REG8(0x2000422CUL)
#endif
#ifndef DAT_20004440
#define DAT_20004440             REG16(0x20004440UL)
#endif
#ifndef DAT_20005000
#define DAT_20005000             REG8(0x20005000UL)
#endif
#ifndef DAT_20005100
#define DAT_20005100             REG32(0x20005100UL)
#endif
#ifndef DAT_20005104
#define DAT_20005104             REG32(0x20005104UL)
#endif
#ifndef DAT_2000510c
#define DAT_2000510c             REG32(0x2000510CUL)
#endif
#ifndef DAT_2000511c
#define DAT_2000511c             REG32(0x2000511CUL)
#endif
#ifndef DAT_20005120
#define DAT_20005120             REG32(0x20005120UL)
#endif
#ifndef DAT_20005124
#define DAT_20005124             REG32(0x20005124UL)
#endif
#ifndef DAT_20005128
#define DAT_20005128             REG32(0x20005128UL)
#endif
#ifndef DAT_2000512c
#define DAT_2000512c             REG8(0x2000512CUL)
#endif
#ifndef DAT_200051cc
#define DAT_200051cc             REG8(0x200051CCUL)
#endif
#ifndef DAT_200051cd
#define DAT_200051cd             REG8(0x200051CDUL)
#endif
#ifndef DAT_200051ce
#define DAT_200051ce             REG8(0x200051CEUL)
#endif
#ifndef DAT_200051cf
#define DAT_200051cf             REG8(0x200051CFUL)
#endif
#ifndef DAT_200051d0
#define DAT_200051d0             REG8(0x200051D0UL)
#endif
#ifndef DAT_200051d1
#define DAT_200051d1             REG8(0x200051D1UL)
#endif
#ifndef DAT_200051d2
#define DAT_200051d2             REG8(0x200051D2UL)
#endif
#ifndef DAT_200052cf
#define DAT_200052cf             REG8(0x200052CFUL)
#endif
#ifndef DAT_200052d0
#define DAT_200052d0             REG8(0x200052D0UL)
#endif
#ifndef DAT_200052d1
#define DAT_200052d1             REG8(0x200052D1UL)
#endif
#ifndef DAT_200052d2
#define DAT_200052d2             REG8(0x200052D2UL)
#endif
#ifndef DAT_200053cc
#define DAT_200053cc             REG8(0x200053CCUL)
#endif
#ifndef DAT_200053cd
#define DAT_200053cd             REG8(0x200053CDUL)
#endif
#ifndef DAT_200053ce
#define DAT_200053ce             REG8(0x200053CEUL)
#endif
#ifndef DAT_200053cf
#define DAT_200053cf             REG8(0x200053CFUL)
#endif
#ifndef DAT_200053d0
#define DAT_200053d0             REG8(0x200053D0UL)
#endif
#ifndef DAT_200053d1
#define DAT_200053d1             REG8(0x200053D1UL)
#endif
#ifndef DAT_200053d2
#define DAT_200053d2             REG8(0x200053D2UL)
#endif
#ifndef DAT_200053d3
#define DAT_200053d3             REG8(0x200053D3UL)
#endif
#ifndef DAT_200053d6
#define DAT_200053d6             REG8(0x200053D6UL)
#endif
#ifndef DAT_200053d7
#define DAT_200053d7             REG8(0x200053D7UL)
#endif
#ifndef DAT_200054cc
#define DAT_200054cc             REG8(0x200054CCUL)
#endif
#ifndef DAT_200054cd
#define DAT_200054cd             REG8(0x200054CDUL)
#endif
#ifndef DAT_200054ce
#define DAT_200054ce             REG8(0x200054CEUL)
#endif
#ifndef DAT_200054cf
#define DAT_200054cf             REG8(0x200054CFUL)
#endif
#ifndef DAT_200054d0
#define DAT_200054d0             REG8(0x200054D0UL)
#endif
#ifndef DAT_200054d1
#define DAT_200054d1             REG8(0x200054D1UL)
#endif
#ifndef DAT_200054d2
#define DAT_200054d2             REG8(0x200054D2UL)
#endif
#ifndef DAT_200054d3
#define DAT_200054d3             REG8(0x200054D3UL)
#endif
#ifndef DAT_200054d6
#define DAT_200054d6             REG8(0x200054D6UL)
#endif
#ifndef DAT_200054d7
#define DAT_200054d7             REG8(0x200054D7UL)
#endif
#ifndef DAT_200055cc
#define DAT_200055cc             REG8(0x200055CCUL)
#endif
#ifndef DAT_200055cd
#define DAT_200055cd             REG8(0x200055CDUL)
#endif
#ifndef DAT_200055ce
#define DAT_200055ce             REG8(0x200055CEUL)
#endif
#ifndef DAT_200055cf
#define DAT_200055cf             REG8(0x200055CFUL)
#endif
#ifndef DAT_200055d0
#define DAT_200055d0             REG8(0x200055D0UL)
#endif
#ifndef DAT_200055d1
#define DAT_200055d1             REG8(0x200055D1UL)
#endif
#ifndef DAT_200055d2
#define DAT_200055d2             REG8(0x200055D2UL)
#endif
#ifndef DAT_200055d3
#define DAT_200055d3             REG8(0x200055D3UL)
#endif
#ifndef DAT_200055d4
#define DAT_200055d4             REG8(0x200055D4UL)
#endif
#ifndef DAT_200055d5
#define DAT_200055d5             REG8(0x200055D5UL)
#endif
#ifndef DAT_200055d6
#define DAT_200055d6             REG8(0x200055D6UL)
#endif
#ifndef DAT_200055d7
#define DAT_200055d7             REG8(0x200055D7UL)
#endif
#ifndef DAT_200055d8
#define DAT_200055d8             REG8(0x200055D8UL)
#endif
#ifndef DAT_200055d9
#define DAT_200055d9             REG8(0x200055D9UL)
#endif
#ifndef DAT_200055da
#define DAT_200055da             REG8(0x200055DAUL)
#endif
#ifndef DAT_200055db
#define DAT_200055db             REG8(0x200055DBUL)
#endif
#ifndef DAT_200055dc
#define DAT_200055dc             REG8(0x200055DCUL)
#endif
#ifndef DAT_200055dd
#define DAT_200055dd             REG8(0x200055DDUL)
#endif
#ifndef DAT_200055de
#define DAT_200055de             REG8(0x200055DEUL)
#endif
#ifndef DAT_200055df
#define DAT_200055df             REG8(0x200055DFUL)
#endif
#ifndef DAT_200055e0
#define DAT_200055e0             REG8(0x200055E0UL)
#endif
#ifndef DAT_200055e1
#define DAT_200055e1             REG8(0x200055E1UL)
#endif
#ifndef DAT_200055e2
#define DAT_200055e2             REG8(0x200055E2UL)
#endif
#ifndef DAT_200055e3
#define DAT_200055e3             REG8(0x200055E3UL)
#endif
#ifndef DAT_200055e4
#define DAT_200055e4             REG8(0x200055E4UL)
#endif
#ifndef DAT_200055e5
#define DAT_200055e5             REG8(0x200055E5UL)
#endif
#ifndef DAT_200055e6
#define DAT_200055e6             REG8(0x200055E6UL)
#endif
#ifndef DAT_200055e7
#define DAT_200055e7             SREG8(0x200055E7UL)
#endif
#ifndef DAT_200055e8
#define DAT_200055e8             REG8(0x200055E8UL)
#endif
#ifndef DAT_200055e9
#define DAT_200055e9             REG8(0x200055E9UL)
#endif
#ifndef DAT_200055ea
#define DAT_200055ea             REG8(0x200055EAUL)
#endif
#ifndef DAT_200055eb
#define DAT_200055eb             REG8(0x200055EBUL)
#endif
#ifndef DAT_200055ec
#define DAT_200055ec             REG8(0x200055ECUL)
#endif
#ifndef DAT_200055ed
#define DAT_200055ed             REG8(0x200055EDUL)
#endif
#ifndef DAT_200055ee
#define DAT_200055ee             REG8(0x200055EEUL)
#endif
#ifndef DAT_200055ef
#define DAT_200055ef             REG8(0x200055EFUL)
#endif
#ifndef DAT_200056cc
#define DAT_200056cc             REG8(0x200056CCUL)
#endif
#ifndef DAT_200057cc
#define DAT_200057cc             REG8(0x200057CCUL)
#endif
#ifndef DAT_200058cc
#define DAT_200058cc             REG8(0x200058CCUL)
#endif
#ifndef DAT_200058fb
#define DAT_200058fb             REG8(0x200058FBUL)
#endif
#ifndef DAT_20005969
#define DAT_20005969             SREG8(0x20005969UL)
#endif
#ifndef DAT_20005991
#define DAT_20005991             REG8(0x20005991UL)
#endif
#ifndef DAT_20005a36
#define DAT_20005a36             REG8(0x20005A36UL)
#endif
#ifndef DAT_20005a37
#define DAT_20005a37             REG8(0x20005A37UL)
#endif
#ifndef DAT_20005a39
#define DAT_20005a39             REG8(0x20005A39UL)
#endif
#ifndef DAT_20005a3a
#define DAT_20005a3a             REG8(0x20005A3AUL)
#endif
#ifndef DAT_20005a4a
#define DAT_20005a4a             REG8(0x20005A4AUL)
#endif
#ifndef DAT_20005a4b
#define DAT_20005a4b             REG8(0x20005A4BUL)
#endif
#ifndef DAT_20005a4c
#define DAT_20005a4c             REG8(0x20005A4CUL)
#endif
#ifndef DAT_20005a4d
#define DAT_20005a4d             REG8(0x20005A4DUL)
#endif
#ifndef DAT_20005a4e
#define DAT_20005a4e             REG8(0x20005A4EUL)
#endif
#ifndef DAT_20005a4f
#define DAT_20005a4f             REG8(0x20005A4FUL)
#endif
#ifndef DAT_20005a50
#define DAT_20005a50             REG8(0x20005A50UL)
#endif
#ifndef DAT_20005a51
#define DAT_20005a51             REG8(0x20005A51UL)
#endif
#ifndef DAT_20005a52
#define DAT_20005a52             REG8(0x20005A52UL)
#endif
#ifndef DAT_20005a53
#define DAT_20005a53             REG8(0x20005A53UL)
#endif
#ifndef DAT_20005a54
#define DAT_20005a54             REG8(0x20005A54UL)
#endif
#ifndef DAT_20005a55
#define DAT_20005a55             REG8(0x20005A55UL)
#endif
#ifndef DAT_20005a56
#define DAT_20005a56             REG8(0x20005A56UL)
#endif
#ifndef DAT_20005a57
#define DAT_20005a57             REG8(0x20005A57UL)
#endif
#ifndef DAT_20005a58
#define DAT_20005a58             REG8(0x20005A58UL)
#endif
#ifndef DAT_20005a59
#define DAT_20005a59             REG8(0x20005A59UL)
#endif
#ifndef DAT_20005a5a
#define DAT_20005a5a             REG8(0x20005A5AUL)
#endif
#ifndef DAT_20005a5b
#define DAT_20005a5b             REG8(0x20005A5BUL)
#endif
#ifndef DAT_20005a5c
#define DAT_20005a5c             REG8(0x20005A5CUL)
#endif
#ifndef DAT_20005a5d
#define DAT_20005a5d             REG8(0x20005A5DUL)
#endif
#ifndef DAT_20005a5e
#define DAT_20005a5e             REG8(0x20005A5EUL)
#endif
#ifndef DAT_20005a5f
#define DAT_20005a5f             REG8(0x20005A5FUL)
#endif
#ifndef DAT_20005a60
#define DAT_20005a60             REG8(0x20005A60UL)
#endif
#ifndef DAT_20005a61
#define DAT_20005a61             REG8(0x20005A61UL)
#endif
#ifndef DAT_20005a62
#define DAT_20005a62             REG8(0x20005A62UL)
#endif
#ifndef DAT_20005a63
#define DAT_20005a63             REG8(0x20005A63UL)
#endif
#ifndef DAT_20005a64
#define DAT_20005a64             REG8(0x20005A64UL)
#endif
#ifndef DAT_20005a65
#define DAT_20005a65             REG8(0x20005A65UL)
#endif
#ifndef DAT_20005a66
#define DAT_20005a66             REG8(0x20005A66UL)
#endif
#ifndef DAT_20005a67
#define DAT_20005a67             REG8(0x20005A67UL)
#endif
#ifndef DAT_20005a68
#define DAT_20005a68             REG8(0x20005A68UL)
#endif
#ifndef DAT_20005a69
#define DAT_20005a69             REG8(0x20005A69UL)
#endif
#ifndef DAT_20005a6a
#define DAT_20005a6a             REG8(0x20005A6AUL)
#endif
#ifndef DAT_20005a6b
#define DAT_20005a6b             REG8(0x20005A6BUL)
#endif
#ifndef DAT_20005a6c
#define DAT_20005a6c             REG8(0x20005A6CUL)
#endif
#ifndef DAT_20005a6d
#define DAT_20005a6d             REG8(0x20005A6DUL)
#endif
#ifndef DAT_20005a6e
#define DAT_20005a6e             REG8(0x20005A6EUL)
#endif
#ifndef DAT_20005a6f
#define DAT_20005a6f             REG8(0x20005A6FUL)
#endif
#ifndef DAT_20005a70
#define DAT_20005a70             REG8(0x20005A70UL)
#endif
#ifndef DAT_20005a71
#define DAT_20005a71             REG8(0x20005A71UL)
#endif
#ifndef DAT_20005a72
#define DAT_20005a72             REG8(0x20005A72UL)
#endif
#ifndef DAT_20005a73
#define DAT_20005a73             REG8(0x20005A73UL)
#endif
#ifndef DAT_20005a74
#define DAT_20005a74             REG8(0x20005A74UL)
#endif
#ifndef DAT_20005a75
#define DAT_20005a75             REG8(0x20005A75UL)
#endif
#ifndef DAT_20005a76
#define DAT_20005a76             REG8(0x20005A76UL)
#endif
#ifndef DAT_20005a77
#define DAT_20005a77             REG8(0x20005A77UL)
#endif
#ifndef DAT_20005a78
#define DAT_20005a78             REG8(0x20005A78UL)
#endif
#ifndef DAT_20005a79
#define DAT_20005a79             REG8(0x20005A79UL)
#endif
#ifndef DAT_20005a7a
#define DAT_20005a7a             REG8(0x20005A7AUL)
#endif
#ifndef DAT_20005a7b
#define DAT_20005a7b             REG8(0x20005A7BUL)
#endif
#ifndef DAT_20005a7c
#define DAT_20005a7c             REG8(0x20005A7CUL)
#endif
#ifndef DAT_20005a7d
#define DAT_20005a7d             REG8(0x20005A7DUL)
#endif
#ifndef DAT_20005a7e
#define DAT_20005a7e             REG8(0x20005A7EUL)
#endif
#ifndef DAT_20005ab0
#define DAT_20005ab0             REG8(0x20005AB0UL)
#endif
#ifndef DAT_20005ac0
#define DAT_20005ac0             REG8(0x20005AC0UL)
#endif
#ifndef DAT_20005ad0
#define DAT_20005ad0             REG8(0x20005AD0UL)
#endif
#ifndef DAT_20005ae0
#define DAT_20005ae0             REG8(0x20005AE0UL)
#endif
#ifndef DAT_20005af0
#define DAT_20005af0             REG8(0x20005AF0UL)
#endif
#ifndef DAT_20005af1
#define DAT_20005af1             REG8(0x20005AF1UL)
#endif
#ifndef DAT_20005af6
#define DAT_20005af6             REG8(0x20005AF6UL)
#endif
#ifndef DAT_20005af7
#define DAT_20005af7             REG8(0x20005AF7UL)
#endif
#ifndef DAT_20005af8
#define DAT_20005af8             REG8(0x20005AF8UL)
#endif
#ifndef DAT_20005af9
#define DAT_20005af9             REG8(0x20005AF9UL)
#endif
#ifndef DAT_20005afa
#define DAT_20005afa             REG8(0x20005AFAUL)
#endif
#ifndef DAT_20005afb
#define DAT_20005afb             REG8(0x20005AFBUL)
#endif
#ifndef DAT_20005afc
#define DAT_20005afc             REG8(0x20005AFCUL)
#endif
#ifndef DAT_20005afd
#define DAT_20005afd             REG8(0x20005AFDUL)
#endif
#ifndef DAT_20005bd0
#define DAT_20005bd0             REG8(0x20005BD0UL)
#endif
#ifndef DAT_20005bd1
#define DAT_20005bd1             REG8(0x20005BD1UL)
#endif
#ifndef DAT_20005bd2
#define DAT_20005bd2             REG8(0x20005BD2UL)
#endif
#ifndef DAT_20005bf0
#define DAT_20005bf0             REG8(0x20005BF0UL)
#endif
#ifndef DAT_20005d48
#define DAT_20005d48             REG8(0x20005D48UL)
#endif
#ifndef DAT_20005db8
#define DAT_20005db8             REG8(0x20005DB8UL)
#endif
#ifndef DAT_20005e1a
#define DAT_20005e1a             REG8(0x20005E1AUL)
#endif
#ifndef DAT_20005e1b
#define DAT_20005e1b             REG8(0x20005E1BUL)
#endif
#ifndef DAT_20005e98
#define DAT_20005e98             REG8(0x20005E98UL)
#endif
#ifndef DAT_20005e99
#define DAT_20005e99             REG8(0x20005E99UL)
#endif
#ifndef DAT_20005e9a
#define DAT_20005e9a             REG8(0x20005E9AUL)
#endif
#ifndef DAT_20005e9b
#define DAT_20005e9b             REG8(0x20005E9BUL)
#endif
#ifndef DAT_20005f18
#define DAT_20005f18             REG8(0x20005F18UL)
#endif
#ifndef DAT_20005f1c
#define DAT_20005f1c             REG8(0x20005F1CUL)
#endif
#ifndef DAT_20005f20
#define DAT_20005f20             REG8(0x20005F20UL)
#endif
#ifndef DAT_20005f24
#define DAT_20005f24             REG8(0x20005F24UL)
#endif
#ifndef DAT_20005f28
#define DAT_20005f28             REG8(0x20005F28UL)
#endif
#ifndef DAT_20005f29
#define DAT_20005f29             REG8(0x20005F29UL)
#endif
#ifndef DAT_20005f2b
#define DAT_20005f2b             REG8(0x20005F2BUL)
#endif
#ifndef DAT_20005f2c
#define DAT_20005f2c             REG8(0x20005F2CUL)
#endif
#ifndef DAT_20005f2d
#define DAT_20005f2d             REG8(0x20005F2DUL)
#endif
#ifndef DAT_20005f2e
#define DAT_20005f2e             REG8(0x20005F2EUL)
#endif
#ifndef DAT_20005f2f
#define DAT_20005f2f             REG8(0x20005F2FUL)
#endif
#ifndef DAT_20005f31
#define DAT_20005f31             REG8(0x20005F31UL)
#endif
#ifndef DAT_20005f3e
#define DAT_20005f3e             REG8(0x20005F3EUL)
#endif
#ifndef DAT_20005f3f
#define DAT_20005f3f             REG8(0x20005F3FUL)
#endif
#ifndef DAT_20005f40
#define DAT_20005f40             REG8(0x20005F40UL)
#endif
#ifndef DAT_20005f41
#define DAT_20005f41             REG8(0x20005F41UL)
#endif
#ifndef DAT_20005f42
#define DAT_20005f42             REG8(0x20005F42UL)
#endif
#ifndef DAT_20005f43
#define DAT_20005f43             REG8(0x20005F43UL)
#endif
#ifndef DAT_20005f44
#define DAT_20005f44             REG8(0x20005F44UL)
#endif
#ifndef DAT_20005f45
#define DAT_20005f45             REG8(0x20005F45UL)
#endif
#ifndef DAT_20005f46
#define DAT_20005f46             REG8(0x20005F46UL)
#endif
#ifndef DAT_20005f47
#define DAT_20005f47             REG8(0x20005F47UL)
#endif
#ifndef DAT_20005f48
#define DAT_20005f48             REG8(0x20005F48UL)
#endif
#ifndef DAT_20005f49
#define DAT_20005f49             REG8(0x20005F49UL)
#endif
#ifndef DAT_20005f7e
#define DAT_20005f7e             REG8(0x20005F7EUL)
#endif
#ifndef DAT_20005f80
#define DAT_20005f80             REG8(0x20005F80UL)
#endif
#ifndef DAT_20005f82
#define DAT_20005f82             REG8(0x20005F82UL)
#endif
#ifndef DAT_20005f83
#define DAT_20005f83             REG8(0x20005F83UL)
#endif
#ifndef DAT_20005f84
#define DAT_20005f84             REG8(0x20005F84UL)
#endif
#ifndef DAT_20005f85
#define DAT_20005f85             REG8(0x20005F85UL)
#endif
#ifndef DAT_20005faa
#define DAT_20005faa             REG8(0x20005FAAUL)
#endif
#ifndef DAT_20005fd2
#define DAT_20005fd2             REG8(0x20005FD2UL)
#endif
#ifndef DAT_20005fd3
#define DAT_20005fd3             REG8(0x20005FD3UL)
#endif
#ifndef DAT_20005fdc
#define DAT_20005fdc             REG8(0x20005FDCUL)
#endif
#ifndef DAT_20005fdd
#define DAT_20005fdd             REG8(0x20005FDDUL)
#endif
#ifndef DAT_20005fe2
#define DAT_20005fe2             REG8(0x20005FE2UL)
#endif
#ifndef DAT_20005fe4
#define DAT_20005fe4             REG8(0x20005FE4UL)
#endif
#ifndef DAT_20005fe6
#define DAT_20005fe6             REG8(0x20005FE6UL)
#endif
#ifndef DAT_20005fe7
#define DAT_20005fe7             REG8(0x20005FE7UL)
#endif
#ifndef DAT_20005fe8
#define DAT_20005fe8             REG8(0x20005FE8UL)
#endif
#ifndef DAT_20005fe9
#define DAT_20005fe9             REG8(0x20005FE9UL)
#endif
#ifndef DAT_200060b0
#define DAT_200060b0             REG8(0x200060B0UL)
#endif
#ifndef DAT_200060b1
#define DAT_200060b1             REG8(0x200060B1UL)
#endif
#ifndef DAT_200060b4
#define DAT_200060b4             REG8(0x200060B4UL)
#endif
#ifndef DAT_200060b5
#define DAT_200060b5             REG8(0x200060B5UL)
#endif
#ifndef DAT_20006150
#define DAT_20006150             REG8(0x20006150UL)
#endif
#ifndef DAT_20006158
#define DAT_20006158             REG8(0x20006158UL)
#endif
#ifndef DAT_2000615c
#define DAT_2000615c             REG8(0x2000615CUL)
#endif
#ifndef DAT_20006160
#define DAT_20006160             REG8(0x20006160UL)
#endif
#ifndef DAT_200061a0
#define DAT_200061a0             REG16(0x200061A0UL)
#endif
#ifndef DAT_200061a2
#define DAT_200061a2             REG8(0x200061A2UL)
#endif
#ifndef DAT_200061a4
#define DAT_200061a4             REG8(0x200061A4UL)
#endif
#ifndef DAT_200061a6
#define DAT_200061a6             REG8(0x200061A6UL)
#endif
#ifndef DAT_200061a8
#define DAT_200061a8             REG8(0x200061A8UL)
#endif
#ifndef DAT_200061aa
#define DAT_200061aa             REG8(0x200061AAUL)
#endif
#ifndef DAT_200061ac
#define DAT_200061ac             REG8(0x200061ACUL)
#endif
#ifndef DAT_200061b8
#define DAT_200061b8             REG8(0x200061B8UL)
#endif
#ifndef DAT_200061c4
#define DAT_200061c4             REG16(0x200061C4UL)
#endif
#ifndef DAT_200061c6
#define DAT_200061c6             REG16(0x200061C6UL)
#endif
#ifndef DAT_200061e2
#define DAT_200061e2             REG16(0x200061E2UL)
#endif
#ifndef DAT_2000b918
#define DAT_2000b918             REG8(0x2000B918UL)
#endif
#ifndef DAT_2000b919
#define DAT_2000b919             REG8(0x2000B919UL)
#endif
#ifndef DAT_2000b91a
#define DAT_2000b91a             REG8(0x2000B91AUL)
#endif
#ifndef DAT_2000b91b
#define DAT_2000b91b             REG8(0x2000B91BUL)
#endif
#ifndef DAT_2000bba4
#define DAT_2000bba4             REG8(0x2000BBA4UL)
#endif
#ifndef DAT_2000bbc8
#define DAT_2000bbc8             REG8(0x2000BBC8UL)
#endif
#ifndef DAT_2000bbd1
#define DAT_2000bbd1             REG8(0x2000BBD1UL)
#endif
#ifndef DAT_2000bbda
#define DAT_2000bbda             REG8(0x2000BBDAUL)
#endif
#ifndef DAT_2000bbe3
#define DAT_2000bbe3             REG8(0x2000BBE3UL)
#endif
#ifndef DAT_2000bbec
#define DAT_2000bbec             REG8(0x2000BBECUL)
#endif
#ifndef DAT_2000bbf5
#define DAT_2000bbf5             REG8(0x2000BBF5UL)
#endif
#ifndef DAT_40000000
#define DAT_40000000             REG32(0x40000000UL)
#endif
#ifndef DAT_40000024
#define DAT_40000024             REG32(0x40000024UL)
#endif
#ifndef DAT_40000400
#define DAT_40000400             REG32(0x40000400UL)
#endif
#ifndef DAT_40000c00
#define DAT_40000c00             REG32(0x40000C00UL)
#endif
#ifndef DAT_40000c24
#define DAT_40000c24             REG32(0x40000C24UL)
#endif
#ifndef DAT_40001000
#define DAT_40001000             REG32(0x40001000UL)
#endif
#ifndef DAT_40001024
#define DAT_40001024             REG32(0x40001024UL)
#endif
#ifndef DAT_4000102c
#define DAT_4000102c             REG32(0x4000102CUL)
#endif
#ifndef DAT_40001400
#define DAT_40001400             REG32(0x40001400UL)
#endif
#ifndef DAT_40001424
#define DAT_40001424             REG32(0x40001424UL)
#endif
#ifndef DAT_4000142c
#define DAT_4000142c             REG32(0x4000142CUL)
#endif
#ifndef DAT_40003000
#define DAT_40003000             REG32(0x40003000UL)
#endif
#ifndef DAT_40003004
#define DAT_40003004             REG32(0x40003004UL)
#endif
#ifndef DAT_40003008
#define DAT_40003008             REG32(0x40003008UL)
#endif
#ifndef DAT_40004400
#define DAT_40004400             REG16(0x40004400UL)
#endif
#ifndef DAT_40004800
#define DAT_40004800             REG32(0x40004800UL)
#endif
#ifndef DAT_40010410
#define DAT_40010410             REG32(0x40010410UL)
#endif
#ifndef DAT_40010800
#define DAT_40010800             REG32(0x40010800UL)
#endif
#ifndef DAT_40010804
#define DAT_40010804             REG32(0x40010804UL)
#endif
#ifndef DAT_40010c00
#define DAT_40010c00             REG32(0x40010C00UL)
#endif
#ifndef DAT_40010c04
#define DAT_40010c04             REG32(0x40010C04UL)
#endif
#ifndef DAT_40011000
#define DAT_40011000             REG32(0x40011000UL)
#endif
#ifndef DAT_40011004
#define DAT_40011004             REG32(0x40011004UL)
#endif
#ifndef DAT_40011400
#define DAT_40011400             REG32(0x40011400UL)
#endif
#ifndef DAT_40011404
#define DAT_40011404             REG32(0x40011404UL)
#endif
#ifndef DAT_40011800
#define DAT_40011800             REG32(0x40011800UL)
#endif
#ifndef DAT_40011804
#define DAT_40011804             REG32(0x40011804UL)
#endif
#ifndef DAT_40011808
#define DAT_40011808             REG32(0x40011808UL)
#endif
#ifndef DAT_40012c00
#define DAT_40012c00             REG32(0x40012C00UL)
#endif
#ifndef DAT_40012c44
#define DAT_40012c44             REG32(0x40012C44UL)
#endif
#ifndef DAT_40013400
#define DAT_40013400             REG32(0x40013400UL)
#endif
#ifndef DAT_40013444
#define DAT_40013444             REG32(0x40013444UL)
#endif
#ifndef DAT_40013800
#define DAT_40013800             REG16(0x40013800UL)
#endif
#ifndef DAT_4002001c
#define DAT_4002001c             REG32(0x4002001CUL)
#endif
#ifndef DAT_40020030
#define DAT_40020030             REG32(0x40020030UL)
#endif
#ifndef DAT_40020044
#define DAT_40020044             REG32(0x40020044UL)
#endif
#ifndef DAT_40020058
#define DAT_40020058             REG32(0x40020058UL)
#endif
#ifndef DAT_4002006c
#define DAT_4002006c             REG32(0x4002006CUL)
#endif
#ifndef DAT_40020080
#define DAT_40020080             REG32(0x40020080UL)
#endif
#ifndef DAT_e000ed0c
#define DAT_e000ed0c             REG32(0xE000ED0CUL)
#endif
#ifndef DAT_e000ed23
#define DAT_e000ed23             REG32(0xE000ED23UL)
#endif
#ifndef _DAT_4221818c
#define _DAT_4221818c            REG32(0x4221818CUL)
#endif
#ifndef _DAT_42228180
#define _DAT_42228180            REG32(0x42228180UL)
#endif
#ifndef _DAT_42228184
#define _DAT_42228184            REG32(0x42228184UL)
#endif
#ifndef _DAT_42228188
#define _DAT_42228188            REG32(0x42228188UL)
#endif
#ifndef _DAT_4222818c
#define _DAT_4222818c            REG32(0x4222818CUL)
#endif
#ifndef _DAT_42228190
#define _DAT_42228190            REG32(0x42228190UL)
#endif
#ifndef _DAT_42228194
#define _DAT_42228194            REG32(0x42228194UL)
#endif
#ifndef _DAT_42228198
#define _DAT_42228198            REG32(0x42228198UL)
#endif
#ifndef _DAT_4222819c
#define _DAT_4222819c            REG32(0x4222819CUL)
#endif

/* ============================================================================
 * 6. FIRMWARE SUBSYSTEM FUNCTION PROTOTYPES (with Flash Addresses)
 * ============================================================================ */

/* --- 6.1 Startup, C Runtime & System Initialization --- */
void Reset_Handler(void);                                           /* 0x08000148 */
void __main(void);                                                  /* 0x08000130 */
void __scatterload(void);                                           /* 0x08000AA0 */
void __decompress1(const uint8_t *src, uint8_t *dst, uint32_t size);/* 0x08000AC4 */
void __scatterload_copy(const uint8_t *src, uint8_t *dst, uint32_t size); /* 0x08042C18 */
void __scatterload_null(void);                                      /* 0x08042C26 */
void __scatterload_zeroinit(const uint8_t *src, uint8_t *dst, uint32_t size); /* 0x08042C28 */
void SystemInit(void);                                              /* 0x0803EE2C */
int  main(void);                                                    /* 0x080433E4 */
undefined4 FUN_080433e4(void);                                      /* 0x080433E4 */

/* --- 6.2 Cortex-M3 Exception & Peripheral Interrupt Handlers --- */
void NMI_Handler(void);                                             /* 0x08016AB6 */
void HardFault_Handler(void);                                       /* 0x080153FE */
void MemManage_Handler(void);                                       /* 0x08016438 */
void BusFault_Handler(void);                                        /* 0x08000E0C */
void UsageFault_Handler(void);                                      /* 0x080422C4 */
void SVC_Handler(void);                                             /* 0x08037926 */
void DebugMon_Handler(void);                                        /* 0x080043B6 */
void PendSV_Handler(void);                                          /* 0x08021792 */
void SysTick_Handler(void);                                         /* 0x0803DFDE */
void Default_Handler(void);                                         /* 0x08000162 */
void EXTI0_IRQHandler(void);                                        /* 0x0800FB4E */
void EXTI1_IRQHandler(void);                                        /* 0x08010620 */
void EXTI2_IRQHandler(void);                                        /* 0x080108D0 */
void EXTI4_IRQHandler(void);                                        /* 0x08011EE2 */
void EXTI9_5_IRQHandler(void);                                      /* 0x0801223C */
void TIM1_UP_IRQHandler(void);                                      /* 0x0803F3FE */
void TIM2_IRQHandler(void);                                         /* 0x0803F58E */
void TIM3_IRQHandler(void);                                         /* 0x0803F680 */
void TIM4_IRQHandler(void);                                         /* 0x0803F6B2 */
void USART1_IRQHandler(void);                                       /* 0x0804031A */
void USART2_IRQHandler(void);                                       /* 0x08040ECE */
void USART3_IRQHandler(void);                                       /* 0x08041C66 */
void EXTI15_10_IRQHandler(void);                                    /* 0x0800FC34 */
void TIM8_UP_IRQHandler(void);                                      /* 0x0803FA26 */
void TIM5_IRQHandler(void);                                         /* 0x0803F6E4 */
void TIM6_IRQHandler(void);                                         /* 0x0803F7FC */
void TIM7_IRQHandler(void);                                         /* 0x0803F84A */

/* --- 6.3 Firmware Subsystem Functions Called by main() & Executive --- */
uint64_t   FUN_080004ea(uint32_t a_lo, uint32_t a_hi, uint32_t b_lo, uint32_t b_hi); /* 0x080004EA: __aeabi_dmul (IEEE-754 double multiply) */
uint64_t   FUN_080005ce(uint32_t a_lo, uint32_t a_hi, uint32_t b_lo, uint32_t b_hi); /* 0x080005CE: __aeabi_ddiv (IEEE-754 double divide) */
uint64_t   FUN_080006e0(int32_t val);                               /* 0x080006E0: __aeabi_i2d (int32 to double) */
uint16_t   FUN_0800072c(void);                                      /* 0x0800072C: __aeabi_d2iz / scale helper */
void       FUN_08000b1a(void);                                      /* 0x08000B1A: TIM2 100us & TIM3/TIM4 timer init */
void       FUN_08000b78(void);                                      /* 0x08000B78: High-speed counter & EXTI init */
uint16_t   FUN_08000dec(uint32_t bkp_reg_offset);                   /* 0x08000DEC: BKP_ReadBackupRegister */
void       FUN_08000dfc(uint32_t bkp_reg_offset, uint32_t val);     /* 0x08000DFC: BKP_WriteBackupRegister */
uint16_t   FUN_08000e0e(volatile void *buf, uintptr_t len);         /* 0x08000E0E: Modbus RTU CRC-16 calculator */
void       FUN_08003ea0(uint32_t ch, uint32_t val);                 /* 0x08003EA0: Update HSC/PTO channel 0 */
void       FUN_08003eb4(uint32_t ch, uint32_t val);                 /* 0x08003EB4: Update HSC/PTO channel 1 */
void       FUN_08003ec8(uint32_t mask);                             /* 0x08003EC8: Clear DMA interrupt flags */
void       FUN_08003ee2(volatile void *dma_ch, uint32_t enable);    /* 0x08003EE2: DMA_Cmd (enable/disable channel) */
void       FUN_08003ef8(void);                                      /* 0x08003EF8: USART1 DMA RX/TX buffer init */
void       FUN_08003fbe(void);                                      /* 0x08003FBE: USART2 DMA RX/TX buffer init */
void       FUN_08004084(void);                                      /* 0x08004084: USART3 DMA RX/TX buffer init */
void       FUN_0800428c(void);                                      /* 0x0800428C: SysTick 1ms & NVIC priority init */
void       FUN_080043b8(int32_t loops);                             /* 0x080043B8: Busy-wait delay loop */
uint64_t   FUN_080047ac(void);                                      /* 0x080047AC: Primary PPI / S7 protocol state machine */
uint32_t   FUN_08012aac(uint32_t cmd, uint32_t bits);               /* 0x08012AAC: Expansion module (EM) bus transfer (mode 1) */
uint32_t   FUN_08013346(uint32_t cmd, uint32_t bits);               /* 0x08013346: Expansion module (EM) bus transfer (mode 2) */
void       FUN_08013d96(void);                                      /* 0x08013D96: OB1 user program scan wrapper (calls 0x08021948) */
void       FUN_08014278(void);                                      /* 0x08014278: SPI2 / W5500 GPIO configuration */
uint32_t   FUN_08014520(volatile void *gpio_base);                  /* 0x08014520: GPIO_ReadInputData (port IDR) */
void       FUN_08014526(volatile void *gpio_base, uint32_t pins);   /* 0x08014526: GPIO_SetBits (port BSRR) */
void       FUN_0801452a(volatile void *gpio_base, uint32_t pins);   /* 0x0801452A: GPIO_ResetBits (port BRR) */
void       FUN_0801520e(void);                                      /* 0x0801520E: Refresh local digital outputs (Q0.0..Q2.7) */
void       FUN_08015400(void);                                      /* 0x08015400: NOP stub */
void       FUN_08015c1a(void);                                      /* 0x08015C1A: Local digital input GPIO init (I0.0..I2.7) */
uint32_t   FUN_0801643a(uint32_t ch);                               /* 0x0801643A: Read HSC / input status channel */
void       FUN_08016c18(void);                                      /* 0x08016C18: TIM6 init (Port 0 frame timer) */
void       FUN_08016c42(void);                                      /* 0x08016C42: TIM7 init (Port 1 frame timer) */
void       FUN_08017228(void);                                      /* 0x08017228: Local digital output & EM bus GPIO init */
uint8_t    FUN_08017450(uint32_t a1, uint32_t port_id, uint32_t a3,
                        volatile void *p_len, volatile void *p_state,
                        volatile void *p_idx, volatile void *tx_buf,
                        volatile void *rx_buf, uint32_t timeout,
                        volatile void *ctx);                        /* 0x08017450: UART Port 0/1/2 frame & PPI handler */
uint32_t   FUN_08021948(volatile void *pc_ptr, uint16_t start_pc,
                        uint16_t end_pc, volatile void *stack_base,
                        volatile void *sp_ptr, volatile void *acc_ptr,
                        volatile void *status_ptr, volatile void * idx_ptr,
                        volatile void *call_stk1, volatile void *call_stk2,
                        volatile void *call_stk3, volatile void *depth_ptr); /* 0x08021948: S7-200 Bytecode VM Executor */
void       FUN_08036bb8(void);                                      /* 0x08036BB8: RTC time-of-day read & BCD conversion */
bool       FUN_08036c70(uint32_t rcc_flag);                         /* 0x08036C70: RCC_GetFlagStatus */
void       FUN_08036cde(void);                                      /* 0x08036CDE: Set initial RTC calendar counter */
uint32_t   FUN_08036d76(void);                                      /* 0x08036D76: Read RTC 32-bit counter (RTC_GetCounter) */
void       FUN_08036dd2(void);                                      /* 0x08036DD2: RTC prescaler & clock config */
void       FUN_08036de2(void);                                      /* 0x08036DE2: Enable PWR/BKP clocks & LSE oscillator */
void       FUN_08036e52(void);                                      /* 0x08036E52: W5500 hardware reset pin toggle */
void       FUN_08036e86(void);                                      /* 0x08036E86: W5500 default network parameters init */
int32_t    FUN_08036f06(uint32_t ch);                               /* 0x08036F06: Check HSC / pulse catch channel state */
void       FUN_08036fb6(uint32_t port_id);                          /* 0x08036FB6: Start RS-485 DMA transmission on Port 0/1 */
void       FUN_08037468(volatile void *src_flash, volatile void *dst_ram, uint32_t len); /* 0x08037468: Copy Flash block to SRAM */
int32_t    FUN_08037494(void);                                      /* 0x08037494: Restore retentive V/M/T/C RAM from Flash */
void       FUN_08037746(void);                                      /* 0x08037746: Save retentive V/M/T/C RAM to Flash on power-loss */
void       FUN_0803bf24(void);                                      /* 0x0803BF24: Verify STM32 UID against Flash license constants */
void       FUN_0803ece0(void);                                      /* 0x0803ECE0: Compile & validate OB1 user program in Flash */
void       FUN_0803ed26(void);                                      /* 0x0803ED26: Compute & verify SDB block checksums */
void       FUN_0803f020(void);                                      /* 0x0803F020: Service W5500 Ethernet socket state machine */
void       FUN_0803fbee(volatile void *tim_base, uint32_t new_state);/* 0x0803FBEE: TIM_ClearITPendingBit */
void       FUN_0803fbf4(volatile void *tim_base, uint32_t new_state);/* 0x0803FBF4: TIM_Cmd (enable/disable timer) */
void       FUN_0803fc1e(volatile void *tim_base);                   /* 0x0803FC1E: TIM3/TIM4 timed interrupt interval setup */
void       FUN_0803ff08(volatile void *tim_base, ...);              /* 0x0803FF08: TIM_ITConfig */
void       FUN_08040228(void);                                      /* 0x08040228: Configure TIM6 timeout for Port 0 baudrate */
void       FUN_0804027e(void);                                      /* 0x0804027E: Configure TIM7 timeout for Port 1 baudrate */
void       FUN_080402d4(void);                                      /* 0x080402D4: USART1 GPIO & RS485 DE pin init */
uint64_t   FUN_08040e18(uint32_t baud, int32_t parity, int32_t bits, uint32_t stop); /* 0x08040E18: USART1 serial init */
void       FUN_08040e8e(void);                                      /* 0x08040E8E: USART2 GPIO & RS485 DE pin init */
uint64_t   FUN_08041b98(uint32_t baud, int32_t parity, int32_t bits, uint32_t stop); /* 0x08041B98: USART2 serial init */
void       FUN_08041c0e(void);                                      /* 0x08041C0E: USART3 GPIO init */
uint64_t   FUN_08041f44(uint32_t baud, int32_t parity, int32_t bits, uint32_t stop); /* 0x08041F44: USART3 serial init */
void       FUN_08041fbe(volatile void *usart_base, uint32_t dma_req, uint32_t state);/* 0x08041FBE: USART_DMACmd */
void       FUN_08042042(volatile void *usart_base, uint32_t it_id, uint32_t state);  /* 0x08042042: USART_ITConfig */
void       FUN_0804211c(void);                                      /* 0x0804211C: Reconfigure TIM3 for SMB34 timed interrupt 0 */
void       FUN_080421f0(void);                                      /* 0x080421F0: Reconfigure TIM4 for SMB35 timed interrupt 1 */
void       FUN_080422c6(void);                                      /* 0x080422C6: SPI2 master mode init */
uint16_t   FUN_08042c44(uint32_t sample);                           /* 0x08042C44: Filter analog potentiometer 0 ADC sample */
uint16_t   FUN_08042c82(uint32_t sample);                           /* 0x08042C82: Filter analog potentiometer 1 ADC sample */
uint16_t   FUN_08042cc0(void);                                      /* 0x08042CC0: Convert scaled pot value to 0..255 for SMB28/29 */

#ifdef __cplusplus
}
#endif

#endif /* S7_226_PLC_H */
