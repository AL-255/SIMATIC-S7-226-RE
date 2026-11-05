/**
 * @file bsp_peripherals.c
 * @brief STM32F10x / APM32E103 Board Support Package (BSP), Standard Peripheral
 *        Library (SPL) Drivers, Internal Flash Storage (OB1/DB1/SDB/Retentive),
 *        W5500 Ethernet SPI2 Driver, ADC1/DMA1/DAC Analog Subsystem, and RTC
 *        for the SIMATIC S7-226 (APM32E103ZET6) PLC Firmware.
 *
 * Reconstructed from firmware address ranges:
 *   - 0x0800016c .. 0x08000e0d : Runtime math/CRC16, ADC1 + DMA1 + BKP drivers
 *   - 0x08003e5c .. 0x080047ab : DAC, DMA1/DMA2, DB1 checksum, W5500 SPI2 socket driver
 *   - 0x08013c9c .. 0x08016c80 : FLASH, GPIO, EXTI, CAN1, NVIC, IWDG, PWR SPL drivers
 *   - 0x08036b20 .. 0x08037927 : RCC, RTC, SPI2, RS-485 TX trigger, Flash OB1/DB1/SDB/Retentive
 *   - 0x0803bb26               : SetSysClock (8 MHz HSE * 12 = 96 MHz PLL)
 *   - 0x0803ece0 .. 0x0803f1b5 : SysTick_Init, SDB checksum, SystemInit, W5500 net config
 *   - 0x0804007e .. 0x080401b3 : RTC BCD calendar to epoch seconds conversion
 *   - 0x080422c6 .. 0x080433e2 : SPI2 init, IEEE-754 soft-float wrappers, ADC filter, NTC lookup
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ==========================================================================
 * Hardware Register Definitions (STM32F103ZE / APM32E103ZE)
 * ========================================================================== */

#define MMIO8(addr)  (*(volatile uint8_t *)(uintptr_t)(addr))
#define MMIO16(addr) (*(volatile uint16_t *)(uintptr_t)(addr))
#define MMIO32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* Cortex-M3 Core Peripherals */
#define SYSTICK_CTRL MMIO32(0xE000E010U)
#define SYSTICK_LOAD MMIO32(0xE000E014U)
#define SYSTICK_VAL  MMIO32(0xE000E018U)
#define SCB_VTOR     MMIO32(0xE000ED08U)
#define SCB_AIRCR    MMIO32(0xE000ED0CU)
#define SCB_SHPR3_P15 MMIO8(0xE000ED23U)

/* RCC (0x40021000) */
typedef struct {
    volatile uint32_t CR;       /* 0x00 */
    volatile uint32_t CFGR;     /* 0x04 */
    volatile uint32_t CIR;      /* 0x08 */
    volatile uint32_t APB2RSTR; /* 0x0C */
    volatile uint32_t APB1RSTR; /* 0x10 */
    volatile uint32_t AHBENR;   /* 0x14 */
    volatile uint32_t APB2ENR;  /* 0x18 */
    volatile uint32_t APB1ENR;  /* 0x1C */
    volatile uint32_t BDCR;     /* 0x20 */
    volatile uint32_t CSR;      /* 0x24 */
} RCC_TypeDef;

#define RCC_BASE ((uintptr_t)0x40021000U)
#define RCC      ((RCC_TypeDef *)RCC_BASE)

/* Bit-band aliases in RCC BDCR (0x40021020) */
#define RCC_BDCR_RTCEN_BB MMIO32(0x4242043CU) /* Bit 15 */
#define RCC_BDCR_BDRST_BB MMIO32(0x42420440U) /* Bit 16 */

/* FLASH Controller (0x40022000) */
typedef struct {
    volatile uint32_t ACR;     /* 0x00 */
    volatile uint32_t KEYR;    /* 0x04 */
    volatile uint32_t OPTKEYR; /* 0x08 */
    volatile uint32_t SR;      /* 0x0C */
    volatile uint32_t CR;      /* 0x10 */
    volatile uint32_t AR;      /* 0x14 */
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;     /* 0x1C */
    volatile uint32_t WRPR;    /* 0x20 */
} FLASH_TypeDef;

#define FLASH_R_BASE ((uintptr_t)0x40022000U)
#define FLASH_R      ((FLASH_TypeDef *)FLASH_R_BASE)

#define FLASH_KEY1      0x45670123UL
#define FLASH_KEY2      0xCDEF89ABUL
#define FLASH_SR_BSY    0x00000001UL
#define FLASH_SR_PGERR  0x00000004UL
#define FLASH_SR_WRPRTERR 0x00000010UL
#define FLASH_SR_EOP    0x00000020UL
#define FLASH_CR_PG     0x00000001UL
#define FLASH_CR_PER    0x00000002UL
#define FLASH_CR_STRT   0x00000040UL
#define FLASH_CR_LOCK   0x00000080UL

typedef enum {
    FLASH_BUSY       = 1,
    FLASH_ERROR_PG   = 2,
    FLASH_ERROR_WRP  = 3,
    FLASH_COMPLETE   = 4,
    FLASH_TIMEOUT    = 5
} FLASH_Status;

/* GPIO (0x40010800 .. 0x40011C00) */
typedef struct {
    volatile uint32_t CRL;  /* 0x00 */
    volatile uint32_t CRH;  /* 0x04 */
    volatile uint32_t IDR;  /* 0x08 */
    volatile uint32_t ODR;  /* 0x0C */
    volatile uint32_t BSRR; /* 0x10 */
    volatile uint32_t BRR;  /* 0x14 */
    volatile uint32_t LCKR; /* 0x18 */
} GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef *)0x40010800U)
#define GPIOB ((GPIO_TypeDef *)0x40010C00U)
#define GPIOC ((GPIO_TypeDef *)0x40011000U)
#define GPIOD ((GPIO_TypeDef *)0x40011400U)
#define GPIOE ((GPIO_TypeDef *)0x40011800U)
#define GPIOF ((GPIO_TypeDef *)0x40011C00U)
#define GPIOG ((GPIO_TypeDef *)0x40012000U)

typedef struct {
    uint16_t GPIO_Pin;
    uint8_t  GPIO_Speed;
    uint8_t  GPIO_Mode;
} GPIO_InitTypeDef;

/* IWDG (0x40003000) */
#define IWDG_KR  MMIO32(0x40003000U)
#define IWDG_PR  MMIO32(0x40003004U)
#define IWDG_RLR MMIO32(0x40003008U)

/* RTC (0x40002800) */
typedef struct {
    volatile uint16_t CRH;  uint16_t RESERVED0;
    volatile uint16_t CRL;  uint16_t RESERVED1;
    volatile uint16_t PRLH; uint16_t RESERVED2;
    volatile uint16_t PRLL; uint16_t RESERVED3;
    volatile uint16_t DIVH; uint16_t RESERVED4;
    volatile uint16_t DIVL; uint16_t RESERVED5;
    volatile uint16_t CNTH; uint16_t RESERVED6;
    volatile uint16_t CNTL; uint16_t RESERVED7;
    volatile uint16_t ALRH; uint16_t RESERVED8;
    volatile uint16_t ALRL; uint16_t RESERVED9;
} RTC_TypeDef;

#define RTC ((RTC_TypeDef *)0x40002800U)

/* BKP (0x40006C00) & PWR (0x40007000) */
#define BKP_BASE      ((uintptr_t)0x40006C00U)
#define PWR_CR_DBP_BB MMIO32(0x420E0020U) /* PWR->CR bit 8 (DBP) */

/* ADC1 (0x40012400) */
typedef struct {
    volatile uint32_t SR;    /* 0x00 */
    volatile uint32_t CR1;   /* 0x04 */
    volatile uint32_t CR2;   /* 0x08 */
    volatile uint32_t SMPR1; /* 0x0C */
    volatile uint32_t SMPR2; /* 0x10 */
    volatile uint32_t JOFR1; /* 0x14 */
    volatile uint32_t JOFR2; /* 0x18 */
    volatile uint32_t JOFR3; /* 0x1C */
    volatile uint32_t JOFR4; /* 0x20 */
    volatile uint32_t HTR;   /* 0x24 */
    volatile uint32_t LTR;   /* 0x28 */
    volatile uint32_t SQR1;  /* 0x2C */
    volatile uint32_t SQR2;  /* 0x30 */
    volatile uint32_t SQR3;  /* 0x34 */
    volatile uint32_t JSQR;  /* 0x38 */
    volatile uint32_t JDR1;  /* 0x3C */
    volatile uint32_t JDR2;  /* 0x40 */
    volatile uint32_t JDR3;  /* 0x44 */
    volatile uint32_t JDR4;  /* 0x48 */
    volatile uint32_t DR;    /* 0x4C */
} ADC_TypeDef;

#define ADC1 ((ADC_TypeDef *)0x40012400U)

typedef struct {
    uint32_t ADC_Mode;
    uint8_t  ADC_ScanConvMode;
    uint8_t  ADC_ContinuousConvMode;
    uint8_t  PAD0[2];
    uint32_t ADC_ExternalTrigConv;
    uint32_t ADC_DataAlign;
    uint8_t  ADC_NbrOfChannel;
    uint8_t  PAD1[3];
} ADC_InitTypeDef;

/* DAC (0x40007400) */
typedef struct {
    volatile uint32_t CR;      /* 0x00 */
    volatile uint32_t SWTRIGR; /* 0x04 */
    volatile uint32_t DHR12R1; /* 0x08 */
    volatile uint32_t DHR12L1; /* 0x0C */
    volatile uint32_t DHR8R1;  /* 0x10 */
    volatile uint32_t DHR12R2; /* 0x14 */
    volatile uint32_t DHR12L2; /* 0x18 */
    volatile uint32_t DHR8R2;  /* 0x1C */
    volatile uint32_t DHR12RD; /* 0x20 */
    volatile uint32_t DHR12LD; /* 0x24 */
    volatile uint32_t DHR8RD;  /* 0x28 */
    volatile uint32_t DOR1;    /* 0x2C */
    volatile uint32_t DOR2;    /* 0x30 */
} DAC_TypeDef;

#define DAC ((DAC_TypeDef *)0x40007400U)

typedef struct {
    uint32_t DAC_Trigger;
    uint32_t DAC_WaveGeneration;
    uint32_t DAC_LFSRUnmask_TriangleAmplitude;
    uint32_t DAC_OutputBuffer;
} DAC_InitTypeDef;

/* DMA1 (0x40020000) & DMA2 (0x40020400) */
typedef struct {
    volatile uint32_t CCR;   /* 0x00 */
    volatile uint32_t CNDTR; /* 0x04 */
    volatile uint32_t CPAR;  /* 0x08 */
    volatile uint32_t CMAR;  /* 0x0C */
} DMA_Channel_TypeDef;

typedef struct {
    volatile uint32_t ISR;  /* 0x00 */
    volatile uint32_t IFCR; /* 0x04 */
} DMA_TypeDef;

#define DMA1          ((DMA_TypeDef *)0x40020000U)
#define DMA2          ((DMA_TypeDef *)0x40020400U)
#define DMA1_Channel1 ((DMA_Channel_TypeDef *)0x40020008U)
#define DMA1_Channel2 ((DMA_Channel_TypeDef *)0x4002001CU)
#define DMA1_Channel3 ((DMA_Channel_TypeDef *)0x40020030U)
#define DMA1_Channel4 ((DMA_Channel_TypeDef *)0x40020044U)
#define DMA1_Channel5 ((DMA_Channel_TypeDef *)0x40020058U)
#define DMA1_Channel6 ((DMA_Channel_TypeDef *)0x4002006CU)
#define DMA1_Channel7 ((DMA_Channel_TypeDef *)0x40020080U)
#define DMA2_Channel1 ((DMA_Channel_TypeDef *)0x40020408U)
#define DMA2_Channel2 ((DMA_Channel_TypeDef *)0x4002041CU)
#define DMA2_Channel3 ((DMA_Channel_TypeDef *)0x40020430U)
#define DMA2_Channel4 ((DMA_Channel_TypeDef *)0x40020444U)
#define DMA2_Channel5 ((DMA_Channel_TypeDef *)0x40020458U)

typedef struct {
    uint32_t DMA_PeripheralBaseAddr;
    uint32_t DMA_MemoryBaseAddr;
    uint32_t DMA_DIR;
    uint32_t DMA_BufferSize;
    uint32_t DMA_PeripheralInc;
    uint32_t DMA_MemoryInc;
    uint32_t DMA_PeripheralDataSize;
    uint32_t DMA_MemoryDataSize;
    uint32_t DMA_Mode;
    uint32_t DMA_Priority;
    uint32_t DMA_M2M;
} DMA_InitTypeDef;

/* SPI2 (0x40003800) */
typedef struct {
    volatile uint16_t CR1;     uint16_t RESERVED0;
    volatile uint16_t CR2;     uint16_t RESERVED1;
    volatile uint16_t SR;      uint16_t RESERVED2;
    volatile uint16_t DR;      uint16_t RESERVED3;
    volatile uint16_t CRCPR;   uint16_t RESERVED4;
    volatile uint16_t RXCRCR;  uint16_t RESERVED5;
    volatile uint16_t TXCRCR;  uint16_t RESERVED6;
    volatile uint16_t I2SCFGR; uint16_t RESERVED7;
    volatile uint16_t I2SPR;   uint16_t RESERVED8;
} SPI_TypeDef;

#define SPI2 ((SPI_TypeDef *)0x40003800U)

typedef struct {
    uint16_t SPI_Direction;
    uint16_t SPI_Mode;
    uint16_t SPI_DataSize;
    uint16_t SPI_CPOL;
    uint16_t SPI_CPHA;
    uint16_t SPI_NSS;
    uint16_t SPI_BaudRatePrescaler;
    uint16_t SPI_FirstBit;
    uint16_t SPI_CRCPolynomial;
} SPI_InitTypeDef;

/* NVIC (0xE000E100) */
typedef struct {
    uint8_t NVIC_IRQChannel;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
    uint8_t NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

typedef struct {
    uint32_t RCC_SYSCLK_Frequency;
    uint32_t RCC_HCLK_Frequency;
    uint32_t RCC_PCLK1_Frequency;
    uint32_t RCC_PCLK2_Frequency;
    uint32_t RCC_ADCCLK_Frequency;
} RCC_ClocksTypeDef;

/* Internal Flash Layout Constants for SIMATIC S7-226 */
#define FLASH_DB1_PAGE0_ADDR   0x08064800UL
#define FLASH_DB1_BODY_ADDR    0x08065000UL
#define FLASH_DB1_PAGE1_ADDR   0x08068000UL
#define FLASH_DB1_CHKSUM_ADDR  0x08068526UL
#define FLASH_RETENTIVE_ADDR   0x08069000UL
#define FLASH_OB1_PAGE0_ADDR   0x0806D800UL
#define FLASH_OB1_PAGE1_ADDR   0x08075800UL
#define FLASH_SDB_PAGE_ADDR    0x0807F800UL

/* Firmware SRAM Globals Referenced by BSP & Flash Subsystems */
#define g_AHBPrescTable        ((const uint8_t *)0x20000000U)
#define g_ADCPrescTable        ((const uint8_t *)0x20000010U)
#define g_SystemCoreClock      MMIO32(0x20000014U) /* 96,000,000 Hz */
#define g_Port0_TxLen          MMIO16(0x200000BCU)
#define g_Port1_TxLen          MMIO16(0x200000C4U)
#define g_EthDynamicCfgMode    MMIO8(0x20000140U)
#define g_Port0_TurnaroundCnt  MMIO16(0x200001D6U)
#define g_Port1_TurnaroundCnt  MMIO16(0x200001DEU)
#define g_SDB_Checksum0        MMIO8(0x2000021CU)
#define g_SDB_Checksum1        MMIO8(0x2000021DU)
#define g_SDB_Checksum2        MMIO8(0x2000021EU)
#define g_SDB_Checksum3        MMIO8(0x2000021FU)
#define g_EthStaticMode        MMIO8(0x20000392U)
#define g_EthCustomPortFlag    MMIO8(0x20000393U)
#define g_EthConfigBytes       ((volatile uint8_t *)0x20000396U)
#define g_DB1_ChecksumLo       MMIO8(0x200003B6U)
#define g_DB1_ChecksumHi       MMIO8(0x200003B7U)
#define g_RTC_LseTimeoutCnt    MMIO16(0x20000480U)
#define g_RTC_BcdCalendar      ((volatile uint8_t *)0x20000498U)
#define g_FlashDB1WritePtr     MMIO32(0x200004ECU)
#define g_EthCustomPorts       ((volatile uint16_t *)0x20000B0CU)
#define g_PLC_M_Memory         ((volatile uint8_t *)0x20001000U)
#define g_PLC_V_Header         ((volatile uint8_t *)0x20001068U)
#define g_PLC_V_Memory         ((volatile uint8_t *)0x2000108CU)
#define g_RetentiveMagicLo     MMIO8(0x20003892U)
#define g_RetentiveMagicHi     MMIO8(0x20003893U)
#define g_RetentiveMagicWord   MMIO16(0x20003892U)
#define g_PLC_TimerCurrVal     ((volatile uint16_t *)0x20004000U)
#define g_PLC_TimerStatus      ((volatile uint8_t *)0x2000422CU)
#define g_PLC_CounterCurrVal   ((volatile uint16_t *)0x20004440U)
#define g_PLC_CounterStatus    ((volatile uint8_t *)0x20005000U)
#define g_PLC_SDB_Buffer       ((volatile uint8_t *)0x200058CCU)
#define g_ADC_DMA_Buffer       ((volatile uint16_t *)0x200061E2U)

/* Forward declarations */
void SetSysClock(void);
void GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_InitTypeDef *GPIO_InitStruct);
void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void NVIC_Init(const NVIC_InitTypeDef *NVIC_InitStruct);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, uint8_t NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, uint8_t NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, uint8_t NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, uint8_t NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, uint8_t NewState);
void DMA_DeInit(DMA_Channel_TypeDef *DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef *DMAy_Channelx, const DMA_InitTypeDef *DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef *DMAy_Channelx, uint8_t NewState);

/* ==========================================================================
 * System Clock & Startup Initialization (0x0803bb26, 0x0803ece0, 0x0803ee2c)
 * ========================================================================== */

/**
 * @brief Configures PLL to 96 MHz from 8 MHz HSE crystal (HSE * 12 = 96 MHz).
 *        Reconstructed from FUN_0803bb26 @ 0x0803bb26.
 *
 * Clock Tree:
 *   - HSE    = 8 MHz external crystal
 *   - SYSCLK = 96 MHz (PLL = HSE * 12, APM32E103 / overclocked STM32F103)
 *   - HCLK   = 96 MHz (AHB DIV1)
 *   - PCLK2  = 96 MHz (APB2 DIV1)
 *   - PCLK1  = 48 MHz (APB1 DIV2)
 */
void SetSysClock(void)
{
    uint32_t startup_counter = 0;
    uint32_t hse_status = 0;

    /* Enable HSE (RCC_CR_HSEON = 0x00010000) */
    RCC->CR |= 0x00010000UL;

    /* Wait until HSE is ready or timeout (0x0500 iterations) */
    do {
        hse_status = RCC->CR & 0x00020000UL;
        startup_counter++;
    } while ((hse_status == 0) && (startup_counter != 0x0500UL));

    if ((RCC->CR & 0x00020000UL) != 0) {
        /* Enable Prefetch Buffer (bit 4) and 2 Wait States (latency = 3 mask / 2 WS) */
        FLASH_R->ACR |= 0x00000010UL;
        FLASH_R->ACR = (FLASH_R->ACR & 0xFFFFFFF8UL) | 0x00000002UL;

        /* HCLK = SYSCLK (DIV1), PCLK2 = HCLK (DIV1), PCLK1 = HCLK / 2 (0x00000400) */
        RCC->CFGR |= 0x00000000UL;
        RCC->CFGR |= 0x00000000UL;
        RCC->CFGR |= 0x00000400UL;

        /* PLL configuration: PLLSRC = HSE (bit 16), PLLMUL = 12 (0x00280000) -> 0x00290000 */
        RCC->CFGR = (RCC->CFGR & 0xFFC0FFFFUL) | 0x00290000UL;

        /* Enable PLL (RCC_CR_PLLON = 0x01000000) */
        RCC->CR |= 0x01000000UL;

        /* Wait until PLL is ready (RCC_CR_PLLRDY = 0x02000000) */
        while ((RCC->CR & 0x02000000UL) == 0) {
        }

        /* Select PLL as system clock source (SW = 0x02) */
        RCC->CFGR = (RCC->CFGR & 0xFFFFFFFCUL) | 0x00000002UL;

        /* Wait until PLL is used as system clock source (SWS == 0x08) */
        while ((RCC->CFGR & 0x0000000CUL) != 0x00000008UL) {
        }
    } else {
        /* HSE failed to start-up: trap in infinite loop */
        while (1) {
        }
    }
}

/**
 * @brief CMSIS SystemInit entry point. Resets RCC, configures 96 MHz PLL,
 *        and sets Vector Table Offset Register to 0x08000000.
 *        Reconstructed from 0x0803ee2c / FUN_0803ee30 @ 0x0803ee30.
 */
void SystemInit(void)
{
    /* Set HSION bit */
    RCC->CR |= 0x00000001UL;

    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
    RCC->CFGR &= 0xF8FF0000UL;

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= 0xFEF6FFFFUL;

    /* Reset HSEBYP bit */
    RCC->CR &= 0xFFFBFFFFUL;

    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    RCC->CFGR &= 0xFF80FFFFUL;

    /* Disable all RCC interrupts and clear pending bits */
    RCC->CIR = 0x009F0000UL;

    /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
    SetSysClock();

    /* Vector Table Relocation in Internal FLASH */
    SCB_VTOR = 0x08000000UL;
}

/**
 * @brief Configures the Cortex-M3 SysTick timer for a 1 ms (1000 Hz) tick.
 *        Reconstructed from FUN_0803ece0 @ 0x0803ece0.
 */
void SysTick_Configuration(void)
{
    uint32_t ticks = g_SystemCoreClock / 1000UL;
    if (ticks < 0x01000000UL) {
        SYSTICK_LOAD   = ticks - 1UL;
        SCB_SHPR3_P15  = 0xF0U; /* Lowest preemption priority for SysTick */
        SYSTICK_VAL    = 0UL;
        SYSTICK_CTRL   = 0x07UL; /* CLKSOURCE | TICKINT | ENABLE */
        SYSTICK_CTRL  |= 0x01UL;
        return;
    }
    while (1) {
    }
}

/* ==========================================================================
 * RCC Standard Peripheral Library Drivers (0x08036b20 .. 0x08036cce)
 * ========================================================================== */

/**
 * @brief Configures the ADC clock (ADCCLK = PCLK2 / 8).
 *        Reconstructed from FUN_08036b20 @ 0x08036b20.
 */
void RCC_ADCCLKConfig(void)
{
    RCC->CFGR = (RCC->CFGR & 0xFFFF3FFFUL) | 0x0000C000UL;
}

/**
 * @brief Enables or disables the AHB peripheral clock.
 *        Reconstructed from FUN_08036b34 @ 0x08036b34.
 */
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, uint8_t NewState)
{
    if (NewState != 0) {
        RCC->AHBENR |= RCC_AHBPeriph;
    } else {
        RCC->AHBENR &= ~RCC_AHBPeriph;
    }
}

/**
 * @brief Enables or disables the Low Speed APB (APB1) peripheral clock.
 *        Reconstructed from FUN_08036b4c @ 0x08036b4c.
 */
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, uint8_t NewState)
{
    if (NewState != 0) {
        RCC->APB1ENR |= RCC_APB1Periph;
    } else {
        RCC->APB1ENR &= ~RCC_APB1Periph;
    }
}

/**
 * @brief Forces or releases Low Speed APB (APB1) peripheral reset.
 *        Reconstructed from FUN_08036b64 @ 0x08036b64.
 */
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, uint8_t NewState)
{
    if (NewState != 0) {
        RCC->APB1RSTR |= RCC_APB1Periph;
    } else {
        RCC->APB1RSTR &= ~RCC_APB1Periph;
    }
}

/**
 * @brief Enables or disables the High Speed APB (APB2) peripheral clock.
 *        Reconstructed from FUN_08036b7c @ 0x08036b7c.
 */
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, uint8_t NewState)
{
    if (NewState != 0) {
        RCC->APB2ENR |= RCC_APB2Periph;
    } else {
        RCC->APB2ENR &= ~RCC_APB2Periph;
    }
}

/**
 * @brief Forces or releases High Speed APB (APB2) peripheral reset.
 *        Reconstructed from FUN_08036b94 @ 0x08036b94.
 */
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, uint8_t NewState)
{
    if (NewState != 0) {
        RCC->APB2RSTR |= RCC_APB2Periph;
    } else {
        RCC->APB2RSTR &= ~RCC_APB2Periph;
    }
}

/**
 * @brief Forces or releases the Backup domain reset.
 *        Reconstructed from FUN_08036bac @ 0x08036bac.
 */
void RCC_BackupResetCmd(uint32_t NewState)
{
    RCC_BDCR_BDRST_BB = NewState;
}

/**
 * @brief Clears the RCC reset flags (RMVF bit 24 in RCC->CSR).
 *        Reconstructed from FUN_08036bb8 @ 0x08036bb8.
 */
void RCC_ClearFlag(void)
{
    RCC->CSR |= 0x01000000UL;
}

/**
 * @brief Computes the frequencies of different on-chip clocks (SYSCLK, HCLK,
 *        PCLK1, PCLK2, ADCCLK).
 *        Reconstructed from FUN_08036bca @ 0x08036bca.
 */
void RCC_GetClocksFreq(RCC_ClocksTypeDef *RCC_Clocks)
{
    uint32_t tmp = RCC->CFGR & 0x0000000CUL;
    uint32_t pllmull;
    uint32_t presc;

    if (tmp == 0x04UL) {
        /* HSE used as system clock */
        RCC_Clocks->RCC_SYSCLK_Frequency = 8000000UL;
    } else if (tmp == 0x08UL) {
        /* PLL used as system clock */
        pllmull = ((RCC->CFGR & 0x003C0000UL) >> 18) + 2UL;
        if ((RCC->CFGR & 0x00010000UL) == 0) {
            /* HSI oscillator clock divided by 2 selected as PLL clock entry */
            RCC_Clocks->RCC_SYSCLK_Frequency = pllmull << 22;
        } else {
            /* HSE selected as PLL clock entry */
            if ((RCC->CFGR & 0x00020000UL) == 0) {
                RCC_Clocks->RCC_SYSCLK_Frequency = pllmull * 8000000UL;
            } else {
                RCC_Clocks->RCC_SYSCLK_Frequency = pllmull * 4000000UL;
            }
        }
    } else {
        /* HSI used as system clock */
        RCC_Clocks->RCC_SYSCLK_Frequency = 8000000UL;
    }

    /* Compute HCLK, PCLK1, PCLK2, and ADCCLK clocks frequencies */
    presc = g_AHBPrescTable[(RCC->CFGR & 0x000000F0UL) >> 4];
    RCC_Clocks->RCC_HCLK_Frequency = RCC_Clocks->RCC_SYSCLK_Frequency >> presc;

    presc = g_AHBPrescTable[(RCC->CFGR & 0x00000700UL) >> 8];
    RCC_Clocks->RCC_PCLK1_Frequency = RCC_Clocks->RCC_HCLK_Frequency >> presc;

    presc = g_AHBPrescTable[(RCC->CFGR & 0x00003800UL) >> 11];
    RCC_Clocks->RCC_PCLK2_Frequency = RCC_Clocks->RCC_HCLK_Frequency >> presc;

    presc = g_ADCPrescTable[(RCC->CFGR & 0x0000C000UL) >> 14];
    RCC_Clocks->RCC_ADCCLK_Frequency = RCC_Clocks->RCC_PCLK2_Frequency / presc;
}

/**
 * @brief Checks whether the specified RCC flag is set or not.
 *        Reconstructed from FUN_08036c70 @ 0x08036c70.
 */
uint32_t RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
    uint32_t reg_idx = RCC_FLAG >> 5;
    uint32_t statusreg;

    if (reg_idx == 1U) {
        statusreg = RCC->CR;
    } else if (reg_idx == 2U) {
        statusreg = RCC->BDCR;
    } else {
        statusreg = RCC->CSR;
    }

    return (statusreg >> (RCC_FLAG & 0x1FU)) & 1UL;
}

/**
 * @brief Configures the External Low Speed oscillator (LSE).
 *        Reconstructed from FUN_08036ca0 @ 0x08036ca0.
 */
void RCC_LSEConfig(uint8_t RCC_LSE)
{
    MMIO8(0x40021020U) = 0x00U;
    MMIO8(0x40021020U) = 0x00U;
    if (RCC_LSE == 0x01U) {
        MMIO8(0x40021020U) = 0x01U; /* RCC_LSE_ON */
    } else if (RCC_LSE == 0x04U) {
        MMIO8(0x40021020U) = 0x05U; /* RCC_LSE_Bypass */
    }
}

/**
 * @brief Enables or disables the RTC clock.
 *        Reconstructed from FUN_08036cc2 @ 0x08036cc2.
 */
void RCC_RTCCLKCmd(uint32_t NewState)
{
    RCC_BDCR_RTCEN_BB = NewState;
}

/**
 * @brief Configures the RTC clock source (LSE = 0x0100).
 *        Reconstructed from FUN_08036cce @ 0x08036cce.
 */
void RCC_RTCCLKConfig(void)
{
    RCC->BDCR |= 0x00000100UL;
}

/* ==========================================================================
 * BKP, PWR, IWDG, and RTC Drivers (0x08000dda..0x08000dfc, 0x08036cde..0x08036df2, 0x0804007e)
 * ========================================================================== */

/**
 * @brief Enables or disables access to the RTC and backup registers.
 *        Reconstructed from FUN_08021788 @ 0x08021788.
 */
void PWR_BackupAccessCmd(uint32_t NewState)
{
    PWR_CR_DBP_BB = NewState;
}

/**
 * @brief Deinitializes the BKP peripheral registers to their default reset values.
 *        Reconstructed from FUN_08000dda @ 0x08000dda.
 */
void BKP_DeInit(void)
{
    RCC_BackupResetCmd(1);
    RCC_BackupResetCmd(0);
}

/**
 * @brief Reads data from the specified Data Backup Register.
 *        Reconstructed from FUN_08000dec @ 0x08000dec.
 */
uint16_t BKP_ReadBackupRegister(uint16_t BKP_DR)
{
    volatile uint32_t addr = BKP_BASE + BKP_DR;
    return MMIO16(addr);
}

/**
 * @brief Writes user data to the specified Data Backup Register.
 *        Reconstructed from FUN_08000dfc @ 0x08000dfc.
 */
void BKP_WriteBackupRegister(uint16_t BKP_DR, uint16_t Data)
{
    volatile uint32_t addr = BKP_BASE + BKP_DR;
    MMIO32(addr) = Data;
}

/**
 * @brief Enters the RTC configuration mode.
 *        Reconstructed from FUN_08036d52 @ 0x08036d52.
 */
void RTC_EnterConfigMode(void)
{
    RTC->CRL |= 0x0010U;
}

/**
 * @brief Exits from the RTC configuration mode.
 *        Reconstructed from FUN_08036d64 @ 0x08036d64.
 */
void RTC_ExitConfigMode(void)
{
    RTC->CRL &= (uint16_t)~0x0010U;
}

/**
 * @brief Gets the RTC counter value.
 *        Reconstructed from FUN_08036d76 @ 0x08036d76.
 */
uint32_t RTC_GetCounter(void)
{
    uint16_t cnth = RTC->CNTH;
    return ((uint32_t)cnth << 16) | (uint32_t)RTC->CNTL;
}

/**
 * @brief Sets the RTC counter value.
 *        Reconstructed from FUN_08036d90 @ 0x08036d90.
 */
void RTC_SetCounter(uint32_t CounterValue)
{
    RTC_EnterConfigMode();
    RTC->CNTH = (uint16_t)(CounterValue >> 16);
    RTC->CNTL = (uint16_t)(CounterValue & 0xFFFFU);
    RTC_ExitConfigMode();
}

/**
 * @brief Sets the RTC prescaler value.
 *        Reconstructed from FUN_08036db0 @ 0x08036db0.
 */
void RTC_SetPrescaler(uint32_t PrescalerValue)
{
    RTC_EnterConfigMode();
    RTC->PRLH = (uint16_t)((PrescalerValue & 0x000F0000UL) >> 16);
    RTC->PRLL = (uint16_t)(PrescalerValue & 0x0000FFFFUL);
    RTC_ExitConfigMode();
}

/**
 * @brief Waits until last write operation on RTC registers has finished.
 *        Reconstructed from FUN_08036dd2 @ 0x08036dd2.
 */
void RTC_WaitForLastTask(void)
{
    while ((RTC->CRL & 0x0020U) == 0) {
    }
}

/**
 * @brief Waits until the RTC registers are synchronized with RTC APB clock.
 *        Reconstructed from FUN_08036de2 @ 0x08036de2.
 */
void RTC_WaitForSynchro(void)
{
    RTC->CRL &= (uint16_t)~0x0008U;
    while ((RTC->CRL & 0x0008U) == 0) {
    }
}

/**
 * @brief Configures the RTC with 32.768 kHz LSE crystal for a 1 Hz second counter.
 *        Reconstructed from FUN_08036cde @ 0x08036cde.
 */
void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks on APB1 (0x18000000) */
    RCC_APB1PeriphClockCmd(0x18000000UL, 1);
    PWR_BackupAccessCmd(1);
    BKP_DeInit();

    /* Enable LSE (0x01) */
    RCC_LSEConfig(0x01U);
    while ((RCC_GetFlagStatus(0x41U) == 0) && (g_RTC_LseTimeoutCnt <= 250U)) {
    }

    RCC_RTCCLKConfig();
    RCC_RTCCLKCmd(1);
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    RTC_WaitForLastTask();

    /* Set RTC prescaler: 32768 Hz / (32767 + 1) = 1 Hz */
    RTC_SetPrescaler(32767UL);
    RTC_WaitForLastTask();
}

/**
 * @brief Converts the S7-200 TOD / READ_RTC BCD date-time buffer at 0x20000498
 *        [YY, MM, DD, HH, mm, ss] into seconds since 2000-01-01 00:00:00 and
 *        updates the hardware RTC counter.
 *        Reconstructed from FUN_0804007e @ 0x0804007e.
 */
void RTC_SetCalendarFromBCD(void)
{
    static const uint8_t kMonthDays[12] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    uint16_t year   = (uint16_t)(g_RTC_BcdCalendar[0] & 0x0FU) +
                      (uint16_t)(g_RTC_BcdCalendar[0] >> 4) * 10U + 2000U;
    uint8_t  month  = (uint8_t)((g_RTC_BcdCalendar[1] & 0x0FU) + (g_RTC_BcdCalendar[1] >> 4) * 10U);
    uint8_t  day    = (uint8_t)((g_RTC_BcdCalendar[2] & 0x0FU) + (g_RTC_BcdCalendar[2] >> 4) * 10U);
    uint8_t  hour   = (uint8_t)((g_RTC_BcdCalendar[3] & 0x0FU) + (g_RTC_BcdCalendar[3] >> 4) * 10U);
    uint8_t  minute = (uint8_t)((g_RTC_BcdCalendar[4] & 0x0FU) + (g_RTC_BcdCalendar[4] >> 4) * 10U);
    uint8_t  second = (uint8_t)((g_RTC_BcdCalendar[5] & 0x0FU) + (g_RTC_BcdCalendar[5] >> 4) * 10U);

    if (year < 2000U || year >= 2100U) {
        return;
    }

    uint32_t total_seconds = 0;
    for (uint16_t y = 2000U; y < year; y++) {
        bool is_leap = ((y & 3U) == 0 && (y % 100U) != 0) || ((y % 400U) == 0);
        total_seconds += is_leap ? 31622400UL : 31536000UL;
    }

    for (uint8_t m = 0; m < (uint8_t)(month - 1U) && m < 12U; m++) {
        total_seconds += (uint32_t)kMonthDays[m] * 86400UL;
        bool is_leap = ((year & 3U) == 0 && (year % 100U) != 0) || ((year % 400U) == 0);
        if (is_leap && m == 1U) {
            total_seconds += 86400UL;
        }
    }

    total_seconds += (uint32_t)(day - 1U) * 86400UL +
                     (uint32_t)hour * 3600UL +
                     (uint32_t)minute * 60UL +
                     (uint32_t)second;

    RCC_APB1PeriphClockCmd(0x18000000UL, 1);
    PWR_BackupAccessCmd(1);
    RTC_SetCounter(total_seconds);
    RTC_WaitForLastTask();
}

/* ==========================================================================
 * GPIO & NVIC SPL Drivers (0x080143dc .. 0x08016866)
 * ========================================================================== */

/**
 * @brief Initializes the GPIOx peripheral according to the specified parameters.
 *        Reconstructed from FUN_080143dc @ 0x080143dc.
 */
void GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_InitTypeDef *GPIO_InitStruct)
{
    uint32_t currentmode = (uint32_t)GPIO_InitStruct->GPIO_Mode & 0x0FUL;
    uint32_t pinmask     = (uint32_t)GPIO_InitStruct->GPIO_Pin;
    uint32_t tmpreg;

    if ((GPIO_InitStruct->GPIO_Mode & 0x10U) != 0) {
        currentmode |= (uint32_t)GPIO_InitStruct->GPIO_Speed;
    }

    /* Configure the eight low port pins (CRL) */
    if ((pinmask & 0x00FFUL) != 0) {
        tmpreg = GPIOx->CRL;
        for (uint32_t pinpos = 0; pinpos < 8U; pinpos++) {
            uint32_t pos = 1UL << pinpos;
            if ((pinmask & pos) == pos) {
                uint32_t shift = pinpos << 2;
                tmpreg &= ~(0x0FUL << shift);
                tmpreg |= (currentmode << shift);
                if (GPIO_InitStruct->GPIO_Mode == 0x28U) {
                    GPIOx->BRR = pos;
                } else if (GPIO_InitStruct->GPIO_Mode == 0x48U) {
                    GPIOx->BSRR = pos;
                }
            }
        }
        GPIOx->CRL = tmpreg;
    }

    /* Configure the eight high port pins (CRH) */
    if (pinmask > 0x00FFUL) {
        tmpreg = GPIOx->CRH;
        for (uint32_t pinpos = 0; pinpos < 8U; pinpos++) {
            uint32_t pos = 1UL << (pinpos + 8U);
            if ((pinmask & pos) == pos) {
                uint32_t shift = pinpos << 2;
                tmpreg &= ~(0x0FUL << shift);
                tmpreg |= (currentmode << shift);
                if (GPIO_InitStruct->GPIO_Mode == 0x28U) {
                    GPIOx->BRR = pos;
                } else if (GPIO_InitStruct->GPIO_Mode == 0x48U) {
                    GPIOx->BSRR = pos;
                }
            }
        }
        GPIOx->CRH = tmpreg;
    }
}

/**
 * @brief Reads the specified input port pin.
 *        Reconstructed from FUN_08014518 @ 0x08014518.
 */
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    return ((GPIOx->IDR & GPIO_Pin) != 0) ? 1U : 0U;
}

/**
 * @brief Clears the selected data port bits.
 *        Reconstructed from FUN_0801452a @ 0x0801452a.
 */
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BRR = GPIO_Pin;
}

/**
 * @brief Sets the selected data port bits.
 *        Reconstructed from FUN_0801452e @ 0x0801452e.
 */
void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BSRR = GPIO_Pin;
}

/**
 * @brief Initializes the NVIC peripheral according to the specified parameters.
 *        Reconstructed from FUN_080167b2 @ 0x080167b2.
 */
void NVIC_Init(const NVIC_InitTypeDef *NVIC_InitStruct)
{
    if (NVIC_InitStruct->NVIC_IRQChannelCmd != 0) {
        uint32_t prioritygroup = (0x0700UL - (SCB_AIRCR & 0x0700UL)) >> 8;
        uint32_t tmppriority   = ((uint32_t)NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << (0x04UL - prioritygroup)) |
                                 ((uint32_t)NVIC_InitStruct->NVIC_IRQChannelSubPriority & (0x0FUL >> (0x04UL - prioritygroup)));
        uint8_t  ch            = NVIC_InitStruct->NVIC_IRQChannel;
        MMIO8(0xE000E400UL + ch) = (uint8_t)(tmppriority << 4);
        MMIO32(0xE000E100UL + ((ch >> 5) << 2)) = 1UL << (ch & 0x1FU);
    } else {
        uint8_t ch = NVIC_InitStruct->NVIC_IRQChannel;
        MMIO32(0xE000E180UL + ((ch >> 5) << 2)) = 1UL << (ch & 0x1FU);
    }
}

/**
 * @brief Configures the NVIC priority grouping.
 *        Reconstructed from FUN_08016866 @ 0x08016866.
 */
void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup)
{
    SCB_AIRCR = 0x05FA0000UL | NVIC_PriorityGroup;
}

/* ==========================================================================
 * ADC1 & DMA1 Analog Input Subsystem (0x08000b1a .. 0x08000dbe)
 * ========================================================================== */

/**
 * @brief Enables or disables the specified ADC peripheral.
 *        Reconstructed from FUN_08000c96 @ 0x08000c96.
 */
void ADC_Cmd(ADC_TypeDef *ADCx, uint8_t NewState)
{
    if (NewState != 0) {
        ADCx->CR2 |= 0x00000001UL;
    } else {
        ADCx->CR2 &= ~0x00000001UL;
    }
}

/**
 * @brief Enables or disables the specified ADC DMA request.
 *        Reconstructed from FUN_08000cb2 @ 0x08000cb2.
 */
void ADC_DMACmd(ADC_TypeDef *ADCx, uint8_t NewState)
{
    if (NewState != 0) {
        ADCx->CR2 |= 0x00000100UL;
    } else {
        ADCx->CR2 &= ~0x00000100UL;
    }
}

/**
 * @brief Gets the selected ADC calibration status.
 *        Reconstructed from FUN_08000cd0 @ 0x08000cd0.
 */
uint8_t ADC_GetCalibrationStatus(ADC_TypeDef *ADCx)
{
    return ((ADCx->CR2 & 0x00000004UL) != 0) ? 1U : 0U;
}

/**
 * @brief Gets the selected ADC reset calibration registers status.
 *        Reconstructed from FUN_08000cdc @ 0x08000cdc.
 */
uint8_t ADC_GetResetCalibrationStatus(ADC_TypeDef *ADCx)
{
    return ((ADCx->CR2 & 0x00000008UL) != 0) ? 1U : 0U;
}

/**
 * @brief Initializes the ADCx peripheral according to the specified parameters.
 *        Reconstructed from FUN_08000ce8 @ 0x08000ce8.
 */
void ADC_Init(ADC_TypeDef *ADCx, const ADC_InitTypeDef *ADC_InitStruct)
{
    ADCx->CR1  = (ADCx->CR1 & 0xFFF0FEFFUL) |
                 ADC_InitStruct->ADC_Mode |
                 ((uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8);
    ADCx->CR2  = (ADCx->CR2 & 0xFFF1F7FDUL) |
                 ADC_InitStruct->ADC_DataAlign |
                 ADC_InitStruct->ADC_ExternalTrigConv |
                 ((uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1);
    ADCx->SQR1 = (ADCx->SQR1 & 0xFF0FFFFFUL) |
                 ((uint32_t)(ADC_InitStruct->ADC_NbrOfChannel - 1U) << 20);
}

/**
 * @brief Configures for the selected ADC regular channel its corresponding
 *        rank in the sequencer and its sample time.
 *        Reconstructed from FUN_08000d1e @ 0x08000d1e.
 */
void ADC_RegularChannelConfig(ADC_TypeDef *ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
    if (ADC_Channel < 10U) {
        uint32_t shift = (uint32_t)ADC_Channel * 3U;
        ADCx->SMPR2 = (ADCx->SMPR2 & ~(0x07UL << shift)) | ((uint32_t)ADC_SampleTime << shift);
    } else {
        uint32_t shift = (uint32_t)(ADC_Channel - 10U) * 3U;
        ADCx->SMPR1 = (ADCx->SMPR1 & ~(0x07UL << shift)) | ((uint32_t)ADC_SampleTime << shift);
    }

    if (Rank < 7U) {
        uint32_t shift = (uint32_t)(Rank - 1U) * 5U;
        ADCx->SQR3 = (ADCx->SQR3 & ~(0x1FUL << shift)) | ((uint32_t)ADC_Channel << shift);
    } else if (Rank < 13U) {
        uint32_t shift = (uint32_t)(Rank - 7U) * 5U;
        ADCx->SQR2 = (ADCx->SQR2 & ~(0x1FUL << shift)) | ((uint32_t)ADC_Channel << shift);
    } else {
        uint32_t shift = (uint32_t)(Rank - 13U) * 5U;
        ADCx->SQR1 = (ADCx->SQR1 & ~(0x1FUL << shift)) | ((uint32_t)ADC_Channel << shift);
    }
}

/**
 * @brief Resets the selected ADC calibration registers.
 *        Reconstructed from FUN_08000d92 @ 0x08000d92.
 */
void ADC_ResetCalibration(ADC_TypeDef *ADCx)
{
    ADCx->CR2 |= 0x00000008UL;
}

/**
 * @brief Enables or disables the selected ADC software start conversion.
 *        Reconstructed from FUN_08000d9a @ 0x08000d9a.
 */
void ADC_SoftwareStartConvCmd(ADC_TypeDef *ADCx, uint8_t NewState)
{
    if (NewState != 0) {
        ADCx->CR2 |= 0x00500000UL;
    } else {
        ADCx->CR2 &= ~0x00500000UL;
    }
}

/**
 * @brief Starts the selected ADC calibration process.
 *        Reconstructed from FUN_08000db6 @ 0x08000db6.
 */
void ADC_StartCalibration(ADC_TypeDef *ADCx)
{
    ADCx->CR2 |= 0x00000004UL;
}

/**
 * @brief Enables or disables the temperature sensor and Vrefint channel.
 *        Reconstructed from FUN_08000dbe @ 0x08000dbe.
 */
void ADC_TempSensorVrefintCmd(uint8_t NewState)
{
    if (NewState != 0) {
        ADC1->CR2 |= 0x00800000UL;
    } else {
        ADC1->CR2 &= ~0x00800000UL;
    }
}

/**
 * @brief Configures analog input GPIO pins (PA0, PA1, PA6, PB0, PB1, PC4) and
 *        enables DMA1 and ADC1 clocks.
 *        Reconstructed from FUN_08000b1a @ 0x08000b1a.
 */
void ADC_GPIO_Configuration(void)
{
    GPIO_InitTypeDef gpio_init;

    /* Enable DMA1 clock on AHB (bit 0) and ADC1 + GPIOA/B/C clocks on APB2 (0x21C) */
    RCC_AHBPeriphClockCmd(0x00000001UL, 1);
    RCC_APB2PeriphClockCmd(0x0000021CUL, 1);

    gpio_init.GPIO_Mode  = 0x00U; /* GPIO_Mode_AIN */
    gpio_init.GPIO_Speed = 0x00U;

    /* PC4 (ADC_IN14) */
    gpio_init.GPIO_Pin = 0x0010U;
    GPIO_Init(GPIOC, &gpio_init);

    /* PA0, PA1, PA6 (ADC_IN0, ADC_IN1, ADC_IN6) */
    gpio_init.GPIO_Pin = 0x0043U;
    GPIO_Init(GPIOA, &gpio_init);

    /* PB0, PB1 (ADC_IN8, ADC_IN9) */
    gpio_init.GPIO_Pin = 0x0003U;
    GPIO_Init(GPIOB, &gpio_init);
}

/**
 * @brief Configures DMA1_Channel1 and ADC1 in 7-channel continuous scan mode
 *        transferring 32 samples into 0x200061E2.
 *        Reconstructed from FUN_08000b78 @ 0x08000b78.
 */
void ADC1_DMA_Configuration(void)
{
    DMA_InitTypeDef dma_init;
    ADC_InitTypeDef adc_init;

    DMA_DeInit(DMA1_Channel1);
    dma_init.DMA_PeripheralBaseAddr = 0x4001244CUL; /* &ADC1->DR */
    dma_init.DMA_MemoryBaseAddr     = 0x200061E2UL; /* g_ADC_DMA_Buffer */
    dma_init.DMA_DIR                = 0x00000000UL; /* PeripheralSRC */
    dma_init.DMA_BufferSize         = 32UL;
    dma_init.DMA_PeripheralInc      = 0x00000000UL;
    dma_init.DMA_MemoryInc          = 0x00000080UL; /* Enable */
    dma_init.DMA_PeripheralDataSize = 0x00000100UL; /* HalfWord */
    dma_init.DMA_MemoryDataSize     = 0x00000400UL; /* HalfWord */
    dma_init.DMA_Mode               = 0x00000020UL; /* Circular */
    dma_init.DMA_Priority           = 0x00002000UL; /* High */
    dma_init.DMA_M2M                = 0x00000000UL;
    DMA_Init(DMA1_Channel1, &dma_init);
    DMA_Cmd(DMA1_Channel1, 1);

    adc_init.ADC_Mode               = 0x00000000UL; /* Independent */
    adc_init.ADC_ScanConvMode       = 1U;
    adc_init.ADC_ContinuousConvMode = 1U;
    adc_init.ADC_ExternalTrigConv   = 0x000E0000UL; /* Trig_None */
    adc_init.ADC_DataAlign          = 0x00000000UL; /* Right */
    adc_init.ADC_NbrOfChannel       = 7U;
    ADC_Init(ADC1, &adc_init);

    RCC_ADCCLKConfig();

    /* Configure 7 regular channels with 55.5 cycles sample time (5) */
    ADC_RegularChannelConfig(ADC1, 6U,  1U, 5U);
    ADC_RegularChannelConfig(ADC1, 14U, 2U, 5U);
    ADC_RegularChannelConfig(ADC1, 8U,  3U, 5U);
    ADC_RegularChannelConfig(ADC1, 9U,  4U, 5U);
    ADC_RegularChannelConfig(ADC1, 16U, 5U, 5U); /* Internal Temp Sensor */
    ADC_RegularChannelConfig(ADC1, 0U,  6U, 5U);
    ADC_RegularChannelConfig(ADC1, 1U,  7U, 5U);

    ADC_TempSensorVrefintCmd(1);
    ADC_DMACmd(ADC1, 1);
    ADC_Cmd(ADC1, 1);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) != 0) {
    }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) != 0) {
    }

    ADC_SoftwareStartConvCmd(ADC1, 1);
}

/**
 * @brief Initializes the Analog Input subsystem (ADC1 + DMA1 + GPIO).
 *        Reconstructed from FUN_08000c86 @ 0x08000c86.
 */
void BSP_ADC1_Init(void)
{
    ADC_GPIO_Configuration();
    ADC1_DMA_Configuration();
}

/* ==========================================================================
 * DAC & DMA1/DMA2 Drivers (0x08003e5c .. 0x080042ff)
 * ========================================================================== */

/**
 * @brief Enables or disables the specified DAC channel.
 *        Reconstructed from FUN_08003e5c @ 0x08003e5c.
 */
void DAC_Cmd(uint32_t DAC_Channel, uint8_t NewState)
{
    if (NewState != 0) {
        DAC->CR |= (1UL << DAC_Channel);
    } else {
        DAC->CR &= ~(1UL << DAC_Channel);
    }
}

/**
 * @brief Initializes the DAC peripheral according to the specified parameters.
 *        Reconstructed from FUN_08003e7c @ 0x08003e7c.
 */
void DAC_Init(uint32_t DAC_Channel, const DAC_InitTypeDef *DAC_InitStruct)
{
    uint32_t tmpreg = DAC_InitStruct->DAC_Trigger |
                      DAC_InitStruct->DAC_WaveGeneration |
                      DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude |
                      DAC_InitStruct->DAC_OutputBuffer;
    DAC->CR = (DAC->CR & ~(0x00000FFEUL << DAC_Channel)) | (tmpreg << DAC_Channel);
}

/**
 * @brief Sets the specified data holding register value for DAC channel 1.
 *        Reconstructed from FUN_08003ea2 @ 0x08003ea2.
 */
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
{
    MMIO32(0x40007408UL + DAC_Align) = Data;
}

/**
 * @brief Sets the specified data holding register value for DAC channel 2.
 *        Reconstructed from FUN_08003eb4 @ 0x08003eb4.
 */
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data)
{
    MMIO32(0x40007414UL + DAC_Align) = Data;
}

/**
 * @brief Configures PA4 and PA5 as DAC analog outputs (Channel 1 and Channel 2).
 *        Reconstructed from FUN_0800428c @ 0x0800428c.
 */
void BSP_DAC_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    DAC_InitTypeDef  dac_init;

    RCC_APB2PeriphClockCmd(0x00000004UL, 1); /* GPIOA */
    RCC_APB1PeriphClockCmd(0x20000000UL, 1); /* DAC */

    gpio_init.GPIO_Pin   = 0x0030U; /* PA4, PA5 */
    gpio_init.GPIO_Mode  = 0x00U;   /* GPIO_Mode_AIN */
    gpio_init.GPIO_Speed = 0x03U;
    GPIO_Init(GPIOA, &gpio_init);

    dac_init.DAC_Trigger                      = 0x00000000UL;
    dac_init.DAC_WaveGeneration               = 0x00000000UL;
    dac_init.DAC_LFSRUnmask_TriangleAmplitude = 0x00000000UL;
    dac_init.DAC_OutputBuffer                 = 0x00000002UL; /* Buffer_Disable */

    DAC_Init(0x00UL, &dac_init);
    DAC_Init(0x10UL, &dac_init);
    DAC_Cmd(0x00UL, 1);
    DAC_Cmd(0x10UL, 1);
    DAC_SetChannel1Data(0x00UL, 0);
    DAC_SetChannel2Data(0x00UL, 0);
}

/**
 * @brief Clears the DMAy Channelx's pending flags.
 *        Reconstructed from FUN_08003ec8 @ 0x08003ec8.
 */
void DMA_ClearFlag(uint32_t DMAy_FLAG)
{
    if ((DMAy_FLAG & 0x10000000UL) != 0) {
        DMA2->IFCR = DMAy_FLAG;
    } else {
        DMA1->IFCR = DMAy_FLAG;
    }
}

/**
 * @brief Enables or disables the specified DMAy Channelx.
 *        Reconstructed from FUN_08003ee0 @ 0x08003ee0.
 */
void DMA_Cmd(DMA_Channel_TypeDef *DMAy_Channelx, uint8_t NewState)
{
    if (NewState != 0) {
        DMAy_Channelx->CCR |= 0x00000001UL;
    } else {
        DMAy_Channelx->CCR &= ~0x00000001UL;
    }
}

/**
 * @brief Deinitializes the DMAy Channelx registers to their default reset values.
 *        Reconstructed from FUN_08004148 @ 0x08004148.
 */
void DMA_DeInit(DMA_Channel_TypeDef *DMAy_Channelx)
{
    DMAy_Channelx->CCR  &= ~0x00000001UL;
    DMAy_Channelx->CCR   = 0;
    DMAy_Channelx->CNDTR = 0;
    DMAy_Channelx->CPAR  = 0;
    DMAy_Channelx->CMAR  = 0;

    if (DMAy_Channelx == DMA1_Channel1)      DMA1->IFCR |= 0x0000000FUL;
    else if (DMAy_Channelx == DMA1_Channel2) DMA1->IFCR |= 0x000000F0UL;
    else if (DMAy_Channelx == DMA1_Channel3) DMA1->IFCR |= 0x00000F00UL;
    else if (DMAy_Channelx == DMA1_Channel4) DMA1->IFCR |= 0x0000F000UL;
    else if (DMAy_Channelx == DMA1_Channel5) DMA1->IFCR |= 0x000F0000UL;
    else if (DMAy_Channelx == DMA1_Channel6) DMA1->IFCR |= 0x00F00000UL;
    else if (DMAy_Channelx == DMA1_Channel7) DMA1->IFCR |= 0x0F000000UL;
    else if (DMAy_Channelx == DMA2_Channel1) DMA2->IFCR |= 0x0000000FUL;
    else if (DMAy_Channelx == DMA2_Channel2) DMA2->IFCR |= 0x000000F0UL;
    else if (DMAy_Channelx == DMA2_Channel3) DMA2->IFCR |= 0x00000F00UL;
    else if (DMAy_Channelx == DMA2_Channel4) DMA2->IFCR |= 0x0000F000UL;
    else if (DMAy_Channelx == DMA2_Channel5) DMA2->IFCR |= 0x000F0000UL;
}

/**
 * @brief Initializes the DMAy Channelx according to the specified parameters.
 *        Reconstructed from FUN_0800424c @ 0x0800424c.
 */
void DMA_Init(DMA_Channel_TypeDef *DMAy_Channelx, const DMA_InitTypeDef *DMA_InitStruct)
{
    DMAy_Channelx->CCR = (DMAy_Channelx->CCR & 0xFFFF800FUL) |
                         DMA_InitStruct->DMA_DIR |
                         DMA_InitStruct->DMA_Mode |
                         DMA_InitStruct->DMA_PeripheralInc |
                         DMA_InitStruct->DMA_MemoryInc |
                         DMA_InitStruct->DMA_PeripheralDataSize |
                         DMA_InitStruct->DMA_MemoryDataSize |
                         DMA_InitStruct->DMA_Priority |
                         DMA_InitStruct->DMA_M2M;
    DMAy_Channelx->CNDTR = DMA_InitStruct->DMA_BufferSize;
    DMAy_Channelx->CPAR  = DMA_InitStruct->DMA_PeripheralBaseAddr;
    DMAy_Channelx->CMAR  = DMA_InitStruct->DMA_MemoryBaseAddr;
}

/**
 * @brief Configures DMA1 Channels 4 & 5 and NVIC for USART1 (Port 0 RS-485).
 *        Reconstructed from FUN_08003ef4 @ 0x08003ef4.
 */
void USART1_DMA_NVIC_Config(void)
{
    DMA_InitTypeDef  dma_init;
    NVIC_InitTypeDef nvic_init;

    RCC_AHBPeriphClockCmd(0x00000001UL, 1);

    /* USART1_TX on DMA1_Channel4 (from 0x20005EC2) */
    DMA_DeInit(DMA1_Channel4);
    dma_init.DMA_PeripheralBaseAddr = 0x40013804UL; /* &USART1->DR */
    dma_init.DMA_MemoryBaseAddr     = 0x20005EC2UL;
    dma_init.DMA_DIR                = 0x00000010UL; /* PeripheralDST */
    dma_init.DMA_BufferSize         = 256UL;
    dma_init.DMA_PeripheralInc      = 0x00000000UL;
    dma_init.DMA_MemoryInc          = 0x00000080UL;
    dma_init.DMA_PeripheralDataSize = 0x00000000UL;
    dma_init.DMA_MemoryDataSize     = 0x00000000UL;
    dma_init.DMA_Mode               = 0x00000000UL;
    dma_init.DMA_Priority           = 0x00001000UL;
    dma_init.DMA_M2M                = 0x00000000UL;
    DMA_Init(DMA1_Channel4, &dma_init);
    DMA_ClearFlag(0x00001000UL);
    DMA_Cmd(DMA1_Channel4, 0);

    nvic_init.NVIC_IRQChannel                   = 37U; /* USART1_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 2U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);

    /* USART1_RX on DMA1_Channel5 (into 0x20005DC2) */
    DMA_DeInit(DMA1_Channel5);
    dma_init.DMA_PeripheralBaseAddr = 0x40013804UL;
    dma_init.DMA_MemoryBaseAddr     = 0x20005DC2UL;
    dma_init.DMA_DIR                = 0x00000000UL;
    dma_init.DMA_BufferSize         = 256UL;
    dma_init.DMA_PeripheralInc      = 0x00000000UL;
    dma_init.DMA_MemoryInc          = 0x00000080UL;
    dma_init.DMA_PeripheralDataSize = 0x00000000UL;
    dma_init.DMA_MemoryDataSize     = 0x00000000UL;
    dma_init.DMA_Mode               = 0x00000000UL;
    dma_init.DMA_Priority           = 0x00003000UL;
    dma_init.DMA_M2M                = 0x00000000UL;
    DMA_Init(DMA1_Channel5, &dma_init);
    DMA_ClearFlag(0x00010000UL);
    DMA_Cmd(DMA1_Channel5, 1);
}

/**
 * @brief Configures DMA1 Channels 6 & 7 and NVIC for USART2 (Port 1 RS-485).
 *        Reconstructed from FUN_08003fba @ 0x08003fba.
 */
void USART2_DMA_NVIC_Config(void)
{
    DMA_InitTypeDef  dma_init;
    NVIC_InitTypeDef nvic_init;

    RCC_AHBPeriphClockCmd(0x00000001UL, 1);

    /* USART2_TX on DMA1_Channel7 (from 0x20005CC2) */
    DMA_DeInit(DMA1_Channel7);
    dma_init.DMA_PeripheralBaseAddr = 0x40004404UL; /* &USART2->DR */
    dma_init.DMA_MemoryBaseAddr     = 0x20005CC2UL;
    dma_init.DMA_DIR                = 0x00000010UL;
    dma_init.DMA_BufferSize         = 256UL;
    dma_init.DMA_PeripheralInc      = 0x00000000UL;
    dma_init.DMA_MemoryInc          = 0x00000080UL;
    dma_init.DMA_PeripheralDataSize = 0x00000000UL;
    dma_init.DMA_MemoryDataSize     = 0x00000000UL;
    dma_init.DMA_Mode               = 0x00000000UL;
    dma_init.DMA_Priority           = 0x00001000UL;
    dma_init.DMA_M2M                = 0x00000000UL;
    DMA_Init(DMA1_Channel7, &dma_init);
    DMA_ClearFlag(0x01000000UL);
    DMA_Cmd(DMA1_Channel7, 0);

    nvic_init.NVIC_IRQChannel                   = 38U; /* USART2_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 2U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);

    /* USART2_RX on DMA1_Channel6 (into 0x20005BC2) */
    DMA_DeInit(DMA1_Channel6);
    dma_init.DMA_PeripheralBaseAddr = 0x40004404UL;
    dma_init.DMA_MemoryBaseAddr     = 0x20005BC2UL;
    dma_init.DMA_DIR                = 0x00000000UL;
    dma_init.DMA_BufferSize         = 256UL;
    dma_init.DMA_PeripheralInc      = 0x00000000UL;
    dma_init.DMA_MemoryInc          = 0x00000080UL;
    dma_init.DMA_PeripheralDataSize = 0x00000000UL;
    dma_init.DMA_MemoryDataSize     = 0x00000000UL;
    dma_init.DMA_Mode               = 0x00000000UL;
    dma_init.DMA_Priority           = 0x00003000UL;
    dma_init.DMA_M2M                = 0x00000000UL;
    DMA_Init(DMA1_Channel6, &dma_init);
    DMA_ClearFlag(0x00100000UL);
    DMA_Cmd(DMA1_Channel6, 1);
}

/**
 * @brief Configures DMA1 Channels 2 & 3 and NVIC for USART3 (EM Expansion Bus).
 *        Reconstructed from FUN_08004080 @ 0x08004080.
 */
void USART3_DMA_NVIC_Config(void)
{
    DMA_InitTypeDef  dma_init;
    NVIC_InitTypeDef nvic_init;

    RCC_AHBPeriphClockCmd(0x00000001UL, 1);

    /* USART3_TX on DMA1_Channel2 (from 0x20005B42) */
    DMA_DeInit(DMA1_Channel2);
    dma_init.DMA_PeripheralBaseAddr = 0x40004804UL; /* &USART3->DR */
    dma_init.DMA_MemoryBaseAddr     = 0x20005B42UL;
    dma_init.DMA_DIR                = 0x00000010UL;
    dma_init.DMA_BufferSize         = 64UL;
    dma_init.DMA_PeripheralInc      = 0x00000000UL;
    dma_init.DMA_MemoryInc          = 0x00000080UL;
    dma_init.DMA_PeripheralDataSize = 0x00000000UL;
    dma_init.DMA_MemoryDataSize     = 0x00000000UL;
    dma_init.DMA_Mode               = 0x00000000UL;
    dma_init.DMA_Priority           = 0x00001000UL;
    dma_init.DMA_M2M                = 0x00000000UL;
    DMA_Init(DMA1_Channel2, &dma_init);
    DMA_ClearFlag(0x00000010UL);
    DMA_Cmd(DMA1_Channel2, 0);

    nvic_init.NVIC_IRQChannel                   = 39U; /* USART3_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 1U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);

    /* USART3_RX on DMA1_Channel3 (into 0x20005AC2) */
    DMA_DeInit(DMA1_Channel3);
    dma_init.DMA_PeripheralBaseAddr = 0x40004804UL;
    dma_init.DMA_MemoryBaseAddr     = 0x20005AC2UL;
    dma_init.DMA_DIR                = 0x00000000UL;
    dma_init.DMA_BufferSize         = 64UL;
    dma_init.DMA_PeripheralInc      = 0x00000000UL;
    dma_init.DMA_MemoryInc          = 0x00000080UL;
    dma_init.DMA_PeripheralDataSize = 0x00000000UL;
    dma_init.DMA_MemoryDataSize     = 0x00000000UL;
    dma_init.DMA_Mode               = 0x00000000UL;
    dma_init.DMA_Priority           = 0x00003000UL;
    dma_init.DMA_M2M                = 0x00000000UL;
    DMA_Init(DMA1_Channel3, &dma_init);
    DMA_ClearFlag(0x00000100UL);
    DMA_Cmd(DMA1_Channel3, 1);
}

/* ==========================================================================
 * SPI2 & W5500 Ethernet Controller Driver (0x080043b8..0x080047ab, 0x08036df4..0x080371ac, 0x0803ee80..0x0803f1b5)
 * ========================================================================== */

/**
 * @brief Busy-wait loop delay (~8000 cycles per unit).
 *        Reconstructed from FUN_080043b8 @ 0x080043b8.
 */
void Delay_RoughMs(uint16_t ms)
{
    for (uint16_t i = 0; i < ms; i++) {
        for (volatile uint16_t j = 0; j < 8000U; j++) {
        }
    }
}

/**
 * @brief Busy-wait loop delay (~8 cycles per unit).
 *        Reconstructed from FUN_080043c8 @ 0x080043c8.
 */
void Delay_RoughUs(uint16_t us)
{
    for (uint16_t i = 0; i < us; i++) {
        for (volatile uint16_t j = 0; j < 8U; j++) {
        }
    }
}

/**
 * @brief Enables or disables the specified SPI peripheral.
 *        Reconstructed from FUN_08037138 @ 0x08037138.
 */
void SPI_Cmd(SPI_TypeDef *SPIx, uint8_t NewState)
{
    if (NewState != 0) {
        SPIx->CR1 |= 0x0040U;
    } else {
        SPIx->CR1 &= (uint16_t)~0x0040U;
    }
}

/**
 * @brief Checks whether the specified SPI flag is set or not.
 *        Reconstructed from FUN_08037150 @ 0x08037150.
 */
bool SPI_I2S_GetFlagStatus(SPI_TypeDef *SPIx, uint16_t SPI_I2S_FLAG)
{
    return (SPIx->SR & SPI_I2S_FLAG) != 0;
}

/**
 * @brief Returns the most recent received data by the SPIx peripheral.
 *        Reconstructed from FUN_0803715e @ 0x0803715e.
 */
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef *SPIx)
{
    return SPIx->DR;
}

/**
 * @brief Transmits a Data through the SPIx peripheral.
 *        Reconstructed from FUN_08037162 @ 0x08037162.
 */
void SPI_I2S_SendData(SPI_TypeDef *SPIx, uint16_t Data)
{
    SPIx->DR = Data;
}

/**
 * @brief Initializes the SPIx peripheral according to the specified parameters.
 *        Reconstructed from FUN_08037166 @ 0x08037166.
 */
void SPI_Init(SPI_TypeDef *SPIx, const SPI_InitTypeDef *SPI_InitStruct)
{
    SPIx->CR1 = SPI_InitStruct->SPI_Direction |
                SPI_InitStruct->SPI_Mode |
                SPI_InitStruct->SPI_DataSize |
                SPI_InitStruct->SPI_CPOL |
                SPI_InitStruct->SPI_CPHA |
                SPI_InitStruct->SPI_NSS |
                SPI_InitStruct->SPI_BaudRatePrescaler |
                SPI_InitStruct->SPI_FirstBit |
                (SPIx->CR1 & 0x3040U);
    SPIx->I2SCFGR &= 0xF7FFU;
    SPIx->CRCPR    = SPI_InitStruct->SPI_CRCPolynomial;
}

/**
 * @brief Configures SPI2 master mode (PB13=SCK, PB14=MISO, PB15=MOSI) for W5500.
 *        Reconstructed from FUN_080422c6 @ 0x080422c6.
 */
void SPI2_Master_Init(void)
{
    SPI_InitTypeDef spi_init;

    RCC_APB1PeriphClockCmd(0x00004000UL, 1); /* SPI2 */
    spi_init.SPI_Direction         = 0x0000U; /* 2Lines_FullDuplex */
    spi_init.SPI_Mode              = 0x0104U; /* Master */
    spi_init.SPI_DataSize          = 0x0000U; /* 8b */
    spi_init.SPI_CPOL              = 0x0000U; /* Low */
    spi_init.SPI_CPHA              = 0x0000U; /* 1Edge */
    spi_init.SPI_NSS               = 0x0200U; /* Soft */
    spi_init.SPI_BaudRatePrescaler = 0x0000U; /* Prescaler_2 */
    spi_init.SPI_FirstBit          = 0x0000U; /* MSB */
    spi_init.SPI_CRCPolynomial     = 7U;
    SPI_Init(SPI2, &spi_init);
    SPI_Cmd(SPI2, 1);
}

/* W5500 CS (PE5) and RST (PE4) control helpers (0x08036df4..0x080371ac) */
static inline void W5500_CS_High(void)  { GPIO_SetBits(GPIOE, 0x0020U); }   /* FUN_08036df4 */
static inline void W5500_CS_Low(void)   { GPIO_ResetBits(GPIOE, 0x0020U); } /* FUN_0803719e */
static inline void W5500_RST_High(void) { GPIO_SetBits(GPIOE, 0x0010U); }   /* FUN_08036e02 */
static inline void W5500_RST_Low(void)  { GPIO_ResetBits(GPIOE, 0x0010U); } /* FUN_080371ac */

/**
 * @brief Reads a single byte from SPI2.
 *        Reconstructed from FUN_080370a8 @ 0x080370a8.
 */
uint8_t SPI2_ReadByte(void)
{
    while (!SPI_I2S_GetFlagStatus(SPI2, 0x0002U)) {
    }
    SPI_I2S_SendData(SPI2, 0xFFFFU);
    while (!SPI_I2S_GetFlagStatus(SPI2, 0x0001U)) {
    }
    return (uint8_t)SPI_I2S_ReceiveData(SPI2);
}

/**
 * @brief Sends a single byte over SPI2.
 *        Reconstructed from FUN_080370d4 @ 0x080370d4.
 */
void SPI2_SendByte(uint8_t byte)
{
    while (!SPI_I2S_GetFlagStatus(SPI2, 0x0002U)) {
    }
    SPI_I2S_SendData(SPI2, byte);
    while (SPI_I2S_GetFlagStatus(SPI2, 0x0080U)) {
    }
    (void)SPI2->DR;
}

/**
 * @brief Sends a 16-bit halfword (big-endian) over SPI2.
 *        Reconstructed from FUN_080370fe @ 0x080370fe.
 */
void SPI2_SendHalfWord(uint16_t hw)
{
    SPI2_SendByte((uint8_t)(hw >> 8));
    SPI2_SendByte((uint8_t)(hw & 0xFFU));
}

/**
 * @brief Writes a 1-byte register in the W5500 common register block.
 *        Reconstructed from FUN_08004636 @ 0x08004636.
 */
void W5500_WriteCommonReg8(uint16_t reg_offset, uint8_t data)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(reg_offset);
    SPI2_SendByte(0x05U); /* Common Reg Block + Write + FDM1 */
    SPI2_SendByte(data);
    W5500_CS_High();
}

/**
 * @brief Reads a 1-byte register from the W5500 socket register block.
 *        Reconstructed from FUN_08004450 @ 0x08004450.
 */
uint8_t W5500_ReadSocketReg8(uint8_t sock, uint16_t reg_offset)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(reg_offset);
    SPI2_SendByte((uint8_t)((sock << 5) | 0x09U));
    uint8_t val = SPI2_ReadByte();
    W5500_CS_High();
    return val;
}

/**
 * @brief Reads a 16-bit register from the W5500 socket register block.
 *        Reconstructed from FUN_08004484 @ 0x08004484.
 */
uint16_t W5500_ReadSocketReg16(uint8_t sock, uint16_t reg_offset)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(reg_offset);
    SPI2_SendByte((uint8_t)((sock << 5) | 0x0AU));
    uint16_t hi = SPI2_ReadByte();
    uint16_t lo = SPI2_ReadByte();
    W5500_CS_High();
    return (uint16_t)((hi << 8) | lo);
}

/**
 * @brief Writes a 1-byte register in the W5500 socket register block.
 *        Reconstructed from FUN_08004698 @ 0x08004698.
 */
void W5500_WriteSocketReg8(uint8_t sock, uint16_t reg_offset, uint8_t data)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(reg_offset);
    SPI2_SendByte((uint8_t)((sock << 5) | 0x0DU));
    SPI2_SendByte(data);
    W5500_CS_High();
}

/**
 * @brief Writes a 16-bit register in the W5500 socket register block.
 *        Reconstructed from FUN_080046c8 @ 0x080046c8.
 */
void W5500_WriteSocketReg16(uint8_t sock, uint16_t reg_offset, uint16_t data)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(reg_offset);
    SPI2_SendByte((uint8_t)((sock << 5) | 0x0EU));
    SPI2_SendHalfWord(data);
    W5500_CS_High();
}

/**
 * @brief Configures W5500 Gateway IP Address (GAR 0x0001..0x0004).
 *        Reconstructed from FUN_080043d8 @ 0x080043d8.
 */
void W5500_SetGateway(uint8_t gw0, uint8_t gw1, uint8_t gw2, uint8_t gw3)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(0x0001U);
    SPI2_SendByte(0x07U); /* Common Reg + Write + FDM4 */
    SPI2_SendByte(gw0);
    SPI2_SendByte(gw1);
    SPI2_SendByte(gw2);
    SPI2_SendByte(gw3);
    W5500_CS_High();
}

/**
 * @brief Configures W5500 Subnet Mask (SUBR 0x0005..0x0008).
 *        Reconstructed from FUN_08004700 @ 0x08004700.
 */
void W5500_SetSubnetMask(uint8_t sub0, uint8_t sub1, uint8_t sub2, uint8_t sub3)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(0x0005U);
    SPI2_SendByte(0x07U);
    SPI2_SendByte(sub0);
    SPI2_SendByte(sub1);
    SPI2_SendByte(sub2);
    SPI2_SendByte(sub3);
    W5500_CS_High();
}

/**
 * @brief Configures W5500 Source Hardware MAC Address (SHAR 0x0009..0x000E).
 *        Reconstructed from FUN_080044c4 @ 0x080044c4.
 */
void W5500_SetMACAddress(uint16_t mac01, uint16_t mac23, uint16_t mac45)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(0x0009U);
    SPI2_SendByte(0x06U);
    SPI2_SendHalfWord(mac01);
    W5500_CS_High();

    W5500_CS_Low();
    SPI2_SendHalfWord(0x000BU);
    SPI2_SendByte(0x06U);
    SPI2_SendHalfWord(mac23);
    W5500_CS_High();

    W5500_CS_Low();
    SPI2_SendHalfWord(0x000DU);
    SPI2_SendByte(0x06U);
    SPI2_SendHalfWord(mac45);
    W5500_CS_High();
}

/**
 * @brief Configures W5500 Source IP Address (SIPR 0x000F..0x0012).
 *        Reconstructed from FUN_08004414 @ 0x08004414.
 */
void W5500_SetIPAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3)
{
    W5500_CS_Low();
    SPI2_SendHalfWord(0x000FU);
    SPI2_SendByte(0x07U);
    SPI2_SendByte(ip0);
    SPI2_SendByte(ip1);
    SPI2_SendByte(ip2);
    SPI2_SendByte(ip3);
    W5500_CS_High();
}

/**
 * @brief Configures a W5500 Socket local port and MSS (1460 bytes).
 *        Reconstructed from FUN_0800455c @ 0x0800455c.
 */
void W5500_ConfigSocketPort(uint8_t sock, uint16_t port)
{
    W5500_WriteSocketReg16(sock, 0x0012U, 1460U); /* Sn_MSSR = 1460 */
    W5500_WriteSocketReg16(sock, 0x0004U, port);  /* Sn_PORT */
}

/**
 * @brief Resets W5500 hardware and configures retry time (200 ms), retry count (8),
 *        and socket interrupt masks.
 *        Reconstructed from FUN_08036e52 @ 0x08036e52.
 */
void W5500_HardwareReset(void)
{
    W5500_RST_Low();
    Delay_RoughMs(5);
    W5500_RST_High();
    Delay_RoughMs(20);

    W5500_WriteCommonReg8(0x0000U, 0x80U); /* MR = RST */
    Delay_RoughMs(5);

    /* RTR = 2000 (200 ms), RCR = 8, IMR = 0xF0, SIMR = 0xFF */
    W5500_CS_Low();
    SPI2_SendHalfWord(0x0019U);
    SPI2_SendByte(0x06U);
    SPI2_SendHalfWord(2000U);
    W5500_CS_High();

    W5500_WriteCommonReg8(0x001BU, 8U);
    W5500_WriteCommonReg8(0x0016U, 0xF0U);
    W5500_WriteCommonReg8(0x0018U, 0xFFU);
}

/**
 * @brief Configures W5500 network parameters (MAC, IP, Gateway, Subnet) and
 *        initializes 6 S7comm sockets (port 102) + 2 Modbus TCP sockets (port 502).
 *        Reconstructed from FUN_0803f020 @ 0x0803f020.
 */
void W5500_ConfigureNetworkAndSockets(void)
{
    if (g_EthDynamicCfgMode == 1U) {
        uint16_t mac01 = (uint16_t)((g_EthConfigBytes[12] << 8) | g_EthConfigBytes[13]);
        uint16_t mac23 = (uint16_t)((g_EthConfigBytes[14] << 8) | g_EthConfigBytes[15]);
        uint16_t mac45 = (uint16_t)((g_EthConfigBytes[16] << 8) | g_EthConfigBytes[17]);
        W5500_SetMACAddress(mac01, mac23, mac45);
        W5500_SetIPAddress(g_EthConfigBytes[0], g_EthConfigBytes[1], g_EthConfigBytes[2], g_EthConfigBytes[3]);
        W5500_SetGateway(g_EthConfigBytes[8], g_EthConfigBytes[9], g_EthConfigBytes[10], g_EthConfigBytes[11]);
        W5500_SetSubnetMask(g_EthConfigBytes[4], g_EthConfigBytes[5], g_EthConfigBytes[6], g_EthConfigBytes[7]);

        for (uint8_t s = 0; s < 6U; s++) {
            W5500_ConfigSocketPort(s, 102U);
        }
        W5500_ConfigSocketPort(6U, 502U);
        W5500_ConfigSocketPort(7U, 502U);

        if (g_EthCustomPortFlag == 1U) {
            W5500_ConfigSocketPort(0U, 102U);
            for (uint8_t s = 1U; s < 8U; s++) {
                W5500_ConfigSocketPort(s, g_EthCustomPorts[s - 1U]);
            }
            return;
        }
    } else {
        /* Default MAC 00:08:DC:01:02:03, IP 192.168.1.218, GW 192.168.1.1, Mask 255.255.255.0 */
        W5500_SetMACAddress(0x0008U, 0xDC01U, 0x0203U);
        W5500_SetIPAddress(192U, 168U, 1U, 218U);
        W5500_SetGateway(192U, 168U, 1U, 1U);
        W5500_SetSubnetMask(255U, 255U, 255U, 0U);
    }

    for (uint8_t s = 0; s < 6U; s++) {
        W5500_ConfigSocketPort(s, 102U); /* S7comm ISO-on-TCP port 102 */
    }
    W5500_ConfigSocketPort(6U, 502U);     /* Modbus TCP port 502 */
    W5500_ConfigSocketPort(7U, 502U);
}

/**
 * @brief Triggers RS-485 DMA transmission on Port 0 (TIM6/USART1) or Port 1
 *        (TIM7/USART2) after the configured turnaround delay.
 *        Reconstructed from FUN_08036fb6 @ 0x08036fb6.
 */
void RS485_StartTransmit_WithTurnaround(uint8_t port, uint16_t tx_len)
{
    if (port == 0U) {
        g_Port0_TxLen = tx_len;
        /* Drive PE14 (Port 0 RS-485 DE/RE) HIGH for transmit */
        GPIO_SetBits(GPIOE, 0x4000U);
        DMA_Cmd(DMA1_Channel5, 0); /* Disable USART1 RX DMA */
        DMA_ClearFlag(0x00001000UL);
        /* Load TIM6 turnaround counter and enable TIM6 */
        MMIO16(0x40001024U) = g_Port0_TurnaroundCnt;
        MMIO16(0x40001000U) |= 0x0001U;
    } else if (port == 1U) {
        g_Port1_TxLen = tx_len;
        DMA_Cmd(DMA1_Channel6, 0); /* Disable USART2 RX DMA */
        DMA_ClearFlag(0x01000000UL);
        /* Load TIM7 turnaround counter and enable TIM7 */
        MMIO16(0x40001424U) = g_Port1_TurnaroundCnt;
        MMIO16(0x40001400U) |= 0x0001U;
    }
}

/* ==========================================================================
 * Internal Flash Driver & PLC Block Storage (0x08013c9c..0x08013d74, 0x080371ba..0x080378c8)
 * ========================================================================== */

/**
 * @brief Clears the FLASH's pending flags.
 *        Reconstructed from FUN_08013c9c @ 0x08013c9c.
 */
void FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
    FLASH_R->SR = FLASH_FLAG;
}

/**
 * @brief Returns the FLASH Status.
 *        Reconstructed from FUN_08013cf0 @ 0x08013cf0.
 */
FLASH_Status FLASH_GetStatus(void)
{
    if ((FLASH_R->SR & FLASH_SR_BSY) != 0) {
        return FLASH_BUSY;
    }
    if ((FLASH_R->SR & FLASH_SR_PGERR) != 0) {
        return FLASH_ERROR_PG;
    }
    if ((FLASH_R->SR & FLASH_SR_WRPRTERR) != 0) {
        return FLASH_ERROR_WRP;
    }
    return FLASH_COMPLETE;
}

/**
 * @brief Waits for a Flash operation to complete or a TIMEOUT to occur.
 *        Reconstructed from FUN_08013d74 @ 0x08013d74.
 */
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout)
{
    FLASH_Status status;
    do {
        status = FLASH_GetStatus();
        if (Timeout == 0) {
            break;
        }
        Timeout--;
    } while (status == FLASH_BUSY);

    if (Timeout == 0) {
        status = FLASH_TIMEOUT;
    }
    return status;
}

/**
 * @brief Unlocks the FLASH Program Erase Controller.
 *        Reconstructed from FUN_08013d56 @ 0x08013d56.
 */
void FLASH_Unlock(void)
{
    FLASH_R->KEYR = FLASH_KEY1;
    FLASH_R->KEYR = FLASH_KEY2;
}

/**
 * @brief Locks the FLASH Program Erase Controller.
 *        Reconstructed from FUN_08013d0e @ 0x08013d0e.
 */
void FLASH_Lock(void)
{
    FLASH_R->CR |= FLASH_CR_LOCK;
}

/**
 * @brief Erases a specified 2 KB FLASH page.
 *        Reconstructed from FUN_08013ca8 @ 0x08013ca8.
 */
FLASH_Status FLASH_ErasePage(uint32_t Page_Address)
{
    FLASH_Status status = FLASH_WaitForLastOperation(0x000B0000UL);
    if (status == FLASH_COMPLETE) {
        FLASH_R->CR |= FLASH_CR_PER;
        FLASH_R->AR  = Page_Address;
        FLASH_R->CR |= FLASH_CR_STRT;
        status = FLASH_WaitForLastOperation(0x000B0000UL);
        FLASH_R->CR &= ~FLASH_CR_PER;
    }
    return status;
}

/**
 * @brief Programs a half word (16-bit) at a specified address.
 *        Reconstructed from FUN_08013d20 @ 0x08013d20.
 */
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
    FLASH_Status status = FLASH_WaitForLastOperation(0x00002000UL);
    if (status == FLASH_COMPLETE) {
        FLASH_R->CR |= FLASH_CR_PG;
        MMIO16(Address) = Data;
        status = FLASH_WaitForLastOperation(0x00002000UL);
        FLASH_R->CR &= ~FLASH_CR_PG;
    }
    return status;
}

/**
 * @brief Computes the 16-bit S7-200 DB1 data block checksum over 0x2824 bytes.
 *        Reconstructed from FUN_08004300 @ 0x08004300.
 *
 * @param from_ram 0 to compute over Flash DB1 (0x08064800), 1 to compute over RAM (0x20001068)
 */
void PLC_CalcDB1Checksum(uint8_t from_ram)
{
    const uint8_t *base = (from_ram == 0) ? (const uint8_t *)FLASH_DB1_PAGE0_ADDR
                                          : (const uint8_t *)g_PLC_V_Header;
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;
    uint32_t w_a   = 0x23U;
    uint32_t w_b   = 0x24U;

    for (uint32_t i = 0; i < 0x2824U; i++) {
        uint8_t b = base[i];
        sum_a += (uint32_t)b * w_a;
        sum_b += (uint32_t)b * w_b;
        w_a = (w_a + 1U) & 0xFFFFU;
        if (w_a == 0xFFU) {
            w_a = 0U;
        }
        w_b = (w_a + 1U) & 0xFFFFU;
    }

    uint32_t fold_a = (sum_a & 0xFFU) + ((sum_a & 0x80FFFFFFUL) >> 16) +
                      (((sum_a & 0x80FFFFFFUL) >> 8) & 0xFF8000FFUL) + 0x17U;
    uint32_t fold_a2 = (fold_a & 0xFFU) + ((fold_a >> 8) & 0xFF8000FFUL);
    g_DB1_ChecksumLo = (uint8_t)(fold_a2 + (fold_a2 >> 8));

    int32_t fold_b = -(int32_t)((sum_b & 0xFFU) + ((sum_b & 0x80FFFFFFUL) >> 16) +
                                (((sum_b & 0x80FFFFFFUL) >> 8) & 0xFF8000FFUL) - 0x3DU);
    g_DB1_ChecksumHi = (uint8_t)(fold_b + ((uint32_t)fold_b >> 8));
}

/**
 * @brief Computes the two 16-bit SDB (System Data Block) checksums over 0x200058CC.
 *        Reconstructed from FUN_0803ed26 @ 0x0803ed26.
 */
void PLC_CalcSDBChecksum(void)
{
    uint32_t w_a   = 0x93U;
    uint32_t w_b   = 0x94U;
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;

    for (uint32_t i = 0; i < 0x16AU; i++) {
        uint8_t b = g_PLC_SDB_Buffer[i];
        sum_a += (uint32_t)b * w_a;
        sum_b += (uint32_t)b * w_b;
        w_a = (w_a + 1U) & 0xFFFFU;
        if (w_a == 0xFFU) {
            w_a = 0U;
        }
        w_b = (w_a + 1U) & 0xFFFFU;
    }

    uint32_t fold_a  = (sum_a & 0xFFU) + ((sum_a & 0x80FFFFFFUL) >> 16) +
                       (((sum_a & 0x80FFFFFFUL) >> 8) & 0xFF8000FFUL) + 0x87U;
    uint32_t fold_a2 = (fold_a & 0xFFU) + ((fold_a >> 8) & 0xFF8000FFUL);
    g_SDB_Checksum0  = (uint8_t)(fold_a2 + (fold_a2 >> 8));

    int32_t fold_b  = -(int32_t)((sum_b & 0xFFU) + ((sum_b & 0x80FFFFFFUL) >> 16) +
                                 (((sum_b & 0x80FFFFFFUL) >> 8) & 0xFF8000FFUL) - 0x11DU);
    g_SDB_Checksum1 = (uint8_t)(fold_b + ((uint32_t)fold_b >> 8));

    uint32_t w_c   = 0x12U;
    uint32_t sum_c = 0;
    uint32_t sum_d = 0;
    for (uint32_t i = 0x16CU; i < 0x17EU; i++) {
        uint8_t b = g_PLC_SDB_Buffer[i];
        sum_d += (uint32_t)b * w_c;
        sum_c += b;
        w_c = (w_c - 1U) & 0xFFFFU;
    }

    uint32_t fold_c  = (sum_c & 0xFFU) + (((sum_c & 0x80FFFFFFUL) >> 8) & 0xFF8000FFUL) + 0x5AU;
    uint32_t fold_c2 = (fold_c & 0xFFU) + ((fold_c >> 8) & 0xFF8000FFUL);
    g_SDB_Checksum2  = (uint8_t)(fold_c2 + (fold_c2 >> 8));

    uint32_t fold_d  = (sum_d & 0xFFU) + (((sum_d & 0x80FFFFFFUL) >> 8) & 0xFF8000FFUL) + 0xB4U;
    uint32_t fold_d2 = (fold_d & 0xFFU) + ((fold_d >> 8) & 0xFF8000FFUL);
    g_SDB_Checksum3  = (uint8_t)(fold_d2 + (fold_d2 >> 8));
}

/**
 * @brief Initializes Flash DB1 pages (0x08064800, 0x08068000) with default
 *        62-byte DB1 header from 0x0804A264 and writes checksum to 0x08068526.
 *        Reconstructed from FUN_080371ba @ 0x080371ba.
 */
void Flash_InitDefaultDB1(void)
{
    uint32_t dst = FLASH_DB1_PAGE0_ADDR;

    FLASH_ClearFlag(0x35UL);
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_DB1_PAGE0_ADDR);
    FLASH_ErasePage(FLASH_DB1_PAGE1_ADDR);

    for (uint32_t off = 0; off < 0x3EU; off += 2U) {
        FLASH_ProgramHalfWord(dst, MMIO16(0x0804A264UL + off));
        IWDG_KR = 0xAAAAU;
        dst += 2U;
    }

    PLC_CalcDB1Checksum(0);
    FLASH_ProgramHalfWord(FLASH_DB1_CHKSUM_ADDR,
                          (uint16_t)g_DB1_ChecksumLo | ((uint16_t)g_DB1_ChecksumHi << 8));
    FLASH_Lock();
}

/**
 * @brief Erases and programs the SDB Flash page (0x0807F800, 384 bytes from
 *        0x200058CC) and resets DB1 to defaults.
 *        Reconstructed from FUN_08037244 @ 0x08037244.
 */
void Flash_WriteSDBAndDefaultDB1(void)
{
    uint32_t sdb_dst = FLASH_SDB_PAGE_ADDR;
    uint32_t db1_dst = FLASH_DB1_PAGE0_ADDR;

    FLASH_ClearFlag(0x35UL);
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_SDB_PAGE_ADDR);
    FLASH_ErasePage(FLASH_DB1_PAGE0_ADDR);
    FLASH_ErasePage(FLASH_DB1_PAGE1_ADDR);

    PLC_CalcSDBChecksum();
    g_PLC_SDB_Buffer[0x16A] = g_SDB_Checksum0;
    g_PLC_SDB_Buffer[0x16B] = g_SDB_Checksum1;
    g_PLC_SDB_Buffer[0x17E] = g_SDB_Checksum2;
    g_PLC_SDB_Buffer[0x17F] = g_SDB_Checksum3;

    for (uint32_t off = 0; off < 0x17FU; off += 2U) {
        FLASH_ProgramHalfWord(sdb_dst, MMIO16(0x200058CCUL + off));
        IWDG_KR = 0xAAAAU;
        sdb_dst += 2U;
    }

    for (uint32_t off = 0; off < 0x3EU; off += 2U) {
        FLASH_ProgramHalfWord(db1_dst, MMIO16(0x0804A264UL + off));
        IWDG_KR = 0xAAAAU;
        db1_dst += 2U;
    }

    PLC_CalcDB1Checksum(0);
    FLASH_ProgramHalfWord(FLASH_DB1_CHKSUM_ADDR,
                          (uint16_t)g_DB1_ChecksumLo | ((uint16_t)g_DB1_ChecksumHi << 8));
    FLASH_Lock();
}

/**
 * @brief Erases DB1 Flash pages (0x08064800 and 0x08068000) and resets write pointer.
 *        Reconstructed from FUN_08037330 @ 0x08037330.
 */
void Flash_EraseDB1Pages(void)
{
    FLASH_ClearFlag(0x35UL);
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_DB1_PAGE0_ADDR);
    IWDG_KR = 0xAAAAU;
    FLASH_ErasePage(FLASH_DB1_PAGE1_ADDR);
    g_FlashDB1WritePtr = FLASH_DB1_BODY_ADDR;
}

/**
 * @brief Erases OB1 Flash pages (0x0806D800 and 0x08075800).
 *        Reconstructed from FUN_08037376 @ 0x08037376.
 */
void Flash_EraseOB1Pages(void)
{
    FLASH_ClearFlag(0x35UL);
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_OB1_PAGE0_ADDR);
    IWDG_KR = 0xAAAAU;
    FLASH_ErasePage(FLASH_OB1_PAGE1_ADDR);
}

/**
 * @brief Erases and initializes default OB1 program block in Flash (0x0806D800
 *        and 0x08075800).
 *        Reconstructed from FUN_080373ac @ 0x080373ac.
 */
void Flash_InitDefaultOB1(void)
{
    uint32_t dst_page0 = FLASH_OB1_PAGE0_ADDR;
    uint32_t dst_page1 = FLASH_OB1_PAGE1_ADDR;

    FLASH_ClearFlag(0x35UL);
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_OB1_PAGE0_ADDR);
    FLASH_ErasePage(FLASH_OB1_PAGE1_ADDR);

    MMIO8(0x200004C6U) = 0x6DU;
    MMIO8(0x20000491U) = 0xEBU;
    MMIO8(0x20000490U) = 0x7CU;

    for (uint32_t off = 0; off < 0x70U; off += 2U) {
        FLASH_ProgramHalfWord(dst_page1, MMIO16(0x0804A1F4UL + off));
        IWDG_KR = 0xAAAAU;
        dst_page1 += 2U;
    }

    for (uint32_t off = 0; off < 0x36U; off += 2U) {
        FLASH_ProgramHalfWord(dst_page0, MMIO16(0x20000101UL + off));
        IWDG_KR = 0xAAAAU;
        dst_page0 += 2U;
    }

    FLASH_Lock();
}

/**
 * @brief Reads 16-bit halfwords from Flash into a RAM byte buffer while
 *        periodically kicking the IWDG watchdog.
 *        Reconstructed from FUN_08037468 @ 0x08037468.
 */
void Flash_ReadHalfWordsToRAM(const uint16_t *src, uint8_t *dst, uint32_t byte_len)
{
    for (uint32_t off = 0; off < byte_len; off += 2U) {
        uint16_t hw = *src++;
        dst[off + 1U] = (uint8_t)(hw >> 8);
        dst[off]      = (uint8_t)(hw & 0xFFU);
        IWDG_KR       = 0xAAAAU;
    }
}

/**
 * @brief Restores retentive V, M, Timer, and Counter ranges from internal Flash
 *        (0x08069000..0x0806BF9C) into SRAM based on the 6 SDB retentive range
 *        descriptors at 0x20005910.
 *        Reconstructed from FUN_08037494 @ 0x08037494.
 */
uint32_t Flash_RestoreRetentiveMemory(void)
{
    IWDG_KR = 0xAAAAU;

    for (uint32_t idx = 0; idx < 6U; idx++) {
        uint32_t desc_off = idx * 14U;
        int8_t   area_id  = (int8_t)MMIO8(0x20005916UL + desc_off);

        if (area_id == 0x1E) {
            /* Counter retentive range (C0..C255) */
            uint32_t start = ((uint32_t)MMIO8(0x20005918UL + desc_off) << 8) |
                             (uint32_t)MMIO8(0x20005919UL + desc_off);
            uint32_t count = ((uint32_t)MMIO8(0x20005912UL + desc_off) << 8) |
                             (uint32_t)MMIO8(0x20005913UL + desc_off);
            if ((start + count < 0x102U) && (count != 0U)) {
                for (uint32_t i = 0; i < count; i++) {
                    g_PLC_CounterStatus[start + i] = MMIO8(0x0806BC9CUL + start + i);
                }
                for (uint32_t i = 0; i < count; i++) {
                    uint32_t k = (start + i) * 2U;
                    g_PLC_CounterCurrVal[start + i] =
                        (uint16_t)MMIO8(0x0806BD9CUL + k) |
                        ((uint16_t)MMIO8(0x0806BD9DUL + k) << 8);
                }
            }
        } else if (area_id == 0x1F) {
            /* Timer retentive range (T0..T255) */
            uint32_t start = ((uint32_t)MMIO8(0x20005918UL + desc_off) << 8) |
                             (uint32_t)MMIO8(0x20005919UL + desc_off);
            uint32_t count = ((uint32_t)MMIO8(0x20005912UL + desc_off) << 8) |
                             (uint32_t)MMIO8(0x20005913UL + desc_off);
            if ((start + count < 0x102U) && (count != 0U)) {
                for (uint32_t i = 0; i < count; i++) {
                    g_PLC_TimerStatus[start + i] = MMIO8(0x0806B99CUL + start + i);
                }
                for (uint32_t i = 0; i < count; i++) {
                    uint32_t k = (start + i) * 2U;
                    g_PLC_TimerCurrVal[start + i] =
                        (uint16_t)MMIO8(0x0806BA9CUL + k) |
                        ((uint16_t)MMIO8(0x0806BA9DUL + k) << 8);
                }
            }
        } else if ((uint8_t)area_id == 0x83U) {
            /* M-memory retentive range (MB0..MB31) */
            uint32_t bit_addr = (((uint32_t)MMIO8(0x20005917UL + desc_off) << 16) |
                                 ((uint32_t)MMIO8(0x20005918UL + desc_off) << 8) |
                                 (uint32_t)MMIO8(0x20005919UL + desc_off)) & 0x7FFFFUL;
            uint32_t byte_off = bit_addr >> 3;
            uint8_t  width    = MMIO8(0x20005911UL + desc_off);
            uint32_t count    = ((uint32_t)MMIO8(0x20005912UL + desc_off) << 8) |
                                (uint32_t)MMIO8(0x20005913UL + desc_off);
            uint32_t byte_len = (width == 2U) ? count : (width == 4U) ? (count * 2U) : (width == 6U) ? (count * 4U) : 0U;

            if ((byte_off + byte_len < 0x21U) && (byte_len != 0U)) {
                for (uint32_t i = 0; i < byte_len; i++) {
                    g_PLC_M_Memory[byte_off + i] = MMIO8(0x0806BF9CUL + byte_off + i);
                }
            }
        } else if ((uint8_t)area_id == 0x84U) {
            /* V-memory retentive range (VB0..VB10239) */
            uint32_t bit_addr = (((uint32_t)MMIO8(0x20005917UL + desc_off) << 16) |
                                 ((uint32_t)MMIO8(0x20005918UL + desc_off) << 8) |
                                 (uint32_t)MMIO8(0x20005919UL + desc_off)) & 0x7FFFFUL;
            uint32_t byte_off = bit_addr >> 3;
            uint8_t  width    = MMIO8(0x20005911UL + desc_off);
            uint32_t count    = ((uint32_t)MMIO8(0x20005912UL + desc_off) << 8) |
                                (uint32_t)MMIO8(0x20005913UL + desc_off);
            uint32_t byte_len = (width == 2U) ? count : (width == 4U) ? (count * 2U) : (width == 6U) ? (count * 4U) : 0U;

            if ((byte_off + byte_len < 0x299BU) && (byte_len != 0U)) {
                for (uint32_t i = 0; i < byte_len; i++) {
                    g_PLC_V_Memory[byte_off + i] = MMIO8(0x08069026UL + byte_off + i);
                }
            }
        }
    }

    MMIO8(0x2000388CU) = 0U;
    return 1UL;
}

/**
 * @brief Saves all retentive PLC memory areas (V-memory, Timer status/CV,
 *        Counter status/CV, M-memory, and magic word) into 9 Flash pages
 *        starting at 0x08069000 on power-down / stop.
 *        Reconstructed from FUN_08037746 @ 0x08037746.
 */
void Flash_SaveRetentiveMemory(void)
{
    uint32_t page_addr = FLASH_RETENTIVE_ADDR;

    FLASH_ClearFlag(0x35UL);
    FLASH_Unlock();

    /* Erase 9 pages (18 KB: 0x08069000 .. 0x0806D7FF) */
    for (uint16_t p = 0; p < 9U; p++) {
        FLASH_ErasePage(page_addr);
        IWDG_KR = 0xAAAAU;
        page_addr += 0x0800UL;
    }

    /* Write header magic marker at 0x08069000 */
    FLASH_ProgramHalfWord(FLASH_RETENTIVE_ADDR,
                          (uint16_t)g_RetentiveMagicLo | ((uint16_t)g_RetentiveMagicHi << 8));
    uint32_t dst = FLASH_RETENTIVE_ADDR + 2UL;
    IWDG_KR = 0xAAAAU;

    /* Save V-memory + header (0x299A bytes from 0x20001068) */
    for (uint32_t off = 0; off < 0x299AU; off += 2U) {
        FLASH_ProgramHalfWord(dst, MMIO16(0x20001068UL + off));
        IWDG_KR = 0xAAAAU;
        dst += 2U;
    }

    /* Save Timer status flags (256 bytes from 0x2000422C) */
    for (uint32_t off = 0; off < 0x100U; off += 2U) {
        FLASH_ProgramHalfWord(dst, MMIO16(0x2000422CUL + off));
        IWDG_KR = 0xAAAAU;
        dst += 2U;
    }

    /* Save Timer current values (256 halfwords from 0x20004000) */
    for (uint32_t i = 0; i < 0x100U; i++) {
        FLASH_ProgramHalfWord(dst, g_PLC_TimerCurrVal[i]);
        IWDG_KR = 0xAAAAU;
        dst += 2U;
    }

    /* Save Counter status flags (256 bytes from 0x20005000) */
    for (uint32_t off = 0; off < 0x100U; off += 2U) {
        FLASH_ProgramHalfWord(dst, MMIO16(0x20005000UL + off));
        IWDG_KR = 0xAAAAU;
        dst += 2U;
    }

    /* Save Counter current values (256 halfwords from 0x20004440) */
    for (uint32_t i = 0; i < 0x100U; i++) {
        FLASH_ProgramHalfWord(dst, g_PLC_CounterCurrVal[i]);
        IWDG_KR = 0xAAAAU;
        dst += 2U;
    }

    /* Save M-memory (32 bytes from 0x20001000) */
    for (uint32_t off = 0; off < 0x20U; off += 2U) {
        FLASH_ProgramHalfWord(dst, MMIO16(0x20001000UL + off));
        IWDG_KR = 0xAAAAU;
        dst += 2U;
    }

    /* Write trailing magic marker */
    FLASH_ProgramHalfWord(dst, g_RetentiveMagicWord);
    IWDG_KR = 0xAAAAU;
    FLASH_Lock();
}

/**
 * @brief Erases and programs only the SDB Flash page (0x0807F800) from 0x200058CC.
 *        Reconstructed from FUN_080378c8 @ 0x080378c8.
 */
void Flash_WriteSDBPage(void)
{
    uint32_t dst = FLASH_SDB_PAGE_ADDR;

    FLASH_ClearFlag(0x35UL);
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_SDB_PAGE_ADDR);

    for (uint32_t off = 0; off < 0x17FU; off += 2U) {
        FLASH_ProgramHalfWord(dst, MMIO16(0x200058CCUL + off));
        IWDG_KR = 0xAAAAU;
        dst += 2U;
    }

    FLASH_Lock();
}

/* ==========================================================================
 * CRC16, Analog Median-Average Filter, & NTC Thermistor Lookup (0x08000e0e, 0x08042c44..0x08042d3e)
 * ========================================================================== */

/**
 * @brief Computes standard Modbus RTU CRC-16 (polynomial 0xA001).
 *        Reconstructed from FUN_08000e0e @ 0x08000e0e.
 */
uint16_t CRC16_Modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if ((crc & 1U) != 0) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Insertion-sorts a 10-sample ADC window and averages the middle 6 samples.
 *        Reconstructed from FUN_08042c44 @ 0x08042c44 and FUN_08042c82 @ 0x08042c82.
 */
uint16_t ADC_FilterMedian6Of10(uint16_t *samples)
{
    for (int i = 1; i < 10; i++) {
        uint16_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    uint32_t sum = 0;
    for (int i = 2; i < 8; i++) {
        sum += samples[i];
    }
    return (uint16_t)(sum / 6U);
}
