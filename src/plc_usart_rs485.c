/**
 * @file plc_usart_rs485.c
 * @brief RS-485 USART1 (Port 0), USART2 (Port 1), and USART3 (Expansion Bus)
 *        Drivers, PPI / Freeport (RCV/XMT) / Modbus RTU Protocol Framing ISRs,
 *        and Turnaround Delay Timers (TIM6 / TIM7) for the SIMATIC S7-226
 *        (APM32E103ZET6) PLC Firmware.
 *
 * Reconstructed from firmware address range 0x080401b4 .. 0x08042317:
 *   - 0x080401b4 : TIM3_IntervalConfig
 *   - 0x08040228 : TIM6_TurnaroundConfig (Port 0 RS-485 TX turnaround delay timer)
 *   - 0x0804027e : TIM7_TurnaroundConfig (Port 1 RS-485 TX turnaround delay timer)
 *   - 0x080402d4 : USART1_GPIO_Config (PA9 TX AF_PP, PA10 RX IN_PU)
 *   - 0x0804031a : USART1_IRQHandler (2,814 bytes: Port 0 RS-485 TC & IDLE ISR,
 *                  PE14 DE/RE direction control, PPI slave 0x10/0x68 framing,
 *                  Freeport RCV/XMT with SMB86..SMB94, and Modbus RTU CRC-16)
 *   - 0x08040e18 : USART1_BaudParityConfig
 *   - 0x08040e8e : USART2_GPIO_Config (PA2 TX AF_PP, PA3 RX IN_FLOATING)
 *   - 0x08040ece : USART2_IRQHandler (3,274 bytes: Port 1 RS-485 TC & IDLE ISR,
 *                  PPI slave 0x10/0x68 framing, Freeport RCV/XMT with SMB186..SMB194,
 *                  and Modbus RTU CRC-16)
 *   - 0x08041b98 : USART2_BaudParityConfig
 *   - 0x08041c0e : USART3_GPIO_Config (PB10 TX AF_PP, PB11 RX IN_FLOATING)
 *   - 0x08041c66 : USART3_IRQHandler (734 bytes: Expansion bus UART ISR)
 *   - 0x08041f44 : USART3_BaudParityConfig
 *   - 0x08041faa .. 0x0804211b : USART SPL driver routines (USART_Cmd, USART_DMACmd,
 *                                USART_DeInit, USART_ITConfig, USART_Init)
 *   - 0x0804211c : Port0_USART1_Init (SDB baudrate 9600/19200/187500 & station addr)
 *   - 0x080421f0 : Port1_USART2_Init (SDB baudrate 9600/19200/187500 & station addr)
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ==========================================================================
 * Hardware Register Definitions (USART1..USART3, TIM3/6/7, DMA1, GPIO)
 * ========================================================================== */

#define MMIO8(addr)  (*(volatile uint8_t *)(uintptr_t)(addr))
#define MMIO16(addr) (*(volatile uint16_t *)(uintptr_t)(addr))
#define MMIO32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

typedef struct {
    volatile uint16_t SR;   uint16_t RESERVED0; /* 0x00 */
    volatile uint16_t DR;   uint16_t RESERVED1; /* 0x04 */
    volatile uint16_t BRR;  uint16_t RESERVED2; /* 0x08 */
    volatile uint16_t CR1;  uint16_t RESERVED3; /* 0x0C */
    volatile uint16_t CR2;  uint16_t RESERVED4; /* 0x10 */
    volatile uint16_t CR3;  uint16_t RESERVED5; /* 0x14 */
    volatile uint16_t GTPR; uint16_t RESERVED6; /* 0x18 */
} USART_TypeDef;

#define USART1 ((USART_TypeDef *)0x40013800U)
#define USART2 ((USART_TypeDef *)0x40004400U)
#define USART3 ((USART_TypeDef *)0x40004800U)
#define UART4  ((USART_TypeDef *)0x40004C00U)
#define UART5  ((USART_TypeDef *)0x40005000U)

typedef struct {
    uint32_t USART_BaudRate;
    uint16_t USART_WordLength;
    uint16_t USART_StopBits;
    uint16_t USART_Parity;
    uint16_t USART_Mode;
    uint16_t USART_HardwareFlowControl;
    uint16_t PAD;
} USART_InitTypeDef;

typedef struct {
    uint16_t GPIO_Pin;
    uint8_t  GPIO_Speed;
    uint8_t  GPIO_Mode;
} GPIO_InitTypeDef;

typedef struct {
    uint16_t TIM_Prescaler;
    uint16_t TIM_CounterMode;
    uint16_t TIM_Period;
    uint16_t TIM_ClockDivision;
    uint8_t  TIM_RepetitionCounter;
    uint8_t  PAD;
} TIM_TimeBaseInitTypeDef;

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

#define GPIOA_BASE ((void *)0x40010800U)
#define GPIOB_BASE ((void *)0x40010C00U)
#define GPIOE_BASE ((void *)0x40011800U)
#define TIM3_BASE  ((void *)0x40000400U)
#define TIM6_BASE  ((void *)0x40001000U)
#define TIM7_BASE  ((void *)0x40001400U)

#define DMA1_CH2_BASE  ((void *)0x4002001CU) /* USART3_TX */
#define DMA1_CH3_BASE  ((void *)0x40020030U) /* USART3_RX */
#define DMA1_CH4_BASE  ((void *)0x40020044U) /* USART1_TX */
#define DMA1_CH5_BASE  ((void *)0x40020058U) /* USART1_RX */
#define DMA1_CH6_BASE  ((void *)0x4002006CU) /* USART2_RX */
#define DMA1_CH7_BASE  ((void *)0x40020080U) /* USART2_TX */

#define DMA1_CH2_CNDTR MMIO32(0x40020020U)
#define DMA1_CH3_CNDTR MMIO32(0x40020034U)
#define DMA1_CH4_CNDTR MMIO32(0x40020048U)
#define DMA1_CH5_CNDTR MMIO32(0x4002005CU)
#define DMA1_CH6_CNDTR MMIO32(0x40020070U)
#define DMA1_CH7_CNDTR MMIO32(0x40020084U)

#define TIM6_CR1       MMIO16(0x40001000U)
#define TIM6_CNT       MMIO16(0x40001024U)
#define TIM7_CR1       MMIO16(0x40001400U)
#define TIM7_CNT       MMIO16(0x40001424U)

/* External BSP & Timer functions */
extern void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, uint8_t NewState);
extern void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, uint8_t NewState);
extern void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, uint8_t NewState);
extern void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, uint8_t NewState);
extern void RCC_GetClocksFreq(RCC_ClocksTypeDef *RCC_Clocks);
extern void GPIO_Init(void *GPIOx, const GPIO_InitTypeDef *GPIO_InitStruct);
extern void GPIO_SetBits(void *GPIOx, uint16_t GPIO_Pin);
extern void GPIO_ResetBits(void *GPIOx, uint16_t GPIO_Pin);
extern void NVIC_Init(const NVIC_InitTypeDef *NVIC_InitStruct);
extern void DMA_Cmd(void *DMAy_Channelx, uint8_t NewState);
extern void DMA_ClearFlag(uint32_t DMAy_FLAG);
extern void USART1_DMA_NVIC_Config(void);
extern void USART2_DMA_NVIC_Config(void);
extern uint16_t CRC16_Modbus(const uint8_t *buf, uint16_t len);
extern void TIM_TimeBaseInit(void *TIMx, const TIM_TimeBaseInitTypeDef *TIM_TimeBaseInitStruct);
extern void TIM_ClearFlag(void *TIMx, uint16_t TIM_FLAG);
extern void TIM_ITConfig(void *TIMx, uint16_t TIM_IT, uint8_t NewState);
extern void TIM_Cmd(void *TIMx, uint8_t NewState);

/* ==========================================================================
 * RS-485 Port 0 / Port 1 / Port 2 SRAM State Variables
 * ========================================================================== */

#define g_Port0_RxWriteIdx     MMIO8(0x200000BAU)
#define g_Port0_TxLen          MMIO16(0x200000BCU)
#define g_Port0_FrameLen       MMIO16(0x200000BEU)
#define g_Port0_TotalRxLen     MMIO16(0x200000C0U)
#define g_Port1_RxWriteIdx     MMIO8(0x200000C2U)
#define g_Port1_TxLen          MMIO16(0x200000C4U)
#define g_Port1_FrameLen       MMIO16(0x200000C6U)
#define g_Port1_TotalRxLen     MMIO16(0x200000C8U)
#define g_Port2_RxWriteIdx     MMIO8(0x200000CAU)
#define g_Port2_FrameLen       MMIO16(0x200000CEU)
#define g_Port2_TotalRxLen     MMIO16(0x200000D0U)

#define g_Port0_StationAddr    MMIO8(0x2000010DU)
#define g_Port1_StationAddr    MMIO8(0x2000010EU)
#define g_PLC_RunMode          MMIO8(0x2000016FU)
#define g_Port0_ProtocolMode   MMIO8(0x20000184U) /* 0=PPI, 1=Freeport, 2=Master/Token, 3=Modbus */
#define g_Port1_ProtocolMode   MMIO8(0x20000185U) /* 0=PPI, 1=Freeport, 2=Master/Token, 3/4=Modbus */
#define g_Port0_XmtActive      MMIO8(0x200001B0U)
#define g_Port0_RcvState       MMIO8(0x200001B1U)
#define g_Port0_RcvStartPos    MMIO8(0x200001B2U)
#define g_Port0_RcvEndPos      MMIO8(0x200001B3U)
#define g_Port0_RcvIdleTimEn   MMIO8(0x200001B5U)
#define g_Port0_RcvByteCount   MMIO8(0x200001B6U)
#define g_Port0_RcvArmed       MMIO8(0x200001B7U)
#define g_Port0_PpiMsgReady    MMIO8(0x200001C1U)
#define g_Port0_TurnaroundCnt  MMIO16(0x200001D6U)
#define g_Port1_TurnaroundCnt  MMIO16(0x200001DEU)
#define g_Port2_TurnaroundCnt  MMIO16(0x200001E6U)
#define g_Port1_XmtActive      MMIO8(0x200001ECU)
#define g_Port1_RcvState       MMIO8(0x200001EDU)
#define g_Port1_RcvStartPos    MMIO8(0x200001EEU)
#define g_Port1_RcvEndPos      MMIO8(0x200001EFU)
#define g_Port1_RcvIdleTimEn   MMIO8(0x200001F1U)
#define g_Port1_RcvByteCount   MMIO8(0x200001F2U)
#define g_Port1_RcvArmed       MMIO8(0x200001F3U)
#define g_Port1_PpiMsgReady    MMIO8(0x200001FCU)

#define g_Port0_InterCharCnt   MMIO16(0x20000228U)
#define g_Port0_IdleTimerCnt   MMIO16(0x2000022AU)
#define g_Port1_InterCharCnt   MMIO16(0x2000022CU)
#define g_Port1_IdleTimerCnt   MMIO16(0x2000022EU)
#define g_Port0_BaudRate       MMIO32(0x200002C4U)
#define g_Port0_Parity         MMIO16(0x200002C8U)
#define g_Port0_WordLen        MMIO16(0x200002CAU)
#define g_Port1_BaudRate       MMIO32(0x200002CCU)
#define g_Port1_Parity         MMIO16(0x200002D0U)
#define g_Port1_WordLen        MMIO16(0x200002D2U)

#define g_SMB30_Port0Cfg       MMIO8(0x200003D9U)
#define g_SMB44_TxIdleFlags    MMIO8(0x200003E7U)
#define g_SMB86_P0RcvStatus    MMIO8(0x20000411U)
#define g_SMB87_P0RcvCtrl      MMIO8(0x20000412U)
#define g_SMB88_P0StartChar    MMIO8(0x20000413U)
#define g_SMB89_P0EndChar      MMIO8(0x20000414U)
#define g_SMW90_P0IdleTimeHi   MMIO8(0x20000415U)
#define g_SMW90_P0IdleTimeLo   MMIO8(0x20000416U)
#define g_SMW92_P0MaxTimeHi    MMIO8(0x20000417U)
#define g_SMW92_P0MaxTimeLo    MMIO8(0x20000418U)
#define g_SMB94_P0MaxChars     MMIO8(0x20000419U)

#define g_SMB186_P1RcvStatus   MMIO8(0x20000475U)
#define g_SMB187_P1RcvCtrl     MMIO8(0x20000476U)
#define g_SMB188_P1StartChar   MMIO8(0x20000477U)
#define g_SMB189_P1EndChar     MMIO8(0x20000478U)
#define g_SMW190_P1IdleTimeHi  MMIO8(0x20000479U)
#define g_SMW190_P1IdleTimeLo  MMIO8(0x2000047AU)
#define g_SMW192_P1MaxTimeHi   MMIO8(0x2000047BU)
#define g_SMW192_P1MaxTimeLo   MMIO8(0x2000047CU)
#define g_SMB194_P1MaxChars    MMIO8(0x2000047DU)

#define g_TimedIntQueueCount   MMIO8(0x200004C2U)
#define g_Port0_RcvBufPtr      MMIO32(0x200004DCU)
#define g_Port1_RcvBufPtr      MMIO32(0x200004E0U)
#define g_TimedIntQueue        ((volatile uint8_t *)0x20000B50U)

#define g_Port2_WorkingRxBuf   ((volatile uint8_t *)0x20005100U)
#define g_Port1_WorkingRxBuf   ((volatile uint8_t *)0x20005470U)
#define g_Port0_WorkingRxBuf   ((volatile uint8_t *)0x20005670U)
#define g_SDB_Port1StationAddr MMIO8(0x200058F9U)
#define g_SDB_Port1BaudCode    MMIO8(0x200058FBU)
#define g_SDB_Port0StationAddr MMIO8(0x2000598FU)
#define g_SDB_Port0BaudCode    MMIO8(0x20005991U)

#define g_USART3_DMA_RxBuf     ((volatile uint8_t *)0x20005AC2U)
#define g_USART3_DMA_TxBuf     ((volatile uint8_t *)0x20005B42U)
#define g_USART2_DMA_RxBuf     ((volatile uint8_t *)0x20005BC2U)
#define g_USART2_DMA_TxBuf     ((volatile uint8_t *)0x20005CC2U)
#define g_USART1_DMA_RxBuf     ((volatile uint8_t *)0x20005DC2U)
#define g_USART1_DMA_TxBuf     ((volatile uint8_t *)0x20005EC2U)

/* Forward declarations */
void USART_DeInit(USART_TypeDef *USARTx);
void USART_Init(USART_TypeDef *USARTx, const USART_InitTypeDef *USART_InitStruct);
void USART_Cmd(USART_TypeDef *USARTx, uint8_t NewState);
void USART_ITConfig(USART_TypeDef *USARTx, uint16_t USART_IT, uint8_t NewState);
void USART_DMACmd(USART_TypeDef *USARTx, uint16_t USART_DMAReq, uint8_t NewState);

static inline void PLC_QueueCommInterrupt(uint8_t event_id)
{
    uint8_t q_idx = g_TimedIntQueueCount;
    for (uint8_t i = 0; i <= q_idx; i++) {
        if (g_TimedIntQueue[i] == event_id) {
            return;
        }
    }
    g_TimedIntQueue[q_idx] = event_id;
    g_TimedIntQueueCount   = (uint8_t)(q_idx + 1U);
}

/* ==========================================================================
 * Turnaround Delay & Interval Timers (TIM3, TIM6, TIM7: 0x080401b4..0x080402d3)
 * ========================================================================== */

/**
 * @brief Configures TIM3 interval timer and enables TIM3_IRQn (channel 29).
 *        Reconstructed from FUN_080401b4 @ 0x080401b4.
 */
void TIM3_IntervalConfig(uint16_t period, uint16_t prescaler)
{
    TIM_TimeBaseInitTypeDef tb_init;
    NVIC_InitTypeDef        nvic_init;

    RCC_APB1PeriphClockCmd(0x00000002UL, 1); /* TIM3 */

    tb_init.TIM_Period            = period;
    tb_init.TIM_Prescaler         = prescaler;
    tb_init.TIM_ClockDivision     = 0U;
    tb_init.TIM_CounterMode       = 0U;
    tb_init.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM3_BASE, &tb_init);
    TIM_ClearFlag(TIM3_BASE, 0x0001U);
    TIM_ITConfig(TIM3_BASE, 0x0001U, 1);
    TIM_Cmd(TIM3_BASE, 1);

    nvic_init.NVIC_IRQChannel                   = 29U; /* TIM3_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 4U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);
}

/**
 * @brief Configures TIM6 (PSC=999, ARR=35999) as the Port 0 RS-485 TX
 *        turnaround delay timer.
 *        Reconstructed from FUN_08040228 @ 0x08040228.
 */
void TIM6_TurnaroundConfig(void)
{
    TIM_TimeBaseInitTypeDef tb_init;
    NVIC_InitTypeDef        nvic_init;

    RCC_APB1PeriphClockCmd(0x00000010UL, 1); /* TIM6 */

    tb_init.TIM_Period            = 35999U;
    tb_init.TIM_Prescaler         = 999U;
    tb_init.TIM_ClockDivision     = 0U;
    tb_init.TIM_CounterMode       = 0U;
    tb_init.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM6_BASE, &tb_init);
    TIM_ClearFlag(TIM6_BASE, 0x0001U);
    TIM_ITConfig(TIM6_BASE, 0x0001U, 1);

    nvic_init.NVIC_IRQChannel                   = 54U; /* TIM6_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 3U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);
}

/**
 * @brief Configures TIM7 (PSC=999, ARR=35999) as the Port 1 RS-485 TX
 *        turnaround delay timer.
 *        Reconstructed from FUN_0804027e @ 0x0804027e.
 */
void TIM7_TurnaroundConfig(void)
{
    TIM_TimeBaseInitTypeDef tb_init;
    NVIC_InitTypeDef        nvic_init;

    RCC_APB1PeriphClockCmd(0x00000020UL, 1); /* TIM7 */

    tb_init.TIM_Period            = 35999U;
    tb_init.TIM_Prescaler         = 999U;
    tb_init.TIM_ClockDivision     = 0U;
    tb_init.TIM_CounterMode       = 0U;
    tb_init.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM7_BASE, &tb_init);
    TIM_ClearFlag(TIM7_BASE, 0x0001U);
    TIM_ITConfig(TIM7_BASE, 0x0001U, 1);

    nvic_init.NVIC_IRQChannel                   = 55U; /* TIM7_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 3U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);
}

/* ==========================================================================
 * Port 0 (USART1) Configuration & Interrupt Handler (0x080402d4 .. 0x08040e8d)
 * ========================================================================== */

/**
 * @brief Configures PA9 (USART1_TX, AF_PP) and PA10 (USART1_RX, IN_PU).
 *        Reconstructed from FUN_080402d4 @ 0x080402d4.
 */
void USART1_GPIO_Config(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(0x00004004UL, 1); /* USART1 + GPIOA */

    gpio_init.GPIO_Pin   = 0x0200U; /* PA9 */
    gpio_init.GPIO_Mode  = 0x18U;   /* GPIO_Mode_AF_PP */
    gpio_init.GPIO_Speed = 0x03U;   /* 50 MHz */
    GPIO_Init(GPIOA_BASE, &gpio_init);

    gpio_init.GPIO_Pin   = 0x0400U; /* PA10 */
    gpio_init.GPIO_Mode  = 0x48U;   /* GPIO_Mode_IPU */
    GPIO_Init(GPIOA_BASE, &gpio_init);
}

/**
 * @brief Configures USART1 baudrate, word length, stop bits, parity, DMA, and
 *        Port 0 turnaround delay counter.
 *        Reconstructed from FUN_08040e18 @ 0x08040e18.
 */
void USART1_BaudParityConfig(uint32_t baudrate, uint16_t word_length,
                             uint16_t stop_bits, uint16_t parity)
{
    USART_InitTypeDef usart_init;

    usart_init.USART_BaudRate            = baudrate;
    usart_init.USART_WordLength          = word_length;
    usart_init.USART_StopBits            = stop_bits;
    usart_init.USART_Parity              = parity;
    usart_init.USART_HardwareFlowControl = 0x0000U;
    usart_init.USART_Mode                = 0x000CU; /* Rx | Tx */

    USART_DeInit(USART1);
    USART_Init(USART1, &usart_init);
    USART_ITConfig(USART1, 0x0525U, 0);
    USART_ITConfig(USART1, 0x0010U, 1); /* USART_IT_IDLE */
    USART_ITConfig(USART1, 0x0040U, 1); /* USART_IT_TC */
    USART_DMACmd(USART1, 0x0080U, 1);   /* USART_DMAReq_Tx */
    USART_DMACmd(USART1, 0x0040U, 1);   /* USART_DMAReq_Rx */
    USART_Cmd(USART1, 1);

    g_Port0_TurnaroundCnt = (uint16_t)(35999UL - (842400UL / baudrate));
}

/**
 * @brief Port 0 RS-485 (USART1) Interrupt Handler.
 *        Reconstructed from FUN_0804031a @ 0x0804031a (2,814 bytes).
 *
 * Handles:
 *   - TC (Transmission Complete, bit 6):
 *       Switches PE14 RS-485 transceiver back to receive mode, sets SMB44 bit 6,
 *       enqueues Port 0 XMT complete interrupt (event 0x09 = 9), and disables
 *       DMA1_Channel4.
 *   - IDLE (Receiver Idle Line Detected, bit 4):
 *       Reads received frame length from DMA1_Channel5->CNDTR (256 - CNDTR) and
 *       dispatches according to Port 0 protocol mode (0x20000184):
 *         * Mode 0 (PPI Slave): parses 0x10 short frames and 0x68 SD2 variable
 *           frames, responds to 0x10 token/status polls via TIM6 turnaround, or
 *           hands off complete requests (`g_Port0_PpiMsgReady = 1`).
 *         * Mode 1 (Freeport RCV): validates SMB86..SMB94 framing (start char
 *           SMB88, end char SMB89, max length SMB94), copies payload to V-memory
 *           buffer (`g_Port0_RcvBufPtr`), and queues Port 0 RCV complete interrupt
 *           (event 0x17 = 23).
 *         * Mode 3 (Modbus RTU): verifies CRC-16 via `CRC16_Modbus` and copies
 *           valid frames into V-memory buffer.
 */
void USART1_IRQHandler(void)
{
    /* 1. Handle Transmission Complete (TC, bit 6) */
    if ((USART1->SR & 0x0040U) != 0) {
        /* Switch PE14 (Port 0 RS-485 DE/RE) LOW to Receive mode */
        GPIO_ResetBits(GPIOE_BASE, 0x4000U);
        g_SMB44_TxIdleFlags |= 0x40U;

        if (g_PLC_RunMode == 1U && g_Port0_XmtActive == 1U) {
            g_Port0_XmtActive = 0U;
            if ((g_SMB87_P0RcvCtrl & 0x80U) != 0U) {
                g_Port0_RcvArmed = 1U;
            }
            PLC_QueueCommInterrupt(0x09U); /* Event 9: Port 0 XMT Done */
        }

        USART1->SR &= (uint16_t)~0x0040U;
        DMA_Cmd(DMA1_CH4_BASE, 0);
    }

    /* 2. Handle Receiver IDLE Line Interrupt (IDLE, bit 4) */
    if ((USART1->SR & 0x0010U) == 0) {
        return;
    }

    /* Clear IDLE flag by reading SR then DR */
    (void)USART1->SR;
    (void)USART1->DR;

    DMA_Cmd(DMA1_CH5_BASE, 0);
    DMA_ClearFlag(0x00010000UL);

    uint16_t rx_chunk_len = (uint16_t)(256UL - DMA1_CH5_CNDTR);
    g_Port0_FrameLen      = rx_chunk_len;

    uint8_t write_idx = g_Port0_RxWriteIdx;
    for (uint8_t i = 0; i < (uint8_t)rx_chunk_len; i++) {
        g_Port0_WorkingRxBuf[write_idx + i] = g_USART1_DMA_RxBuf[i];
    }

    g_Port0_RxWriteIdx = (uint8_t)(write_idx + rx_chunk_len);
    g_Port0_TotalRxLen = (uint16_t)(g_Port0_TotalRxLen + rx_chunk_len);

    if (g_Port0_TotalRxLen >= 257U) {
        g_Port0_RxWriteIdx = 0U;
        g_Port0_TotalRxLen = 0U;
    }

    switch (g_Port0_ProtocolMode) {
    case 0U: {
        /* PPI Slave Mode */
        if (g_Port0_WorkingRxBuf[0] == 0x10U &&
            g_Port0_WorkingRxBuf[1] == g_Port0_StationAddr &&
            g_Port0_WorkingRxBuf[5] == 0x16U &&
            g_Port0_TotalRxLen == 6U) {
            uint8_t fcs = (uint8_t)(g_Port0_WorkingRxBuf[1] +
                                    g_Port0_WorkingRxBuf[2] +
                                    g_Port0_WorkingRxBuf[3]);
            if (fcs == g_Port0_WorkingRxBuf[4]) {
                if (g_Port0_WorkingRxBuf[3] == 0x49U) {
                    /* FDL Status Poll -> Respond with 6-byte short frame */
                    g_USART1_DMA_TxBuf[0] = 0x10U;
                    g_USART1_DMA_TxBuf[1] = g_Port0_WorkingRxBuf[2];
                    g_USART1_DMA_TxBuf[2] = g_Port0_StationAddr;
                    g_USART1_DMA_TxBuf[3] = 0x00U;
                    g_USART1_DMA_TxBuf[4] = (uint8_t)(g_USART1_DMA_TxBuf[1] + g_USART1_DMA_TxBuf[2]);
                    g_USART1_DMA_TxBuf[5] = 0x16U;
                    g_Port0_TxLen         = 6U;

                    GPIO_SetBits(GPIOE_BASE, 0x4000U);
                    DMA_Cmd(DMA1_CH5_BASE, 0);
                    DMA_ClearFlag(0x00001000UL);
                    TIM6_CNT  = g_Port0_TurnaroundCnt;
                    TIM6_CR1 |= 0x0001U;
                    g_Port0_RxWriteIdx = 0U;
                    g_Port0_TotalRxLen = 0U;
                    return;
                } else if (g_Port0_WorkingRxBuf[3] == 0x5CU) {
                    g_Port0_PpiMsgReady = 1U;
                }
            }
            g_Port0_RxWriteIdx = 0U;
            g_Port0_TotalRxLen = 0U;
        } else if (g_Port0_WorkingRxBuf[0] == 0x68U &&
                   g_Port0_WorkingRxBuf[1] == g_Port0_WorkingRxBuf[2] &&
                   g_Port0_WorkingRxBuf[3] == 0x68U &&
                   g_Port0_WorkingRxBuf[4] == g_Port0_StationAddr &&
                   g_Port0_TotalRxLen >= (uint16_t)(g_Port0_WorkingRxBuf[1] + 6U)) {
            g_Port0_PpiMsgReady = 1U;
        }
        break;
    }

    case 1U: {
        /* Freeport RCV Mode (SMB86..SMB94) */
        if (g_PLC_RunMode == 1U && g_Port0_RcvArmed == 1U && (g_SMB87_P0RcvCtrl & 0x80U) != 0U) {
            uint8_t *dst_buf = (uint8_t *)(uintptr_t)g_Port0_RcvBufPtr;
            uint8_t  copy_len = (uint8_t)g_Port0_TotalRxLen;
            if (copy_len > g_SMB94_P0MaxChars) {
                copy_len          = g_SMB94_P0MaxChars;
                g_SMB86_P0RcvStatus = 0x04U; /* Max character count reached */
            } else {
                g_SMB86_P0RcvStatus = 0x20U; /* End character / frame received */
            }
            dst_buf[0] = copy_len;
            for (uint8_t i = 0; i < copy_len; i++) {
                dst_buf[1U + i] = g_Port0_WorkingRxBuf[i];
            }
            g_Port0_RcvArmed   = 0U;
            g_Port0_RxWriteIdx = 0U;
            g_Port0_TotalRxLen = 0U;
            PLC_QueueCommInterrupt(0x17U); /* Event 23: Port 0 RCV Done */
        }
        break;
    }

    case 3U: {
        /* Modbus RTU Mode */
        if (g_Port0_TotalRxLen >= 4U) {
            uint16_t calc_crc = CRC16_Modbus((const uint8_t *)g_Port0_WorkingRxBuf,
                                             (uint16_t)(g_Port0_TotalRxLen - 2U));
            uint16_t rx_crc   = (uint16_t)g_Port0_WorkingRxBuf[g_Port0_TotalRxLen - 2U] |
                                ((uint16_t)g_Port0_WorkingRxBuf[g_Port0_TotalRxLen - 1U] << 8);
            if (calc_crc == rx_crc) {
                uint8_t *dst_buf = (uint8_t *)(uintptr_t)g_Port0_RcvBufPtr;
                if (dst_buf != NULL) {
                    dst_buf[0] = (uint8_t)g_Port0_TotalRxLen;
                    for (uint16_t i = 0; i < g_Port0_TotalRxLen; i++) {
                        dst_buf[1U + i] = g_Port0_WorkingRxBuf[i];
                    }
                }
                PLC_QueueCommInterrupt(0x17U);
            }
            g_Port0_RxWriteIdx = 0U;
            g_Port0_TotalRxLen = 0U;
        }
        break;
    }

    default:
        g_Port0_RxWriteIdx = 0U;
        g_Port0_TotalRxLen = 0U;
        break;
    }

    DMA1_CH5_CNDTR = 256UL;
    DMA_Cmd(DMA1_CH5_BASE, 1);
}

/* ==========================================================================
 * Port 1 (USART2) Configuration & Interrupt Handler (0x08040e8e .. 0x08041c0d)
 * ========================================================================== */

/**
 * @brief Configures PA2 (USART2_TX, AF_PP) and PA3 (USART2_RX, IN_FLOATING).
 *        Reconstructed from FUN_08040e8e @ 0x08040e8e.
 */
void USART2_GPIO_Config(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB1PeriphClockCmd(0x00020000UL, 1); /* USART2 */

    gpio_init.GPIO_Pin   = 0x0004U; /* PA2 */
    gpio_init.GPIO_Mode  = 0x18U;   /* GPIO_Mode_AF_PP */
    gpio_init.GPIO_Speed = 0x03U;
    GPIO_Init(GPIOA_BASE, &gpio_init);

    gpio_init.GPIO_Pin   = 0x0008U; /* PA3 */
    gpio_init.GPIO_Mode  = 0x04U;   /* GPIO_Mode_IN_FLOATING */
    GPIO_Init(GPIOA_BASE, &gpio_init);
}

/**
 * @brief Configures USART2 baudrate, word length, stop bits, parity, DMA, and
 *        Port 1 turnaround delay counter.
 *        Reconstructed from FUN_08041b98 @ 0x08041b98.
 */
void USART2_BaudParityConfig(uint32_t baudrate, uint16_t word_length,
                             uint16_t stop_bits, uint16_t parity)
{
    USART_InitTypeDef usart_init;

    usart_init.USART_BaudRate            = baudrate;
    usart_init.USART_WordLength          = word_length;
    usart_init.USART_StopBits            = stop_bits;
    usart_init.USART_Parity              = parity;
    usart_init.USART_HardwareFlowControl = 0x0000U;
    usart_init.USART_Mode                = 0x000CU;

    USART_DeInit(USART2);
    USART_Init(USART2, &usart_init);
    USART_ITConfig(USART2, 0x0525U, 0);
    USART_ITConfig(USART2, 0x0010U, 1); /* USART_IT_IDLE */
    USART_ITConfig(USART2, 0x0040U, 1); /* USART_IT_TC */
    USART_DMACmd(USART2, 0x0080U, 1);   /* USART_DMAReq_Tx */
    USART_DMACmd(USART2, 0x0040U, 1);   /* USART_DMAReq_Rx */
    USART_Cmd(USART2, 1);

    g_Port1_TurnaroundCnt = (uint16_t)(35999UL - (842400UL / baudrate));
}

/**
 * @brief Port 1 RS-485 (USART2) Interrupt Handler.
 *        Reconstructed from FUN_08040ece @ 0x08040ece (3,274 bytes).
 */
void USART2_IRQHandler(void)
{
    /* 1. Handle Transmission Complete (TC, bit 6) */
    if ((USART2->SR & 0x0040U) != 0) {
        g_SMB44_TxIdleFlags |= 0x80U;

        if (g_PLC_RunMode == 1U && g_Port1_XmtActive == 1U) {
            g_Port1_XmtActive = 0U;
            if ((g_SMB187_P1RcvCtrl & 0x80U) != 0U) {
                g_Port1_RcvArmed = 1U;
            }
            PLC_QueueCommInterrupt(0x0AU); /* Event 10: Port 1 XMT Done */
        }

        USART2->SR &= (uint16_t)~0x0040U;
        DMA_Cmd(DMA1_CH7_BASE, 0);
    }

    /* 2. Handle Receiver IDLE Line Interrupt (IDLE, bit 4) */
    if ((USART2->SR & 0x0010U) == 0) {
        return;
    }

    (void)USART2->SR;
    (void)USART2->DR;

    DMA_Cmd(DMA1_CH6_BASE, 0);
    DMA_ClearFlag(0x00100000UL);

    uint16_t rx_chunk_len = (uint16_t)(256UL - DMA1_CH6_CNDTR);
    g_Port1_FrameLen      = rx_chunk_len;

    uint8_t write_idx = g_Port1_RxWriteIdx;
    for (uint8_t i = 0; i < (uint8_t)rx_chunk_len; i++) {
        g_Port1_WorkingRxBuf[write_idx + i] = g_USART2_DMA_RxBuf[i];
    }

    g_Port1_RxWriteIdx = (uint8_t)(write_idx + rx_chunk_len);
    g_Port1_TotalRxLen = (uint16_t)(g_Port1_TotalRxLen + rx_chunk_len);

    if (g_Port1_TotalRxLen >= 257U) {
        g_Port1_RxWriteIdx = 0U;
        g_Port1_TotalRxLen = 0U;
    }

    switch (g_Port1_ProtocolMode) {
    case 0U: {
        /* PPI Slave Mode on Port 1 */
        if (g_Port1_WorkingRxBuf[0] == 0x10U &&
            g_Port1_WorkingRxBuf[1] == g_Port1_StationAddr &&
            g_Port1_WorkingRxBuf[5] == 0x16U &&
            g_Port1_TotalRxLen == 6U) {
            uint8_t fcs = (uint8_t)(g_Port1_WorkingRxBuf[1] +
                                    g_Port1_WorkingRxBuf[2] +
                                    g_Port1_WorkingRxBuf[3]);
            if (fcs == g_Port1_WorkingRxBuf[4]) {
                if (g_Port1_WorkingRxBuf[3] == 0x49U) {
                    g_USART2_DMA_TxBuf[0] = 0x10U;
                    g_USART2_DMA_TxBuf[1] = g_Port1_WorkingRxBuf[2];
                    g_USART2_DMA_TxBuf[2] = g_Port1_StationAddr;
                    g_USART2_DMA_TxBuf[3] = 0x00U;
                    g_USART2_DMA_TxBuf[4] = (uint8_t)(g_USART2_DMA_TxBuf[1] + g_USART2_DMA_TxBuf[2]);
                    g_USART2_DMA_TxBuf[5] = 0x16U;
                    g_Port1_TxLen         = 6U;

                    DMA_Cmd(DMA1_CH6_BASE, 0);
                    DMA_ClearFlag(0x01000000UL);
                    TIM7_CNT  = g_Port1_TurnaroundCnt;
                    TIM7_CR1 |= 0x0001U;
                    g_Port1_RxWriteIdx = 0U;
                    g_Port1_TotalRxLen = 0U;
                    return;
                } else if (g_Port1_WorkingRxBuf[3] == 0x5CU) {
                    g_Port1_PpiMsgReady = 1U;
                }
            }
            g_Port1_RxWriteIdx = 0U;
            g_Port1_TotalRxLen = 0U;
        } else if (g_Port1_WorkingRxBuf[0] == 0x68U &&
                   g_Port1_WorkingRxBuf[1] == g_Port1_WorkingRxBuf[2] &&
                   g_Port1_WorkingRxBuf[3] == 0x68U &&
                   g_Port1_WorkingRxBuf[4] == g_Port1_StationAddr &&
                   g_Port1_TotalRxLen >= (uint16_t)(g_Port1_WorkingRxBuf[1] + 6U)) {
            g_Port1_PpiMsgReady = 1U;
        }
        break;
    }

    case 1U: {
        /* Freeport RCV Mode (SMB186..SMB194) */
        if (g_PLC_RunMode == 1U && g_Port1_RcvArmed == 1U && (g_SMB187_P1RcvCtrl & 0x80U) != 0U) {
            uint8_t *dst_buf  = (uint8_t *)(uintptr_t)g_Port1_RcvBufPtr;
            uint8_t  copy_len = (uint8_t)g_Port1_TotalRxLen;
            if (copy_len > g_SMB194_P1MaxChars) {
                copy_len            = g_SMB194_P1MaxChars;
                g_SMB186_P1RcvStatus = 0x04U;
            } else {
                g_SMB186_P1RcvStatus = 0x20U;
            }
            dst_buf[0] = copy_len;
            for (uint8_t i = 0; i < copy_len; i++) {
                dst_buf[1U + i] = g_Port1_WorkingRxBuf[i];
            }
            g_Port1_RcvArmed   = 0U;
            g_Port1_RxWriteIdx = 0U;
            g_Port1_TotalRxLen = 0U;
            PLC_QueueCommInterrupt(0x18U); /* Event 24: Port 1 RCV Done */
        }
        break;
    }

    case 3U:
    case 4U: {
        /* Modbus RTU Mode on Port 1 */
        if (g_Port1_TotalRxLen >= 4U) {
            uint16_t calc_crc = CRC16_Modbus((const uint8_t *)g_Port1_WorkingRxBuf,
                                             (uint16_t)(g_Port1_TotalRxLen - 2U));
            uint16_t rx_crc   = (uint16_t)g_Port1_WorkingRxBuf[g_Port1_TotalRxLen - 2U] |
                                ((uint16_t)g_Port1_WorkingRxBuf[g_Port1_TotalRxLen - 1U] << 8);
            if (calc_crc == rx_crc) {
                uint8_t *dst_buf = (uint8_t *)(uintptr_t)g_Port1_RcvBufPtr;
                if (dst_buf != NULL) {
                    dst_buf[0] = (uint8_t)g_Port1_TotalRxLen;
                    for (uint16_t i = 0; i < g_Port1_TotalRxLen; i++) {
                        dst_buf[1U + i] = g_Port1_WorkingRxBuf[i];
                    }
                }
                PLC_QueueCommInterrupt(0x18U);
            }
            g_Port1_RxWriteIdx = 0U;
            g_Port1_TotalRxLen = 0U;
        }
        break;
    }

    default:
        g_Port1_RxWriteIdx = 0U;
        g_Port1_TotalRxLen = 0U;
        break;
    }

    DMA1_CH6_CNDTR = 256UL;
    DMA_Cmd(DMA1_CH6_BASE, 1);
}

/* ==========================================================================
 * Expansion Bus (USART3) Configuration & Interrupt Handler (0x08041c0e .. 0x08041fa9)
 * ========================================================================== */

/**
 * @brief Configures PB10 (USART3_TX, AF_PP) and PB11 (USART3_RX, IN_FLOATING).
 *        Reconstructed from FUN_08041c0e @ 0x08041c0e.
 */
void USART3_GPIO_Config(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB1PeriphClockCmd(0x00040000UL, 1); /* USART3 */
    RCC_APB2PeriphClockCmd(0x00000008UL, 1); /* GPIOB */

    gpio_init.GPIO_Pin   = 0x0400U; /* PB10 */
    gpio_init.GPIO_Mode  = 0x18U;   /* GPIO_Mode_AF_PP */
    gpio_init.GPIO_Speed = 0x03U;
    GPIO_Init(GPIOB_BASE, &gpio_init);

    gpio_init.GPIO_Pin   = 0x0800U; /* PB11 */
    gpio_init.GPIO_Mode  = 0x04U;   /* GPIO_Mode_IN_FLOATING */
    GPIO_Init(GPIOB_BASE, &gpio_init);
}

/**
 * @brief Configures USART3 baudrate, word length, stop bits, parity, and DMA.
 *        Reconstructed from FUN_08041f44 @ 0x08041f44.
 */
void USART3_BaudParityConfig(uint32_t baudrate, uint16_t word_length,
                             uint16_t stop_bits, uint16_t parity)
{
    USART_InitTypeDef usart_init;

    usart_init.USART_BaudRate            = baudrate;
    usart_init.USART_WordLength          = word_length;
    usart_init.USART_StopBits            = stop_bits;
    usart_init.USART_Parity              = parity;
    usart_init.USART_HardwareFlowControl = 0x0000U;
    usart_init.USART_Mode                = 0x000CU;

    USART_DeInit(USART3);
    USART_Init(USART3, &usart_init);
    USART_ITConfig(USART3, 0x0525U, 0);
    USART_ITConfig(USART3, 0x0010U, 1); /* USART_IT_IDLE */
    USART_ITConfig(USART3, 0x0040U, 1); /* USART_IT_TC */
    USART_DMACmd(USART3, 0x0080U, 1);   /* USART_DMAReq_Tx */
    USART_DMACmd(USART3, 0x0040U, 1);   /* USART_DMAReq_Rx */
    USART_Cmd(USART3, 1);

    g_Port2_TurnaroundCnt = (uint16_t)(35999UL - (842400UL / baudrate));
}

/**
 * @brief Expansion Module Bus (USART3) Interrupt Handler.
 *        Reconstructed from FUN_08041c66 @ 0x08041c66 (734 bytes).
 */
void USART3_IRQHandler(void)
{
    if ((USART3->SR & 0x0040U) != 0) {
        USART3->SR &= (uint16_t)~0x0040U;
        DMA_Cmd(DMA1_CH2_BASE, 0);
    }

    if ((USART3->SR & 0x0010U) != 0) {
        (void)USART3->SR;
        (void)USART3->DR;

        DMA_Cmd(DMA1_CH3_BASE, 0);
        DMA_ClearFlag(0x00000100UL);

        uint16_t rx_chunk_len = (uint16_t)(64UL - DMA1_CH3_CNDTR);
        g_Port2_FrameLen      = rx_chunk_len;

        uint8_t write_idx = g_Port2_RxWriteIdx;
        for (uint8_t i = 0; i < (uint8_t)rx_chunk_len; i++) {
            g_Port2_WorkingRxBuf[write_idx + i] = g_USART3_DMA_RxBuf[i];
        }

        g_Port2_RxWriteIdx = (uint8_t)(write_idx + rx_chunk_len);
        g_Port2_TotalRxLen = (uint16_t)(g_Port2_TotalRxLen + rx_chunk_len);

        if (g_Port2_TotalRxLen >= 65U) {
            g_Port2_RxWriteIdx = 0U;
            g_Port2_TotalRxLen = 0U;
        }

        DMA1_CH3_CNDTR = 64UL;
        DMA_Cmd(DMA1_CH3_BASE, 1);
    }
}

/* ==========================================================================
 * USART Standard Peripheral Library Functions (0x08041faa .. 0x0804211b)
 * ========================================================================== */

/**
 * @brief Enables or disables the specified USART peripheral.
 *        Reconstructed from FUN_08041faa @ 0x08041faa.
 */
void USART_Cmd(USART_TypeDef *USARTx, uint8_t NewState)
{
    if (NewState != 0) {
        USARTx->CR1 |= 0x2000U;
    } else {
        USARTx->CR1 &= 0xDFFFU;
    }
}

/**
 * @brief Enables or disables the USART's DMA interface.
 *        Reconstructed from FUN_08041fc4 @ 0x08041fc4.
 */
void USART_DMACmd(USART_TypeDef *USARTx, uint16_t USART_DMAReq, uint8_t NewState)
{
    if (NewState != 0) {
        USARTx->CR3 |= USART_DMAReq;
    } else {
        USARTx->CR3 &= (uint16_t)~USART_DMAReq;
    }
}

/**
 * @brief Deinitializes the USARTx peripheral registers to their default reset values.
 *        Reconstructed from FUN_08041fd8 @ 0x08041fd8.
 */
void USART_DeInit(USART_TypeDef *USARTx)
{
    if (USARTx == USART1) {
        RCC_APB2PeriphResetCmd(0x00004000UL, 1);
        RCC_APB2PeriphResetCmd(0x00004000UL, 0);
    } else if (USARTx == USART2) {
        RCC_APB1PeriphResetCmd(0x00020000UL, 1);
        RCC_APB1PeriphResetCmd(0x00020000UL, 0);
    } else if (USARTx == USART3) {
        RCC_APB1PeriphResetCmd(0x00040000UL, 1);
        RCC_APB1PeriphResetCmd(0x00040000UL, 0);
    } else if (USARTx == UART4) {
        RCC_APB1PeriphResetCmd(0x00080000UL, 1);
        RCC_APB1PeriphResetCmd(0x00080000UL, 0);
    } else if (USARTx == UART5) {
        RCC_APB1PeriphResetCmd(0x00100000UL, 1);
        RCC_APB1PeriphResetCmd(0x00100000UL, 0);
    }
}

/**
 * @brief Enables or disables the specified USART interrupts.
 *        Reconstructed from FUN_0804202e @ 0x0804202e.
 */
void USART_ITConfig(USART_TypeDef *USARTx, uint16_t USART_IT, uint8_t NewState)
{
    uint32_t reg_idx = (uint32_t)(USART_IT >> 5) & 0x03UL;
    uint32_t mask    = 1UL << (USART_IT & 0x1FU);
    volatile uint16_t *reg = (reg_idx == 1U) ? &USARTx->CR1 :
                             (reg_idx == 2U) ? &USARTx->CR2 : &USARTx->CR3;

    if (NewState != 0) {
        *reg |= (uint16_t)mask;
    } else {
        *reg &= (uint16_t)~mask;
    }
}

/**
 * @brief Initializes the USARTx peripheral according to the specified parameters.
 *        Reconstructed from FUN_08042072 @ 0x08042072.
 */
void USART_Init(USART_TypeDef *USARTx, const USART_InitTypeDef *USART_InitStruct)
{
    RCC_ClocksTypeDef clocks;

    USARTx->CR2 = (USARTx->CR2 & 0xCFFFU) | USART_InitStruct->USART_StopBits;
    USARTx->CR1 = (USARTx->CR1 & 0xE9F3U) |
                  USART_InitStruct->USART_WordLength |
                  USART_InitStruct->USART_Parity |
                  USART_InitStruct->USART_Mode;
    USARTx->CR3 = (USARTx->CR3 & 0xFCFFU) | USART_InitStruct->USART_HardwareFlowControl;

    RCC_GetClocksFreq(&clocks);
    uint32_t apbclock = (USARTx == USART1) ? clocks.RCC_PCLK2_Frequency
                                           : clocks.RCC_PCLK1_Frequency;

    uint32_t integerdivider    = (apbclock * 25UL) / (USART_InitStruct->USART_BaudRate * 4UL);
    uint32_t tmpreg            = (integerdivider / 100UL) << 4;
    uint32_t fractionaldivider = integerdivider - ((tmpreg >> 4) * 100UL);

    USARTx->BRR = (uint16_t)(tmpreg | (((fractionaldivider * 16UL + 50UL) / 100UL) & 0x0FUL));
}

/* ==========================================================================
 * High-Level Port 0 (USART1) and Port 1 (USART2) Initialization (0x0804211c .. 0x080422c5)
 * ========================================================================== */

/**
 * @brief Initializes RS-485 Port 0 (USART1 + DMA1_Ch4/5 + TIM6) using station
 *        address and baudrate code from the System Data Block (SDB).
 *        Reconstructed from FUN_0804211c @ 0x0804211c.
 */
void Port0_USART1_Init(void)
{
    USART1_GPIO_Config();
    USART1_DMA_NVIC_Config();

    g_Port0_StationAddr = g_SDB_Port0StationAddr;
    if (g_Port0_StationAddr >= 127U) {
        g_Port0_StationAddr  = 2U;
        g_SDB_Port0StationAddr = 2U;
    }

    uint8_t baud_code = g_SDB_Port0BaudCode;
    if (baud_code == 0U) {
        g_Port0_BaudRate = 9600UL;
        g_SMB30_Port0Cfg = 0x09U;
    } else if (baud_code == 2U) {
        g_Port0_BaudRate = 19200UL;
        g_SMB30_Port0Cfg = 0x0AU;
    } else if (baud_code == 4U) {
        g_Port0_BaudRate = 187500UL;
        g_SMB30_Port0Cfg = 0x0FU;
    } else {
        g_Port0_BaudRate    = 9600UL;
        g_SDB_Port0BaudCode = 0U;
    }

    g_Port0_Parity  = 0x0400U; /* USART_Parity_Even */
    g_Port0_WordLen = 0x1000U; /* USART_WordLength_9b (8E1) */

    USART1_BaudParityConfig(g_Port0_BaudRate, g_Port0_WordLen, 0x0000U, g_Port0_Parity);
    TIM6_TurnaroundConfig();
}

/**
 * @brief Initializes RS-485 Port 1 (USART2 + DMA1_Ch6/7 + TIM7) using station
 *        address and baudrate code from the System Data Block (SDB).
 *        Reconstructed from FUN_080421f0 @ 0x080421f0.
 */
void Port1_USART2_Init(void)
{
    USART2_GPIO_Config();
    USART2_DMA_NVIC_Config();

    g_Port1_StationAddr = g_SDB_Port1StationAddr;
    if (g_Port1_StationAddr >= 127U) {
        g_Port1_StationAddr  = 2U;
        g_SDB_Port1StationAddr = 2U;
    }

    uint8_t baud_code = g_SDB_Port1BaudCode;
    if (baud_code == 0U) {
        g_Port1_BaudRate = 9600UL;
    } else if (baud_code == 2U) {
        g_Port1_BaudRate = 19200UL;
    } else if (baud_code == 4U) {
        g_Port1_BaudRate = 187500UL;
    } else {
        g_Port1_BaudRate    = 9600UL;
        g_SDB_Port1BaudCode = 0U;
    }

    g_Port1_Parity  = 0x0400U; /* USART_Parity_Even */
    g_Port1_WordLen = 0x1000U; /* USART_WordLength_9b (8E1) */

    USART2_BaudParityConfig(g_Port1_BaudRate, g_Port1_WordLen, 0x0000U, g_Port1_Parity);
    TIM7_TurnaroundConfig();
}
