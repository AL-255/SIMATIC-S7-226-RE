/**
 * @file plc_timers_pto.c
 * @brief Hardware Timer / PTO / PWM / High-Speed Counter (HSC) / SysTick Subsystem
 *        for the SIMATIC S7-226 (APM32E103ZET6) PLC Firmware.
 *
 * Reconstructed from firmware address range 0x0803dfde .. 0x080401b3:
 *   - 0x0803dfde : SysTick_Handler (1 ms system tick, T32/T96/T160/T224 1 ms PLC
 *                  timers, SMB34/SMB35 timed interrupts, HSC0/HSC1 TIM3/TIM4
 *                  quadrature/up-down accumulators, and PTO0/PTO1 frequency ramp
 *                  envelope generators)
 *   - 0x0803f1b6 .. 0x0803f29b : TIM_OC1Init, TIM_OC2Init
 *   - 0x0803f29c .. 0x0803f3fd : PTO0_TIM1_TIM2_Config (Q0.0 pulse output & slave counter)
 *   - 0x0803f3fe .. 0x0803f58d : TIM1_UP_IRQHandler (PTO0 shadow register reload & stop)
 *   - 0x0803f58e .. 0x0803f67f : TIM2_IRQHandler (PTO0 60,000-pulse overflow & target compare)
 *   - 0x0803f680 .. 0x0803f6b1 : TIM3_IRQHandler (HSC0 16-bit overflow/underflow tracker)
 *   - 0x0803f6b2 .. 0x0803f6e3 : TIM4_IRQHandler (HSC1 16-bit overflow/underflow tracker)
 *   - 0x0803f6e4 .. 0x0803f7fb : TIM5_IRQHandler (PTO1 60,000-pulse overflow & target compare)
 *   - 0x0803f7fc .. 0x0803f849 : TIM6_IRQHandler (Port 0 RS-485 TX turnaround delay completion)
 *   - 0x0803f84a .. 0x0803f8ab : TIM7_IRQHandler (Port 1 RS-485 TX turnaround delay completion)
 *   - 0x0803f8ac .. 0x0803fa25 : PTO1_TIM8_TIM5_Config (Q0.1 pulse output & slave counter)
 *   - 0x0803fa26 .. 0x0803fbd9 : TIM8_UP_IRQHandler (PTO1 shadow register reload & stop)
 *   - 0x0803fbda .. 0x0804007d : TIM1..TIM8 SPL driver routines
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ==========================================================================
 * Hardware Register Definitions (TIM1..TIM8, EXTI, DMA1)
 * ========================================================================== */

#define MMIO8(addr)  (*(volatile uint8_t *)(uintptr_t)(addr))
#define MMIO16(addr) (*(volatile uint16_t *)(uintptr_t)(addr))
#define MMIO32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

typedef struct {
    volatile uint16_t CR1;   uint16_t RESERVED0; /* 0x00 */
    volatile uint16_t CR2;   uint16_t RESERVED1; /* 0x04 */
    volatile uint16_t SMCR;  uint16_t RESERVED2; /* 0x08 */
    volatile uint16_t DIER;  uint16_t RESERVED3; /* 0x0C */
    volatile uint16_t SR;    uint16_t RESERVED4; /* 0x10 */
    volatile uint16_t EGR;   uint16_t RESERVED5; /* 0x14 */
    volatile uint16_t CCMR1; uint16_t RESERVED6; /* 0x18 */
    volatile uint16_t CCMR2; uint16_t RESERVED7; /* 0x1C */
    volatile uint16_t CCER;  uint16_t RESERVED8; /* 0x20 */
    volatile uint16_t CNT;   uint16_t RESERVED9; /* 0x24 */
    volatile uint16_t PSC;   uint16_t RESERVED10;/* 0x28 */
    volatile uint16_t ARR;   uint16_t RESERVED11;/* 0x2C */
    volatile uint16_t RCR;   uint16_t RESERVED12;/* 0x30 */
    volatile uint16_t CCR1;  uint16_t RESERVED13;/* 0x34 */
    volatile uint16_t CCR2;  uint16_t RESERVED14;/* 0x38 */
    volatile uint16_t CCR3;  uint16_t RESERVED15;/* 0x3C */
    volatile uint16_t CCR4;  uint16_t RESERVED16;/* 0x40 */
    volatile uint16_t BDTR;  uint16_t RESERVED17;/* 0x44 */
    volatile uint16_t DCR;   uint16_t RESERVED18;/* 0x48 */
    volatile uint16_t DMAR;  uint16_t RESERVED19;/* 0x4C */
} TIM_TypeDef;

#define TIM1 ((TIM_TypeDef *)0x40012C00U)
#define TIM2 ((TIM_TypeDef *)0x40000000U)
#define TIM3 ((TIM_TypeDef *)0x40000400U)
#define TIM4 ((TIM_TypeDef *)0x40000800U)
#define TIM5 ((TIM_TypeDef *)0x40000C00U)
#define TIM6 ((TIM_TypeDef *)0x40001000U)
#define TIM7 ((TIM_TypeDef *)0x40001400U)
#define TIM8 ((TIM_TypeDef *)0x40013400U)

#define EXTI_SWIER MMIO32(0x40010410U)
#define GPIOC_IDR  MMIO32(0x40011008U)
#define GPIOE_IDR  MMIO32(0x40011808U)

typedef struct {
    uint16_t TIM_Prescaler;
    uint16_t TIM_CounterMode;
    uint16_t TIM_Period;
    uint16_t TIM_ClockDivision;
    uint8_t  TIM_RepetitionCounter;
    uint8_t  PAD;
} TIM_TimeBaseInitTypeDef;

typedef struct {
    uint16_t TIM_OCMode;
    uint16_t TIM_OutputState;
    uint16_t TIM_OutputNState;
    uint16_t TIM_Pulse;
    uint16_t TIM_OCPolarity;
    uint16_t TIM_OCNPolarity;
    uint16_t TIM_OCIdleState;
    uint16_t TIM_OCNIdleState;
} TIM_OCInitTypeDef;

typedef struct {
    uint16_t TIM_Channel;
    uint16_t TIM_ICPolarity;
    uint16_t TIM_ICSelection;
    uint16_t TIM_ICPrescaler;
    uint16_t TIM_ICFilter;
} TIM_ICInitTypeDef;

typedef struct {
    uint8_t NVIC_IRQChannel;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
    uint8_t NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

/* External BSP helpers implemented in bsp_peripherals.c */
extern void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, uint8_t NewState);
extern void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, uint8_t NewState);
extern void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, uint8_t NewState);
extern void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, uint8_t NewState);
extern void NVIC_Init(const NVIC_InitTypeDef *NVIC_InitStruct);
extern uint8_t GPIO_ReadInputDataBit(void *GPIOx, uint16_t GPIO_Pin);

/* ==========================================================================
 * PLC Timer, HSC, and PTO/PWM SRAM State Variables (0x20000037 .. 0x20006167)
 * ========================================================================== */

#define g_PTO0_BusyFlag        MMIO8(0x20000037U)
#define g_PTO1_BusyFlag        MMIO8(0x20000038U)
#define g_PTO0_Overflow60k     MMIO32(0x20000084U)
#define g_PTO0_OverflowShadow  MMIO32(0x20000088U)
#define g_PTO0_CurrFreqScaled  MMIO32(0x2000008CU)
#define g_PTO0_PrevFreqScaled  MMIO32(0x20000090U)
#define g_PTO1_Overflow60k     MMIO32(0x2000009CU)
#define g_PTO1_OverflowShadow  MMIO32(0x200000A0U)
#define g_PTO1_CurrFreqScaled  MMIO32(0x200000A4U)
#define g_PTO1_PrevFreqScaled  MMIO32(0x200000A8U)
#define g_Port0_TxLen          MMIO16(0x200000BCU)
#define g_Port1_TxLen          MMIO16(0x200000C4U)
#define g_PTO0_AccelStepIdx    (*(volatile uint64_t *)(uintptr_t)0x200000E0U)
#define g_PTO0_DecelStepIdx    (*(volatile uint64_t *)(uintptr_t)0x200000E8U)
#define g_PTO1_AccelStepIdx    (*(volatile uint64_t *)(uintptr_t)0x200000F0U)
#define g_PTO1_DecelStepIdx    (*(volatile uint64_t *)(uintptr_t)0x200000F8U)
#define g_PLC_RunMode          MMIO8(0x2000016FU)
#define g_PTO0_State           MMIO8(0x2000018EU)
#define g_PTO0_StopMode        MMIO8(0x20000192U)
#define g_PTO1_State           MMIO8(0x20000195U)
#define g_PTO1_StopMode        MMIO8(0x20000199U)
#define g_SysTick_FreeRunMs    MMIO32(0x20000220U)
#define g_SysTick_RunModeMs    MMIO32(0x20000224U)
#define g_PTO0_TotalPulses     MMIO32(0x200002DCU)
#define g_PTO1_TotalPulses     MMIO32(0x200002E0U)
#define g_PTO0_PulseCount64    (*(volatile uint64_t *)(uintptr_t)0x20000380U)
#define g_PTO1_PulseCount64    (*(volatile uint64_t *)(uintptr_t)0x20000388U)
#define g_SMB34_IntervalMs     MMIO8(0x200003DDU)
#define g_SMB35_IntervalMs     MMIO8(0x200003DEU)
#define g_SMB34_CounterMs      MMIO8(0x200003DFU)
#define g_SMB35_CounterMs      MMIO8(0x200003E0U)
#define g_PTO0_HighSpeedFlag   MMIO8(0x200003EAU)
#define g_PTO0_InitFreq        MMIO32(0x200003F0U)
#define g_PTO0_StartFreq       MMIO32(0x200003F4U)
#define g_PTO0_AccelSteps      MMIO32(0x200003F8U)
#define g_PTO0_DecelSteps      MMIO32(0x200003FCU)
#define g_PTO0_ActiveFreq      MMIO32(0x20000400U)
#define g_PTO0_BaseOffset      MMIO16(0x20000406U)
#define g_PTO0_TargetFreq      MMIO32(0x2000040CU)
#define g_PTO0_SlopeCoeff      MMIO16(0x20000416U)
#define g_PTO0_DeltaFreqMax    MMIO32(0x20000418U)
#define g_PTO0_DeltaFreqMin    MMIO32(0x2000041CU)
#define g_PTO1_HighSpeedFlag   MMIO8(0x20000426U)
#define g_PTO1_InitFreq        MMIO32(0x2000042CU)
#define g_PTO1_StartFreq       MMIO32(0x20000430U)
#define g_PTO1_AccelSteps      MMIO32(0x20000434U)
#define g_PTO1_DecelSteps      MMIO32(0x20000438U)
#define g_PTO1_ActiveFreq      MMIO32(0x2000043CU)
#define g_PTO1_BaseOffset      MMIO16(0x20000442U)
#define g_PTO1_TargetFreq      MMIO32(0x20000448U)
#define g_PTO1_SlopeCoeff      MMIO16(0x20000452U)
#define g_PTO1_DeltaFreqMax    MMIO32(0x20000454U)
#define g_PTO1_DeltaFreqMin    MMIO32(0x20000458U)
#define g_TimedIntQueueCount   MMIO8(0x200004C2U)
#define g_HSC_IntTriggerType   MMIO8(0x200004C8U)
#define g_PTO0_IntDoneFlag     MMIO8(0x200004CEU)
#define g_PTO1_IntDoneFlag     MMIO8(0x200004CFU)
#define g_PTO0_ShadowARR       MMIO16(0x20000534U)
#define g_PTO0_ShadowPSC       MMIO16(0x20000536U)
#define g_PTO0_ShadowCCR1      MMIO16(0x20000538U)
#define g_PTO1_ShadowARR       MMIO16(0x2000053AU)
#define g_PTO1_ShadowPSC       MMIO16(0x2000053CU)
#define g_PTO1_ShadowCCR1      MMIO16(0x2000053EU)
#define g_PTO0_TargetPulses1   MMIO32(0x20000540U)
#define g_PTO0_TargetPulses2   MMIO32(0x20000544U)
#define g_PTO1_TargetPulses1   MMIO32(0x20000548U)
#define g_PTO1_TargetPulses2   MMIO32(0x2000054CU)
#define g_HSC0_PresetVal       MMIO32(0x20000B24U)
#define g_HSC1_PresetVal       MMIO32(0x20000B28U)
#define g_HSC0_SoftOverflow    MMIO32(0x20000B2CU)
#define g_HSC1_SoftOverflow    MMIO32(0x20000B30U)
#define g_HSC0_PrevDirBit      MMIO8(0x20000B34U)
#define g_HSC1_PrevDirBit      MMIO8(0x20000B35U)
#define g_HSC0_DirChangeArmed  MMIO8(0x20000B3AU)
#define g_HSC1_DirChangeArmed  MMIO8(0x20000B3BU)
#define g_HSC0_Mode            MMIO8(0x20000B3CU)
#define g_HSC1_Mode            MMIO8(0x20000B3DU)
#define g_HSC0_ResetLevel      MMIO8(0x20000B40U)
#define g_HSC0_DirLevel        MMIO8(0x20000B42U)
#define g_HSC1_DirLevel        MMIO8(0x20000B43U)
#define g_TimedIntQueue        ((volatile uint8_t *)0x20000B50U)
#define g_PLC_TimerCurrVal     ((volatile uint16_t *)0x20004000U)
#define g_PLC_TimerStatus      ((volatile uint8_t *)0x2000422CU)
#define g_PLC_TimerPresetVal   ((volatile uint16_t *)0x20005E12U)
#define g_HSC0_CurrVal         MMIO32(0x20006160U) /* SMD38 */
#define g_HSC1_CurrVal         MMIO32(0x20006164U) /* SMD48 */

/* Forward declarations of SPL functions */
void TIM_DeInit(TIM_TypeDef *TIMx);
void TIM_TimeBaseInit(TIM_TypeDef *TIMx, const TIM_TimeBaseInitTypeDef *TIM_TimeBaseInitStruct);
void TIM_OC1Init(TIM_TypeDef *TIMx, const TIM_OCInitTypeDef *TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef *TIMx, const TIM_OCInitTypeDef *TIM_OCInitStruct);
void TIM_ICInit(TIM_TypeDef *TIMx, const TIM_ICInitTypeDef *TIM_ICInitStruct);
void TIM_Cmd(TIM_TypeDef *TIMx, uint8_t NewState);
void TIM_CtrlPWMOutputs(TIM_TypeDef *TIMx, uint8_t NewState);
void TIM_ITConfig(TIM_TypeDef *TIMx, uint16_t TIM_IT, uint8_t NewState);
void TIM_ClearFlag(TIM_TypeDef *TIMx, uint16_t TIM_FLAG);
void TIM_ARRPreloadConfig(TIM_TypeDef *TIMx, uint8_t NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef *TIMx, uint16_t TIM_OCPreload);
void TIM_SelectOutputTrigger(TIM_TypeDef *TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectInputTrigger(TIM_TypeDef *TIMx, uint16_t TIM_InputTriggerSource);
void TIM_SelectSlaveMode_External1(TIM_TypeDef *TIMx);
void TIM_SelectMasterSlaveMode(TIM_TypeDef *TIMx, uint16_t TIM_MasterSlaveMode);

/* ==========================================================================
 * Helper: Enqueue Timed / High-Speed Timer Interrupt Event
 * ========================================================================== */

static inline void PLC_QueueTimedInterrupt(uint8_t event_id)
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
 * Helper: Update PTO Shadow Timer Registers (ARR, PSC, CCR1) from Scaled Frequency
 * ========================================================================== */

static void PTO_ComputeShadowTimings(uint32_t freq_scaled,
                                     volatile uint16_t *out_arr,
                                     volatile uint16_t *out_psc,
                                     volatile uint16_t *out_ccr1)
{
    uint32_t arr;
    if ((freq_scaled - 2000UL) < 0x30570UL) {
        /* Medium frequency range (20 Hz .. 2 kHz): PSC = 59 (0x3B) */
        arr      = (120000000UL / freq_scaled) - 1UL;
        *out_arr = (uint16_t)arr;
        *out_psc = 0x003BU;
    } else if ((freq_scaled - 0x30D41UL) < 19799999UL) {
        /* High frequency range (> 2 kHz .. 200 kHz): PSC = 0 */
        uint32_t f_div100 = freq_scaled / 100UL;
        arr      = (168000000UL / f_div100) - 1UL;
        *out_arr = (uint16_t)arr;
        *out_psc = 0x0000U;
    } else {
        /* Low frequency range (< 20 Hz): PSC = 1199 (0x04AF) */
        arr      = (6000000UL / freq_scaled) - 1UL;
        *out_arr = (uint16_t)arr;
        *out_psc = 0x04AFU;
    }
    /* 50% duty cycle square wave */
    *out_ccr1 = (uint16_t)(((arr & 0xFFFFUL) + 1UL) >> 1);
}

/* ==========================================================================
 * 1 ms System Tick Handler (SysTick_Handler @ 0x0803dfde, 3,330 bytes)
 * ========================================================================== */

/**
 * @brief 1 ms System Tick ISR.
 *        Reconstructed from FUN_0803dfde @ 0x0803dfde.
 *
 * Responsibilities:
 *   1. Increments 1 ms free-running counter (0x20000220) and RUN-mode counter (0x20000224).
 *   2. Updates the four 1 ms S7-200 high-speed timers (T32, T96, T160, T224) and
 *      enqueues T32 (event 0x16 = 22) / T96 (event 0x17 = 23) interrupts.
 *   3. Updates SMB34 (Timed Interrupt 0, event 0x0B = 11) and SMB35 (Timed
 *      Interrupt 1, event 0x0C = 12).
 *   4. Samples hardware High-Speed Counters HSC0 (TIM3) and HSC1 (TIM4), checks
 *      CV == PV and direction-change interrupts (EXTI->SWIER |= 2), and handles
 *      external direction (PC6/PC7) and reset (PE1) pins.
 *   5. Steps the PTO0 (TIM1/TIM2 on Q0.0) and PTO1 (TIM8/TIM5 on Q0.1) pulse
 *      frequency ramp / envelope generators.
 */
void SysTick_Handler(void)
{
    g_SysTick_FreeRunMs++;

    if (g_PLC_RunMode == 1U) {
        g_SysTick_RunModeMs++;

        /* ------------------------------------------------------------------
         * 1. S7-200 1 ms Resolution Timers: T32, T96, T160, T224
         *    (Timer indices: k * 32 for k = 1, 3, 5, 7 -> 32, 96, 160, 224)
         * ------------------------------------------------------------------ */
        for (uint16_t k = 0; k < 8U; k++) {
            uint16_t t_idx  = (uint16_t)(k << 5);
            uint8_t  status = g_PLC_TimerStatus[t_idx];

            if ((status & 0x80U) == 0x80U) {
                /* TON / TOF active */
                if ((status & 0x08U) == 0U) {
                    if (g_PLC_TimerCurrVal[t_idx] < 32767U) {
                        g_PLC_TimerCurrVal[t_idx]++;
                    }
                    if (g_PLC_TimerCurrVal[t_idx] == g_PLC_TimerPresetVal[t_idx]) {
                        g_PLC_TimerStatus[t_idx] |= 0x01U;
                        if (k == 1U) {
                            PLC_QueueTimedInterrupt(0x16U); /* T32 interrupt (event 22) */
                        } else if (k == 3U) {
                            PLC_QueueTimedInterrupt(0x17U); /* T96 interrupt (event 23) */
                        }
                    }
                } else {
                    g_PLC_TimerCurrVal[t_idx] = 0U;
                }
            } else if ((status & 0x50U) == 0x50U) {
                /* TONR retentive on-delay timer active */
                if ((status & 0x08U) == 0U) {
                    if (g_PLC_TimerCurrVal[t_idx] < 32767U) {
                        g_PLC_TimerCurrVal[t_idx]++;
                    }
                    if (g_PLC_TimerCurrVal[t_idx] == g_PLC_TimerPresetVal[t_idx]) {
                        g_PLC_TimerStatus[t_idx] |= 0x01U;
                        if (k == 1U) {
                            PLC_QueueTimedInterrupt(0x16U);
                        } else if (k == 3U) {
                            PLC_QueueTimedInterrupt(0x17U);
                        }
                    }
                } else {
                    g_PLC_TimerCurrVal[t_idx] = 0U;
                    g_PLC_TimerStatus[t_idx] &= (uint8_t)~0x01U;
                }
            }
        }

        /* ------------------------------------------------------------------
         * 2. Timed Interrupts 0 & 1 (SMB34 / SMB35, 1..255 ms)
         * ------------------------------------------------------------------ */
        if (g_SMB34_IntervalMs != 0U) {
            g_SMB34_CounterMs++;
            if (g_SMB34_CounterMs >= g_SMB34_IntervalMs) {
                g_SMB34_CounterMs = 0U;
                PLC_QueueTimedInterrupt(0x0BU); /* Event 11: Timed Interrupt 0 */
            }
        }

        if (g_SMB35_IntervalMs != 0U) {
            g_SMB35_CounterMs++;
            if (g_SMB35_CounterMs >= g_SMB35_IntervalMs) {
                g_SMB35_CounterMs = 0U;
                PLC_QueueTimedInterrupt(0x0CU); /* Event 12: Timed Interrupt 1 */
            }
        }
    }

    /* ----------------------------------------------------------------------
     * 3. High-Speed Counters HSC0 (TIM3) and HSC1 (TIM4)
     * ---------------------------------------------------------------------- */
    if (g_HSC0_Mode != 0U) {
        g_HSC0_CurrVal = (uint32_t)TIM3->CNT + (g_HSC0_SoftOverflow << 16);
        if (g_HSC0_CurrVal == g_HSC0_PresetVal) {
            g_HSC_IntTriggerType = 1U;
            EXTI_SWIER |= 0x00000002UL;
        }

        uint8_t dir_bit = (uint8_t)((TIM3->CR1 >> 4) & 1U);
        if ((dir_bit != g_HSC0_PrevDirBit) && (g_HSC0_DirChangeArmed == 2U)) {
            g_HSC_IntTriggerType = 2U;
            EXTI_SWIER |= 0x00000002UL;
        }
        g_HSC0_PrevDirBit     = (uint8_t)((TIM3->CR1 >> 4) & 1U);
        g_HSC0_DirChangeArmed = 2U;
    }

    if (g_HSC1_Mode != 0U) {
        g_HSC1_CurrVal = (uint32_t)TIM4->CNT + (g_HSC1_SoftOverflow << 16);
        if (g_HSC1_CurrVal == g_HSC1_PresetVal) {
            g_HSC_IntTriggerType = 3U;
            EXTI_SWIER |= 0x00000002UL;
        }

        uint8_t dir_bit = (uint8_t)((TIM4->CR1 >> 4) & 1U);
        if ((dir_bit != g_HSC1_PrevDirBit) && (g_HSC1_DirChangeArmed == 2U)) {
            g_HSC_IntTriggerType = 4U;
            EXTI_SWIER |= 0x00000002UL;
        }
        g_HSC1_PrevDirBit     = (uint8_t)((TIM4->CR1 >> 4) & 1U);
        g_HSC1_DirChangeArmed = 2U;
    }

    /* HSC0 External Direction (PC6) and Reset (PE1) Pin Handling */
    if (g_HSC0_Mode == 2U || g_HSC0_Mode == 4U || g_HSC0_Mode == 5U) {
        uint8_t pc6 = ((GPIOC_IDR & 0x0040UL) != 0) ? 1U : 0U;
        if (pc6 != g_HSC0_DirLevel) {
            TIM3->CR1 &= (uint16_t)~0x0010U; /* Count Up */
        } else {
            TIM3->CR1 |= 0x0010U;            /* Count Down */
        }

        if (g_HSC0_Mode == 5U) {
            uint8_t pe1 = ((GPIOE_IDR & 0x0002UL) != 0) ? 1U : 0U;
            if (pe1 == g_HSC0_ResetLevel) {
                TIM3->CNT          = 0U;
                g_HSC0_SoftOverflow = 0UL;
                g_HSC0_CurrVal      = 0UL;
            }
        }
    }

    /* HSC1 External Direction (PC7) Pin Handling */
    if (g_HSC1_Mode == 8U || g_HSC1_Mode == 11U) {
        uint8_t pc7 = ((GPIOC_IDR & 0x0080UL) != 0) ? 1U : 0U;
        if (pc7 != g_HSC1_DirLevel) {
            TIM4->CR1 &= (uint16_t)~0x0010U; /* Count Up */
        } else {
            TIM4->CR1 |= 0x0010U;            /* Count Down */
        }
    }

    /* ----------------------------------------------------------------------
     * 4. PTO0 (Q0.0, TIM1 Master + TIM2 Slave Counter) Envelope Generator
     * ---------------------------------------------------------------------- */
    if ((uint8_t)(g_PTO0_State - 0x0BU) < 9U) {
        g_PTO0_OverflowShadow = g_PTO0_Overflow60k;
        g_PTO0_TotalPulses    = (uint32_t)TIM2->CNT + g_PTO0_Overflow60k * 60000UL;
        g_PTO0_PulseCount64   = (uint64_t)g_PTO0_TotalPulses;
    }

    g_PTO0_BusyFlag = 0U;
    uint32_t delta_f0 = g_PTO0_DeltaFreqMax - g_PTO0_DeltaFreqMin;
    bool update_pto0  = false;

    switch (g_PTO0_State) {
    case 0x0CU: {
        uint64_t prod = g_PTO0_PulseCount64 * (uint64_t)g_PTO0_SlopeCoeff;
        uint64_t num  = (uint64_t)g_PTO0_BaseOffset - prod;
        uint32_t div  = (g_PTO0_HighSpeedFlag != 0U) ? 100000000UL : 100000UL;
        g_PTO0_CurrFreqScaled = (uint32_t)(div / num);
        update_pto0 = true;
        break;
    }
    case 0x0DU: {
        uint32_t step = (uint32_t)(((uint64_t)delta_f0 * g_PTO0_AccelStepIdx) / g_PTO0_AccelSteps);
        g_PTO0_CurrFreqScaled = g_PTO0_InitFreq + step;
        if (g_PTO0_CurrFreqScaled >= g_PTO0_TargetFreq) {
            g_PTO0_CurrFreqScaled = g_PTO0_TargetFreq;
        } else {
            g_PTO0_AccelStepIdx++;
        }
        update_pto0 = true;
        break;
    }
    case 0x0EU: {
        uint64_t prod = g_PTO0_PulseCount64 * (uint64_t)g_PTO0_SlopeCoeff + (uint64_t)g_PTO0_BaseOffset;
        uint32_t div  = (g_PTO0_HighSpeedFlag != 0U) ? 100000000UL : 100000UL;
        g_PTO0_CurrFreqScaled = (uint32_t)(div / prod);
        update_pto0 = true;
        break;
    }
    case 0x0FU: {
        uint32_t denom = g_PTO0_DecelSteps + (g_PTO0_DecelSteps >> 5);
        uint32_t step  = (uint32_t)(((uint64_t)delta_f0 * g_PTO0_DecelStepIdx) / denom);
        g_PTO0_CurrFreqScaled = g_PTO0_InitFreq - step;
        if ((g_PTO0_CurrFreqScaled > g_PTO0_TargetFreq) && (g_PTO0_CurrFreqScaled < 20000001UL)) {
            g_PTO0_DecelStepIdx++;
        } else {
            g_PTO0_CurrFreqScaled = g_PTO0_TargetFreq;
        }
        update_pto0 = true;
        break;
    }
    case 0x11U: {
        uint32_t step = (uint32_t)(((uint64_t)delta_f0 * g_PTO0_DecelStepIdx) / g_PTO0_DecelSteps);
        g_PTO0_CurrFreqScaled = g_PTO0_StartFreq - step;
        if ((g_PTO0_CurrFreqScaled > g_PTO0_TargetFreq) && (g_PTO0_CurrFreqScaled < 20000001UL)) {
            g_PTO0_DecelStepIdx++;
        } else {
            g_PTO0_CurrFreqScaled = g_PTO0_TargetFreq;
            g_PTO0_State          = 0x0BU;
            if (g_PTO0_StopMode == 2U) {
                g_PTO0_StopMode = 1U;
                TIM1->SR        = 0xFFFEU;
                TIM1->DIER     |= 0x0001U;
            }
        }
        update_pto0 = true;
        break;
    }
    case 0x16U: {
        uint32_t step = (uint32_t)(((uint64_t)delta_f0 * g_PTO0_AccelStepIdx) / g_PTO0_AccelSteps);
        g_PTO0_CurrFreqScaled = g_PTO0_InitFreq + step;
        if (g_PTO0_CurrFreqScaled < g_PTO0_TargetFreq) {
            g_PTO0_AccelStepIdx++;
        } else {
            g_PTO0_State          = 0x15U;
            g_PTO0_CurrFreqScaled = g_PTO0_TargetFreq;
        }
        update_pto0 = true;
        break;
    }
    case 0x18U: {
        uint32_t step = (uint32_t)(((uint64_t)delta_f0 * g_PTO0_DecelStepIdx) / g_PTO0_DecelSteps);
        g_PTO0_CurrFreqScaled = g_PTO0_StartFreq - step;
        if ((g_PTO0_CurrFreqScaled > g_PTO0_TargetFreq) && (g_PTO0_CurrFreqScaled < 20000001UL)) {
            g_PTO0_DecelStepIdx++;
        } else {
            g_PTO0_State = 0x15U;
            if (g_PTO0_StopMode == 2U) {
                g_PTO0_State    = 0x1CU;
                g_PTO0_StopMode = 1U;
                TIM1->SR        = 0xFFFEU;
                TIM1->DIER     |= 0x0001U;
            }
            g_PTO0_CurrFreqScaled = g_PTO0_TargetFreq;
        }
        update_pto0 = true;
        break;
    }
    default:
        break;
    }

    if (update_pto0) {
        g_PTO0_BusyFlag = 1U;
        PTO_ComputeShadowTimings(g_PTO0_CurrFreqScaled,
                                 &g_PTO0_ShadowARR,
                                 &g_PTO0_ShadowPSC,
                                 &g_PTO0_ShadowCCR1);
        if (g_PTO0_PrevFreqScaled != g_PTO0_CurrFreqScaled) {
            g_PTO0_ActiveFreq = g_PTO0_CurrFreqScaled;
            TIM1->SR          = 0xFFFEU;
            TIM1->DIER       |= 0x0001U;
        }
        g_PTO0_PrevFreqScaled = g_PTO0_CurrFreqScaled;
    }

    /* ----------------------------------------------------------------------
     * 5. PTO1 (Q0.1, TIM8 Master + TIM5 Slave Counter) Envelope Generator
     * ---------------------------------------------------------------------- */
    if ((uint8_t)(g_PTO1_State - 0x0BU) < 9U) {
        g_PTO1_OverflowShadow = g_PTO1_Overflow60k;
        g_PTO1_TotalPulses    = (uint32_t)TIM5->CNT + g_PTO1_Overflow60k * 60000UL;
        g_PTO1_PulseCount64   = (uint64_t)g_PTO1_TotalPulses;
    }

    g_PTO1_BusyFlag = 0U;
    uint32_t delta_f1 = g_PTO1_DeltaFreqMax - g_PTO1_DeltaFreqMin;
    bool update_pto1  = false;

    switch (g_PTO1_State) {
    case 0x0CU: {
        uint64_t prod = g_PTO1_PulseCount64 * (uint64_t)g_PTO1_SlopeCoeff;
        uint64_t num  = (uint64_t)g_PTO1_BaseOffset - prod;
        uint32_t div  = (g_PTO1_HighSpeedFlag != 0U) ? 100000000UL : 100000UL;
        g_PTO1_CurrFreqScaled = (uint32_t)(div / num);
        update_pto1 = true;
        break;
    }
    case 0x0DU: {
        uint32_t step = (uint32_t)(((uint64_t)delta_f1 * g_PTO1_AccelStepIdx) / g_PTO1_AccelSteps);
        g_PTO1_CurrFreqScaled = g_PTO1_InitFreq + step;
        if (g_PTO1_CurrFreqScaled >= g_PTO1_TargetFreq) {
            g_PTO1_CurrFreqScaled = g_PTO1_TargetFreq;
        } else {
            g_PTO1_AccelStepIdx++;
        }
        update_pto1 = true;
        break;
    }
    case 0x0EU: {
        uint64_t prod = g_PTO1_PulseCount64 * (uint64_t)g_PTO1_SlopeCoeff + (uint64_t)g_PTO1_BaseOffset;
        uint32_t div  = (g_PTO1_HighSpeedFlag != 0U) ? 100000000UL : 100000UL;
        g_PTO1_CurrFreqScaled = (uint32_t)(div / prod);
        update_pto1 = true;
        break;
    }
    case 0x0FU: {
        uint32_t denom = g_PTO1_DecelSteps + (g_PTO1_DecelSteps >> 5);
        uint32_t step  = (uint32_t)(((uint64_t)delta_f1 * g_PTO1_DecelStepIdx) / denom);
        g_PTO1_CurrFreqScaled = g_PTO1_InitFreq - step;
        if ((g_PTO1_CurrFreqScaled > g_PTO1_TargetFreq) && (g_PTO1_CurrFreqScaled < 20000001UL)) {
            g_PTO1_DecelStepIdx++;
        } else {
            g_PTO1_CurrFreqScaled = g_PTO1_TargetFreq;
        }
        update_pto1 = true;
        break;
    }
    case 0x11U: {
        uint32_t step = (uint32_t)(((uint64_t)delta_f1 * g_PTO1_DecelStepIdx) / g_PTO1_DecelSteps);
        g_PTO1_CurrFreqScaled = g_PTO1_StartFreq - step;
        if ((g_PTO1_CurrFreqScaled > g_PTO1_TargetFreq) && (g_PTO1_CurrFreqScaled < 20000001UL)) {
            g_PTO1_DecelStepIdx++;
        } else {
            g_PTO1_CurrFreqScaled = g_PTO1_TargetFreq;
            g_PTO1_State          = 0x0BU;
            if (g_PTO1_StopMode == 2U) {
                g_PTO1_StopMode = 1U;
                TIM8->SR        = 0xFFFEU;
                TIM8->DIER     |= 0x0001U;
            }
        }
        update_pto1 = true;
        break;
    }
    case 0x16U: {
        uint32_t step = (uint32_t)(((uint64_t)delta_f1 * g_PTO1_AccelStepIdx) / g_PTO1_AccelSteps);
        g_PTO1_CurrFreqScaled = g_PTO1_InitFreq + step;
        if (g_PTO1_CurrFreqScaled < g_PTO1_TargetFreq) {
            g_PTO1_AccelStepIdx++;
        } else {
            g_PTO1_State          = 0x15U;
            g_PTO1_CurrFreqScaled = g_PTO1_TargetFreq;
        }
        update_pto1 = true;
        break;
    }
    case 0x18U: {
        uint32_t step = (uint32_t)(((uint64_t)delta_f1 * g_PTO1_DecelStepIdx) / g_PTO1_DecelSteps);
        g_PTO1_CurrFreqScaled = g_PTO1_StartFreq - step;
        if ((g_PTO1_CurrFreqScaled > g_PTO1_TargetFreq) && (g_PTO1_CurrFreqScaled < 20000001UL)) {
            g_PTO1_DecelStepIdx++;
        } else {
            g_PTO1_State = 0x15U;
            if (g_PTO1_StopMode == 2U) {
                g_PTO1_State    = 0x1CU;
                g_PTO1_StopMode = 1U;
                TIM8->SR        = 0xFFFEU;
                TIM8->DIER     |= 0x0001U;
            }
            g_PTO1_CurrFreqScaled = g_PTO1_TargetFreq;
        }
        update_pto1 = true;
        break;
    }
    default:
        break;
    }

    if (update_pto1) {
        g_PTO1_BusyFlag = 1U;
        PTO_ComputeShadowTimings(g_PTO1_CurrFreqScaled,
                                 &g_PTO1_ShadowARR,
                                 &g_PTO1_ShadowPSC,
                                 &g_PTO1_ShadowCCR1);
        if (g_PTO1_PrevFreqScaled != g_PTO1_CurrFreqScaled) {
            g_PTO1_ActiveFreq = g_PTO1_CurrFreqScaled;
            TIM8->SR          = 0xFFFEU;
            TIM8->DIER       |= 0x0001U;
        }
        g_PTO1_PrevFreqScaled = g_PTO1_CurrFreqScaled;
    }
}

/* ==========================================================================
 * PTO0 / PTO1 Configuration & Hardware Timer ISRs (0x0803f29c .. 0x0803fbd9)
 * ========================================================================== */

/**
 * @brief Configures TIM1 (master PWM generator on Q0.0) and TIM2 (slave pulse
 *        counter gated via ITR0) for PTO/PWM Channel 0.
 *        Reconstructed from FUN_0803f29c @ 0x0803f29c.
 */
void PTO0_TIM1_TIM2_Config(uint16_t period, uint16_t prescaler, uint16_t pulse)
{
    TIM_TimeBaseInitTypeDef tb_init;
    TIM_OCInitTypeDef       oc_init;
    NVIC_InitTypeDef        nvic_init;

    TIM_DeInit(TIM1);
    TIM_DeInit(TIM2);

    RCC_APB2PeriphClockCmd(0x00000800UL, 1); /* TIM1 */
    RCC_APB1PeriphClockCmd(0x00000001UL, 1); /* TIM2 */

    tb_init.TIM_Period            = period;
    tb_init.TIM_Prescaler         = prescaler;
    tb_init.TIM_ClockDivision     = 0x0000U;
    tb_init.TIM_CounterMode       = 0x0000U;
    tb_init.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &tb_init);
    TIM_ARRPreloadConfig(TIM1, 1);

    oc_init.TIM_OCMode       = 0x0060U; /* TIM_OCMode_PWM1 */
    oc_init.TIM_OutputState  = 0x0001U;
    oc_init.TIM_OutputNState = 0x0000U;
    oc_init.TIM_Pulse        = pulse;
    oc_init.TIM_OCPolarity   = 0x0002U; /* Low */
    oc_init.TIM_OCNPolarity  = 0x0000U;
    oc_init.TIM_OCIdleState  = 0x0000U;
    oc_init.TIM_OCNIdleState = 0x0000U;
    TIM_OC1Init(TIM1, &oc_init);
    TIM_OC1PreloadConfig(TIM1, 0x0008U);

    /* Configure TIM1 TRGO = Update (0x20) to drive TIM2 via ITR0 (0x00) */
    TIM_SelectOutputTrigger(TIM1, 0x0020U);
    TIM_SelectMasterSlaveMode(TIM1, 0x0080U);

    tb_init.TIM_Period            = 60000U;
    tb_init.TIM_Prescaler         = 0U;
    tb_init.TIM_ClockDivision     = 0U;
    tb_init.TIM_CounterMode       = 0U;
    tb_init.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM2, &tb_init);

    TIM_SelectInputTrigger(TIM2, 0x0000U); /* TS = ITR0 (TIM1) */
    TIM2->SMCR |= 0x0007U;                 /* External Clock Mode 1 */
    TIM_SelectSlaveMode_External1(TIM2);

    nvic_init.NVIC_IRQChannel                   = 25U; /* TIM1_UP_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 1U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);

    nvic_init.NVIC_IRQChannel                   = 28U; /* TIM2_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 0U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);

    TIM_ClearFlag(TIM2, 0x0001U);
    TIM_ITConfig(TIM2, 0x0001U, 1);
    TIM_CtrlPWMOutputs(TIM1, 1);
    TIM_Cmd(TIM2, 0);
    TIM_Cmd(TIM1, 0);
}

/**
 * @brief TIM1 Update ISR (PTO0 shadow register reload & envelope segment transition).
 *        Reconstructed from FUN_0803f3fe @ 0x0803f3fe.
 */
void TIM1_UP_IRQHandler(void)
{
    if ((TIM1->SR & 0x0001U) != 0 && (TIM1->DIER & 0x0001U) != 0) {
        TIM1->SR    = 0xFFFEU;
        TIM1->DIER &= (uint16_t)~0x0001U;

        TIM1->ARR  = g_PTO0_ShadowARR;
        TIM1->PSC  = g_PTO0_ShadowPSC;
        TIM1->CCR1 = g_PTO0_ShadowCCR1;

        if (g_PTO0_StopMode == 1U) {
            g_PTO0_StopMode = 0U;
            TIM1->CR1      &= (uint16_t)~0x0001U;
            TIM2->CR1      &= (uint16_t)~0x0001U;

            if (g_PTO0_State == 0x1CU) {
                g_PTO0_IntDoneFlag = 1U;
                EXTI_SWIER        |= 0x00000004UL;
            }
            g_PTO0_State = 0U;
        } else if (g_PTO0_State == 0x13U) {
            g_PTO0_State = 0x14U;
        }
    }
}

/**
 * @brief TIM2 ISR (PTO0 60,000-pulse slave counter overflow & target comparison).
 *        Reconstructed from FUN_0803f58e @ 0x0803f58e.
 */
void TIM2_IRQHandler(void)
{
    if ((TIM2->SR & 0x0001U) != 0 && (TIM2->DIER & 0x0001U) != 0) {
        TIM2->SR = 0xFFFEU;
        g_PTO0_Overflow60k++;

        uint32_t total = (uint32_t)TIM2->CNT + g_PTO0_Overflow60k * 60000UL;
        g_PTO0_TotalPulses = total;

        if (g_PTO0_Overflow60k >= g_PTO0_TargetPulses1 &&
            total >= g_PTO0_TargetPulses2 &&
            g_PTO0_State != 0x15U) {
            TIM1->CR1 &= (uint16_t)~0x0001U;
            TIM2->CR1 &= (uint16_t)~0x0001U;

            TIM1->CNT            = 0U;
            TIM2->CNT            = 0U;
            g_PTO0_Overflow60k   = 0UL;
            g_PTO0_TotalPulses   = 0UL;
            TIM2->ARR            = 60000U;
            g_PTO0_TargetPulses1 = 0UL;
            g_PTO0_TargetPulses2 = 0UL;
            g_PTO0_StopMode      = 0U;
            g_PTO0_State         = 0U;
            g_PTO0_IntDoneFlag   = 1U;
            EXTI_SWIER          |= 0x00000004UL;
        } else if (g_PTO0_Overflow60k == g_PTO0_TargetPulses1 &&
                   g_PTO0_TargetPulses2 > total) {
            TIM2->ARR = (uint16_t)(g_PTO0_TargetPulses2 - total);
        }
    }
}

/**
 * @brief TIM3 ISR (HSC0 16-bit hardware counter overflow/underflow handler).
 *        Reconstructed from 0x0803f680 in full_disasm.s.
 */
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & 0x0001U) != 0 && (TIM3->DIER & 0x0001U) != 0) {
        TIM3->SR = 0xFFFEU;
        if ((TIM3->CR1 & 0x0010U) == 0) {
            g_HSC0_SoftOverflow++;
        } else {
            g_HSC0_SoftOverflow--;
        }
    }
}

/**
 * @brief TIM4 ISR (HSC1 16-bit hardware counter overflow/underflow handler).
 *        Reconstructed from 0x0803f6b2 in full_disasm.s.
 */
void TIM4_IRQHandler(void)
{
    if ((TIM4->SR & 0x0001U) != 0 && (TIM4->DIER & 0x0001U) != 0) {
        TIM4->SR = 0xFFFEU;
        if ((TIM4->CR1 & 0x0010U) == 0) {
            g_HSC1_SoftOverflow++;
        } else {
            g_HSC1_SoftOverflow--;
        }
    }
}

/**
 * @brief TIM5 ISR (PTO1 60,000-pulse slave counter overflow & target comparison).
 *        Reconstructed from 0x0803f6e4 in full_disasm.s.
 */
void TIM5_IRQHandler(void)
{
    if ((TIM5->SR & 0x0001U) != 0 && (TIM5->DIER & 0x0001U) != 0) {
        TIM5->SR = 0xFFFEU;
        g_PTO1_Overflow60k++;

        uint32_t total = (uint32_t)TIM5->CNT + g_PTO1_Overflow60k * 60000UL;
        g_PTO1_TotalPulses = total;

        if (g_PTO1_Overflow60k >= g_PTO1_TargetPulses1 &&
            total >= g_PTO1_TargetPulses2 &&
            g_PTO1_State != 0x15U) {
            TIM8->CR1 &= (uint16_t)~0x0001U;
            TIM5->CR1 &= (uint16_t)~0x0001U;

            TIM8->CNT            = 0U;
            TIM5->CNT            = 0U;
            g_PTO1_Overflow60k   = 0UL;
            g_PTO1_TotalPulses   = 0UL;
            TIM5->ARR            = 60000U;
            g_PTO1_TargetPulses1 = 0UL;
            g_PTO1_TargetPulses2 = 0UL;
            g_PTO1_StopMode      = 0U;
            g_PTO1_State         = 0U;
            g_PTO1_IntDoneFlag   = 1U;
            EXTI_SWIER          |= 0x00000004UL;
        } else if (g_PTO1_Overflow60k == g_PTO1_TargetPulses1 &&
                   g_PTO1_TargetPulses2 > total) {
            TIM5->ARR = (uint16_t)(g_PTO1_TargetPulses2 - total);
        }
    }
}

/**
 * @brief TIM6 ISR (Port 0 RS-485 turnaround delay timer expiration: starts
 *        DMA1_Channel4 USART1 TX and re-arms DMA1_Channel5 RX).
 *        Reconstructed from 0x0803f7fc in full_disasm.s.
 */
void TIM6_IRQHandler(void)
{
    if ((TIM6->SR & 0x0001U) != 0 && (TIM6->DIER & 0x0001U) != 0) {
        TIM6->SR   = 0xFFFEU;
        TIM6->CR1 &= (uint16_t)~0x0001U;

        /* Start DMA1_Channel4 (USART1_TX) with g_Port0_TxLen bytes */
        MMIO32(0x40020048U)  = g_Port0_TxLen;
        MMIO32(0x40020044U) |= 0x00000001UL;

        /* Reset DMA1_Channel5 (USART1_RX) buffer size to 256 and enable */
        MMIO32(0x4002005CU)  = 256UL;
        MMIO32(0x40020058U) |= 0x00000001UL;
    }
}

/**
 * @brief TIM7 ISR (Port 1 RS-485 turnaround delay timer expiration: starts
 *        DMA1_Channel7 USART2 TX and re-arms DMA1_Channel6 RX).
 *        Reconstructed from 0x0803f84a in full_disasm.s.
 */
void TIM7_IRQHandler(void)
{
    if ((TIM7->SR & 0x0001U) != 0 && (TIM7->DIER & 0x0001U) != 0) {
        TIM7->SR   = 0xFFFEU;
        TIM7->CR1 &= (uint16_t)~0x0001U;

        /* Start DMA1_Channel7 (USART2_TX) with g_Port1_TxLen bytes */
        MMIO32(0x40020084U)  = g_Port1_TxLen;
        MMIO32(0x40020080U) |= 0x00000001UL;

        /* Reset DMA1_Channel6 (USART2_RX) buffer size to 256 and enable */
        MMIO32(0x40020070U)  = 256UL;
        MMIO32(0x4002006CU) |= 0x00000001UL;
    }
}

/**
 * @brief Configures TIM8 (master PWM generator on Q0.1) and TIM5 (slave pulse
 *        counter gated via ITR3) for PTO/PWM Channel 1.
 *        Reconstructed from FUN_0803f8ac @ 0x0803f8ac.
 */
void PTO1_TIM8_TIM5_Config(uint16_t period, uint16_t prescaler, uint16_t pulse)
{
    TIM_TimeBaseInitTypeDef tb_init;
    TIM_OCInitTypeDef       oc_init;
    NVIC_InitTypeDef        nvic_init;

    TIM_DeInit(TIM8);
    TIM_DeInit(TIM5);

    RCC_APB2PeriphClockCmd(0x00002000UL, 1); /* TIM8 */
    RCC_APB1PeriphClockCmd(0x00000008UL, 1); /* TIM5 */

    tb_init.TIM_Period            = period;
    tb_init.TIM_Prescaler         = prescaler;
    tb_init.TIM_ClockDivision     = 0x0000U;
    tb_init.TIM_CounterMode       = 0x0000U;
    tb_init.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM8, &tb_init);
    TIM_ARRPreloadConfig(TIM8, 1);

    oc_init.TIM_OCMode       = 0x0060U; /* TIM_OCMode_PWM1 */
    oc_init.TIM_OutputState  = 0x0001U;
    oc_init.TIM_OutputNState = 0x0000U;
    oc_init.TIM_Pulse        = pulse;
    oc_init.TIM_OCPolarity   = 0x0002U; /* Low */
    oc_init.TIM_OCNPolarity  = 0x0000U;
    oc_init.TIM_OCIdleState  = 0x0000U;
    oc_init.TIM_OCNIdleState = 0x0000U;
    TIM_OC1Init(TIM8, &oc_init);
    TIM_OC1PreloadConfig(TIM8, 0x0008U);

    /* Configure TIM8 TRGO = Update (0x20) to drive TIM5 via ITR3 (0x30) */
    TIM_SelectOutputTrigger(TIM8, 0x0020U);
    TIM_SelectMasterSlaveMode(TIM8, 0x0080U);

    tb_init.TIM_Period            = 60000U;
    tb_init.TIM_Prescaler         = 0U;
    tb_init.TIM_ClockDivision     = 0U;
    tb_init.TIM_CounterMode       = 0U;
    tb_init.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM5, &tb_init);

    TIM_SelectInputTrigger(TIM5, 0x0030U); /* TS = ITR3 (TIM8) */
    TIM5->SMCR |= 0x0007U;
    TIM_SelectSlaveMode_External1(TIM5);

    nvic_init.NVIC_IRQChannel                   = 44U; /* TIM8_UP_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 1U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);

    nvic_init.NVIC_IRQChannel                   = 50U; /* TIM5_IRQn */
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority        = 0U;
    nvic_init.NVIC_IRQChannelCmd                = 1U;
    NVIC_Init(&nvic_init);

    TIM_ClearFlag(TIM5, 0x0001U);
    TIM_ITConfig(TIM5, 0x0001U, 1);
    TIM_CtrlPWMOutputs(TIM8, 1);
    TIM_Cmd(TIM5, 0);
    TIM_Cmd(TIM8, 0);
}

/**
 * @brief TIM8 Update ISR (PTO1 shadow register reload & envelope segment transition).
 *        Reconstructed from FUN_0803fa26 @ 0x0803fa26.
 */
void TIM8_UP_IRQHandler(void)
{
    if ((TIM8->SR & 0x0001U) != 0 && (TIM8->DIER & 0x0001U) != 0) {
        TIM8->SR    = 0xFFFEU;
        TIM8->DIER &= (uint16_t)~0x0001U;

        TIM8->ARR  = g_PTO1_ShadowARR;
        TIM8->PSC  = g_PTO1_ShadowPSC;
        TIM8->CCR1 = g_PTO1_ShadowCCR1;

        if (g_PTO1_StopMode == 1U) {
            g_PTO1_StopMode = 0U;
            TIM8->CR1      &= (uint16_t)~0x0001U;
            TIM5->CR1      &= (uint16_t)~0x0001U;

            if (g_PTO1_State == 0x1CU) {
                g_PTO1_IntDoneFlag = 1U;
                EXTI_SWIER        |= 0x00000004UL;
            }
            g_PTO1_State = 0U;
        } else if (g_PTO1_State == 0x13U) {
            g_PTO1_State = 0x14U;
        }
    }
}

/* ==========================================================================
 * TIM1..TIM8 Standard Peripheral Library Functions (0x0803f1b6 .. 0x0804007d)
 * ========================================================================== */

/**
 * @brief Initializes the TIMx Channel1 according to the specified parameters.
 *        Reconstructed from FUN_0803f1b6 @ 0x0803f1b6.
 */
void TIM_OC1Init(TIM_TypeDef *TIMx, const TIM_OCInitTypeDef *TIM_OCInitStruct)
{
    TIMx->CCER &= 0xFFFEU;
    uint16_t ccer_mask = (TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM2 ||
                          TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5) ? 0xFFFDU : 0xFFF5U;
    uint16_t ccer = (TIMx->CCER & ccer_mask) |
                    TIM_OCInitStruct->TIM_OCPolarity |
                    TIM_OCInitStruct->TIM_OutputState;
    uint16_t ccmr1 = (TIMx->CCMR1 & 0xFF8CU) | TIM_OCInitStruct->TIM_OCMode;

    if (TIMx == TIM1 || TIMx == TIM8) {
        ccer = (ccer & 0xFFF3U) |
               TIM_OCInitStruct->TIM_OCNPolarity |
               TIM_OCInitStruct->TIM_OutputNState;
        TIMx->CR2 = (TIMx->CR2 & 0xFCFFU) |
                    TIM_OCInitStruct->TIM_OCIdleState |
                    TIM_OCInitStruct->TIM_OCNIdleState;
    }

    TIMx->CCR1  = TIM_OCInitStruct->TIM_Pulse;
    TIMx->CCMR1 = ccmr1;
    TIMx->CCER  = ccer;
}

/**
 * @brief Initializes the TIMx Channel2 according to the specified parameters.
 *        Reconstructed from FUN_0803f222 @ 0x0803f222.
 */
void TIM_OC2Init(TIM_TypeDef *TIMx, const TIM_OCInitTypeDef *TIM_OCInitStruct)
{
    TIMx->CCER &= 0xFFEFU;
    uint16_t ccer_mask = (TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM2 ||
                          TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5) ? 0xFFDFU : 0xFF5FU;
    uint16_t ccer = (TIMx->CCER & ccer_mask) |
                    (uint16_t)(TIM_OCInitStruct->TIM_OCPolarity << 4) |
                    (uint16_t)(TIM_OCInitStruct->TIM_OutputState << 4);
    uint16_t ccmr1 = (TIMx->CCMR1 & 0x8CFFU) |
                     (uint16_t)(TIM_OCInitStruct->TIM_OCMode << 8);

    if (TIMx == TIM1 || TIMx == TIM8) {
        ccer = (ccer & 0xFF3FU) |
               (uint16_t)(TIM_OCInitStruct->TIM_OCNPolarity << 4) |
               (uint16_t)(TIM_OCInitStruct->TIM_OutputNState << 4);
        TIMx->CR2 = (TIMx->CR2 & 0xF3FFU) |
                    (uint16_t)(TIM_OCInitStruct->TIM_OCIdleState << 2) |
                    (uint16_t)(TIM_OCInitStruct->TIM_OCNIdleState << 2);
    }

    TIMx->CCR2  = TIM_OCInitStruct->TIM_Pulse;
    TIMx->CCMR1 = ccmr1;
    TIMx->CCER  = ccer;
}

/**
 * @brief Enables or disables TIMx peripheral Preload register on ARR.
 *        Reconstructed from FUN_0803fbda @ 0x0803fbda.
 */
void TIM_ARRPreloadConfig(TIM_TypeDef *TIMx, uint8_t NewState)
{
    if (NewState != 0) {
        TIMx->CR1 |= 0x0080U;
    } else {
        TIMx->CR1 &= (uint16_t)~0x0080U;
    }
}

/**
 * @brief Clears the TIMx's pending flags.
 *        Reconstructed from FUN_0803fbee @ 0x0803fbee.
 */
void TIM_ClearFlag(TIM_TypeDef *TIMx, uint16_t TIM_FLAG)
{
    TIMx->SR = (uint16_t)~TIM_FLAG;
}

/**
 * @brief Enables or disables the specified TIM peripheral.
 *        Reconstructed from FUN_0803fbf4 @ 0x0803fbf4.
 */
void TIM_Cmd(TIM_TypeDef *TIMx, uint8_t NewState)
{
    if (NewState != 0) {
        TIMx->CR1 |= 0x0001U;
    } else {
        TIMx->CR1 &= (uint16_t)~0x0001U;
    }
}

/**
 * @brief Enables or disables the TIM peripheral Main Outputs (MOE bit in BDTR).
 *        Reconstructed from FUN_0803fc08 @ 0x0803fc08.
 */
void TIM_CtrlPWMOutputs(TIM_TypeDef *TIMx, uint8_t NewState)
{
    if (NewState != 0) {
        TIMx->BDTR |= 0x8000U;
    } else {
        TIMx->BDTR &= 0x7FFFU;
    }
}

/**
 * @brief Deinitializes the TIMx peripheral registers to their default reset values.
 *        Reconstructed from FUN_0803fc1e @ 0x0803fc1e.
 */
void TIM_DeInit(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM1) {
        RCC_APB2PeriphResetCmd(0x00000800UL, 1);
        RCC_APB2PeriphResetCmd(0x00000800UL, 0);
    } else if (TIMx == TIM2) {
        RCC_APB1PeriphResetCmd(0x00000001UL, 1);
        RCC_APB1PeriphResetCmd(0x00000001UL, 0);
    } else if (TIMx == TIM3) {
        RCC_APB1PeriphResetCmd(0x00000002UL, 1);
        RCC_APB1PeriphResetCmd(0x00000002UL, 0);
    } else if (TIMx == TIM4) {
        RCC_APB1PeriphResetCmd(0x00000004UL, 1);
        RCC_APB1PeriphResetCmd(0x00000004UL, 0);
    } else if (TIMx == TIM5) {
        RCC_APB1PeriphResetCmd(0x00000008UL, 1);
        RCC_APB1PeriphResetCmd(0x00000008UL, 0);
    } else if (TIMx == TIM6) {
        RCC_APB1PeriphResetCmd(0x00000010UL, 1);
        RCC_APB1PeriphResetCmd(0x00000010UL, 0);
    } else if (TIMx == TIM7) {
        RCC_APB1PeriphResetCmd(0x00000020UL, 1);
        RCC_APB1PeriphResetCmd(0x00000020UL, 0);
    } else if (TIMx == TIM8) {
        RCC_APB2PeriphResetCmd(0x00002000UL, 1);
        RCC_APB2PeriphResetCmd(0x00002000UL, 0);
    }
}

/**
 * @brief Configures the TIMx Encoder Interface.
 *        Reconstructed from FUN_0803fd8e @ 0x0803fd8e.
 */
void TIM_EncoderInterfaceConfig(TIM_TypeDef *TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity)
{
    TIMx->SMCR  = (TIMx->SMCR & 0xFFF8U) | TIM_EncoderMode;
    TIMx->CCMR1 = (TIMx->CCMR1 & 0xFCFCU) | 0x0101U;
    TIMx->CCER  = (TIMx->CCER & 0xFFDDU) | TIM_IC1Polarity | (uint16_t)(TIM_IC2Polarity << 4);
}

/**
 * @brief Configures the TIMx event to be generated by software.
 *        Reconstructed from FUN_0803fdc2 @ 0x0803fdc2.
 */
void TIM_GenerateEvent(TIM_TypeDef *TIMx, uint16_t TIM_EventSource)
{
    TIMx->EGR = TIM_EventSource;
}

/**
 * @brief Sets the TIMx Input Capture 1..4 prescalers.
 *        Reconstructed from FUN_0803ff62 .. FUN_0803ffa0.
 */
void TIM_SetIC1Prescaler(TIM_TypeDef *TIMx, uint16_t TIM_ICPSC)
{
    TIMx->CCMR1 = (TIMx->CCMR1 & 0xFFF3U) | TIM_ICPSC;
}

void TIM_SetIC2Prescaler(TIM_TypeDef *TIMx, uint16_t TIM_ICPSC)
{
    TIMx->CCMR1 = (TIMx->CCMR1 & 0xF3FFU) | (uint16_t)(TIM_ICPSC << 8);
}

void TIM_SetIC3Prescaler(TIM_TypeDef *TIMx, uint16_t TIM_ICPSC)
{
    TIMx->CCMR2 = (TIMx->CCMR2 & 0xFFF3U) | TIM_ICPSC;
}

void TIM_SetIC4Prescaler(TIM_TypeDef *TIMx, uint16_t TIM_ICPSC)
{
    TIMx->CCMR2 = (TIMx->CCMR2 & 0xF3FFU) | (uint16_t)(TIM_ICPSC << 8);
}

/**
 * @brief Initializes the TIM peripheral according to the specified IC parameters.
 *        Reconstructed from FUN_0803fdc6 @ 0x0803fdc6.
 */
void TIM_ICInit(TIM_TypeDef *TIMx, const TIM_ICInitTypeDef *TIM_ICInitStruct)
{
    uint16_t ch = TIM_ICInitStruct->TIM_Channel;
    if (ch == 0x0000U) {
        TIMx->CCER &= 0xFFFEU;
        TIMx->CCMR1 = (TIMx->CCMR1 & 0xFF0CU) |
                      TIM_ICInitStruct->TIM_ICSelection |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICFilter << 4);
        TIMx->CCER  = (TIMx->CCER & 0xFFFDU) | TIM_ICInitStruct->TIM_ICPolarity | 0x0001U;
        TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
    } else if (ch == 0x0004U) {
        TIMx->CCER &= 0xFFEFU;
        TIMx->CCMR1 = (TIMx->CCMR1 & 0x0CFFU) |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICSelection << 8) |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICFilter << 12);
        TIMx->CCER  = (TIMx->CCER & 0xFFDFU) |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICPolarity << 4) | 0x0010U;
        TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
    } else if (ch == 0x0008U) {
        TIMx->CCER &= 0xFEFFU;
        TIMx->CCMR2 = (TIMx->CCMR2 & 0xFF0CU) |
                      TIM_ICInitStruct->TIM_ICSelection |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICFilter << 4);
        TIMx->CCER  = (TIMx->CCER & 0xFDFFU) |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICPolarity << 8) | 0x0100U;
        TIM_SetIC3Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
    } else {
        TIMx->CCER &= 0xEFFU;
        TIMx->CCMR2 = (TIMx->CCMR2 & 0x0CFFU) |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICSelection << 8) |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICFilter << 12);
        TIMx->CCER  = (TIMx->CCER & 0xDFFU) |
                      (uint16_t)(TIM_ICInitStruct->TIM_ICPolarity << 12) | 0x1000U;
        TIM_SetIC4Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
    }
}

/**
 * @brief Enables or disables the specified TIM interrupts.
 *        Reconstructed from FUN_0803ff08 @ 0x0803ff08.
 */
void TIM_ITConfig(TIM_TypeDef *TIMx, uint16_t TIM_IT, uint8_t NewState)
{
    if (NewState != 0) {
        TIMx->DIER |= TIM_IT;
    } else {
        TIMx->DIER &= (uint16_t)~TIM_IT;
    }
}

/**
 * @brief Selects the TIMx Slave Mode = External Clock Mode 1 (0x07).
 *        Reconstructed from FUN_0803ff18 @ 0x0803ff18.
 */
void TIM_SelectSlaveMode_External1(TIM_TypeDef *TIMx)
{
    TIMx->SMCR |= 0x0007U;
}

/**
 * @brief Enables or disables the TIMx peripheral Preload register on CCR1.
 *        Reconstructed from FUN_0803ff2a @ 0x0803ff2a.
 */
void TIM_OC1PreloadConfig(TIM_TypeDef *TIMx, uint16_t TIM_OCPreload)
{
    TIMx->CCMR1 = (TIMx->CCMR1 & 0xFFF7U) | TIM_OCPreload;
}

/**
 * @brief Selects the Input Trigger source for TIMx.
 *        Reconstructed from FUN_0803ff36 @ 0x0803ff36.
 */
void TIM_SelectInputTrigger(TIM_TypeDef *TIMx, uint16_t TIM_InputTriggerSource)
{
    TIMx->SMCR = (TIMx->SMCR & 0xFF8FU) | TIM_InputTriggerSource;
}

/**
 * @brief Sets or resets the TIMx Master/Slave Mode bit.
 *        Reconstructed from FUN_0803ff42 @ 0x0803ff42.
 */
void TIM_SelectMasterSlaveMode(TIM_TypeDef *TIMx, uint16_t TIM_MasterSlaveMode)
{
    TIMx->SMCR = (TIMx->SMCR & 0xFF7FU) | TIM_MasterSlaveMode;
}

/**
 * @brief Selects the TIMx Trigger Output Mode (TRGO).
 *        Reconstructed from FUN_0803ff52 @ 0x0803ff52.
 */
void TIM_SelectOutputTrigger(TIM_TypeDef *TIMx, uint16_t TIM_TRGOSource)
{
    TIMx->CR2 = (TIMx->CR2 & 0xFF8FU) | TIM_TRGOSource;
}

/**
 * @brief Configures the TIMx Trigger as External Clock.
 *        Reconstructed from FUN_0803ffb2 @ 0x0803ffb2.
 */
void TIM_TIxExternalClockConfig(TIM_TypeDef *TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter)
{
    if (TIM_TIxExternalCLKSource == 0x0060U) {
        TIMx->CCER &= 0xFFEFU;
        TIMx->CCMR1 = (TIMx->CCMR1 & 0x0CFFU) | 0x0100U | (uint16_t)(ICFilter << 12);
        TIMx->CCER  = (TIMx->CCER & 0xFFDFU) | (uint16_t)(TIM_ICPolarity << 4) | 0x0010U;
    } else {
        TIMx->CCER &= 0xFFFEU;
        TIMx->CCMR1 = (TIMx->CCMR1 & 0xFF0CU) | 0x0001U | (uint16_t)(ICFilter << 4);
        TIMx->CCER  = (TIMx->CCER & 0xFFFDU) | TIM_ICPolarity | 0x0001U;
    }
    TIM_SelectInputTrigger(TIMx, TIM_TIxExternalCLKSource);
    TIMx->SMCR |= 0x0007U;
}

/**
 * @brief Initializes the TIMx Time Base Unit peripheral according to the
 *        specified parameters.
 *        Reconstructed from FUN_0803ffe0 @ 0x0803ffe0.
 */
void TIM_TimeBaseInit(TIM_TypeDef *TIMx, const TIM_TimeBaseInitTypeDef *TIM_TimeBaseInitStruct)
{
    uint16_t cr1 = TIMx->CR1;
    if (TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM2 ||
        TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5) {
        cr1 = (cr1 & 0xFF8FU) | TIM_TimeBaseInitStruct->TIM_CounterMode;
    }
    if (TIMx != TIM6 && TIMx != TIM7) {
        cr1 = (cr1 & 0xFCFFU) | TIM_TimeBaseInitStruct->TIM_ClockDivision;
    }
    TIMx->CR1 = cr1;
    TIMx->ARR = TIM_TimeBaseInitStruct->TIM_Period;
    TIMx->PSC = TIM_TimeBaseInitStruct->TIM_Prescaler;
    if (TIMx == TIM1 || TIMx == TIM8) {
        TIMx->RCR = TIM_TimeBaseInitStruct->TIM_RepetitionCounter;
    }
    TIMx->EGR = 0x0001U; /* TIM_PSCReloadMode_Immediate */
}
