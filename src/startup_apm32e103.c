/**
 * @file startup_apm32e103.c
 * @brief Cortex-M3 Vector Table, Keil C Runtime Startup, Scatterload & .data Section
 *        for SIMATIC S7-226 PLC Firmware (APM32E103 / STM32F103HD)
 *
 * Reconstructed from APM32E103_backup.bin:
 *   - Vector Table:            0x08000000 - 0x0800012F (76 entries: 16 core + 60 IRQs)
 *   - __main / __rt_entry:     0x08000130 - 0x08000147
 *   - Reset_Handler:           0x08000148 - 0x0800014F
 *   - Default IRQ Stubs:       0x08000150 - 0x08000163
 *   - __scatterload:           0x08000AA0 - 0x08000AC3
 *   - __decompress1 (LZ77+RLE):0x08000AC4 - 0x08000B19
 *   - __scatterload_copy:      0x08042C18 - 0x08042C25
 *   - __scatterload_null:      0x08042C26 - 0x08042C27
 *   - __scatterload_zeroinit:  0x08042C28 - 0x08042C35
 *   - Region$$Table:           0x0804A960 - 0x0804A97F
 *   - Compressed .data image:  0x0807D800 - 0x0807D867 -> SRAM 0x20000000 - 0x200007F3
 */

#include "s7_226_plc.h"

/* ============================================================================
 * 1. WEAK DEFAULT EXCEPTION & PERIPHERAL IRQ HANDLERS
 * ============================================================================
 * In the binary:
 *   - NMI_Handler (0x08016ab6), SVC_Handler (0x08037926),
 *     DebugMon_Handler (0x080043b6), PendSV_Handler (0x08021792) are "bx lr".
 *   - HardFault_Handler (0x080153fe), MemManage_Handler (0x08016438),
 *     BusFault_Handler (0x08000e0c), UsageFault_Handler (0x080422c4) are "b .".
 *   - Unused peripheral IRQs point to Default_Handler at 0x08000162 ("b .").
 */

/* 0x08016AB6 */
__attribute__((weak)) void NMI_Handler(void)
{
    return;
}

/* 0x080153FE */
__attribute__((weak)) void HardFault_Handler(void)
{
    while (1) {
        /* Infinite loop on HardFault */
    }
}

/* 0x08016438 */
__attribute__((weak)) void MemManage_Handler(void)
{
    while (1) {
        /* Infinite loop on MemManage Fault */
    }
}

/* 0x08000E0C */
__attribute__((weak)) void BusFault_Handler(void)
{
    while (1) {
        /* Infinite loop on BusFault */
    }
}

/* 0x080422C4 */
__attribute__((weak)) void UsageFault_Handler(void)
{
    while (1) {
        /* Infinite loop on UsageFault */
    }
}

/* 0x08037926 */
__attribute__((weak)) void SVC_Handler(void)
{
    return;
}

/* 0x080043B6 */
__attribute__((weak)) void DebugMon_Handler(void)
{
    return;
}

/* 0x08021792 */
__attribute__((weak)) void PendSV_Handler(void)
{
    return;
}

/* 0x08000162 */
__attribute__((weak)) void Default_Handler(void)
{
    while (1) {
        /* Unhandled peripheral interrupt trap (0x08000162: e7fe b 0x08000162) */
    }
}

/* Weak aliases for active peripheral ISRs (implemented in their respective modules) */
__attribute__((weak, alias("Default_Handler"))) void SysTick_Handler(void);      /* 0x0803DFDE */
__attribute__((weak, alias("Default_Handler"))) void EXTI0_IRQHandler(void);     /* 0x0800FB4E */
__attribute__((weak, alias("Default_Handler"))) void EXTI1_IRQHandler(void);     /* 0x08010620 */
__attribute__((weak, alias("Default_Handler"))) void EXTI2_IRQHandler(void);     /* 0x080108D0 */
__attribute__((weak, alias("Default_Handler"))) void EXTI4_IRQHandler(void);     /* 0x08011EE2 */
__attribute__((weak, alias("Default_Handler"))) void EXTI9_5_IRQHandler(void);   /* 0x0801223C */
__attribute__((weak, alias("Default_Handler"))) void TIM1_UP_IRQHandler(void);   /* 0x0803F3FE */
__attribute__((weak, alias("Default_Handler"))) void TIM2_IRQHandler(void);      /* 0x0803F58E */
__attribute__((weak, alias("Default_Handler"))) void TIM3_IRQHandler(void);      /* 0x0803F680 */
__attribute__((weak, alias("Default_Handler"))) void TIM4_IRQHandler(void);      /* 0x0803F6B2 */
__attribute__((weak, alias("Default_Handler"))) void USART1_IRQHandler(void);    /* 0x0804031A */
__attribute__((weak, alias("Default_Handler"))) void USART2_IRQHandler(void);    /* 0x08040ECE */
__attribute__((weak, alias("Default_Handler"))) void USART3_IRQHandler(void);    /* 0x08041C66 */
__attribute__((weak, alias("Default_Handler"))) void EXTI15_10_IRQHandler(void); /* 0x0800FC34 */
__attribute__((weak, alias("Default_Handler"))) void TIM8_UP_IRQHandler(void);   /* 0x0803FA26 */
__attribute__((weak, alias("Default_Handler"))) void TIM5_IRQHandler(void);      /* 0x0803F6E4 */
__attribute__((weak, alias("Default_Handler"))) void TIM6_IRQHandler(void);      /* 0x0803F7FC */
__attribute__((weak, alias("Default_Handler"))) void TIM7_IRQHandler(void);      /* 0x0803F84A */
__attribute__((weak)) void SystemInit(void) {}                                   /* 0x0803EE2C */

/* ============================================================================
 * 2. CORTEX-M3 VECTOR TABLE (0x08000000 - 0x0800012F)
 * ============================================================================
 * 76 words (304 bytes):
 *   - Word 0: Initial Main Stack Pointer = 0x2000CE38
 *   - Words 1..15: ARM Cortex-M3 system exceptions
 *   - Words 16..75: STM32F103HD / APM32E103 external interrupts (IRQ0..IRQ59)
 */
__attribute__((section(".isr_vector"), used))
const uintptr_t g_pfnVectors[76] = {
    /* --- Cortex-M3 Core Exceptions (0x08000000 - 0x0800003F) --- */
    [0]  = INITIAL_MSP_ADDR,                    /* 0x08000000: Top of Stack (0x2000CE38)       */
    [1]  = (uintptr_t)Reset_Handler,            /* 0x08000004: Reset Handler (0x08000149)      */
    [2]  = (uintptr_t)NMI_Handler,              /* 0x08000008: NMI Handler (0x08016AB7)        */
    [3]  = (uintptr_t)HardFault_Handler,        /* 0x0800000C: HardFault Handler (0x080153FF)  */
    [4]  = (uintptr_t)MemManage_Handler,        /* 0x08000010: MPU Fault Handler (0x08016439)  */
    [5]  = (uintptr_t)BusFault_Handler,         /* 0x08000014: BusFault Handler (0x08000E0D)   */
    [6]  = (uintptr_t)UsageFault_Handler,       /* 0x08000018: UsageFault Handler (0x080422C5) */
    [7]  = 0x00000000UL,                        /* 0x0800001C: Reserved                        */
    [8]  = 0x00000000UL,                        /* 0x08000020: Reserved                        */
    [9]  = 0x00000000UL,                        /* 0x08000024: Reserved                        */
    [10] = 0x00000000UL,                        /* 0x08000028: Reserved                        */
    [11] = (uintptr_t)SVC_Handler,              /* 0x0800002C: SVCall Handler (0x08037927)     */
    [12] = (uintptr_t)DebugMon_Handler,         /* 0x08000030: Debug Monitor (0x080043B7)      */
    [13] = 0x00000000UL,                        /* 0x08000034: Reserved                        */
    [14] = (uintptr_t)PendSV_Handler,           /* 0x08000038: PendSV Handler (0x08021793)     */
    [15] = (uintptr_t)SysTick_Handler,          /* 0x0800003C: SysTick Handler (0x0803DFDF)    */

    /* --- STM32F103HD / APM32E103 External Interrupts (0x08000040 - 0x0800012F) --- */
    [16] = (uintptr_t)Default_Handler,          /* 0x08000040 [IRQ 0]:  WWDG (0x08000163)              */
    [17] = (uintptr_t)Default_Handler,          /* 0x08000044 [IRQ 1]:  PVD (0x08000163)               */
    [18] = (uintptr_t)Default_Handler,          /* 0x08000048 [IRQ 2]:  TAMPER (0x08000163)            */
    [19] = (uintptr_t)Default_Handler,          /* 0x0800004C [IRQ 3]:  RTC (0x08000163)               */
    [20] = (uintptr_t)Default_Handler,          /* 0x08000050 [IRQ 4]:  FLASH (0x08000163)             */
    [21] = (uintptr_t)Default_Handler,          /* 0x08000054 [IRQ 5]:  RCC (0x08000163)               */
    [22] = (uintptr_t)EXTI0_IRQHandler,         /* 0x08000058 [IRQ 6]:  EXTI0 (0x0800FB4F)             */
    [23] = (uintptr_t)EXTI1_IRQHandler,         /* 0x0800005C [IRQ 7]:  EXTI1 (0x08010621)             */
    [24] = (uintptr_t)EXTI2_IRQHandler,         /* 0x08000060 [IRQ 8]:  EXTI2 (0x080108D1)             */
    [25] = (uintptr_t)Default_Handler,          /* 0x08000064 [IRQ 9]:  EXTI3 (0x08000163)             */
    [26] = (uintptr_t)EXTI4_IRQHandler,         /* 0x08000068 [IRQ 10]: EXTI4 (0x08011EE3)             */
    [27] = (uintptr_t)Default_Handler,          /* 0x0800006C [IRQ 11]: DMA1_Channel1 (0x08000163)     */
    [28] = (uintptr_t)Default_Handler,          /* 0x08000070 [IRQ 12]: DMA1_Channel2 (0x08000163)     */
    [29] = (uintptr_t)Default_Handler,          /* 0x08000074 [IRQ 13]: DMA1_Channel3 (0x08000163)     */
    [30] = (uintptr_t)Default_Handler,          /* 0x08000078 [IRQ 14]: DMA1_Channel4 (0x08000163)     */
    [31] = (uintptr_t)Default_Handler,          /* 0x0800007C [IRQ 15]: DMA1_Channel5 (0x08000163)     */
    [32] = (uintptr_t)Default_Handler,          /* 0x08000080 [IRQ 16]: DMA1_Channel6 (0x08000163)     */
    [33] = (uintptr_t)Default_Handler,          /* 0x08000084 [IRQ 17]: DMA1_Channel7 (0x08000163)     */
    [34] = (uintptr_t)Default_Handler,          /* 0x08000088 [IRQ 18]: ADC1_2 (0x08000163)            */
    [35] = (uintptr_t)Default_Handler,          /* 0x0800008C [IRQ 19]: USB_HP_CAN1_TX (0x08000163)    */
    [36] = (uintptr_t)Default_Handler,          /* 0x08000090 [IRQ 20]: USB_LP_CAN1_RX0 (0x08000163)   */
    [37] = (uintptr_t)Default_Handler,          /* 0x08000094 [IRQ 21]: CAN1_RX1 (0x08000163)          */
    [38] = (uintptr_t)Default_Handler,          /* 0x08000098 [IRQ 22]: CAN1_SCE (0x08000163)          */
    [39] = (uintptr_t)EXTI9_5_IRQHandler,       /* 0x0800009C [IRQ 23]: EXTI9_5 (0x0801223D)           */
    [40] = (uintptr_t)Default_Handler,          /* 0x080000A0 [IRQ 24]: TIM1_BRK (0x08000163)          */
    [41] = (uintptr_t)TIM1_UP_IRQHandler,       /* 0x080000A4 [IRQ 25]: TIM1_UP (0x0803F3FF)           */
    [42] = (uintptr_t)Default_Handler,          /* 0x080000A8 [IRQ 26]: TIM1_TRG_COM (0x08000163)      */
    [43] = (uintptr_t)Default_Handler,          /* 0x080000AC [IRQ 27]: TIM1_CC (0x08000163)           */
    [44] = (uintptr_t)TIM2_IRQHandler,          /* 0x080000B0 [IRQ 28]: TIM2 (0x0803F58F)              */
    [45] = (uintptr_t)TIM3_IRQHandler,          /* 0x080000B4 [IRQ 29]: TIM3 (0x0803F681)              */
    [46] = (uintptr_t)TIM4_IRQHandler,          /* 0x080000B8 [IRQ 30]: TIM4 (0x0803F6B3)              */
    [47] = (uintptr_t)Default_Handler,          /* 0x080000BC [IRQ 31]: I2C1_EV (0x08000163)           */
    [48] = (uintptr_t)Default_Handler,          /* 0x080000C0 [IRQ 32]: I2C1_ER (0x08000163)           */
    [49] = (uintptr_t)Default_Handler,          /* 0x080000C4 [IRQ 33]: I2C2_EV (0x08000163)           */
    [50] = (uintptr_t)Default_Handler,          /* 0x080000C8 [IRQ 34]: I2C2_ER (0x08000163)           */
    [51] = (uintptr_t)Default_Handler,          /* 0x080000CC [IRQ 35]: SPI1 (0x08000163)              */
    [52] = (uintptr_t)Default_Handler,          /* 0x080000D0 [IRQ 36]: SPI2 (0x08000163)              */
    [53] = (uintptr_t)USART1_IRQHandler,        /* 0x080000D4 [IRQ 37]: USART1 (0x0804031B)            */
    [54] = (uintptr_t)USART2_IRQHandler,        /* 0x080000D8 [IRQ 38]: USART2 (0x08040ECF)            */
    [55] = (uintptr_t)USART3_IRQHandler,        /* 0x080000DC [IRQ 39]: USART3 (0x08041C67)            */
    [56] = (uintptr_t)EXTI15_10_IRQHandler,     /* 0x080000E0 [IRQ 40]: EXTI15_10 (0x0800FC35)         */
    [57] = (uintptr_t)Default_Handler,          /* 0x080000E4 [IRQ 41]: RTCAlarm (0x08000163)          */
    [58] = (uintptr_t)Default_Handler,          /* 0x080000E8 [IRQ 42]: USBWakeUp (0x08000163)         */
    [59] = (uintptr_t)Default_Handler,          /* 0x080000EC [IRQ 43]: TIM8_BRK (0x08000163)          */
    [60] = (uintptr_t)TIM8_UP_IRQHandler,       /* 0x080000F0 [IRQ 44]: TIM8_UP (0x0803FA27)           */
    [61] = (uintptr_t)Default_Handler,          /* 0x080000F4 [IRQ 45]: TIM8_TRG_COM (0x08000163)      */
    [62] = (uintptr_t)Default_Handler,          /* 0x080000F8 [IRQ 46]: TIM8_CC (0x08000163)           */
    [63] = (uintptr_t)Default_Handler,          /* 0x080000FC [IRQ 47]: ADC3 (0x08000163)              */
    [64] = (uintptr_t)Default_Handler,          /* 0x08000100 [IRQ 48]: FSMC (0x08000163)              */
    [65] = (uintptr_t)Default_Handler,          /* 0x08000104 [IRQ 49]: SDIO (0x08000163)              */
    [66] = (uintptr_t)TIM5_IRQHandler,          /* 0x08000108 [IRQ 50]: TIM5 (0x0803F6E5)              */
    [67] = (uintptr_t)Default_Handler,          /* 0x0800010C [IRQ 51]: SPI3 (0x08000163)              */
    [68] = (uintptr_t)Default_Handler,          /* 0x08000110 [IRQ 52]: UART4 (0x08000163)             */
    [69] = (uintptr_t)Default_Handler,          /* 0x08000114 [IRQ 53]: UART5 (0x08000163)             */
    [70] = (uintptr_t)TIM6_IRQHandler,          /* 0x08000118 [IRQ 54]: TIM6 (0x0803F7FD)              */
    [71] = (uintptr_t)TIM7_IRQHandler,          /* 0x0800011C [IRQ 55]: TIM7 (0x0803F84B)              */
    [72] = (uintptr_t)Default_Handler,          /* 0x08000120 [IRQ 56]: DMA2_Channel1 (0x08000163)     */
    [73] = (uintptr_t)Default_Handler,          /* 0x08000124 [IRQ 57]: DMA2_Channel2 (0x08000163)     */
    [74] = (uintptr_t)Default_Handler,          /* 0x08000128 [IRQ 58]: DMA2_Channel3 (0x08000163)     */
    [75] = (uintptr_t)Default_Handler           /* 0x0800012C [IRQ 59]: DMA2_Channel4_5 (0x08000163)   */
};

/* ============================================================================
 * 3. KEIL SCATTERLOAD REGION TABLE & DECOMPRESSOR (0x08000130 - 0x0804A97F)
 * ============================================================================ */

typedef void (*scatter_handler_fn)(const uint8_t *src, uint8_t *dst, uint32_t size);

typedef struct {
    const uint8_t     *src;     /* Load address in Flash                      */
    uint8_t           *dst;     /* Execution address in SRAM                  */
    uint32_t           size;    /* Uncompressed byte length in SRAM           */
    scatter_handler_fn handler; /* Decompressor / copy / zeroinit function    */
} ScatterRegionEntry;

/**
 * Compressed .data stream stored in Flash at 0x0807D800 - 0x0807D867 (104 bytes).
 * Decompressed at boot by __decompress1 (0x08000AC4) into SRAM 0x20000000 - 0x200007F3 (2,036 bytes).
 */
__attribute__((section(".rodata.scatter_data"), used))
const uint8_t g_flash_compressed_data[104] = {
    /* 0x0807d800 */ 0x45, 0x02, 0x04, 0x06, 0x08, 0x2d, 0x01, 0x02, 0x03, 0x04, 0x04, 0x15, 0x06, 0x07, 0x08, 0x09,
    /* 0x0807d810 */ 0x84, 0xd8, 0xb8, 0x05, 0x79, 0x14, 0x01, 0xd8, 0x23, 0x70, 0x70, 0x13, 0x13, 0x08, 0x32, 0x01,
    /* 0x0807d820 */ 0x3a, 0x34, 0xf1, 0x4e, 0xc0, 0x13, 0x82, 0x2d, 0x50, 0x06, 0x31, 0x32, 0x39, 0x20, 0x0b, 0x10,
    /* 0x0807d830 */ 0xfe, 0x24, 0x01, 0x4a, 0x82, 0x64, 0x09, 0xed, 0xd6, 0x23, 0xe9, 0xcd, 0x2b, 0xe5, 0xb5, 0x1a,
    /* 0x0807d840 */ 0x01, 0xff, 0x01, 0xff, 0x01, 0xff, 0x01, 0xc7, 0x12, 0x1f, 0x12, 0x3b, 0x12, 0x5a, 0x12, 0x78,
    /* 0x0807d850 */ 0x12, 0x97, 0x12, 0xb5, 0x12, 0xd4, 0x12, 0xf3, 0x00, 0x09, 0xff, 0x11, 0x01, 0x30, 0x01, 0x4e,
    /* 0x0807d860 */ 0x01, 0x6d, 0x01, 0x01, 0xff, 0x01, 0xe0, 0x00
};

/**
 * Keil Region$$Table at 0x0804A960 - 0x0804A980:
 *   Entry 0 (0x0804A960): src=0x0807D800, dst=0x20000000, size=0x000007F4, fn=__decompress1 (0x08000AC4)
 *   Entry 1 (0x0804A970): src=0x0807D868, dst=0x200007F4, size=0x0000C644, fn=__scatterload_zeroinit (0x08042C28)
 */
__attribute__((section(".rodata.region_table"), used))
const ScatterRegionEntry Region_Table[2] = {
    {
        (const uint8_t *)FLASH_SCATTER_DATA_BASE, /* 0x0807D800 */
        (uint8_t *)0x20000000UL,                  /* 0x20000000 */
        0x000007F4UL,                             /* 2,036 bytes (.data) */
        __decompress1                             /* 0x08000AC4 */
    },
    {
        (const uint8_t *)FLASH_SCATTER_ZERO_BASE, /* 0x0807D868 */
        (uint8_t *)0x200007F4UL,                  /* 0x200007F4 */
        0x0000C644UL,                             /* 50,756 bytes (.bss + stack) */
        __scatterload_zeroinit                    /* 0x08042C28 */
    }
};

/**
 * @brief Reset_Handler (0x08000148)
 *
 * Disassembly:
 *   08000148: 4806      ldr r0, [pc, #24]   ; 0x0803ee2d (SystemInit)
 *   0800014a: 4780      blx r0
 *   0800014c: 4806      ldr r0, [pc, #24]   ; 0x08000131 (__main)
 *   0800014e: 4700      bx  r0
 */
void Reset_Handler(void)
{
    SystemInit();
    __main();
}

/**
 * @brief Keil C Runtime Entry Point __main (0x08000130)
 *
 * Disassembly:
 *   08000130: 460ef8df  ldr.w sp, [pc, #14] ; SP = 0x2000ce38
 *   08000134: 0800f000  bl  0x08000aa0      ; __scatterload
 *   08000138: 4803      ldr r0, [pc, #12]   ; 0x080433e5 (main)
 *   0800013a: 4700      bx  r0
 */
void __main(void)
{
    __scatterload();
    (void)main();
    while (1) {
        /* Should never return */
    }
}

/**
 * @brief Keil __scatterload (0x08000AA0)
 *
 * Iterates over Region$$Table (0x0804A960..0x0804A980), invoking each entry's
 * handler with (r0 = src, r1 = dst, r2 = size).
 *
 * Disassembly:
 *   08000aa0: 4c06      ldr r4, [pc, #24]   ; 0x0804a960 (Region$$Table$$Base)
 *   08000aa2: 4d07      ldr r5, [pc, #28]   ; 0x0804a980 (Region$$Table$$Limit)
 *   08000aa4: e006      b   0x08000ab4
 *   08000aa6: c83c      ldmia r0!, {r2, r3, r4, r5} ; wait: r4 is cursor
 *   08000aa6: cc0f      ldmia r4!, {r0, r1, r2, r3}
 *   08000aa8: f043 0301 orr r3, r3, #1
 *   08000aac: e92d 0030 push {r4, r5}
 *   08000ab0: 4798      blx r3
 *   08000ab2: e8bd 0030 pop {r4, r5}
 *   08000ab4: 42ac      cmp r4, r5
 *   08000ab6: d3f6      bcc 0x08000aa6
 */
void __scatterload(void)
{
    const ScatterRegionEntry *curr = &Region_Table[0];
    const ScatterRegionEntry *end  = &Region_Table[2];

    while (curr < end) {
        curr->handler(curr->src, curr->dst, curr->size);
        curr++;
    }
}

/**
 * @brief Keil __decompress1 / __dczerorl2 (0x08000AC4 - 0x08000B19)
 *
 * Decompresses Keil's LZ77 + Zero-Run-Length encoded `.data` section from
 * Flash (`src` = 0x0807D800) to SRAM (`dst` = 0x20000000, `size` = 0x7F4).
 *
 * Control byte format (`ctrl = *src++`):
 *   - Low nibble `lit_len = ctrl & 0x0F`:
 *       If `lit_len == 0`, reads extension bytes (`b = *src++`) while `b == 0`,
 *       accumulating `lit_len += (255 * zero_count) + b`.
 *       Then copies `lit_len - 1` literal bytes from `src` to `dst`.
 *   - High nibble `run_code = ctrl >> 4`:
 *       - If `(ctrl & 0x80) == 0` (Zero RLE run):
 *           `zero_len = ctrl >> 4`.
 *           If `zero_len == 7`, reads extension bytes (`b = *src++`) while `b == 0xFF`,
 *           accumulating `zero_len += b`.
 *           Writes `zero_len` zero bytes (`0x00`) to `dst`.
 *       - If `(ctrl & 0x80) != 0` (LZ77 back-reference):
 *           `offset = *src++` (1..255 bytes back from current `dst`).
 *           `match_len = (ctrl >> 4) & 0x07`.
 *           If `match_len == 0`, reads extension bytes (`b = *src++`) while `b == 0`,
 *           accumulating `match_len += (255 * zero_count) + b`.
 *           Copies `match_len + 1` bytes from `dst - offset` to `dst`.
 */
void __decompress1(const uint8_t *src, uint8_t *dst, uint32_t size)
{
    uint8_t * const dst_end = dst + size;

    while (dst < dst_end) {
        uint32_t ctrl = *src++;

        /* --- Step 1: Copy literal bytes --- */
        uint32_t lit_len = ctrl & 0x0FU;
        if (lit_len == 0U) {
            uint32_t ext;
            do {
                ext = *src++;
                if (ext == 0U) {
                    lit_len += 255U;
                }
            } while (ext == 0U);
            lit_len += ext;
        }

        while (--lit_len != 0U) {
            *dst++ = *src++;
        }

        if (dst >= dst_end) {
            break;
        }

        /* --- Step 2: Zero run or LZ77 back-reference --- */
        if ((ctrl & 0x80U) == 0U) {
            /* Zero-run-length encoding (RLE of 0x00 bytes) */
            uint32_t zero_len = ctrl >> 4;
            if (zero_len == 7U) {
                uint32_t ext;
                do {
                    ext = *src++;
                    zero_len += ext;
                } while (ext == 0xFFU);
            }
            while (zero_len-- != 0U) {
                *dst++ = 0x00U;
            }
        } else {
            /* LZ77 back-reference copy */
            uint32_t match_len = (ctrl >> 4) & 0x07U;
            uint32_t offset    = *src++;
            if (match_len == 0U) {
                uint32_t ext;
                do {
                    ext = *src++;
                    if (ext == 0U) {
                        match_len += 255U;
                    }
                } while (ext == 0U);
                match_len += ext;
            }
            const uint8_t *ref = dst - offset;
            match_len += 1U;
            while (match_len-- != 0U) {
                *dst++ = *ref++;
            }
        }
    }
}

/**
 * @brief Keil __scatterload_copy (0x08042C18)
 */
void __scatterload_copy(const uint8_t *src, uint8_t *dst, uint32_t size)
{
    while (size >= 4U) {
        *(uint32_t *)dst = *(const uint32_t *)src;
        src  += 4U;
        dst  += 4U;
        size -= 4U;
    }
    while (size != 0U) {
        *dst++ = *src++;
        size--;
    }
}

/**
 * @brief Keil __scatterload_null (0x08042C26)
 */
void __scatterload_null(void)
{
    return;
}

/**
 * @brief Keil __scatterload_zeroinit (0x08042C28)
 *
 * Zero-initializes `size` bytes at `dst` (used for 0x200007F4..0x2000CE37).
 */
void __scatterload_zeroinit(const uint8_t *src, uint8_t *dst, uint32_t size)
{
    (void)src;
    while (size >= 4U) {
        *(uint32_t *)dst = 0x00000000UL;
        dst  += 4U;
        size -= 4U;
    }
    while (size != 0U) {
        *dst++ = 0x00U;
        size--;
    }
}

/* ============================================================================
 * 4. INITIALIZED .DATA VARIABLES (SRAM 0x20000000 - 0x200007F3)
 * ============================================================================
 * Reconstructed from sram_data.bin (decompressed output of __decompress1):
 *   - 0x20000000..0x20000003: ADCPrescTable[4]         (stm32f10x_rcc.c)
 *   - 0x20000004..0x20000013: APBAHBPrescTable[16]     (stm32f10x_rcc.c)
 *   - 0x20000014..0x20000017: SystemCoreClock          (system_stm32f10x.c, 96 MHz)
 *   - 0x2000001C..0x2000002B: AHBPrescTable[16]        (system_stm32f10x.c)
 *   - 0x20000100..0x20000137: Default OB1 block header (56 bytes)
 *   - 0x200004FC..0x20000517: RTC cumulative days before month table (13 uint16_t)
 */

/* 0x20000000: ADC prescaler table used by RCC_GetClocksFreq (0x08036bc4) */
uint8_t ADCPrescTable[4] = {
    2, 4, 6, 8
};

/* 0x20000004: APB/AHB prescaler shift table used by RCC_GetClocksFreq (0x08036bc4) */
uint8_t APBAHBPrescTable[16] = {
    0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9
};

/* 0x20000014: System core clock frequency = 96,000,000 Hz (0x05B8D800) */
uint32_t SystemCoreClock = 96000000UL;

/* 0x2000001C: AHB prescaler table from system_stm32f10x.c */
uint8_t AHBPrescTable[16] = {
    0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9, 0, 0, 0, 0
};

/**
 * 0x20000100 - 0x20000137: Default empty OB1 user program block template (56 bytes).
 * Contains the STEP 7-Micro/WIN block signature ("pp", 0x13, 0x08) and a minimal
 * compiled END instruction stub. Note that byte 0 (0x20000100) is also reused at
 * runtime as the Port 2 (USART3) transmit-complete flag (`DAT_20000100`).
 */
uint8_t g_default_ob1_block_data[56] = {
    /* 0x20000100 */ 0x00, 0x70, 0x70, 0x00, 0x00, 0x13, 0x08, 0x00,
    /* 0x20000108 */ 0x32, 0x01, 0x3A, 0x34, 0x00, 0x00, 0x00, 0x00,
    /* 0x20000110 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* 0x20000118 */ 0x00, 0x00, 0x4E, 0xC0, 0x00, 0x00, 0x82, 0x2D,
    /* 0x20000120 */ 0x50, 0x06, 0x31, 0x32, 0x39, 0x20, 0x0B, 0x10,
    /* 0x20000128 */ 0xFE, 0x24, 0x01, 0x4A, 0x82, 0x64, 0x09, 0xED,
    /* 0x20000130 */ 0xD6, 0x23, 0xE9, 0xCD, 0x2B, 0xE5, 0xB5, 0x1A
};

/**
 * 0x200004FC - 0x20000517: Cumulative days elapsed before each month (non-leap year)
 * Used by RTC calendar conversion routines (0x08042744 / 0x08042874).
 */
uint16_t g_rtc_month_day_offsets[13] = {
    0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365
};
