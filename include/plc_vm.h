/**
 * @file plc_vm.h
 * @brief SIMATIC S7-226 (APM32E103) User Program Parser/Validator and
 *        MC7/STL Virtual Machine Bytecode Interpreter Definitions.
 *
 * Reconstructed from firmware flash regions:
 *   - User Program Parser & CALL Parameter Marshaler: 0x08000e0e .. 0x08003e5b
 *   - Operand Address Decoders & Writers:             0x0801452e .. 0x08016550,
 *                                                     0x0803b424 .. 0x0803bf40
 *   - MC7/STL Bytecode Interpreter (FUN_08021948):    0x08021948 .. 0x08036b1f
 */

#ifndef PLC_VM_H
#define PLC_VM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 1. S7-226 Physical Memory Map & Virtual Pointer Area Identifiers
 * ============================================================================
 * S7-200 32-bit virtual pointers (&VBx, &IBx, etc.) encode:
 *   bits [31:24] = Area ID (plc_mem_area_t)
 *   bits [23:0]  = Byte offset within that memory area
 * Translated to physical SRAM/Flash addresses by plc_vptr_to_phys (0x08000e4a)
 * and plc_vptr_to_phys_local (0x08000f56).
 */

typedef enum {
    PLC_AREA_I      = 0x00, /* Discrete Input Process Image (IB0..IB15, 16B)       */
    PLC_AREA_Q      = 0x01, /* Discrete Output Process Image (QB0..QB15, 16B)      */
    PLC_AREA_M      = 0x02, /* Bit Memory Area (MB0..MB31, 32B)                    */
    PLC_AREA_S      = 0x03, /* Sequential Control Relay Area (SB0..SB31, 32B)      */
    PLC_AREA_SM     = 0x04, /* Special Memory Area (SMB0..SMB549, 550B)            */
    PLC_AREA_OB1    = 0x05, /* Program / Data Block Flash Region (64KB)            */
    PLC_AREA_V      = 0x08, /* Variable Data Memory (VB0..VB10239, 10240B)         */
    PLC_AREA_T      = 0x09, /* Timer Current Value Words (T0..T255, 512B)          */
    PLC_AREA_C      = 0x0A, /* Counter Current Value Words (C0..C255, 512B)        */
    PLC_AREA_HC     = 0x0B, /* High-Speed Counter Current DWords (HC0..HC5, 24B)   */
    PLC_AREA_AI     = 0x0C, /* Analog Input Words (AIW0..AIW62, 64B)               */
    PLC_AREA_AQ     = 0x0D, /* Analog Output Words (AQW0..AQW62, 64B)              */
    PLC_AREA_L      = 0x0E  /* Subroutine/Interrupt Local Stack (LB0..LB63, 64B)   */
} plc_mem_area_t;

/* Physical SRAM and Flash base addresses on the APM32E103 S7-226 */
#define PLC_PHYS_BASE_I             ((uint8_t *)0x20005BD0U)
#define PLC_PHYS_BASE_Q             ((uint8_t *)0x20001046U)
#define PLC_PHYS_BASE_M             ((uint8_t *)0x20001000U)
#define PLC_PHYS_BASE_S             ((uint8_t *)0x20001024U)
#define PLC_PHYS_BASE_SM            ((uint8_t *)0x20005F28U)
#define PLC_PHYS_BASE_OB1_FLASH     ((uint8_t *)0x0806D800U)
#define PLC_PHYS_BASE_OB1_BACKUP    ((uint8_t *)0x08075800U)
#define PLC_PHYS_BASE_V             ((uint8_t *)0x2000108CU)
#define PLC_PHYS_BASE_T_VAL         ((uint16_t *)0x20004000U)
#define PLC_PHYS_BASE_T_STATUS      ((uint8_t *)0x2000422CU)
#define PLC_PHYS_BASE_T_LAST_TICK   ((uint16_t *)0x20005BF0U)
#define PLC_PHYS_BASE_C_VAL         ((uint16_t *)0x20004440U)
#define PLC_PHYS_BASE_C_STATUS      ((uint8_t *)0x20005000U)
#define PLC_PHYS_BASE_HC_VAL        ((int32_t *)0x20006150U)
#define PLC_PHYS_BASE_AI            ((uint16_t *)0x20005E1AU)
#define PLC_PHYS_BASE_AQ            ((uint16_t *)0x20005E98U)
#define PLC_PHYS_BASE_EDGE_BITMAP   ((uint8_t *)0x2000BC00U)
#define PLC_PHYS_BASE_EDGE_PC_TBL   ((uint16_t *)0x2000BBFEU)
#define PLC_PHYS_BASE_LOCAL_STACK   ((uint8_t *)0x200077EFU)

/* Area size limits enforced by plc_vptr_to_phys (0x08000e4a) */
#define PLC_LIMIT_I_BYTES           0x000010U   /* 16 bytes (I0.0 .. I15.7)     */
#define PLC_LIMIT_Q_BYTES           0x000010U   /* 16 bytes (Q0.0 .. Q15.7)     */
#define PLC_LIMIT_M_BYTES           0x000020U   /* 32 bytes (M0.0 .. M31.7)     */
#define PLC_LIMIT_S_BYTES           0x000020U   /* 32 bytes (S0.0 .. S31.7)     */
#define PLC_LIMIT_SM_BYTES          0x000226U   /* 550 bytes (SMB0 .. SMB549)   */
#define PLC_LIMIT_OB1_BYTES         0x010000U   /* 64 KB user program flash     */
#define PLC_LIMIT_V_BYTES           0x002800U   /* 10240 bytes (VB0 .. VB10239) */
#define PLC_LIMIT_T_BYTES           0x000200U   /* 256 words (T0 .. T255)       */
#define PLC_LIMIT_C_BYTES           0x000200U   /* 256 words (C0 .. C255)       */
#define PLC_LIMIT_AI_BYTES          0x00003FU   /* 32 words (AIW0 .. AIW62)     */
#define PLC_LIMIT_AQ_BYTES          0x00003FU   /* 32 words (AQW0 .. AQW62)     */
#define PLC_LIMIT_L_BYTES           0x000040U   /* 64 bytes (LB0 .. LB63)       */
#define PLC_MAX_EDGE_DETECTORS      256U        /* EU/ED max 256 edge bits      */
#define PLC_MAX_SBR_COUNT           64U         /* SBR_0 .. SBR_63              */
#define PLC_MAX_INT_COUNT           128U        /* INT_0 .. INT_127             */
#define PLC_MAX_INT_EVENTS          34U         /* Interrupt events 0 .. 33     */
#define PLC_MAX_FOR_DEPTH           8U          /* Maximum nested FOR/NEXT      */
#define PLC_MAX_CALL_DEPTH          9U          /* Maximum nested SBR CALLs     */

/* Watchdog reload register (APM32E103 IWDG->KR at 0x40003000) */
#define PLC_IWDG_KR_REG             (*(volatile uint32_t *)0x40003000U)
#define PLC_IWDG_RELOAD_KEY         0x0000AAAAU

/* ============================================================================
 * 2. Accumulators (AC0..AC3), Status Bytes (SMB0..SMB4), and VM Error Codes
 * ============================================================================
 */

/* Four 32-bit accumulators AC0..AC3 at 0x20005f18..0x20005f27 */
#define PLC_ACCUM_BASE              ((uint32_t *)0x20005F18U)
#define PLC_AC0                     (PLC_ACCUM_BASE[0]) /* 0x20005F18 */
#define PLC_AC1                     (PLC_ACCUM_BASE[1]) /* 0x20005F1C */
#define PLC_AC2                     (PLC_ACCUM_BASE[2]) /* 0x20005F20 */
#define PLC_AC3                     (PLC_ACCUM_BASE[3]) /* 0x20005F24 */

/* Special Memory Bytes SMB0, SMB1, SMB4 */
#define PLC_SMB0                    (PLC_PHYS_BASE_SM[0]) /* 0x20005F28: System status bits */
#define PLC_SMB1                    (PLC_PHYS_BASE_SM[1]) /* 0x20005F29: Execution status   */
#define PLC_SMB4                    (PLC_PHYS_BASE_SM[4]) /* 0x20005F2C: Interrupt/Error    */

/* SMB0 Bit Masks (0x20005F28) */
#define PLC_SM0_0_ALWAYS_ON         (1U << 0) /* SM0.0: Always 1                     */
#define PLC_SM0_1_FIRST_SCAN        (1U << 1) /* SM0.1: 1 on first scan only         */
#define PLC_SM0_2_RETENTIVE_LOST    (1U << 2) /* SM0.2: Retentive data lost          */
#define PLC_SM0_3_RUN_POWERUP       (1U << 3) /* SM0.3: Power-up into RUN mode       */
#define PLC_SM0_4_CLOCK_60S         (1U << 4) /* SM0.4: 60s clock pulse (30s on/off) */
#define PLC_SM0_5_CLOCK_1S          (1U << 5) /* SM0.5: 1s clock pulse (0.5s on/off) */
#define PLC_SM0_6_SCAN_CLOCK        (1U << 6) /* SM0.6: Scan cycle toggle bit        */
#define PLC_SM0_7_MODE_SWITCH_TERM  (1U << 7) /* SM0.7: Mode switch in TERM position */

/* SMB1 Instruction Execution Status Bits (0x20005F29) */
#define PLC_SM1_0_ZERO              (1U << 0) /* SM1.0: Result of operation == 0     */
#define PLC_SM1_1_OVERFLOW          (1U << 1) /* SM1.1: Overflow or illegal value    */
#define PLC_SM1_2_NEGATIVE          (1U << 2) /* SM1.2: Result is negative (< 0)     */
#define PLC_SM1_3_DIV_BY_ZERO       (1U << 3) /* SM1.3: Division by zero attempted   */
#define PLC_SM1_4_TABLE_OVERFLOW    (1U << 4) /* SM1.4: ATT table full               */
#define PLC_SM1_5_TABLE_EMPTY       (1U << 5) /* SM1.5: LIFO/FIFO table empty        */
#define PLC_SM1_6_NOT_BCD           (1U << 6) /* SM1.6: Non-BCD value in conversion  */
#define PLC_SM1_7_NOT_HEX_ASCII     (1U << 7) /* SM1.7: Invalid ASCII hex digit      */

/* SMB4 System Error & Interrupt Status Bits (0x20005F2C) */
#define PLC_SM4_3_RUN_TIME_ERROR    (1U << 3) /* SM4.3: Non-fatal runtime error      */
#define PLC_SM4_4_INT_ENABLED       (1U << 4) /* SM4.4: Global interrupt enable flag */

/* ENO (Enable Output) tracking byte flag (bit 2 = 0x04) */
#define PLC_ENO_FLAG_BIT            (1U << 2) /* 0x04: Instruction ENO = 1           */

/* Timer Status Byte (0x2000422C[t_num]) & Counter Status Byte (0x20005000[c_num]) */
#define PLC_TIMER_BIT_MASK          (1U << 1) /* 0x02: Timer output bit (T-bit)      */
#define PLC_TIMER_PREV_EN_MASK      (1U << 3) /* 0x08: Previous scan enable state    */
#define PLC_TIMER_ACTIVE_MASK       (1U << 7) /* 0x80: Timer actively timing         */
#define PLC_COUNTER_BIT_MASK        (1U << 4) /* 0x10: Counter output bit (C-bit)    */
#define PLC_COUNTER_PREV_CU_MASK    (1U << 0) /* 0x01: Previous CU / CD edge state   */
#define PLC_COUNTER_PREV_CD_MASK    (1U << 1) /* 0x02: Previous CD edge state (CTUD) */

/* Non-Fatal Compile & Runtime Error Codes written to DAT_200003ba (g_plc_vm_error) */
typedef enum {
    PLC_ERR_NONE                    = 0x0000,
    /* Compile / Rule-check errors (0x8001 .. 0x803F, set by parser or VM) */
    PLC_ERR_ILLEGAL_FOR_NEXT        = 0x8001, /* FOR/NEXT mismatch or nesting > 8    */
    PLC_ERR_MISSING_MEND            = 0x8006, /* Missing MEND / RET / RETI           */
    PLC_ERR_DUP_INT_EVENT           = 0x8013, /* Duplicate interrupt event attach    */
    PLC_ERR_DISALLOWED_IN_INT       = 0x801F, /* Instruction not allowed in INT_n    */
    PLC_ERR_ILLEGAL_SCR_IN_SBR      = 0x8021, /* SCR instruction illegal in SBR/INT  */
    PLC_ERR_XMT_IN_INT_CONFLICT     = 0x803C, /* XMT conflict in interrupt routine   */
    PLC_ERR_RCV_IN_INT_CONFLICT     = 0x803D, /* RCV conflict in interrupt routine   */
    PLC_ERR_ENCO_ZERO_INPUT         = 0x803F, /* ENCO input word is zero             */
    /* Runtime non-fatal errors (0x8060 .. 0x8128, set with SM4.3) */
    PLC_ERR_OPERAND_RANGE           = 0x8066, /* Value or operand out of range       */
    PLC_ERR_INDIRECT_ADDR_RANGE     = 0x8091, /* Indirect pointer out of area bounds */
    PLC_ERR_HDEF_DUP_OR_MODE        = 0x8102, /* HDEF illegal mode or duplicate      */
    PLC_ERR_EDGE_OVERFLOW           = 0x8120, /* >256 EU/ED edge instructions        */
    PLC_ERR_UNSUPPORTED_SUB_OPCODE  = 0x8122, /* Unsupported 0xFE extended opcode    */
    PLC_ERR_IMM_IO_RANGE            = 0x8123, /* Immediate I/O byte out of range     */
    PLC_ERR_XMT_NOT_FREEPORT        = 0x8127, /* XMT port not in Freeport mode       */
    PLC_ERR_RCV_NOT_FREEPORT        = 0x8128  /* RCV port not in Freeport mode       */
} plc_vm_error_t;

/* Global VM non-fatal error code word at 0x200003BA */
#define PLC_VM_ERROR_REG            (*(volatile uint16_t *)0x200003BAU)

/* ============================================================================
 * 3. Operand Addressing Mode Nibble (4-Bit Mode in MC7 Instructions)
 * ============================================================================
 */
typedef enum {
    PLC_ADDR_MODE_DIRECT_MEM    = 0x0, /* 2B [area_hi, off_lo] or 4B [0xF0, area, hi, lo] */
    PLC_ADDR_MODE_IMM_BYTE      = 0x4, /* 1-byte immediate constant                       */
    PLC_ADDR_MODE_IMM_WORD      = 0x5, /* 2-byte big-endian immediate constant            */
    PLC_ADDR_MODE_IMM_DWORD     = 0x6, /* 4-byte big-endian immediate constant / float    */
    PLC_ADDR_MODE_IMM_ADDR_PTR  = 0x7, /* 4-byte virtual address pointer (&VBx, etc.)     */
    PLC_ADDR_MODE_INDIRECT_MEM  = 0x8, /* Indirect pointer stored in V or L (*VDx / *LDx) */
    PLC_ADDR_MODE_INDIRECT_AC1  = 0x9, /* Indirect pointer in AC1 (*AC1, 0 stream bytes)  */
    PLC_ADDR_MODE_INDIRECT_AC2  = 0xA, /* Indirect pointer in AC2 (*AC2, 0 stream bytes)  */
    PLC_ADDR_MODE_INDIRECT_AC3  = 0xB, /* Indirect pointer in AC3 (*AC3, 0 stream bytes)  */
    PLC_ADDR_MODE_DIRECT_AC0    = 0xC, /* Direct Accumulator AC0 (0 stream bytes)         */
    PLC_ADDR_MODE_DIRECT_AC1    = 0xD, /* Direct Accumulator AC1 (0 stream bytes)         */
    PLC_ADDR_MODE_DIRECT_AC2    = 0xE, /* Direct Accumulator AC2 (0 stream bytes)         */
    PLC_ADDR_MODE_DIRECT_AC3    = 0xF  /* Direct Accumulator AC3 (0 stream bytes)         */
} plc_addr_mode_t;

/* ============================================================================
 * 4. MC7 / STL Primary Opcodes (0x00 .. 0xFE) & 0xFE Extended Sub-Opcodes
 * ============================================================================
 */

/* Compact 2-byte bit logic areas: area_code = (opcode >> 1) & 7 */
typedef enum {
    PLC_BIT_AREA_I  = 0, /* Input bit I0.0 .. I15.7         */
    PLC_BIT_AREA_Q  = 1, /* Output bit Q0.0 .. Q15.7        */
    PLC_BIT_AREA_M  = 2, /* Memory bit M0.0 .. M31.7        */
    PLC_BIT_AREA_S  = 3, /* SCR bit S0.0 .. S31.7           */
    PLC_BIT_AREA_SM = 4, /* Special Memory SM0.0 .. SM255.7 */
    PLC_BIT_AREA_T  = 5, /* Timer bit T0 .. T255            */
    PLC_BIT_AREA_C  = 6  /* Counter bit C0 .. C255          */
} plc_compact_bit_area_t;

/* Primary Opcode Definitions (0x00 .. 0xFE) */
typedef enum {
    /* 0x00..0x5D: Compact 2-Byte Bit Logic (high nibble = op, bits [3:1] = area, bit 0 = hi8) */
    PLC_OP_LD_COMPACT_BASE  = 0x00, /* 0x00..0x0D: LD  (Load bit onto logic stack)     */
    PLC_OP_LDN_COMPACT_BASE = 0x10, /* 0x10..0x1D: LDN (Load inverted bit onto stack)  */
    PLC_OP_A_COMPACT_BASE   = 0x20, /* 0x20..0x2D: A   (AND bit with top of stack)     */
    PLC_OP_AN_COMPACT_BASE  = 0x30, /* 0x30..0x3D: AN  (AND inverted bit with stack)   */
    PLC_OP_O_COMPACT_BASE   = 0x40, /* 0x40..0x4D: O   (OR bit with top of stack)      */
    PLC_OP_ON_COMPACT_BASE  = 0x50, /* 0x50..0x5D: ON  (OR inverted bit with stack)    */
    PLC_OP_ASSIGN_COMPACT   = 0x60, /* 0x60..0x6D: =   (Assign stack top to coil)      */

    /* 0x80..0x8D: 3-Byte / 5-Byte Extended Bit Logic (SM, V, L, and Immediate I/Q) */
    PLC_OP_LD_EXT           = 0x80, /* LD  SMx.y / Vx.y / Lx.y                         */
    PLC_OP_LDN_EXT          = 0x81, /* LDN SMx.y / Vx.y / Lx.y                         */
    PLC_OP_A_EXT            = 0x82, /* A   SMx.y / Vx.y / Lx.y                         */
    PLC_OP_AN_EXT           = 0x83, /* AN  SMx.y / Vx.y / Lx.y                         */
    PLC_OP_O_EXT            = 0x84, /* O   SMx.y / Vx.y / Lx.y                         */
    PLC_OP_ON_EXT           = 0x85, /* ON  SMx.y / Vx.y / Lx.y                         */
    PLC_OP_ASSIGN_EXT       = 0x86, /* =   SMx.y / Vx.y / Lx.y                         */
    PLC_OP_LDI              = 0x87, /* LDI Ix.y (Load Immediate Input bit)             */
    PLC_OP_LDNI             = 0x88, /* LDNI Ix.y (Load Negated Immediate Input bit)    */
    PLC_OP_AI               = 0x89, /* AI  Ix.y (AND Immediate Input bit)              */
    PLC_OP_ANI              = 0x8A, /* ANI Ix.y (AND Negated Immediate Input bit)      */
    PLC_OP_OI               = 0x8B, /* OI  Ix.y (OR Immediate Input bit)               */
    PLC_OP_ONI              = 0x8C, /* ONI Ix.y (OR Negated Immediate Input bit)       */
    PLC_OP_ASSIGN_IMM       = 0x8D, /* =I  Qx.y (Assign Immediate Output coil)         */

    /* 0x90..0xA4: Primary Byte/Word/DWord Contact Comparisons (==, <=, >=) */
    PLC_OP_LDW_EQ           = 0x90, /* LDW=  IN1, IN2                                  */
    PLC_OP_LDB_EQ           = 0x91, /* LDB=  IN1, IN2                                  */
    PLC_OP_AW_EQ            = 0x92, /* AW=   IN1, IN2                                  */
    PLC_OP_AB_EQ            = 0x93, /* AB=   IN1, IN2                                  */
    PLC_OP_OW_EQ            = 0x94, /* OW=   IN1, IN2                                  */
    PLC_OP_OB_EQ            = 0x95, /* OB=   IN1, IN2                                  */
    PLC_OP_LDW_LE           = 0x96, /* LDW<= IN1, IN2                                  */
    PLC_OP_LDB_LE           = 0x97, /* LDB<= IN1, IN2                                  */
    PLC_OP_LDW_GE           = 0x98, /* LDW>= IN1, IN2                                  */
    PLC_OP_LDB_GE           = 0x99, /* LDB>= IN1, IN2                                  */
    PLC_OP_AW_LE            = 0x9A, /* AW<=  IN1, IN2                                  */
    PLC_OP_AB_LE            = 0x9B, /* AB<=  IN1, IN2                                  */
    PLC_OP_AW_GE            = 0x9C, /* AW>=  IN1, IN2                                  */
    PLC_OP_AB_GE            = 0x9D, /* AB>=  IN1, IN2                                  */
    PLC_OP_OW_LE            = 0x9E, /* OW<=  IN1, IN2                                  */
    PLC_OP_OB_LE            = 0x9F, /* OB<=  IN1, IN2                                  */
    PLC_OP_OW_GE            = 0xA0, /* OW>=  IN1, IN2                                  */
    PLC_OP_OB_GE            = 0xA1, /* OB>=  IN1, IN2                                  */
    PLC_OP_LDD_EQ           = 0xA2, /* LDD=  IN1, IN2                                  */
    PLC_OP_LDD_LE           = 0xA3, /* LDD<= IN1, IN2                                  */
    PLC_OP_LDD_GE           = 0xA4, /* LDD>= IN1, IN2                                  */

    /* 0xA5..0xAF: Watchdog, Timers, Counters, and 2-Operand Integer Math */
    PLC_OP_WDR              = 0xA5, /* WDR   (Watchdog Reset, 1 byte)                  */
    PLC_OP_TON              = 0xA6, /* TON   Txxx, PT (On-Delay Timer)                 */
    PLC_OP_TONR             = 0xA7, /* TONR  Txxx, PT (Retentive On-Delay Timer)       */
    PLC_OP_CTU              = 0xA8, /* CTU   Cxxx, PV (Count Up Counter)               */
    PLC_OP_CTUD             = 0xA9, /* CTUD  Cxxx, PV (Count Up/Down Counter)          */
    PLC_OP_ADD_I            = 0xAA, /* +I    IN1, OUT (OUT = OUT + IN1, 16-bit)        */
    PLC_OP_ADD_D            = 0xAB, /* +D    IN1, OUT (OUT = OUT + IN1, 32-bit)        */
    PLC_OP_SUB_I            = 0xAC, /* -I    IN1, OUT (OUT = OUT - IN1, 16-bit)        */
    PLC_OP_SUB_D            = 0xAD, /* -D    IN1, OUT (OUT = OUT - IN1, 32-bit)        */
    PLC_OP_MUL              = 0xAE, /* MUL   IN1, OUT (int16 * int16 -> int32 OUT)     */
    PLC_OP_DIV              = 0xAF, /* DIV   IN1, OUT (int32 / int16 -> rem16:quot16)  */

    /* 0xB0..0xBF: PID, Inc/Dec/Invert Word/DWord, DWord AND Compare, Word/DWord Bitwise */
    PLC_OP_PID              = 0xB0, /* PID   TBL, LOOP (PID Loop Controller)           */
    PLC_OP_INC_W            = 0xB1, /* INCW  OUT (OUT = OUT + 1, 16-bit)               */
    PLC_OP_INC_D            = 0xB2, /* INCD  OUT (OUT = OUT + 1, 32-bit)               */
    PLC_OP_DEC_W            = 0xB3, /* DECW  OUT (OUT = OUT - 1, 16-bit)               */
    PLC_OP_DEC_D            = 0xB4, /* DECD  OUT (OUT = OUT - 1, 32-bit)               */
    PLC_OP_AD_EQ            = 0xB5, /* AD=   IN1, IN2                                  */
    PLC_OP_AD_LE            = 0xB6, /* AD<=  IN1, IN2                                  */
    PLC_OP_AD_GE            = 0xB7, /* AD>=  IN1, IN2                                  */
    PLC_OP_AND_W            = 0xB8, /* ANDW  IN1, OUT                                  */
    PLC_OP_AND_D            = 0xB9, /* ANDD  IN1, OUT                                  */
    PLC_OP_OR_W             = 0xBA, /* ORW   IN1, OUT                                  */
    PLC_OP_OR_D             = 0xBB, /* ORD   IN1, OUT                                  */
    PLC_OP_XOR_W            = 0xBC, /* XORW  IN1, OUT                                  */
    PLC_OP_XOR_D            = 0xBD, /* XORD  IN1, OUT                                  */
    PLC_OP_INV_W            = 0xBE, /* INVW  OUT (OUT = ~OUT, 16-bit)                  */
    PLC_OP_INV_D            = 0xBF, /* INVD  OUT (OUT = ~OUT, 32-bit)                  */

    /* 0xC0..0xCF: DWord OR Compare, PLS, BCD, SCR Control, Interrupt Attach/Detach */
    PLC_OP_OD_EQ            = 0xC0, /* OD=   IN1, IN2                                  */
    PLC_OP_OD_LE            = 0xC1, /* OD<=  IN1, IN2                                  */
    PLC_OP_OD_GE            = 0xC2, /* OD>=  IN1, IN2                                  */
    PLC_OP_PLS              = 0xC3, /* PLS   Q0.0 / Q0.1 (PTO / PWM Pulse Output)      */
    PLC_OP_SCRE             = 0xC5, /* SCRE  (SCR End, 1 byte)                         */
    PLC_OP_BTI              = 0xC8, /* BTI   OUT (BCD to Integer 16-bit)               */
    PLC_OP_ITB              = 0xC9, /* ITB   OUT (Integer to BCD 16-bit)               */
    PLC_OP_SCR              = 0xCA, /* LSCR  Sx.y (Load SCR state, 2 bytes)            */
    PLC_OP_SCRT             = 0xCB, /* SCRT  Sx.y (SCR Transition, 2 bytes)            */
    PLC_OP_CSCRE            = 0xCC, /* CSCRE (Conditional SCR End, 1 byte)             */
    PLC_OP_ATCH             = 0xCE, /* ATCH  INT_n, EVNT (Attach Interrupt, 4 bytes)   */
    PLC_OP_DTCH             = 0xCF, /* DTCH  EVNT (Detach Interrupt, 3 bytes)          */

    /* 0xD0..0xDF: Block Move, Jump/Label, Subroutine Call, Move, RTC Read, Logic Stack */
    PLC_OP_BMD              = 0xD0, /* BMD   IN, OUT, N (Block Move DWord)             */
    PLC_OP_BMB              = 0xD1, /* BMB   IN, OUT, N (Block Move Byte)              */
    PLC_OP_BMW              = 0xD2, /* BMW   IN, OUT, N (Block Move Word)              */
    PLC_OP_JMP              = 0xD3, /* JMP   n (Jump to Label n, 3 bytes)              */
    PLC_OP_LBL              = 0xD4, /* LBL   n (Label Definition n, 3 bytes)           */
    PLC_OP_CALL_NO_ARGS     = 0xD5, /* CALL  SBR_n (Subroutine Call without params)    */
    PLC_OP_FILL             = 0xD7, /* FILL  IN, OUT, N (Fill N Words with IN)         */
    PLC_OP_MOVB             = 0xD8, /* MOVB  IN, OUT (Move Byte)                       */
    PLC_OP_MOVW             = 0xD9, /* MOVW  IN, OUT (Move Word)                       */
    PLC_OP_MOVD             = 0xDA, /* MOVD  IN, OUT (Move DWord)                      */
    PLC_OP_READ_RTC         = 0xDB, /* TODR  T (Read Real-Time Clock to 8-byte buffer) */
    PLC_OP_ALD              = 0xDC, /* ALD   (AND Load logic stack, 1 byte)            */
    PLC_OP_OLD              = 0xDD, /* OLD   (OR Load logic stack, 1 byte)             */
    PLC_OP_LPS              = 0xDE, /* LPS   (Logic Push, 1 byte)                      */
    PLC_OP_LRD              = 0xDF, /* LRD   (Logic Read, 1 byte)                      */

    /* 0xE0..0xEF: Logic Pop, Edge Detect, NOT, Return/End, Interrupts, NEXT, RTC Write, SWAP */
    PLC_OP_LPP              = 0xE0, /* LPP   (Logic Pop, 1 byte)                       */
    PLC_OP_EU               = 0xE1, /* EU    (Edge Up / Rising Edge, 1 byte)           */
    PLC_OP_ED               = 0xE2, /* ED    (Edge Down / Falling Edge, 1 byte)        */
    PLC_OP_NOT              = 0xE3, /* NOT   (Invert top of logic stack, 1 byte)       */
    PLC_OP_END_COND         = 0xE4, /* END   (Conditional End of OB1 scan, 1 byte)     */
    PLC_OP_RETI             = 0xE5, /* RETI  (Return from Interrupt, 0xE5 0xB5)        */
    PLC_OP_ENI              = 0xE6, /* ENI   (Enable Interrupts globally, 1 byte)      */
    PLC_OP_DISI             = 0xE7, /* DISI  (Disable Interrupts globally, 1 byte)     */
    PLC_OP_NEXT             = 0xE8, /* NEXT  (FOR/NEXT loop end, 1 byte)               */
    PLC_OP_RET              = 0xE9, /* RET   (Return from Subroutine, 0xE9 0xCD)       */
    PLC_OP_CRET             = 0xEA, /* CRET  (Conditional Return from SBR, 1 byte)     */
    PLC_OP_SET_RTC          = 0xEB, /* TODW  T (Write Real-Time Clock from 8B buffer)  */
    PLC_OP_STOP             = 0xEC, /* STOP  (Transition PLC to STOP mode, 1 byte)     */
    PLC_OP_MEND             = 0xED, /* MEND  (Main Program End, 0xED 0xD6)             */
    PLC_OP_CRETI            = 0xEE, /* CRETI (Conditional Return from Interrupt, 1B)   */
    PLC_OP_SWAP             = 0xEF, /* SWAP  IN_OUT (Byte-swap 16-bit Word)            */

    /* 0xF0..0xFE: NOP, Multi-Bit Set/Reset, Byte Inc/Dec/Inv, AENO, Table Ops, 0xFE Prefix */
    PLC_OP_NOP              = 0xF0, /* NOP   N (No Operation, 2 bytes)                 */
    PLC_OP_SET_BITS         = 0xF1, /* S     S_BIT, N (Set N bits)                     */
    PLC_OP_RESET_BITS       = 0xF2, /* R     S_BIT, N (Reset N bits / Timer / Counter) */
    PLC_OP_XOR_BITS         = 0xF3, /* XORB_BIT S_BIT, N (Toggle N bits)               */
    PLC_OP_INV_B            = 0xF4, /* INVB  OUT (OUT = ~OUT, 8-bit)                   */
    PLC_OP_INC_B            = 0xF5, /* INCB  OUT (OUT = OUT + 1, 8-bit)                */
    PLC_OP_DEC_B            = 0xF6, /* DECB  OUT (OUT = OUT - 1, 8-bit)                */
    PLC_OP_AENO             = 0xF7, /* AENO  (AND ENO bit with top of stack, 1 byte)   */
    PLC_OP_ATT              = 0xFA, /* ATT   DATA, TBL (Add Word to Table)             */
    PLC_OP_LIFO             = 0xFB, /* LIFO  TBL, DATA (Last-In-First-Out Table Pop)   */
    PLC_OP_FIFO             = 0xFC, /* FIFO  TBL, DATA (First-In-First-Out Table Pop)  */
    PLC_OP_EXT_FE           = 0xFE  /* 0xFE Extended Instruction Prefix (0x07..0x94)   */
} plc_opcode_t;

/* 0xFE Extended Sub-Opcodes (Dispatched via 142-case jump table at 0x0804AEC8) */
typedef enum {
    PLC_EXT_SHRB            = 0x07, /* SHRB   DATA, S_BIT, N (Shift Register Bit)      */
    PLC_EXT_RSVD_08         = 0x08, /* Unsupported -> sets 0x8122                      */
    PLC_EXT_DECO            = 0x09, /* DECO   IN, OUT (Decode 4-bit to 16-bit)         */
    PLC_EXT_ENCO            = 0x0A, /* ENCO   IN, OUT (Encode 16-bit LSB to 4-bit)     */
    PLC_EXT_ATH             = 0x0B, /* ATH    IN, OUT, LEN (ASCII to Hex)              */
    PLC_EXT_HTA             = 0x0C, /* HTA    IN, OUT, LEN (Hex to ASCII)              */
    PLC_EXT_XMT             = 0x0D, /* XMT    TBL, PORT (Freeport Transmit)            */
    PLC_EXT_HDEF            = 0x0E, /* HDEF   HSC, MODE (High-Speed Counter Define)    */
    PLC_EXT_TOF             = 0x0F, /* TOF    Txxx, PT (Off-Delay Timer)               */
    PLC_EXT_CTD             = 0x10, /* CTD    Cxxx, PV (Count Down Counter)            */
    PLC_EXT_MUL_D           = 0x11, /* *D     IN1, OUT (32-bit Signed Multiply)        */
    PLC_EXT_DIV_D           = 0x12, /* /D     IN1, OUT (32-bit Signed Divide)          */
    PLC_EXT_MUL_I           = 0x13, /* *I     IN1, OUT (16-bit Signed Multiply)        */
    PLC_EXT_DIV_I           = 0x14, /* /I     IN1, OUT (16-bit Signed Divide)          */
    PLC_EXT_ITA             = 0x15, /* ITA    IN, OUT, FMT (Int16 to ASCII)            */
    PLC_EXT_FOR             = 0x16, /* FOR    INDX, INIT, FINAL (Loop Start)           */
    PLC_EXT_DTA             = 0x17, /* DTA    IN, OUT, FMT (DInt32 to ASCII)           */
    PLC_EXT_RTA             = 0x18, /* RTA    IN, OUT, FMT (Real to ASCII)             */
    PLC_EXT_FND             = 0x19, /* FND    TBL, PATRN, INDX (Table Find)            */
    PLC_EXT_SFND            = 0x1A, /* S_FIND IN1, IN2, OUT (String Find)              */
    PLC_EXT_CFND            = 0x1B, /* C_FIND IN1, IN2, OUT (Char Find in String)      */
    PLC_EXT_SFND_1C         = 0x1C, /* String Find variant 0x1C                        */
    PLC_EXT_SFND_1D         = 0x1D, /* String Find variant 0x1D                        */
    PLC_EXT_RSVD_1E         = 0x1E, /* Unsupported -> sets 0x8122                      */
    PLC_EXT_SI_1F           = 0x1F, /* SI     Q_BIT, N (Set Immediate Output Bits)     */
    PLC_EXT_RI_20           = 0x20, /* RI     Q_BIT, N (Reset Immediate Output Bits)   */
    PLC_EXT_SI_21           = 0x21, /* SI     Extended Immediate Set                   */
    PLC_EXT_RI_22           = 0x22, /* RI     Extended Immediate Reset                 */
    PLC_EXT_CALL_ARGS       = 0x23, /* CALL   SBR_n, params... (Call with parameters)  */
    PLC_EXT_RSVD_24         = 0x24, /* Unsupported -> sets 0x8122                      */
    PLC_EXT_SLB             = 0x25, /* SLB    OUT, N (Shift Left Byte)                 */
    PLC_EXT_SRB             = 0x26, /* SRB    OUT, N (Shift Right Byte)                */
    PLC_EXT_RLB             = 0x27, /* RLB    OUT, N (Rotate Left Byte)                */
    PLC_EXT_RRB             = 0x28, /* RRB    OUT, N (Rotate Right Byte)               */
    PLC_EXT_SLW             = 0x29, /* SLW    OUT, N (Shift Left Word)                 */
    PLC_EXT_SRW             = 0x2A, /* SRW    OUT, N (Shift Right Word)                */
    PLC_EXT_SLD             = 0x2B, /* SLD    OUT, N (Shift Left DWord)                */
    PLC_EXT_SRD             = 0x2C, /* SRD    OUT, N (Shift Right DWord)               */
    PLC_EXT_RLW             = 0x2D, /* RLW    OUT, N (Rotate Left Word)                */
    PLC_EXT_RRW             = 0x2E, /* RRW    OUT, N (Rotate Right Word)               */
    PLC_EXT_RLD             = 0x2F, /* RLD    OUT, N (Rotate Left DWord)               */
    PLC_EXT_RRD             = 0x30, /* RRD    OUT, N (Rotate Right DWord)              */
    PLC_EXT_SEG             = 0x31, /* SEG    IN, OUT (7-Segment Display Decode)       */
    PLC_EXT_ADD_R           = 0x32, /* +R     IN1, OUT (Real IEEE-754 Add)             */
    PLC_EXT_SUB_R           = 0x33, /* -R     IN1, OUT (Real IEEE-754 Subtract)        */
    PLC_EXT_MUL_R           = 0x34, /* *R     IN1, OUT (Real IEEE-754 Multiply)        */
    PLC_EXT_DIV_R           = 0x35, /* /R     IN1, OUT (Real IEEE-754 Divide)          */
    PLC_EXT_ROUND           = 0x36, /* ROUND  IN, OUT (Real to DInt32 Round)           */
    PLC_EXT_TRUNC           = 0x37, /* TRUNC  IN, OUT (Real to DInt32 Truncate)        */
    PLC_EXT_MOVR            = 0x38, /* MOVR   IN, OUT (Move 32-bit Real)               */
    PLC_EXT_LDR_EQ          = 0x39, /* LDR=   IN1, IN2                                 */
    PLC_EXT_LDR_LE          = 0x3A, /* LDR<=  IN1, IN2                                 */
    PLC_EXT_LDR_GE          = 0x3B, /* LDR>=  IN1, IN2                                 */
    PLC_EXT_AR_EQ           = 0x3C, /* AR=    IN1, IN2                                 */
    PLC_EXT_AR_LE           = 0x3D, /* AR<=   IN1, IN2                                 */
    PLC_EXT_AR_GE           = 0x3E, /* AR>=   IN1, IN2                                 */
    PLC_EXT_OR_EQ           = 0x3F, /* OR=    IN1, IN2                                 */
    PLC_EXT_OR_LE           = 0x40, /* OR<=   IN1, IN2                                 */
    PLC_EXT_OR_GE           = 0x41, /* OR>=   IN1, IN2                                 */
    PLC_EXT_SQRT            = 0x42, /* SQRT   IN, OUT (Real Square Root)               */
    PLC_EXT_DTR             = 0x43, /* DTR    IN, OUT (DInt32 to Real)                 */
    PLC_EXT_BIR             = 0x44, /* BIR    IN, OUT (Move Immediate Read Input Byte) */
    PLC_EXT_BIW             = 0x45, /* BIW    IN, OUT (Move Immediate Write Out Byte)  */
    PLC_EXT_LDB_NE          = 0x46, /* LDB<>  IN1, IN2                                 */
    PLC_EXT_AB_NE           = 0x47, /* AB<>   IN1, IN2                                 */
    PLC_EXT_OB_NE           = 0x48, /* OB<>   IN1, IN2                                 */
    PLC_EXT_LDB_LT          = 0x49, /* LDB<   IN1, IN2                                 */
    PLC_EXT_LDB_GT          = 0x4A, /* LDB>   IN1, IN2                                 */
    PLC_EXT_AB_LT           = 0x4B, /* AB<    IN1, IN2                                 */
    PLC_EXT_AB_GT           = 0x4C, /* AB>    IN1, IN2                                 */
    PLC_EXT_OB_LT           = 0x4D, /* OB<    IN1, IN2                                 */
    PLC_EXT_OB_GT           = 0x4E, /* OB>    IN1, IN2                                 */
    PLC_EXT_LDW_NE          = 0x4F, /* LDW<>  IN1, IN2                                 */
    PLC_EXT_AW_NE           = 0x50, /* AW<>   IN1, IN2                                 */
    PLC_EXT_OW_NE           = 0x51, /* OW<>   IN1, IN2                                 */
    PLC_EXT_LDW_LT          = 0x52, /* LDW<   IN1, IN2                                 */
    PLC_EXT_LDW_GT          = 0x53, /* LDW>   IN1, IN2                                 */
    PLC_EXT_AW_LT           = 0x54, /* AW<    IN1, IN2                                 */
    PLC_EXT_AW_GT           = 0x55, /* AW>    IN1, IN2                                 */
    PLC_EXT_OW_LT           = 0x56, /* OW<    IN1, IN2                                 */
    PLC_EXT_OW_GT           = 0x57, /* OW>    IN1, IN2                                 */
    PLC_EXT_LDD_NE          = 0x58, /* LDD<>  IN1, IN2                                 */
    PLC_EXT_AD_NE           = 0x59, /* AD<>   IN1, IN2                                 */
    PLC_EXT_OD_NE           = 0x5A, /* OD<>   IN1, IN2                                 */
    PLC_EXT_LDD_LT          = 0x5B, /* LDD<   IN1, IN2                                 */
    PLC_EXT_LDD_GT          = 0x5C, /* LDD>   IN1, IN2                                 */
    PLC_EXT_AD_LT           = 0x5D, /* AD<    IN1, IN2                                 */
    PLC_EXT_AD_GT           = 0x5E, /* AD>    IN1, IN2                                 */
    PLC_EXT_OD_LT           = 0x5F, /* OD<    IN1, IN2                                 */
    PLC_EXT_OD_GT           = 0x60, /* OD>    IN1, IN2                                 */
    PLC_EXT_LDR_NE          = 0x61, /* LDR<>  IN1, IN2                                 */
    PLC_EXT_AR_NE           = 0x62, /* AR<>   IN1, IN2                                 */
    PLC_EXT_OR_NE           = 0x63, /* OR<>   IN1, IN2                                 */
    PLC_EXT_LDR_LT          = 0x64, /* LDR<   IN1, IN2                                 */
    PLC_EXT_LDR_GT          = 0x65, /* LDR>   IN1, IN2                                 */
    PLC_EXT_AR_LT           = 0x66, /* AR<    IN1, IN2                                 */
    PLC_EXT_AR_GT           = 0x67, /* AR>    IN1, IN2                                 */
    PLC_EXT_OR_LT           = 0x68, /* OR<    IN1, IN2                                 */
    PLC_EXT_OR_GT           = 0x69, /* OR>    IN1, IN2                                 */
    PLC_EXT_B_TO_I          = 0x6A, /* BTI    IN_BYTE, OUT_WORD (Byte to Int16)        */
    PLC_EXT_I_TO_B          = 0x6B, /* ITB    IN_WORD, OUT_BYTE (Int16 to Byte)        */
    PLC_EXT_ITD             = 0x6C, /* ITD    IN_WORD, OUT_DWORD (Int16 to DInt32)     */
    PLC_EXT_DTI             = 0x6D, /* DTI    IN_DWORD, OUT_WORD (DInt32 to Int16)     */
    PLC_EXT_STI_6E          = 0x6E, /* STI    IN_STR, INDX, OUT_WORD (String to Int)   */
    PLC_EXT_STD_6F          = 0x6F, /* STD    IN_STR, INDX, OUT_DWORD (String to DInt) */
    PLC_EXT_STR_70          = 0x70, /* STR    IN_STR, INDX, OUT_REAL (String to Real)  */
    PLC_EXT_BITWISE_B       = 0x71, /* ANDB / ORB / XORB (Byte Bitwise Logic)          */
    PLC_EXT_ADDSUB_B        = 0x72, /* +B / -B (Byte Addition / Subtraction)           */
    PLC_EXT_SIN             = 0x73, /* SIN    IN, OUT (Real Sine)                      */
    PLC_EXT_COS             = 0x74, /* COS    IN, OUT (Real Cosine)                    */
    PLC_EXT_TAN             = 0x75, /* TAN    IN, OUT (Real Tangent)                   */
    PLC_EXT_LN              = 0x76, /* LN     IN, OUT (Real Natural Logarithm)         */
    PLC_EXT_EXP             = 0x77, /* EXP    IN, OUT (Real Exponential e^x)           */
    PLC_EXT_NETR            = 0x78, /* NETR   TBL, PORT (Network Read)                 */
    PLC_EXT_NETW            = 0x79, /* NETW   TBL, PORT (Network Write)                */
    PLC_EXT_STR_7A          = 0x7A, /* STR    String to Real variant 0x7A              */
    PLC_EXT_STD_7B          = 0x7B, /* STD    String to DInt variant 0x7B              */
    PLC_EXT_STI_7C          = 0x7C, /* STI    String to Int variant 0x7C               */
    PLC_EXT_SCPY            = 0x7D, /* SCPY   IN_STR, OUT_STR (String Copy)            */
    PLC_EXT_SCAT            = 0x7E, /* SCAT   IN_STR, OUT_STR (String Concatenate)     */
    PLC_EXT_SLEN            = 0x7F, /* SLEN   IN_STR, OUT_BYTE (String Length)         */
    PLC_EXT_LDS_EQ          = 0x80, /* LDS=   STR1, STR2                               */
    PLC_EXT_AS_EQ           = 0x81, /* AS=    STR1, STR2                               */
    PLC_EXT_OS_EQ           = 0x82, /* OS=    STR1, STR2                               */
    PLC_EXT_LDS_NE          = 0x83, /* LDS<>  STR1, STR2                               */
    PLC_EXT_AS_NE           = 0x84, /* AS<>   STR1, STR2                               */
    PLC_EXT_OS_NE           = 0x85, /* OS<>   STR1, STR2                               */
    PLC_EXT_ITS             = 0x86, /* ITS    IN, FMT, OUT_STR (Int16 to String)       */
    PLC_EXT_DTS             = 0x87, /* DTS    IN, FMT, OUT_STR (DInt32 to String)      */
    PLC_EXT_RTS             = 0x88, /* RTS    IN, FMT, OUT_STR (Real to String)        */
    PLC_EXT_SSCPY           = 0x89, /* SSCPY  IN_STR, INDX, N, OUT_STR (Substr Copy)   */
    PLC_EXT_DIAG_LED        = 0x8A, /* DIAG_LED / SF LED Control                       */
    PLC_EXT_BITIM           = 0x8B, /* BITIM  OUT (Begin Interval Time -> ms tick)     */
    PLC_EXT_CITIM           = 0x8C, /* CITIM  IN, OUT (Calculate Interval Time)        */
    PLC_EXT_RSVD_8D         = 0x8D, /* Unsupported -> sets 0x8122                      */
    PLC_EXT_HSC             = 0x8E, /* HSC    N (High-Speed Counter Load/Control)      */
    PLC_EXT_RSVD_8F         = 0x8F, /* Unsupported -> sets 0x8122                      */
    PLC_EXT_RSVD_90         = 0x90, /* Unsupported -> sets 0x8122                      */
    PLC_EXT_RSVD_91         = 0x91, /* Unsupported -> sets 0x8122                      */
    PLC_EXT_GPA             = 0x92, /* GPA    ADDR, PORT (Get Port Station Address)    */
    PLC_EXT_SPA             = 0x93, /* SPA    ADDR, PORT (Set Port Station Address)    */
    PLC_EXT_RCV             = 0x94  /* RCV    TBL, PORT (Freeport Receive)             */
} plc_ext_fe_opcode_t;

/* ============================================================================
 * 5. VM Execution Context & Parser Structures
 * ============================================================================
 */

/**
 * @brief Subroutine / Interrupt Call Parameter Descriptor
 *
 * Marshaled into the callee's 64-byte LB0..LB63 frame by plc_call_marshal_in_params
 * (0x0800133c) and unmarshaled back to the caller by plc_call_unmarshal_out_params
 * (0x080027ea).
 */
typedef enum {
    PLC_PARAM_DIR_IN        = 0x10, /* Input-only parameter (copied into LBx)          */
    PLC_PARAM_DIR_IN_OUT    = 0x20, /* In/Out parameter (copied in and back out)       */
    PLC_PARAM_DIR_OUT       = 0x30  /* Output-only parameter (copied out on return)    */
} plc_call_param_dir_t;

/**
 * @brief VM Execution Frame / State passed to plc_vm_exec_block (FUN_08021948)
 */
typedef struct {
    uint16_t    pc_offset;          /* Current bytecode offset in OB1 (0x0806D800)     */
    uint16_t    logic_stack;        /* 16-bit boolean RLO logic stack (bit 0 = TOS)    */
    uint8_t     eno_flags;          /* Bit 2 (0x04) = current instruction ENO status   */
    uint8_t     local_mem[64];      /* 64-byte local memory frame LB0..LB63            */
    uint16_t    edge_idx;           /* Current EU/ED edge detector counter (1..256)    */
    uint16_t    call_depth;         /* Current subroutine nesting depth (0..9)         */
    uint8_t     block_type;         /* 0 = OB1/SBR, 1 = Interrupt routine (INT_n)      */
} plc_vm_frame_t;

/* ============================================================================
 * 6. Function Prototypes
 * ============================================================================
 */

/* --- User Program Parser & Validator (src/plc_prog_parser.c) --- */

/**
 * @brief Compute 16-bit CRC (CCITT polynomial 0x1021) over a buffer.
 *        Reconstructed from FUN_08000e0e (0x08000e0e .. 0x08000e49).
 */
uint16_t plc_crc16_calc(uint8_t *buf, uint16_t len);

/**
 * @brief Translate a 32-bit S7-200 virtual pointer ((area << 24) | offset)
 *        into a physical SRAM/Flash byte pointer. Sets error 0x8091 on bounds
 *        violation.
 *        Reconstructed from FUN_08000e4a (0x08000e4a .. 0x08000f55).
 */
uint8_t *plc_vptr_to_phys(uint32_t vptr);

/**
 * @brief Translate a 32-bit S7-200 virtual pointer, including Local Memory
 *        (area 0x0E) relative to the active LB0..LB63 frame.
 *        Reconstructed from FUN_08000f56 (0x08000f56 .. 0x0800106f).
 */
uint8_t *plc_vptr_to_phys_local(uint32_t vptr, uint8_t *local_base);

/**
 * @brief Scan and validate the user program in OB1 Flash (0x0806D800), locate
 *        MEND (0xED 0xD6), RET (0xE9 0xCD), and RETI (0xE5 0xB5), build the
 *        SBR_n (DAT_20006020) and INT_n (DAT_20005EFC) entry offset tables,
 *        and pre-validate FOR/NEXT nesting.
 *        Reconstructed from FUN_08001070 (0x08001070 .. 0x0800133b).
 */
void plc_prog_scan_and_validate(void);

/**
 * @brief Marshal IN and IN_OUT parameters from the CALL instruction stream at
 *        caller_pc into the callee's 64-byte local memory frame (callee_local),
 *        returning the updated caller_pc past the parameter block.
 *        Reconstructed from FUN_0800133c (0x0800133c .. 0x080027e9).
 */
uint16_t plc_call_marshal_in_params(uint16_t caller_pc,
                                    uint8_t *caller_local,
                                    uint8_t *callee_local,
                                    uint16_t *logic_stack);

/**
 * @brief Unmarshal IN_OUT and OUT parameters from the callee's 64-byte local
 *        memory frame (callee_local) back into the caller's operands after
 *        subroutine execution, returning the updated caller_pc.
 *        Reconstructed from FUN_080027ea (0x080027ea .. 0x08003c21).
 */
uint16_t plc_call_unmarshal_out_params(uint16_t caller_pc,
                                       uint8_t *caller_local,
                                       uint8_t *callee_local);

/**
 * @brief Skip over the encoded parameter descriptors of a CALL with parameters
 *        (0xFE 0x23) when EN (top of logic stack) is 0.
 *        Reconstructed from FUN_08003c22 (0x08003c22 .. 0x08003e5b).
 */
uint16_t plc_call_skip_params(uint16_t caller_pc);

/* --- MC7/STL Bytecode Interpreter (src/plc_vm_interpreter.c) --- */

/**
 * @brief Resolve a direct memory operand at ob1[pc] into a physical byte pointer.
 *        Reconstructed from FUN_0801452e (0x0801452e .. 0x08014679).
 */
uint8_t *plc_oper_resolve_direct_ptr(uint16_t pc,
                                     uint8_t *local_mem,
                                     uint8_t *out_len);

/**
 * @brief Decode and read an 8-bit operand according to its 4-bit addressing mode.
 *        Reconstructed from FUN_0801467a (0x0801467a .. 0x08014ac1).
 */
uint8_t plc_oper_read_u8(uint8_t addr_mode,
                         uint16_t pc,
                         uint8_t *local_mem,
                         uint8_t *out_len,
                         uint8_t *trace_buf);

/**
 * @brief Decode and read a 16-bit operand according to its 4-bit addressing mode.
 *        Reconstructed from FUN_08014e6e (0x08014e6e .. 0x08015ca5).
 */
int16_t plc_oper_read_i16(uint8_t addr_mode,
                          uint16_t pc,
                          uint8_t *local_mem,
                          uint8_t *out_len,
                          uint8_t *trace_buf);

/**
 * @brief Decode and read a 32-bit operand according to its 4-bit addressing mode.
 *        Reconstructed from FUN_08014ac2 (0x08014ac2 .. 0x08014e6d).
 */
uint32_t plc_oper_read_u32(uint8_t addr_mode,
                           uint16_t pc,
                           uint8_t *local_mem,
                           uint8_t *out_len,
                           uint8_t *trace_buf);

/**
 * @brief Write an 8-bit result to an operand according to its 4-bit addressing mode.
 *        Reconstructed from FUN_0803b424 (0x0803b424 .. 0x0803b711).
 */
void plc_oper_write_u8(uint8_t addr_mode,
                       uint16_t pc,
                       uint8_t *local_mem,
                       uint8_t *out_len,
                       uint8_t *trace_buf,
                       uint8_t val);

/**
 * @brief Write a 16-bit result to an operand according to its 4-bit addressing mode.
 *        Reconstructed from FUN_0803bbd2 (0x0803bbd2 .. 0x0803bf40).
 */
void plc_oper_write_u16(uint8_t addr_mode,
                        uint16_t pc,
                        uint8_t *local_mem,
                        uint8_t *out_len,
                        uint8_t *trace_buf,
                        uint16_t val);

/**
 * @brief Write a 32-bit result to an operand according to its 4-bit addressing mode.
 *        Reconstructed from FUN_0803b712 (0x0803b712 .. 0x0803bbd1).
 */
void plc_oper_write_u32(uint8_t addr_mode,
                        uint16_t pc,
                        uint8_t *local_mem,
                        uint8_t *out_len,
                        uint8_t *trace_buf,
                        uint32_t val);

/**
 * @brief Execute an S7-200 MC7/STL program block (OB1, SBR_n, or INT_n) starting
 *        at `*pc_ptr` until `MEND` (0xED), `RET` (0xE9), `RETI` (0xE5), `END`,
 *        or `STOP` is encountered.
 *        Reconstructed from FUN_08021948 (0x08021948 .. 0x08036b1f).
 *
 * @param pc_ptr        In/Out pointer to current bytecode offset in OB1 (0x0806D800)
 * @param logic_stack   In/Out pointer to 16-bit RLO logic stack
 * @param eno_flags     In/Out pointer to instruction ENO status byte (bit 2 = ENO)
 * @param local_mem     Pointer to active 64-byte local memory frame (LB0..LB63)
 * @param edge_idx_ptr  In/Out pointer to EU/ED edge detector counter
 * @param call_depth    In/Out pointer to subroutine recursion depth
 */
void plc_vm_exec_block(uint16_t *pc_ptr,
                       uint16_t *logic_stack,
                       uint8_t *eno_flags,
                       uint8_t *local_mem,
                       uint16_t *edge_idx_ptr,
                       uint16_t *call_depth);

#ifdef __cplusplus
}
#endif

#endif /* PLC_VM_H */
