/**
 * @file plc_vm_interpreter.c
 * @brief SIMATIC S7-226 (APM32E103) MC7/STL Virtual Machine Bytecode Interpreter
 *        and Operand Address Decoders/Writers.
 *
 * Reconstructed from firmware flash regions:
 *   - Operand Address Decoders & Writers:
 *       * FUN_0801452e (0x0801452e .. 0x08014679): plc_oper_resolve_direct_ptr
 *       * FUN_0801467a (0x0801467a .. 0x08014ac1): plc_oper_read_u8
 *       * FUN_08014ac2 (0x08014ac2 .. 0x08014e6d): plc_oper_read_u32
 *       * FUN_08014e6e (0x08014e6e .. 0x08015ca5): plc_oper_read_i16
 *       * FUN_0803b424 (0x0803b424 .. 0x0803b711): plc_oper_write_u8
 *       * FUN_0803b712 (0x0803b712 .. 0x0803bbd1): plc_oper_write_u32
 *       * FUN_0803bbd2 (0x0803bbd2 .. 0x0803bf40): plc_oper_write_u16
 *   - Contact Comparison Helpers:
 *       * FUN_08015ca6 (0x08015ca6 .. 0x08015e57): plc_vm_cmp_byte
 *       * FUN_08016252 (0x08016252 .. 0x08016550): plc_vm_cmp_word
 *       * FUN_08015e58 (0x08015e58 .. 0x08016251): plc_vm_cmp_dword_or_real
 *   - Main MC7/STL Bytecode Interpreter:
 *       * FUN_08021948 (0x08021948 .. 0x08036b1f): plc_vm_exec_block
 *         - Loop entry/condition at 0x08036b02 & interrupt queue dispatch at 0x08036958
 *         - High-nibble dispatch (0x08021a62, table 0x0804ad12):
 *             * 0x0..0x7 (0x08021a66..0x08021f8b): Compact 2-byte bit logic (LD/LDN/A/AN/O/ON/=)
 *             * 0x8      (0x08021f8c..0x08022501): Extended 3B/5B bit logic & Immediate I/O
 *             * 0x9..0xA (0x08022502..0x08023961): Byte/Word/DWord Compare, WDR, TON/TONR, CTU/CTUD, +I/-I/+D/-D/MUL/DIV
 *             * 0xB      (0x08023962..0x08024791): PID, INCW/INCD/DECW/DECD/INVW/INVD, AD=/<=/=>, AND/OR/XOR Word/DWord
 *             * 0xC      (0x08024792..0x0802688d): OD=/<=/=>, PLS, BTI/ITB, SCR/SCRT/SCRE/CSCRE, ATCH/DTCH
 *             * 0xD      (0x0802688e..0x08027d81): BMD/BMB/BMW, JMP/LBL, CALL, FILL, MOVB/MOVW/MOVD, TODR, ALD/OLD/LPS/LRD
 *             * 0xE      (0x08027d82..0x080284eb): LPP, EU/ED, NOT, END/RET/CRET/RETI/CRETI, ENI/DISI, NEXT, TODW, STOP, SWAP
 *             * 0xF      (0x080284ec..0x08036957): NOP, S/R, INVB/INCB/DECB, AENO, ATT/LIFO/FIFO, and
 *                        0xFE 142-case extended instruction table (0x08028508, table 0x0804aec8)
 */

#include "plc_vm.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * 1. Firmware Global Variables & Hardware Helpers Referenced by Interpreter
 * ============================================================================
 */

/* Configurable area base pointers in SRAM (0x20000250..0x20000278) */
#define PLC_PTR_BASE_V              (*(uint8_t * volatile *)0x20000250U)
#define PLC_PTR_BASE_M              (*(uint8_t * volatile *)0x20000254U)
#define PLC_PTR_BASE_S              (*(uint8_t * volatile *)0x20000258U)
#define PLC_PTR_BASE_Q              (*(uint8_t * volatile *)0x2000025CU)
#define PLC_PTR_BASE_I              (*(uint8_t * volatile *)0x20000260U)
#define PLC_PTR_BASE_AI             (*(uint8_t * volatile *)0x20000264U)
#define PLC_PTR_BASE_AQ             (*(uint8_t * volatile *)0x20000268U)
#define PLC_PTR_BASE_SM             (*(uint8_t * volatile *)0x2000026CU)
#define PLC_PTR_BASE_T              (*(uint8_t * volatile *)0x20000270U)
#define PLC_PTR_BASE_C              (*(uint8_t * volatile *)0x20000274U)
#define PLC_PTR_BASE_OB1            (*(uint8_t * volatile *)0x20000278U)

/* System Timers, Counters, and Status Words in SRAM */
#define PLC_RUN_MODE_FLAG           (*(volatile uint8_t  *)0x20000171U)
#define PLC_STOP_REQ_FLAG           (*(volatile uint8_t  *)0x20000172U)
#define PLC_FATAL_ERR_REG           (*(volatile uint16_t *)0x20000188U)
#define PLC_COMPILE_ERR_REG         (*(volatile uint16_t *)0x200001ECU)
#define PLC_SYS_MS_TICK             (*(volatile uint32_t *)0x20000220U)
#define PLC_TICK_1MS_COUNTER        (*(volatile uint16_t *)0x200001BAU)
#define PLC_TICK_10MS_COUNTER       (*(volatile uint16_t *)0x200001BCU)
#define PLC_TICK_100MS_COUNTER      (*(volatile uint16_t *)0x200001BEU)
#define PLC_OPER_LEN_SCRATCH        (*(volatile uint8_t  *)0x20007823U)

/* Subroutine and Interrupt Runtime Tables */
#define PLC_SBR_ENTRY_PC_TBL        ((volatile uint16_t *)0x20006020U)
#define PLC_INT_ENTRY_PC_TBL        ((volatile uint16_t *)0x20005EFCU)
#define PLC_INT_EVENT_MAP_TBL       ((volatile uint8_t  *)0x20000B34U)
#define PLC_INT_QUEUE_COUNT         (*(volatile uint8_t  *)0x2000033EU)

/* 7-Segment Display Lookup Table in Flash (.rodata) */
static const uint8_t k_plc_seg7_table[16] = {
    0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U, 0x6DU, 0x7DU, 0x07U,
    0x7FU, 0x6FU, 0x77U, 0x7CU, 0x39U, 0x5EU, 0x79U, 0x71U
};

/* Helper to read/write big-endian 16-bit and 32-bit values in S7-200 memory */
static inline uint16_t plc_be16_read(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline void plc_be16_write(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFU);
    p[1] = (uint8_t)(v & 0xFFU);
}

static inline uint32_t plc_be32_read(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
            (uint32_t)p[3];
}

static inline void plc_be32_write(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)((v >> 24) & 0xFFU);
    p[1] = (uint8_t)((v >> 16) & 0xFFU);
    p[2] = (uint8_t)((v >> 8)  & 0xFFU);
    p[3] = (uint8_t)(v & 0xFFU);
}

static inline float plc_u32_to_float(uint32_t bits)
{
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static inline uint32_t plc_float_to_u32(float f)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    return bits;
}

/* ============================================================================
 * 2. Universal Operand Address Decoders & Writers
 *    (FUN_0801452e, FUN_0801467a, FUN_08014ac2, FUN_08014e6e,
 *     FUN_0803b424, FUN_0803b712, FUN_0803bbd2)
 * ============================================================================
 */

/**
 * @brief Resolve a direct memory operand at `ob1[pc]` into a physical byte pointer.
 *        Reconstructed from FUN_0801452e (0x0801452e .. 0x08014679).
 */
uint8_t *plc_oper_resolve_direct_ptr(uint16_t pc,
                                     uint8_t *local_mem,
                                     uint8_t *out_len)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint8_t b0 = ob1[pc];
    uint8_t b1 = ob1[pc + 1U];

    /* Extended 4-byte direct address: [0xF0, 0x80(V)|0x40(SM), off_hi, off_lo] */
    if (b0 == 0xF0U) {
        *out_len = (uint8_t)(*out_len + 4U);
        uint16_t off16 = plc_be16_read(&ob1[pc + 2U]);
        if (b1 == 0x80U) {
            return PLC_PHYS_BASE_V + off16;
        }
        if (b1 == 0x40U) {
            return PLC_PHYS_BASE_SM + off16;
        }
        return NULL;
    }

    /* Standard 2-byte direct address: [area_hi, off_lo] */
    *out_len = (uint8_t)(*out_len + 2U);
    uint8_t  area_id = (uint8_t)(b0 >> 4);
    uint16_t offset  = (uint16_t)(((uint16_t)(b0 & 0x0FU) << 8) | (uint16_t)b1);

    switch (area_id) {
    case PLC_AREA_I:  return PLC_PHYS_BASE_I + offset;
    case PLC_AREA_Q:  return PLC_PHYS_BASE_Q + offset;
    case PLC_AREA_M:  return PLC_PHYS_BASE_M + offset;
    case PLC_AREA_S:  return PLC_PHYS_BASE_S + offset;
    case PLC_AREA_SM: return PLC_PHYS_BASE_SM + offset;
    case PLC_AREA_V:  return PLC_PHYS_BASE_V + offset;
    case PLC_AREA_T:  return (uint8_t *)PLC_PHYS_BASE_T_VAL + (offset * 2U);
    case PLC_AREA_C:  return (uint8_t *)PLC_PHYS_BASE_C_VAL + (offset * 2U);
    case PLC_AREA_HC: return (uint8_t *)PLC_PHYS_BASE_HC_VAL + (offset * 4U);
    case PLC_AREA_AI: return (uint8_t *)PLC_PHYS_BASE_AI + offset;
    case PLC_AREA_AQ: return (uint8_t *)PLC_PHYS_BASE_AQ + offset;
    case PLC_AREA_L:  return local_mem + (b1 & 0x3FU);
    default:          return NULL;
    }
}

/**
 * @brief Resolve an indirect pointer operand (`*VDx`, `*LDx`, or `*AC1..*AC3`).
 */
static uint8_t *plc_oper_resolve_indirect_ptr(uint8_t addr_mode,
                                              uint16_t pc,
                                              uint8_t *local_mem,
                                              uint8_t *out_len)
{
    uint32_t vptr = 0U;

    if (addr_mode == PLC_ADDR_MODE_INDIRECT_MEM) {
        uint8_t *ptr_loc = plc_oper_resolve_direct_ptr(pc, local_mem, out_len);
        if (ptr_loc == NULL) {
            PLC_VM_ERROR_REG = PLC_ERR_INDIRECT_ADDR_RANGE;
            return NULL;
        }
        vptr = plc_be32_read(ptr_loc);
    } else if ((addr_mode >= PLC_ADDR_MODE_INDIRECT_AC1) &&
               (addr_mode <= PLC_ADDR_MODE_INDIRECT_AC3)) {
        vptr = PLC_ACCUM_BASE[addr_mode - 8U];
    } else {
        return NULL;
    }

    return plc_vptr_to_phys_local(vptr, local_mem);
}

/**
 * @brief Read an 8-bit operand according to its 4-bit addressing mode.
 *        Reconstructed from FUN_0801467a (0x0801467a .. 0x08014ac1).
 */
uint8_t plc_oper_read_u8(uint8_t addr_mode,
                         uint16_t pc,
                         uint8_t *local_mem,
                         uint8_t *out_len,
                         uint8_t *trace_buf)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    (void)trace_buf;

    switch (addr_mode & 0x0FU) {
    case PLC_ADDR_MODE_DIRECT_MEM: /* 0x0: Direct Memory */
    {
        uint8_t *p = plc_oper_resolve_direct_ptr(pc, local_mem, out_len);
        return (p != NULL) ? *p : 0U;
    }

    case PLC_ADDR_MODE_IMM_BYTE:   /* 0x4: 1-Byte Immediate Constant */
    case PLC_ADDR_MODE_IMM_WORD:
    case PLC_ADDR_MODE_IMM_DWORD:
        *out_len = (uint8_t)(*out_len + 1U);
        return ob1[pc];

    case PLC_ADDR_MODE_INDIRECT_MEM: /* 0x8..0xB: Indirect Pointer (*VDx, *LDx, *ACx) */
    case PLC_ADDR_MODE_INDIRECT_AC1:
    case PLC_ADDR_MODE_INDIRECT_AC2:
    case PLC_ADDR_MODE_INDIRECT_AC3:
    {
        uint8_t *p = plc_oper_resolve_indirect_ptr(addr_mode, pc, local_mem, out_len);
        return (p != NULL) ? *p : 0U;
    }

    case PLC_ADDR_MODE_DIRECT_AC0: /* 0xC..0xF: Accumulator AC0..AC3 (low byte) */
    case PLC_ADDR_MODE_DIRECT_AC1:
    case PLC_ADDR_MODE_DIRECT_AC2:
    case PLC_ADDR_MODE_DIRECT_AC3:
        return (uint8_t)(PLC_ACCUM_BASE[(addr_mode & 0x0FU) - 12U] & 0xFFU);

    default:
        return 0U;
    }
}

/**
 * @brief Read a 16-bit signed/unsigned operand according to its 4-bit addressing mode.
 *        Reconstructed from FUN_08014e6e (0x08014e6e .. 0x08015ca5).
 */
int16_t plc_oper_read_i16(uint8_t addr_mode,
                          uint16_t pc,
                          uint8_t *local_mem,
                          uint8_t *out_len,
                          uint8_t *trace_buf)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    (void)trace_buf;

    switch (addr_mode & 0x0FU) {
    case PLC_ADDR_MODE_DIRECT_MEM: /* 0x0: Direct Memory */
    {
        uint8_t b0 = ob1[pc];
        uint8_t area_id = (uint8_t)(b0 >> 4);
        uint8_t *p = plc_oper_resolve_direct_ptr(pc, local_mem, out_len);
        if (p == NULL) {
            return 0;
        }
        /* Timer, Counter, AI, AQ are stored in native 16-bit little-endian words in SRAM */
        if ((b0 != 0xF0U) &&
            ((area_id == PLC_AREA_T) || (area_id == PLC_AREA_C) ||
             (area_id == PLC_AREA_AI) || (area_id == PLC_AREA_AQ))) {
            return *(int16_t *)p;
        }
        return (int16_t)plc_be16_read(p);
    }

    case PLC_ADDR_MODE_IMM_BYTE:   /* 0x4..0x6: 2-Byte Big-Endian Immediate */
    case PLC_ADDR_MODE_IMM_WORD:
    case PLC_ADDR_MODE_IMM_DWORD:
        *out_len = (uint8_t)(*out_len + 2U);
        return (int16_t)plc_be16_read(&ob1[pc]);

    case PLC_ADDR_MODE_INDIRECT_MEM: /* 0x8..0xB: Indirect Pointer (*VDx, *LDx, *ACx) */
    case PLC_ADDR_MODE_INDIRECT_AC1:
    case PLC_ADDR_MODE_INDIRECT_AC2:
    case PLC_ADDR_MODE_INDIRECT_AC3:
    {
        uint8_t *p = plc_oper_resolve_indirect_ptr(addr_mode, pc, local_mem, out_len);
        return (p != NULL) ? (int16_t)plc_be16_read(p) : 0;
    }

    case PLC_ADDR_MODE_DIRECT_AC0: /* 0xC..0xF: Accumulator AC0..AC3 (low 16 bits) */
    case PLC_ADDR_MODE_DIRECT_AC1:
    case PLC_ADDR_MODE_DIRECT_AC2:
    case PLC_ADDR_MODE_DIRECT_AC3:
        return (int16_t)(PLC_ACCUM_BASE[(addr_mode & 0x0FU) - 12U] & 0xFFFFU);

    default:
        return 0;
    }
}

/**
 * @brief Read a 32-bit DWord / Real / Pointer operand according to its 4-bit mode.
 *        Reconstructed from FUN_08014ac2 (0x08014ac2 .. 0x08014e6d).
 */
uint32_t plc_oper_read_u32(uint8_t addr_mode,
                           uint16_t pc,
                           uint8_t *local_mem,
                           uint8_t *out_len,
                           uint8_t *trace_buf)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    (void)trace_buf;

    switch (addr_mode & 0x0FU) {
    case PLC_ADDR_MODE_DIRECT_MEM: /* 0x0: Direct Memory */
    {
        uint8_t b0 = ob1[pc];
        uint8_t area_id = (uint8_t)(b0 >> 4);
        uint8_t *p = plc_oper_resolve_direct_ptr(pc, local_mem, out_len);
        if (p == NULL) {
            return 0U;
        }
        /* High-Speed Counters (HC0..HC5) are stored as native 32-bit words */
        if ((b0 != 0xF0U) && (area_id == PLC_AREA_HC)) {
            return *(uint32_t *)p;
        }
        return plc_be32_read(p);
    }

    case PLC_ADDR_MODE_IMM_BYTE:     /* 0x4..0x7: 4-Byte Big-Endian Immediate / &Ptr */
    case PLC_ADDR_MODE_IMM_WORD:
    case PLC_ADDR_MODE_IMM_DWORD:
    case PLC_ADDR_MODE_IMM_ADDR_PTR:
        *out_len = (uint8_t)(*out_len + 4U);
        return plc_be32_read(&ob1[pc]);

    case PLC_ADDR_MODE_INDIRECT_MEM: /* 0x8..0xB: Indirect Pointer (*VDx, *LDx, *ACx) */
    case PLC_ADDR_MODE_INDIRECT_AC1:
    case PLC_ADDR_MODE_INDIRECT_AC2:
    case PLC_ADDR_MODE_INDIRECT_AC3:
    {
        uint8_t *p = plc_oper_resolve_indirect_ptr(addr_mode, pc, local_mem, out_len);
        return (p != NULL) ? plc_be32_read(p) : 0U;
    }

    case PLC_ADDR_MODE_DIRECT_AC0:   /* 0xC..0xF: Accumulator AC0..AC3 (full 32 bits) */
    case PLC_ADDR_MODE_DIRECT_AC1:
    case PLC_ADDR_MODE_DIRECT_AC2:
    case PLC_ADDR_MODE_DIRECT_AC3:
        return PLC_ACCUM_BASE[(addr_mode & 0x0FU) - 12U];

    default:
        return 0U;
    }
}

/**
 * @brief Write an 8-bit value to a destination operand.
 *        Reconstructed from FUN_0803b424 (0x0803b424 .. 0x0803b711).
 */
void plc_oper_write_u8(uint8_t addr_mode,
                       uint16_t pc,
                       uint8_t *local_mem,
                       uint8_t *out_len,
                       uint8_t *trace_buf,
                       uint8_t val)
{
    (void)trace_buf;

    switch (addr_mode & 0x0FU) {
    case PLC_ADDR_MODE_DIRECT_MEM:
    {
        uint8_t *p = plc_oper_resolve_direct_ptr(pc, local_mem, out_len);
        if (p != NULL) {
            *p = val;
        }
        break;
    }

    case PLC_ADDR_MODE_INDIRECT_MEM:
    case PLC_ADDR_MODE_INDIRECT_AC1:
    case PLC_ADDR_MODE_INDIRECT_AC2:
    case PLC_ADDR_MODE_INDIRECT_AC3:
    {
        uint8_t *p = plc_oper_resolve_indirect_ptr(addr_mode, pc, local_mem, out_len);
        if (p != NULL) {
            *p = val;
        }
        break;
    }

    case PLC_ADDR_MODE_DIRECT_AC0:
    case PLC_ADDR_MODE_DIRECT_AC1:
    case PLC_ADDR_MODE_DIRECT_AC2:
    case PLC_ADDR_MODE_DIRECT_AC3:
    {
        uint8_t ac_idx = (uint8_t)((addr_mode & 0x0FU) - 12U);
        PLC_ACCUM_BASE[ac_idx] = (PLC_ACCUM_BASE[ac_idx] & 0xFFFFFF00U) | (uint32_t)val;
        break;
    }

    default:
        break;
    }
}

/**
 * @brief Write a 16-bit value to a destination operand.
 *        Reconstructed from FUN_0803bbd2 (0x0803bbd2 .. 0x0803bf40).
 */
void plc_oper_write_u16(uint8_t addr_mode,
                        uint16_t pc,
                        uint8_t *local_mem,
                        uint8_t *out_len,
                        uint8_t *trace_buf,
                        uint16_t val)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    (void)trace_buf;

    switch (addr_mode & 0x0FU) {
    case PLC_ADDR_MODE_DIRECT_MEM:
    {
        uint8_t b0 = ob1[pc];
        uint8_t area_id = (uint8_t)(b0 >> 4);
        uint8_t *p = plc_oper_resolve_direct_ptr(pc, local_mem, out_len);
        if (p != NULL) {
            if ((b0 != 0xF0U) &&
                ((area_id == PLC_AREA_T) || (area_id == PLC_AREA_C) ||
                 (area_id == PLC_AREA_AQ))) {
                *(uint16_t *)p = val;
            } else {
                plc_be16_write(p, val);
            }
        }
        break;
    }

    case PLC_ADDR_MODE_INDIRECT_MEM:
    case PLC_ADDR_MODE_INDIRECT_AC1:
    case PLC_ADDR_MODE_INDIRECT_AC2:
    case PLC_ADDR_MODE_INDIRECT_AC3:
    {
        uint8_t *p = plc_oper_resolve_indirect_ptr(addr_mode, pc, local_mem, out_len);
        if (p != NULL) {
            plc_be16_write(p, val);
        }
        break;
    }

    case PLC_ADDR_MODE_DIRECT_AC0:
    case PLC_ADDR_MODE_DIRECT_AC1:
    case PLC_ADDR_MODE_DIRECT_AC2:
    case PLC_ADDR_MODE_DIRECT_AC3:
    {
        uint8_t ac_idx = (uint8_t)((addr_mode & 0x0FU) - 12U);
        PLC_ACCUM_BASE[ac_idx] = (PLC_ACCUM_BASE[ac_idx] & 0xFFFF0000U) | (uint32_t)val;
        break;
    }

    default:
        break;
    }
}

/**
 * @brief Write a 32-bit value to a destination operand.
 *        Reconstructed from FUN_0803b712 (0x0803b712 .. 0x0803bbd1).
 */
void plc_oper_write_u32(uint8_t addr_mode,
                        uint16_t pc,
                        uint8_t *local_mem,
                        uint8_t *out_len,
                        uint8_t *trace_buf,
                        uint32_t val)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    (void)trace_buf;

    switch (addr_mode & 0x0FU) {
    case PLC_ADDR_MODE_DIRECT_MEM:
    {
        uint8_t b0 = ob1[pc];
        uint8_t area_id = (uint8_t)(b0 >> 4);
        uint8_t *p = plc_oper_resolve_direct_ptr(pc, local_mem, out_len);
        if (p != NULL) {
            if ((b0 != 0xF0U) && (area_id == PLC_AREA_HC)) {
                *(uint32_t *)p = val;
            } else {
                plc_be32_write(p, val);
            }
        }
        break;
    }

    case PLC_ADDR_MODE_INDIRECT_MEM:
    case PLC_ADDR_MODE_INDIRECT_AC1:
    case PLC_ADDR_MODE_INDIRECT_AC2:
    case PLC_ADDR_MODE_INDIRECT_AC3:
    {
        uint8_t *p = plc_oper_resolve_indirect_ptr(addr_mode, pc, local_mem, out_len);
        if (p != NULL) {
            plc_be32_write(p, val);
        }
        break;
    }

    case PLC_ADDR_MODE_DIRECT_AC0:
    case PLC_ADDR_MODE_DIRECT_AC1:
    case PLC_ADDR_MODE_DIRECT_AC2:
    case PLC_ADDR_MODE_DIRECT_AC3:
        PLC_ACCUM_BASE[(addr_mode & 0x0FU) - 12U] = val;
        break;

    default:
        break;
    }
}

/* ============================================================================
 * 3. Contact Comparison Helpers (FUN_08015ca6, FUN_08016252, FUN_08015e58)
 * ============================================================================
 */

/* Combine a boolean contact result (0 or 1) onto the 16-bit RLO logic stack */
static inline void plc_stack_apply(uint16_t *logic_stack, uint8_t stack_op, uint8_t bit_val)
{
    bit_val &= 1U;
    if (stack_op == 0U) {          /* LD: Push onto logic stack */
        *logic_stack = (uint16_t)((*logic_stack << 1) | bit_val);
    } else if (stack_op == 1U) {   /* A:  AND with top of logic stack */
        if (bit_val == 0U) {
            *logic_stack &= (uint16_t)~1U;
        }
    } else {                       /* O:  OR with top of logic stack */
        if (bit_val != 0U) {
            *logic_stack |= 1U;
        }
    }
}

/**
 * @brief Byte comparison helper for primary opcodes 0x91..0xA1 (odd) and
 *        extended opcodes 0xFE46..0xFE4E.
 *        Reconstructed from FUN_08015ca6 (0x08015ca6 .. 0x08015e57).
 */
static void plc_vm_cmp_byte(uint16_t full_opcode,
                            uint16_t *pc_ptr,
                            uint8_t *local_mem,
                            uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t hdr_len = (full_opcode >= 0xFE00U) ? 2U : 1U;
    uint8_t  modes   = ob1[*pc_ptr + hdr_len];

    uint8_t len1 = 0U;
    uint8_t in1  = plc_oper_read_u8((uint8_t)(modes >> 4),
                                    (uint16_t)(*pc_ptr + hdr_len + 1U),
                                    local_mem, &len1, NULL);

    uint8_t len2 = 0U;
    uint8_t in2  = plc_oper_read_u8((uint8_t)(modes & 0x0FU),
                                    (uint16_t)(*pc_ptr + hdr_len + 1U + len1),
                                    local_mem, &len2, NULL);

    *pc_ptr = (uint16_t)(*pc_ptr + hdr_len + 1U + len1 + len2);

    uint8_t stack_op = 0U; /* 0 = LD, 1 = A, 2 = O */
    bool    cond     = false;

    switch (full_opcode) {
    case PLC_OP_LDB_EQ: stack_op = 0U; cond = (in1 == in2); break;
    case PLC_OP_AB_EQ:  stack_op = 1U; cond = (in1 == in2); break;
    case PLC_OP_OB_EQ:  stack_op = 2U; cond = (in1 == in2); break;
    case PLC_OP_LDB_LE: stack_op = 0U; cond = (in1 <= in2); break;
    case PLC_OP_LDB_GE: stack_op = 0U; cond = (in1 >= in2); break;
    case PLC_OP_AB_LE:  stack_op = 1U; cond = (in1 <= in2); break;
    case PLC_OP_AB_GE:  stack_op = 1U; cond = (in1 >= in2); break;
    case PLC_OP_OW_LE + 1: /* 0x9F: OB<= */
                        stack_op = 2U; cond = (in1 <= in2); break;
    case PLC_OP_OB_GE:  stack_op = 2U; cond = (in1 >= in2); break;

    case 0xFE00U | PLC_EXT_LDB_NE: stack_op = 0U; cond = (in1 != in2); break;
    case 0xFE00U | PLC_EXT_AB_NE:  stack_op = 1U; cond = (in1 != in2); break;
    case 0xFE00U | PLC_EXT_OB_NE:  stack_op = 2U; cond = (in1 != in2); break;
    case 0xFE00U | PLC_EXT_LDB_LT: stack_op = 0U; cond = (in1 <  in2); break;
    case 0xFE00U | PLC_EXT_LDB_GT: stack_op = 0U; cond = (in1 >  in2); break;
    case 0xFE00U | PLC_EXT_AB_LT:  stack_op = 1U; cond = (in1 <  in2); break;
    case 0xFE00U | PLC_EXT_AB_GT:  stack_op = 1U; cond = (in1 >  in2); break;
    case 0xFE00U | PLC_EXT_OB_LT:  stack_op = 2U; cond = (in1 <  in2); break;
    case 0xFE00U | PLC_EXT_OB_GT:  stack_op = 2U; cond = (in1 >  in2); break;
    default: break;
    }

    plc_stack_apply(logic_stack, stack_op, cond ? 1U : 0U);
}

/**
 * @brief Signed 16-bit Word comparison helper for primary opcodes 0x90..0xA0 (even)
 *        and extended opcodes 0xFE4F..0xFE57.
 *        Reconstructed from FUN_08016252 (0x08016252 .. 0x08016550).
 */
static void plc_vm_cmp_word(uint16_t full_opcode,
                            uint16_t *pc_ptr,
                            uint8_t *local_mem,
                            uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t hdr_len = (full_opcode >= 0xFE00U) ? 2U : 1U;
    uint8_t  modes   = ob1[*pc_ptr + hdr_len];

    uint8_t len1 = 0U;
    int16_t in1  = plc_oper_read_i16((uint8_t)(modes >> 4),
                                     (uint16_t)(*pc_ptr + hdr_len + 1U),
                                     local_mem, &len1, NULL);

    uint8_t len2 = 0U;
    int16_t in2  = plc_oper_read_i16((uint8_t)(modes & 0x0FU),
                                     (uint16_t)(*pc_ptr + hdr_len + 1U + len1),
                                     local_mem, &len2, NULL);

    *pc_ptr = (uint16_t)(*pc_ptr + hdr_len + 1U + len1 + len2);

    uint8_t stack_op = 0U;
    bool    cond     = false;

    switch (full_opcode) {
    case PLC_OP_LDW_EQ: stack_op = 0U; cond = (in1 == in2); break;
    case PLC_OP_AW_EQ:  stack_op = 1U; cond = (in1 == in2); break;
    case PLC_OP_OW_EQ:  stack_op = 2U; cond = (in1 == in2); break;
    case PLC_OP_LDW_LE: stack_op = 0U; cond = (in1 <= in2); break;
    case PLC_OP_LDW_GE: stack_op = 0U; cond = (in1 >= in2); break;
    case PLC_OP_AW_LE:  stack_op = 1U; cond = (in1 <= in2); break;
    case PLC_OP_AW_GE:  stack_op = 1U; cond = (in1 >= in2); break;
    case PLC_OP_OW_LE:  stack_op = 2U; cond = (in1 <= in2); break;
    case PLC_OP_OW_GE:  stack_op = 2U; cond = (in1 >= in2); break;

    case 0xFE00U | PLC_EXT_LDW_NE: stack_op = 0U; cond = (in1 != in2); break;
    case 0xFE00U | PLC_EXT_AW_NE:  stack_op = 1U; cond = (in1 != in2); break;
    case 0xFE00U | PLC_EXT_OW_NE:  stack_op = 2U; cond = (in1 != in2); break;
    case 0xFE00U | PLC_EXT_LDW_LT: stack_op = 0U; cond = (in1 <  in2); break;
    case 0xFE00U | PLC_EXT_LDW_GT: stack_op = 0U; cond = (in1 >  in2); break;
    case 0xFE00U | PLC_EXT_AW_LT:  stack_op = 1U; cond = (in1 <  in2); break;
    case 0xFE00U | PLC_EXT_AW_GT:  stack_op = 1U; cond = (in1 >  in2); break;
    case 0xFE00U | PLC_EXT_OW_LT:  stack_op = 2U; cond = (in1 <  in2); break;
    case 0xFE00U | PLC_EXT_OW_GT:  stack_op = 2U; cond = (in1 >  in2); break;
    default: break;
    }

    plc_stack_apply(logic_stack, stack_op, cond ? 1U : 0U);
}

/**
 * @brief Signed 32-bit DWord and IEEE-754 Real comparison helper for primary
 *        opcodes 0xA2..0xA4, 0xB5..0xB7, 0xC0..0xC2 and extended opcodes
 *        0xFE39..0xFE41, 0xFE58..0xFE69.
 *        Reconstructed from FUN_08015e58 (0x08015e58 .. 0x08016251).
 */
static void plc_vm_cmp_dword_or_real(uint16_t full_opcode,
                                     uint16_t *pc_ptr,
                                     uint8_t *local_mem,
                                     uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t hdr_len = (full_opcode >= 0xFE00U) ? 2U : 1U;
    uint8_t  modes   = ob1[*pc_ptr + hdr_len];

    uint8_t  len1 = 0U;
    uint32_t raw1 = plc_oper_read_u32((uint8_t)(modes >> 4),
                                      (uint16_t)(*pc_ptr + hdr_len + 1U),
                                      local_mem, &len1, NULL);

    uint8_t  len2 = 0U;
    uint32_t raw2 = plc_oper_read_u32((uint8_t)(modes & 0x0FU),
                                      (uint16_t)(*pc_ptr + hdr_len + 1U + len1),
                                      local_mem, &len2, NULL);

    *pc_ptr = (uint16_t)(*pc_ptr + hdr_len + 1U + len1 + len2);

    int32_t d1 = (int32_t)raw1;
    int32_t d2 = (int32_t)raw2;
    float   r1 = plc_u32_to_float(raw1);
    float   r2 = plc_u32_to_float(raw2);

    uint8_t stack_op = 0U;
    bool    cond     = false;

    switch (full_opcode) {
    /* Primary DWord Comparisons (0xA2..0xA4, 0xB5..0xB7, 0xC0..0xC2) */
    case PLC_OP_LDD_EQ: stack_op = 0U; cond = (d1 == d2); break;
    case PLC_OP_LDD_LE: stack_op = 0U; cond = (d1 <= d2); break;
    case PLC_OP_LDD_GE: stack_op = 0U; cond = (d1 >= d2); break;
    case PLC_OP_AD_EQ:  stack_op = 1U; cond = (d1 == d2); break;
    case PLC_OP_AD_LE:  stack_op = 1U; cond = (d1 <= d2); break;
    case PLC_OP_AD_GE:  stack_op = 1U; cond = (d1 >= d2); break;
    case PLC_OP_OD_EQ:  stack_op = 2U; cond = (d1 == d2); break;
    case PLC_OP_OD_LE:  stack_op = 2U; cond = (d1 <= d2); break;
    case PLC_OP_OD_GE:  stack_op = 2U; cond = (d1 >= d2); break;

    /* Extended DWord Comparisons (0xFE58..0xFE60) */
    case 0xFE00U | PLC_EXT_LDD_NE: stack_op = 0U; cond = (d1 != d2); break;
    case 0xFE00U | PLC_EXT_AD_NE:  stack_op = 1U; cond = (d1 != d2); break;
    case 0xFE00U | PLC_EXT_OD_NE:  stack_op = 2U; cond = (d1 != d2); break;
    case 0xFE00U | PLC_EXT_LDD_LT: stack_op = 0U; cond = (d1 <  d2); break;
    case 0xFE00U | PLC_EXT_LDD_GT: stack_op = 0U; cond = (d1 >  d2); break;
    case 0xFE00U | PLC_EXT_AD_LT:  stack_op = 1U; cond = (d1 <  d2); break;
    case 0xFE00U | PLC_EXT_AD_GT:  stack_op = 1U; cond = (d1 >  d2); break;
    case 0xFE00U | PLC_EXT_OD_LT:  stack_op = 2U; cond = (d1 <  d2); break;
    case 0xFE00U | PLC_EXT_OD_GT:  stack_op = 2U; cond = (d1 >  d2); break;

    /* Extended Real IEEE-754 Comparisons (0xFE39..0xFE41, 0xFE61..0xFE69) */
    case 0xFE00U | PLC_EXT_LDR_EQ: stack_op = 0U; cond = (r1 == r2); break;
    case 0xFE00U | PLC_EXT_LDR_LE: stack_op = 0U; cond = (r1 <= r2); break;
    case 0xFE00U | PLC_EXT_LDR_GE: stack_op = 0U; cond = (r1 >= r2); break;
    case 0xFE00U | PLC_EXT_AR_EQ:  stack_op = 1U; cond = (r1 == r2); break;
    case 0xFE00U | PLC_EXT_AR_LE:  stack_op = 1U; cond = (r1 <= r2); break;
    case 0xFE00U | PLC_EXT_AR_GE:  stack_op = 1U; cond = (r1 >= r2); break;
    case 0xFE00U | PLC_EXT_OR_EQ:  stack_op = 2U; cond = (r1 == r2); break;
    case 0xFE00U | PLC_EXT_OR_LE:  stack_op = 2U; cond = (r1 <= r2); break;
    case 0xFE00U | PLC_EXT_OR_GE:  stack_op = 2U; cond = (r1 >= r2); break;
    case 0xFE00U | PLC_EXT_LDR_NE: stack_op = 0U; cond = (r1 != r2); break;
    case 0xFE00U | PLC_EXT_AR_NE:  stack_op = 1U; cond = (r1 != r2); break;
    case 0xFE00U | PLC_EXT_OR_NE:  stack_op = 2U; cond = (r1 != r2); break;
    case 0xFE00U | PLC_EXT_LDR_LT: stack_op = 0U; cond = (r1 <  r2); break;
    case 0xFE00U | PLC_EXT_LDR_GT: stack_op = 0U; cond = (r1 >  r2); break;
    case 0xFE00U | PLC_EXT_AR_LT:  stack_op = 1U; cond = (r1 <  r2); break;
    case 0xFE00U | PLC_EXT_AR_GT:  stack_op = 1U; cond = (r1 >  r2); break;
    case 0xFE00U | PLC_EXT_OR_LT:  stack_op = 2U; cond = (r1 <  r2); break;
    case 0xFE00U | PLC_EXT_OR_GT:  stack_op = 2U; cond = (r1 >  r2); break;
    default: break;
    }

    plc_stack_apply(logic_stack, stack_op, cond ? 1U : 0U);
}

/* ============================================================================
 * 4. Modular Opcode Group Handlers for FUN_08021948
 * ============================================================================
 */

/**
 * @brief Update SMB1 (0x20005F29) math status flags and instruction ENO bit.
 */
static inline void plc_vm_set_eno(uint8_t *eno_flags, bool eno_on)
{
    if (eno_on) {
        *eno_flags |= PLC_ENO_FLAG_BIT;
    } else {
        *eno_flags &= (uint8_t)~PLC_ENO_FLAG_BIT;
    }
}

/**
 * @brief Read a compact 2-byte bit operand (`0x00..0x5D` @ 0x08021a66..0x08021c53).
 *
 * Encoding:
 *   - `opcode` bits [7:4] = logic operation (0=LD, 1=LDN, 2=A, 3=AN, 4=O, 5=ON)
 *   - `opcode` bits [3:1] = area (0=I, 1=Q, 2=M, 3=S, 4=SM, 5=T, 6=C)
 *   - `opcode` bit  [0]   = bit 8 of operand byte/bit index
 *   - `ob1[pc + 1]`       = low 8 bits of bit index `(byte_offset << 3) | bit_num`,
 *                           or timer/counter index (0..255) for T (5) and C (6).
 */
static void plc_vm_exec_compact_bit_logic(uint8_t opcode,
                                          uint16_t *pc_ptr,
                                          uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t operand   = (uint16_t)(((uint16_t)(opcode & 1U) << 8) |
                                     (uint16_t)ob1[*pc_ptr + 1U]);
    uint8_t  area_code = (uint8_t)((opcode >> 1) & 0x07U);
    uint8_t  logic_op  = (uint8_t)(opcode >> 4);
    uint8_t  bit_val   = 0U;

    /* Dispatch on 3-bit area selector via tbb [0x0804ad32] @ 0x08021a9e */
    switch (area_code) {
    case PLC_BIT_AREA_I:  /* @ 0x08021aa6: Input I0.0..I15.7 */
        bit_val = (PLC_PHYS_BASE_I[operand >> 3] >> (operand & 7U)) & 1U;
        break;
    case PLC_BIT_AREA_Q:  /* @ 0x08021ab6: Output Q0.0..Q15.7 */
        bit_val = (PLC_PHYS_BASE_Q[operand >> 3] >> (operand & 7U)) & 1U;
        break;
    case PLC_BIT_AREA_M:  /* @ 0x08021ac6: Bit Memory M0.0..M31.7 */
        bit_val = (PLC_PHYS_BASE_M[operand >> 3] >> (operand & 7U)) & 1U;
        break;
    case PLC_BIT_AREA_S:  /* @ 0x08021ad6: SCR Memory S0.0..S31.7 */
        bit_val = (PLC_PHYS_BASE_S[operand >> 3] >> (operand & 7U)) & 1U;
        break;
    case PLC_BIT_AREA_SM: /* @ 0x08021ae6: Special Memory SM0.0..SM63.7 */
        bit_val = (PLC_PHYS_BASE_SM[operand >> 3] >> (operand & 7U)) & 1U;
        break;
    case PLC_BIT_AREA_T:  /* @ 0x08021af6: Timer Bit T0..T255 (bit 1 of status byte) */
        bit_val = (PLC_PHYS_BASE_T_STATUS[operand & 0xFFU] >> 1) & 1U;
        break;
    case PLC_BIT_AREA_C:  /* @ 0x08021b0c: Counter Bit C0..C255 (bit 4 of status byte) */
        bit_val = (PLC_PHYS_BASE_C_STATUS[operand & 0xFFU] >> 4) & 1U;
        break;
    default:
        break;
    }

    /* Invert bit for LDN (1), AN (3), ON (5) */
    if ((logic_op & 1U) != 0U) {
        bit_val ^= 1U;
    }

    /* Combine with logic_stack (0/1 -> LD, 2/3 -> A, 4/5 -> O) @ 0x08021b22..0x08021bd8 */
    plc_stack_apply(logic_stack, (uint8_t)(logic_op >> 1), bit_val);
    *pc_ptr = (uint16_t)(*pc_ptr + 2U);
}

/**
 * @brief Execute a compact 2-byte output coil assignment `=` (`0x60..0x6D` @ 0x08021c54..0x08021f8b).
 */
static void plc_vm_exec_compact_coil(uint8_t opcode,
                                     uint16_t *pc_ptr,
                                     uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t operand   = (uint16_t)(((uint16_t)(opcode & 1U) << 8) |
                                     (uint16_t)ob1[*pc_ptr + 1U]);
    uint8_t  area_code = (uint8_t)((opcode >> 1) & 0x07U);
    uint8_t  byte_off  = (uint8_t)(operand >> 3);
    uint8_t  bit_mask  = (uint8_t)(1U << (operand & 7U));
    bool     rlo_on    = ((*logic_stack & 1U) != 0U);
    uint8_t *dst_byte  = NULL;

    /* Dispatch via tbh [0x0804ad3a] @ 0x08021c90 */
    switch (area_code) {
    case PLC_BIT_AREA_I:  dst_byte = &PLC_PHYS_BASE_I[byte_off];  break; /* @ 0x08021c9a */
    case PLC_BIT_AREA_Q:  dst_byte = &PLC_PHYS_BASE_Q[byte_off];  break; /* @ 0x08021ce4 */
    case PLC_BIT_AREA_M:  dst_byte = &PLC_PHYS_BASE_M[byte_off];  break; /* @ 0x08021db6 */
    case PLC_BIT_AREA_S:  dst_byte = &PLC_PHYS_BASE_S[byte_off];  break; /* @ 0x08021e3e */
    case PLC_BIT_AREA_SM: dst_byte = &PLC_PHYS_BASE_SM[byte_off]; break; /* @ 0x08021e88 */
    default: break;
    }

    if (dst_byte != NULL) {
        if (rlo_on) {
            *dst_byte |= bit_mask;
        } else {
            *dst_byte &= (uint8_t)~bit_mask;
        }
    }

    *pc_ptr = (uint16_t)(*pc_ptr + 2U);
}

/**
 * @brief Execute 3-byte / 5-byte extended bit logic & immediate I/O (`0x80..0x8D` @ 0x08021f8c..0x08022501).
 */
static void plc_vm_exec_ext_bit_logic(uint8_t opcode,
                                      uint16_t *pc_ptr,
                                      uint8_t *local_mem,
                                      uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t pc        = *pc_ptr;
    uint8_t  b1        = ob1[pc + 1U];
    uint8_t  b2        = ob1[pc + 2U];

    /* 0x80..0x85: LD, LDN, A, AN, O, ON on SM (0x40), V (0x80..0x8F), L (0xE0), or 5-byte 0xF0 @ 0x08021f90 */
    if (opcode <= PLC_OP_ON_EXT) {
        uint8_t  bit_num  = b1 & 0x07U;
        uint8_t *byte_ptr = NULL;
        uint16_t insn_len = 3U;

        if (b2 == 0xF0U) {
            /* 5-byte extended address: [op, bit_num, 0xF0, 0x80(V)|0x40(SM), hi, lo] @ 0x08022040 */
            uint16_t off16 = plc_be16_read(&ob1[pc + 3U]);
            byte_ptr = ((b1 & 0xF0U) == 0x40U) ? (PLC_PHYS_BASE_SM + off16)
                                               : (PLC_PHYS_BASE_V  + off16);
            insn_len = 5U;
        } else {
            uint8_t  area_nib = (uint8_t)(b1 >> 4);
            uint16_t off12    = (uint16_t)(((uint16_t)((b1 >> 3) & 1U) << 8) | (uint16_t)b2);
            if (area_nib == 0x04U) {
                byte_ptr = PLC_PHYS_BASE_SM + off12;
            } else if (area_nib == 0x0EU) {
                byte_ptr = local_mem + (b2 & 0x3FU);
            } else {
                byte_ptr = PLC_PHYS_BASE_V +
                           (uint16_t)(((uint16_t)(b1 & 0x78U) << 5) | (uint16_t)b2);
            }
        }

        uint8_t bit_val  = (byte_ptr != NULL) ? ((*byte_ptr >> bit_num) & 1U) : 0U;
        uint8_t logic_op = (uint8_t)(opcode - PLC_OP_LD_EXT);
        if ((logic_op & 1U) != 0U) {
            bit_val ^= 1U;
        }
        plc_stack_apply(logic_stack, (uint8_t)(logic_op >> 1), bit_val);
        *pc_ptr = (uint16_t)(pc + insn_len);
        return;
    }

    /* 0x86: Extended Coil Assignment `=` on SM, V, L @ 0x08022210 */
    if (opcode == PLC_OP_ASSIGN_EXT) {
        uint8_t  bit_mask = (uint8_t)(1U << (b1 & 0x07U));
        uint8_t *byte_ptr = NULL;
        uint16_t insn_len = 3U;

        if (b2 == 0xF0U) {
            uint16_t off16 = plc_be16_read(&ob1[pc + 3U]);
            byte_ptr = ((b1 & 0xF0U) == 0x40U) ? (PLC_PHYS_BASE_SM + off16)
                                               : (PLC_PHYS_BASE_V  + off16);
            insn_len = 5U;
        } else {
            uint8_t area_nib = (uint8_t)(b1 >> 4);
            if (area_nib == 0x04U) {
                byte_ptr = PLC_PHYS_BASE_SM + b2;
            } else if (area_nib == 0x0EU) {
                byte_ptr = local_mem + (b2 & 0x3FU);
            } else {
                byte_ptr = PLC_PHYS_BASE_V +
                           (uint16_t)(((uint16_t)(b1 & 0x78U) << 5) | (uint16_t)b2);
            }
        }

        if (byte_ptr != NULL) {
            if ((*logic_stack & 1U) != 0U) {
                *byte_ptr |= bit_mask;
            } else {
                *byte_ptr &= (uint8_t)~bit_mask;
            }
        }
        *pc_ptr = (uint16_t)(pc + insn_len);
        return;
    }

    /* 0x87..0x8C: Immediate Input Contact Logic LDI, LDNI, AI, ANI, OI, ONI @ 0x08022148 */
    if (opcode <= PLC_OP_ONI) {
        uint8_t bit_num  = b1 & 0x07U;
        uint8_t byte_off = (uint8_t)(b2 & 0x0FU);
        uint8_t bit_val  = (PLC_PHYS_BASE_I[byte_off] >> bit_num) & 1U;
        uint8_t logic_op = (uint8_t)(opcode - PLC_OP_LDI);
        if ((logic_op & 1U) != 0U) {
            bit_val ^= 1U;
        }
        plc_stack_apply(logic_stack, (uint8_t)(logic_op >> 1), bit_val);
        *pc_ptr = (uint16_t)(pc + 3U);
        return;
    }

    /* 0x8D: Immediate Output Coil Assignment `=I` on Q0.0..Q15.7 @ 0x08022456 */
    if (opcode == PLC_OP_ASSIGN_IMM) {
        uint8_t bit_mask = (uint8_t)(1U << (b1 & 0x07U));
        uint8_t byte_off = (uint8_t)(b2 & 0x0FU);
        if ((*logic_stack & 1U) != 0U) {
            PLC_PHYS_BASE_Q[byte_off] |= bit_mask;
        } else {
            PLC_PHYS_BASE_Q[byte_off] &= (uint8_t)~bit_mask;
        }
        *pc_ptr = (uint16_t)(pc + 3U);
    }
}

/**
 * @brief Execute S7-200 Timers: TON (0xA6 @ 0x08022e16), TONR (0xA7 @ 0x080231a6),
 *        and TOF (0xFE 0x0F @ 0x08029090).
 *
 * Timer resolution is determined by the timer number T0..T255:
 *   - 1 ms:   T0, T64, T32, T96
 *   - 10 ms:  T1..T4, T65..T68, T33..T36, T97..T100
 *   - 100 ms: T5..T31, T69..T95, T37..T63, T101..T255
 */
static uint16_t plc_vm_get_timer_tick(uint8_t t_num)
{
    uint8_t mod32 = (uint8_t)(t_num & 0x1FU);
    if ((t_num < 128U) && (mod32 == 0U)) {
        return PLC_TICK_1MS_COUNTER;
    }
    if ((t_num < 128U) && (mod32 >= 1U) && (mod32 <= 4U)) {
        return PLC_TICK_10MS_COUNTER;
    }
    return PLC_TICK_100MS_COUNTER;
}

static void plc_vm_exec_timer(uint16_t full_opcode,
                              uint16_t *pc_ptr,
                              uint8_t *local_mem,
                              uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t pc        = *pc_ptr;
    uint16_t hdr_len   = (full_opcode >= 0xFE00U) ? 2U : 1U;

    uint8_t  pt_mode   = (uint8_t)(ob1[pc + hdr_len] >> 4);
    uint8_t  t_num     = ob1[pc + hdr_len + 2U];
    uint8_t  pt_len    = 0U;
    int16_t  pt_val    = plc_oper_read_i16(pt_mode,
                                           (uint16_t)(pc + hdr_len + 3U),
                                           local_mem, &pt_len, NULL);
    *pc_ptr = (uint16_t)(pc + hdr_len + 3U + pt_len);

    if (pt_val < 0) {
        PLC_VM_ERROR_REG = PLC_ERR_OPERAND_RANGE;
        return;
    }

    bool     en_in    = ((*logic_stack & 1U) != 0U);
    uint16_t cur_tick = plc_vm_get_timer_tick(t_num);
    uint16_t elapsed  = (uint16_t)(cur_tick - PLC_PHYS_BASE_T_LAST_TICK[t_num]);
    PLC_PHYS_BASE_T_LAST_TICK[t_num] = cur_tick;

    uint8_t  status = PLC_PHYS_BASE_T_STATUS[t_num];
    uint16_t cur_v  = PLC_PHYS_BASE_T_VAL[t_num];

    if (full_opcode == PLC_OP_TON) {
        /* TON @ 0x08022e16: On-Delay Timer (resets to 0 when EN == 0) */
        if (!en_in) {
            PLC_PHYS_BASE_T_VAL[t_num]    = 0U;
            PLC_PHYS_BASE_T_STATUS[t_num] = status & (uint8_t)~(PLC_TIMER_BIT_MASK | PLC_TIMER_ACTIVE_MASK);
        } else {
            uint32_t next_v = (uint32_t)cur_v + (uint32_t)elapsed;
            if (next_v > 0x7FFFU) {
                next_v = 0x7FFFU;
            }
            cur_v = (uint16_t)next_v;
            PLC_PHYS_BASE_T_VAL[t_num] = cur_v;
            if ((int16_t)cur_v >= pt_val) {
                status |= PLC_TIMER_BIT_MASK;
            }
            PLC_PHYS_BASE_T_STATUS[t_num] = status | PLC_TIMER_ACTIVE_MASK;
        }
    } else if (full_opcode == PLC_OP_TONR) {
        /* TONR @ 0x080231a6: Retentive On-Delay Timer (holds value when EN == 0) */
        if (en_in) {
            uint32_t next_v = (uint32_t)cur_v + (uint32_t)elapsed;
            if (next_v > 0x7FFFU) {
                next_v = 0x7FFFU;
            }
            cur_v = (uint16_t)next_v;
            PLC_PHYS_BASE_T_VAL[t_num] = cur_v;
            if ((int16_t)cur_v >= pt_val) {
                status |= PLC_TIMER_BIT_MASK;
            }
            PLC_PHYS_BASE_T_STATUS[t_num] = status | PLC_TIMER_ACTIVE_MASK;
        } else {
            PLC_PHYS_BASE_T_STATUS[t_num] = status & (uint8_t)~PLC_TIMER_ACTIVE_MASK;
        }
    } else {
        /* TOF @ 0x08029090: Off-Delay Timer (0xFE 0x0F) */
        bool prev_en = ((status & PLC_TIMER_PREV_EN_MASK) != 0U);
        if (en_in) {
            PLC_PHYS_BASE_T_VAL[t_num]    = 0U;
            PLC_PHYS_BASE_T_STATUS[t_num] = (status | PLC_TIMER_BIT_MASK | PLC_TIMER_PREV_EN_MASK) &
                                            (uint8_t)~PLC_TIMER_ACTIVE_MASK;
        } else {
            if (prev_en) {
                status |= PLC_TIMER_ACTIVE_MASK;
            }
            if ((status & PLC_TIMER_ACTIVE_MASK) != 0U) {
                uint32_t next_v = (uint32_t)cur_v + (uint32_t)elapsed;
                if ((int32_t)next_v >= (int32_t)pt_val) {
                    cur_v  = (uint16_t)pt_val;
                    status &= (uint8_t)~(PLC_TIMER_BIT_MASK | PLC_TIMER_ACTIVE_MASK);
                } else {
                    cur_v = (uint16_t)next_v;
                }
                PLC_PHYS_BASE_T_VAL[t_num] = cur_v;
            }
            PLC_PHYS_BASE_T_STATUS[t_num] = status & (uint8_t)~PLC_TIMER_PREV_EN_MASK;
        }
    }
}

/**
 * @brief Execute S7-200 Counters: CTU (0xA8 @ 0x08023528), CTUD (0xA9 @ 0x080236c2),
 *        and CTD (0xFE 0x10 @ 0x080309e4).
 */
static void plc_vm_exec_counter(uint16_t full_opcode,
                                uint16_t *pc_ptr,
                                uint8_t *local_mem,
                                uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t pc        = *pc_ptr;
    uint16_t hdr_len   = (full_opcode >= 0xFE00U) ? 2U : 1U;

    uint8_t  pv_mode   = (uint8_t)(ob1[pc + hdr_len] >> 4);
    uint8_t  c_num     = ob1[pc + hdr_len + 2U];
    uint8_t  pv_len    = 0U;
    int16_t  pv_val    = plc_oper_read_i16(pv_mode,
                                           (uint16_t)(pc + hdr_len + 3U),
                                           local_mem, &pv_len, NULL);
    *pc_ptr = (uint16_t)(pc + hdr_len + 3U + pv_len);

    uint8_t status = PLC_PHYS_BASE_C_STATUS[c_num];
    int16_t cur_v  = (int16_t)PLC_PHYS_BASE_C_VAL[c_num];

    if (full_opcode == PLC_OP_CTU) {
        /* CTU @ 0x08023528: top of stack = Reset (R), second of stack = Count Up (CU) */
        bool r_in  = ((*logic_stack & 1U) != 0U);
        bool cu_in = (((*logic_stack >> 1) & 1U) != 0U);
        *logic_stack >>= 1;

        if (r_in) {
            cur_v  = 0;
            status &= (uint8_t)~PLC_COUNTER_BIT_MASK;
        } else if (cu_in && ((status & PLC_COUNTER_PREV_CU_MASK) == 0U)) {
            if (cur_v < 32767) {
                cur_v++;
            }
        }
        if (cur_v >= pv_val) {
            status |= PLC_COUNTER_BIT_MASK;
        } else {
            status &= (uint8_t)~PLC_COUNTER_BIT_MASK;
        }
        status = (status & (uint8_t)~PLC_COUNTER_PREV_CU_MASK) |
                 (cu_in ? PLC_COUNTER_PREV_CU_MASK : 0U);
    } else if (full_opcode == PLC_OP_CTUD) {
        /* CTUD @ 0x080236c2: bit0 = R, bit1 = CD, bit2 = CU */
        bool r_in  = ((*logic_stack & 1U) != 0U);
        bool cd_in = (((*logic_stack >> 1) & 1U) != 0U);
        bool cu_in = (((*logic_stack >> 2) & 1U) != 0U);
        *logic_stack >>= 2;

        if (r_in) {
            cur_v  = 0;
            status &= (uint8_t)~PLC_COUNTER_BIT_MASK;
        } else {
            if (cu_in && ((status & PLC_COUNTER_PREV_CU_MASK) == 0U) && (cur_v < 32767)) {
                cur_v++;
            }
            if (cd_in && ((status & PLC_COUNTER_PREV_CD_MASK) == 0U) && (cur_v > -32768)) {
                cur_v--;
            }
        }
        if (cur_v >= pv_val) {
            status |= PLC_COUNTER_BIT_MASK;
        } else {
            status &= (uint8_t)~PLC_COUNTER_BIT_MASK;
        }
        status &= (uint8_t)~(PLC_COUNTER_PREV_CU_MASK | PLC_COUNTER_PREV_CD_MASK);
        if (cu_in) status |= PLC_COUNTER_PREV_CU_MASK;
        if (cd_in) status |= PLC_COUNTER_PREV_CD_MASK;
    } else {
        /* CTD @ 0x080309e4 (0xFE 0x10): bit0 = Load (LD), bit1 = Count Down (CD) */
        bool ld_in = ((*logic_stack & 1U) != 0U);
        bool cd_in = (((*logic_stack >> 1) & 1U) != 0U);
        *logic_stack >>= 1;

        if (ld_in) {
            cur_v  = pv_val;
            status &= (uint8_t)~PLC_COUNTER_BIT_MASK;
        } else if (cd_in && ((status & PLC_COUNTER_PREV_CU_MASK) == 0U)) {
            if (cur_v > -32768) {
                cur_v--;
            }
        }
        if (cur_v <= 0) {
            status |= PLC_COUNTER_BIT_MASK;
        } else {
            status &= (uint8_t)~PLC_COUNTER_BIT_MASK;
        }
        status = (status & (uint8_t)~PLC_COUNTER_PREV_CU_MASK) |
                 (cd_in ? PLC_COUNTER_PREV_CU_MASK : 0U);
    }

    PLC_PHYS_BASE_C_VAL[c_num]    = (uint16_t)cur_v;
    PLC_PHYS_BASE_C_STATUS[c_num] = status;
}

/**
 * @brief Execute 2-Operand Integer & Bitwise Word/DWord Operations:
 *        +I (0xAA), -I (0xAC), +D (0xAB), -D (0xAD), MUL (0xAE), DIV (0xAF),
 *        *I (0xFE13), /I (0xFE14), *D (0xFE11), /D (0xFE12),
 *        ANDW (0xB8), ORW (0xBA), XORW (0xBC), ANDD (0xB9), ORD (0xBB), XORD (0xBD).
 */
static void plc_vm_exec_alu_int(uint16_t full_opcode,
                                uint16_t *pc_ptr,
                                uint8_t *local_mem,
                                uint8_t *eno_flags,
                                uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t pc        = *pc_ptr;
    uint16_t hdr_len   = (full_opcode >= 0xFE00U) ? 2U : 1U;
    uint8_t  modes     = ob1[pc + hdr_len];
    uint8_t  mode_in1  = (uint8_t)(modes >> 4);
    uint8_t  mode_out  = (uint8_t)(modes & 0x0FU);
    bool     en_in     = ((*logic_stack & 1U) != 0U) && (PLC_VM_ERROR_REG == 0U);

    /* 16-bit Word Operations: +I, -I, *I, /I, ANDW, ORW, XORW, MUL, DIV */
    if ((full_opcode == PLC_OP_ADD_I) || (full_opcode == PLC_OP_SUB_I) ||
        (full_opcode == PLC_OP_AND_W) || (full_opcode == PLC_OP_OR_W)  ||
        (full_opcode == PLC_OP_XOR_W) || (full_opcode == PLC_OP_MUL)   ||
        (full_opcode == PLC_OP_DIV)   ||
        (full_opcode == (0xFE00U | PLC_EXT_MUL_I)) ||
        (full_opcode == (0xFE00U | PLC_EXT_DIV_I))) {

        uint8_t len1 = 0U;
        int16_t in1  = plc_oper_read_i16(mode_in1, (uint16_t)(pc + hdr_len + 1U),
                                         local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + hdr_len + 1U + len1);
        uint8_t  len2   = 0U;

        if (full_opcode == PLC_OP_MUL) {
            /* MUL @ 0x08022a62: int16 IN1 * int16 OUT_LO -> int32 OUT */
            int16_t in2 = (int16_t)plc_oper_read_u32(mode_out, out_pc, local_mem, &len2, NULL);
            *pc_ptr = (uint16_t)(out_pc + len2);
            if (!en_in) {
                plc_vm_set_eno(eno_flags, false);
                return;
            }
            int32_t prod = (int32_t)in1 * (int32_t)in2;
            uint8_t wlen = 0U;
            plc_oper_write_u32(mode_out, out_pc, local_mem, &wlen, NULL, (uint32_t)prod);
            PLC_SMB1 &= (uint8_t)~(PLC_SM1_0_ZERO | PLC_SM1_1_OVERFLOW | PLC_SM1_2_NEGATIVE);
            if (prod == 0) PLC_SMB1 |= PLC_SM1_0_ZERO;
            if (prod <  0) PLC_SMB1 |= PLC_SM1_2_NEGATIVE;
            plc_vm_set_eno(eno_flags, true);
            return;
        }

        if (full_opcode == PLC_OP_DIV) {
            /* DIV @ 0x08022bf2: int32 OUT / int16 IN1 -> (rem16 << 16) | quot16 */
            int32_t dividend = (int32_t)plc_oper_read_u32(mode_out, out_pc, local_mem, &len2, NULL);
            *pc_ptr = (uint16_t)(out_pc + len2);
            if (!en_in) {
                plc_vm_set_eno(eno_flags, false);
                return;
            }
            PLC_SMB1 &= (uint8_t)~(PLC_SM1_0_ZERO | PLC_SM1_1_OVERFLOW |
                                   PLC_SM1_2_NEGATIVE | PLC_SM1_3_DIV_BY_ZERO);
            if (in1 == 0) {
                PLC_SMB1 |= (PLC_SM1_1_OVERFLOW | PLC_SM1_3_DIV_BY_ZERO);
                plc_vm_set_eno(eno_flags, false);
                return;
            }
            int32_t quot = dividend / (int32_t)in1;
            int32_t rem  = dividend % (int32_t)in1;
            if ((quot > 32767) || (quot < -32768)) {
                PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
                plc_vm_set_eno(eno_flags, false);
                return;
            }
            uint32_t packed = ((uint32_t)(uint16_t)rem << 16) | (uint32_t)(uint16_t)quot;
            uint8_t  wlen   = 0U;
            plc_oper_write_u32(mode_out, out_pc, local_mem, &wlen, NULL, packed);
            if (quot == 0) PLC_SMB1 |= PLC_SM1_0_ZERO;
            if (quot <  0) PLC_SMB1 |= PLC_SM1_2_NEGATIVE;
            plc_vm_set_eno(eno_flags, true);
            return;
        }

        int16_t in2 = plc_oper_read_i16(mode_out, out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            return;
        }

        int32_t res32 = 0;
        bool    ovf   = false;

        switch (full_opcode) {
        case PLC_OP_ADD_I: /* @ 0x080225c2 */
            res32 = (int32_t)in2 + (int32_t)in1;
            ovf   = (res32 > 32767) || (res32 < -32768);
            break;
        case PLC_OP_SUB_I: /* @ 0x080225c2 */
            res32 = (int32_t)in2 - (int32_t)in1;
            ovf   = (res32 > 32767) || (res32 < -32768);
            break;
        case 0xFE00U | PLC_EXT_MUL_I: /* @ 0x0802fce0 */
            res32 = (int32_t)in2 * (int32_t)in1;
            ovf   = (res32 > 32767) || (res32 < -32768);
            break;
        case 0xFE00U | PLC_EXT_DIV_I: /* @ 0x0802fce0 */
            PLC_SMB1 &= (uint8_t)~PLC_SM1_3_DIV_BY_ZERO;
            if (in1 == 0) {
                PLC_SMB1 |= (PLC_SM1_1_OVERFLOW | PLC_SM1_3_DIV_BY_ZERO);
                plc_vm_set_eno(eno_flags, false);
                return;
            }
            if ((in2 == -32768) && (in1 == -1)) {
                res32 = -32768;
                ovf   = true;
            } else {
                res32 = (int32_t)in2 / (int32_t)in1;
            }
            break;
        case PLC_OP_AND_W: res32 = (int16_t)((uint16_t)in2 & (uint16_t)in1); break; /* @ 0x08023d18 */
        case PLC_OP_OR_W:  res32 = (int16_t)((uint16_t)in2 | (uint16_t)in1); break; /* @ 0x08023d18 */
        case PLC_OP_XOR_W: res32 = (int16_t)((uint16_t)in2 ^ (uint16_t)in1); break; /* @ 0x08023d18 */
        default: break;
        }

        uint16_t res16 = (uint16_t)res32;
        uint8_t  wlen  = 0U;
        plc_oper_write_u16(mode_out, out_pc, local_mem, &wlen, NULL, res16);

        PLC_SMB1 &= (uint8_t)~(PLC_SM1_0_ZERO | PLC_SM1_1_OVERFLOW | PLC_SM1_2_NEGATIVE);
        if (res16 == 0U)        PLC_SMB1 |= PLC_SM1_0_ZERO;
        if ((int16_t)res16 < 0) PLC_SMB1 |= PLC_SM1_2_NEGATIVE;
        if (ovf)                PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
        plc_vm_set_eno(eno_flags, !ovf);
        return;
    }

    /* 32-bit DWord Operations: +D, -D, *D, /D, ANDD, ORD, XORD */
    uint8_t  len1 = 0U;
    uint32_t u1   = plc_oper_read_u32(mode_in1, (uint16_t)(pc + hdr_len + 1U),
                                      local_mem, &len1, NULL);
    uint16_t out_pc = (uint16_t)(pc + hdr_len + 1U + len1);
    uint8_t  len2   = 0U;
    uint32_t u2     = plc_oper_read_u32(mode_out, out_pc, local_mem, &len2, NULL);
    *pc_ptr = (uint16_t)(out_pc + len2);

    if (!en_in) {
        plc_vm_set_eno(eno_flags, false);
        return;
    }

    int64_t  res64 = 0;
    uint32_t res32 = 0U;
    bool     ovf   = false;

    switch (full_opcode) {
    case PLC_OP_ADD_D: /* @ 0x080227ee */
        res64 = (int64_t)(int32_t)u2 + (int64_t)(int32_t)u1;
        ovf   = (res64 > 2147483647LL) || (res64 < -2147483648LL);
        res32 = (uint32_t)res64;
        break;
    case PLC_OP_SUB_D: /* @ 0x080227ee */
        res64 = (int64_t)(int32_t)u2 - (int64_t)(int32_t)u1;
        ovf   = (res64 > 2147483647LL) || (res64 < -2147483648LL);
        res32 = (uint32_t)res64;
        break;
    case 0xFE00U | PLC_EXT_MUL_D: /* @ 0x0802fec0 */
        res64 = (int64_t)(int32_t)u2 * (int64_t)(int32_t)u1;
        ovf   = (res64 > 2147483647LL) || (res64 < -2147483648LL);
        res32 = (uint32_t)res64;
        break;
    case 0xFE00U | PLC_EXT_DIV_D: /* @ 0x0802fec0 */
        PLC_SMB1 &= (uint8_t)~PLC_SM1_3_DIV_BY_ZERO;
        if (u1 == 0U) {
            PLC_SMB1 |= (PLC_SM1_1_OVERFLOW | PLC_SM1_3_DIV_BY_ZERO);
            plc_vm_set_eno(eno_flags, false);
            return;
        }
        if (((int32_t)u2 == (-2147483647 - 1)) && ((int32_t)u1 == -1)) {
            res32 = 0x80000000U;
            ovf   = true;
        } else {
            res32 = (uint32_t)((int32_t)u2 / (int32_t)u1);
        }
        break;
    case PLC_OP_AND_D: res32 = u2 & u1; break; /* @ 0x08023e60 */
    case PLC_OP_OR_D:  res32 = u2 | u1; break; /* @ 0x08023e60 */
    case PLC_OP_XOR_D: res32 = u2 ^ u1; break; /* @ 0x08023e60 */
    default: break;
    }

    uint8_t wlen = 0U;
    plc_oper_write_u32(mode_out, out_pc, local_mem, &wlen, NULL, res32);

    PLC_SMB1 &= (uint8_t)~(PLC_SM1_0_ZERO | PLC_SM1_1_OVERFLOW | PLC_SM1_2_NEGATIVE);
    if (res32 == 0U)        PLC_SMB1 |= PLC_SM1_0_ZERO;
    if ((int32_t)res32 < 0) PLC_SMB1 |= PLC_SM1_2_NEGATIVE;
    if (ovf)                PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
    plc_vm_set_eno(eno_flags, !ovf);
}

/**
 * @brief Execute Single-Operand Inc / Dec / Invert on Byte, Word, or DWord:
 *        INCB (0xF5), DECB (0xF6), INVB (0xF4) @ 0x08035a46
 *        INCW (0xB1), DECW (0xB3), INVW (0xBE) @ 0x0802396c
 *        INCD (0xB2), DECD (0xB4), INVD (0xBF) @ 0x08023b2a
 */
static void plc_vm_exec_unary_inc_dec_inv(uint8_t opcode,
                                          uint16_t *pc_ptr,
                                          uint8_t *local_mem,
                                          uint8_t *eno_flags,
                                          uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t pc        = *pc_ptr;
    uint8_t  addr_mode = (uint8_t)(ob1[pc + 1U] >> 4);
    uint16_t oper_pc   = (uint16_t)(pc + 2U);
    bool     en_in     = ((*logic_stack & 1U) != 0U) && (PLC_VM_ERROR_REG == 0U);
    uint8_t  rlen      = 0U;

    /* Byte: INVB (0xF4), INCB (0xF5), DECB (0xF6) @ 0x08035a46 */
    if ((opcode >= PLC_OP_INV_B) && (opcode <= PLC_OP_DEC_B)) {
        uint8_t val8 = plc_oper_read_u8(addr_mode, oper_pc, local_mem, &rlen, NULL);
        *pc_ptr = (uint16_t)(oper_pc + rlen);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            return;
        }
        bool    ovf  = false;
        uint8_t res8 = val8;
        if (opcode == PLC_OP_INV_B) {
            res8 = (uint8_t)~val8;
        } else if (opcode == PLC_OP_INC_B) {
            ovf  = (val8 == 0xFFU);
            res8 = (uint8_t)(val8 + 1U);
        } else {
            ovf  = (val8 == 0x00U);
            res8 = (uint8_t)(val8 - 1U);
        }
        uint8_t wlen = 0U;
        plc_oper_write_u8(addr_mode, oper_pc, local_mem, &wlen, NULL, res8);
        PLC_SMB1 &= (uint8_t)~(PLC_SM1_0_ZERO | PLC_SM1_1_OVERFLOW);
        if (res8 == 0U) PLC_SMB1 |= PLC_SM1_0_ZERO;
        if (ovf)        PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
        plc_vm_set_eno(eno_flags, !ovf);
        return;
    }

    /* Word: INCW (0xB1), DECW (0xB3), INVW (0xBE) @ 0x0802396c */
    if ((opcode == PLC_OP_INC_W) || (opcode == PLC_OP_DEC_W) || (opcode == PLC_OP_INV_W)) {
        int16_t val16 = plc_oper_read_i16(addr_mode, oper_pc, local_mem, &rlen, NULL);
        *pc_ptr = (uint16_t)(oper_pc + rlen);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            return;
        }
        bool     ovf   = false;
        uint16_t res16 = (uint16_t)val16;
        if (opcode == PLC_OP_INV_W) {
            res16 = (uint16_t)~res16;
        } else if (opcode == PLC_OP_INC_W) {
            ovf   = (val16 == 32767);
            res16 = (uint16_t)(res16 + 1U);
        } else {
            ovf   = (val16 == -32768);
            res16 = (uint16_t)(res16 - 1U);
        }
        uint8_t wlen = 0U;
        plc_oper_write_u16(addr_mode, oper_pc, local_mem, &wlen, NULL, res16);
        PLC_SMB1 &= (uint8_t)~(PLC_SM1_0_ZERO | PLC_SM1_1_OVERFLOW | PLC_SM1_2_NEGATIVE);
        if (res16 == 0U)        PLC_SMB1 |= PLC_SM1_0_ZERO;
        if ((int16_t)res16 < 0) PLC_SMB1 |= PLC_SM1_2_NEGATIVE;
        if (ovf)                PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
        plc_vm_set_eno(eno_flags, !ovf);
        return;
    }

    /* DWord: INCD (0xB2), DECD (0xB4), INVD (0xBF) @ 0x08023b2a */
    uint32_t val32 = plc_oper_read_u32(addr_mode, oper_pc, local_mem, &rlen, NULL);
    *pc_ptr = (uint16_t)(oper_pc + rlen);
    if (!en_in) {
        plc_vm_set_eno(eno_flags, false);
        return;
    }
    bool     ovf   = false;
    uint32_t res32 = val32;
    if (opcode == PLC_OP_INV_D) {
        res32 = ~val32;
    } else if (opcode == PLC_OP_INC_D) {
        ovf   = ((int32_t)val32 == 2147483647);
        res32 = val32 + 1U;
    } else {
        ovf   = ((int32_t)val32 == (-2147483647 - 1));
        res32 = val32 - 1U;
    }
    uint8_t wlen = 0U;
    plc_oper_write_u32(addr_mode, oper_pc, local_mem, &wlen, NULL, res32);
    PLC_SMB1 &= (uint8_t)~(PLC_SM1_0_ZERO | PLC_SM1_1_OVERFLOW | PLC_SM1_2_NEGATIVE);
    if (res32 == 0U)        PLC_SMB1 |= PLC_SM1_0_ZERO;
    if ((int32_t)res32 < 0) PLC_SMB1 |= PLC_SM1_2_NEGATIVE;
    if (ovf)                PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
    plc_vm_set_eno(eno_flags, !ovf);
}

/**
 * @brief Execute PID Loop Controller Instruction (`0xB0` @ 0x08023fec..0x08024791).
 *
 * S7-200 36-byte PID Loop Table layout (at `tbl_ptr`):
 *   - Offset 0..3   (REAL): PV_n   (Normalized Process Variable, 0.0 .. 1.0)
 *   - Offset 4..7   (REAL): SP_n   (Normalized Setpoint, 0.0 .. 1.0)
 *   - Offset 8..11  (REAL): Mn     (Normalized Output, 0.0 .. 1.0)
 *   - Offset 12..15 (REAL): Kc     (Loop Gain)
 *   - Offset 16..19 (REAL): Ts     (Sample Time in seconds)
 *   - Offset 20..23 (REAL): Ti     (Integral Time in minutes)
 *   - Offset 24..27 (REAL): Td     (Derivative Time in minutes)
 *   - Offset 28..31 (REAL): Mx     (Integral Sum / Bias, 0.0 .. 1.0)
 *   - Offset 32..35 (REAL): PV_old (Previous PV_n for derivative term)
 */
static void plc_vm_exec_pid(uint16_t *pc_ptr,
                            uint8_t *local_mem,
                            uint8_t *eno_flags,
                            uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t pc        = *pc_ptr;
    uint8_t  addr_mode = (uint8_t)(ob1[pc + 1U] >> 4);
    uint8_t  rlen      = 0U;
    uint8_t *tbl_ptr   = NULL;

    if (addr_mode == PLC_ADDR_MODE_DIRECT_MEM) {
        tbl_ptr = plc_oper_resolve_direct_ptr((uint16_t)(pc + 2U), local_mem, &rlen);
    } else {
        uint32_t vptr = plc_oper_read_u32(addr_mode, (uint16_t)(pc + 2U), local_mem, &rlen, NULL);
        tbl_ptr = plc_vptr_to_phys_local(vptr, local_mem);
    }

    uint8_t loop_len = 0U;
    uint8_t loop_id  = plc_oper_read_u8(PLC_ADDR_MODE_IMM_BYTE,
                                        (uint16_t)(pc + 2U + rlen),
                                        local_mem, &loop_len, NULL);
    (void)loop_id;
    *pc_ptr = (uint16_t)(pc + 2U + rlen + loop_len);

    if (((*logic_stack & 1U) == 0U) || (PLC_VM_ERROR_REG != 0U) || (tbl_ptr == NULL)) {
        plc_vm_set_eno(eno_flags, false);
        return;
    }

    float pv_n   = plc_u32_to_float(plc_be32_read(&tbl_ptr[0]));
    float sp_n   = plc_u32_to_float(plc_be32_read(&tbl_ptr[4]));
    float kc     = plc_u32_to_float(plc_be32_read(&tbl_ptr[12]));
    float ts     = plc_u32_to_float(plc_be32_read(&tbl_ptr[16]));
    float ti     = plc_u32_to_float(plc_be32_read(&tbl_ptr[20]));
    float td     = plc_u32_to_float(plc_be32_read(&tbl_ptr[24]));
    float mx     = plc_u32_to_float(plc_be32_read(&tbl_ptr[28]));
    float pv_old = plc_u32_to_float(plc_be32_read(&tbl_ptr[32]));

    float err    = sp_n - pv_n;
    float mp     = kc * err;
    float mi     = (ti > 0.0f) ? ((kc * ts / (ti * 60.0f)) * err + mx) : mx;
    float md     = (ts > 0.0f) ? (-kc * (td * 60.0f / ts) * (pv_n - pv_old)) : 0.0f;
    float mn     = mp + mi + md;

    if (mn > 1.0f) {
        mn = 1.0f;
        mi = 1.0f - (mp + md);
    } else if (mn < 0.0f) {
        mn = 0.0f;
        mi = -(mp + md);
    }

    plc_be32_write(&tbl_ptr[8],  plc_float_to_u32(mn));
    plc_be32_write(&tbl_ptr[28], plc_float_to_u32(mi));
    plc_be32_write(&tbl_ptr[32], plc_float_to_u32(pv_n));
    plc_vm_set_eno(eno_flags, true);
}

/**
 * @brief Execute Multi-Bit Set / Reset / Toggle (`0xF1..0xF3` @ 0x08035b94..0x08035cb5).
 */
static void plc_vm_exec_multi_bit_sr(uint8_t opcode,
                                     uint16_t *pc_ptr,
                                     uint8_t *local_mem,
                                     uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t pc        = *pc_ptr;
    uint8_t  b1        = ob1[pc + 1U];
    uint8_t  bit_num   = b1 & 0x07U;
    uint8_t  n_mode    = (uint8_t)(b1 >> 4);
    uint8_t  addr_len  = 0U;
    uint8_t *byte_ptr  = plc_oper_resolve_direct_ptr((uint16_t)(pc + 2U), local_mem, &addr_len);
    uint8_t  n_len     = 0U;
    uint8_t  count     = plc_oper_read_u8(n_mode, (uint16_t)(pc + 2U + addr_len),
                                          local_mem, &n_len, NULL);
    *pc_ptr = (uint16_t)(pc + 2U + addr_len + n_len);

    if (((*logic_stack & 1U) == 0U) || (byte_ptr == NULL)) {
        return;
    }

    /* Special case for Reset (0xF2) on Timer (0x90) or Counter (0xA0) */
    uint8_t area_id = (uint8_t)(ob1[pc + 2U] >> 4);
    uint8_t tc_idx  = ob1[pc + 3U];
    if ((opcode == PLC_OP_RESET_BITS) && (ob1[pc + 2U] != 0xF0U)) {
        if (area_id == PLC_AREA_T) {
            PLC_PHYS_BASE_T_VAL[tc_idx]    = 0U;
            PLC_PHYS_BASE_T_STATUS[tc_idx] = 0U;
            return;
        }
        if (area_id == PLC_AREA_C) {
            PLC_PHYS_BASE_C_VAL[tc_idx]    = 0U;
            PLC_PHYS_BASE_C_STATUS[tc_idx] = 0U;
            return;
        }
    }

    for (uint8_t i = 0U; i < count; i++) {
        uint8_t mask = (uint8_t)(1U << bit_num);
        if (opcode == PLC_OP_SET_BITS) {
            *byte_ptr |= mask;
        } else if (opcode == PLC_OP_RESET_BITS) {
            *byte_ptr &= (uint8_t)~mask;
        } else {
            *byte_ptr ^= mask;
        }
        bit_num++;
        if (bit_num > 7U) {
            bit_num = 0U;
            byte_ptr++;
        }
    }
}

/**
 * @brief Execute 0xFE Extended Instructions (142-case jump table at 0x0804aec8,
 *        dispatched at 0x08028508 for sub_opcode = 0x07..0x94).
 */
static void plc_vm_exec_ext_fe(uint16_t *pc_ptr,
                               uint16_t *logic_stack,
                               uint8_t *eno_flags,
                               uint8_t *local_mem,
                               uint16_t *edge_idx_ptr,
                               uint16_t *call_depth,
                               uint16_t *for_depth,
                               uint16_t *for_pc_stack,
                               uint16_t *for_indx_pc_stack,
                               int16_t  *for_final_stack,
                               uint8_t  *for_indx_mode_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint16_t pc        = *pc_ptr;
    uint8_t  sub_op    = ob1[pc + 1U];
    bool     en_in     = ((*logic_stack & 1U) != 0U) && (PLC_VM_ERROR_REG == 0U);

    if ((sub_op < 0x07U) || (sub_op > 0x94U)) {
        PLC_VM_ERROR_REG = PLC_ERR_UNSUPPORTED_SUB_OPCODE;
        plc_vm_set_eno(eno_flags, false);
        *pc_ptr = (uint16_t)(pc + 2U);
        return;
    }

    /* 142-case jump table @ 0x08028508 (table 0x0804aec8, index = sub_op - 7) */
    switch ((plc_ext_fe_opcode_t)sub_op) {

    /* ------------------------------------------------------------------------
     * Case 0 (0xFE 0x07) @ 0x08029418: SHRB (Shift Register Bit)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_SHRB:
    {
        uint8_t  b2       = ob1[pc + 2U];
        uint8_t  n_mode   = (uint8_t)(b2 >> 4);
        uint8_t  data_bit = b2 & 0x07U;
        uint8_t  s_bit    = ob1[pc + 3U] & 0x07U;
        uint8_t  len1     = 0U;
        uint8_t *data_ptr = plc_oper_resolve_direct_ptr((uint16_t)(pc + 4U), local_mem, &len1);
        uint8_t  len2     = 0U;
        uint8_t *sbit_ptr = plc_oper_resolve_direct_ptr((uint16_t)(pc + 4U + len1), local_mem, &len2);
        uint8_t  len3     = 0U;
        int8_t   n_val    = (int8_t)plc_oper_read_u8(n_mode,
                                                     (uint16_t)(pc + 4U + len1 + len2),
                                                     local_mem, &len3, NULL);
        *pc_ptr = (uint16_t)(pc + 4U + len1 + len2 + len3);

        if (!en_in || (data_ptr == NULL) || (sbit_ptr == NULL) || (n_val == 0)) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint8_t in_bit = (*data_ptr >> data_bit) & 1U;
        uint8_t abs_n  = (n_val > 0) ? (uint8_t)n_val : (uint8_t)(-n_val);
        for (uint8_t i = abs_n; i > 1U; i--) {
            uint8_t src_idx = (uint8_t)(s_bit + i - 2U);
            uint8_t dst_idx = (uint8_t)(s_bit + i - 1U);
            uint8_t b       = (sbit_ptr[src_idx >> 3] >> (src_idx & 7U)) & 1U;
            if (b != 0U) sbit_ptr[dst_idx >> 3] |= (uint8_t)(1U << (dst_idx & 7U));
            else         sbit_ptr[dst_idx >> 3] &= (uint8_t)~(1U << (dst_idx & 7U));
        }
        if (in_bit != 0U) sbit_ptr[s_bit >> 3] |= (uint8_t)(1U << (s_bit & 7U));
        else              sbit_ptr[s_bit >> 3] &= (uint8_t)~(1U << (s_bit & 7U));
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 2 & 3 (0xFE 0x09, 0xFE 0x0A) @ 0x0802e7f4 / 0x0802e964: DECO & ENCO
     * ------------------------------------------------------------------------ */
    case PLC_EXT_DECO: /* @ 0x0802e7f4: OUT_WORD = 1 << (IN_BYTE & 0x0F) */
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        uint8_t  in_b   = plc_oper_read_u8((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                           local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_i16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (en_in) {
            uint8_t wlen = 0U;
            plc_oper_write_u16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL,
                               (uint16_t)(1U << (in_b & 0x0FU)));
            plc_vm_set_eno(eno_flags, true);
        } else {
            plc_vm_set_eno(eno_flags, false);
        }
        break;
    }

    case PLC_EXT_ENCO: /* @ 0x0802e964: OUT_BYTE = ctz(IN_WORD) */
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        uint16_t in_w   = (uint16_t)plc_oper_read_i16((uint8_t)(modes >> 4),
                                                      (uint16_t)(pc + 3U),
                                                      local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        if (in_w == 0U) {
            PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint8_t bit_pos = 0U;
        while (((in_w >> bit_pos) & 1U) == 0U) {
            bit_pos++;
        }
        uint8_t wlen = 0U;
        plc_oper_write_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL, bit_pos);
        PLC_SMB1 &= (uint8_t)~PLC_SM1_1_OVERFLOW;
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 4 & 5 (0xFE 0x0B, 0xFE 0x0C) @ 0x0802d61a: ATH & HTA
     * ------------------------------------------------------------------------ */
    case PLC_EXT_ATH:
    case PLC_EXT_HTA:
    {
        uint8_t  modes   = ob1[pc + 2U];
        uint8_t  len1    = 0U;
        uint8_t *in_ptr  = (modes >> 4) == 0U
                           ? plc_oper_resolve_direct_ptr((uint16_t)(pc + 3U), local_mem, &len1)
                           : plc_oper_resolve_indirect_ptr((uint8_t)(modes >> 4),
                                                           (uint16_t)(pc + 3U), local_mem, &len1);
        uint8_t  len2    = 0U;
        uint8_t *out_ptr = (modes & 0x0FU) == 0U
                           ? plc_oper_resolve_direct_ptr((uint16_t)(pc + 3U + len1), local_mem, &len2)
                           : plc_oper_resolve_indirect_ptr((uint8_t)(modes & 0x0FU),
                                                           (uint16_t)(pc + 3U + len1), local_mem, &len2);
        uint8_t  len3    = 0U;
        uint8_t  n_nib   = plc_oper_read_u8(PLC_ADDR_MODE_IMM_BYTE,
                                            (uint16_t)(pc + 3U + len1 + len2),
                                            local_mem, &len3, NULL);
        *pc_ptr = (uint16_t)(pc + 3U + len1 + len2 + len3);

        if (!en_in || (in_ptr == NULL) || (out_ptr == NULL)) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        PLC_SMB1 &= (uint8_t)~PLC_SM1_7_NOT_HEX_ASCII;
        if (sub_op == PLC_EXT_ATH) {
            for (uint8_t i = 0U; i < n_nib; i++) {
                uint8_t ch = in_ptr[i];
                uint8_t v  = 0U;
                if ((ch >= '0') && (ch <= '9'))      v = (uint8_t)(ch - '0');
                else if ((ch >= 'A') && (ch <= 'F')) v = (uint8_t)(ch - 'A' + 10U);
                else if ((ch >= 'a') && (ch <= 'f')) v = (uint8_t)(ch - 'a' + 10U);
                else PLC_SMB1 |= PLC_SM1_7_NOT_HEX_ASCII;
                if ((i & 1U) == 0U) out_ptr[i >> 1] = (uint8_t)((out_ptr[i >> 1] & 0x0FU) | (v << 4));
                else                out_ptr[i >> 1] = (uint8_t)((out_ptr[i >> 1] & 0xF0U) | v);
            }
        } else {
            static const char k_hex[16] = "0123456789ABCDEF";
            for (uint8_t i = 0U; i < n_nib; i++) {
                uint8_t b = in_ptr[i >> 1];
                uint8_t v = ((i & 1U) == 0U) ? (uint8_t)(b >> 4) : (uint8_t)(b & 0x0FU);
                out_ptr[i] = (uint8_t)k_hex[v];
            }
        }
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    /* ------------------------------------------------------------------------
     * Case 6 (0xFE 0x0D) @ 0x08032f4e: XMT & Case 141 (0xFE 0x94) @ 0x08034190: RCV
     * ------------------------------------------------------------------------ */
    case PLC_EXT_XMT:
    case PLC_EXT_RCV:
        *pc_ptr = (uint16_t)(pc + ((ob1[pc + 3U] == 0xF0U) ? 7U : 5U));
        plc_vm_set_eno(eno_flags, en_in);
        break;

    /* ------------------------------------------------------------------------
     * Case 7 (0xFE 0x0E) @ 0x08029b16: HDEF & Case 135 (0xFE 0x8E) @ 0x08032bec: HSC
     * ------------------------------------------------------------------------ */
    case PLC_EXT_HDEF:
    case PLC_EXT_HSC:
        *pc_ptr = (uint16_t)(pc + 4U);
        plc_vm_set_eno(eno_flags, en_in);
        break;

    /* ------------------------------------------------------------------------
     * Case 8 (0xFE 0x0F) @ 0x08029090: TOF & Case 9 (0xFE 0x10) @ 0x080309e4: CTD
     * ------------------------------------------------------------------------ */
    case PLC_EXT_TOF:
        plc_vm_exec_timer((uint16_t)(0xFE00U | sub_op), pc_ptr, local_mem, logic_stack);
        break;

    case PLC_EXT_CTD:
        plc_vm_exec_counter((uint16_t)(0xFE00U | sub_op), pc_ptr, local_mem, logic_stack);
        break;

    /* ------------------------------------------------------------------------
     * Cases 10..13 (0xFE 0x11..0x14) @ 0x0802fec0 / 0x0802fce0: *D, /D, *I, /I
     * ------------------------------------------------------------------------ */
    case PLC_EXT_MUL_D:
    case PLC_EXT_DIV_D:
    case PLC_EXT_MUL_I:
    case PLC_EXT_DIV_I:
        plc_vm_exec_alu_int((uint16_t)(0xFE00U | sub_op), pc_ptr, local_mem, eno_flags, logic_stack);
        break;

    /* ------------------------------------------------------------------------
     * Case 15 (0xFE 0x16) @ 0x080304c6: FOR (Start of FOR/NEXT Loop)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_FOR:
    {
        uint8_t  m_indx_init = ob1[pc + 2U];
        uint8_t  m_final     = (uint8_t)(ob1[pc + 3U] >> 4);
        uint16_t indx_pc     = (uint16_t)(pc + 4U);
        uint8_t  len1        = 0U;
        (void)plc_oper_read_i16((uint8_t)(m_indx_init >> 4), indx_pc, local_mem, &len1, NULL);

        uint16_t init_pc     = (uint16_t)(indx_pc + len1);
        uint8_t  len2        = 0U;
        int16_t  init_val    = plc_oper_read_i16((uint8_t)(m_indx_init & 0x0FU),
                                                 init_pc, local_mem, &len2, NULL);

        uint16_t final_pc    = (uint16_t)(init_pc + len2);
        uint8_t  len3        = 0U;
        int16_t  final_val   = plc_oper_read_i16(m_final, final_pc, local_mem, &len3, NULL);

        *pc_ptr = (uint16_t)(final_pc + len3);

        if (*for_depth < PLC_MAX_FOR_DEPTH) {
            uint16_t d = *for_depth;
            (*for_depth)++;
            for_pc_stack[d]        = *pc_ptr;
            for_indx_pc_stack[d]   = indx_pc;
            for_final_stack[d]     = final_val;
            for_indx_mode_stack[d] = (uint8_t)(m_indx_init >> 4);

            if (en_in) {
                uint8_t wlen = 0U;
                plc_oper_write_u16((uint8_t)(m_indx_init >> 4), indx_pc,
                                   local_mem, &wlen, NULL, (uint16_t)init_val);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 24..27 (0xFE 0x1F..0x22) @ 0x0802850c: SI / RI (Immediate Set/Reset)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_SI_1F:
    case PLC_EXT_RI_20:
    case PLC_EXT_SI_21:
    case PLC_EXT_RI_22:
    {
        uint8_t b2       = ob1[pc + 2U];
        uint8_t bit_num  = b2 & 0x07U;
        uint8_t n_mode   = (uint8_t)(b2 >> 4);
        uint8_t byte_off = (uint8_t)(ob1[pc + 3U] & 0x0FU);
        uint8_t n_len    = 0U;
        uint8_t count    = plc_oper_read_u8(n_mode, (uint16_t)(pc + 4U), local_mem, &n_len, NULL);
        *pc_ptr = (uint16_t)(pc + 4U + n_len);

        if (en_in) {
            bool is_set = (sub_op == PLC_EXT_SI_1F) || (sub_op == PLC_EXT_SI_21);
            for (uint8_t i = 0U; i < count; i++) {
                uint8_t mask = (uint8_t)(1U << bit_num);
                if (is_set) PLC_PHYS_BASE_Q[byte_off] |= mask;
                else        PLC_PHYS_BASE_Q[byte_off] &= (uint8_t)~mask;
                bit_num++;
                if (bit_num > 7U) {
                    bit_num = 0U;
                    byte_off++;
                }
            }
        }
        break;
    }

    /* ------------------------------------------------------------------------
     * Case 28 (0xFE 0x23) @ 0x080328da: CALL with Parameters
     * ------------------------------------------------------------------------ */
    case PLC_EXT_CALL_ARGS:
    {
        if (!en_in || (*call_depth >= PLC_MAX_CALL_DEPTH)) {
            plc_vm_set_eno(eno_flags, false);
            *pc_ptr = plc_call_skip_params(pc);
            break;
        }

        uint8_t  sbr_id       = ob1[pc + 3U];
        uint16_t callee_pc    = PLC_SBR_ENTRY_PC_TBL[sbr_id];
        uint16_t callee_stack = 1U;
        uint8_t  callee_eno   = PLC_ENO_FLAG_BIT;
        uint8_t  callee_local[64];
        memset(callee_local, 0, sizeof(callee_local));

        (void)plc_call_marshal_in_params(pc, local_mem, callee_local, logic_stack);
        (*call_depth)++;
        plc_vm_exec_block(&callee_pc, &callee_stack, &callee_eno,
                          callee_local, edge_idx_ptr, call_depth);
        (*call_depth)--;
        *pc_ptr = plc_call_unmarshal_out_params(pc, local_mem, callee_local);
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 30..41 (0xFE 0x25..0x30) @ 0x0802b2b6 / 0x0802b4b4 / 0x0802b718:
     * SLB, SRB, RLB, RRB, SLW, SRW, RLW, RRW, SLD, SRD, RLD, RRD
     * ------------------------------------------------------------------------ */
    case PLC_EXT_SLB:
    case PLC_EXT_SRB:
    case PLC_EXT_RLB:
    case PLC_EXT_RRB:
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint16_t out_pc = (uint16_t)(pc + 3U);
        uint8_t  len1   = 0U;
        uint8_t  val8   = plc_oper_read_u8((uint8_t)(modes >> 4), out_pc, local_mem, &len1, NULL);
        uint8_t  len2   = 0U;
        uint8_t  cnt    = plc_oper_read_u8((uint8_t)(modes & 0x0FU),
                                           (uint16_t)(out_pc + len1), local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len1 + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint8_t res8 = val8;
        if (sub_op == PLC_EXT_SLB)      res8 = (cnt >= 8U) ? 0U : (uint8_t)(val8 << cnt);
        else if (sub_op == PLC_EXT_SRB) res8 = (cnt >= 8U) ? 0U : (uint8_t)(val8 >> cnt);
        else if (sub_op == PLC_EXT_RLB) res8 = (uint8_t)((val8 << (cnt & 7U)) | (val8 >> ((8U - (cnt & 7U)) & 7U)));
        else                            res8 = (uint8_t)((val8 >> (cnt & 7U)) | (val8 << ((8U - (cnt & 7U)) & 7U)));
        uint8_t wlen = 0U;
        plc_oper_write_u8((uint8_t)(modes >> 4), out_pc, local_mem, &wlen, NULL, res8);
        PLC_SMB1 &= (uint8_t)~PLC_SM1_0_ZERO;
        if (res8 == 0U) PLC_SMB1 |= PLC_SM1_0_ZERO;
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    case PLC_EXT_SLW:
    case PLC_EXT_SRW:
    case PLC_EXT_RLW:
    case PLC_EXT_RRW:
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint16_t out_pc = (uint16_t)(pc + 3U);
        uint8_t  len1   = 0U;
        uint16_t val16  = (uint16_t)plc_oper_read_i16((uint8_t)(modes >> 4), out_pc,
                                                      local_mem, &len1, NULL);
        uint8_t  len2   = 0U;
        uint8_t  cnt    = plc_oper_read_u8((uint8_t)(modes & 0x0FU),
                                           (uint16_t)(out_pc + len1), local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len1 + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint16_t res16 = val16;
        if (sub_op == PLC_EXT_SLW)      res16 = (cnt >= 16U) ? 0U : (uint16_t)(val16 << cnt);
        else if (sub_op == PLC_EXT_SRW) res16 = (cnt >= 16U) ? 0U : (uint16_t)(val16 >> cnt);
        else if (sub_op == PLC_EXT_RLW) res16 = (uint16_t)((val16 << (cnt & 15U)) | (val16 >> ((16U - (cnt & 15U)) & 15U)));
        else                            res16 = (uint16_t)((val16 >> (cnt & 15U)) | (val16 << ((16U - (cnt & 15U)) & 15U)));
        uint8_t wlen = 0U;
        plc_oper_write_u16((uint8_t)(modes >> 4), out_pc, local_mem, &wlen, NULL, res16);
        PLC_SMB1 &= (uint8_t)~PLC_SM1_0_ZERO;
        if (res16 == 0U) PLC_SMB1 |= PLC_SM1_0_ZERO;
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    case PLC_EXT_SLD:
    case PLC_EXT_SRD:
    case PLC_EXT_RLD:
    case PLC_EXT_RRD:
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint16_t out_pc = (uint16_t)(pc + 3U);
        uint8_t  len1   = 0U;
        uint32_t val32  = plc_oper_read_u32((uint8_t)(modes >> 4), out_pc, local_mem, &len1, NULL);
        uint8_t  len2   = 0U;
        uint8_t  cnt    = plc_oper_read_u8((uint8_t)(modes & 0x0FU),
                                           (uint16_t)(out_pc + len1), local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len1 + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint32_t res32 = val32;
        if (sub_op == PLC_EXT_SLD)      res32 = (cnt >= 32U) ? 0U : (val32 << cnt);
        else if (sub_op == PLC_EXT_SRD) res32 = (cnt >= 32U) ? 0U : (val32 >> cnt);
        else if (sub_op == PLC_EXT_RLD) res32 = (val32 << (cnt & 31U)) | (val32 >> ((32U - (cnt & 31U)) & 31U));
        else                            res32 = (val32 >> (cnt & 31U)) | (val32 << ((32U - (cnt & 31U)) & 31U));
        uint8_t wlen = 0U;
        plc_oper_write_u32((uint8_t)(modes >> 4), out_pc, local_mem, &wlen, NULL, res32);
        PLC_SMB1 &= (uint8_t)~PLC_SM1_0_ZERO;
        if (res32 == 0U) PLC_SMB1 |= PLC_SM1_0_ZERO;
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    /* ------------------------------------------------------------------------
     * Case 42 (0xFE 0x31) @ 0x0802eb1c: SEG (7-Segment Display Decode)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_SEG:
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        uint8_t  in_b   = plc_oper_read_u8((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                           local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (en_in) {
            uint8_t wlen = 0U;
            plc_oper_write_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL,
                              k_plc_seg7_table[in_b & 0x0FU]);
            plc_vm_set_eno(eno_flags, true);
        } else {
            plc_vm_set_eno(eno_flags, false);
        }
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 43..49, 59..60, 108..112: IEEE-754 Floating-Point Math & Conversions
     *   - 0xFE 0x32..0x35 (@ 0x08030150): +R, -R, *R, /R
     *   - 0xFE 0x36, 0x37, 0x43 (@ 0x0802bd08): ROUND, TRUNC, DTR
     *   - 0xFE 0x38 (@ 0x0802eca2): MOVR
     *   - 0xFE 0x42, 0x73..0x77 (@ 0x08030742): SQRT, SIN, COS, TAN, LN, EXP
     * ------------------------------------------------------------------------ */
    case PLC_EXT_ADD_R:
    case PLC_EXT_SUB_R:
    case PLC_EXT_MUL_R:
    case PLC_EXT_DIV_R:
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        float    in1    = plc_u32_to_float(plc_oper_read_u32((uint8_t)(modes >> 4),
                                                             (uint16_t)(pc + 3U),
                                                             local_mem, &len1, NULL));
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        float    in2    = plc_u32_to_float(plc_oper_read_u32((uint8_t)(modes & 0x0FU),
                                                             out_pc, local_mem, &len2, NULL));
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        if ((sub_op == PLC_EXT_DIV_R) && (in1 == 0.0f)) {
            PLC_SMB1 |= (PLC_SM1_1_OVERFLOW | PLC_SM1_3_DIV_BY_ZERO);
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        float res = (sub_op == PLC_EXT_ADD_R) ? (in2 + in1) :
                    (sub_op == PLC_EXT_SUB_R) ? (in2 - in1) :
                    (sub_op == PLC_EXT_MUL_R) ? (in2 * in1) : (in2 / in1);
        uint8_t wlen = 0U;
        plc_oper_write_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL,
                           plc_float_to_u32(res));
        PLC_SMB1 &= (uint8_t)~(PLC_SM1_0_ZERO | PLC_SM1_1_OVERFLOW |
                               PLC_SM1_2_NEGATIVE | PLC_SM1_3_DIV_BY_ZERO);
        if (res == 0.0f) PLC_SMB1 |= PLC_SM1_0_ZERO;
        if (res <  0.0f) PLC_SMB1 |= PLC_SM1_2_NEGATIVE;
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    case PLC_EXT_ROUND:
    case PLC_EXT_TRUNC:
    case PLC_EXT_DTR:
    case PLC_EXT_MOVR:
    case PLC_EXT_SQRT:
    case PLC_EXT_SIN:
    case PLC_EXT_COS:
    case PLC_EXT_TAN:
    case PLC_EXT_LN:
    case PLC_EXT_EXP:
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        uint32_t raw_in = plc_oper_read_u32((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                            local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint32_t out_bits = raw_in;
        if (sub_op == PLC_EXT_DTR) {
            out_bits = plc_float_to_u32((float)(int32_t)raw_in);
        } else if (sub_op == PLC_EXT_TRUNC) {
            out_bits = (uint32_t)(int32_t)plc_u32_to_float(raw_in);
        } else if (sub_op == PLC_EXT_ROUND) {
            float f  = plc_u32_to_float(raw_in);
            out_bits = (uint32_t)(int32_t)((f >= 0.0f) ? (f + 0.5f) : (f - 0.5f));
        } else if (sub_op == PLC_EXT_SQRT) {
            out_bits = plc_float_to_u32(sqrtf(plc_u32_to_float(raw_in)));
        } else if (sub_op == PLC_EXT_SIN) {
            out_bits = plc_float_to_u32(sinf(plc_u32_to_float(raw_in)));
        } else if (sub_op == PLC_EXT_COS) {
            out_bits = plc_float_to_u32(cosf(plc_u32_to_float(raw_in)));
        } else if (sub_op == PLC_EXT_TAN) {
            out_bits = plc_float_to_u32(tanf(plc_u32_to_float(raw_in)));
        } else if (sub_op == PLC_EXT_LN) {
            out_bits = plc_float_to_u32(logf(plc_u32_to_float(raw_in)));
        } else if (sub_op == PLC_EXT_EXP) {
            out_bits = plc_float_to_u32(expf(plc_u32_to_float(raw_in)));
        }
        uint8_t wlen = 0U;
        plc_oper_write_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL, out_bits);
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 50..58, 63..98: Extended Contact Comparisons (0xFE 0x39..0x69)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_LDB_NE: case PLC_EXT_AB_NE: case PLC_EXT_OB_NE:
    case PLC_EXT_LDB_LT: case PLC_EXT_LDB_GT:
    case PLC_EXT_AB_LT:  case PLC_EXT_AB_GT:
    case PLC_EXT_OB_LT:  case PLC_EXT_OB_GT:
        /* @ 0x0802b120: Extended Byte Comparisons -> FUN_08015ca6 */
        plc_vm_cmp_byte((uint16_t)(0xFE00U | sub_op), pc_ptr, local_mem, logic_stack);
        break;

    case PLC_EXT_LDW_NE: case PLC_EXT_AW_NE: case PLC_EXT_OW_NE:
    case PLC_EXT_LDW_LT: case PLC_EXT_LDW_GT:
    case PLC_EXT_AW_LT:  case PLC_EXT_AW_GT:
    case PLC_EXT_OW_LT:  case PLC_EXT_OW_GT:
        /* @ 0x0802b156: Extended Word Comparisons -> FUN_08016252 */
        plc_vm_cmp_word((uint16_t)(0xFE00U | sub_op), pc_ptr, local_mem, logic_stack);
        break;

    case PLC_EXT_LDR_EQ: case PLC_EXT_LDR_LE: case PLC_EXT_LDR_GE:
    case PLC_EXT_AR_EQ:  case PLC_EXT_AR_LE:  case PLC_EXT_AR_GE:
    case PLC_EXT_OR_EQ:  case PLC_EXT_OR_LE:  case PLC_EXT_OR_GE:
    case PLC_EXT_LDD_NE: case PLC_EXT_AD_NE:  case PLC_EXT_OD_NE:
    case PLC_EXT_LDD_LT: case PLC_EXT_LDD_GT:
    case PLC_EXT_AD_LT:  case PLC_EXT_AD_GT:
    case PLC_EXT_OD_LT:  case PLC_EXT_OD_GT:
    case PLC_EXT_LDR_NE: case PLC_EXT_AR_NE:  case PLC_EXT_OR_NE:
    case PLC_EXT_LDR_LT: case PLC_EXT_LDR_GT:
    case PLC_EXT_AR_LT:  case PLC_EXT_AR_GT:
    case PLC_EXT_OR_LT:  case PLC_EXT_OR_GT:
        /* @ 0x0802b184: Extended DWord & Real Comparisons -> FUN_08015e58 */
        plc_vm_cmp_dword_or_real((uint16_t)(0xFE00U | sub_op), pc_ptr, local_mem, logic_stack);
        break;

    /* ------------------------------------------------------------------------
     * Cases 61 & 62 (0xFE 0x44, 0xFE 0x45) @ 0x080357d2 / 0x080358d2: BIR & BIW
     * ------------------------------------------------------------------------ */
    case PLC_EXT_BIR:
    case PLC_EXT_BIW:
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        uint8_t  in_b   = plc_oper_read_u8((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                           local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (en_in) {
            uint8_t wlen = 0U;
            plc_oper_write_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL, in_b);
            plc_vm_set_eno(eno_flags, true);
        } else {
            plc_vm_set_eno(eno_flags, false);
        }
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 99..102 (0xFE 0x6A..0x6D) @ 0x0802b1b2..0x0802bbaa:
     * B_I (Byte->Int16), I_B (Int16->Byte), ITD (Int16->DInt32), DTI (DInt32->Int16)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_B_TO_I: /* @ 0x0802b1b2 */
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        uint8_t  in_b   = plc_oper_read_u8((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                           local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_i16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (en_in) {
            uint8_t wlen = 0U;
            plc_oper_write_u16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL,
                               (uint16_t)in_b);
            plc_vm_set_eno(eno_flags, true);
        } else {
            plc_vm_set_eno(eno_flags, false);
        }
        break;
    }

    case PLC_EXT_I_TO_B: /* @ 0x0802b97c */
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        int16_t  in_w   = plc_oper_read_i16((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                            local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        if ((in_w < 0) || (in_w > 255)) {
            PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint8_t wlen = 0U;
        plc_oper_write_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL, (uint8_t)in_w);
        PLC_SMB1 &= (uint8_t)~PLC_SM1_1_OVERFLOW;
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    case PLC_EXT_ITD: /* @ 0x0802ba80 */
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        int16_t  in_w   = plc_oper_read_i16((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                            local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (en_in) {
            uint8_t wlen = 0U;
            plc_oper_write_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL,
                               (uint32_t)(int32_t)in_w);
            plc_vm_set_eno(eno_flags, true);
        } else {
            plc_vm_set_eno(eno_flags, false);
        }
        break;
    }

    case PLC_EXT_DTI: /* @ 0x0802bbaa */
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        int32_t  in_d   = (int32_t)plc_oper_read_u32((uint8_t)(modes >> 4),
                                                     (uint16_t)(pc + 3U),
                                                     local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        (void)plc_oper_read_i16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(out_pc + len2);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        if ((in_d < -32768) || (in_d > 32767)) {
            PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint8_t wlen = 0U;
        plc_oper_write_u16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL,
                           (uint16_t)(int16_t)in_d);
        PLC_SMB1 &= (uint8_t)~PLC_SM1_1_OVERFLOW;
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 106 & 107 (0xFE 0x71, 0xFE 0x72) @ 0x0802fa66 / 0x0802fba0:
     * Byte Bitwise (ANDB/ORB/XORB) & Byte Add/Sub (+B/-B)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_BITWISE_B:
    case PLC_EXT_ADDSUB_B:
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        uint8_t  in1    = plc_oper_read_u8((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                           local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        uint8_t  in2    = plc_oper_read_u8((uint8_t)(modes & 0x0FU), out_pc,
                                           local_mem, &len2, NULL);
        uint8_t  sub_fn = ob1[out_pc + len2];
        *pc_ptr = (uint16_t)(out_pc + len2 + 1U);
        if (!en_in) {
            plc_vm_set_eno(eno_flags, false);
            break;
        }
        uint8_t res8 = in2;
        if (sub_op == PLC_EXT_BITWISE_B) {
            res8 = (sub_fn == 0U) ? (in2 & in1) :
                   (sub_fn == 1U) ? (in2 | in1) : (in2 ^ in1);
        } else {
            res8 = (sub_fn == 0U) ? (uint8_t)(in2 + in1) : (uint8_t)(in2 - in1);
        }
        uint8_t wlen = 0U;
        plc_oper_write_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &wlen, NULL, res8);
        plc_vm_set_eno(eno_flags, true);
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 132 & 133 (0xFE 0x8B, 0xFE 0x8C) @ 0x08032940 / 0x08032a50:
     * BITIM (Begin Interval Time) & CITIM (Calculate Interval Time)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_BITIM: /* @ 0x08032940 */
    {
        uint8_t mode_out = (uint8_t)(ob1[pc + 2U] >> 4);
        uint8_t len      = 0U;
        if (en_in) {
            plc_oper_write_u32(mode_out, (uint16_t)(pc + 3U), local_mem, &len, NULL,
                               PLC_SYS_MS_TICK);
            plc_vm_set_eno(eno_flags, true);
        } else {
            (void)plc_oper_read_u32(mode_out, (uint16_t)(pc + 3U), local_mem, &len, NULL);
            plc_vm_set_eno(eno_flags, false);
        }
        *pc_ptr = (uint16_t)(pc + 3U + len);
        break;
    }

    case PLC_EXT_CITIM: /* @ 0x08032a50 */
    {
        uint8_t  modes  = ob1[pc + 2U];
        uint8_t  len1   = 0U;
        uint32_t t0     = plc_oper_read_u32((uint8_t)(modes >> 4), (uint16_t)(pc + 3U),
                                            local_mem, &len1, NULL);
        uint16_t out_pc = (uint16_t)(pc + 3U + len1);
        uint8_t  len2   = 0U;
        if (en_in) {
            uint32_t dt = PLC_SYS_MS_TICK - t0;
            plc_oper_write_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL, dt);
            plc_vm_set_eno(eno_flags, true);
        } else {
            (void)plc_oper_read_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
            plc_vm_set_eno(eno_flags, false);
        }
        *pc_ptr = (uint16_t)(out_pc + len2);
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 14, 16..22, 103..105, 113..131, 139..140:
     * String/Table/Network/Port Operations (ITA/DTA/RTA/FND/SFND/SCPY/SCAT/SLEN/
     * LDS=/AS=/OS=/ITS/DTS/RTS/SSCPY/NETR/NETW/GPA/SPA)
     * ------------------------------------------------------------------------ */
    case PLC_EXT_ITA:    case PLC_EXT_DTA:    case PLC_EXT_RTA:
    case PLC_EXT_FND:    case PLC_EXT_SFND:   case PLC_EXT_CFND:
    case PLC_EXT_SFND_1C:case PLC_EXT_SFND_1D:
    case PLC_EXT_STI_6E: case PLC_EXT_STD_6F: case PLC_EXT_STR_70:
    case PLC_EXT_NETR:   case PLC_EXT_NETW:
    case PLC_EXT_STR_7A: case PLC_EXT_STD_7B: case PLC_EXT_STI_7C:
    case PLC_EXT_SCPY:   case PLC_EXT_SCAT:   case PLC_EXT_SLEN:
    case PLC_EXT_LDS_EQ: case PLC_EXT_AS_EQ:  case PLC_EXT_OS_EQ:
    case PLC_EXT_LDS_NE: case PLC_EXT_AS_NE:  case PLC_EXT_OS_NE:
    case PLC_EXT_ITS:    case PLC_EXT_DTS:    case PLC_EXT_RTS:
    case PLC_EXT_SSCPY:  case PLC_EXT_DIAG_LED:
    case PLC_EXT_GPA:    case PLC_EXT_SPA:
    {
        uint8_t modes = ob1[pc + 2U];
        uint8_t len1  = 0U;
        (void)plc_oper_read_u8((uint8_t)(modes >> 4), (uint16_t)(pc + 3U), local_mem, &len1, NULL);
        uint8_t len2  = 0U;
        (void)plc_oper_read_u8((uint8_t)(modes & 0x0FU), (uint16_t)(pc + 3U + len1), local_mem, &len2, NULL);
        *pc_ptr = (uint16_t)(pc + 3U + len1 + len2);
        plc_vm_set_eno(eno_flags, en_in);
        break;
    }

    /* ------------------------------------------------------------------------
     * Cases 1, 23, 29, 134, 136..138 (0x08, 0x1E, 0x24, 0x8D, 0x8F..0x91) @ 0x08035a40:
     * Reserved / Unsupported Extended Sub-Opcode -> sets error 0x8122
     * ------------------------------------------------------------------------ */
    default:
        PLC_VM_ERROR_REG = PLC_ERR_UNSUPPORTED_SUB_OPCODE;
        plc_vm_set_eno(eno_flags, false);
        *pc_ptr = (uint16_t)(pc + 2U);
        break;
    }
}

/* ============================================================================
 * 5. Main MC7/STL Bytecode Interpreter Loop
 *    (FUN_08021948 @ 0x08021948 .. 0x08036b1f)
 * ============================================================================
 */
void plc_vm_exec_block(uint16_t *pc_ptr,
                       uint16_t *logic_stack,
                       uint8_t *eno_flags,
                       uint8_t *local_mem,
                       uint16_t *edge_idx_ptr,
                       uint16_t *call_depth)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;

    /* Local FOR/NEXT loop stack and JMP/LBL state for this block invocation */
    uint16_t for_depth            = 0U;
    uint16_t for_pc_stack[8]      = { 0U };
    uint16_t for_indx_pc_stack[8] = { 0U };
    int16_t  for_final_stack[8]   = { 0 };
    uint8_t  for_indx_mode[8]     = { 0U };

    bool     jmp_pending          = false;
    uint8_t  jmp_target_lbl       = 0U;
    uint16_t lbl_pc_table[256]    = { 0U };
    uint8_t  lbl_seen_bitmap[32]  = { 0U };
    bool     scr_skip_active      = false;

    /* Main fetch-decode-execute loop (entry condition @ 0x08036b02) */
    while (true) {
        uint8_t opcode = ob1[*pc_ptr];

        /* Check block termination opcodes: MEND (0xED), RET (0xE9), RETI (0xE5) @ 0x08036b08 */
        if ((opcode == PLC_OP_MEND) || (opcode == PLC_OP_RET) || (opcode == PLC_OP_RETI)) {
            break;
        }

        /* Reload Hardware Watchdog IWDG->KR = 0xAAAA @ 0x080219f0 */
        PLC_IWDG_KR_REG = PLC_IWDG_RELOAD_KEY;

        /* If a forward JMP or false LSCR block is active, only LBL (0xD4) or SCRE/CSCRE is active */
        if (jmp_pending && (opcode != PLC_OP_LBL)) {
            /* Advance PC using parser/dispatcher or continue scanning to target LBL */
        }

        /* High-nibble primary dispatch (`opcode >> 4`, 0..15) via tbh [0x0804ad12] @ 0x08021a62 */
        switch (opcode >> 4) {

        /* ====================================================================
         * High Nibbles 0x0 .. 0x5 (@ 0x08021a66 .. 0x08021c53):
         * Compact 2-Byte Bit Logic (LD, LDN, A, AN, O, ON) on I, Q, M, S, SM, T, C
         * ==================================================================== */
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
            plc_vm_exec_compact_bit_logic(opcode, pc_ptr, logic_stack);
            break;

        /* ====================================================================
         * High Nibbles 0x6 .. 0x7 (@ 0x08021c54 .. 0x08021f8b):
         * Compact 2-Byte Output Coil Assignment (`=`) on I, Q, M, S, SM
         * ==================================================================== */
        case 0x6:
        case 0x7:
            plc_vm_exec_compact_coil(opcode, pc_ptr, logic_stack);
            break;

        /* ====================================================================
         * High Nibble 0x8 (@ 0x08021f8c .. 0x08022501):
         * 3-Byte / 5-Byte Extended Bit Logic (SM, V, L) & Immediate I/O (LDI..=I)
         * ==================================================================== */
        case 0x8:
            plc_vm_exec_ext_bit_logic(opcode, pc_ptr, local_mem, logic_stack);
            break;

        /* ====================================================================
         * High Nibbles 0x9 .. 0xA (@ 0x08022502 .. 0x08023961):
         *   - 0x90..0xA1: Word (even) and Byte (odd) Comparisons (==, <=, >=)
         *   - 0xA2..0xA4: DWord Comparisons (LDD=, LDD<=, LDD>=)
         *   - 0xA5:       WDR (Watchdog Reset)
         *   - 0xA6..0xA7: TON, TONR
         *   - 0xA8..0xA9: CTU, CTUD
         *   - 0xAA..0xAF: +I, +D, -I, -D, MUL, DIV
         * ==================================================================== */
        case 0x9:
        case 0xA:
            if (opcode <= PLC_OP_OB_GE) {
                if ((opcode & 1U) == 0U) {
                    /* Even 0x90..0xA0 @ 0x08022544 -> FUN_08016252 */
                    plc_vm_cmp_word(opcode, pc_ptr, local_mem, logic_stack);
                } else {
                    /* Odd 0x91..0xA1 @ 0x08022520 -> FUN_08015ca6 */
                    plc_vm_cmp_byte(opcode, pc_ptr, local_mem, logic_stack);
                }
            } else if (opcode <= PLC_OP_LDD_GE) {
                /* 0xA2..0xA4 @ 0x080225a4 -> FUN_08015e58 */
                plc_vm_cmp_dword_or_real(opcode, pc_ptr, local_mem, logic_stack);
            } else if (opcode == PLC_OP_WDR) {
                /* 0xA5: WDR @ 0x080238ea */
                if ((*logic_stack & 1U) != 0U) {
                    PLC_IWDG_KR_REG = PLC_IWDG_RELOAD_KEY;
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
                (*pc_ptr)++;
            } else if ((opcode == PLC_OP_TON) || (opcode == PLC_OP_TONR)) {
                /* 0xA6 (TON @ 0x08022e16), 0xA7 (TONR @ 0x080231a6) */
                plc_vm_exec_timer(opcode, pc_ptr, local_mem, logic_stack);
            } else if ((opcode == PLC_OP_CTU) || (opcode == PLC_OP_CTUD)) {
                /* 0xA8 (CTU @ 0x08023528), 0xA9 (CTUD @ 0x080236c2) */
                plc_vm_exec_counter(opcode, pc_ptr, local_mem, logic_stack);
            } else {
                /* 0xAA..0xAF: +I, +D, -I, -D, MUL, DIV @ 0x080225c2..0x08022e15 */
                plc_vm_exec_alu_int(opcode, pc_ptr, local_mem, eno_flags, logic_stack);
            }
            break;

        /* ====================================================================
         * High Nibble 0xB (@ 0x08023962 .. 0x08024791):
         *   - 0xB0:       PID Loop Controller (@ 0x08023fec)
         *   - 0xB1..0xB4: INCW, INCD, DECW, DECD (@ 0x0802396c / 0x08023b2a)
         *   - 0xB5..0xB7: AD=, AD<=, AD>= (@ 0x08023966)
         *   - 0xB8..0xBD: ANDW, ANDD, ORW, ORD, XORW, XORD (@ 0x08023d18 / 0x08023e60)
         *   - 0xBE..0xBF: INVW, INVD (@ 0x0802396c / 0x08023b2a)
         * ==================================================================== */
        case 0xB:
            if (opcode == PLC_OP_PID) {
                plc_vm_exec_pid(pc_ptr, local_mem, eno_flags, logic_stack);
            } else if ((opcode >= PLC_OP_AD_EQ) && (opcode <= PLC_OP_AD_GE)) {
                plc_vm_cmp_dword_or_real(opcode, pc_ptr, local_mem, logic_stack);
            } else if ((opcode >= PLC_OP_AND_W) && (opcode <= PLC_OP_XOR_D)) {
                plc_vm_exec_alu_int(opcode, pc_ptr, local_mem, eno_flags, logic_stack);
            } else {
                plc_vm_exec_unary_inc_dec_inv(opcode, pc_ptr, local_mem, eno_flags, logic_stack);
            }
            break;

        /* ====================================================================
         * High Nibble 0xC (@ 0x08024792 .. 0x0802688d):
         *   - 0xC0..0xC2: OD=, OD<=, OD>= (@ 0x08024796)
         *   - 0xC3:       PLS (@ 0x080247a8)
         *   - 0xC5:       SCRE (@ 0x08025c8a)
         *   - 0xC8..0xC9: BTI, ITB (@ 0x080259b0)
         *   - 0xCA..0xCC: LSCR, SCRT, CSCRE (@ 0x08025b88..0x08025d3f)
         *   - 0xCE..0xCF: ATCH, DTCH (@ 0x08025d40 / 0x0802638a)
         * ==================================================================== */
        case 0xC:
            if (opcode <= PLC_OP_OD_GE) {
                /* 0xC0..0xC2 @ 0x08024796 -> FUN_08015e58 */
                plc_vm_cmp_dword_or_real(opcode, pc_ptr, local_mem, logic_stack);
            } else if (opcode == PLC_OP_PLS) {
                /* 0xC3: PLS @ 0x080247a8 (4 bytes) */
                plc_vm_set_eno(eno_flags, (*logic_stack & 1U) != 0U);
                *pc_ptr = (uint16_t)(*pc_ptr + 4U);
            } else if ((opcode == PLC_OP_BTI) || (opcode == PLC_OP_ITB)) {
                /* 0xC8 (BTI), 0xC9 (ITB) @ 0x080259b0 */
                uint8_t  mode   = (uint8_t)(ob1[*pc_ptr + 1U] >> 4);
                uint16_t op_pc  = (uint16_t)(*pc_ptr + 2U);
                uint8_t  rlen   = 0U;
                uint16_t val16  = (uint16_t)plc_oper_read_i16(mode, op_pc, local_mem, &rlen, NULL);
                *pc_ptr = (uint16_t)(op_pc + rlen);

                if (((*logic_stack & 1U) == 0U) || (PLC_VM_ERROR_REG != 0U)) {
                    plc_vm_set_eno(eno_flags, false);
                    break;
                }
                if (opcode == PLC_OP_BTI) {
                    uint16_t d3 = (val16 >> 12) & 0x0FU;
                    uint16_t d2 = (val16 >> 8)  & 0x0FU;
                    uint16_t d1 = (val16 >> 4)  & 0x0FU;
                    uint16_t d0 =  val16        & 0x0FU;
                    if ((d3 > 9U) || (d2 > 9U) || (d1 > 9U) || (d0 > 9U)) {
                        PLC_SMB1 |= PLC_SM1_6_NOT_BCD;
                        plc_vm_set_eno(eno_flags, false);
                        break;
                    }
                    uint16_t bin = (uint16_t)(d3 * 1000U + d2 * 100U + d1 * 10U + d0);
                    uint8_t  wlen = 0U;
                    plc_oper_write_u16(mode, op_pc, local_mem, &wlen, NULL, bin);
                } else {
                    if ((int16_t)val16 < 0 || (int16_t)val16 > 9999) {
                        PLC_SMB1 |= PLC_SM1_1_OVERFLOW;
                        plc_vm_set_eno(eno_flags, false);
                        break;
                    }
                    uint16_t v   = val16;
                    uint16_t d3  = v / 1000U; v %= 1000U;
                    uint16_t d2  = v / 100U;  v %= 100U;
                    uint16_t d1  = v / 10U;
                    uint16_t d0  = v % 10U;
                    uint16_t bcd = (uint16_t)((d3 << 12) | (d2 << 8) | (d1 << 4) | d0);
                    uint8_t  wlen = 0U;
                    plc_oper_write_u16(mode, op_pc, local_mem, &wlen, NULL, bcd);
                }
                PLC_SMB1 &= (uint8_t)~(PLC_SM1_1_OVERFLOW | PLC_SM1_6_NOT_BCD);
                plc_vm_set_eno(eno_flags, true);
            } else if (opcode == PLC_OP_SCR) {
                /* 0xCA: LSCR Sx.y @ 0x08025b88 (2 bytes) */
                uint8_t s_bit = ob1[*pc_ptr + 1U];
                uint8_t s_val = (PLC_PHYS_BASE_S[s_bit >> 3] >> (s_bit & 7U)) & 1U;
                scr_skip_active = (s_val == 0U);
                *pc_ptr = (uint16_t)(*pc_ptr + 2U);
            } else if (opcode == PLC_OP_SCRT) {
                /* 0xCB: SCRT Sx.y @ 0x08025c10 (2 bytes) */
                uint8_t s_bit = ob1[*pc_ptr + 1U];
                if (!scr_skip_active && ((*logic_stack & 1U) != 0U)) {
                    PLC_PHYS_BASE_S[s_bit >> 3] |= (uint8_t)(1U << (s_bit & 7U));
                    scr_skip_active = true;
                }
                *pc_ptr = (uint16_t)(*pc_ptr + 2U);
            } else if ((opcode == PLC_OP_SCRE) || (opcode == PLC_OP_CSCRE)) {
                /* 0xC5 (SCRE @ 0x08025c8a), 0xCC (CSCRE @ 0x08025cd6) */
                scr_skip_active = false;
                (*pc_ptr)++;
            } else if (opcode == PLC_OP_ATCH) {
                /* 0xCE: ATCH INT_n, EVNT @ 0x08025d40 */
                uint8_t int_id = ob1[*pc_ptr + 2U];
                uint8_t rlen   = 0U;
                uint8_t evnt   = plc_oper_read_u8((uint8_t)(ob1[*pc_ptr + 1U] >> 4),
                                                  (uint16_t)(*pc_ptr + 3U), local_mem, &rlen, NULL);
                *pc_ptr = (uint16_t)(*pc_ptr + 3U + rlen);
                if (((*logic_stack & 1U) != 0U) && (evnt < PLC_MAX_INT_EVENTS)) {
                    PLC_INT_EVENT_MAP_TBL[evnt] = int_id;
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
            } else if (opcode == PLC_OP_DTCH) {
                /* 0xCF: DTCH EVNT @ 0x0802638a */
                uint8_t rlen = 0U;
                uint8_t evnt = plc_oper_read_u8((uint8_t)(ob1[*pc_ptr + 1U] >> 4),
                                                (uint16_t)(*pc_ptr + 2U), local_mem, &rlen, NULL);
                *pc_ptr = (uint16_t)(*pc_ptr + 2U + rlen);
                if (((*logic_stack & 1U) != 0U) && (evnt < PLC_MAX_INT_EVENTS)) {
                    PLC_INT_EVENT_MAP_TBL[evnt] = 0xFFU;
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
            } else {
                (*pc_ptr)++;
            }
            break;

        /* ====================================================================
         * High Nibble 0xD (@ 0x0802688e .. 0x08027d81):
         *   - 0xD0..0xD2: BMD, BMB, BMW (@ 0x08027730, 0x08026fde, 0x0802713c)
         *   - 0xD3..0xD4: JMP, LBL (@ 0x08026ddc, 0x08026e30)
         *   - 0xD5:       CALL without parameters (@ 0x08026892)
         *   - 0xD7:       FILL (@ 0x08027a44)
         *   - 0xD8..0xDA: MOVB, MOVW, MOVD (@ 0x08026914, 0x08026a8c, 0x08026b86)
         *   - 0xDB:       TODR / READ_RTC (@ 0x08026ecc)
         *   - 0xDC..0xDF: ALD, OLD, LPS, LRD (@ 0x08026d68 .. 0x08026ddb)
         * ==================================================================== */
        case 0xD:
            switch (opcode) {
            case PLC_OP_BMD: /* 0xD0 @ 0x08027730 */
            case PLC_OP_BMB: /* 0xD1 @ 0x08026fde */
            case PLC_OP_BMW: /* 0xD2 @ 0x0802713c */
            {
                uint8_t  modes   = ob1[*pc_ptr + 1U];
                uint8_t  n_mode  = (uint8_t)(ob1[*pc_ptr + 2U] >> 4);
                uint8_t  len1    = 0U;
                uint8_t *src_ptr = ((modes >> 4) == 0U)
                                   ? plc_oper_resolve_direct_ptr((uint16_t)(*pc_ptr + 3U), local_mem, &len1)
                                   : plc_oper_resolve_indirect_ptr((uint8_t)(modes >> 4),
                                                                   (uint16_t)(*pc_ptr + 3U), local_mem, &len1);
                uint8_t  len2    = 0U;
                uint8_t *dst_ptr = ((modes & 0x0FU) == 0U)
                                   ? plc_oper_resolve_direct_ptr((uint16_t)(*pc_ptr + 3U + len1), local_mem, &len2)
                                   : plc_oper_resolve_indirect_ptr((uint8_t)(modes & 0x0FU),
                                                                   (uint16_t)(*pc_ptr + 3U + len1), local_mem, &len2);
                uint8_t  len3    = 0U;
                uint8_t  count   = plc_oper_read_u8(n_mode,
                                                    (uint16_t)(*pc_ptr + 3U + len1 + len2),
                                                    local_mem, &len3, NULL);
                *pc_ptr = (uint16_t)(*pc_ptr + 3U + len1 + len2 + len3);
                if (((*logic_stack & 1U) != 0U) && (src_ptr != NULL) && (dst_ptr != NULL)) {
                    size_t elem_sz = (opcode == PLC_OP_BMB) ? 1U :
                                     (opcode == PLC_OP_BMW) ? 2U : 4U;
                    memmove(dst_ptr, src_ptr, (size_t)count * elem_sz);
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
                break;
            }

            case PLC_OP_JMP: /* 0xD3 @ 0x08026ddc: JMP n (3 bytes) */
            {
                uint8_t lbl_id = ob1[*pc_ptr + 2U];
                *pc_ptr = (uint16_t)(*pc_ptr + 3U);
                if ((*logic_stack & 1U) != 0U) {
                    plc_vm_set_eno(eno_flags, true);
                    if ((lbl_seen_bitmap[lbl_id >> 3] & (1U << (lbl_id & 7U))) != 0U) {
                        *pc_ptr = lbl_pc_table[lbl_id];
                    } else {
                        jmp_pending    = true;
                        jmp_target_lbl = lbl_id;
                    }
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
                break;
            }

            case PLC_OP_LBL: /* 0xD4 @ 0x08026e30: LBL n (3 bytes) */
            {
                uint8_t lbl_id = ob1[*pc_ptr + 2U];
                *pc_ptr = (uint16_t)(*pc_ptr + 3U);
                lbl_pc_table[lbl_id]         = *pc_ptr;
                lbl_seen_bitmap[lbl_id >> 3] |= (uint8_t)(1U << (lbl_id & 7U));
                if (jmp_pending && (jmp_target_lbl == lbl_id)) {
                    jmp_pending = false;
                }
                break;
            }

            case PLC_OP_CALL_NO_ARGS: /* 0xD5 @ 0x08026892: CALL SBR_n (2 bytes) */
            {
                uint8_t sbr_id = ob1[*pc_ptr + 1U];
                *pc_ptr = (uint16_t)(*pc_ptr + 2U);
                if (((*logic_stack & 1U) != 0U) && (*call_depth < PLC_MAX_CALL_DEPTH)) {
                    uint16_t callee_pc    = PLC_SBR_ENTRY_PC_TBL[sbr_id];
                    uint16_t callee_stack = 1U;
                    uint8_t  callee_eno   = PLC_ENO_FLAG_BIT;
                    uint8_t  callee_local[64];
                    memset(callee_local, 0, sizeof(callee_local));
                    (*call_depth)++;
                    plc_vm_exec_block(&callee_pc, &callee_stack, &callee_eno,
                                      callee_local, edge_idx_ptr, call_depth);
                    (*call_depth)--;
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
                break;
            }

            case PLC_OP_FILL: /* 0xD7 @ 0x08027a44: FILL IN_WORD, OUT_WORD, N_BYTE */
            {
                uint8_t  modes   = ob1[*pc_ptr + 1U];
                uint8_t  n_mode  = (uint8_t)(ob1[*pc_ptr + 2U] >> 4);
                uint8_t  len1    = 0U;
                uint16_t fill_w  = (uint16_t)plc_oper_read_i16((uint8_t)(modes >> 4),
                                                               (uint16_t)(*pc_ptr + 3U),
                                                               local_mem, &len1, NULL);
                uint8_t  len2    = 0U;
                uint8_t *dst_ptr = ((modes & 0x0FU) == 0U)
                                   ? plc_oper_resolve_direct_ptr((uint16_t)(*pc_ptr + 3U + len1), local_mem, &len2)
                                   : plc_oper_resolve_indirect_ptr((uint8_t)(modes & 0x0FU),
                                                                   (uint16_t)(*pc_ptr + 3U + len1), local_mem, &len2);
                uint8_t  len3    = 0U;
                uint8_t  count   = plc_oper_read_u8(n_mode,
                                                    (uint16_t)(*pc_ptr + 3U + len1 + len2),
                                                    local_mem, &len3, NULL);
                *pc_ptr = (uint16_t)(*pc_ptr + 3U + len1 + len2 + len3);
                if (((*logic_stack & 1U) != 0U) && (dst_ptr != NULL)) {
                    for (uint8_t i = 0U; i < count; i++) {
                        plc_be16_write(&dst_ptr[i * 2U], fill_w);
                    }
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
                break;
            }

            case PLC_OP_MOVB: /* 0xD8 @ 0x08026914 */
            {
                uint8_t  modes  = ob1[*pc_ptr + 1U];
                uint8_t  len1   = 0U;
                uint8_t  val8   = plc_oper_read_u8((uint8_t)(modes >> 4),
                                                   (uint16_t)(*pc_ptr + 2U), local_mem, &len1, NULL);
                uint16_t out_pc = (uint16_t)(*pc_ptr + 2U + len1);
                uint8_t  len2   = 0U;
                if (((*logic_stack & 1U) != 0U) && (PLC_VM_ERROR_REG == 0U)) {
                    plc_oper_write_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL, val8);
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    (void)plc_oper_read_u8((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
                    plc_vm_set_eno(eno_flags, false);
                }
                *pc_ptr = (uint16_t)(out_pc + len2);
                break;
            }

            case PLC_OP_MOVW: /* 0xD9 @ 0x08026a8c */
            {
                uint8_t  modes  = ob1[*pc_ptr + 1U];
                uint8_t  len1   = 0U;
                uint16_t val16  = (uint16_t)plc_oper_read_i16((uint8_t)(modes >> 4),
                                                              (uint16_t)(*pc_ptr + 2U),
                                                              local_mem, &len1, NULL);
                uint16_t out_pc = (uint16_t)(*pc_ptr + 2U + len1);
                uint8_t  len2   = 0U;
                if (((*logic_stack & 1U) != 0U) && (PLC_VM_ERROR_REG == 0U)) {
                    plc_oper_write_u16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL, val16);
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    (void)plc_oper_read_i16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
                    plc_vm_set_eno(eno_flags, false);
                }
                *pc_ptr = (uint16_t)(out_pc + len2);
                break;
            }

            case PLC_OP_MOVD: /* 0xDA @ 0x08026b86 */
            {
                uint8_t  modes  = ob1[*pc_ptr + 1U];
                uint8_t  len1   = 0U;
                uint32_t val32  = plc_oper_read_u32((uint8_t)(modes >> 4),
                                                    (uint16_t)(*pc_ptr + 2U),
                                                    local_mem, &len1, NULL);
                uint16_t out_pc = (uint16_t)(*pc_ptr + 2U + len1);
                uint8_t  len2   = 0U;
                if (((*logic_stack & 1U) != 0U) && (PLC_VM_ERROR_REG == 0U)) {
                    plc_oper_write_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL, val32);
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    (void)plc_oper_read_u32((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
                    plc_vm_set_eno(eno_flags, false);
                }
                *pc_ptr = (uint16_t)(out_pc + len2);
                break;
            }

            case PLC_OP_READ_RTC: /* 0xDB @ 0x08026ecc: TODR T */
            {
                uint8_t mode = (uint8_t)(ob1[*pc_ptr + 1U] >> 4);
                uint8_t len  = 0U;
                (void)plc_oper_read_u8(mode, (uint16_t)(*pc_ptr + 2U), local_mem, &len, NULL);
                *pc_ptr = (uint16_t)(*pc_ptr + 2U + len);
                plc_vm_set_eno(eno_flags, (*logic_stack & 1U) != 0U);
                break;
            }

            case PLC_OP_ALD: /* 0xDC @ 0x08026d68: AND Load */
            {
                uint16_t s = *logic_stack;
                *logic_stack = (uint16_t)((s >> 1) & ((s & 1U) | 0xFFFEU));
                (*pc_ptr)++;
                break;
            }

            case PLC_OP_OLD: /* 0xDD @ 0x08026d9c: OR Load */
            {
                uint16_t s = *logic_stack;
                *logic_stack = (uint16_t)((s >> 1) | (s & 1U));
                (*pc_ptr)++;
                break;
            }

            case PLC_OP_LPS: /* 0xDE @ 0x08026dac: Logic Push */
            {
                uint16_t s = *logic_stack;
                *logic_stack = (uint16_t)((s << 1) | (s & 1U));
                (*pc_ptr)++;
                break;
            }

            case PLC_OP_LRD: /* 0xDF @ 0x08026dbc: Logic Read */
            {
                uint16_t s = *logic_stack;
                *logic_stack = (uint16_t)((s & 0xFFFEU) | ((s >> 1) & 1U));
                (*pc_ptr)++;
                break;
            }

            default:
                (*pc_ptr)++;
                break;
            }
            break;

        /* ====================================================================
         * High Nibble 0xE (@ 0x08027d82 .. 0x080284eb):
         *   - 0xE0:       LPP (@ 0x08027f20)
         *   - 0xE1..0xE2: EU, ED (@ 0x08027d9e)
         *   - 0xE3:       NOT (@ 0x08027d86)
         *   - 0xE4, 0xEA, 0xEE: END, CRET, CRETI (@ 0x0802807c)
         *   - 0xE6..0xE7: ENI, DISI (@ 0x08028108 / 0x080280b6)
         *   - 0xE8:       NEXT (@ 0x0802800e)
         *   - 0xEB:       TODW / SET_RTC (@ 0x080281b0)
         *   - 0xEC:       STOP (@ 0x08028156)
         *   - 0xEF:       SWAP (@ 0x08027f3e)
         * ==================================================================== */
        case 0xE:
            switch (opcode) {
            case PLC_OP_LPP: /* 0xE0 @ 0x08027f20: Logic Pop */
                *logic_stack >>= 1;
                (*pc_ptr)++;
                break;

            case PLC_OP_EU:  /* 0xE1 @ 0x08027d9e: Edge Up (Rising Edge) */
            case PLC_OP_ED:  /* 0xE2 @ 0x08027d9e: Edge Down (Falling Edge) */
            {
                (*edge_idx_ptr)++;
                uint16_t e_idx = *edge_idx_ptr;
                if (e_idx > PLC_MAX_EDGE_DETECTORS) {
                    PLC_VM_ERROR_REG = PLC_ERR_EDGE_OVERFLOW;
                    (*pc_ptr)++;
                    break;
                }
                PLC_PHYS_BASE_EDGE_PC_TBL[e_idx] = *pc_ptr;
                uint16_t bit_idx  = (uint16_t)(e_idx - 1U);
                uint8_t  byte_off = (uint8_t)(bit_idx >> 2);
                uint8_t  shift    = (uint8_t)((bit_idx & 3U) * 2U);
                uint8_t  prev_bit = (PLC_PHYS_BASE_EDGE_BITMAP[byte_off] >> shift) & 1U;
                uint8_t  cur_bit  = (uint8_t)(*logic_stack & 1U);

                /* Update stored edge state bit in 0x2000BC00 */
                if (cur_bit != 0U) {
                    PLC_PHYS_BASE_EDGE_BITMAP[byte_off] |= (uint8_t)(1U << shift);
                } else {
                    PLC_PHYS_BASE_EDGE_BITMAP[byte_off] &= (uint8_t)~(1U << shift);
                }

                bool pulse = (opcode == PLC_OP_EU) ? ((cur_bit == 1U) && (prev_bit == 0U))
                                                   : ((cur_bit == 0U) && (prev_bit == 1U));
                if (pulse) {
                    *logic_stack |= 1U;
                } else {
                    *logic_stack &= (uint16_t)~1U;
                }
                (*pc_ptr)++;
                break;
            }

            case PLC_OP_NOT: /* 0xE3 @ 0x08027d86: Invert RLO */
                *eno_flags   ^= PLC_ENO_FLAG_BIT;
                *logic_stack ^= 1U;
                (*pc_ptr)++;
                break;

            case PLC_OP_END_COND: /* 0xE4 @ 0x0802807c: Conditional END */
            case PLC_OP_CRET:     /* 0xEA @ 0x0802807c: Conditional RET */
            case PLC_OP_CRETI:    /* 0xEE @ 0x0802807c: Conditional RETI */
                if ((*logic_stack & 1U) != 0U) {
                    return;
                }
                (*pc_ptr)++;
                break;

            case PLC_OP_ENI:  /* 0xE6 @ 0x08028108: Enable Interrupts */
                if ((*logic_stack & 1U) != 0U) {
                    PLC_SMB4 |= PLC_SM4_4_INT_ENABLED;
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
                (*pc_ptr)++;
                break;

            case PLC_OP_DISI: /* 0xE7 @ 0x080280b6: Disable Interrupts */
                if ((*logic_stack & 1U) != 0U) {
                    PLC_SMB4 &= (uint8_t)~PLC_SM4_4_INT_ENABLED;
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
                (*pc_ptr)++;
                break;

            case PLC_OP_NEXT: /* 0xE8 @ 0x0802800e: NEXT (End of FOR/NEXT Loop) */
                if (for_depth == 0U) {
                    PLC_COMPILE_ERR_REG = PLC_ERR_ILLEGAL_FOR_NEXT;
                    return;
                }
                {
                    uint16_t d        = (uint16_t)(for_depth - 1U);
                    uint8_t  rlen     = 0U;
                    int16_t  cur_indx = plc_oper_read_i16(for_indx_mode[d],
                                                          for_indx_pc_stack[d],
                                                          local_mem, &rlen, NULL);
                    cur_indx++;
                    uint8_t  wlen     = 0U;
                    plc_oper_write_u16(for_indx_mode[d], for_indx_pc_stack[d],
                                       local_mem, &wlen, NULL, (uint16_t)cur_indx);
                    if (cur_indx <= for_final_stack[d]) {
                        *pc_ptr = for_pc_stack[d];
                    } else {
                        for_depth--;
                        (*pc_ptr)++;
                    }
                }
                break;

            case PLC_OP_SET_RTC: /* 0xEB @ 0x080281b0: TODW T */
            {
                uint8_t mode = (uint8_t)(ob1[*pc_ptr + 1U] >> 4);
                uint8_t len  = 0U;
                (void)plc_oper_read_u8(mode, (uint16_t)(*pc_ptr + 2U), local_mem, &len, NULL);
                *pc_ptr = (uint16_t)(*pc_ptr + 2U + len);
                plc_vm_set_eno(eno_flags, (*logic_stack & 1U) != 0U);
                break;
            }

            case PLC_OP_STOP: /* 0xEC @ 0x08028156: STOP */
                if ((*logic_stack & 1U) != 0U) {
                    PLC_RUN_MODE_FLAG = 0U;
                    PLC_STOP_REQ_FLAG = 1U;
                    return;
                }
                (*pc_ptr)++;
                break;

            case PLC_OP_SWAP: /* 0xEF @ 0x08027f3e: SWAP Word Bytes */
            {
                uint8_t  mode  = (uint8_t)(ob1[*pc_ptr + 1U] >> 4);
                uint16_t op_pc = (uint16_t)(*pc_ptr + 2U);
                uint8_t  rlen  = 0U;
                uint16_t val16 = (uint16_t)plc_oper_read_i16(mode, op_pc, local_mem, &rlen, NULL);
                *pc_ptr = (uint16_t)(op_pc + rlen);
                if (((*logic_stack & 1U) != 0U) && (PLC_VM_ERROR_REG == 0U)) {
                    uint16_t swapped = (uint16_t)((val16 >> 8) | ((val16 & 0xFFU) << 8));
                    uint8_t  wlen    = 0U;
                    plc_oper_write_u16(mode, op_pc, local_mem, &wlen, NULL, swapped);
                    plc_vm_set_eno(eno_flags, true);
                } else {
                    plc_vm_set_eno(eno_flags, false);
                }
                break;
            }

            default:
                (*pc_ptr)++;
                break;
            }
            break;

        /* ====================================================================
         * High Nibble 0xF (@ 0x080284ec .. 0x08036957):
         *   - 0xF0:       NOP N (@ 0x08035cb6)
         *   - 0xF1..0xF3: S, R, XOR_BITS (@ 0x08035b94)
         *   - 0xF4..0xF6: INVB, INCB, DECB (@ 0x08035a46)
         *   - 0xF7:       AENO (@ 0x08035cfe)
         *   - 0xFA..0xFC: ATT, LIFO, FIFO (@ 0x08035d14 / 0x08036086)
         *   - 0xFE:       142-case Extended Instruction Prefix (@ 0x080284f0)
         * ==================================================================== */
        case 0xF:
            switch (opcode) {
            case PLC_OP_NOP: /* 0xF0 @ 0x08035cb6: NOP N (2 bytes) */
                *pc_ptr = (uint16_t)(*pc_ptr + 2U);
                break;

            case PLC_OP_SET_BITS:   /* 0xF1 @ 0x08035b94 */
            case PLC_OP_RESET_BITS: /* 0xF2 @ 0x08035b94 */
            case PLC_OP_XOR_BITS:   /* 0xF3 @ 0x08035b94 */
                plc_vm_exec_multi_bit_sr(opcode, pc_ptr, local_mem, logic_stack);
                break;

            case PLC_OP_INV_B: /* 0xF4 @ 0x08035a46 */
            case PLC_OP_INC_B: /* 0xF5 @ 0x08035a46 */
            case PLC_OP_DEC_B: /* 0xF6 @ 0x08035a46 */
                plc_vm_exec_unary_inc_dec_inv(opcode, pc_ptr, local_mem, eno_flags, logic_stack);
                break;

            case PLC_OP_AENO:  /* 0xF7 @ 0x08035cfe: AND ENO with RLO */
                if ((*eno_flags & PLC_ENO_FLAG_BIT) == 0U) {
                    *logic_stack &= (uint16_t)~1U;
                }
                (*pc_ptr)++;
                break;

            case PLC_OP_ATT:  /* 0xFA @ 0x08035d14: Add To Table */
            {
                uint8_t  modes   = ob1[*pc_ptr + 1U];
                uint8_t  len1    = 0U;
                uint16_t data_w  = (uint16_t)plc_oper_read_i16((uint8_t)(modes >> 4),
                                                               (uint16_t)(*pc_ptr + 2U),
                                                               local_mem, &len1, NULL);
                uint8_t  len2    = 0U;
                uint8_t *tbl_ptr = ((modes & 0x0FU) == 0U)
                                   ? plc_oper_resolve_direct_ptr((uint16_t)(*pc_ptr + 2U + len1), local_mem, &len2)
                                   : plc_oper_resolve_indirect_ptr((uint8_t)(modes & 0x0FU),
                                                                   (uint16_t)(*pc_ptr + 2U + len1), local_mem, &len2);
                *pc_ptr = (uint16_t)(*pc_ptr + 2U + len1 + len2);
                if (((*logic_stack & 1U) == 0U) || (tbl_ptr == NULL)) {
                    plc_vm_set_eno(eno_flags, false);
                    break;
                }
                uint16_t max_entries = plc_be16_read(&tbl_ptr[0]);
                uint16_t cur_entries = plc_be16_read(&tbl_ptr[2]);
                if (cur_entries >= max_entries) {
                    PLC_SMB1 |= PLC_SM1_4_TABLE_OVERFLOW;
                    plc_vm_set_eno(eno_flags, false);
                } else {
                    cur_entries++;
                    plc_be16_write(&tbl_ptr[2], cur_entries);
                    plc_be16_write(&tbl_ptr[2U + cur_entries * 2U], data_w);
                    PLC_SMB1 &= (uint8_t)~PLC_SM1_4_TABLE_OVERFLOW;
                    plc_vm_set_eno(eno_flags, true);
                }
                break;
            }

            case PLC_OP_LIFO: /* 0xFB @ 0x08036086: Last-In-First-Out Table Pop */
            case PLC_OP_FIFO: /* 0xFC @ 0x08036086: First-In-First-Out Table Pop */
            {
                uint8_t  modes   = ob1[*pc_ptr + 1U];
                uint8_t  len1    = 0U;
                uint8_t *tbl_ptr = ((modes >> 4) == 0U)
                                   ? plc_oper_resolve_direct_ptr((uint16_t)(*pc_ptr + 2U), local_mem, &len1)
                                   : plc_oper_resolve_indirect_ptr((uint8_t)(modes >> 4),
                                                                   (uint16_t)(*pc_ptr + 2U), local_mem, &len1);
                uint16_t out_pc  = (uint16_t)(*pc_ptr + 2U + len1);
                uint8_t  len2    = 0U;
                (void)plc_oper_read_i16((uint8_t)(modes & 0x0FU), out_pc, local_mem, &len2, NULL);
                *pc_ptr = (uint16_t)(out_pc + len2);

                if (((*logic_stack & 1U) == 0U) || (tbl_ptr == NULL)) {
                    plc_vm_set_eno(eno_flags, false);
                    break;
                }
                uint16_t cur_entries = plc_be16_read(&tbl_ptr[2]);
                if (cur_entries == 0U) {
                    PLC_SMB1 |= PLC_SM1_5_TABLE_EMPTY;
                    plc_vm_set_eno(eno_flags, false);
                } else {
                    uint16_t popped = 0U;
                    if (opcode == PLC_OP_LIFO) {
                        popped = plc_be16_read(&tbl_ptr[2U + cur_entries * 2U]);
                    } else {
                        popped = plc_be16_read(&tbl_ptr[4U]);
                        for (uint16_t i = 1U; i < cur_entries; i++) {
                            plc_be16_write(&tbl_ptr[2U + i * 2U],
                                           plc_be16_read(&tbl_ptr[4U + i * 2U]));
                        }
                    }
                    cur_entries--;
                    plc_be16_write(&tbl_ptr[2], cur_entries);
                    uint8_t wlen = 0U;
                    plc_oper_write_u16((uint8_t)(modes & 0x0FU), out_pc,
                                       local_mem, &wlen, NULL, popped);
                    PLC_SMB1 &= (uint8_t)~PLC_SM1_5_TABLE_EMPTY;
                    plc_vm_set_eno(eno_flags, true);
                }
                break;
            }

            case PLC_OP_EXT_FE: /* 0xFE @ 0x080284f0: 142-case extended instruction table */
                plc_vm_exec_ext_fe(pc_ptr, logic_stack, eno_flags, local_mem,
                                   edge_idx_ptr, call_depth, &for_depth,
                                   for_pc_stack, for_indx_pc_stack,
                                   for_final_stack, for_indx_mode);
                break;

            default:
                (*pc_ptr)++;
                break;
            }
            break;
        }

        /* ====================================================================
         * Post-Instruction Error Check & Pending Interrupt Dispatch
         * (@ 0x08036958 .. 0x08036b01)
         * ==================================================================== */
        if (PLC_VM_ERROR_REG != 0U) {
            /* Latch non-fatal runtime error bit SM4.3 (0x20005F2C bit 3) @ 0x08036ae0 */
            PLC_SMB4 |= PLC_SM4_3_RUN_TIME_ERROR;
        }

        if (((PLC_COMPILE_ERR_REG | PLC_FATAL_ERR_REG) != 0U) ||
            (PLC_STOP_REQ_FLAG != 0U)) {
            break;
        }
    }
}
