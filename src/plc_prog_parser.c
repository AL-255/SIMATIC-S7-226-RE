/**
 * @file plc_prog_parser.c
 * @brief SIMATIC S7-226 (APM32E103) User Program Parser, Virtual Pointer
 *        Translator, and Subroutine CALL Parameter Marshaler.
 *
 * Reconstructed from firmware flash region 0x08000e0e .. 0x08003e5b:
 *   - FUN_08000e0e (0x08000e0e .. 0x08000e49): plc_crc16_calc
 *   - FUN_08000e4a (0x08000e4a .. 0x08000f55): plc_vptr_to_phys
 *   - FUN_08000f56 (0x08000f56 .. 0x0800106f): plc_vptr_to_phys_local
 *   - FUN_08001070 (0x08001070 .. 0x0800133b): plc_prog_scan_and_validate
 *   - FUN_0800133c (0x0800133c .. 0x080027e9): plc_call_marshal_in_params
 *   - FUN_080027ea (0x080027ea .. 0x08003c21): plc_call_unmarshal_out_params
 *   - FUN_08003c22 (0x08003c22 .. 0x08003e5b): plc_call_skip_params
 */

#include "plc_vm.h"

/* ============================================================================
 * Firmware Global Tables & Runtime Pointers Referenced by 0x08000e0e..0x08003e5b
 * ============================================================================
 */

/* Configurable area base pointer table at 0x20000250 .. 0x20000278 */
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

/* Program scanner state and block entry/exit tables in SRAM */
#define PLC_COMPILE_ERR_REG         (*(volatile uint16_t *)0x200001ECU)
#define PLC_FATAL_ERR_REG           (*(volatile uint16_t *)0x20000188U)
#define PLC_SCAN_CUR_PC             (*(volatile uint16_t *)0x200004D0U)
#define PLC_OB1_TOTAL_LEN           (*(volatile uint16_t *)0x20000516U)
#define PLC_SCAN_BLOCK_ACTIVE       (*(volatile uint8_t  *)0x2000051CU)
#define PLC_SCAN_BLOCK_TYPE         (*(volatile uint8_t  *)0x20000495U)
#define PLC_SCAN_STEP_COUNTER       (*(volatile uint16_t *)0x2000051EU)
#define PLC_SCAN_FOR_COUNTER        (*(volatile uint16_t *)0x20000520U)
#define PLC_SCAN_STATE_TAG          (*(volatile uint8_t  *)0x20000522U)
#define PLC_OB1_MEND_PC             (*(volatile uint16_t *)0x20000524U)

/* Subroutine (SBR_0..SBR_127) and Interrupt (INT_0..INT_127) PC offset tables */
#define PLC_SBR_START_PC_TBL        ((volatile uint16_t *)0x20004640U)
#define PLC_INT_START_PC_TBL        ((volatile uint16_t *)0x20004740U)
#define PLC_SBR_END_PC_TBL          ((volatile uint16_t *)0x20004840U)
#define PLC_INT_END_PC_TBL          ((volatile uint16_t *)0x20004940U)
#define PLC_SBR_STEP_BASE_TBL       ((volatile uint16_t *)0x20004A40U)
#define PLC_INT_STEP_BASE_TBL       ((volatile uint16_t *)0x20004B40U)
#define PLC_SBR_FOR_BASE_TBL        ((volatile uint16_t *)0x20004C40U)
#define PLC_INT_FOR_BASE_TBL        ((volatile uint16_t *)0x20004D40U)

/* Runtime active SBR and INT entry tables populated before VM execution */
#define PLC_RUNTIME_SBR_PC_TBL      ((volatile uint16_t *)0x20006020U)
#define PLC_RUNTIME_INT_PC_TBL      ((volatile uint16_t *)0x20005EFCU)

/* CRC-16 lookup tables in Flash (.rodata at 0x0804A548 and 0x0804A648) */
#define PLC_CRC16_HI_TBL            ((const uint8_t *)0x0804A548U)
#define PLC_CRC16_LO_TBL            ((const uint8_t *)0x0804A648U)

/* External block scanner helpers in firmware */
extern void FUN_08013d96(void); /* OB1 main program scanner (0x08013d96) */
extern void FUN_08037928(uint32_t start_pc, uint16_t end_pc, int16_t step_base,
                         uint16_t for_base, void *ai_base, void *ctx); /* SBR scanner (0x08037928) */
extern void FUN_08015402(uint32_t start_pc, uint16_t end_pc, int16_t step_base,
                         uint16_t for_base, void *ai_base, void *ctx); /* INT scanner (0x08015402) */

/* ============================================================================
 * 1. CRC-16 Calculator (FUN_08000e0e @ 0x08000e0e .. 0x08000e49)
 * ============================================================================
 */
uint16_t plc_crc16_calc(uint8_t *buf, uint16_t len)
{
    uint8_t crc_hi = 0xFFU;
    uint8_t crc_lo = 0xFFU;
    uint8_t count  = (uint8_t)(len & 0xFFU);

    while (count != 0U) {
        count--;
        PLC_IWDG_KR_REG = PLC_IWDG_RELOAD_KEY;

        uint8_t idx = (*buf++) ^ crc_hi;
        crc_hi = PLC_CRC16_HI_TBL[idx] ^ crc_lo;
        crc_lo = PLC_CRC16_LO_TBL[idx];
    }

    return (uint16_t)((uint16_t)crc_lo | ((uint16_t)crc_hi << 8));
}

/* ============================================================================
 * 2. Virtual Pointer to Physical Address Translator
 *    (FUN_08000e4a @ 0x08000e4a .. 0x08000f55)
 * ============================================================================
 * Translates a 32-bit S7-200 virtual address pointer ((area_id << 24) | offset)
 * into a physical SRAM/Flash pointer for global memory areas (I, Q, M, S, SM,
 * OB1, V, T, C, AI, AQ). Sets PLC_VM_ERROR_REG = 0x8091 (range error) if the
 * area or byte offset is out of range.
 */
uint8_t *plc_vptr_to_phys(uint32_t vptr)
{
    uint32_t area_id = (vptr & 0x0FFFFFFFU) >> 24;
    uint32_t offset  = vptr & 0x00FFFFFFU;

    switch (area_id) {
    case PLC_AREA_I:   /* 0x00: Input Image IB0..IB15 (max < 0x10) */
        if (vptr < 0x00000010U) {
            return PLC_PTR_BASE_I + offset;
        }
        break;

    case PLC_AREA_Q:   /* 0x01: Output Image QB0..QB15 (max < 0x01000010) */
        if (vptr < 0x01000010U) {
            return PLC_PTR_BASE_Q + offset;
        }
        break;

    case PLC_AREA_M:   /* 0x02: Bit Memory MB0..MB31 (max < 0x02000020) */
        if (vptr < 0x02000020U) {
            return PLC_PTR_BASE_M + offset;
        }
        break;

    case PLC_AREA_S:   /* 0x03: SCR Memory SB0..SB31 (max < 0x03000020) */
        if (vptr < 0x03000020U) {
            return PLC_PTR_BASE_S + offset;
        }
        break;

    case PLC_AREA_SM:  /* 0x04: Special Memory SMB0..SMB549 (max < 0x04000226) */
        if (vptr < 0x04000226U) {
            return PLC_PTR_BASE_SM + offset;
        }
        break;

    case PLC_AREA_OB1: /* 0x05: Program Flash 0x0806D800 (max < 0x05010000) */
        if (vptr < 0x05010000U) {
            return PLC_PTR_BASE_OB1 + offset;
        }
        break;

    case PLC_AREA_V:   /* 0x08: Variable Memory VB0..VB10239 (max < 0x08002800) */
        if (vptr < 0x08002800U) {
            return PLC_PTR_BASE_V + offset;
        }
        break;

    case PLC_AREA_T:   /* 0x09: Timer Current Words T0..T255 (max < 0x09000200) */
        if (vptr < 0x09000200U) {
            return PLC_PTR_BASE_T + offset;
        }
        break;

    case PLC_AREA_C:   /* 0x0A: Counter Current Words C0..C255 (max < 0x0A000200) */
        if (vptr < 0x0A000200U) {
            return PLC_PTR_BASE_C + offset;
        }
        break;

    case PLC_AREA_AI:  /* 0x0C: Analog Inputs AIW0..AIW62 (max < 0x0C00003F) */
        if (vptr < 0x0C00003FU) {
            return PLC_PTR_BASE_AI + offset;
        }
        break;

    case PLC_AREA_AQ:  /* 0x0D: Analog Outputs AQW0..AQW62 (max < 0x0D00003F) */
        if (vptr < 0x0D00003FU) {
            return PLC_PTR_BASE_AQ + offset;
        }
        break;

    default:
        break;
    }

    /* Out-of-bounds indirect address: set non-fatal error 0x8091 */
    PLC_VM_ERROR_REG = PLC_ERR_INDIRECT_ADDR_RANGE;
    return NULL;
}

/* ============================================================================
 * 3. Virtual Pointer Translator with Local Frame Support
 *    (FUN_08000f56 @ 0x08000f56 .. 0x0800106f)
 * ============================================================================
 * Identical to plc_vptr_to_phys, but additionally supports area 0x0E (Local
 * Memory LB0..LB63) resolved relative to the caller-supplied `local_base`.
 */
uint8_t *plc_vptr_to_phys_local(uint32_t vptr, uint8_t *local_base)
{
    uint32_t area_id = (vptr & 0x0FFFFFFFU) >> 24;

    if (area_id == PLC_AREA_L) {
        if (vptr > 0x0E00003FU) {
            PLC_VM_ERROR_REG = PLC_ERR_INDIRECT_ADDR_RANGE;
            return NULL;
        }
        return local_base + (vptr & 0xFFU);
    }

    return plc_vptr_to_phys(vptr);
}

/* ============================================================================
 * 4. User Program Block Scanner & Validator
 *    (FUN_08001070 @ 0x08001070 .. 0x0800133b)
 * ============================================================================
 * Scans the user program in OB1 Flash (0x0806D800):
 *   1. Clears the SBR_0..127 and INT_0..127 start/end PC offset tables.
 *   2. Invokes FUN_08013d96() to validate the main OB1 body up to MEND
 *      (0xED 0xD6) and records PLC_OB1_MEND_PC = PLC_SCAN_CUR_PC.
 *   3. Loops over all following Subroutine blocks (prefixed by 0xD6 after MEND
 *      or previous RET, terminated by RET 0xE9 0xCD), recording each SBR_n
 *      entry PC in PLC_SBR_START_PC_TBL[sbr_id] and validating its instructions
 *      via FUN_08037928().
 *   4. Loops over all following Interrupt blocks (prefixed by 0xCD after last
 *      SBR or 0xB5 after previous RETI, terminated by RETI 0xE5 0xB5),
 *      recording each INT_n entry PC in PLC_INT_START_PC_TBL[int_id] and
 *      validating its instructions via FUN_08015402().
 */
void plc_prog_scan_and_validate(void)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;

    /* Abort immediately if a fatal or compile error is already latched */
    if (((uint32_t)PLC_COMPILE_ERR_REG | (uint32_t)PLC_FATAL_ERR_REG) != 0U) {
        return;
    }

    /* 1. Initialize SBR and INT start/end offset tables (128 entries each) */
    for (uint32_t i = 0; i < 128U; i++) {
        PLC_SBR_START_PC_TBL[i] = 0U;
        PLC_INT_START_PC_TBL[i] = 0U;
        PLC_SBR_END_PC_TBL[i]   = 0U;
        PLC_INT_END_PC_TBL[i]   = 0U;
    }

    /* 2. Scan and validate OB1 main program block (0x0000 .. MEND 0xED 0xD6) */
    PLC_SCAN_STATE_TAG    = 0x2BU;
    PLC_SCAN_BLOCK_ACTIVE = 1U;
    PLC_SCAN_BLOCK_TYPE   = 2U;
    FUN_08013d96();

    if (PLC_COMPILE_ERR_REG != 0U) {
        return;
    }

    PLC_OB1_MEND_PC = PLC_SCAN_CUR_PC;
    uint16_t pc = (uint16_t)(PLC_SCAN_CUR_PC + 1U);
    uint32_t local_ctx[2] = { 0U, 0U };

    if ((int32_t)pc < ((int32_t)PLC_OB1_TOTAL_LEN - 2)) {
        PLC_SCAN_BLOCK_TYPE   = 0U;
        PLC_SCAN_BLOCK_ACTIVE = 0U;

        /* 3. Scan Subroutine blocks (SBR_n): marker byte 0xD6 (-0x2A) */
        if (ob1[pc] == 0xD6U) {
            do {
                pc++;
                uint8_t has_params = ob1[pc];
                pc++;
                uint8_t sbr_id = ob1[pc];
                if (sbr_id > 0x7FU) {
                    return;
                }

                /* Skip 9-byte subroutine parameter signature block if present */
                pc = (uint16_t)(pc + (uint16_t)has_params * 9U + 1U);
                PLC_SBR_START_PC_TBL[sbr_id] = pc;

                uint16_t for_base  = PLC_SCAN_FOR_COUNTER;
                int16_t  step_base = (int16_t)(PLC_SCAN_STEP_COUNTER + 2);
                PLC_SBR_FOR_BASE_TBL[sbr_id]  = for_base;
                PLC_SBR_STEP_BASE_TBL[sbr_id] = (uint16_t)step_base;

                PLC_SCAN_BLOCK_ACTIVE = 1U;
                PLC_SCAN_BLOCK_TYPE   = 2U;

                /* Validate SBR_n body if not an immediately empty RET (0xE9) */
                if (ob1[pc] != (uint8_t)PLC_OP_RET) {
                    FUN_08037928(pc,
                                 PLC_SBR_END_PC_TBL[sbr_id],
                                 step_base,
                                 for_base,
                                 (void *)PLC_PHYS_BASE_AI,
                                 local_ctx);
                } else {
                    PLC_SCAN_STEP_COUNTER = (uint16_t)step_base;
                    PLC_SCAN_CUR_PC       = pc;
                }

                if (has_params == 1U) {
                    PLC_SCAN_STEP_COUNTER++;
                }

                PLC_SBR_END_PC_TBL[sbr_id] = PLC_SCAN_CUR_PC;
                if (PLC_COMPILE_ERR_REG != 0U) {
                    return;
                }

                pc = (uint16_t)(PLC_SCAN_CUR_PC + 1U);
            } while ((ob1[pc] == 0xD6U) &&
                     ((int32_t)pc < ((int32_t)PLC_OB1_TOTAL_LEN - 2)));
        }

        /* 4. Scan Interrupt blocks (INT_n): marker byte 0xCD (-0x33) */
        if (((int32_t)pc < ((int32_t)PLC_OB1_TOTAL_LEN - 2)) &&
            (ob1[pc] == 0xCDU)) {
            PLC_SCAN_BLOCK_TYPE   = 0U;
            PLC_SCAN_BLOCK_ACTIVE = 0U;

            do {
                pc++;
                uint8_t has_params = ob1[pc];
                pc++;
                uint8_t int_id = ob1[pc];
                if (int_id > 0x7FU) {
                    return;
                }

                pc = (uint16_t)(pc + (uint16_t)has_params * 9U + 1U);
                PLC_INT_START_PC_TBL[int_id] = pc;

                uint16_t for_base  = PLC_SCAN_FOR_COUNTER;
                int16_t  step_base = (int16_t)(PLC_SCAN_STEP_COUNTER + 2);
                PLC_INT_FOR_BASE_TBL[int_id]  = for_base;
                PLC_INT_STEP_BASE_TBL[int_id] = (uint16_t)step_base;

                PLC_SCAN_BLOCK_ACTIVE = 1U;
                PLC_SCAN_BLOCK_TYPE   = 2U;

                /* Validate INT_n body if not an immediately empty RETI (0xE5) */
                if (ob1[pc] != (uint8_t)PLC_OP_RETI) {
                    FUN_08015402(pc,
                                 PLC_INT_END_PC_TBL[int_id],
                                 step_base,
                                 for_base,
                                 (void *)PLC_PHYS_BASE_AI,
                                 local_ctx);
                } else {
                    PLC_SCAN_STEP_COUNTER = (uint16_t)step_base;
                    PLC_SCAN_CUR_PC       = pc;
                }

                if (has_params == 1U) {
                    PLC_SCAN_STEP_COUNTER++;
                }

                PLC_INT_END_PC_TBL[int_id] = PLC_SCAN_CUR_PC;
                if (PLC_COMPILE_ERR_REG != 0U) {
                    return;
                }

                pc = (uint16_t)(PLC_SCAN_CUR_PC + 1U);
            } while ((ob1[pc] == 0xCDU) &&
                     ((int32_t)pc < ((int32_t)PLC_OB1_TOTAL_LEN - 2)));
        }
    }

    PLC_SCAN_BLOCK_ACTIVE = 0U;
    PLC_SCAN_BLOCK_TYPE   = 0U;
}

/* ============================================================================
 * Internal Helper: Resolve a Direct or Extended Direct Address in CALL Params
 * ============================================================================
 * In CALL parameter blocks (FUN_0800133c / FUN_080027ea), direct memory
 * operands are encoded either as:
 *   - 2 bytes: [area_hi, off_lo] -> vptr = ((area_hi >> 4) << 24) | ((area_hi & 0x0F) << 8) | off_lo
 *   - 4 bytes (when ob1[pc] == 0xF0): [0xF0, area_hi, off_hi, off_lo]
 *     -> vptr = ((area_hi >> 4) << 24) | (off_hi << 8) | off_lo
 */
static uint8_t *plc_call_resolve_param_ptr(uint16_t pc,
                                           uint8_t *caller_local,
                                           uint16_t *out_advance)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint8_t b0 = ob1[pc];
    uint32_t vptr;

    if (b0 == 0xF0U) {
        uint8_t area_byte = ob1[pc + 1U];
        uint32_t hi       = ((uint32_t)(area_byte & 0x0FU)) |
                            (((uint32_t)(area_byte >> 4)) << 16);
        uint32_t off16    = ((uint32_t)ob1[pc + 2U] << 8) | (uint32_t)ob1[pc + 3U];
        vptr              = (hi << 8) + off16;
        *out_advance      = 6U; /* 2B descriptor + 4B extended address */
    } else {
        uint32_t hi  = ((uint32_t)(b0 & 0x0FU)) | (((uint32_t)(b0 >> 4)) << 16);
        vptr         = (hi << 8) | (uint32_t)ob1[pc + 1U];
        *out_advance = 4U; /* 2B descriptor + 2B direct address */
    }

    if ((vptr >> 24) == PLC_AREA_L) {
        if (vptr > 0x0E00003FU) {
            PLC_VM_ERROR_REG = PLC_ERR_INDIRECT_ADDR_RANGE;
            return NULL;
        }
        return caller_local + (vptr & 0xFFU);
    }

    return plc_vptr_to_phys(vptr);
}

/* ============================================================================
 * 5. Marshal Subroutine CALL Input / In-Out Parameters into Callee Local Frame
 *    (FUN_0800133c @ 0x0800133c .. 0x080027e9)
 * ============================================================================
 * Decodes each parameter descriptor following a `CALL SBR_n` with parameters
 * (`0xFE 0x23`), copies IN (`0x40xx..0x6A00`) and IN_OUT (`0x80xx..0x9Fxx`)
 * values from the caller context into `callee_local` (LB0..LB63), and skips
 * OUT (`0xC0xx..0xDFxx`) descriptors while advancing the callee LB offset.
 */
uint16_t plc_call_marshal_in_params(uint16_t caller_pc,
                                    uint8_t *caller_local,
                                    uint8_t *callee_local,
                                    uint16_t *logic_stack)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint8_t param_hdr_count = ob1[caller_pc + 2U];
    uint16_t pc = (uint16_t)(caller_pc + 7U);

    uint8_t lb_byte = 0U;
    uint8_t lb_bit  = 0U;

    for (uint16_t idx = 0U; idx <= (uint16_t)(param_hdr_count - 2U); idx++) {
        uint16_t desc = ((uint16_t)ob1[pc] << 8) | (uint16_t)ob1[pc + 1U];

        /* Compact BOOL logic-stack parameter descriptor: 0x6A [bit_off+0x81] [bit_off] */
        if ((ob1[pc] == 0x6AU) &&
            (ob1[pc + 1U] == (uint8_t)(ob1[pc + 2U] + 0x81U))) {
            desc = 0x6A00U;
        }

        /* Align to next whole byte in callee LB frame for non-boolean parameters */
        bool is_bool_param = (desc == 0x6A00U) ||
                             ((desc & 0x3FFFU) == 0x0001U);
        if (!is_bool_param && (lb_bit != 0U)) {
            lb_byte++;
            lb_bit = 0U;
        }

        uint16_t adv = 2U;

        switch (desc) {
        /* --------------------------------------------------------------------
         * A. Boolean Bit Parameters (0x6A00 = RLO stack bit, 0x4001/0x8001/0xC001)
         * -------------------------------------------------------------------- */
        case 0x6A00U: /* @ 0x0800254a: IN BOOL from caller logic_stack bit 0 */
        {
            uint8_t dst_byte = ob1[pc + 3U];
            uint8_t dst_bit  = ob1[pc + 2U] & 0x07U;
            if ((*logic_stack & 1U) != 0U) {
                callee_local[dst_byte] |= (uint8_t)(1U << dst_bit);
            } else {
                callee_local[dst_byte] &= (uint8_t)~(1U << dst_bit);
            }
            adv = 4U;
            break;
        }

        case 0x4001U: /* @ 0x080015e6: IN BOOL direct bit operand */
        case 0x8001U: /* @ 0x080015e6: IN_OUT BOOL direct bit operand */
        {
            uint8_t bit_num = ob1[pc + 2U] & 0x07U;
            uint8_t *src_ptr = plc_call_resolve_param_ptr((uint16_t)(pc + 3U), caller_local, &adv);
            adv = (uint16_t)(adv + 1U);
            uint8_t bit_val = (src_ptr != NULL) ? ((*src_ptr >> bit_num) & 1U) : 0U;
            if (bit_val != 0U) {
                callee_local[lb_byte] |= (uint8_t)(1U << lb_bit);
            } else {
                callee_local[lb_byte] &= (uint8_t)~(1U << lb_bit);
            }
            lb_bit++;
            if (lb_bit > 7U) {
                lb_bit = 0U;
                lb_byte++;
            }
            break;
        }

        case 0xC001U: /* @ 0x08002586: OUT BOOL direct bit operand (skip on entry) */
            adv = (ob1[pc + 3U] == 0xF0U) ? 7U : 5U;
            lb_bit++;
            if (lb_bit > 7U) {
                lb_bit = 0U;
                lb_byte++;
            }
            break;

        /* --------------------------------------------------------------------
         * B. 8-Bit Byte Parameters (IN / IN_OUT / OUT)
         * -------------------------------------------------------------------- */
        case 0x4281U: /* @ 0x0800196a: IN BYTE 1-byte immediate constant */
        case 0x4A81U:
        case 0x5281U:
            callee_local[lb_byte] = ob1[pc + 2U];
            lb_byte += 1U;
            adv = 3U;
            break;

        case 0x4081U: /* @ 0x08001820: IN / IN_OUT BYTE direct memory */
        case 0x8081U:
        case 0x4481U: /* @ 0x08001e86: IN / IN_OUT BYTE indirect memory (*VDx / *LDx) */
        case 0x8481U:
        {
            uint8_t *src_ptr = plc_call_resolve_param_ptr((uint16_t)(pc + 2U), caller_local, &adv);
            if (((desc & 0x0400U) != 0U) && (src_ptr != NULL)) {
                uint32_t vptr = ((uint32_t)src_ptr[0] << 24) |
                                ((uint32_t)src_ptr[1] << 16) |
                                ((uint32_t)src_ptr[2] << 8)  |
                                 (uint32_t)src_ptr[3];
                src_ptr = plc_vptr_to_phys_local(vptr, caller_local);
            }
            callee_local[lb_byte] = (src_ptr != NULL) ? *src_ptr : 0U;
            lb_byte += 1U;
            break;
        }

        case 0x4681U: /* @ 0x0800198c: IN / IN_OUT BYTE from AC0..AC3 */
        case 0x4E81U:
        case 0x5681U:
        case 0x5E81U:
        case 0x8681U:
        case 0x8E81U:
        case 0x9681U:
        case 0x9E81U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            callee_local[lb_byte] = (uint8_t)(PLC_ACCUM_BASE[ac_idx] & 0xFFU);
            lb_byte += 1U;
            adv = 2U;
            break;
        }

        case 0x4C81U: /* @ 0x08001f92: IN / IN_OUT BYTE indirect via *AC1..*AC3 */
        case 0x5481U:
        case 0x5C81U:
        case 0x8C81U:
        case 0x9481U:
        case 0x9C81U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint8_t *src_ptr = plc_vptr_to_phys_local(PLC_ACCUM_BASE[ac_idx], caller_local);
            callee_local[lb_byte] = (src_ptr != NULL) ? *src_ptr : 0U;
            lb_byte += 1U;
            adv = 2U;
            break;
        }

        case 0xC081U: /* @ 0x08002598: OUT BYTE direct/indirect memory (skip on entry) */
        case 0xC481U:
            adv = (ob1[pc + 2U] == 0xF0U) ? 6U : 4U;
            lb_byte += 1U;
            break;

        case 0xC681U: /* @ 0x080025b2: OUT BYTE via AC0..AC3 or *AC1..*AC3 */
        case 0xCC81U:
        case 0xCE81U:
        case 0xD481U:
        case 0xD681U:
        case 0xDC81U:
        case 0xDE81U:
            adv = 2U;
            lb_byte += 1U;
            break;

        /* --------------------------------------------------------------------
         * C. 16-Bit Word / Int Parameters (IN / IN_OUT / OUT)
         * -------------------------------------------------------------------- */
        case 0x4301U: /* @ 0x08001bf8: IN WORD 2-byte immediate constant */
        case 0x4B01U:
        case 0x5301U:
            callee_local[lb_byte]      = ob1[pc + 2U];
            callee_local[lb_byte + 1U] = ob1[pc + 3U];
            lb_byte += 2U;
            adv = 4U;
            break;

        case 0x4101U: /* @ 0x080019f8: IN / IN_OUT WORD direct memory */
        case 0x8101U:
        case 0x4501U: /* @ 0x08001fd0: IN / IN_OUT WORD indirect memory (*VDx / *LDx) */
        case 0x8501U:
        {
            uint8_t *src_ptr = plc_call_resolve_param_ptr((uint16_t)(pc + 2U), caller_local, &adv);
            if (((desc & 0x0400U) != 0U) && (src_ptr != NULL)) {
                uint32_t vptr = ((uint32_t)src_ptr[0] << 24) |
                                ((uint32_t)src_ptr[1] << 16) |
                                ((uint32_t)src_ptr[2] << 8)  |
                                 (uint32_t)src_ptr[3];
                src_ptr = plc_vptr_to_phys_local(vptr, caller_local);
            }
            if (src_ptr != NULL) {
                callee_local[lb_byte]      = src_ptr[0];
                callee_local[lb_byte + 1U] = src_ptr[1];
            }
            lb_byte += 2U;
            break;
        }

        case 0x4701U: /* @ 0x080019be: IN / IN_OUT WORD from AC0..AC3 */
        case 0x4F01U:
        case 0x5701U:
        case 0x5F01U:
        case 0x8701U:
        case 0x8F01U:
        case 0x9701U:
        case 0x9F01U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint32_t val   = PLC_ACCUM_BASE[ac_idx];
            callee_local[lb_byte]      = (uint8_t)((val >> 8) & 0xFFU);
            callee_local[lb_byte + 1U] = (uint8_t)(val & 0xFFU);
            lb_byte += 2U;
            adv = 2U;
            break;
        }

        case 0x4D01U: /* @ 0x08002148: IN / IN_OUT WORD indirect via *AC1..*AC3 */
        case 0x5501U:
        case 0x5D01U:
        case 0x8D01U:
        case 0x9501U:
        case 0x9D01U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint8_t *src_ptr = plc_vptr_to_phys_local(PLC_ACCUM_BASE[ac_idx], caller_local);
            if (src_ptr != NULL) {
                callee_local[lb_byte]      = src_ptr[0];
                callee_local[lb_byte + 1U] = src_ptr[1];
            }
            lb_byte += 2U;
            adv = 2U;
            break;
        }

        case 0xC101U: /* @ 0x080025d4: OUT WORD direct/indirect memory (skip on entry) */
        case 0xC501U:
            adv = (ob1[pc + 2U] == 0xF0U) ? 6U : 4U;
            lb_byte += 2U;
            break;

        case 0xC701U: /* @ 0x080025ac: OUT WORD via AC0..AC3 or *AC1..*AC3 */
        case 0xCD01U:
        case 0xCF01U:
        case 0xD501U:
        case 0xD701U:
        case 0xDD01U:
        case 0xDF01U:
            adv = 2U;
            lb_byte += 2U;
            break;

        /* --------------------------------------------------------------------
         * D. 32-Bit DWord / Real / Pointer Parameters (IN / IN_OUT / OUT)
         * -------------------------------------------------------------------- */
        case 0x4381U: /* @ 0x08001c1e: IN DWORD/REAL/PTR 4-byte immediate constant */
        case 0x4B81U:
        case 0x5381U:
        case 0x5B81U: /* @ 0x0800234a: IN STRING/ADDR pointer */
        case 0x6381U:
            callee_local[lb_byte]      = ob1[pc + 2U];
            callee_local[lb_byte + 1U] = ob1[pc + 3U];
            callee_local[lb_byte + 2U] = ob1[pc + 4U];
            callee_local[lb_byte + 3U] = ob1[pc + 5U];
            lb_byte += 4U;
            adv = 6U;
            break;

        case 0x4181U: /* @ 0x08001cb8: IN / IN_OUT DWORD direct memory */
        case 0x8181U:
        case 0x4581U: /* @ 0x08002216: IN / IN_OUT DWORD indirect memory (*VDx / *LDx) */
        case 0x8581U:
        {
            uint8_t *src_ptr = plc_call_resolve_param_ptr((uint16_t)(pc + 2U), caller_local, &adv);
            if (((desc & 0x0400U) != 0U) && (src_ptr != NULL)) {
                uint32_t vptr = ((uint32_t)src_ptr[0] << 24) |
                                ((uint32_t)src_ptr[1] << 16) |
                                ((uint32_t)src_ptr[2] << 8)  |
                                 (uint32_t)src_ptr[3];
                src_ptr = plc_vptr_to_phys_local(vptr, caller_local);
            }
            if (src_ptr != NULL) {
                callee_local[lb_byte]      = src_ptr[0];
                callee_local[lb_byte + 1U] = src_ptr[1];
                callee_local[lb_byte + 2U] = src_ptr[2];
                callee_local[lb_byte + 3U] = src_ptr[3];
            }
            lb_byte += 4U;
            break;
        }

        case 0x4781U: /* @ 0x08001c6e: IN / IN_OUT DWORD from AC0..AC3 */
        case 0x4F81U:
        case 0x5781U:
        case 0x5F81U:
        case 0x8781U:
        case 0x8F81U:
        case 0x9781U:
        case 0x9F81U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint32_t val   = PLC_ACCUM_BASE[ac_idx];
            callee_local[lb_byte]      = (uint8_t)((val >> 24) & 0xFFU);
            callee_local[lb_byte + 1U] = (uint8_t)((val >> 16) & 0xFFU);
            callee_local[lb_byte + 2U] = (uint8_t)((val >> 8)  & 0xFFU);
            callee_local[lb_byte + 3U] = (uint8_t)(val & 0xFFU);
            lb_byte += 4U;
            adv = 2U;
            break;
        }

        case 0x4D81U: /* @ 0x080021ba: IN / IN_OUT DWORD indirect via *AC1..*AC3 */
        case 0x5581U:
        case 0x5D81U:
        case 0x8D81U:
        case 0x9581U:
        case 0x9D81U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint8_t *src_ptr = plc_vptr_to_phys_local(PLC_ACCUM_BASE[ac_idx], caller_local);
            if (src_ptr != NULL) {
                callee_local[lb_byte]      = src_ptr[0];
                callee_local[lb_byte + 1U] = src_ptr[1];
                callee_local[lb_byte + 2U] = src_ptr[2];
                callee_local[lb_byte + 3U] = src_ptr[3];
            }
            lb_byte += 4U;
            adv = 2U;
            break;
        }

        case 0xC181U: /* @ 0x080025f8: OUT DWORD direct/indirect memory (skip on entry) */
        case 0xC581U:
            adv = (ob1[pc + 2U] == 0xF0U) ? 6U : 4U;
            lb_byte += 4U;
            break;

        case 0xC781U: /* @ 0x080025ee: OUT DWORD via AC0..AC3 or *AC1..*AC3 */
        case 0xCD81U:
        case 0xCF81U:
        case 0xD581U:
        case 0xD781U:
        case 0xDD81U:
        case 0xDF81U:
            adv = 2U;
            lb_byte += 4U;
            break;

        default:
            PLC_COMPILE_ERR_REG = 0x8010U;
            return pc;
        }

        pc = (uint16_t)(pc + adv);
    }

    return pc;
}

/* ============================================================================
 * 6. Unmarshal Subroutine CALL In-Out / Output Parameters Back to Caller
 *    (FUN_080027ea @ 0x080027ea .. 0x08003c21)
 * ============================================================================
 * Executed after `plc_vm_exec_block` returns from a parameterized `CALL`
 * (`0xFE 0x23`). Skips IN parameters (`0x40xx..0x6A00`) while tracking the
 * LB offset in `callee_local`, and copies IN_OUT (`0x80xx..0x9Fxx`) and OUT
 * (`0xC0xx..0xDFxx`) values from `callee_local` back to the caller's operands.
 */
uint16_t plc_call_unmarshal_out_params(uint16_t caller_pc,
                                       uint8_t *caller_local,
                                       uint8_t *callee_local)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint8_t param_hdr_count = ob1[caller_pc + 2U];
    uint16_t pc = (uint16_t)(caller_pc + 7U);

    uint8_t lb_byte = 0U;
    uint8_t lb_bit  = 0U;

    for (uint16_t idx = 0U; idx <= (uint16_t)(param_hdr_count - 2U); idx++) {
        uint16_t desc = ((uint16_t)ob1[pc] << 8) | (uint16_t)ob1[pc + 1U];

        if ((ob1[pc] == 0x6AU) &&
            (ob1[pc + 1U] == (uint8_t)(ob1[pc + 2U] + 0x81U))) {
            desc = 0x6A00U;
        }

        bool is_bool_param = (desc == 0x6A00U) ||
                             ((desc & 0x3FFFU) == 0x0001U);
        if (!is_bool_param && (lb_bit != 0U)) {
            lb_byte++;
            lb_bit = 0U;
        }

        uint16_t adv = 2U;

        switch (desc) {
        /* --------------------------------------------------------------------
         * A. Skip IN-Only Parameters (0x4001 .. 0x6A00) @ 0x08003a58..0x08003b0c
         * -------------------------------------------------------------------- */
        case 0x6A00U: /* @ 0x08003b0c */
            adv = 4U;
            break;

        case 0x4001U: /* @ 0x08003a58: IN BOOL direct bit */
            adv = (ob1[pc + 3U] == 0xF0U) ? 7U : 5U;
            lb_bit++;
            if (lb_bit > 7U) {
                lb_bit = 0U;
                lb_byte++;
            }
            break;

        case 0x4081U: /* @ 0x08003a62: IN BYTE direct/indirect */
        case 0x4481U:
            adv = (ob1[pc + 2U] == 0xF0U) ? 6U : 4U;
            lb_byte += 1U;
            break;

        case 0x4281U: /* @ 0x08003a68: IN BYTE immediate */
        case 0x4A81U:
        case 0x5281U:
            adv = 3U;
            lb_byte += 1U;
            break;

        case 0x4681U: /* @ 0x08003a7a: IN BYTE ACx / *ACx */
        case 0x4C81U:
        case 0x4E81U:
        case 0x5481U:
        case 0x5681U:
        case 0x5C81U:
        case 0x5E81U:
            adv = 2U;
            lb_byte += 1U;
            break;

        case 0x4101U: /* @ 0x08003a96: IN WORD direct/indirect */
        case 0x4501U:
            adv = (ob1[pc + 2U] == 0xF0U) ? 6U : 4U;
            lb_byte += 2U;
            break;

        case 0x4301U: /* @ 0x08003ab0: IN WORD immediate */
        case 0x4B01U:
        case 0x5301U:
            adv = 4U;
            lb_byte += 2U;
            break;

        case 0x4701U: /* @ 0x08003a84: IN WORD ACx / *ACx */
        case 0x4D01U:
        case 0x4F01U:
        case 0x5501U:
        case 0x5701U:
        case 0x5D01U:
        case 0x5F01U:
            adv = 2U;
            lb_byte += 2U;
            break;

        case 0x4181U: /* @ 0x08003af0: IN DWORD direct/indirect */
        case 0x4581U:
            adv = (ob1[pc + 2U] == 0xF0U) ? 6U : 4U;
            lb_byte += 4U;
            break;

        case 0x4381U: /* @ 0x08003ad0: IN DWORD/PTR immediate */
        case 0x4B81U:
        case 0x5381U:
        case 0x5B81U:
        case 0x6381U:
            adv = 6U;
            lb_byte += 4U;
            break;

        case 0x4781U: /* @ 0x08003ac2: IN DWORD ACx / *ACx */
        case 0x4D81U:
        case 0x4F81U:
        case 0x5581U:
        case 0x5781U:
        case 0x5D81U:
        case 0x5F81U:
            adv = 2U;
            lb_byte += 4U;
            break;

        /* --------------------------------------------------------------------
         * B. Unmarshal IN_OUT & OUT Parameters (0x8001 .. 0xDF81)
         * -------------------------------------------------------------------- */
        case 0x8001U: /* @ 0x08002a9a: IN_OUT / OUT BOOL direct bit */
        case 0xC001U:
        {
            uint8_t dst_bit = ob1[pc + 2U] & 0x07U;
            uint8_t *dst_ptr = plc_call_resolve_param_ptr((uint16_t)(pc + 3U), caller_local, &adv);
            adv = (uint16_t)(adv + 1U);
            uint8_t bit_val = (callee_local[lb_byte] >> lb_bit) & 1U;
            if (dst_ptr != NULL) {
                if (bit_val != 0U) {
                    *dst_ptr |= (uint8_t)(1U << dst_bit);
                } else {
                    *dst_ptr &= (uint8_t)~(1U << dst_bit);
                }
            }
            lb_bit++;
            if (lb_bit > 7U) {
                lb_bit = 0U;
                lb_byte++;
            }
            break;
        }

        case 0x8081U: /* @ 0x08002da4: IN_OUT / OUT BYTE direct memory */
        case 0xC081U:
        case 0x8481U: /* @ 0x080034cc: IN_OUT / OUT BYTE indirect memory */
        case 0xC481U:
        {
            uint8_t *dst_ptr = plc_call_resolve_param_ptr((uint16_t)(pc + 2U), caller_local, &adv);
            if (((desc & 0x0400U) != 0U) && (dst_ptr != NULL)) {
                uint32_t vptr = ((uint32_t)dst_ptr[0] << 24) |
                                ((uint32_t)dst_ptr[1] << 16) |
                                ((uint32_t)dst_ptr[2] << 8)  |
                                 (uint32_t)dst_ptr[3];
                dst_ptr = plc_vptr_to_phys_local(vptr, caller_local);
            }
            if (dst_ptr != NULL) {
                *dst_ptr = callee_local[lb_byte];
            }
            lb_byte += 1U;
            break;
        }

        case 0x8681U: /* @ 0x08002f52: IN_OUT / OUT BYTE to AC0..AC3 */
        case 0x8E81U:
        case 0x9681U:
        case 0x9E81U:
        case 0xC681U:
        case 0xCE81U:
        case 0xD681U:
        case 0xDE81U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            PLC_ACCUM_BASE[ac_idx] = (PLC_ACCUM_BASE[ac_idx] & 0xFFFFFF00U) |
                                     (uint32_t)callee_local[lb_byte];
            lb_byte += 1U;
            adv = 2U;
            break;
        }

        case 0x8C81U: /* @ 0x080035e2: IN_OUT / OUT BYTE indirect via *AC1..*AC3 */
        case 0x9481U:
        case 0x9C81U:
        case 0xCC81U:
        case 0xD481U:
        case 0xDC81U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint8_t *dst_ptr = plc_vptr_to_phys_local(PLC_ACCUM_BASE[ac_idx], caller_local);
            if (dst_ptr != NULL) {
                *dst_ptr = callee_local[lb_byte];
            }
            lb_byte += 1U;
            adv = 2U;
            break;
        }

        case 0x8101U: /* @ 0x0800306a: IN_OUT / OUT WORD direct memory */
        case 0xC101U:
        case 0x8501U: /* @ 0x080036f4: IN_OUT / OUT WORD indirect memory */
        case 0xC501U:
        {
            uint8_t *dst_ptr = plc_call_resolve_param_ptr((uint16_t)(pc + 2U), caller_local, &adv);
            if (((desc & 0x0400U) != 0U) && (dst_ptr != NULL)) {
                uint32_t vptr = ((uint32_t)dst_ptr[0] << 24) |
                                ((uint32_t)dst_ptr[1] << 16) |
                                ((uint32_t)dst_ptr[2] << 8)  |
                                 (uint32_t)dst_ptr[3];
                dst_ptr = plc_vptr_to_phys_local(vptr, caller_local);
            }
            if (dst_ptr != NULL) {
                dst_ptr[0] = callee_local[lb_byte];
                dst_ptr[1] = callee_local[lb_byte + 1U];
            }
            lb_byte += 2U;
            break;
        }

        case 0x8701U: /* @ 0x08002fe6: IN_OUT / OUT WORD to AC0..AC3 */
        case 0x8F01U:
        case 0x9701U:
        case 0x9F01U:
        case 0xC701U:
        case 0xCF01U:
        case 0xD701U:
        case 0xDF01U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint32_t wval  = ((uint32_t)callee_local[lb_byte] << 8) |
                              (uint32_t)callee_local[lb_byte + 1U];
            PLC_ACCUM_BASE[ac_idx] = (PLC_ACCUM_BASE[ac_idx] & 0xFFFF0000U) | wval;
            lb_byte += 2U;
            adv = 2U;
            break;
        }

        case 0x8D01U: /* @ 0x08003652: IN_OUT / OUT WORD indirect via *AC1..*AC3 */
        case 0x9501U:
        case 0x9D01U:
        case 0xCD01U:
        case 0xD501U:
        case 0xDD01U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint8_t *dst_ptr = plc_vptr_to_phys_local(PLC_ACCUM_BASE[ac_idx], caller_local);
            if (dst_ptr != NULL) {
                dst_ptr[0] = callee_local[lb_byte];
                dst_ptr[1] = callee_local[lb_byte + 1U];
            }
            lb_byte += 2U;
            adv = 2U;
            break;
        }

        case 0x8181U: /* @ 0x0800332e: IN_OUT / OUT DWORD direct memory */
        case 0xC181U:
        case 0x8581U: /* @ 0x080038f8: IN_OUT / OUT DWORD indirect memory */
        case 0xC581U:
        {
            uint8_t *dst_ptr = plc_call_resolve_param_ptr((uint16_t)(pc + 2U), caller_local, &adv);
            if (((desc & 0x0400U) != 0U) && (dst_ptr != NULL)) {
                uint32_t vptr = ((uint32_t)dst_ptr[0] << 24) |
                                ((uint32_t)dst_ptr[1] << 16) |
                                ((uint32_t)dst_ptr[2] << 8)  |
                                 (uint32_t)dst_ptr[3];
                dst_ptr = plc_vptr_to_phys_local(vptr, caller_local);
            }
            if (dst_ptr != NULL) {
                dst_ptr[0] = callee_local[lb_byte];
                dst_ptr[1] = callee_local[lb_byte + 1U];
                dst_ptr[2] = callee_local[lb_byte + 2U];
                dst_ptr[3] = callee_local[lb_byte + 3U];
            }
            lb_byte += 4U;
            break;
        }

        case 0x8781U: /* @ 0x080032a8: IN_OUT / OUT DWORD to AC0..AC3 */
        case 0x8F81U:
        case 0x9781U:
        case 0x9F81U:
        case 0xC781U:
        case 0xCF81U:
        case 0xD781U:
        case 0xDF81U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            PLC_ACCUM_BASE[ac_idx] = ((uint32_t)callee_local[lb_byte]      << 24) |
                                     ((uint32_t)callee_local[lb_byte + 1U] << 16) |
                                     ((uint32_t)callee_local[lb_byte + 2U] << 8)  |
                                      (uint32_t)callee_local[lb_byte + 3U];
            lb_byte += 4U;
            adv = 2U;
            break;
        }

        case 0x8D81U: /* @ 0x08003864: IN_OUT / OUT DWORD indirect via *AC1..*AC3 */
        case 0x9581U:
        case 0x9D81U:
        case 0xCD81U:
        case 0xD581U:
        case 0xDD81U:
        {
            uint8_t ac_idx = (uint8_t)((desc >> 11) & 0x03U);
            uint8_t *dst_ptr = plc_vptr_to_phys_local(PLC_ACCUM_BASE[ac_idx], caller_local);
            if (dst_ptr != NULL) {
                dst_ptr[0] = callee_local[lb_byte];
                dst_ptr[1] = callee_local[lb_byte + 1U];
                dst_ptr[2] = callee_local[lb_byte + 2U];
                dst_ptr[3] = callee_local[lb_byte + 3U];
            }
            lb_byte += 4U;
            adv = 2U;
            break;
        }

        default:
            break;
        }

        pc = (uint16_t)(pc + adv);
    }

    return pc;
}

/* ============================================================================
 * 7. Skip Subroutine CALL Parameters When EN == 0
 *    (FUN_08003c22 @ 0x08003c22 .. 0x08003e5b)
 * ============================================================================
 * Advances `caller_pc` past all encoded parameter descriptors of a `CALL`
 * with parameters (`0xFE 0x23`) when the top of the logic stack (EN) is 0.
 */
uint16_t plc_call_skip_params(uint16_t caller_pc)
{
    const uint8_t *ob1 = PLC_PHYS_BASE_OB1_FLASH;
    uint8_t param_hdr_count = ob1[caller_pc + 2U];
    uint16_t pc = (uint16_t)(caller_pc + 7U);

    for (uint16_t idx = 0U; idx <= (uint16_t)(param_hdr_count - 2U); idx++) {
        uint16_t desc = ((uint16_t)ob1[pc] << 8) | (uint16_t)ob1[pc + 1U];

        if ((ob1[pc] == 0x6AU) &&
            (ob1[pc + 1U] == (uint8_t)(ob1[pc + 2U] + 0x81U))) {
            pc = (uint16_t)(pc + 4U);
            continue;
        }

        switch (desc) {
        /* @ 0x08003e42: 1-byte immediate constant (2B desc + 1B imm = 3B) */
        case 0x4281U:
        case 0x4A81U:
        case 0x5281U:
            pc = (uint16_t)(pc + 3U);
            break;

        /* @ 0x08003e3e: 2-byte immediate constant (2B desc + 2B imm = 4B) */
        case 0x4301U:
        case 0x4B01U:
        case 0x5301U:
            pc = (uint16_t)(pc + 4U);
            break;

        /* @ 0x08003e46: 4-byte immediate constant / pointer (2B desc + 4B imm = 6B) */
        case 0x4381U:
        case 0x4B81U:
        case 0x5381U:
        case 0x5B81U:
        case 0x6381U:
            pc = (uint16_t)(pc + 6U);
            break;

        /* @ 0x08003e4a: Register / Indirect Register AC0..AC3 or *AC1..*AC3 (2B desc only) */
        case 0x4681U: case 0x4701U: case 0x4781U:
        case 0x4C81U: case 0x4D01U: case 0x4D81U:
        case 0x4E81U: case 0x4F01U: case 0x4F81U:
        case 0x5481U: case 0x5501U: case 0x5581U:
        case 0x5681U: case 0x5701U: case 0x5781U:
        case 0x5C81U: case 0x5D01U: case 0x5D81U:
        case 0x5E81U: case 0x5F01U: case 0x5F81U:
        case 0x8681U: case 0x8701U: case 0x8781U:
        case 0x8C81U: case 0x8D01U: case 0x8D81U:
        case 0x8E81U: case 0x8F01U: case 0x8F81U:
        case 0x9481U: case 0x9501U: case 0x9581U:
        case 0x9681U: case 0x9701U: case 0x9781U:
        case 0x9C81U: case 0x9D01U: case 0x9D81U:
        case 0x9E81U: case 0x9F01U: case 0x9F81U:
        case 0xC681U: case 0xC701U: case 0xC781U:
        case 0xCC81U: case 0xCD01U: case 0xCD81U:
        case 0xCE81U: case 0xCF01U: case 0xCF81U:
        case 0xD481U: case 0xD501U: case 0xD581U:
        case 0xD681U: case 0xD701U: case 0xD781U:
        case 0xDC81U: case 0xDD01U: case 0xDD81U:
        case 0xDE81U: case 0xDF01U: case 0xDF81U:
            pc = (uint16_t)(pc + 2U);
            break;

        /* Direct or Indirect Memory Address (or BOOL bit operand) */
        case 0x4001U:
        case 0x8001U:
        case 0xC001U:
            pc = (uint16_t)(pc + ((ob1[pc + 3U] == 0xF0U) ? 7U : 5U));
            break;

        default:
            pc = (uint16_t)(pc + ((ob1[pc + 2U] == 0xF0U) ? 6U : 4U));
            break;
        }
    }

    return pc;
}
