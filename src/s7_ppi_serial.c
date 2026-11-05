/**
 * @file s7_ppi_serial.c
 * @brief SIMATIC S7-226 (APM32E103) RS-485 PPI (Point-to-Point Interface)
 *        Multi-Drop Serial Communication Protocol Engine & Block Checksum Verifiers.
 *
 * Clean, modular C reconstruction of firmware functions:
 *   - FUN_08017450 (0x08017450..0x08021787, 41,768 bytes: RS-485 PPI Protocol Engine)
 *   - FUN_08021794 (0x08021794..0x0802189f, 268 bytes: Fletcher-16 Block Checksum Verifier)
 *   - FUN_080218a0 (0x080218a0..0x08021947, 168 bytes: Weighted Block Checksum Verifier)
 *
 * Hardware & Protocol Architecture:
 *   - Operates on Port 0 (USART1 RS-485), Port 1 (USART2 RS-485), and Port 2.
 *   - Implements the two-phase Profibus FDL / PPI request-response handshake:
 *     1. Master sends an SD2 variable-length frame:
 *        [68, LE, LER, 68, DA, SA, FC, S7comm_PDU..., FCS, 16]
 *        The PLC validates FCS, immediately places 0xE5 (PPI_SC short ack) into tx_buf[0]
 *        (*tx_len_ptr = 1), and constructs the complete SD2 response frame in tx_buf[1..]
 *        with its length stored in *saved_tx_len and *rx_sd_state = 0x68.
 *     2. Master follows up with an SD1 fixed-length data poll frame:
 *        [10, DA, SA, 5C, FCS, 16]
 *        The PLC transmits the queued SD2 response frame from tx_buf[1..].
 *   - During STEP 7-Micro/WIN Block Downloads (0x1A Request Download), the PLC acts as the
 *     active job initiator on subsequent SD1 polls, emitting 37-byte SD2 frames containing
 *     S7comm Job 0x1B (Download Block) and 0x1C (Download Ended) for _0B00000P (SDB),
 *     _0800001P (OB1), and _0A00001P (DB1).
 *   - Supports PG/Multi-Master 6-byte extended routing headers when DA > 0x7F (high bit set).
 */

#include "s7_ppi_comm.h"

/* ============================================================================
 * External Firmware Hardware & Runtime Helpers
 * ============================================================================ */

extern void     FUN_08036bc8(uint32_t flash_page_addr);
extern int      FUN_08036ca0(uint32_t flash_addr, uint16_t halfword);
extern void     FUN_080219c8(void);
extern void     FUN_0802b8e8(void);
extern void     FUN_0802bd58(void);
extern void     FUN_0802bd88(void);
extern void     FUN_0802c010(uint8_t baud_code);
extern void     FUN_0802c060(uint8_t baud_code);
extern void     FUN_0802fd3c(void);
extern void     FUN_08032998(uint16_t delay_ms);
extern uint32_t FUN_08003ff0(const uint8_t *buf);
extern uint16_t FUN_08004198(const uint8_t *buf);
extern uint16_t FUN_0800422c(uint8_t type_code, uint16_t count);

/* ============================================================================
 * Memory-Mapped Firmware Globals & Per-Port State (APM32E103 SRAM / Flash)
 * ============================================================================ */

#define MEM_U8(addr)                    (*(volatile uint8_t  *)(uintptr_t)(addr))
#define MEM_U16(addr)                   (*(volatile uint16_t *)(uintptr_t)(addr))
#define MEM_U32(addr)                   (*(volatile uint32_t *)(uintptr_t)(addr))
#define MEM_PTR(addr)                   ((uint8_t *)(uintptr_t)(addr))
#define CONST_MEM_PTR(addr)             ((const uint8_t *)(uintptr_t)(addr))

#define g_plc_run_init_flag             MEM_U8(0x20000008UL)  /* DAT_20000008 */
#define g_plc_run_mode                  MEM_U8(0x20000009UL)  /* DAT_20000009: 1=RUN, 0=STOP */
#define g_plc_prev_run_mode             MEM_U8(0x20000011UL)  /* DAT_20000011 */
#define g_plc_hw_fault_flag             MEM_U8(0x20000017UL)  /* DAT_20000017 */
#define g_rtc_bcd_year                  MEM_U8(0x2000001CUL)  /* DAT_2000001c */
#define g_rtc_bcd_month                 MEM_U8(0x2000001DUL)  /* DAT_2000001d */
#define g_rtc_bcd_day                   MEM_U8(0x2000001EUL)  /* DAT_2000001e */
#define g_rtc_bcd_hour                  MEM_U8(0x2000001FUL)  /* DAT_2000001f */
#define g_rtc_bcd_min                   MEM_U8(0x20000020UL)  /* DAT_20000020 */
#define g_rtc_bcd_sec                   MEM_U8(0x20000021UL)  /* DAT_20000021 */
#define g_rtc_bcd_dow                   MEM_U8(0x20000023UL)  /* DAT_20000023 */
#define g_rtc_write_buf                 MEM_PTR(0x20000024UL) /* DAT_20000024..0x2000002A */
#define g_ppi_port0_station_addr        MEM_U8(0x20000030UL)  /* DAT_20000030 */
#define g_ppi_port1_station_addr        MEM_U8(0x20000031UL)  /* DAT_20000031 */
#define g_ppi_route_byte_port2          MEM_U8(0x20000034UL)  /* DAT_20000034 */
#define g_ppi_route_byte_port1          MEM_U8(0x20000035UL)  /* DAT_20000035 */
#define g_ppi_route_byte_port0          MEM_U8(0x20000036UL)  /* DAT_20000036 */

#define g_dl_commit_pending_type        MEM_U8(0x20000484UL)  /* DAT_20000484: 1=SDB, 2=OB1, 3=DB1 */
#define g_dl_active_block_type          MEM_U8(0x20000487UL)  /* DAT_20000487: 1=SDB, 2=OB1, 3=DB1 */
#define g_dl_poll_ready_port0           MEM_U8(0x20000488UL)  /* DAT_20000488: Port 0 download poll ready */
#define g_dl_poll_ready_port1           MEM_U8(0x20000489UL)  /* DAT_20000489: Port 1 download poll ready */
#define g_dl_poll_ready_port2           MEM_U8(0x2000048AUL)  /* DAT_2000048a: Port 2 download poll ready */
#define g_dl_chunk_seq_num              MEM_U16(0x2000048EUL) /* DAT_2000048e */
#define g_block_xfer_offset             MEM_U16(0x200004C0UL) /* DAT_200004c0 */
#define g_status_resp_len               MEM_U16(0x200004C2UL) /* DAT_200004c2 */
#define g_ob1_block_len                 MEM_U16(0x200004C6UL) /* DAT_200004c6 */
#define g_sdb_block_len                 MEM_U16(0x200004C8UL) /* DAT_200004c8 */
#define g_db1_block_len                 MEM_U16(0x200004CAUL) /* DAT_200004ca */
#define g_ob1_compile_err_code          MEM_U16(0x200004CCUL) /* DAT_200004cc */
#define g_force_table_count             MEM_U8(0x200004CEUL)  /* DAT_200004ce */
#define g_force_io_bit_active           MEM_U8(0x200004CFUL)  /* DAT_200004cf */
#define g_password_auth_port0           MEM_U8(0x200004D0UL)  /* DAT_200004d0 */
#define g_password_auth_port1           MEM_U8(0x200004D1UL)  /* DAT_200004d1 */
#define g_password_auth_port2           MEM_U8(0x200004D2UL)  /* DAT_200004d2 */
#define g_status_monitor_trigger        MEM_U8(0x2000080DUL)  /* DAT_2000080d */

/* ============================================================================
 * 1. Block Checksum Verification Functions (FUN_08021794 & FUN_080218a0)
 * ============================================================================ */

/**
 * @brief Verifies the 16-bit Fletcher-like checksum appended to a raw PLC block
 *        (DB1 at 0x08064800, raw OB1 at 0x08075800, or SDB at 0x0807f800).
 *        Reconstructed from FUN_08021794 (0x08021794..0x0802189f, 268 bytes).
 *
 * @param block_ptr     Pointer to start of block data
 * @param block_len     Length of block body in bytes (checksum is at block_ptr[block_len..block_len+1])
 * @return 0 if valid, 1 if checksum mismatch
 */
uint8_t s7_block_verify_fletcher_checksum(const uint8_t *block_ptr, uint16_t block_len)
{
    uint8_t sum1 = 0U;
    uint8_t sum2 = 0U;

    for (uint16_t i = 0; i < block_len; i++) {
        uint16_t uVar2 = (uint16_t)sum1 + (uint16_t)block_ptr[i];
        sum1 = (uint8_t)uVar2;
        if (uVar2 > 0xFFU) {
            sum1 = (uint8_t)(uVar2 + 1U);
        }

        uVar2 = (uint16_t)sum2 + (uint16_t)sum1;
        sum2 = (uint8_t)uVar2;
        if (uVar2 > 0xFFU) {
            sum2 = (uint8_t)(uVar2 + 1U);
        }
    }

    uint8_t expected_lo = (uint8_t)(0xFFU - ((uint16_t)(sum1 + sum2) % 0xFFU));
    uint8_t expected_hi = (uint8_t)(0xFFU - ((uint16_t)(sum1 + expected_lo) % 0xFFU));

    if (expected_lo == block_ptr[block_len] && expected_hi == block_ptr[block_len + 1U]) {
        return 0U;
    }
    return 1U;
}

uint8_t FUN_08021794(const uint8_t *param_1, uint16_t param_2)
{
    return s7_block_verify_fletcher_checksum(param_1, param_2);
}

/**
 * @brief Verifies the 16-bit weighted XOR-sum / additive checksum appended to
 *        the compiled OB1 block at 0x0806d800.
 *        Reconstructed from FUN_080218a0 (0x080218a0..0x08021947, 168 bytes).
 *
 * @param block_ptr     Pointer to start of block data
 * @param block_len     Length of block body in bytes (checksum is at block_ptr[block_len..block_len+1])
 * @return 0 if valid, 1 if checksum mismatch
 */
uint8_t s7_block_verify_weighted_checksum(const uint8_t *block_ptr, uint16_t block_len)
{
    uint8_t acc_xor_add = 0U;
    uint8_t acc_sum     = 0U;

    for (uint16_t i = 0; i < block_len; i++) {
        acc_xor_add = (uint8_t)((acc_xor_add + block_ptr[i]) ^ block_ptr[i]);
        acc_sum     = (uint8_t)(acc_sum + block_ptr[i]);
    }

    if (acc_xor_add == block_ptr[block_len] && acc_sum == block_ptr[block_len + 1U]) {
        return 0U;
    }
    return 1U;
}

uint8_t FUN_080218a0(const uint8_t *param_1, uint16_t param_2)
{
    return s7_block_verify_weighted_checksum(param_1, param_2);
}

/* ============================================================================
 * 2. Internal Helpers for PPI Frame Checksum & Flash Block Commit
 * ============================================================================ */

static inline uint16_t ppi_read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline void ppi_write_be16(uint8_t *p, uint16_t val)
{
    p[0] = (uint8_t)(val >> 8);
    p[1] = (uint8_t)(val & 0xFFU);
}

static inline void ppi_write_be32(uint8_t *p, uint32_t val)
{
    p[0] = (uint8_t)(val >> 24);
    p[1] = (uint8_t)(val >> 16);
    p[2] = (uint8_t)(val >> 8);
    p[3] = (uint8_t)(val & 0xFFU);
}

/**
 * @brief Computes the 8-bit PPI Frame Check Sequence (FCS) over `len` bytes
 *        (DA + SA + FC + S7comm PDU).
 */
static uint8_t ppi_compute_fcs(const uint8_t *buf, uint16_t len)
{
    uint8_t fcs = 0U;
    for (uint16_t i = 0; i < len; i++) {
        fcs = (uint8_t)(fcs + buf[i]);
    }
    return fcs;
}

static volatile uint8_t *ppi_get_port_poll_flag(uint8_t port_id)
{
    if (port_id == PPI_PORT_0_USART1) return &g_dl_poll_ready_port0;
    if (port_id == PPI_PORT_1_USART2) return &g_dl_poll_ready_port1;
    return &g_dl_poll_ready_port2;
}

static volatile uint8_t *ppi_get_port_auth_flag(uint8_t port_id)
{
    if (port_id == PPI_PORT_0_USART1) return &g_password_auth_port0;
    if (port_id == PPI_PORT_1_USART2) return &g_password_auth_port1;
    return &g_password_auth_port2;
}

static volatile uint8_t *ppi_get_port_route_byte(uint8_t port_id)
{
    if (port_id == PPI_PORT_0_USART1) return &g_ppi_route_byte_port0;
    if (port_id == PPI_PORT_1_USART2) return &g_ppi_route_byte_port1;
    return &g_ppi_route_byte_port2;
}

static void ppi_program_fletcher_checksum(const uint8_t *data, uint16_t len, uint32_t flash_addr)
{
    uint8_t sum1 = 0U;
    uint8_t sum2 = 0U;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t t1 = (uint16_t)sum1 + (uint16_t)data[i];
        sum1 = (t1 > 0xFFU) ? (uint8_t)(t1 + 1U) : (uint8_t)t1;
        uint16_t t2 = (uint16_t)sum2 + (uint16_t)sum1;
        sum2 = (t2 > 0xFFU) ? (uint8_t)(t2 + 1U) : (uint8_t)t2;
    }
    uint8_t lo = (uint8_t)(0xFFU - ((uint16_t)(sum1 + sum2) % 0xFFU));
    uint8_t hi = (uint8_t)(0xFFU - ((uint16_t)(sum1 + lo) % 0xFFU));
    (void)FUN_08036ca0(flash_addr, (uint16_t)(((uint16_t)hi << 8) | lo));
}

static void ppi_program_weighted_checksum(const uint8_t *data, uint16_t len, uint32_t flash_addr)
{
    uint8_t acc_xor = 0U;
    uint8_t acc_sum = 0U;
    for (uint16_t i = 0; i < len; i++) {
        acc_xor = (uint8_t)((acc_xor + data[i]) ^ data[i]);
        acc_sum = (uint8_t)(acc_sum + data[i]);
    }
    (void)FUN_08036ca0(flash_addr, (uint16_t)(((uint16_t)acc_sum << 8) | acc_xor));
}

/**
 * @brief Executes the deferred Flash commit after a PPI Block Download completes
 *        (triggered when g_dl_commit_pending_type != 0 on the final SD1 data poll).
 */
static void ppi_commit_downloaded_block(uint8_t commit_type)
{
    if (commit_type == S7_DL_BLOCK_SDB) {
        /* Copy staged SDB from 0x2000108C to 0x200058CC and program to 0x0807F800 */
        uint16_t sdb_len = g_sdb_block_len;
        if (sdb_len > 0x0180U) sdb_len = 0x0180U;
        uint8_t *sdb_ram = MEM_PTR(SRAM_SDB_MIRROR_ADDR);
        memcpy(sdb_ram, CONST_MEM_PTR(SRAM_V_BASE_ADDR), sdb_len);

        FUN_08036bc8(FLASH_SDB_BASE_ADDR);
        uint32_t f_addr = FLASH_SDB_BASE_ADDR;
        for (uint16_t i = 0; i < sdb_len; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)sdb_ram[i + 1U] << 8) | sdb_ram[i]);
            if (FUN_08036ca0(f_addr, hw) != 4) break;
            f_addr += 2U;
        }
        ppi_program_fletcher_checksum(CONST_MEM_PTR(FLASH_SDB_BASE_ADDR), sdb_len, f_addr);

        /* Restore DB1 V-memory from Flash 0x08064800 */
        memcpy(MEM_PTR(SRAM_DB1_HEADER_ADDR), CONST_MEM_PTR(FLASH_DB1_BASE_ADDR), g_db1_block_len);
        FUN_0802bd58();
        FUN_0802bd88();
    } else if (commit_type == S7_DL_BLOCK_OB1) {
        /* Program Fletcher checksum at end of raw OB1 (0x08075800 + g_ob1_block_len) */
        uint16_t ob1_len = g_ob1_block_len;
        ppi_program_fletcher_checksum(
            CONST_MEM_PTR(FLASH_OB1_RAW_BASE_ADDR),
            ob1_len,
            FLASH_OB1_RAW_BASE_ADDR + ob1_len
        );

        /* Erase compiled OB1 region (0x0806D800..0x08075000) and de-obfuscate */
        for (uint32_t page = FLASH_OB1_COMPILED_BASE_ADDR; page <= 0x08075000UL; page += 0x0800UL) {
            FUN_08036bc8(page);
        }
        const uint8_t *raw_ob1 = CONST_MEM_PTR(FLASH_OB1_RAW_BASE_ADDR);
        uint32_t write_addr = FLASH_OB1_COMPILED_BASE_ADDR;
        for (uint16_t i = 0; i < 38U && i < ob1_len; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)raw_ob1[i + 1U] << 8) | raw_ob1[i]);
            (void)FUN_08036ca0(write_addr, hw);
            write_addr += 2U;
        }
        if (ob1_len > 38U) {
            s7_block_deobfuscate_ob1(raw_ob1 + 38U, (uint16_t)(ob1_len - 38U), &write_addr);
        }
        uint16_t comp_len = ppi_read_be16(CONST_MEM_PTR(FLASH_OB1_COMPILED_BASE_ADDR + 10U));
        if (comp_len == 0U || comp_len > 0x7FF0U) comp_len = ob1_len;
        ppi_program_weighted_checksum(
            CONST_MEM_PTR(FLASH_OB1_COMPILED_BASE_ADDR),
            comp_len,
            FLASH_OB1_COMPILED_BASE_ADDR + comp_len
        );
        FUN_080219c8();
    } else if (commit_type == S7_DL_BLOCK_DB1) {
        /* Erase DB1 Flash (0x08064800..0x08068000) and program DB1 from RAM (0x20001068) */
        for (uint32_t page = FLASH_DB1_BASE_ADDR; page <= 0x08068000UL; page += 0x0800UL) {
            FUN_08036bc8(page);
        }
        const uint8_t *db1_ram = CONST_MEM_PTR(SRAM_DB1_HEADER_ADDR);
        uint16_t db1_len = g_db1_block_len;
        if (db1_len > 0x3D24U) db1_len = 0x3D24U;
        uint32_t f_addr = FLASH_DB1_BASE_ADDR;
        for (uint16_t i = 0; i < db1_len; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)db1_ram[i + 1U] << 8) | db1_ram[i]);
            if (FUN_08036ca0(f_addr, hw) != 4) break;
            f_addr += 2U;
        }
        ppi_program_fletcher_checksum(
            CONST_MEM_PTR(FLASH_DB1_BASE_ADDR),
            db1_len,
            FLASH_DB1_CHECKSUM_ADDR
        );
    }
}

/* ============================================================================
 * 3. PPI Memory Read / Write / Force / Status Helpers
 * ============================================================================ */

static uint16_t ppi_read_var_item(
    const s7_any_item_t *item,
    uint8_t             *dst_data,
    bool                 is_last_item
)
{
    uint8_t  transport_type = item->transport_type;
    uint16_t count          = ppi_read_be16(&item->count_hi);
    uint16_t db_num         = ppi_read_be16(&item->db_num_hi);
    uint8_t  area           = item->area;
    uint32_t bit_addr       = ((uint32_t)item->bit_addr_hi << 16) |
                              ((uint32_t)item->bit_addr_mid << 8) |
                              (uint32_t)item->bit_addr_lo;
    uint16_t byte_offset    = (uint16_t)(bit_addr >> 3);
    uint8_t  bit_index      = (uint8_t)(bit_addr & 0x07U);

    uint16_t byte_len = FUN_0800422c(transport_type, count);
    uint8_t *payload  = dst_data + 4U;

    dst_data[0] = S7_RET_OK;
    if (transport_type == S7_PARAM_TYPE_BIT) {
        dst_data[1] = S7_TS_BIT;
        ppi_write_be16(&dst_data[2], 1U);
        byte_len = 1U;
    } else if (transport_type == S7_PARAM_TYPE_C ||
               transport_type == S7_PARAM_TYPE_T ||
               transport_type == S7_PARAM_TYPE_HC) {
        dst_data[1] = S7_TS_OCTET_STRING;
        ppi_write_be16(&dst_data[2], byte_len);
    } else {
        dst_data[1] = S7_TS_BYTE_WORD_DWORD;
        ppi_write_be16(&dst_data[2], (uint16_t)(byte_len << 3));
    }

    const uint8_t *src_base = NULL;
    uint16_t max_offset = 0U;

    switch (area) {
        case S7_AREA_I:     src_base = CONST_MEM_PTR(SRAM_I_BASE_ADDR);  max_offset = 32U;  break;
        case S7_AREA_Q:     src_base = CONST_MEM_PTR(SRAM_Q_BASE_ADDR);  max_offset = 32U;  break;
        case S7_AREA_M:     src_base = CONST_MEM_PTR(SRAM_M_BASE_ADDR);  max_offset = 32U;  break;
        case S7_AREA_S:     src_base = CONST_MEM_PTR(SRAM_S_BASE_ADDR);  max_offset = 32U;  break;
        case S7_AREA_SM:    src_base = CONST_MEM_PTR(SRAM_SM_BASE_ADDR); max_offset = 550U; break;
        case S7_AREA_AI:    src_base = CONST_MEM_PTR(SRAM_AI_BASE_ADDR); max_offset = 128U; break;
        case S7_AREA_AQ:    src_base = CONST_MEM_PTR(SRAM_AQ_BASE_ADDR); max_offset = 128U; break;
        case S7_AREA_V_DB1:
            src_base   = (db_num == 1U) ? CONST_MEM_PTR(SRAM_V_BASE_ADDR) : CONST_MEM_PTR(SRAM_DB1_HEADER_ADDR);
            max_offset = (db_num == 1U) ? 0x297CU : 0x29A0U;
            break;
        case S7_AREA_C: {
            uint16_t c_idx = (transport_type == S7_PARAM_TYPE_C) ? (uint16_t)bit_addr : (byte_offset >> 1);
            const uint8_t  *c_stat = CONST_MEM_PTR(SRAM_C_STATUS_BITS_ADDR);
            const uint16_t *c_val  = (const uint16_t *)(uintptr_t)SRAM_C_CURRENT_VAL_ADDR;
            if (transport_type == S7_PARAM_TYPE_C) {
                for (uint16_t i = 0; i < count; i++) {
                    uint16_t idx = (uint16_t)((c_idx + i) & 0xFFU);
                    payload[i * 3U + 0U] = (c_stat[idx] & 0x10U) ? 0x01U : 0x00U;
                    ppi_write_be16(&payload[i * 3U + 1U], c_val[idx]);
                }
            } else {
                for (uint16_t i = 0; i < count; i++) {
                    ppi_write_be16(&payload[i * 2U], c_val[(c_idx + i) & 0xFFU]);
                }
            }
            goto item_done;
        }
        case S7_AREA_T: {
            uint16_t t_idx = (transport_type == S7_PARAM_TYPE_T) ? (uint16_t)bit_addr : (byte_offset >> 1);
            const uint8_t  *t_stat = CONST_MEM_PTR(SRAM_T_STATUS_BITS_ADDR);
            const uint16_t *t_val  = (const uint16_t *)(uintptr_t)SRAM_T_CURRENT_VAL_ADDR;
            if (transport_type == S7_PARAM_TYPE_T) {
                for (uint16_t i = 0; i < count; i++) {
                    uint16_t idx = (uint16_t)((t_idx + i) & 0xFFU);
                    payload[i * 5U + 0U] = (t_stat[idx] & 0x02U) ? 0x01U : 0x00U;
                    payload[i * 5U + 1U] = 0x00U;
                    payload[i * 5U + 2U] = 0x00U;
                    ppi_write_be16(&payload[i * 5U + 3U], t_val[idx]);
                }
            } else {
                for (uint16_t i = 0; i < count; i++) {
                    ppi_write_be16(&payload[i * 2U], t_val[(t_idx + i) & 0xFFU]);
                }
            }
            goto item_done;
        }
        case S7_AREA_HC: {
            uint16_t hc_idx = (transport_type == S7_PARAM_TYPE_HC) ? (uint16_t)bit_addr : (byte_offset >> 2);
            const uint32_t *hc_val = (const uint32_t *)(uintptr_t)SRAM_HC_CURRENT_VAL_ADDR;
            for (uint16_t i = 0; i < count; i++) {
                ppi_write_be32(&payload[i * 4U], hc_val[(hc_idx + i) % 20U]);
            }
            goto item_done;
        }
        case S7_AREA_SYS_INFO: {
            memset(payload, 0, byte_len);
            if (byte_len >= 1U) payload[0] = g_plc_run_mode;
            if (byte_len >= 2U) payload[1] = (g_force_table_count > 0U || g_force_io_bit_active != 0U) ? 0x80U : 0x00U;
            goto item_done;
        }
        default:
            dst_data[0] = S7_RET_INVALID_ADDR;
            dst_data[1] = S7_TS_NULL;
            ppi_write_be16(&dst_data[2], 0U);
            return 4U;
    }

    if (byte_offset >= max_offset) {
        dst_data[0] = S7_RET_INVALID_ADDR;
        dst_data[1] = S7_TS_NULL;
        ppi_write_be16(&dst_data[2], 0U);
        return 4U;
    }

    if (transport_type == S7_PARAM_TYPE_BIT) {
        payload[0] = (src_base[byte_offset] & (1U << bit_index)) ? 0x01U : 0x00U;
    } else {
        for (uint16_t i = 0; i < byte_len; i++) {
            payload[i] = ((uint16_t)(byte_offset + i) < max_offset) ? src_base[byte_offset + i] : 0x00U;
        }
    }

item_done:;
    uint16_t total = (uint16_t)(4U + byte_len);
    if (!is_last_item && (byte_len & 1U) != 0U) {
        dst_data[total] = 0x00U;
        total++;
    }
    return total;
}

static bool ppi_is_forced(uint8_t s7200_area, uint8_t sub_area, uint16_t byte_offset)
{
    const s7200_force_entry_t *tbl = (const s7200_force_entry_t *)(uintptr_t)SRAM_FORCE_TABLE_ADDR;
    for (uint8_t i = 0; i < 16U; i++) {
        if (tbl[i].area == s7200_area &&
            (sub_area == 0U || tbl[i].sub_area == sub_area) &&
            tbl[i].byte_offset == byte_offset) {
            return true;
        }
    }
    return false;
}

static uint8_t ppi_write_var_item(
    const s7_any_item_t *item,
    const uint8_t       *item_data,
    uint16_t            *consumed_bytes
)
{
    uint8_t  transport_type = item->transport_type;
    uint16_t count          = ppi_read_be16(&item->count_hi);
    uint16_t db_num         = ppi_read_be16(&item->db_num_hi);
    uint8_t  area           = item->area;
    uint32_t bit_addr       = ((uint32_t)item->bit_addr_hi << 16) |
                              ((uint32_t)item->bit_addr_mid << 8) |
                              (uint32_t)item->bit_addr_lo;
    uint16_t byte_offset    = (uint16_t)(bit_addr >> 3);
    uint8_t  bit_mask       = (uint8_t)(1U << (bit_addr & 0x07U));

    uint8_t  ts_type  = item_data[1];
    uint16_t raw_len  = ppi_read_be16(&item_data[2]);
    uint16_t byte_len = (ts_type == S7_TS_BYTE_WORD_DWORD) ? (uint16_t)(raw_len >> 3) : raw_len;
    if (transport_type == S7_PARAM_TYPE_BIT) byte_len = 1U;
    const uint8_t *src = item_data + 4U;
    *consumed_bytes = (uint16_t)(4U + byte_len + (byte_len & 1U));

    switch (area) {
        case S7_AREA_I: {
            uint8_t *i_ram = MEM_PTR(SRAM_I_BASE_ADDR);
            const uint8_t *f_mask = CONST_MEM_PTR(SRAM_FORCE_I_MASK_ADDR);
            if (byte_offset >= 32U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (byte_offset < 16U && (f_mask[byte_offset] & bit_mask) != 0U) return S7_RET_OK;
                if (src[0] != 0U) i_ram[byte_offset] |= bit_mask;
                else              i_ram[byte_offset] &= (uint8_t)~bit_mask;
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 32U; i++) {
                    uint16_t off = (uint16_t)(byte_offset + i);
                    uint8_t m = (off < 16U) ? f_mask[off] : 0U;
                    i_ram[off] = (uint8_t)((i_ram[off] & m) | (src[i] & (uint8_t)~m));
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_Q: {
            uint8_t *q_ram = MEM_PTR(SRAM_Q_BASE_ADDR);
            const uint8_t *f_mask = CONST_MEM_PTR(SRAM_FORCE_Q_MASK_ADDR);
            if (byte_offset >= 32U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (byte_offset < 16U && (f_mask[byte_offset] & bit_mask) != 0U) return S7_RET_OK;
                if (src[0] != 0U) q_ram[byte_offset] |= bit_mask;
                else              q_ram[byte_offset] &= (uint8_t)~bit_mask;
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 32U; i++) {
                    uint16_t off = (uint16_t)(byte_offset + i);
                    uint8_t m = (off < 16U) ? f_mask[off] : 0U;
                    q_ram[off] = (uint8_t)((q_ram[off] & m) | (src[i] & (uint8_t)~m));
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_M: {
            uint8_t *m_ram = MEM_PTR(SRAM_M_BASE_ADDR);
            if (byte_offset >= 32U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (!ppi_is_forced(S7200_AREA_M, 0U, byte_offset)) {
                    if (src[0] != 0U) m_ram[byte_offset] |= bit_mask;
                    else              m_ram[byte_offset] &= (uint8_t)~bit_mask;
                }
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 32U; i++) {
                    if (!ppi_is_forced(S7200_AREA_M, 0U, (uint16_t)(byte_offset + i))) {
                        m_ram[byte_offset + i] = src[i];
                    }
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_S: {
            uint8_t *s_ram = MEM_PTR(SRAM_S_BASE_ADDR);
            if (byte_offset >= 32U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (src[0] != 0U) s_ram[byte_offset] |= bit_mask;
                else              s_ram[byte_offset] &= (uint8_t)~bit_mask;
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 32U; i++) {
                    s_ram[byte_offset + i] = src[i];
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_SM: {
            uint8_t *sm_ram = MEM_PTR(SRAM_SM_BASE_ADDR);
            if (byte_offset >= 550U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (src[0] != 0U) sm_ram[byte_offset] |= bit_mask;
                else              sm_ram[byte_offset] &= (uint8_t)~bit_mask;
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 550U; i++) {
                    sm_ram[byte_offset + i] = src[i];
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_AI: {
            uint8_t *ai_ram = MEM_PTR(SRAM_AI_BASE_ADDR);
            if (byte_offset >= 128U) return S7_RET_INVALID_ADDR;
            if (!ppi_is_forced(S7200_AREA_IO_PORT, S7_AREA_AI, byte_offset)) {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 128U; i++) {
                    ai_ram[byte_offset + i] = src[i];
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_AQ: {
            uint8_t *aq_ram = MEM_PTR(SRAM_AQ_BASE_ADDR);
            if (byte_offset >= 128U) return S7_RET_INVALID_ADDR;
            if (!ppi_is_forced(S7200_AREA_IO_PORT, S7_AREA_AQ, byte_offset)) {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 128U; i++) {
                    aq_ram[byte_offset + i] = src[i];
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_V_DB1: {
            uint8_t *v_ram = (db_num == 1U) ? MEM_PTR(SRAM_V_BASE_ADDR) : MEM_PTR(SRAM_DB1_HEADER_ADDR);
            uint16_t limit = (db_num == 1U) ? 0x297CU : 0x29A0U;
            if (byte_offset >= limit) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (!ppi_is_forced(S7200_AREA_V_DB1, 0U, byte_offset)) {
                    if (src[0] != 0U) v_ram[byte_offset] |= bit_mask;
                    else              v_ram[byte_offset] &= (uint8_t)~bit_mask;
                }
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < limit; i++) {
                    if (!ppi_is_forced(S7200_AREA_V_DB1, 0U, (uint16_t)(byte_offset + i))) {
                        v_ram[byte_offset + i] = src[i];
                    }
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_C: {
            uint16_t c_idx = (transport_type == S7_PARAM_TYPE_C) ? (uint16_t)bit_addr : (byte_offset >> 1);
            uint8_t  *c_stat = MEM_PTR(SRAM_C_STATUS_BITS_ADDR);
            uint16_t *c_val  = (uint16_t *)(uintptr_t)SRAM_C_CURRENT_VAL_ADDR;
            if (transport_type == S7_PARAM_TYPE_C && byte_len >= 3U) {
                for (uint16_t i = 0; i < count; i++) {
                    uint16_t idx = (uint16_t)((c_idx + i) & 0xFFU);
                    if (src[i * 3U] != 0U) c_stat[idx] |= 0x10U;
                    else                   c_stat[idx] &= (uint8_t)~0x10U;
                    c_val[idx] = ppi_read_be16(&src[i * 3U + 1U]);
                }
            } else if (byte_len >= 2U) {
                c_val[c_idx & 0xFFU] = ppi_read_be16(src);
            }
            return S7_RET_OK;
        }
        case S7_AREA_T: {
            uint16_t t_idx = (transport_type == S7_PARAM_TYPE_T) ? (uint16_t)bit_addr : (byte_offset >> 1);
            uint8_t  *t_stat = MEM_PTR(SRAM_T_STATUS_BITS_ADDR);
            uint16_t *t_val  = (uint16_t *)(uintptr_t)SRAM_T_CURRENT_VAL_ADDR;
            if (transport_type == S7_PARAM_TYPE_T && byte_len >= 5U) {
                for (uint16_t i = 0; i < count; i++) {
                    uint16_t idx = (uint16_t)((t_idx + i) & 0xFFU);
                    if (src[i * 5U] != 0U) t_stat[idx] |= 0x02U;
                    else                   t_stat[idx] &= (uint8_t)~0x02U;
                    t_val[idx] = ppi_read_be16(&src[i * 5U + 3U]);
                }
            } else if (byte_len >= 2U) {
                t_val[t_idx & 0xFFU] = ppi_read_be16(src);
            }
            return S7_RET_OK;
        }
        case S7_AREA_HC: {
            uint16_t hc_idx = (transport_type == S7_PARAM_TYPE_HC) ? (uint16_t)bit_addr : (byte_offset >> 2);
            uint32_t *hc_val = (uint32_t *)(uintptr_t)SRAM_HC_CURRENT_VAL_ADDR;
            if (byte_len >= 4U) {
                hc_val[hc_idx % 20U] = FUN_08003ff0(src);
            }
            return S7_RET_OK;
        }
        default:
            return S7_RET_INVALID_ADDR;
    }
}

static bool ppi_verify_password(const uint8_t *pw_in)
{
    const uint8_t *sdb_pw = CONST_MEM_PTR(SRAM_SDB_PASSWORD_ADDR);
    return ((uint8_t)(0xFFU - pw_in[0]) == sdb_pw[0]) &&
           ((uint8_t)(0xFFU - pw_in[1]) == sdb_pw[1]) &&
           (pw_in[2]                    == sdb_pw[2]) &&
           (pw_in[3]                    == sdb_pw[3]) &&
           ((uint8_t)(0xFFU - pw_in[4]) == sdb_pw[4]) &&
           ((uint8_t)(0xFFU - pw_in[5]) == sdb_pw[5]) &&
           (pw_in[6]                    == sdb_pw[6]) &&
           (pw_in[7]                    == sdb_pw[7]);
}

/* ============================================================================
 * 4. Main Reconstructed Entry Point: FUN_08017450 (0x08017450..0x08021787)
 * ============================================================================ */

void s7_ppi_serial_process_port(
    uint8_t   port_id,
    uint8_t  *rx_buf,
    uint16_t *rx_len_ptr,
    uint8_t  *tx_buf,
    uint16_t *tx_len_ptr,
    uint16_t *saved_tx_len,
    uint8_t  *rx_sd_state,
    uint8_t  *station_addr,
    uint8_t  *req_state
)
{
    /* Template for PLC-initiated S7comm 0x1B Download Block Job (30 bytes) */
    static const uint8_t s_ppi_dl_block_job_pdu[30] = {
        0x32U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x12U, 0x00U, 0x00U,
        0x1BU, 0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x09U,
        '_',   '0',   'B',   '0',   '0',   '0',   '0',   '0',   'P',
        0x00U, 0x16U
    };

    /* Template for PLC-initiated S7comm 0x1C Download Ended Job (30 bytes) */
    static const uint8_t s_ppi_dl_ended_job_pdu[30] = {
        0x32U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x12U, 0x00U, 0x00U,
        0x1CU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x09U,
        '_',   '0',   'B',   '0',   '0',   '0',   '0',   '0',   'P',
        0x00U, 0x16U
    };

    volatile uint8_t *port_poll_flag  = ppi_get_port_poll_flag(port_id);
    volatile uint8_t *port_auth_flag  = ppi_get_port_auth_flag(port_id);
    volatile uint8_t *port_route_byte = ppi_get_port_route_byte(port_id);

    uint8_t state = *req_state;

    /* ========================================================================
     * Phase 1 (req_state == 1): PPI SD1 (0x10) Short Frame / Token / Data Poll
     * (Firmware 0x08017488..0x08017b06)
     * ======================================================================== */
    if (state == PPI_REQ_STATE_SD1_POLL) {
        if (rx_buf[0] != PPI_SD1 || rx_buf[5] != PPI_ED ||
            rx_buf[4] != (uint8_t)(rx_buf[1] + rx_buf[2] + rx_buf[3])) {
            *req_state = PPI_REQ_STATE_IDLE;
            return;
        }

        /* 1A. Check if PLC is actively pulling block chunks (0x1B) from STEP 7-Micro/WIN */
        if (g_dl_active_block_type != S7_DL_BLOCK_NONE && *port_poll_flag == 1U) {
            *port_poll_flag = 0U;

            tx_buf[0] = PPI_SD2;
            tx_buf[1] = 0x1FU;
            tx_buf[2] = 0x1FU;
            tx_buf[3] = PPI_SD2;
            tx_buf[4] = (uint8_t)(rx_buf[2] & 0x7FU);
            tx_buf[5] = *station_addr;
            tx_buf[6] = PPI_FC_S7_REQ_ODD;
            memcpy(&tx_buf[7], s_ppi_dl_block_job_pdu, sizeof(s_ppi_dl_block_job_pdu));

            ppi_write_be16(&tx_buf[11], g_dl_chunk_seq_num);
            g_dl_chunk_seq_num++;

            if (g_dl_active_block_type == S7_DL_BLOCK_OB1) {
                tx_buf[28] = '8';
                tx_buf[33] = '1';
            } else if (g_dl_active_block_type == S7_DL_BLOCK_DB1) {
                tx_buf[28] = 'A';
                tx_buf[33] = '1';
            }

            tx_buf[35] = ppi_compute_fcs(&tx_buf[4], 0x1FU);
            tx_buf[36] = PPI_ED;
            *tx_len_ptr = 0x25U; /* 37 bytes total */
            *req_state  = PPI_REQ_STATE_IDLE;
            return;
        }

        /* 1B. Master Data Poll (FC == 0x5C): Transmit queued SD2 or SC response */
        if (rx_buf[3] == PPI_FC_DATA_POLL) {
            if (*rx_sd_state == PPI_SD2) {
                *tx_len_ptr  = *saved_tx_len;
                *rx_sd_state = 0U;
                if (g_dl_active_block_type != S7_DL_BLOCK_NONE) {
                    *port_poll_flag = 1U;
                }
            } else if (*rx_sd_state == PPI_SC) {
                tx_buf[1]    = PPI_SC;
                *tx_len_ptr  = 1U;
                *rx_sd_state = 0U;
            }

            /* Execute pending Flash erase/write/compile after final Download Ended Ack */
            if (g_dl_commit_pending_type != S7_DL_BLOCK_NONE) {
                ppi_commit_downloaded_block(g_dl_commit_pending_type);
                g_dl_commit_pending_type = S7_DL_BLOCK_NONE;
            }
        }
        /* 1C. FDL Station Status Poll (FC == 0x49): Respond with station status */
        else if (rx_buf[3] == PPI_FC_FDL_STATUS_REQ) {
            tx_buf[1] = PPI_SD1;
            tx_buf[2] = rx_buf[2];
            tx_buf[3] = rx_buf[1];
            tx_buf[4] = (rx_buf[2] < 0x80U) ? PPI_FC_RESP_STATUS_OK : PPI_FC_RESP_STATUS_MASTER;
            tx_buf[5] = (uint8_t)(tx_buf[2] + tx_buf[3] + tx_buf[4]);
            tx_buf[6] = PPI_ED;
            *tx_len_ptr = 6U;
        }

        *req_state = PPI_REQ_STATE_IDLE;
        return;
    }

    /* ========================================================================
     * Phase 2 (req_state == 2): PPI SD2 (0x68) Variable-Length S7comm Frame
     * (Firmware 0x08017b08..0x08021787)
     * ======================================================================== */
    if (state == PPI_REQ_STATE_SD2_DATA) {
        uint16_t rx_len = *rx_len_ptr;
        if (rx_len < 9U || rx_buf[0] != PPI_SD2 || rx_buf[3] != PPI_SD2 ||
            rx_buf[1] != rx_buf[2] || rx_buf[rx_len - 1U] != PPI_ED) {
            *req_state = PPI_REQ_STATE_IDLE;
            return;
        }

        uint8_t frame_le = rx_buf[1];
        if (ppi_compute_fcs(&rx_buf[4], frame_le) != rx_buf[rx_len - 2U]) {
            *req_state = PPI_REQ_STATE_IDLE;
            return;
        }

        /* Immediately set tx_buf[0] = 0xE5 (SC) so UART sends 1-byte ack */
        tx_buf[0]    = PPI_SC;
        *tx_len_ptr  = 1U;
        *rx_sd_state = PPI_SD2;

        uint8_t orig_da = rx_buf[4];
        uint8_t orig_sa = rx_buf[5];

        /* Strip 6-byte extended PG routing header if DA has bit 7 set (> 0x7F) */
        const uint8_t *s7_rx = &rx_buf[7];
        uint16_t s7_rx_len   = (uint16_t)(rx_len - 9U);
        if (orig_da > 0x7FU && s7_rx_len > 6U && rx_buf[7] != S7COMM_PROTO_ID) {
            *port_route_byte = rx_buf[11];
            s7_rx     += 6U;
            s7_rx_len -= 6U;
        }

        /* Deferred SD2 response frame is built starting at tx_buf[1..] */
        uint8_t *resp_frame = tx_buf + 1U;
        resp_frame[0] = PPI_SD2;
        resp_frame[3] = PPI_SD2;
        resp_frame[4] = (uint8_t)(orig_sa & 0x7FU);
        resp_frame[5] = *station_addr;
        resp_frame[6] = PPI_FC_S7_RESP_DATA;

        uint8_t *s7_tx = &resp_frame[7];
        uint16_t s7_tx_len = 0U;

        if (s7_rx_len >= 10U && s7_rx[0] == S7COMM_PROTO_ID) {
            uint8_t  rosctr    = s7_rx[1];
            uint16_t pdu_ref   = ppi_read_be16(&s7_rx[4]);
            uint16_t param_len = ppi_read_be16(&s7_rx[6]);
            uint16_t data_len  = ppi_read_be16(&s7_rx[8]);

            /* ----------------------------------------------------------------
             * 2A. ROSCTR == 0x03: Ack_Data from STEP 7-Micro/WIN during PLC-
             *     initiated Block Download (0x1B chunk payload)
             * ---------------------------------------------------------------- */
            if (rosctr == S7_ROSCTR_ACK_DATA && s7_rx_len >= 14U) {
                const uint8_t *ack_param = s7_rx + 12U;
                const uint8_t *ack_data  = ack_param + param_len;

                if (param_len >= 2U && ack_param[0] == S7_FUNC_DOWNLOAD_BLOCK) {
                    uint8_t more_data = ack_param[1];
                    if (data_len >= 4U) {
                        uint16_t chunk_len = ppi_read_be16(&ack_data[0]);
                        if (chunk_len > (uint16_t)(data_len - 4U)) {
                            chunk_len = (uint16_t)(data_len - 4U);
                        }
                        const uint8_t *chunk_bytes = ack_data + 4U;
                        uint16_t offset = g_block_xfer_offset;

                        if (g_dl_active_block_type == S7_DL_BLOCK_OB1) {
                            uint32_t f_addr = FLASH_OB1_RAW_BASE_ADDR + offset;
                            for (uint16_t i = 0; i < chunk_len; i += 2U) {
                                uint8_t b0 = chunk_bytes[i];
                                uint8_t b1 = ((uint16_t)(i + 1U) < chunk_len) ? chunk_bytes[i + 1U] : 0xFFU;
                                (void)FUN_08036ca0(f_addr + i, (uint16_t)(((uint16_t)b1 << 8) | b0));
                            }
                        } else if (g_dl_active_block_type == S7_DL_BLOCK_DB1) {
                            if ((uint32_t)offset + chunk_len <= 0x3D24U) {
                                memcpy(MEM_PTR(SRAM_DB1_HEADER_ADDR) + offset, chunk_bytes, chunk_len);
                            }
                        } else if (g_dl_active_block_type == S7_DL_BLOCK_SDB) {
                            if ((uint32_t)offset + chunk_len <= 0x2800U) {
                                memcpy(MEM_PTR(SRAM_V_BASE_ADDR) + offset, chunk_bytes, chunk_len);
                            }
                        }
                        g_block_xfer_offset = (uint16_t)(offset + chunk_len);
                    }

                    if (more_data != 0U) {
                        /* More chunks remain: respond with 0xE5 on poll and pull next 0x1B chunk */
                        *rx_sd_state    = PPI_SC;
                        *saved_tx_len   = 1U;
                        *port_poll_flag = 1U;
                        *req_state      = PPI_REQ_STATE_IDLE;
                        return;
                    } else {
                        /* Final chunk received: queue PLC-initiated 0x1C (Download Ended) Job */
                        resp_frame[6] = PPI_FC_S7_REQ_ODD;
                        memcpy(s7_tx, s_ppi_dl_ended_job_pdu, sizeof(s_ppi_dl_ended_job_pdu));
                        ppi_write_be16(&s7_tx[4], g_dl_chunk_seq_num);
                        if (g_dl_active_block_type == S7_DL_BLOCK_OB1) {
                            s7_tx[21] = '8';
                            s7_tx[26] = '1';
                            g_ob1_block_len = g_block_xfer_offset;
                        } else if (g_dl_active_block_type == S7_DL_BLOCK_DB1) {
                            s7_tx[21] = 'A';
                            s7_tx[26] = '1';
                            g_db1_block_len = g_block_xfer_offset;
                        } else if (g_dl_active_block_type == S7_DL_BLOCK_SDB) {
                            g_sdb_block_len = g_block_xfer_offset;
                        }
                        g_dl_commit_pending_type = g_dl_active_block_type;
                        g_dl_active_block_type   = S7_DL_BLOCK_NONE;
                        g_block_xfer_offset      = 0U;
                        s7_tx_len = 28U;
                    }
                }
            }
            /* ----------------------------------------------------------------
             * 2B. ROSCTR == 0x02: Simple Ack from STEP 7-Micro/WIN for 0x1C
             * ---------------------------------------------------------------- */
            else if (rosctr == S7_ROSCTR_ACK) {
                *rx_sd_state  = PPI_SC;
                *saved_tx_len = 1U;
                *req_state    = PPI_REQ_STATE_IDLE;
                return;
            }
            /* ----------------------------------------------------------------
             * 2C. ROSCTR == 0x01: S7comm Job Request from Master
             * ---------------------------------------------------------------- */
            else if (rosctr == S7_ROSCTR_JOB && param_len >= 1U) {
                const uint8_t *rx_param = s7_rx + 10U;
                const uint8_t *rx_data  = rx_param + param_len;
                uint8_t func_code = rx_param[0];

                s7_tx[0]  = S7COMM_PROTO_ID;
                s7_tx[1]  = S7_ROSCTR_ACK_DATA;
                s7_tx[2]  = 0x00U;
                s7_tx[3]  = 0x00U;
                ppi_write_be16(&s7_tx[4], pdu_ref);
                s7_tx[10] = 0x00U;
                s7_tx[11] = 0x00U;
                uint8_t *tx_param = s7_tx + 12U;

                switch (func_code) {
                    case S7_FUNC_SETUP_COMM: {
                        ppi_write_be16(&s7_tx[6], 8U);
                        ppi_write_be16(&s7_tx[8], 0U);
                        tx_param[0] = S7_FUNC_SETUP_COMM;
                        tx_param[1] = 0x00U;
                        tx_param[2] = 0x00U;
                        tx_param[3] = 0x01U;
                        tx_param[4] = 0x00U;
                        tx_param[5] = 0x01U;
                        ppi_write_be16(&tx_param[6], 0x00F0U);
                        s7_tx_len = 20U;
                        break;
                    }
                    case S7_FUNC_PLC_STOP: {
                        g_plc_run_mode           = 0U;
                        g_plc_prev_run_mode      = 0U;
                        g_status_monitor_trigger = 0U;
                        FUN_0802b8e8();
                        ppi_write_be16(&s7_tx[6], 1U);
                        ppi_write_be16(&s7_tx[8], 0U);
                        tx_param[0] = S7_FUNC_PLC_STOP;
                        s7_tx_len = 13U;
                        break;
                    }
                    case S7_FUNC_PI_SERVICE: {
                        bool is_run = false;
                        for (uint16_t i = 0; i + 8U < param_len; i++) {
                            if (memcmp(&rx_param[i], "P_PROGRAM", 9U) == 0) {
                                is_run = true;
                                break;
                            }
                        }
                        if (is_run) {
                            FUN_080219c8();
                            if (g_ob1_compile_err_code == 0U && g_plc_hw_fault_flag == 0U) {
                                g_plc_run_init_flag = 1U;
                                g_plc_run_mode      = 1U;
                            } else {
                                s7_tx[10] = 0x82U;
                                s7_tx[11] = 0x04U;
                            }
                        }
                        ppi_write_be16(&s7_tx[6], 1U);
                        ppi_write_be16(&s7_tx[8], 0U);
                        tx_param[0] = S7_FUNC_PI_SERVICE;
                        s7_tx_len = 13U;
                        break;
                    }
                    case S7_FUNC_READ_VAR: {
                        uint8_t item_count = (param_len >= 2U) ? rx_param[1] : 0U;
                        tx_param[0] = S7_FUNC_READ_VAR;
                        tx_param[1] = item_count;
                        uint8_t *tx_data = tx_param + 2U;
                        uint16_t out_data_len = 0U;
                        const uint8_t *item_ptr = rx_param + 2U;

                        for (uint8_t i = 0; i < item_count; i++) {
                            bool is_last = (i == (uint8_t)(item_count - 1U));
                            if (item_ptr[0] == 0x12U && item_ptr[1] == 0x0AU && item_ptr[2] == 0x10U) {
                                uint16_t w = ppi_read_var_item(
                                    (const s7_any_item_t *)item_ptr,
                                    tx_data + out_data_len,
                                    is_last
                                );
                                out_data_len = (uint16_t)(out_data_len + w);
                                item_ptr += 12U;
                            } else {
                                uint8_t step = (uint8_t)(item_ptr[1] + 2U);
                                tx_data[out_data_len + 0U] = S7_RET_OK;
                                tx_data[out_data_len + 1U] = S7_TS_OCTET_STRING;
                                ppi_write_be16(&tx_data[out_data_len + 2U], 1U);
                                tx_data[out_data_len + 4U] = g_plc_run_mode;
                                out_data_len = (uint16_t)(out_data_len + (is_last ? 5U : 6U));
                                item_ptr += step;
                            }
                        }
                        ppi_write_be16(&s7_tx[6], 2U);
                        ppi_write_be16(&s7_tx[8], out_data_len);
                        s7_tx_len = (uint16_t)(14U + out_data_len);
                        break;
                    }
                    case S7_FUNC_WRITE_VAR: {
                        uint8_t priv_level = MEM_U8(SRAM_SDB_PRIV_LEVEL_ADDR);
                        if (priv_level >= 3U && *port_auth_flag == 0U) {
                            s7_tx[10] = 0x85U;
                            s7_tx[11] = 0x00U;
                            ppi_write_be16(&s7_tx[6], 0U);
                            ppi_write_be16(&s7_tx[8], 0U);
                            s7_tx_len = 12U;
                            break;
                        }
                        uint8_t item_count = (param_len >= 2U) ? rx_param[1] : 0U;
                        tx_param[0] = S7_FUNC_WRITE_VAR;
                        tx_param[1] = item_count;
                        uint8_t *tx_data = tx_param + 2U;
                        const uint8_t *item_ptr = rx_param + 2U;
                        const uint8_t *val_ptr  = rx_data;
                        for (uint8_t i = 0; i < item_count; i++) {
                            uint16_t consumed = 0U;
                            if (item_ptr[0] == 0x12U && item_ptr[1] == 0x0AU && item_ptr[2] == 0x10U) {
                                tx_data[i] = ppi_write_var_item(
                                    (const s7_any_item_t *)item_ptr,
                                    val_ptr,
                                    &consumed
                                );
                                item_ptr += 12U;
                                val_ptr  += consumed;
                            } else {
                                tx_data[i] = S7_RET_OK;
                            }
                        }
                        ppi_write_be16(&s7_tx[6], 2U);
                        ppi_write_be16(&s7_tx[8], item_count);
                        s7_tx_len = (uint16_t)(14U + item_count);
                        break;
                    }
                    case S7_FUNC_REQ_DOWNLOAD:
                    case S7_FUNC_REQ_DOWNLOAD_EXT: {
                        g_plc_prev_run_mode = g_plc_run_mode;
                        g_plc_run_mode      = 0U;
                        g_block_xfer_offset = 0U;
                        g_dl_chunk_seq_num  = 1U;

                        if (param_len >= 18U) {
                            if (memcmp(&rx_param[10], "0B00000P", 8U) == 0) {
                                g_dl_active_block_type = S7_DL_BLOCK_SDB;
                            } else if (memcmp(&rx_param[10], "0A00001P", 8U) == 0) {
                                g_dl_active_block_type = S7_DL_BLOCK_DB1;
                            } else {
                                g_dl_active_block_type = S7_DL_BLOCK_OB1;
                                for (uint32_t page = FLASH_OB1_RAW_BASE_ADDR; page <= 0x0807D000UL; page += 0x0800UL) {
                                    FUN_08036bc8(page);
                                }
                            }
                        }
                        ppi_write_be16(&s7_tx[6], 1U);
                        ppi_write_be16(&s7_tx[8], 0U);
                        tx_param[0] = func_code;
                        s7_tx_len = 13U;
                        break;
                    }
                    case S7_FUNC_START_UPLOAD: {
                        g_block_xfer_offset = 0U;
                        uint16_t blk_len = g_ob1_block_len;
                        if (param_len >= 16U) {
                            if (memcmp(&rx_param[10], "0A00001P", 8U) == 0) {
                                blk_len = g_db1_block_len;
                                g_dl_active_block_type = S7_DL_BLOCK_DB1;
                            } else if (memcmp(&rx_param[10], "0B00000P", 8U) == 0) {
                                blk_len = g_sdb_block_len;
                                g_dl_active_block_type = S7_DL_BLOCK_SDB;
                            } else {
                                blk_len = g_ob1_block_len;
                                g_dl_active_block_type = S7_DL_BLOCK_OB1;
                            }
                        }
                        ppi_write_be16(&s7_tx[6], 16U);
                        ppi_write_be16(&s7_tx[8], 0U);
                        tx_param[0] = S7_FUNC_START_UPLOAD;
                        memset(&tx_param[1], 0, 7U);
                        tx_param[7] = 0x07U;
                        uint16_t rem = blk_len;
                        for (int d = 5; d >= 1; d--) {
                            tx_param[8 + d] = (uint8_t)('0' + (rem % 10U));
                            rem /= 10U;
                        }
                        tx_param[8]  = '0';
                        tx_param[14] = 'P';
                        tx_param[15] = 0x00U;
                        s7_tx_len = 28U;
                        break;
                    }
                    case S7_FUNC_UPLOAD: {
                        const uint8_t *blk_base = CONST_MEM_PTR(FLASH_OB1_RAW_BASE_ADDR);
                        uint16_t total_len = g_ob1_block_len;
                        if (g_dl_active_block_type == S7_DL_BLOCK_DB1) {
                            blk_base  = CONST_MEM_PTR(FLASH_DB1_BASE_ADDR);
                            total_len = g_db1_block_len;
                        } else if (g_dl_active_block_type == S7_DL_BLOCK_SDB) {
                            blk_base  = CONST_MEM_PTR(FLASH_SDB_BASE_ADDR);
                            total_len = g_sdb_block_len;
                        }
                        uint16_t offset = g_block_xfer_offset;
                        uint16_t rem = (offset < total_len) ? (uint16_t)(total_len - offset) : 0U;
                        uint16_t chunk_len = (rem > 222U) ? 222U : rem;
                        tx_param[0] = S7_FUNC_UPLOAD;
                        tx_param[1] = (rem > chunk_len) ? 0x01U : 0x00U;
                        uint8_t *tx_data = tx_param + 2U;
                        ppi_write_be16(&tx_data[0], chunk_len);
                        tx_data[2] = 0x00U;
                        tx_data[3] = 0xFBU;
                        if (chunk_len > 0U) {
                            memcpy(&tx_data[4], blk_base + offset, chunk_len);
                        }
                        g_block_xfer_offset = (uint16_t)(offset + chunk_len);
                        ppi_write_be16(&s7_tx[6], 2U);
                        ppi_write_be16(&s7_tx[8], (uint16_t)(chunk_len + 4U));
                        s7_tx_len = (uint16_t)(18U + chunk_len);
                        break;
                    }
                    case S7_FUNC_END_UPLOAD: {
                        g_block_xfer_offset    = 0U;
                        g_dl_active_block_type = S7_DL_BLOCK_NONE;
                        ppi_write_be16(&s7_tx[6], 1U);
                        ppi_write_be16(&s7_tx[8], 0U);
                        tx_param[0] = S7_FUNC_END_UPLOAD;
                        s7_tx_len = 13U;
                        break;
                    }
                    default:
                        break;
                }
            }
            /* ----------------------------------------------------------------
             * 2D. ROSCTR == 0x07: Userdata (CPU ID, Password, RTC, Force, Status)
             * ---------------------------------------------------------------- */
            else if (rosctr == S7_ROSCTR_USERDATA) {
                const uint8_t *rx_param = s7_rx + 10U;
                const uint8_t *rx_data  = rx_param + param_len;

                s7_tx[0] = S7COMM_PROTO_ID;
                s7_tx[1] = S7_ROSCTR_USERDATA;
                s7_tx[2] = 0x00U;
                s7_tx[3] = 0x00U;
                ppi_write_be16(&s7_tx[4], pdu_ref);

                uint16_t out_param_len = (param_len <= 12U) ? param_len : 12U;
                uint8_t *ud_param = s7_tx + 10U;
                memcpy(ud_param, rx_param, out_param_len);
                uint8_t *ud_data = ud_param + out_param_len;

                uint8_t grp = (param_len >= 6U) ? rx_param[5] : 0U;
                uint8_t cmd = (param_len >= 7U) ? rx_param[6] : 0U;

                if ((grp == 0x01U && cmd == 0x01U) ||
                    (data_len >= 6U && rx_data[4] == 0x00U && rx_data[5] == 0x11U)) {
                    /* CPU Identification ("CPU 226   CN    02010100") */
                    ud_data[0] = S7_RET_OK;
                    ud_data[1] = S7_TS_OCTET_STRING;
                    ppi_write_be16(&ud_data[2], 40U);
                    memcpy(&ud_data[4], CONST_MEM_PTR(FLASH_CPU_ID_STRING_ADDR), 24U);
                    ppi_write_be16(&ud_data[28], g_ob1_block_len);
                    ppi_write_be16(&ud_data[30], g_db1_block_len);
                    ppi_write_be16(&ud_data[32], g_sdb_block_len);
                    ppi_write_be16(&ud_data[34], 0x2800U);
                    ud_data[36] = g_plc_run_mode;
                    ud_data[37] = MEM_U8(SRAM_SDB_PRIV_LEVEL_ADDR);
                    ppi_write_be16(&ud_data[38], g_ob1_compile_err_code);
                    ppi_write_be16(&ud_data[40], 0x0000U);
                    ppi_write_be16(&s7_tx[6], out_param_len);
                    ppi_write_be16(&s7_tx[8], 44U);
                    s7_tx_len = (uint16_t)(10U + out_param_len + 44U);
                } else if (grp == 0x05U || (param_len >= 8U && rx_param[7] == 0x57U)) {
                    /* Password Verification */
                    if (data_len >= 12U && ppi_verify_password(&rx_data[4])) {
                        *port_auth_flag = 1U;
                        ud_data[0] = S7_RET_OK;
                        ud_data[1] = S7_TS_OCTET_STRING;
                    } else {
                        ud_data[0] = S7_RET_ACCESS_FAULT;
                        ud_data[1] = S7_TS_NULL;
                        if (out_param_len >= 12U) {
                            ud_param[10] = 0xD6U;
                            ud_param[11] = 0x02U;
                        }
                    }
                    ppi_write_be16(&ud_data[2], 0U);
                    ppi_write_be16(&s7_tx[6], out_param_len);
                    ppi_write_be16(&s7_tx[8], 4U);
                    s7_tx_len = (uint16_t)(10U + out_param_len + 4U);
                } else if (grp == 0x07U) {
                    /* Real-Time Clock Read / Write */
                    if (cmd == 0x01U) {
                        ud_data[0]  = S7_RET_OK;
                        ud_data[1]  = S7_TS_OCTET_STRING;
                        ppi_write_be16(&ud_data[2], 8U);
                        ud_data[4]  = 0x00U;
                        ud_data[5]  = g_rtc_bcd_year;
                        ud_data[6]  = g_rtc_bcd_month;
                        ud_data[7]  = g_rtc_bcd_day;
                        ud_data[8]  = g_rtc_bcd_hour;
                        ud_data[9]  = g_rtc_bcd_min;
                        ud_data[10] = g_rtc_bcd_sec;
                        ud_data[11] = g_rtc_bcd_dow;
                        ppi_write_be16(&s7_tx[6], out_param_len);
                        ppi_write_be16(&s7_tx[8], 12U);
                        s7_tx_len = (uint16_t)(10U + out_param_len + 12U);
                    } else {
                        if (data_len >= 12U) {
                            uint8_t *rtc_stage = g_rtc_write_buf;
                            for (uint8_t i = 0; i < 7U; i++) {
                                rtc_stage[i] = rx_data[5U + i];
                            }
                            FUN_0802fd3c();
                        }
                        ud_data[0] = S7_RET_OK;
                        ud_data[1] = S7_TS_NULL;
                        ppi_write_be16(&ud_data[2], 0U);
                        ppi_write_be16(&s7_tx[6], out_param_len);
                        ppi_write_be16(&s7_tx[8], 4U);
                        s7_tx_len = (uint16_t)(10U + out_param_len + 4U);
                    }
                } else if (grp == 0x02U) {
                    /* Online Status Monitoring */
                    if (cmd == 0x01U && data_len > 4U) {
                        uint16_t cfg_len = (uint16_t)(data_len - 4U);
                        if (cfg_len > 500U) cfg_len = 500U;
                        memcpy(MEM_PTR(SRAM_STATUS_CFG_BUF_ADDR), rx_data + 4U, cfg_len);
                        g_status_monitor_trigger = 1U;
                    }
                    uint16_t snap_len = g_status_resp_len;
                    if (snap_len > 220U) snap_len = 220U;
                    ud_data[0] = S7_RET_OK;
                    ud_data[1] = S7_TS_OCTET_STRING;
                    ppi_write_be16(&ud_data[2], snap_len);
                    if (snap_len > 0U) {
                        memcpy(&ud_data[4], CONST_MEM_PTR(SRAM_STATUS_RESP_BUF_ADDR), snap_len);
                    }
                    ppi_write_be16(&s7_tx[6], out_param_len);
                    ppi_write_be16(&s7_tx[8], (uint16_t)(4U + snap_len));
                    s7_tx_len = (uint16_t)(10U + out_param_len + 4U + snap_len);
                } else {
                    ud_data[0] = S7_RET_OK;
                    ud_data[1] = S7_TS_OCTET_STRING;
                    ppi_write_be16(&ud_data[2], 0U);
                    ppi_write_be16(&s7_tx[6], out_param_len);
                    ppi_write_be16(&s7_tx[8], 4U);
                    s7_tx_len = (uint16_t)(10U + out_param_len + 4U);
                }
            }
        }

        if (s7_tx_len > 0U) {
            uint8_t out_le = (uint8_t)(s7_tx_len + 3U);
            resp_frame[1] = out_le;
            resp_frame[2] = out_le;
            resp_frame[7U + s7_tx_len] = ppi_compute_fcs(&resp_frame[4], out_le);
            resp_frame[8U + s7_tx_len] = PPI_ED;
            *saved_tx_len = (uint16_t)(s7_tx_len + 9U);

            /* ================================================================
             * LAB_080216d8: Extended PG / Multi-Master Routing Header Insertion
             * If incoming DA had bit 7 set (orig_da > 0x7F), shift the S7comm
             * payload right by 6 bytes and prepend the 6-byte routing header.
             * ================================================================ */
            if (orig_da > 0x7FU) {
                for (uint16_t i = *saved_tx_len; i > 7U; i--) {
                    resp_frame[i + 5U] = resp_frame[i - 1U];
                }
                resp_frame[1]  = (uint8_t)(resp_frame[1] + 6U);
                resp_frame[2]  = (uint8_t)(resp_frame[2] + 6U);
                resp_frame[5]  = (uint8_t)(*station_addr + 0x80U);
                resp_frame[7]  = 0x00U;
                resp_frame[8]  = 0x04U;
                resp_frame[9]  = 0x00U;
                resp_frame[10] = 0x80U;
                resp_frame[11] = 0x09U;
                resp_frame[12] = *port_route_byte;
                *saved_tx_len  = (uint16_t)(*saved_tx_len + 6U);

                uint8_t ext_le = resp_frame[1];
                resp_frame[*saved_tx_len - 2U] = ppi_compute_fcs(&resp_frame[4], ext_le);
                resp_frame[*saved_tx_len - 1U] = PPI_ED;
            }
        }

        *req_state = PPI_REQ_STATE_IDLE;
    }
}

void FUN_08017450(
    uint8_t   param_1,
    uint8_t  *param_2,
    uint16_t *param_3,
    uint8_t  *param_4,
    uint16_t *param_5,
    uint16_t *param_6,
    uint8_t  *param_7,
    uint8_t  *param_8,
    uint8_t  *param_9
)
{
    s7_ppi_serial_process_port(param_1, param_2, param_3, param_4, param_5, param_6, param_7, param_8, param_9);
}
