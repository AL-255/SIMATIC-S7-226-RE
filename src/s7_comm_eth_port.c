/**
 * @file s7_comm_eth_port.c
 * @brief SIMATIC S7-226 (APM32E103) Ethernet ISO-on-TCP (RFC 1006) S7comm &
 *        Bridged PPI Protocol Engine.
 *
 * Clean, modular C reconstruction of firmware function:
 *   - FUN_080047ac (0x080047ac..0x0800fb4d, 45,966 bytes)
 *   - FUN_08004300 (0x08004300..0x080043b5, 182 bytes: OB1 nibble-XOR de-obfuscation)
 *
 * Supported Protocol Layers & Services:
 *   1. RFC 1006 TPKT (0x03 0x00 len_hi len_lo) + ISO 8073 COTP:
 *      - COTP Connection Request (0xE0 CR) -> Connection Confirm (0xD0 CC)
 *      - COTP Data Transfer (0xF0 DT) -> S7comm PDU (0x32)
 *   2. Bridged PPI Serial Frames (req_state == 1 for SD1 0x10, req_state == 2 for SD2 0x68)
 *   3. S7comm Job Services (ROSCTR == 0x01):
 *      - 0xF0 : Setup Communication (Negotiate PDU length)
 *      - 0x04 : Read Variable (I, Q, M, V/DB1, S, SM, AI, AQ, T, C, HC, System Status)
 *      - 0x05 : Write Variable (Bit, Byte, Word, DWord with active Force-Mask protection)
 *      - 0x1A / 0xFA : Request Block Download (OB1, DB1, SDB)
 *      - 0x1B / 0xFB : Download Block Chunk (Flash/RAM staging)
 *      - 0x1C / 0xFC : Download Ended (Flash commit, de-obfuscation, checksums, compile)
 *      - 0x1D : Start Block Upload (OB1 0x08075800, DB1 0x08064800, SDB 0x0807f800)
 *      - 0x1E : Upload Block Chunk
 *      - 0x1F : End Block Upload
 *      - 0x28 : PI Service ("P_PROGRAM" RUN mode transition, Block Link 'I'/'D', "_GARB")
 *      - 0x29 : PLC Stop ("P_STOP" transition to STOP mode)
 *   4. S7comm Userdata Services (ROSCTR == 0x07):
 *      - CPU Identification ("CPU 226   CN    02010100") & System Status / SZL
 *      - Block Directory & Block Attribute Info (_0800001P, _0A00001P, _0B00000P)
 *      - Password Authentication & Session Privilege (0x08068524 / 0x2000596A)
 *      - Real-Time Clock (RTC) Read & Write (BCD Year..DayOfWeek)
 *      - Force Variables (Force Bit/Byte/Word/DWord, Unforce Single, Unforce All, Read Force Table)
 *      - Online Status Monitoring (Chart Status / Program Status via 0x20005C20 & 0x20005570)
 *      - Clear PLC Memory (Selective / Full Reset of OB1, DB1, SDB, and RAM areas)
 */

#include "s7_ppi_comm.h"

/* ============================================================================
 * External Firmware Hardware & Runtime Helpers
 * ============================================================================ */

#if defined(S7_FIRMWARE_BUILD)
extern void     FUN_08036bc8(uint32_t flash_page_addr);              /* Flash erase 2KB page */
extern int      FUN_08036ca0(uint32_t flash_addr, uint16_t halfword);/* Flash program 16-bit halfword */
extern void     FUN_080219c8(void);                                  /* Validate & compile OB1 user program */
extern void     FUN_0802b8e8(void);                                  /* Reset runtime I/O & timers on STOP/Clear */
extern void     FUN_0802bd58(void);                                  /* Apply Port 0 baudrate from SDB */
extern void     FUN_0802bd88(void);                                  /* Apply Port 1 baudrate from SDB */
extern void     FUN_0802c010(uint8_t baud_code);                     /* Configure USART1 baudrate */
extern void     FUN_0802c060(uint8_t baud_code);                     /* Configure USART2 baudrate */
extern void     FUN_0802fd3c(void);                                  /* Commit staged RTC registers to hardware */
extern void     FUN_08032998(uint16_t delay_ms);                     /* Watchdog-safe millisecond delay */
extern uint32_t FUN_08003ff0(const uint8_t *buf);                    /* Read 32-bit big-endian value */
extern uint16_t FUN_08004198(const uint8_t *buf);                    /* Parse S7comm item bit/byte offset */
extern uint16_t FUN_0800422c(uint8_t type_code, uint16_t count);     /* Compute byte length for S7 transport type */
#else
/* Standalone / Host-compilation weak stubs for hardware primitives */
__attribute__((weak)) void     FUN_08036bc8(uint32_t flash_page_addr) { (void)flash_page_addr; }
__attribute__((weak)) int      FUN_08036ca0(uint32_t flash_addr, uint16_t halfword) { (void)flash_addr; (void)halfword; return 4; }
__attribute__((weak)) void     FUN_080219c8(void) {}
__attribute__((weak)) void     FUN_0802b8e8(void) {}
__attribute__((weak)) void     FUN_0802bd58(void) {}
__attribute__((weak)) void     FUN_0802bd88(void) {}
__attribute__((weak)) void     FUN_0802c010(uint8_t baud_code) { (void)baud_code; }
__attribute__((weak)) void     FUN_0802c060(uint8_t baud_code) { (void)baud_code; }
__attribute__((weak)) void     FUN_0802fd3c(void) {}
__attribute__((weak)) void     FUN_08032998(uint16_t delay_ms) { (void)delay_ms; }
__attribute__((weak)) uint32_t FUN_08003ff0(const uint8_t *buf) {
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8)  | (uint32_t)buf[3];
}
__attribute__((weak)) uint16_t FUN_08004198(const uint8_t *buf) {
    uint32_t bit_addr = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    return (uint16_t)(bit_addr >> 3);
}
__attribute__((weak)) uint16_t FUN_0800422c(uint8_t type_code, uint16_t count) {
    switch (type_code) {
        case S7_PARAM_TYPE_BIT:
        case S7_PARAM_TYPE_BYTE:
        case S7_PARAM_TYPE_CHAR:  return count;
        case S7_PARAM_TYPE_WORD:
        case S7_PARAM_TYPE_INT:   return (uint16_t)(count * 2U);
        case S7_PARAM_TYPE_DWORD:
        case S7_PARAM_TYPE_DINT:
        case S7_PARAM_TYPE_REAL:
        case S7_PARAM_TYPE_HC:    return (uint16_t)(count * 4U);
        case S7_PARAM_TYPE_C:     return (uint16_t)(count * 3U);
        case S7_PARAM_TYPE_T:     return (uint16_t)(count * 5U);
        default:                  return count;
    }
}
#endif

/* ============================================================================
 * Memory-Mapped Firmware Globals & Area Pointers (APM32E103 SRAM / Flash)
 * ============================================================================ */

#define MEM_U8(addr)                    (*(volatile uint8_t  *)(uintptr_t)(addr))
#define MEM_U16(addr)                   (*(volatile uint16_t *)(uintptr_t)(addr))
#define MEM_U32(addr)                   (*(volatile uint32_t *)(uintptr_t)(addr))
#define MEM_PTR(addr)                   ((uint8_t *)(uintptr_t)(addr))
#define CONST_MEM_PTR(addr)             ((const uint8_t *)(uintptr_t)(addr))

/** Firmware Runtime State Variables in SRAM (0x20000000..0x20000900) */
#define g_plc_run_init_flag             MEM_U8(0x20000008UL)  /* DAT_20000008: First-scan init flag */
#define g_plc_run_mode                  MEM_U8(0x20000009UL)  /* DAT_20000009: 1 = RUN, 0 = STOP */
#define g_plc_prev_run_mode             MEM_U8(0x20000011UL)  /* DAT_20000011: Saved RUN mode during download */
#define g_plc_hw_fault_flag             MEM_U8(0x20000017UL)  /* DAT_20000017: Hardware / fatal error flag */
#define g_rtc_bcd_year                  MEM_U8(0x2000001CUL)  /* DAT_2000001c: Current RTC Year (BCD) */
#define g_rtc_bcd_month                 MEM_U8(0x2000001DUL)  /* DAT_2000001d: Current RTC Month (BCD) */
#define g_rtc_bcd_day                   MEM_U8(0x2000001EUL)  /* DAT_2000001e: Current RTC Day (BCD) */
#define g_rtc_bcd_hour                  MEM_U8(0x2000001FUL)  /* DAT_2000001f: Current RTC Hour (BCD) */
#define g_rtc_bcd_min                   MEM_U8(0x20000020UL)  /* DAT_20000020: Current RTC Minute (BCD) */
#define g_rtc_bcd_sec                   MEM_U8(0x20000021UL)  /* DAT_20000021: Current RTC Second (BCD) */
#define g_rtc_bcd_dow                   MEM_U8(0x20000023UL)  /* DAT_20000023: Current RTC Day of Week (1..7) */
#define g_rtc_write_buf                 MEM_PTR(0x20000024UL) /* DAT_20000024..0x2000002A: Staged RTC write regs */
#define g_ppi_port0_station_addr        MEM_U8(0x20000030UL)  /* DAT_20000030: Port 0 station address */
#define g_ppi_port1_station_addr        MEM_U8(0x20000031UL)  /* DAT_20000031: Port 1 station address */
#define g_eth_bridged_routing_hdr       MEM_PTR(0x20000034UL) /* DAT_20000034: Bridged extended routing byte */

/** Block Download / Upload / Status Engine State in SRAM (0x20000480..0x20000810) */
#define g_dl_commit_pending_type        MEM_U8(0x20000484UL)  /* DAT_20000484: 1=SDB, 2=OB1, 3=DB1 flash commit */
#define g_dl_active_block_type          MEM_U8(0x20000487UL)  /* DAT_20000487: 1=SDB, 2=OB1, 3=DB1 download active */
#define g_dl_eth_poll_ready             MEM_U8(0x2000048BUL)  /* DAT_2000048b: Poll ready to emit 0x1B job */
#define g_dl_chunk_seq_num              MEM_U16(0x2000048EUL) /* DAT_2000048e: Download block sequence counter */
#define g_block_xfer_offset             MEM_U16(0x200004C0UL) /* DAT_200004c0: Current upload/download byte offset */
#define g_status_resp_len               MEM_U16(0x200004C2UL) /* DAT_200004c2: Online status response byte length */
#define g_ob1_block_len                 MEM_U16(0x200004C6UL) /* DAT_200004c6: Stored OB1 raw block length */
#define g_sdb_block_len                 MEM_U16(0x200004C8UL) /* DAT_200004c8: Stored SDB block length */
#define g_db1_block_len                 MEM_U16(0x200004CAUL) /* DAT_200004ca: Stored DB1 block length */
#define g_ob1_compile_err_code          MEM_U16(0x200004CCUL) /* DAT_200004cc: User program compile status (0=OK) */
#define g_force_table_count             MEM_U8(0x200004CEUL)  /* DAT_200004ce: Active entries in 0x20005AF0 */
#define g_force_io_bit_active           MEM_U8(0x200004CFUL)  /* DAT_200004cf: Non-zero if any I/Q bit is forced */
#define g_password_auth_ok_eth          MEM_U8(0x200004D2UL)  /* DAT_200004d2: Session password unlocked on Eth */
#define g_status_monitor_trigger        MEM_U8(0x2000080DUL)  /* DAT_2000080d: 1=Arm status monitor, 2=Sampled */

/* ============================================================================
 * Helper Functions: Endianness & Block De-obfuscation
 * ============================================================================ */

static inline uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline void write_be16(uint8_t *p, uint16_t val)
{
    p[0] = (uint8_t)(val >> 8);
    p[1] = (uint8_t)(val & 0xFFU);
}

static inline void write_be32(uint8_t *p, uint32_t val)
{
    p[0] = (uint8_t)(val >> 24);
    p[1] = (uint8_t)(val >> 16);
    p[2] = (uint8_t)(val >> 8);
    p[3] = (uint8_t)(val & 0xFFU);
}

/**
 * @brief Computes the 1-byte PPI Frame Check Sequence (FCS = sum of bytes mod 256).
 */
static uint8_t ppi_calc_fcs(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

/**
 * @brief De-obfuscates a downloaded OB1 block payload using the 16x16 nibble XOR lookup
 *        table at 0x0804A0F4 and writes the decoded halfwords into Flash at *flash_write_addr.
 *        Reconstructed from FUN_08004300 (0x08004300..0x080043b5).
 */
void s7_block_deobfuscate_ob1(const uint8_t *src_buf, uint16_t length, uint32_t *flash_write_addr)
{
    const uint8_t *xor_table = CONST_MEM_PTR(FLASH_OB1_XOR_TABLE_ADDR);
    uint8_t decoded_pair[2] = {0, 0};

    for (uint16_t i = 0; i < length; i++) {
        uint8_t raw_byte = src_buf[i];
        uint8_t row = (uint8_t)(raw_byte >> 4);
        uint8_t col = (uint8_t)(raw_byte & 0x0FU);
        uint8_t xored = xor_table[(row * 16U) + col] ^ raw_byte;

        /* Swap nibbles after table XOR */
        decoded_pair[i & 1U] = (uint8_t)((xored << 4) | (xored >> 4));

        if ((i & 1U) == 1U) {
            uint16_t halfword = (uint16_t)(((uint16_t)decoded_pair[1] << 8) | decoded_pair[0]);
            int status = FUN_08036ca0(*flash_write_addr, halfword);
            if (status != 4) {
                break;
            }
            *flash_write_addr += 2U;
        }
    }
}

void FUN_08004300(const uint8_t *param_1, uint16_t param_2, uint32_t *param_3)
{
    s7_block_deobfuscate_ob1(param_1, param_2, param_3);
}

/* ============================================================================
 * Flash Block Programming & Checksum Commit Helpers
 * ============================================================================ */

/**
 * @brief Computes the 16-bit Fletcher-like checksum over a memory buffer (matching FUN_08021794)
 *        and writes the 2-byte checksum to Flash at target_flash_addr.
 */
static void s7_flash_write_fletcher_checksum(const uint8_t *data, uint16_t len, uint32_t target_flash_addr)
{
    uint8_t sum1 = 0;
    uint8_t sum2 = 0;

    for (uint16_t i = 0; i < len; i++) {
        uint16_t temp = (uint16_t)sum1 + (uint16_t)data[i];
        if (temp > 0xFFU) {
            temp = (temp & 0xFFU) + 1U;
        }
        sum1 = (uint8_t)temp;

        uint16_t temp2 = (uint16_t)sum2 + (uint16_t)sum1;
        if (temp2 > 0xFFU) {
            temp2 = (temp2 & 0xFFU) + 1U;
        }
        sum2 = (uint8_t)temp2;
    }

    uint8_t chk_lo = (uint8_t)(0xFFU - ((uint16_t)(sum1 + sum2) % 0xFFU));
    uint8_t chk_hi = (uint8_t)(0xFFU - ((uint16_t)(sum1 + chk_lo) % 0xFFU));
    uint16_t chk_word = (uint16_t)(((uint16_t)chk_hi << 8) | chk_lo);
    (void)FUN_08036ca0(target_flash_addr, chk_word);
}

/**
 * @brief Computes the 16-bit weighted checksum over the compiled OB1 Flash region
 *        (matching FUN_080218a0) and writes it at target_flash_addr.
 */
static void s7_flash_write_weighted_checksum(const uint8_t *data, uint16_t len, uint32_t target_flash_addr)
{
    uint8_t acc_xor_add = 0;
    uint8_t acc_sum     = 0;

    for (uint16_t i = 0; i < len; i++) {
        acc_xor_add = (uint8_t)((acc_xor_add + data[i]) ^ data[i]);
        acc_sum     = (uint8_t)(acc_sum + data[i]);
    }

    uint16_t chk_word = (uint16_t)(((uint16_t)acc_sum << 8) | acc_xor_add);
    (void)FUN_08036ca0(target_flash_addr, chk_word);
}

/**
 * @brief Commits a downloaded SDB, OB1, or DB1 block to APM32E103 Flash.
 *        Invoked when Download Ended (0x1C / 0xFC) completes.
 */
static void s7_commit_downloaded_block_to_flash(uint8_t block_type)
{
    if (block_type == S7_DL_BLOCK_SDB) {
        /* Commit SDB (System Data Block) from staging RAM (0x2000108C) to 0x200058CC & Flash 0x0807F800 */
        uint8_t *sdb_ram = MEM_PTR(SRAM_SDB_MIRROR_ADDR);
        const uint8_t *staged = CONST_MEM_PTR(SRAM_V_BASE_ADDR);
        uint16_t sdb_len = g_sdb_block_len;
        if (sdb_len > 0x0180U) {
            sdb_len = 0x0180U;
        }
        memcpy(sdb_ram, staged, sdb_len);

        FUN_08036bc8(FLASH_SDB_BASE_ADDR);
        uint32_t flash_addr = FLASH_SDB_BASE_ADDR;
        for (uint16_t i = 0; i < sdb_len; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)sdb_ram[i + 1U] << 8) | sdb_ram[i]);
            if (FUN_08036ca0(flash_addr, hw) != 4) {
                break;
            }
            flash_addr += 2U;
        }
        s7_flash_write_fletcher_checksum(CONST_MEM_PTR(FLASH_SDB_BASE_ADDR), sdb_len, flash_addr);

        /* Restore DB1 V-memory from Flash 0x08064800 after using 0x2000108C as staging buffer */
        memcpy(MEM_PTR(SRAM_DB1_HEADER_ADDR), CONST_MEM_PTR(FLASH_DB1_BASE_ADDR), g_db1_block_len);

        /* Apply updated station addresses and baudrates from SDB */
        FUN_0802bd58();
        FUN_0802bd88();
    } else if (block_type == S7_DL_BLOCK_OB1) {
        /* Finalize raw OB1 in Flash at 0x08075800 and de-obfuscate into 0x0806D800 */
        uint16_t ob1_raw_len = g_ob1_block_len;
        s7_flash_write_fletcher_checksum(
            CONST_MEM_PTR(FLASH_OB1_RAW_BASE_ADDR),
            ob1_raw_len,
            FLASH_OB1_RAW_BASE_ADDR + ob1_raw_len
        );

        /* Erase compiled OB1 region (0x0806D800..0x08075000, sixteen 2KB pages = 32KB) */
        for (uint32_t page = FLASH_OB1_COMPILED_BASE_ADDR; page <= 0x08075000UL; page += 0x0800UL) {
            FUN_08036bc8(page);
        }

        /* De-obfuscate raw OB1 (0x08075800) into compiled OB1 Flash (0x0806D800) */
        uint32_t write_addr = FLASH_OB1_COMPILED_BASE_ADDR;
        const uint8_t *raw_ob1 = CONST_MEM_PTR(FLASH_OB1_RAW_BASE_ADDR);

        /* Copy 38-byte block header directly, then de-obfuscate encrypted body */
        for (uint16_t i = 0; i < 38U && i < ob1_raw_len; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)raw_ob1[i + 1U] << 8) | raw_ob1[i]);
            (void)FUN_08036ca0(write_addr, hw);
            write_addr += 2U;
        }
        if (ob1_raw_len > 38U) {
            s7_block_deobfuscate_ob1(raw_ob1 + 38U, (uint16_t)(ob1_raw_len - 38U), &write_addr);
        }

        uint16_t compiled_len = read_be16(CONST_MEM_PTR(FLASH_OB1_COMPILED_BASE_ADDR + 10U));
        if (compiled_len == 0U || compiled_len > 0x7FF0U) {
            compiled_len = ob1_raw_len;
        }
        s7_flash_write_weighted_checksum(
            CONST_MEM_PTR(FLASH_OB1_COMPILED_BASE_ADDR),
            compiled_len,
            FLASH_OB1_COMPILED_BASE_ADDR + compiled_len
        );

        /* Validate and compile OB1 ladder/STL instructions */
        FUN_080219c8();
    } else if (block_type == S7_DL_BLOCK_DB1) {
        /* Commit DB1 from RAM (0x20001068) to Flash (0x08064800..0x08068000) */
        for (uint32_t page = FLASH_DB1_BASE_ADDR; page <= 0x08068000UL; page += 0x0800UL) {
            FUN_08036bc8(page);
        }
        const uint8_t *db1_ram = CONST_MEM_PTR(SRAM_DB1_HEADER_ADDR);
        uint16_t db1_len = g_db1_block_len;
        if (db1_len > 0x3D24U) {
            db1_len = 0x3D24U;
        }
        uint32_t flash_addr = FLASH_DB1_BASE_ADDR;
        for (uint16_t i = 0; i < db1_len; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)db1_ram[i + 1U] << 8) | db1_ram[i]);
            if (FUN_08036ca0(flash_addr, hw) != 4) {
                break;
            }
            flash_addr += 2U;
        }
        s7_flash_write_fletcher_checksum(
            CONST_MEM_PTR(FLASH_DB1_BASE_ADDR),
            db1_len,
            FLASH_DB1_CHECKSUM_ADDR
        );
    }
}

/* ============================================================================
 * S7-200 Memory Area Read & Write Engine (with Force Table Protection)
 * ============================================================================ */

/**
 * @brief Reads a single variable item specified by an S7-Any 12-byte descriptor
 *        and appends the S7comm Data Item ([ret_code, transport_size, len_hi, len_lo, payload...])
 *        to dst_data.
 *
 * @return Number of bytes written to dst_data (including 4-byte item header + even-byte padding).
 */
static uint16_t s7_handle_read_var_item(
    const s7_any_item_t *item,
    uint8_t             *dst_data,
    bool                 is_last_item
)
{
    uint8_t  transport_type = item->transport_type;
    uint16_t count          = read_be16(&item->count_hi);
    uint16_t db_num         = read_be16(&item->db_num_hi);
    uint8_t  area           = item->area;
    uint32_t bit_addr       = ((uint32_t)item->bit_addr_hi << 16) |
                              ((uint32_t)item->bit_addr_mid << 8) |
                              (uint32_t)item->bit_addr_lo;
    uint16_t byte_offset    = (uint16_t)(bit_addr >> 3);
    uint8_t  bit_index      = (uint8_t)(bit_addr & 0x07U);

    uint16_t byte_len = FUN_0800422c(transport_type, count);
    uint8_t *payload  = dst_data + 4U;

    /* Default Data Item Header: Return Code = 0xFF (OK) */
    dst_data[0] = S7_RET_OK;

    if (transport_type == S7_PARAM_TYPE_BIT) {
        dst_data[1] = S7_TS_BIT;
        write_be16(&dst_data[2], 1U);
        byte_len = 1U;
    } else if (transport_type == S7_PARAM_TYPE_C ||
               transport_type == S7_PARAM_TYPE_T ||
               transport_type == S7_PARAM_TYPE_HC) {
        dst_data[1] = S7_TS_OCTET_STRING;
        write_be16(&dst_data[2], byte_len);
    } else {
        dst_data[1] = S7_TS_BYTE_WORD_DWORD;
        write_be16(&dst_data[2], (uint16_t)(byte_len << 3)); /* Length in bits */
    }

    const uint8_t *src_base = NULL;
    uint16_t max_offset = 0;

    switch (area) {
        case S7_AREA_I:
            src_base   = CONST_MEM_PTR(SRAM_I_BASE_ADDR);
            max_offset = 32U;
            break;
        case S7_AREA_Q:
            src_base   = CONST_MEM_PTR(SRAM_Q_BASE_ADDR);
            max_offset = 32U;
            break;
        case S7_AREA_M:
            src_base   = CONST_MEM_PTR(SRAM_M_BASE_ADDR);
            max_offset = 32U;
            break;
        case S7_AREA_S:
            src_base   = CONST_MEM_PTR(SRAM_S_BASE_ADDR);
            max_offset = 32U;
            break;
        case S7_AREA_SM:
            src_base   = CONST_MEM_PTR(SRAM_SM_BASE_ADDR);
            max_offset = 550U;
            break;
        case S7_AREA_AI:
            src_base   = CONST_MEM_PTR(SRAM_AI_BASE_ADDR);
            max_offset = 128U;
            break;
        case S7_AREA_AQ:
            src_base   = CONST_MEM_PTR(SRAM_AQ_BASE_ADDR);
            max_offset = 128U;
            break;
        case S7_AREA_V_DB1:
            if (db_num == 1U) {
                src_base   = CONST_MEM_PTR(SRAM_V_BASE_ADDR);
                max_offset = 0x297CU;
            } else {
                src_base   = CONST_MEM_PTR(SRAM_DB1_HEADER_ADDR);
                max_offset = 0x29A0U;
            }
            break;
        case S7_AREA_C: {
            /* Counter area (0x1E): index in byte_offset or bit_addr */
            uint16_t c_idx = (transport_type == S7_PARAM_TYPE_C) ? (uint16_t)bit_addr : (byte_offset >> 1);
            const uint8_t  *c_stat = CONST_MEM_PTR(SRAM_C_STATUS_BITS_ADDR);
            const uint16_t *c_val  = (const uint16_t *)(uintptr_t)SRAM_C_CURRENT_VAL_ADDR;
            if (transport_type == S7_PARAM_TYPE_C) {
                for (uint16_t i = 0; i < count; i++) {
                    uint16_t idx = (uint16_t)((c_idx + i) & 0xFFU);
                    payload[i * 3U + 0U] = (c_stat[idx] & 0x10U) ? 0x01U : 0x00U;
                    write_be16(&payload[i * 3U + 1U], c_val[idx]);
                }
            } else if (transport_type == S7_PARAM_TYPE_BIT) {
                payload[0] = (c_stat[c_idx & 0xFFU] & 0x10U) ? 0x01U : 0x00U;
            } else {
                for (uint16_t i = 0; i < count; i++) {
                    write_be16(&payload[i * 2U], c_val[(c_idx + i) & 0xFFU]);
                }
            }
            goto done_item;
        }
        case S7_AREA_T: {
            /* Timer area (0x1F): 5-byte descriptor or 2-byte current value */
            uint16_t t_idx = (transport_type == S7_PARAM_TYPE_T) ? (uint16_t)bit_addr : (byte_offset >> 1);
            const uint8_t  *t_stat = CONST_MEM_PTR(SRAM_T_STATUS_BITS_ADDR);
            const uint16_t *t_val  = (const uint16_t *)(uintptr_t)SRAM_T_CURRENT_VAL_ADDR;
            if (transport_type == S7_PARAM_TYPE_T) {
                for (uint16_t i = 0; i < count; i++) {
                    uint16_t idx = (uint16_t)((t_idx + i) & 0xFFU);
                    payload[i * 5U + 0U] = (t_stat[idx] & 0x02U) ? 0x01U : 0x00U;
                    payload[i * 5U + 1U] = 0x00U;
                    payload[i * 5U + 2U] = 0x00U;
                    write_be16(&payload[i * 5U + 3U], t_val[idx]);
                }
            } else if (transport_type == S7_PARAM_TYPE_BIT) {
                payload[0] = (t_stat[t_idx & 0xFFU] & 0x02U) ? 0x01U : 0x00U;
            } else {
                for (uint16_t i = 0; i < count; i++) {
                    write_be16(&payload[i * 2U], t_val[(t_idx + i) & 0xFFU]);
                }
            }
            goto done_item;
        }
        case S7_AREA_HC: {
            /* High-Speed Counter area (0x20): 32-bit values at 0x20006150 */
            uint16_t hc_idx = (transport_type == S7_PARAM_TYPE_HC) ? (uint16_t)bit_addr : (byte_offset >> 2);
            const uint32_t *hc_val = (const uint32_t *)(uintptr_t)SRAM_HC_CURRENT_VAL_ADDR;
            for (uint16_t i = 0; i < count; i++) {
                uint16_t idx = (uint16_t)((hc_idx + i) % 20U);
                write_be32(&payload[i * 4U], hc_val[idx]);
            }
            goto done_item;
        }
        case S7_AREA_SYS_INFO: {
            /* Area 0x03: System status / PLC run state / Force status */
            memset(payload, 0, byte_len);
            if (byte_len >= 1U) {
                payload[0] = g_plc_run_mode;
            }
            if (byte_len >= 2U) {
                payload[1] = (g_force_table_count > 0U || g_force_io_bit_active != 0U) ? 0x80U : 0x00U;
            }
            goto done_item;
        }
        default:
            dst_data[0] = S7_RET_INVALID_ADDR;
            dst_data[1] = S7_TS_NULL;
            write_be16(&dst_data[2], 0U);
            return 4U;
    }

    if (byte_offset >= max_offset) {
        dst_data[0] = S7_RET_INVALID_ADDR;
        dst_data[1] = S7_TS_NULL;
        write_be16(&dst_data[2], 0U);
        return 4U;
    }

    if (transport_type == S7_PARAM_TYPE_BIT) {
        payload[0] = (src_base[byte_offset] & (1U << bit_index)) ? 0x01U : 0x00U;
    } else {
        for (uint16_t i = 0; i < byte_len; i++) {
            if ((uint16_t)(byte_offset + i) < max_offset) {
                payload[i] = src_base[byte_offset + i];
            } else {
                payload[i] = 0x00U;
            }
        }
    }

done_item:;
    uint16_t total_item_bytes = (uint16_t)(4U + byte_len);
    if (!is_last_item && (byte_len & 1U) != 0U) {
        dst_data[total_item_bytes] = 0x00U; /* S7comm even-byte alignment pad */
        total_item_bytes++;
    }
    return total_item_bytes;
}

/**
 * @brief Checks if a non-bit variable in the 16-entry Force Table (0x20005AF0) is
 *        currently forced, preventing normal Write Var requests from overwriting it.
 */
static bool s7_is_address_forced(uint8_t s7200_area, uint8_t sub_area, uint16_t byte_offset)
{
    const s7200_force_entry_t *table = (const s7200_force_entry_t *)(uintptr_t)SRAM_FORCE_TABLE_ADDR;
    for (uint8_t i = 0; i < 16U; i++) {
        if (table[i].area == s7200_area &&
            (sub_area == 0U || table[i].sub_area == sub_area) &&
            table[i].byte_offset == byte_offset) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Writes a single variable item specified by an S7-Any 12-byte descriptor
 *        while respecting active Force masks on I, Q, V, M, S, AI, AQ, T, C, HC.
 *
 * @return S7comm return code (0xFF = S7_RET_OK, 0x05 = S7_RET_INVALID_ADDR).
 */
static uint8_t s7_handle_write_var_item(
    const s7_any_item_t *item,
    const uint8_t       *item_data,
    uint16_t            *consumed_data_bytes
)
{
    uint8_t  transport_type = item->transport_type;
    uint16_t count          = read_be16(&item->count_hi);
    uint16_t db_num         = read_be16(&item->db_num_hi);
    uint8_t  area           = item->area;
    uint32_t bit_addr       = ((uint32_t)item->bit_addr_hi << 16) |
                              ((uint32_t)item->bit_addr_mid << 8) |
                              (uint32_t)item->bit_addr_lo;
    uint16_t byte_offset    = (uint16_t)(bit_addr >> 3);
    uint8_t  bit_index      = (uint8_t)(bit_addr & 0x07U);
    uint8_t  bit_mask       = (uint8_t)(1U << bit_index);

    uint8_t  ts_type = item_data[1];
    uint16_t raw_len = read_be16(&item_data[2]);
    uint16_t byte_len = (ts_type == S7_TS_BYTE_WORD_DWORD) ? (uint16_t)(raw_len >> 3) : raw_len;
    if (transport_type == S7_PARAM_TYPE_BIT) {
        byte_len = 1U;
    }
    const uint8_t *src_val = item_data + 4U;
    *consumed_data_bytes = (uint16_t)(4U + byte_len + (byte_len & 1U));

    switch (area) {
        case S7_AREA_I: {
            uint8_t *i_ram   = MEM_PTR(SRAM_I_BASE_ADDR);
            const uint8_t *f_mask = CONST_MEM_PTR(SRAM_FORCE_I_MASK_ADDR);
            if (byte_offset >= 32U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (byte_offset < 16U && (f_mask[byte_offset] & bit_mask) != 0U) {
                    return S7_RET_OK; /* Forced bit protected */
                }
                if (src_val[0] != 0U) i_ram[byte_offset] |= bit_mask;
                else                  i_ram[byte_offset] &= (uint8_t)~bit_mask;
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 32U; i++) {
                    uint16_t off = (uint16_t)(byte_offset + i);
                    uint8_t mask = (off < 16U) ? f_mask[off] : 0U;
                    i_ram[off] = (uint8_t)((i_ram[off] & mask) | (src_val[i] & (uint8_t)~mask));
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_Q: {
            uint8_t *q_ram   = MEM_PTR(SRAM_Q_BASE_ADDR);
            const uint8_t *f_mask = CONST_MEM_PTR(SRAM_FORCE_Q_MASK_ADDR);
            if (byte_offset >= 32U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (byte_offset < 16U && (f_mask[byte_offset] & bit_mask) != 0U) {
                    return S7_RET_OK;
                }
                if (src_val[0] != 0U) q_ram[byte_offset] |= bit_mask;
                else                  q_ram[byte_offset] &= (uint8_t)~bit_mask;
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 32U; i++) {
                    uint16_t off = (uint16_t)(byte_offset + i);
                    uint8_t mask = (off < 16U) ? f_mask[off] : 0U;
                    q_ram[off] = (uint8_t)((q_ram[off] & mask) | (src_val[i] & (uint8_t)~mask));
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_M: {
            uint8_t *m_ram = MEM_PTR(SRAM_M_BASE_ADDR);
            if (byte_offset >= 32U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (!s7_is_address_forced(S7200_AREA_M, 0U, byte_offset)) {
                    if (src_val[0] != 0U) m_ram[byte_offset] |= bit_mask;
                    else                  m_ram[byte_offset] &= (uint8_t)~bit_mask;
                }
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 32U; i++) {
                    if (!s7_is_address_forced(S7200_AREA_M, 0U, (uint16_t)(byte_offset + i))) {
                        m_ram[byte_offset + i] = src_val[i];
                    }
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_S: {
            uint8_t *s_ram = MEM_PTR(SRAM_S_BASE_ADDR);
            if (byte_offset >= 32U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (src_val[0] != 0U) s_ram[byte_offset] |= bit_mask;
                else                  s_ram[byte_offset] &= (uint8_t)~bit_mask;
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 32U; i++) {
                    s_ram[byte_offset + i] = src_val[i];
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_SM: {
            uint8_t *sm_ram = MEM_PTR(SRAM_SM_BASE_ADDR);
            if (byte_offset >= 550U) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (src_val[0] != 0U) sm_ram[byte_offset] |= bit_mask;
                else                  sm_ram[byte_offset] &= (uint8_t)~bit_mask;
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 550U; i++) {
                    sm_ram[byte_offset + i] = src_val[i];
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_AI: {
            uint8_t *ai_ram = MEM_PTR(SRAM_AI_BASE_ADDR);
            if (byte_offset >= 128U) return S7_RET_INVALID_ADDR;
            if (!s7_is_address_forced(S7200_AREA_IO_PORT, S7_AREA_AI, byte_offset)) {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 128U; i++) {
                    ai_ram[byte_offset + i] = src_val[i];
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_AQ: {
            uint8_t *aq_ram = MEM_PTR(SRAM_AQ_BASE_ADDR);
            if (byte_offset >= 128U) return S7_RET_INVALID_ADDR;
            if (!s7_is_address_forced(S7200_AREA_IO_PORT, S7_AREA_AQ, byte_offset)) {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < 128U; i++) {
                    aq_ram[byte_offset + i] = src_val[i];
                }
            }
            return S7_RET_OK;
        }
        case S7_AREA_V_DB1: {
            uint8_t *v_ram = (db_num == 1U) ? MEM_PTR(SRAM_V_BASE_ADDR) : MEM_PTR(SRAM_DB1_HEADER_ADDR);
            uint16_t limit = (db_num == 1U) ? 0x297CU : 0x29A0U;
            if (byte_offset >= limit) return S7_RET_INVALID_ADDR;
            if (transport_type == S7_PARAM_TYPE_BIT) {
                if (!s7_is_address_forced(S7200_AREA_V_DB1, 0U, byte_offset)) {
                    if (src_val[0] != 0U) v_ram[byte_offset] |= bit_mask;
                    else                  v_ram[byte_offset] &= (uint8_t)~bit_mask;
                }
            } else {
                for (uint16_t i = 0; i < byte_len && (byte_offset + i) < limit; i++) {
                    if (!s7_is_address_forced(S7200_AREA_V_DB1, 0U, (uint16_t)(byte_offset + i))) {
                        v_ram[byte_offset + i] = src_val[i];
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
                    if (src_val[i * 3U] != 0U) c_stat[idx] |= 0x10U;
                    else                       c_stat[idx] &= (uint8_t)~0x10U;
                    c_val[idx] = read_be16(&src_val[i * 3U + 1U]);
                }
            } else if (byte_len >= 2U) {
                c_val[c_idx & 0xFFU] = read_be16(src_val);
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
                    if (src_val[i * 5U] != 0U) t_stat[idx] |= 0x02U;
                    else                       t_stat[idx] &= (uint8_t)~0x02U;
                    t_val[idx] = read_be16(&src_val[i * 5U + 3U]);
                }
            } else if (byte_len >= 2U) {
                t_val[t_idx & 0xFFU] = read_be16(src_val);
            }
            return S7_RET_OK;
        }
        case S7_AREA_HC: {
            uint16_t hc_idx = (transport_type == S7_PARAM_TYPE_HC) ? (uint16_t)bit_addr : (byte_offset >> 2);
            uint32_t *hc_val = (uint32_t *)(uintptr_t)SRAM_HC_CURRENT_VAL_ADDR;
            if (byte_len >= 4U) {
                hc_val[hc_idx % 20U] = FUN_08003ff0(src_val);
            }
            return S7_RET_OK;
        }
        default:
            return S7_RET_INVALID_ADDR;
    }
}

/* ============================================================================
 * Force Variables Engine (ROSCTR = 0x07 Userdata Force / Unforce)
 * ============================================================================ */

static void s7_update_force_status_sm_bit(void)
{
    uint8_t any_io_forced = 0U;
    const uint8_t *i_mask = CONST_MEM_PTR(SRAM_FORCE_I_MASK_ADDR);
    const uint8_t *q_mask = CONST_MEM_PTR(SRAM_FORCE_Q_MASK_ADDR);

    for (uint8_t i = 0; i < 16U; i++) {
        if (i_mask[i] != 0U || q_mask[i] != 0U) {
            any_io_forced = 1U;
            break;
        }
    }
    g_force_io_bit_active = any_io_forced;

    /* SM4.7 (0x20005F2C bit 7) indicates Force Active in S7-200 */
    uint8_t *sm4 = MEM_PTR(SRAM_SM_BASE_ADDR + 4U);
    if (any_io_forced != 0U || g_force_table_count != 0U) {
        *sm4 |= 0x80U;
    } else {
        *sm4 &= (uint8_t)~0x80U;
    }
}

static void s7_unforce_all_variables(void)
{
    memset(MEM_PTR(SRAM_FORCE_I_MASK_ADDR), 0, 16U);
    memset(MEM_PTR(SRAM_FORCE_I_VAL_ADDR),  0, 16U);
    memset(MEM_PTR(SRAM_FORCE_Q_MASK_ADDR), 0, 16U);
    memset(MEM_PTR(SRAM_FORCE_Q_VAL_ADDR),  0, 16U);
    memset(MEM_PTR(SRAM_FORCE_TABLE_ADDR),  0, 16U * sizeof(s7200_force_entry_t));
    g_force_table_count   = 0U;
    g_force_io_bit_active = 0U;
    MEM_U8(SRAM_SM_BASE_ADDR + 4U) &= (uint8_t)~0x80U;
}

static void s7_apply_force_item(
    uint8_t        s7200_area,
    uint8_t        sub_area,
    uint16_t       byte_offset,
    uint8_t        bit_or_len,
    const uint8_t *val_bytes
)
{
    if (s7200_area == S7200_AREA_IO_PORT && (sub_area == S7_AREA_I || sub_area == S7_AREA_Q)) {
        uint8_t *f_mask = (sub_area == S7_AREA_I) ? MEM_PTR(SRAM_FORCE_I_MASK_ADDR)
                                                  : MEM_PTR(SRAM_FORCE_Q_MASK_ADDR);
        uint8_t *f_val  = (sub_area == S7_AREA_I) ? MEM_PTR(SRAM_FORCE_I_VAL_ADDR)
                                                  : MEM_PTR(SRAM_FORCE_Q_VAL_ADDR);
        uint8_t *io_ram = (sub_area == S7_AREA_I) ? MEM_PTR(SRAM_I_BASE_ADDR)
                                                  : MEM_PTR(SRAM_Q_BASE_ADDR);
        if (byte_offset < 16U) {
            uint8_t mask = (uint8_t)(1U << (bit_or_len & 0x07U));
            f_mask[byte_offset] |= mask;
            if (val_bytes[0] != 0U) {
                f_val[byte_offset]  |= mask;
                io_ram[byte_offset] |= mask;
            } else {
                f_val[byte_offset]  &= (uint8_t)~mask;
                io_ram[byte_offset] &= (uint8_t)~mask;
            }
        }
        s7_update_force_status_sm_bit();
        return;
    }

    /* Non-I/Q bit or Word/DWord entry in the 16-slot Force Table at 0x20005AF0 */
    s7200_force_entry_t *table = (s7200_force_entry_t *)(uintptr_t)SRAM_FORCE_TABLE_ADDR;
    int target_slot = -1;
    for (int i = 0; i < 16; i++) {
        if (table[i].area == s7200_area &&
            table[i].sub_area == sub_area &&
            table[i].byte_offset == byte_offset) {
            target_slot = i;
            break;
        }
        if (target_slot < 0 && table[i].area == 0U) {
            target_slot = i;
        }
    }

    if (target_slot >= 0) {
        if (table[target_slot].area == 0U && g_force_table_count < 16U) {
            g_force_table_count++;
        }
        table[target_slot].area        = s7200_area;
        table[target_slot].sub_area    = sub_area;
        table[target_slot].bit_mask    = bit_or_len;
        table[target_slot].byte_offset = byte_offset;
        memcpy(table[target_slot].value, val_bytes, 4U);
    }
    s7_update_force_status_sm_bit();
}

/* ============================================================================
 * Password Verification & Clear PLC Memory Helpers
 * ============================================================================ */

/**
 * @brief Verifies an 8-byte password supplied in a Userdata PDU against the
 *        SDB password stored at 0x2000596A..0x20005971.
 *
 * Encoding rule in S7-226 firmware:
 *   - Bytes 0, 1, 4, 5 are stored inverted: sdb_pw[i] == (0xFF - input[i])
 *   - Bytes 2, 3, 6, 7 are stored directly: sdb_pw[i] == input[i]
 */
static bool s7_verify_password(const uint8_t *pw_in)
{
    const uint8_t *sdb_pw = CONST_MEM_PTR(SRAM_SDB_PASSWORD_ADDR);
    if ((uint8_t)(0xFFU - pw_in[0]) != sdb_pw[0]) return false;
    if ((uint8_t)(0xFFU - pw_in[1]) != sdb_pw[1]) return false;
    if (pw_in[2]                    != sdb_pw[2]) return false;
    if (pw_in[3]                    != sdb_pw[3]) return false;
    if ((uint8_t)(0xFFU - pw_in[4]) != sdb_pw[4]) return false;
    if ((uint8_t)(0xFFU - pw_in[5]) != sdb_pw[5]) return false;
    if (pw_in[6]                    != sdb_pw[6]) return false;
    if (pw_in[7]                    != sdb_pw[7]) return false;
    return true;
}

/**
 * @brief Clears PLC memory (OB1, DB1, SDB, V/M/S/T/C RAM) and restores factory
 *        default templates from ROM (0x0804A1F4 & 0x0804A264).
 */
static void s7_clear_plc_memory(uint8_t clear_mask)
{
    g_plc_run_mode          = 0U;
    g_plc_prev_run_mode     = 0U;
    g_status_monitor_trigger = 0U;
    s7_unforce_all_variables();

    /* Clear M, S, Q, V, T, C in SRAM */
    memset(MEM_PTR(SRAM_M_BASE_ADDR), 0, 32U);
    memset(MEM_PTR(SRAM_S_BASE_ADDR), 0, 32U);
    memset(MEM_PTR(SRAM_Q_BASE_ADDR), 0, 32U);
    memset(MEM_PTR(SRAM_V_BASE_ADDR), 0, 0x2800U);
    memset(MEM_PTR(SRAM_T_CURRENT_VAL_ADDR), 0, 512U);
    memset(MEM_PTR(SRAM_T_STATUS_BITS_ADDR), 0, 256U);
    memset(MEM_PTR(SRAM_C_CURRENT_VAL_ADDR), 0, 512U);
    memset(MEM_PTR(SRAM_C_STATUS_BITS_ADDR), 0, 256U);

    if ((clear_mask & 0x01U) != 0U) {
        /* Reset OB1 to default ROM template (0x0804A1F4, 110 bytes) */
        g_ob1_block_len = 0x006EU;
        FUN_08036bc8(FLASH_OB1_RAW_BASE_ADDR);
        FUN_08036bc8(FLASH_OB1_COMPILED_BASE_ADDR);
        const uint8_t *def_ob1 = CONST_MEM_PTR(FLASH_DEFAULT_OB1_TEMPLATE_ADDR);
        uint32_t raw_addr = FLASH_OB1_RAW_BASE_ADDR;
        for (uint16_t i = 0; i < 0x006EU; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)def_ob1[i + 1U] << 8) | def_ob1[i]);
            (void)FUN_08036ca0(raw_addr, hw);
            raw_addr += 2U;
        }
        s7_flash_write_fletcher_checksum(
            CONST_MEM_PTR(FLASH_OB1_RAW_BASE_ADDR),
            0x006EU,
            FLASH_OB1_RAW_BASE_ADDR + 0x006EU
        );
        uint32_t comp_addr = FLASH_OB1_COMPILED_BASE_ADDR;
        for (uint16_t i = 0; i < 38U; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)def_ob1[i + 1U] << 8) | def_ob1[i]);
            (void)FUN_08036ca0(comp_addr, hw);
            comp_addr += 2U;
        }
        s7_block_deobfuscate_ob1(def_ob1 + 38U, 0x006EU - 38U, &comp_addr);
        FUN_080219c8();
    }

    if ((clear_mask & 0x02U) != 0U) {
        /* Reset DB1 to default ROM template (0x0804A264, 36 bytes) */
        g_db1_block_len = 0x0024U;
        memcpy(MEM_PTR(SRAM_DB1_HEADER_ADDR), CONST_MEM_PTR(FLASH_DEFAULT_DB1_TEMPLATE_ADDR), 0x0024U);
        FUN_08036bc8(FLASH_DB1_BASE_ADDR);
        const uint8_t *def_db1 = CONST_MEM_PTR(FLASH_DEFAULT_DB1_TEMPLATE_ADDR);
        uint32_t db1_addr = FLASH_DB1_BASE_ADDR;
        for (uint16_t i = 0; i < 0x0024U; i += 2U) {
            uint16_t hw = (uint16_t)(((uint16_t)def_db1[i + 1U] << 8) | def_db1[i]);
            (void)FUN_08036ca0(db1_addr, hw);
            db1_addr += 2U;
        }
    }

    if ((clear_mask & 0x04U) != 0U) {
        /* Reset SDB in Flash (0x0807F800) and restore default station address 2 / 9.6 kbps */
        g_sdb_block_len = 0U;
        memset(MEM_PTR(SRAM_SDB_MIRROR_ADDR), 0, 0x0180U);
        FUN_08036bc8(FLASH_SDB_BASE_ADDR);
        g_ppi_port0_station_addr = 2U;
        g_ppi_port1_station_addr = 2U;
        FUN_0802c010(2U);
        FUN_0802c060(2U);
    }

    FUN_0802b8e8();
}

/* ============================================================================
 * S7comm PDU Core Dispatcher (Shared by ISO-on-TCP COTP DT & Bridged PPI SD2)
 * ============================================================================ */

/**
 * @brief Processes an S7comm PDU starting at `s7_rx` (where `s7_rx[0] == 0x32`)
 *        and writes the response S7comm PDU into `s7_tx`.
 *
 * @param s7_rx         Pointer to incoming S7comm header (0x32 ...)
 * @param s7_rx_len     Length of incoming S7comm PDU
 * @param s7_tx         Pointer to output S7comm response buffer
 * @return Length of response S7comm PDU written to `s7_tx`
 */
static uint16_t s7_process_pdu_core(const uint8_t *s7_rx, uint16_t s7_rx_len, uint8_t *s7_tx)
{
    if (s7_rx_len < sizeof(s7comm_hdr_t) || s7_rx[0] != S7COMM_PROTO_ID) {
        return 0U;
    }

    uint8_t  rosctr    = s7_rx[1];
    uint16_t pdu_ref   = read_be16(&s7_rx[4]);
    uint16_t param_len = read_be16(&s7_rx[6]);
    uint16_t data_len  = read_be16(&s7_rx[8]);

    const uint8_t *rx_param = s7_rx + 10U;
    const uint8_t *rx_data  = rx_param + param_len;

    /* Prepare common Ack_Data (ROSCTR = 0x03) response header */
    s7_tx[0] = S7COMM_PROTO_ID;
    s7_tx[1] = S7_ROSCTR_ACK_DATA;
    s7_tx[2] = 0x00U;
    s7_tx[3] = 0x00U;
    write_be16(&s7_tx[4], pdu_ref);
    s7_tx[10] = 0x00U; /* err_class = 0x00 */
    s7_tx[11] = 0x00U; /* err_code  = 0x00 */

    uint8_t *tx_param = s7_tx + 12U;

    /* ------------------------------------------------------------------------
     * Branch A: ROSCTR == 0x01 (S7comm Job Request)
     * ------------------------------------------------------------------------ */
    if (rosctr == S7_ROSCTR_JOB && param_len >= 1U) {
        uint8_t func_code = rx_param[0];

        switch (func_code) {
            /* ----------------------------------------------------------------
             * 0xF0: Setup Communication (Negotiate PDU length)
             * ---------------------------------------------------------------- */
            case S7_FUNC_SETUP_COMM: {
                write_be16(&s7_tx[6], 8U); /* param_len = 8 */
                write_be16(&s7_tx[8], 0U); /* data_len  = 0 */
                tx_param[0] = S7_FUNC_SETUP_COMM;
                tx_param[1] = 0x00U;
                tx_param[2] = 0x00U;
                tx_param[3] = 0x01U; /* Max AmQ calling = 1 */
                tx_param[4] = 0x00U;
                tx_param[5] = 0x01U; /* Max AmQ called  = 1 */
                /* Echo or cap negotiated PDU size (default 0x00F0 = 240 bytes) */
                uint16_t req_pdu = (param_len >= 8U) ? read_be16(&rx_param[6]) : 0x00F0U;
                if (req_pdu > 0x03C0U) {
                    req_pdu = 0x03C0U;
                }
                write_be16(&tx_param[6], req_pdu);
                return 20U;
            }

            /* ----------------------------------------------------------------
             * 0x29: PLC Stop ("P_STOP")
             * ---------------------------------------------------------------- */
            case S7_FUNC_PLC_STOP: {
                g_plc_run_mode           = 0U;
                g_plc_prev_run_mode      = 0U;
                g_status_monitor_trigger = 0U;
                FUN_0802b8e8();

                write_be16(&s7_tx[6], 1U); /* param_len = 1 */
                write_be16(&s7_tx[8], 0U); /* data_len  = 0 */
                tx_param[0] = S7_FUNC_PLC_STOP;
                return 13U;
            }

            /* ----------------------------------------------------------------
             * 0x28: PI Service ("P_PROGRAM" RUN, Block Link 'I'/'D', "_GARB")
             * ---------------------------------------------------------------- */
            case S7_FUNC_PI_SERVICE: {
                /* Check for "P_PROGRAM" transition to RUN mode */
                bool is_p_program = false;
                for (uint16_t i = 0; i + 8U < param_len; i++) {
                    if (memcmp(&rx_param[i], "P_PROGRAM", 9U) == 0) {
                        is_p_program = true;
                        break;
                    }
                }

                if (is_p_program) {
                    FUN_080219c8();
                    if (g_ob1_compile_err_code == 0U && g_plc_hw_fault_flag == 0U) {
                        g_plc_run_init_flag = 1U;
                        g_plc_run_mode      = 1U;
                    } else {
                        s7_tx[10] = 0x82U;
                        s7_tx[11] = 0x04U;
                    }
                } else if (param_len >= 16U) {
                    /* Block Link / Insert ('I'=0x49) or Delete ('D'=0x44) for _0800001P / _0A00001P / _0B00000P */
                    uint8_t sub_op = rx_param[9];
                    if (sub_op == 'D') {
                        if (memcmp(&rx_param[12], "0800001P", 8U) == 0) {
                            s7_clear_plc_memory(0x01U);
                        } else if (memcmp(&rx_param[12], "0A00001P", 8U) == 0) {
                            s7_clear_plc_memory(0x02U);
                        } else if (memcmp(&rx_param[12], "0B00000P", 8U) == 0) {
                            s7_clear_plc_memory(0x04U);
                        }
                    }
                }

                write_be16(&s7_tx[6], 1U);
                write_be16(&s7_tx[8], 0U);
                tx_param[0] = S7_FUNC_PI_SERVICE;
                return 13U;
            }

            /* ----------------------------------------------------------------
             * 0x04: Read Variable (Multi-item S7-Any Read)
             * ---------------------------------------------------------------- */
            case S7_FUNC_READ_VAR: {
                uint8_t item_count = (param_len >= 2U) ? rx_param[1] : 0U;
                tx_param[0] = S7_FUNC_READ_VAR;
                tx_param[1] = item_count;

                uint8_t *tx_data = tx_param + 2U;
                uint16_t total_data_len = 0U;
                const uint8_t *item_ptr = rx_param + 2U;

                for (uint8_t i = 0; i < item_count; i++) {
                    bool is_last = (i == (uint8_t)(item_count - 1U));
                    if (item_ptr[0] == 0x12U && item_ptr[1] == 0x0AU && item_ptr[2] == 0x10U) {
                        uint16_t written = s7_handle_read_var_item(
                            (const s7_any_item_t *)item_ptr,
                            tx_data + total_data_len,
                            is_last
                        );
                        total_data_len = (uint16_t)(total_data_len + written);
                        item_ptr += 12U;
                    } else {
                        /* Fallback for compact S7-200 status/area descriptors */
                        uint8_t hdr_len = (uint8_t)(item_ptr[1] + 2U);
                        tx_data[total_data_len + 0U] = S7_RET_OK;
                        tx_data[total_data_len + 1U] = S7_TS_OCTET_STRING;
                        write_be16(&tx_data[total_data_len + 2U], 1U);
                        tx_data[total_data_len + 4U] = g_plc_run_mode;
                        total_data_len = (uint16_t)(total_data_len + (is_last ? 5U : 6U));
                        item_ptr += hdr_len;
                    }
                }

                write_be16(&s7_tx[6], 2U);
                write_be16(&s7_tx[8], total_data_len);
                return (uint16_t)(14U + total_data_len);
            }

            /* ----------------------------------------------------------------
             * 0x05: Write Variable (Multi-item S7-Any Write with Force protection)
             * ---------------------------------------------------------------- */
            case S7_FUNC_WRITE_VAR: {
                /* Check password privilege if write-protected (level >= 3 without auth) */
                uint8_t priv_level = MEM_U8(SRAM_SDB_PRIV_LEVEL_ADDR);
                if (priv_level >= 3U && g_password_auth_ok_eth == 0U) {
                    s7_tx[10] = 0x85U;
                    s7_tx[11] = 0x00U;
                    write_be16(&s7_tx[6], 0U);
                    write_be16(&s7_tx[8], 0U);
                    return 12U;
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
                        tx_data[i] = s7_handle_write_var_item(
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

                write_be16(&s7_tx[6], 2U);
                write_be16(&s7_tx[8], item_count);
                return (uint16_t)(14U + item_count);
            }

            /* ----------------------------------------------------------------
             * 0x1D: Start Block Upload (_0800001P OB1, _0A00001P DB1, _0B00000P SDB)
             * ---------------------------------------------------------------- */
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

                write_be16(&s7_tx[6], 16U); /* param_len = 16 */
                write_be16(&s7_tx[8], 0U);  /* data_len  = 0  */
                tx_param[0] = S7_FUNC_START_UPLOAD;
                memset(&tx_param[1], 0, 7U);
                tx_param[7] = 0x07U; /* Upload string length = 7 */
                /* Format 5-digit decimal block length string + "P" */
                uint16_t rem = blk_len;
                for (int d = 5; d >= 1; d--) {
                    tx_param[8 + d] = (uint8_t)('0' + (rem % 10U));
                    rem /= 10U;
                }
                tx_param[8]  = '0';
                tx_param[14] = 'P';
                tx_param[15] = 0x00U;
                return 28U;
            }

            /* ----------------------------------------------------------------
             * 0x1E: Upload Block Chunk
             * ---------------------------------------------------------------- */
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
                uint16_t remaining = (offset < total_len) ? (uint16_t)(total_len - offset) : 0U;
                uint16_t chunk_len = (remaining > 222U) ? 222U : remaining;
                uint8_t  more_data = (remaining > chunk_len) ? 0x01U : 0x00U;

                tx_param[0] = S7_FUNC_UPLOAD;
                tx_param[1] = more_data;

                uint8_t *tx_data = tx_param + 2U;
                write_be16(&tx_data[0], chunk_len);
                tx_data[2] = 0x00U;
                tx_data[3] = 0xFBU;
                if (chunk_len > 0U) {
                    memcpy(&tx_data[4], blk_base + offset, chunk_len);
                }
                g_block_xfer_offset = (uint16_t)(offset + chunk_len);

                write_be16(&s7_tx[6], 2U);
                write_be16(&s7_tx[8], (uint16_t)(chunk_len + 4U));
                return (uint16_t)(18U + chunk_len);
            }

            /* ----------------------------------------------------------------
             * 0x1F: End Block Upload
             * ---------------------------------------------------------------- */
            case S7_FUNC_END_UPLOAD: {
                g_block_xfer_offset    = 0U;
                g_dl_active_block_type = S7_DL_BLOCK_NONE;
                write_be16(&s7_tx[6], 1U);
                write_be16(&s7_tx[8], 0U);
                tx_param[0] = S7_FUNC_END_UPLOAD;
                return 13U;
            }

            /* ----------------------------------------------------------------
             * 0x1A / 0xFA: Request Block Download (_0800001P, _0A00001P, _0B00000P)
             * ---------------------------------------------------------------- */
            case S7_FUNC_REQ_DOWNLOAD:
            case S7_FUNC_REQ_DOWNLOAD_EXT: {
                g_plc_prev_run_mode = g_plc_run_mode;
                g_plc_run_mode      = 0U;
                g_block_xfer_offset = 0U;
                g_dl_chunk_seq_num  = 1U;

                /* Parse block type and declared block size from parameter block */
                uint16_t decl_len = 0U;
                if (param_len >= 18U) {
                    for (uint8_t d = 0; d < 6U; d++) {
                        uint8_t ch = rx_param[param_len - 7U + d];
                        if (ch >= '0' && ch <= '9') {
                            decl_len = (uint16_t)(decl_len * 10U + (uint16_t)(ch - '0'));
                        }
                    }
                    if (memcmp(&rx_param[10], "0B00000P", 8U) == 0) {
                        g_dl_active_block_type = S7_DL_BLOCK_SDB;
                        g_sdb_block_len        = decl_len;
                    } else if (memcmp(&rx_param[10], "0A00001P", 8U) == 0) {
                        g_dl_active_block_type = S7_DL_BLOCK_DB1;
                        g_db1_block_len        = decl_len;
                    } else {
                        g_dl_active_block_type = S7_DL_BLOCK_OB1;
                        g_ob1_block_len        = decl_len;
                        /* Erase raw OB1 Flash staging pages (0x08075800..0x0807D000) */
                        for (uint32_t page = FLASH_OB1_RAW_BASE_ADDR; page <= 0x0807D000UL; page += 0x0800UL) {
                            FUN_08036bc8(page);
                        }
                    }
                }

                write_be16(&s7_tx[6], 1U);
                write_be16(&s7_tx[8], 0U);
                tx_param[0] = func_code;
                return 13U;
            }

            /* ----------------------------------------------------------------
             * 0x1B / 0xFB: Download Block Chunk (Direct TCP Push)
             * ---------------------------------------------------------------- */
            case S7_FUNC_DOWNLOAD_BLOCK:
            case S7_FUNC_DOWNLOAD_BLOCK_EXT: {
                if (data_len >= 4U) {
                    uint16_t chunk_len = read_be16(&rx_data[0]);
                    if (chunk_len > (uint16_t)(data_len - 4U)) {
                        chunk_len = (uint16_t)(data_len - 4U);
                    }
                    const uint8_t *chunk_payload = rx_data + 4U;
                    uint16_t offset = g_block_xfer_offset;

                    if (g_dl_active_block_type == S7_DL_BLOCK_OB1) {
                        uint32_t flash_addr = FLASH_OB1_RAW_BASE_ADDR + offset;
                        for (uint16_t i = 0; i < chunk_len; i += 2U) {
                            uint8_t b0 = chunk_payload[i];
                            uint8_t b1 = ((uint16_t)(i + 1U) < chunk_len) ? chunk_payload[i + 1U] : 0xFFU;
                            uint16_t hw = (uint16_t)(((uint16_t)b1 << 8) | b0);
                            (void)FUN_08036ca0(flash_addr + i, hw);
                        }
                    } else if (g_dl_active_block_type == S7_DL_BLOCK_DB1) {
                        uint8_t *db1_dst = MEM_PTR(SRAM_DB1_HEADER_ADDR);
                        if ((uint32_t)offset + chunk_len <= 0x3D24U) {
                            memcpy(db1_dst + offset, chunk_payload, chunk_len);
                        }
                    } else if (g_dl_active_block_type == S7_DL_BLOCK_SDB) {
                        uint8_t *sdb_stage = MEM_PTR(SRAM_V_BASE_ADDR);
                        if ((uint32_t)offset + chunk_len <= 0x2800U) {
                            memcpy(sdb_stage + offset, chunk_payload, chunk_len);
                        }
                    }
                    g_block_xfer_offset = (uint16_t)(offset + chunk_len);
                }

                write_be16(&s7_tx[6], 2U);
                write_be16(&s7_tx[8], 0U);
                tx_param[0] = func_code;
                tx_param[1] = 0x00U;
                return 14U;
            }

            /* ----------------------------------------------------------------
             * 0x1C / 0xFC: Download Ended (Commit to Flash & Compile)
             * ---------------------------------------------------------------- */
            case S7_FUNC_DOWNLOAD_ENDED:
            case S7_FUNC_DOWNLOAD_ENDED_EXT: {
                if (g_dl_active_block_type != S7_DL_BLOCK_NONE) {
                    if (g_dl_active_block_type == S7_DL_BLOCK_OB1 && g_block_xfer_offset > 0U) {
                        g_ob1_block_len = g_block_xfer_offset;
                    } else if (g_dl_active_block_type == S7_DL_BLOCK_DB1 && g_block_xfer_offset > 0U) {
                        g_db1_block_len = g_block_xfer_offset;
                    } else if (g_dl_active_block_type == S7_DL_BLOCK_SDB && g_block_xfer_offset > 0U) {
                        g_sdb_block_len = g_block_xfer_offset;
                    }
                    s7_commit_downloaded_block_to_flash(g_dl_active_block_type);
                    g_dl_active_block_type = S7_DL_BLOCK_NONE;
                    g_block_xfer_offset    = 0U;
                }

                write_be16(&s7_tx[6], 1U);
                write_be16(&s7_tx[8], 0U);
                tx_param[0] = func_code;
                return 13U;
            }

            default:
                break;
        }
    }

    /* ------------------------------------------------------------------------
     * Branch B: ROSCTR == 0x07 (S7comm Userdata / Programmer Subfunctions)
     * ------------------------------------------------------------------------ */
    if (rosctr == S7_ROSCTR_USERDATA) {
        s7_tx[1] = S7_ROSCTR_USERDATA;
        uint8_t *ud_param = s7_tx + 10U;
        uint16_t out_param_len = (param_len <= 12U) ? param_len : 12U;
        memcpy(ud_param, rx_param, out_param_len);

        uint8_t *ud_data = ud_param + out_param_len;
        uint8_t subfunc_grp = (param_len >= 6U) ? rx_param[5] : 0U;
        uint8_t subfunc_cmd = (param_len >= 7U) ? rx_param[6] : 0U;

        /* 1. CPU Identification ("CPU 226   CN    02010100") & Block Sizes */
        if ((subfunc_grp == 0x01U && subfunc_cmd == 0x01U) ||
            (data_len >= 6U && rx_data[4] == 0x00U && rx_data[5] == 0x11U)) {
            ud_data[0] = S7_RET_OK;
            ud_data[1] = S7_TS_OCTET_STRING;
            write_be16(&ud_data[2], 40U);
            memcpy(&ud_data[4], CONST_MEM_PTR(FLASH_CPU_ID_STRING_ADDR), 24U);
            write_be16(&ud_data[28], g_ob1_block_len);
            write_be16(&ud_data[30], g_db1_block_len);
            write_be16(&ud_data[32], g_sdb_block_len);
            write_be16(&ud_data[34], 0x2800U); /* Max V memory = 10240 bytes */
            ud_data[36] = g_plc_run_mode;
            ud_data[37] = MEM_U8(SRAM_SDB_PRIV_LEVEL_ADDR);
            write_be16(&ud_data[38], g_ob1_compile_err_code);
            write_be16(&ud_data[40], 0x0000U);
            write_be16(&s7_tx[6], out_param_len);
            write_be16(&s7_tx[8], 44U);
            return (uint16_t)(10U + out_param_len + 44U);
        }

        /* 2. Password Authentication (Compare 8-byte password at 0x2000596A / 0x08068524) */
        if (subfunc_grp == 0x05U || (param_len >= 8U && rx_param[7] == 0x57U)) {
            if (data_len >= 12U && s7_verify_password(&rx_data[4])) {
                g_password_auth_ok_eth = 1U;
                ud_data[0] = S7_RET_OK;
                ud_data[1] = S7_TS_OCTET_STRING;
                write_be16(&ud_data[2], 0U);
            } else {
                ud_data[0] = S7_RET_ACCESS_FAULT;
                ud_data[1] = S7_TS_NULL;
                write_be16(&ud_data[2], 0U);
                if (out_param_len >= 12U) {
                    ud_param[10] = 0xD6U;
                    ud_param[11] = 0x02U;
                }
            }
            write_be16(&s7_tx[6], out_param_len);
            write_be16(&s7_tx[8], 4U);
            return (uint16_t)(10U + out_param_len + 4U);
        }

        /* 3. Real-Time Clock (RTC) Read & Write */
        if (subfunc_grp == 0x07U) {
            if (subfunc_cmd == 0x01U) {
                /* Read PLC Clock -> Return 8-byte BCD timestamp */
                ud_data[0] = S7_RET_OK;
                ud_data[1] = S7_TS_OCTET_STRING;
                write_be16(&ud_data[2], 8U);
                ud_data[4]  = 0x00U;
                ud_data[5]  = g_rtc_bcd_year;
                ud_data[6]  = g_rtc_bcd_month;
                ud_data[7]  = g_rtc_bcd_day;
                ud_data[8]  = g_rtc_bcd_hour;
                ud_data[9]  = g_rtc_bcd_min;
                ud_data[10] = g_rtc_bcd_sec;
                ud_data[11] = g_rtc_bcd_dow;
                write_be16(&s7_tx[6], out_param_len);
                write_be16(&s7_tx[8], 12U);
                return (uint16_t)(10U + out_param_len + 12U);
            } else if (subfunc_cmd == 0x02U && data_len >= 12U) {
                /* Write PLC Clock -> Stage BCD timestamp into 0x20000024..0x2000002A & commit */
                uint8_t *rtc_stage = g_rtc_write_buf;
                for (uint8_t i = 0; i < 7U; i++) {
                    rtc_stage[i] = rx_data[5U + i];
                }
                FUN_0802fd3c();
                ud_data[0] = S7_RET_OK;
                ud_data[1] = S7_TS_NULL;
                write_be16(&ud_data[2], 0U);
                write_be16(&s7_tx[6], out_param_len);
                write_be16(&s7_tx[8], 4U);
                return (uint16_t)(10U + out_param_len + 4U);
            }
        }

        /* 4. Force Variables (Force / Unforce Single / Unforce All / Read Force Table) */
        if (subfunc_grp == 0x06U || (param_len >= 8U && rx_param[7] == 0x10U)) {
            if (subfunc_cmd == 0x04U) {
                /* Unforce All Variables */
                s7_unforce_all_variables();
                ud_data[0] = S7_RET_OK;
                ud_data[1] = S7_TS_NULL;
                write_be16(&ud_data[2], 0U);
                write_be16(&s7_tx[6], out_param_len);
                write_be16(&s7_tx[8], 4U);
                return (uint16_t)(10U + out_param_len + 4U);
            } else if (subfunc_cmd == 0x01U && data_len >= 12U) {
                /* Apply Force Item */
                uint8_t  f_area   = rx_data[4];
                uint8_t  f_sub    = rx_data[5];
                uint16_t f_offset = read_be16(&rx_data[6]);
                uint8_t  f_bitlen = rx_data[8];
                s7_apply_force_item(f_area, f_sub, f_offset, f_bitlen, &rx_data[9]);
                ud_data[0] = S7_RET_OK;
                ud_data[1] = S7_TS_NULL;
                write_be16(&ud_data[2], 0U);
                write_be16(&s7_tx[6], out_param_len);
                write_be16(&s7_tx[8], 4U);
                return (uint16_t)(10U + out_param_len + 4U);
            }
        }

        /* 5. Online Status Monitoring (Chart Status / Program Status via 0x20005C20 & 0x20005570) */
        if (subfunc_grp == 0x02U) {
            if (subfunc_cmd == 0x01U && data_len > 4U) {
                /* Configure watched status items into 0x20005C20 and arm single-scan capture */
                uint16_t cfg_len = (uint16_t)(data_len - 4U);
                if (cfg_len > 500U) cfg_len = 500U;
                memcpy(MEM_PTR(SRAM_STATUS_CFG_BUF_ADDR), rx_data + 4U, cfg_len);
                g_status_monitor_trigger = 1U;
            }
            uint16_t snap_len = g_status_resp_len;
            if (snap_len > 220U) snap_len = 220U;
            ud_data[0] = S7_RET_OK;
            ud_data[1] = S7_TS_OCTET_STRING;
            write_be16(&ud_data[2], snap_len);
            if (snap_len > 0U) {
                memcpy(&ud_data[4], CONST_MEM_PTR(SRAM_STATUS_RESP_BUF_ADDR), snap_len);
            }
            write_be16(&s7_tx[6], out_param_len);
            write_be16(&s7_tx[8], (uint16_t)(4U + snap_len));
            return (uint16_t)(10U + out_param_len + 4U + snap_len);
        }

        /* 6. Clear PLC Memory Command */
        if (subfunc_grp == 0x08U) {
            uint8_t clear_mask = (data_len >= 5U) ? rx_data[4] : 0x07U;
            s7_clear_plc_memory(clear_mask);
            ud_data[0] = S7_RET_OK;
            ud_data[1] = S7_TS_NULL;
            write_be16(&ud_data[2], 0U);
            write_be16(&s7_tx[6], out_param_len);
            write_be16(&s7_tx[8], 4U);
            return (uint16_t)(10U + out_param_len + 4U);
        }

        /* Default Userdata Positive Ack */
        ud_data[0] = S7_RET_OK;
        ud_data[1] = S7_TS_OCTET_STRING;
        write_be16(&ud_data[2], 0U);
        write_be16(&s7_tx[6], out_param_len);
        write_be16(&s7_tx[8], 4U);
        return (uint16_t)(10U + out_param_len + 4U);
    }

    return 0U;
}

/* ============================================================================
 * Main Reconstructed Entry Point: FUN_080047ac (0x080047ac..0x0800fb4d)
 * ============================================================================ */

void s7_comm_eth_process_packet(
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
    uint8_t state = *req_state;

    /* ========================================================================
     * Phase 1 (req_state == 1): Bridged PPI SD1 (0x10) Short Frame / Token Poll
     * (Firmware 0x080047e0..0x08004d38)
     * ======================================================================== */
    if (state == 1U) {
        if (rx_buf[0] != PPI_SD1 || rx_buf[5] != PPI_ED ||
            rx_buf[4] != (uint8_t)(rx_buf[1] + rx_buf[2] + rx_buf[3])) {
            *req_state = 0U;
            return;
        }

        /* If a PLC-initiated Block Download is active and ready to poll for next chunk */
        if (g_dl_active_block_type != S7_DL_BLOCK_NONE && g_dl_eth_poll_ready == 1U) {
            static const uint8_t s_dl_req_template[30] = {
                0x32U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x12U, 0x00U, 0x00U,
                0x1BU, 0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x09U,
                '_',   '0',   'B',   '0',   '0',   '0',   '0',   '0',   'P',
                0x00U, 0x16U
            };
            g_dl_eth_poll_ready = 0U;
            tx_buf[0] = PPI_SD2;
            tx_buf[1] = 0x1FU;
            tx_buf[2] = 0x1FU;
            tx_buf[3] = PPI_SD2;
            tx_buf[4] = (uint8_t)(rx_buf[2] & 0x7FU);
            tx_buf[5] = *station_addr;
            tx_buf[6] = PPI_FC_S7_REQ_ODD;
            memcpy(&tx_buf[7], s_dl_req_template, sizeof(s_dl_req_template));

            write_be16(&tx_buf[11], g_dl_chunk_seq_num);
            g_dl_chunk_seq_num++;

            if (g_dl_active_block_type == S7_DL_BLOCK_OB1) {
                tx_buf[28] = '8';
                tx_buf[33] = '1';
            } else if (g_dl_active_block_type == S7_DL_BLOCK_DB1) {
                tx_buf[28] = 'A';
                tx_buf[33] = '1';
            }

            tx_buf[35] = ppi_calc_fcs(&tx_buf[4], 0x1FU);
            tx_buf[36] = PPI_ED;
            *tx_len_ptr = 0x25U;
            *req_state  = 0U;
            return;
        }

        if (rx_buf[3] == PPI_FC_DATA_POLL) {
            /* Master is polling for the deferred response queued during SD2 (req_state == 2) */
            if (*rx_sd_state == PPI_SD2) {
                *tx_len_ptr  = *saved_tx_len;
                *rx_sd_state = 0U;
                if (g_dl_active_block_type != S7_DL_BLOCK_NONE) {
                    g_dl_eth_poll_ready = 1U;
                }
            } else if (*rx_sd_state == PPI_SC) {
                tx_buf[1]   = PPI_SC;
                *tx_len_ptr = 1U;
                *rx_sd_state = 0U;
            }

            /* Check if a downloaded block is awaiting Flash commit after final Ack */
            if (g_dl_commit_pending_type != S7_DL_BLOCK_NONE) {
                s7_commit_downloaded_block_to_flash(g_dl_commit_pending_type);
                g_dl_commit_pending_type = S7_DL_BLOCK_NONE;
            }
        } else if (rx_buf[3] == PPI_FC_FDL_STATUS_REQ) {
            /* Station Status Request (0x49) */
            tx_buf[1] = PPI_SD1;
            tx_buf[2] = rx_buf[2];
            tx_buf[3] = rx_buf[1];
            tx_buf[4] = (rx_buf[2] < 0x80U) ? PPI_FC_RESP_STATUS_OK : PPI_FC_RESP_STATUS_MASTER;
            tx_buf[5] = (uint8_t)(tx_buf[2] + tx_buf[3] + tx_buf[4]);
            tx_buf[6] = PPI_ED;
            *tx_len_ptr = 6U;
        }

        *req_state = 0U;
        return;
    }

    /* ========================================================================
     * Phase 2 (req_state == 2): Bridged PPI SD2 (0x68) Variable-Length Frame
     * (Firmware 0x08004d3a..0x0800880e)
     * ======================================================================== */
    if (state == 2U) {
        uint16_t rx_len = *rx_len_ptr;
        if (rx_len < 9U || rx_buf[0] != PPI_SD2 || rx_buf[3] != PPI_SD2 ||
            rx_buf[1] != rx_buf[2] || rx_buf[rx_len - 1U] != PPI_ED) {
            *req_state = 0U;
            return;
        }

        uint8_t fcs = ppi_calc_fcs(&rx_buf[4], rx_buf[1]);
        if (fcs != rx_buf[rx_len - 2U]) {
            *req_state = 0U;
            return;
        }

        /* Immediately acknowledge with 0xE5 (SC) and build response in tx_buf[1..] */
        tx_buf[0]   = PPI_SC;
        *tx_len_ptr = 1U;
        *rx_sd_state = PPI_SD2;

        uint8_t *resp_frame = tx_buf + 1U;
        uint16_t s7_pdu_len = s7_process_pdu_core(&rx_buf[7], (uint16_t)(rx_len - 9U), &resp_frame[7]);
        if (s7_pdu_len > 0U) {
            uint8_t le = (uint8_t)(s7_pdu_len + 3U);
            resp_frame[0] = PPI_SD2;
            resp_frame[1] = le;
            resp_frame[2] = le;
            resp_frame[3] = PPI_SD2;
            resp_frame[4] = (uint8_t)(rx_buf[5] & 0x7FU);
            resp_frame[5] = *station_addr;
            resp_frame[6] = PPI_FC_S7_RESP_DATA;
            resp_frame[7U + s7_pdu_len] = ppi_calc_fcs(&resp_frame[4], le);
            resp_frame[8U + s7_pdu_len] = PPI_ED;
            *saved_tx_len = (uint16_t)(s7_pdu_len + 9U);
        }

        *req_state = 0U;
        return;
    }

    /* ========================================================================
     * Phase 3 (req_state == 3): Native ISO-on-TCP (RFC 1006 TPKT + COTP + S7comm)
     * (Firmware 0x08008810..0x0800fb4d)
     * ======================================================================== */
    if (state == 3U) {
        uint16_t rx_len = *rx_len_ptr;
        if (rx_len < 7U || rx_buf[0] != TPKT_VERSION || rx_buf[1] != TPKT_RESERVED) {
            *req_state = 0U;
            return;
        }

        uint8_t cotp_pdu_type = (uint8_t)(rx_buf[5] & 0xF0U);

        /* --------------------------------------------------------------------
         * 3A. COTP Connection Request (0xE0 CR) -> Connection Confirm (0xD0 CC)
         * -------------------------------------------------------------------- */
        if (cotp_pdu_type == COTP_PDU_TYPE_CR) {
            uint16_t pkt_len = read_be16(&rx_buf[2]);
            if (pkt_len > 64U) {
                pkt_len = 22U;
            }
            memcpy(tx_buf, rx_buf, pkt_len);
            tx_buf[5] = COTP_PDU_TYPE_CC; /* 0xD0 Connection Confirm */
            /* Swap Destination Reference (bytes 6..7) with Source Reference (bytes 8..9) */
            tx_buf[6] = rx_buf[8];
            tx_buf[7] = rx_buf[9];
            tx_buf[8] = 0x00U;
            tx_buf[9] = 0x01U;
            *tx_len_ptr = pkt_len;
            *req_state  = 0U;
            return;
        }

        /* --------------------------------------------------------------------
         * 3B. COTP Data Transfer (0xF0 DT) -> S7comm PDU (0x32)
         * -------------------------------------------------------------------- */
        if (cotp_pdu_type == COTP_PDU_TYPE_DT && rx_len >= 17U && rx_buf[7] == S7COMM_PROTO_ID) {
            uint16_t s7_rx_len = (uint16_t)(rx_len - 7U);
            uint16_t s7_tx_len = s7_process_pdu_core(&rx_buf[7], s7_rx_len, &tx_buf[7]);

            if (s7_tx_len > 0U) {
                uint16_t total_tpkt_len = (uint16_t)(7U + s7_tx_len);
                tx_buf[0] = TPKT_VERSION;
                tx_buf[1] = TPKT_RESERVED;
                write_be16(&tx_buf[2], total_tpkt_len);
                tx_buf[4] = 0x02U;
                tx_buf[5] = COTP_PDU_TYPE_DT;
                tx_buf[6] = COTP_DT_EOT_FLAG;
                *tx_len_ptr = total_tpkt_len;
            }
        }

        *req_state = 0U;
    }
}

void FUN_080047ac(
    uint8_t  *param_1,
    uint16_t *param_2,
    uint8_t  *param_3,
    uint16_t *param_4,
    uint16_t *param_5,
    uint8_t  *param_6,
    uint8_t  *param_7,
    uint8_t  *param_8
)
{
    s7_comm_eth_process_packet(param_1, param_2, param_3, param_4, param_5, param_6, param_7, param_8);
}
