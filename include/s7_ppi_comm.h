/**
 * @file s7_ppi_comm.h
 * @brief SIMATIC S7-226 (APM32E103) S7comm (ISO-on-TCP / RFC 1006) & S7-200 PPI
 *        (RS-485 Point-to-Point Interface) Communication Protocol Engine Definitions.
 *
 * Reconstructed from APM32E103 firmware regions:
 *   - 0x080047ac..0x0800fb4d : FUN_080047ac (Ethernet ISO-on-TCP / S7comm & PPI-over-TCP Engine)
 *   - 0x08017450..0x08021787 : FUN_08017450 (RS-485 PPI Multi-Drop Serial Protocol Engine)
 *   - 0x08021794..0x0802189f : FUN_08021794 (Fletcher-16 Block Checksum Verifier)
 *   - 0x080218a0..0x08021947 : FUN_080218a0 (Weighted Sums Block Checksum Verifier)
 *   - 0x08004300..0x080043b5 : FUN_08004300 (OB1 Nibble-XOR Block De-obfuscator)
 */

#ifndef S7_PPI_COMM_H
#define S7_PPI_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 1. S7-200 PPI (RS-485 Point-to-Point Interface) Serial Frame Constants
 * ============================================================================ */

/** PPI / IEC 60870-5-101 / Profibus FDL Frame Delimiters */
#define PPI_SD1                         0x10U   /**< Start Delimiter 1: Fixed-length frame (10 DA SA FC FCS 16) */
#define PPI_SD2                         0x68U   /**< Start Delimiter 2: Variable-length frame (68 L L 68 DA SA FC ... FCS 16) */
#define PPI_ED                          0x16U   /**< End Delimiter (0x16) */
#define PPI_SC                          0xE5U   /**< Single-Character Short Acknowledgment (0xE5) */
#define PPI_SD4                         0xDCU   /**< Start Delimiter 4: Token passing frame (DC DA SA) */

/** PPI Serial Port Indices passed to FUN_08017450 */
#define PPI_PORT_0_USART1               0U      /**< Port 0 (RS-485 USART1) */
#define PPI_PORT_1_USART2               1U      /**< Port 1 (RS-485 USART2) */
#define PPI_PORT_2_INTERNAL             2U      /**< Port 2 (Auxiliary / Expansion bus) */

/** PPI Dispatch Request States (param_9 of FUN_08017450) */
#define PPI_REQ_STATE_IDLE              0U      /**< Idle / No complete frame pending */
#define PPI_REQ_STATE_SD1_POLL          1U      /**< SD1 (0x10) short frame / token / data poll received */
#define PPI_REQ_STATE_SD2_DATA          2U      /**< SD2 (0x68) variable-length S7comm/PPI request frame received */

/** PPI FDL Control / Function Codes (FC byte at frame[3] in SD1 or frame[6] in SD2) */
#define PPI_FC_RESP_STATUS_OK           0x00U   /**< FDL Status Response (Slave ready) */
#define PPI_FC_RESP_STATUS_MASTER       0x02U   /**< FDL Status Response (Station in ring / ready) */
#define PPI_FC_S7_RESP_DATA             0x08U   /**< FDL User Data Response (Ack_Data / Userdata reply) */
#define PPI_FC_FDL_STATUS_REQ           0x49U   /**< FDL Request Status of Station (Poll / bus scan) */
#define PPI_FC_DATA_POLL                0x5CU   /**< FDL Request Data Poll (Master polls after receiving 0xE5 SC) */
#define PPI_FC_S7_REQ_EVEN              0x6CU   /**< FDL Send Data with Acknowledge (FCB=0, FCV=1) */
#define PPI_FC_S7_REQ_ODD               0x7CU   /**< FDL Send Data with Acknowledge (FCB=1, FCV=1) */
#define PPI_FC_ADDR_HIGH_BIT            0x80U   /**< Extended PG/Routing address flag on DA/SA */

/* ============================================================================
 * 2. ISO-on-TCP (RFC 1006) TPKT & ISO 8073 COTP Header Constants
 * ============================================================================ */

#define TPKT_VERSION                    0x03U   /**< RFC 1006 TPKT Version 3 */
#define TPKT_RESERVED                   0x00U   /**< TPKT Reserved byte (0x00) */
#define TPKT_HEADER_LEN                 4U      /**< 4-byte TPKT header: [03, 00, len_hi, len_lo] */

/** COTP (ISO 8073) PDU Types */
#define COTP_PDU_TYPE_CR                0xE0U   /**< Connection Request (CR) */
#define COTP_PDU_TYPE_CC                0xD0U   /**< Connection Confirm (CC) */
#define COTP_PDU_TYPE_DR                0x80U   /**< Disconnect Request (DR) */
#define COTP_PDU_TYPE_DT                0xF0U   /**< Data Transfer (DT) */
#define COTP_DT_EOT_FLAG                0x80U   /**< Last Data Unit / End of TSDU flag in COTP DT */
#define COTP_DT_HEADER_LEN              3U      /**< 3-byte COTP DT header: [02, F0, 80] */

/* ============================================================================
 * 3. S7comm Protocol Constants, ROSCTR Types & Function Codes
 * ============================================================================ */

#define S7COMM_PROTO_ID                 0x32U   /**< S7comm Protocol Magic Byte (0x32) */

/** S7comm ROSCTR (Remote Operating Service Control) Message Types */
#define S7_ROSCTR_JOB                   0x01U   /**< Job Request: Master -> PLC command */
#define S7_ROSCTR_ACK                   0x02U   /**< Ack: Simple acknowledgment without data */
#define S7_ROSCTR_ACK_DATA              0x03U   /**< Ack_Data: Acknowledgment with response payload */
#define S7_ROSCTR_USERDATA              0x07U   /**< Userdata: Programmer/diagnostic/status/force subfunctions */

/** S7comm Job / Ack_Data Function Codes (Parameter[0]) */
#define S7_FUNC_READ_VAR                0x04U   /**< Read Variable (Read memory areas) */
#define S7_FUNC_WRITE_VAR               0x05U   /**< Write Variable (Write memory areas) */
#define S7_FUNC_REQ_DOWNLOAD            0x1AU   /**< Request Block Download (Standard 0x1A) */
#define S7_FUNC_DOWNLOAD_BLOCK          0x1BU   /**< Download Block Chunk (Standard 0x1B) */
#define S7_FUNC_DOWNLOAD_ENDED          0x1CU   /**< Download Ended (Standard 0x1C) */
#define S7_FUNC_START_UPLOAD            0x1DU   /**< Start Block Upload (0x1D) */
#define S7_FUNC_UPLOAD                  0x1EU   /**< Upload Block Chunk (0x1E) */
#define S7_FUNC_END_UPLOAD              0x1FU   /**< End Block Upload (0x1F) */
#define S7_FUNC_PI_SERVICE              0x28U   /**< Program Invocation Service (P_PROGRAM RUN / Block Link / _GARB) */
#define S7_FUNC_PLC_STOP                0x29U   /**< PLC Stop (P_STOP -> Transition to STOP mode) */
#define S7_FUNC_SETUP_COMM              0xF0U   /**< Setup Communication (Negotiate PDU size & max jobs) */
#define S7_FUNC_REQ_DOWNLOAD_EXT        0xFAU   /**< S7-200 Extended Request Download (0xFA) */
#define S7_FUNC_DOWNLOAD_BLOCK_EXT      0xFBU   /**< S7-200 Extended Download Block (0xFB) */
#define S7_FUNC_DOWNLOAD_ENDED_EXT      0xFCU   /**< S7-200 Extended Download Ended (0xFC) */

/** S7comm Item Return Codes (Data Section) */
#define S7_RET_RESERVED                 0x00U
#define S7_RET_HW_FAULT                 0x01U   /**< Hardware error */
#define S7_RET_ACCESS_FAULT             0x03U   /**< Object access not allowed / privilege error */
#define S7_RET_INVALID_ADDR             0x05U   /**< Invalid address / out of range */
#define S7_RET_DATA_TYPE_UNSUPPORTED    0x06U   /**< Data type not supported */
#define S7_RET_DATA_TYPE_INCONSISTENT   0x07U   /**< Data type inconsistent */
#define S7_RET_OBJECT_NOT_EXIST         0x0AU   /**< Object does not exist */
#define S7_RET_OK                       0xFFU   /**< Success (0xFF) */

/** S7comm Data Transport Size Codes (Data Item Header byte 1) */
#define S7_TS_NULL                      0x00U
#define S7_TS_BIT                       0x03U   /**< Bit access */
#define S7_TS_BYTE_WORD_DWORD           0x04U   /**< Byte / Word / DWord (length in bits in some headers, or bytes) */
#define S7_TS_INTEGER                   0x05U   /**< Integer */
#define S7_TS_REAL                      0x07U   /**< IEEE-754 Float */
#define S7_TS_OCTET_STRING              0x09U   /**< Octet string (length in bytes) */

/* ============================================================================
 * 4. S7-200 Memory Area Codes & Variable Type Selectors
 * ============================================================================ */

/**
 * S7-200 Native Area / Type Codes (used in PPI & S7comm Status Monitoring,
 * Force tables, and S7-200 specific item descriptors):
 */
#define S7200_AREA_OB1                  0x03U   /**< Program Block OB1 / Code area (0x03) */
#define S7200_AREA_IO_PORT              0x13U   /**< Physical I/O & Analog (I, Q, AI, AQ) */
#define S7200_AREA_S                    0x14U   /**< Sequential Control Relay (S) */
#define S7200_AREA_V_DB1                0x18U   /**< Variable Memory (V / DB1) */
#define S7200_AREA_M                    0x1CU   /**< Marker / Bit Memory (M) */
#define S7200_AREA_C                    0x1DU   /**< Counter (C) */
#define S7200_AREA_T                    0x1EU   /**< Timer (T) */
#define S7200_AREA_HC                   0x1FU   /**< High-Speed Counter (HC) */
#define S7200_AREA_SM                   0x20U   /**< Special Memory (SM) */

/**
 * S7comm S7-Any Item Area Selectors (12-byte item spec: 12 0A 10 <type> <count> <db> <area> <bit_addr_24>):
 */
#define S7_AREA_SYS_INFO                0x03U   /**< System info / PLC Status / Force / Diagnostic area */
#define S7_AREA_S                       0x04U   /**< Sequential Control Relay S (0x20001024, 32 bytes) */
#define S7_AREA_SM                      0x05U   /**< Special Memory SM (0x20005f28, 550 bytes) */
#define S7_AREA_AI                      0x06U   /**< Analog Inputs AI (0x20005e1a, 128 bytes) */
#define S7_AREA_AQ                      0x07U   /**< Analog Outputs AQ (0x20005e98, 128 bytes) */
#define S7_AREA_C                       0x1EU   /**< S7-200 Counter C (256 counters: status + uint16 CV) */
#define S7_AREA_T                       0x1FU   /**< S7-200 Timer T (256 timers: status + uint16/uint32 CV) */
#define S7_AREA_HC                      0x20U   /**< S7-200 High-Speed Counter HC (0x20006150, uint32 CV) */
#define S7_AREA_I                       0x81U   /**< Process Image Input I (0x20005bd0, 32 bytes) */
#define S7_AREA_Q                       0x82U   /**< Process Image Output Q (0x20001046, 32 bytes) */
#define S7_AREA_M                       0x83U   /**< Bit Memory / Flags M (0x20001000, 32 bytes) */
#define S7_AREA_V_DB1                   0x84U   /**< Variable Memory V / DB1 (0x2000108c, 10240 bytes) */

/** S7-Any Parameter Transport Type Codes (Item byte +3) */
#define S7_PARAM_TYPE_BIT               0x01U   /**< Bit (1 bit) */
#define S7_PARAM_TYPE_BYTE              0x02U   /**< Byte (1 byte) */
#define S7_PARAM_TYPE_CHAR              0x03U   /**< Char (1 byte) */
#define S7_PARAM_TYPE_WORD              0x04U   /**< Word (2 bytes) */
#define S7_PARAM_TYPE_INT               0x05U   /**< Int (2 bytes) */
#define S7_PARAM_TYPE_DWORD             0x06U   /**< DWord (4 bytes) */
#define S7_PARAM_TYPE_DINT              0x07U   /**< DInt (4 bytes) */
#define S7_PARAM_TYPE_REAL              0x08U   /**< Real (4 bytes) */
#define S7_PARAM_TYPE_C                 0x1DU   /**< S7-200 Counter descriptor (3 bytes: status + uint16 CV) */
#define S7_PARAM_TYPE_T                 0x1EU   /**< S7-200 Timer descriptor (5 bytes: status + 32-bit CV) */
#define S7_PARAM_TYPE_HC                0x1FU   /**< S7-200 High-Speed Counter descriptor (4 bytes: uint32 CV) */

/** Block Download / Upload Target Identifiers (DAT_20000487) */
#define S7_DL_BLOCK_NONE                0U
#define S7_DL_BLOCK_SDB                 1U      /**< "_0B00000P" -> System Data Block (Flash 0x0807f800, RAM 0x200058cc) */
#define S7_DL_BLOCK_OB1                 2U      /**< "_0800001P" -> Program Block OB1 (Flash 0x08075800 / 0x0806d800) */
#define S7_DL_BLOCK_DB1                 3U      /**< "_0A00001P" -> Data Block DB1 (Flash 0x08064800, RAM 0x20001068) */

/* ============================================================================
 * 5. APM32E103 Firmware Flash & SRAM Memory Map Addresses
 * ============================================================================ */

/** Flash Storage Regions */
#define FLASH_OB1_XOR_TABLE_ADDR        0x0804A0F4UL /**< 256-byte (16x16) nibble XOR de-obfuscation table */
#define FLASH_DEFAULT_OB1_TEMPLATE_ADDR 0x0804A1F4UL /**< Default empty OB1 block template in ROM */
#define FLASH_DEFAULT_DB1_TEMPLATE_ADDR 0x0804A264UL /**< Default empty DB1 block header template in ROM */
#define FLASH_CPU_ID_STRING_ADDR        0x0804A3BCUL /**< "CPU 226   CN    02010100" identification string */
#define FLASH_DB1_BASE_ADDR             0x08064800UL /**< DB1 Data Block Flash storage (16 KB) */
#define FLASH_PASSWORD_STORAGE_ADDR     0x08068524UL /**< DB1 Flash tail / Password & checksum storage */
#define FLASH_DB1_CHECKSUM_ADDR         0x08068526UL /**< 16-bit Fletcher checksum of DB1 in Flash */
#define FLASH_OB1_COMPILED_BASE_ADDR    0x0806D800UL /**< Compiled / De-obfuscated OB1 execution image (32 KB) */
#define FLASH_OB1_RAW_BASE_ADDR         0x08075800UL /**< Raw downloaded OB1 block image in Flash (32 KB) */
#define FLASH_SDB_BASE_ADDR             0x0807F800UL /**< System Data Block (SDB0) Flash storage (384 bytes) */

/** SRAM PLC Memory Areas */
#define SRAM_M_BASE_ADDR                0x20001000UL /**< Marker M memory (32 bytes: M0.0..M31.7) */
#define SRAM_S_BASE_ADDR                0x20001024UL /**< Sequential Control Relay S (32 bytes: S0.0..S31.7) */
#define SRAM_Q_BASE_ADDR                0x20001046UL /**< Output Process Image Q (32 bytes: Q0.0..Q31.7) */
#define SRAM_DB1_HEADER_ADDR            0x20001068UL /**< DB1 36-byte block header in RAM */
#define SRAM_V_BASE_ADDR                0x2000108CUL /**< Variable V memory (10,240 bytes: VB0..VB10239) */
#define SRAM_T_CURRENT_VAL_ADDR         0x20004000UL /**< Timers T0..T255 16-bit current values (uint16_t[256]) */
#define SRAM_T_STATUS_BITS_ADDR         0x2000422CUL /**< Timers T0..T255 status bytes (bit 1 = timer output bit) */
#define SRAM_C_CURRENT_VAL_ADDR         0x20004440UL /**< Counters C0..C255 16-bit current values (uint16_t[256]) */
#define SRAM_C_STATUS_BITS_ADDR         0x20005000UL /**< Counters C0..C255 status bytes (bit 4 = counter output bit) */
#define SRAM_STATUS_RESP_BUF_ADDR       0x20005570UL /**< Online Status Monitoring response snapshot buffer */
#define SRAM_SDB_MIRROR_ADDR            0x200058CCUL /**< SDB RAM mirror (384 bytes: 0x200058CC..0x20005A4B) */
#define SRAM_SDB_PRIV_LEVEL_ADDR        0x20005969UL /**< Configured password protection level (1..4) in SDB RAM */
#define SRAM_SDB_PASSWORD_ADDR          0x2000596AUL /**< 8-byte encoded password in SDB RAM (0x2000596A..0x20005971) */
#define SRAM_FORCE_I_MASK_ADDR          0x20005AB0UL /**< Force bitmask for Inputs I (16 bytes) */
#define SRAM_FORCE_I_VAL_ADDR           0x20005AC0UL /**< Forced values for Inputs I (16 bytes) */
#define SRAM_FORCE_Q_MASK_ADDR          0x20005AD0UL /**< Force bitmask for Outputs Q (16 bytes) */
#define SRAM_FORCE_Q_VAL_ADDR           0x20005AE0UL /**< Forced values for Outputs Q (16 bytes) */
#define SRAM_FORCE_TABLE_ADDR           0x20005AF0UL /**< 16-entry non-bit Force Table (16 * 14 bytes = 224 bytes) */
#define SRAM_I_BASE_ADDR                0x20005BD0UL /**< Input Process Image I (32 bytes: I0.0..I31.7) */
#define SRAM_STATUS_CFG_BUF_ADDR        0x20005C20UL /**< Online Status Monitoring item configuration buffer */
#define SRAM_AI_BASE_ADDR               0x20005E1AUL /**< Analog Inputs AIW0..AIW126 (128 bytes) */
#define SRAM_AQ_BASE_ADDR               0x20005E98UL /**< Analog Outputs AQW0..AQW126 (128 bytes) */
#define SRAM_SM_BASE_ADDR               0x20005F28UL /**< Special Memory SMB0..SMB549 (550 bytes) */
#define SRAM_HC_CURRENT_VAL_ADDR        0x20006150UL /**< High-Speed Counters HC0..HC19 (uint32_t[20]) */

/* ============================================================================
 * 6. Protocol Header Structures (Packed, Big-Endian Wire Format)
 * ============================================================================ */

#pragma pack(push, 1)

/** RFC 1006 TPKT Header (4 bytes) */
typedef struct {
    uint8_t  version;       /**< Always 0x03 (TPKT_VERSION) */
    uint8_t  reserved;      /**< Always 0x00 */
    uint8_t  length_hi;     /**< Total packet length high byte */
    uint8_t  length_lo;     /**< Total packet length low byte */
} s7_tpkt_hdr_t;

/** ISO 8073 COTP Data Transfer (DT) Header (3 bytes) */
typedef struct {
    uint8_t  header_len;    /**< Header length minus 1 (0x02 for DT) */
    uint8_t  pdu_type;      /**< 0xF0 (COTP_PDU_TYPE_DT) */
    uint8_t  tpdu_nr_eot;   /**< 0x80 (TPDU number 0 + EOT bit set) */
} s7_cotp_dt_hdr_t;

/** PPI Variable-Length Frame Header (SD2 = 0x68, 7 bytes before S7comm PDU) */
typedef struct {
    uint8_t  sd2_start;     /**< 0x68 (PPI_SD2) */
    uint8_t  le;            /**< Frame payload length (DA + SA + FC + PDU) */
    uint8_t  ler;           /**< Repeated payload length (must equal le) */
    uint8_t  sd2_repeat;    /**< 0x68 (PPI_SD2) */
    uint8_t  da;            /**< Destination station address (0..126, +0x80 if routed) */
    uint8_t  sa;            /**< Source station address (0..126, +0x80 if routed) */
    uint8_t  fc;            /**< FDL Function Code (0x6C/0x7C request, 0x08 response) */
} ppi_sd2_hdr_t;

/** PPI Fixed-Length Frame (SD1 = 0x10, 6 bytes total) */
typedef struct {
    uint8_t  sd1;           /**< 0x10 (PPI_SD1) */
    uint8_t  da;            /**< Destination station address */
    uint8_t  sa;            /**< Source station address */
    uint8_t  fc;            /**< Function code (0x5C data poll, 0x49 status poll, 0x00/0x02 status resp) */
    uint8_t  fcs;           /**< Checksum: (da + sa + fc) & 0xFF */
    uint8_t  ed;            /**< 0x16 (PPI_ED) */
} ppi_sd1_frame_t;

/** S7comm Standard 10-Byte Header (Job=0x01, Ack=0x02, Userdata=0x07) */
typedef struct {
    uint8_t  proto_id;      /**< 0x32 (S7COMM_PROTO_ID) */
    uint8_t  rosctr;        /**< 0x01=Job, 0x02=Ack, 0x03=Ack_Data, 0x07=Userdata */
    uint16_t red_id;        /**< Redundancy ID (0x0000) */
    uint16_t pdu_ref;       /**< Protocol Data Unit Reference (echoed in response) */
    uint16_t param_len_be;  /**< Parameter section length (big-endian) */
    uint16_t data_len_be;   /**< Data section length (big-endian) */
} s7comm_hdr_t;

/** S7comm 12-Byte Ack_Data Header (ROSCTR = 0x03, includes 2-byte error class/code) */
typedef struct {
    s7comm_hdr_t base;      /**< 10-byte base header */
    uint8_t  err_class;     /**< Error class (0x00 = no error) */
    uint8_t  err_code;      /**< Error code (0x00 = no error) */
} s7comm_ack_data_hdr_t;

/** S7comm 12-Byte S7-Any Variable Address Specification (Read/Write Var item) */
typedef struct {
    uint8_t  var_spec;      /**< 0x12 (Variable specification) */
    uint8_t  addr_len;      /**< 0x0A (10 bytes following) */
    uint8_t  syntax_id;     /**< 0x10 (S7-Any syntax ID) */
    uint8_t  transport_type;/**< S7_PARAM_TYPE_* (0x01=Bit, 0x02=Byte, 0x04=Word, 0x06=DWord, 0x1D=C, 0x1E=T, 0x1F=HC) */
    uint8_t  count_hi;      /**< Repetition factor / element count high byte */
    uint8_t  count_lo;      /**< Repetition factor / element count low byte */
    uint8_t  db_num_hi;     /**< DB number high byte (0x0001 for V memory, 0x0000 for others) */
    uint8_t  db_num_lo;     /**< DB number low byte */
    uint8_t  area;          /**< S7_AREA_* (0x81=I, 0x82=Q, 0x83=M, 0x84=V, 0x04=S, 0x05=SM, 0x06=AI, 0x07=AQ, 0x1E=C, 0x1F=T, 0x20=HC) */
    uint8_t  bit_addr_hi;   /**< 24-bit bit offset high byte (byte_offset >> 13) */
    uint8_t  bit_addr_mid;  /**< 24-bit bit offset middle byte ((byte_offset >> 5) & 0xFF) */
    uint8_t  bit_addr_lo;   /**< 24-bit bit offset low byte (((byte_offset & 0x1F) << 3) | bit_num) */
} s7_any_item_t;

/** S7-200 14-Byte Force Table Entry (stored at SRAM_FORCE_TABLE_ADDR = 0x20005AF0) */
typedef struct {
    uint8_t  area;          /**< Memory area (0x18=V, 0x1C=M, 0x14=S, 0x20=SM, 0x13=AI/AQ, 0x1D=C, 0x1E=T, 0x1F=HC) */
    uint8_t  sub_area;      /**< Sub-area or size code (1=Byte, 2=Word, 4=DWord, or 0x06=AI / 0x07=AQ) */
    uint8_t  bit_mask;      /**< Bit mask or active flag */
    uint8_t  reserved;      /**< Padding / alignment */
    uint16_t byte_offset;   /**< Target byte offset or element index (little-endian) */
    uint8_t  value[4];      /**< Forced value (1, 2, or 4 bytes in big-endian order) */
    uint8_t  extra[4];      /**< Auxiliary timer/counter force state bytes */
} s7200_force_entry_t;

#pragma pack(pop)

/* ============================================================================
 * 7. Reconstructed Protocol Engine API Prototypes
 * ============================================================================ */

/**
 * @brief Ethernet ISO-on-TCP (RFC 1006) / S7comm & PPI-over-TCP Protocol Engine.
 *        Reconstructed from FUN_080047ac (0x080047ac..0x0800fb4d).
 *
 * Processes an incoming TCP payload containing either:
 *   - An ISO-on-TCP TPKT (0x03 0x00 ...) + COTP (CR 0xE0 or DT 0xF0) + S7comm (0x32) PDU, or
 *   - A raw PPI serial frame (SD1 0x10 or SD2 0x68) tunneled over Ethernet/serial bridge.
 *
 * @param rx_buf        Pointer to received frame buffer (param_1)
 * @param rx_len_ptr    Pointer to received byte count (param_2)
 * @param tx_buf        Pointer to transmit response buffer (param_3)
 * @param tx_len_ptr    Pointer to transmit byte count output (param_4)
 * @param saved_tx_len  Pointer to cached response length for PPI SD1 poll (param_5)
 * @param rx_sd_state   Pointer to received Start Delimiter state (0x10 / 0x68 / 0xE5) (param_6)
 * @param station_addr  Pointer to local station address (param_7)
 * @param req_state     Pointer to protocol engine dispatch state (param_8)
 */
void s7_comm_eth_process_packet(
    uint8_t  *rx_buf,
    uint16_t *rx_len_ptr,
    uint8_t  *tx_buf,
    uint16_t *tx_len_ptr,
    uint16_t *saved_tx_len,
    uint8_t  *rx_sd_state,
    uint8_t  *station_addr,
    uint8_t  *req_state
);

/**
 * @brief RS-485 PPI Multi-Drop Serial Protocol Engine (Port 0 USART1 / Port 1 USART2).
 *        Reconstructed from FUN_08017450 (0x08017450..0x08021787).
 *
 * Handles STEP 7-Micro/WIN, TD 200, and HMI multi-master PPI frames:
 *   - req_state == 1 (SD1 0x10): Token/Station Status Poll, Queued Response Transmission,
 *     or PLC-Initiated Block Download Request (ROSCTR=0x01, Func=0x1B).
 *   - req_state == 2 (SD2 0x68): Variable-length S7comm/PPI request validation, immediate
 *     0xE5 (SC) single-character ack setup, and full S7comm response preparation in tx_buf[1..].
 *
 * @param port_id       Serial port index (0 = Port 0 USART1, 1 = Port 1 USART2, 2 = Internal) (param_1)
 * @param rx_buf        Pointer to received frame buffer (param_2)
 * @param rx_len_ptr    Pointer to received frame length (param_3)
 * @param tx_buf        Pointer to transmit buffer (tx_buf[0] holds 0xE5 SC ack, tx_buf[1..] holds deferred SD2 frame) (param_4)
 * @param tx_len_ptr    Pointer to immediate transmit byte count (param_5)
 * @param saved_tx_len  Pointer to deferred response length sent on next SD1 poll (param_6)
 * @param rx_sd_state   Pointer to pending response delimiter type (0x68 SD2 or 0xE5 SC) (param_7)
 * @param station_addr  Pointer to local PPI station address (param_8)
 * @param req_state     Pointer to current PPI request state (1=SD1, 2=SD2, reset to 0 on exit) (param_9)
 */
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
);

/**
 * @brief Verifies the 16-bit Fletcher-like checksum appended to a raw PLC block
 *        (such as DB1 at 0x08064800 or raw OB1 at 0x08075800).
 *        Reconstructed from FUN_08021794 (0x08021794..0x0802189f).
 *
 * @param block_ptr     Pointer to start of block data
 * @param block_len     Number of bytes in block body (2-byte checksum follows at block_ptr + block_len)
 * @return 0 if checksum matches, 1 if checksum mismatch
 */
uint8_t s7_block_verify_fletcher_checksum(const uint8_t *block_ptr, uint16_t block_len);

/**
 * @brief Verifies the 16-bit weighted checksum appended to the compiled OB1 block
 *        at 0x0806d800 or SDB block.
 *        Reconstructed from FUN_080218a0 (0x080218a0..0x08021947).
 *
 * @param block_ptr     Pointer to start of block data
 * @param block_len     Number of bytes in block body (2-byte checksum follows at block_ptr + block_len)
 * @return 0 if checksum matches, 1 if checksum mismatch
 */
uint8_t s7_block_verify_weighted_checksum(const uint8_t *block_ptr, uint16_t block_len);

/**
 * @brief De-obfuscates a downloaded OB1 block payload using the 16x16 nibble XOR lookup
 *        table at 0x0804A0F4 and writes the decoded bytes into Flash at *flash_write_addr.
 *        Reconstructed from FUN_08004300 (0x08004300..0x080043b5).
 *
 * @param src_buf           Pointer to obfuscated OB1 block bytes in RAM
 * @param length            Number of bytes to decode and program
 * @param flash_write_addr  In/Out pointer to target Flash address (starts at 0x0806D800)
 */
void s7_block_deobfuscate_ob1(const uint8_t *src_buf, uint16_t length, uint32_t *flash_write_addr);

/* Ghidra symbol aliases for direct cross-reference compatibility */
void    FUN_080047ac(uint8_t *param_1, uint16_t *param_2, uint8_t *param_3, uint16_t *param_4,
                     uint16_t *param_5, uint8_t *param_6, uint8_t *param_7, uint8_t *param_8);
void    FUN_08017450(uint8_t param_1, uint8_t *param_2, uint16_t *param_3, uint8_t *param_4,
                     uint16_t *param_5, uint16_t *param_6, uint8_t *param_7, uint8_t *param_8,
                     uint8_t *param_9);
uint8_t FUN_08021794(const uint8_t *param_1, uint16_t param_2);
uint8_t FUN_080218a0(const uint8_t *param_1, uint16_t param_2);
void    FUN_08004300(const uint8_t *param_1, uint16_t param_2, uint32_t *param_3);

#ifdef __cplusplus
}
#endif

#endif /* S7_PPI_COMM_H */
