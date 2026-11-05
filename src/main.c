/**
 * @file main.c
 * @brief Complete Reconstructed Main Executive (`main` / `FUN_080433e4`)
 *        for SIMATIC S7-226 PLC Firmware (APM32E103 / STM32F103HD Cortex-M3)
 *
 * Flash Address Range: 0x080433E4 - 0x08049F1D (27,442 bytes)
 *
 * Logical Architecture of `main` (`FUN_080433e4`):
 *   - Section 1 (0x080433E4): Power-On vs. IWDG Watchdog Reset Check & State Init
 *   - Section 2 (0x08043470): PLC Memory Area Descriptor Table Init (0x20000250..0x2000027B)
 *   - Section 3 (0x080434E0): GPIO Ports A..E, UART1..3, SPI2 & W5500 Hardware Init
 *   - Section 4 (0x08043548): System Data Block (SDB) Load (0x0807F800 -> 0x200058CC),
 *                             Checksum Verification & UART/DMA/TIM6/TIM7 Configuration
 *   - Section 5 (0x08043720): Cold-Boot Retentive RAM Restore (0x08069000) & DB1 Header Copy
 *   - Section 6 (0x08043880): Author Signature, Special Memory (SMB) Defaults &
 *                             96-Bit STM32 Hardware UID / License Check (0x1FFFF7E8 / 0x08052000)
 *   - Section 7 (0x08043A90): Backup Domain (BKP DR1 0xA5A5), LSE RTC, TIM2 (100us) & SysTick (1ms)
 *   - Section 8 (0x08043B10): Main Cyclic PLC Scan Loop:
 *       8.1 Timed Interrupts (SMB34/SMB35), RTC Calendar & S7 Timer Resolution Updates
 *       8.2 OB1 User Program Execution (`FUN_08013d96` -> `plc_vm_execute` `0x08021948`)
 *       8.3 Communication Port 0/1/2 Frame Servicing (`FUN_08017450`)
 *       8.4 Local Digital Input Filtering, Analog Potentiometers (SMB28/29) & ADC Scaling
 *       8.5 Expansion Module (EM0..EM6) Bus Discovery & Input Data Exchange
 *       8.6 Primary PPI / S7 Protocol Servicing (`FUN_080047ac`) & Flash Block Programming
 *       8.7 Local Digital Output Refresh (`FUN_0801520e`) & Power-Fail Retentive Save (`FUN_08037746`)
 *       8.8 Dynamic Freeport / Baudrate Reconfiguration (SMB30/SMB130) & W5500 Polling
 *       8.9 Built-in Modbus RTU Slave Handler (Ports 0 & 1, CRC-16 `FUN_08000e0e`)
 *       8.10 Expansion Module Output Refresh, I/O Forcing & RUN/STOP/SF Status LEDs
 */

#include "s7_226_plc.h"

undefined4 FUN_080433e4(void)

{
  int iVar1;
  byte bVar2;
  byte bVar3;
  byte bVar4;
  byte bVar5;
  undefined2 uVar6;
  ushort uVar7;
  int iVar8;
  undefined4 uVar9;
  uint uVar10;
  volatile undefined2 *puVar11;
  undefined1 uVar12;
  char cVar13;
  uint uVar14;
  uint uVar15;
  volatile undefined1 *puVar16;
  volatile byte *pbVar17;
  volatile undefined1 *puVar18;
  uint uVar19;
  byte extraout_r3;
  byte bVar20;
  uint extraout_r3_00;
  uint extraout_r3_01;
  uint uVar21;
  volatile byte *pbVar22;
  volatile byte *pbVar23;
  volatile byte *pbVar24;
  uint uVar25;
  byte bVar26;
  volatile undefined1 *extraout_r12;
  volatile undefined1 *extraout_r12_00;
  volatile undefined1 *extraout_r12_01;
  uint uVar27;
  bool bVar28;
  bool bVar29;
  bool bVar30;
  undefined8 uVar31;
  ulonglong uVar32;
  volatile undefined1 *in_stack_ffffffc0;
  volatile undefined1 *in_stack_ffffffc8;
  volatile undefined1 *in_stack_ffffffcc;
  

  /* ==========================================================================
   * SECTION 1: Power-On vs. Watchdog Reset Check & Runtime Flag Initialization
   *   - Checks RCC_CSR IWDGRSTF (flag 0x7D) via FUN_08036c70(0x7d).
   *   - If cold power-on reset (iVar8 == 0), clears warm-reset flag DAT_200001d0.
   *   - If watchdog reset, increments diagnostic reset counters (0x20003896/38a8).
   *   - Clears process-image input/status area (0x20005bd0..0x20005bef) and
   *     sets default BCD Time-Of-Day (87-11-25 15:32:21 Wed at 0x2000017d..0183).
   * ========================================================================== */
  iVar8 = FUN_08036c70(0x7d);
  if (iVar8 == 0) {
    DAT_200001d0 = '\0';
  }
  else {
    DAT_20003896 = DAT_20003896 + '\x01';
    DAT_200038a8 = DAT_200038a8 + '\x01';
  }
  DAT_20000145 = '\0';
  DAT_200001a0 = 0;
  DAT_20000184 = '\0';
  DAT_20000185 = '\0';
  DAT_20000186 = 0;
  uVar25 = 0;
  DAT_20000187 = 0;
  DAT_200005f2 = 0;
  DAT_200005f3 = 0;
  DAT_200001ba = 0;
  DAT_200001bb = 0;
  DAT_200001bf = 0;
  DAT_200001c0 = 0;
  DAT_20000042 = 0;
  DAT_2000003e = 0;
  DAT_20000040 = 0;
  DAT_20000100 = 0;
  DAT_20000151 = 0;
  DAT_20000152 = 0;
  DAT_20000153 = 0;
  DAT_20000154 = 0;
  DAT_2000014b = 0;
  DAT_2000014c = 0;
  DAT_2000014d = 0;
  DAT_2000014e = 0;
  DAT_2000014f = 0;
  DAT_20000150 = 0;
  DAT_200001c7 = '\0';
  DAT_200001c5 = 1;
  DAT_200001c3 = 1;
  uVar14 = 0;
  do {
    (&DAT_20005bd0)[uVar14] = 0;
    uVar14 = uVar14 + 1 & 0xffff;
  } while (uVar14 < 0x20);
  DAT_20005f18 = 0;
  DAT_20005f1c = 0;
  DAT_20005f20 = 0;
  DAT_20005f24 = 0;
  DAT_2000017d = 0x87;
  DAT_2000017e = 0x11;
  DAT_2000017f = 0x25;
  DAT_20000180 = 0x15;
  DAT_20000181 = 0x32;
  DAT_20000182 = 0x21;
  DAT_20000183 = 3;

  /* ==========================================================================
   * SECTION 2: PLC Memory Area Descriptor Table Initialization (0x20000250..0x2000027B)
   *   - 0x20000250 = 0x1800108C : Area 0x18 -> V memory (DB1) at 0x2000108C
   *   - 0x20000254 = 0x1E001000 : Area 0x1E -> T status/flags at 0x20001000
   *   - 0x20000258 = 0x1D001024 : Area 0x1D -> C status/flags at 0x20001024
   *   - 0x2000025C = 0x1F001046 : Area 0x1F -> HC current values at 0x20001046
   *   - 0x20000260 = 0x20005BD0 : Area 0x20 -> I/Q/SM base at 0x20005BD0
   *   - 0x20000264 = 0x14005E1A : Area 0x14 -> S (SCR) memory at 0x20005E1A
   *   - 0x20000268 = 0x13005E98 : Area 0x13 -> AI/AQ/SM image at 0x20005E98
   *   - 0x2000026C = 0x1C005F28 : Area 0x1C -> M / SMB memory at 0x20005F28
   *   - 0x20000270 = 0x17004000 : Area 0x17 -> Timer current words at 0x20004000
   *   - 0x20000274 = 0x16004440 : Area 0x16 -> Counter current words at 0x20004440
   *   - 0x20000278 = 0x0306D800 : Area 0x03 -> Compiled OB1 in Flash at 0x0806D800
   * ========================================================================== */
  DAT_20000250 = 0x1800108c;
  DAT_20000254 = 0x1e001000;
  DAT_20000258 = 0x1d001024;
  DAT_2000025c = 0x1f001046;
  DAT_20000260 = 0x20005bd0UL; /* &DAT_20005bd0 */
  DAT_20000264 = 0x14005e1a;
  DAT_20000268 = 0x13005e98;
  DAT_2000026c = 0x1c005f28;
  DAT_20000270 = 0x17004000;
  DAT_20000274 = 0x16004440;
  DAT_20000278 = 0x306d800;
  DAT_200006d2 = 0;
  DAT_200006d4 = 0;
  DAT_200006d6 = 0;
  DAT_200005ed = 0;
  DAT_200007e9 = 0;
  DAT_200007ea = 0;
  DAT_200006d8 = 0;
  DAT_200007e8 = 0;
  DAT_200001c2 = 0;
  DAT_200001ec = 0;
  DAT_20000220 = 0;

  /* ==========================================================================
   * SECTION 3: GPIO Port, UART, SPI2 & W5500 Ethernet Hardware Initialization
   *   - Sets GPIOA..GPIOE CRL/CRH = 0x88888888 (inputs with pull-up/pull-down).
   *   - Initializes local digital inputs (0x08015c1a), digital outputs & EM bus
   *     (0x08017228), USART1..3 GPIOs, SPI2 & W5500 network defaults.
   * ========================================================================== */
  DAT_40010800 = 0x88888888;
  DAT_40010804 = 0x88888888;
  DAT_40010c00 = 0x88888888;
  DAT_40010c04 = 0x88888888;
  DAT_40011000 = 0x88888888;
  DAT_40011004 = 0x88888888;
  DAT_40011400 = 0x88888888;
  DAT_40011404 = 0x88888888;
  DAT_40011800 = 0x88888888;
  DAT_40011804 = 0x88888888;
  FUN_08015c1a();
  FUN_08017228();
  FUN_080402d4();
  FUN_08040e8e();
  FUN_08041c0e();
  FUN_08014278();
  FUN_080422c6();
  FUN_08036e52();
  FUN_08036e86();
  FUN_08015400();
  FUN_080043b8(8000);

  /* ==========================================================================
   * SECTION 4: System Data Block (SDB) Load, Validation & Baudrate/UART Setup
   *   - Copies 0x180 bytes from Flash SDB (0x0807F800) to SRAM (0x200058CC).
   *   - Verifies SDB checksums via FUN_0803ed26(). On mismatch, forces STOP mode
   *     (DAT_2000016f = 0) and sets error code 5 (DAT_20000188 = 5).
   *   - Configures Port 0 (USART1), Port 1 (USART2), Port 2 (USART3), DMA
   *     channels, and frame turnaround timers TIM6/TIM7.
   * ========================================================================== */
  FUN_08037468(&DAT_0807f800,&DAT_200058cc,0x180);
  FUN_0803ed26();
  if (((DAT_20005a36 != DAT_2000021c || DAT_20005a37 != DAT_2000021d) ||
      DAT_20005a4a != DAT_2000021e) || DAT_20005a4b != DAT_2000021f) {
    DAT_2000016f = '\0';
    DAT_20000188 = 5;
  }
  DAT_20000174 = DAT_20005991;
  if (((DAT_20005991 != '\0') && (DAT_20005991 != '\x02')) && (DAT_20005991 != '\x04')) {
    DAT_20000174 = '\0';
  }
  DAT_20000175 = DAT_200058fb;
  if (((DAT_200058fb != '\0') && (DAT_200058fb != '\x02')) && (DAT_200058fb != '\x04')) {
    DAT_20000175 = '\0';
  }
  if (DAT_20000174 == '\0') {
    uVar9 = 0x2580;
  }
  else if (DAT_20000174 == '\x02') {
    uVar9 = 0x4b00;
  }
  else {
    uVar9 = 0x2dc6c;
  }
  FUN_08040e18(uVar9,2,9,1);
  FUN_08003ef8();
  FUN_08041fbe(&DAT_40013800,0x80,1);
  FUN_08041fbe(&DAT_40013800,0x40,1);
  FUN_08042042(&DAT_40013800,0x424,1);
  FUN_08042042(&DAT_40013800,0x626,1);
  FUN_08003ee2(&DAT_40020044,0);
  FUN_08003ee2(&DAT_40020058,1);
  if (DAT_20000175 == '\0') {
    uVar9 = 0x2580;
  }
  else if (DAT_20000175 == '\x02') {
    uVar9 = 0x4b00;
  }
  else {
    uVar9 = 0x2dc6c;
  }
  FUN_08041b98(uVar9,2,9,1);
  FUN_08003fbe();
  FUN_08041fbe(&DAT_40004400,0x80,1);
  FUN_08041fbe(&DAT_40004400,0x40,1);
  FUN_08042042(&DAT_40004400,0x424,1);
  FUN_08042042(&DAT_40004400,0x626,1);
  FUN_08003ee2(&DAT_40020080,0);
  FUN_08003ee2(&DAT_4002006c,1);
  FUN_08041f44(0x2580,2,9,1);
  FUN_08004084();
  FUN_08041fbe(&DAT_40004800,0x80,1);
  FUN_08041fbe(&DAT_40004800,0x40,1);
  FUN_08042042(&DAT_40004800,0x424,1);
  FUN_08042042(&DAT_40004800,0x626,1);
  FUN_08003ee2(&DAT_4002001c,0);
  FUN_08003ee2(&DAT_40020030,1);
  DAT_2000003f = 2;
  FUN_08016c18();
  FUN_08040228();
  FUN_08016c42();
  FUN_0804027e();
  DAT_20000186 = 0;
  DAT_20000187 = 0;
  DAT_200005f2 = '\0';
  DAT_200005f3 = '\0';
  DAT_200001ba = '\0';
  DAT_200001bb = '\0';
  DAT_200001bf = '\0';
  DAT_200001c0 = '\0';
  DAT_20000042 = 0;
  DAT_2000003e = 0;
  DAT_20000043 = 0;
  DAT_20000040 = '\0';
  DAT_20000100 = '\0';
  DAT_200004d8 = 0;
  DAT_200004da = 0;
  DAT_2000051d = 0;
  FUN_08014526(&DAT_40011000,0x2003);
  FUN_0801452a(&DAT_40011000,0x1000);
  FUN_08014526(&DAT_40010800,0x180);
  FUN_08014526(&DAT_40011400,0x3f);
  FUN_08014526(&DAT_40011000,0x300);
  FUN_0801452a(&DAT_40011000,0x800);
  DAT_20000179 = 1;
  DAT_20000176 = '\0';
  DAT_20000177 = '\0';
  DAT_20000178 = '\x01';
  DAT_20000188 = '\0';

  /* ==========================================================================
   * SECTION 5: Retentive Memory Restore (Cold vs. Warm Boot) & DB1 Header Copy
   *   - On cold boot (DAT_200001d0 == 0), clears T, C, V, M, HC areas and calls
   *     FUN_08037494() to restore retentive memory ranges from Flash 0x08069000.
   *   - Copies 0x24-byte DB1 header from Flash 0x08064800 to SRAM 0x20001068.
   * ========================================================================== */
  if (DAT_200001d0 == '\0') {
    FUN_0801452a(&DAT_40010c00,0xa000);
    uVar14 = 0x20;
    uVar10 = 0;
    do {
      (&DAT_2000422c)[uVar14] = 0;
      (&DAT_20004000)[uVar14] = 0;
      uVar14 = uVar14 + 1 & 0xffff;
    } while (uVar14 < 0x40);
    uVar14 = 0x60;
    do {
      (&DAT_2000422c)[uVar14] = 0;
      (&DAT_20004000)[uVar14] = 0;
      uVar14 = uVar14 + 1 & 0xffff;
    } while (uVar14 < 0x100);
    uVar14 = 0;
    do {
      (&DAT_2000422c)[uVar14] = 0;
      (&DAT_20004000)[uVar14] = 0;
      uVar14 = uVar14 + 1 & 0xffff;
    } while (uVar14 < 0x20);
    uVar14 = 0x40;
    do {
      (&DAT_2000422c)[uVar14] = 0;
      (&DAT_20004000)[uVar14] = 0;
      uVar14 = uVar14 + 1 & 0xffff;
    } while (uVar14 < 0x60);
    uVar14 = 0;
    do {
      (&DAT_20005000)[uVar14] = 0;
      (&DAT_20004440)[uVar14] = 0;
      uVar14 = uVar14 + 1 & 0xffff;
    } while (uVar14 < 0x100);
    uVar14 = 0;
    do {
      (&DAT_20001000)[uVar14] = 0;
      uVar14 = uVar14 + 1 & 0xffff;
    } while (uVar14 < 0x20);
    DAT_200001e0 = 0;
    for (; uVar10 < 0x2800; uVar10 = uVar10 + 1 & 0xffff) {
      (&DAT_2000108c)[uVar10] = 0;
    }
    iVar8 = FUN_08037494();
    DAT_20000147 = (byte)iVar8;
    if (iVar8 == 0) {
      DAT_2000016f = '\0';
      DAT_20000188 = '\x05';
      DAT_200001e0 = 0;
      for (uVar14 = 0; uVar14 < 0x2800; uVar14 = uVar14 + 1 & 0xffff) {
        (&DAT_2000108c)[uVar14] = 0;
      }
    }
    uVar14 = 0;
    do {
      (&DAT_20001024)[uVar14] = 0;
      uVar14 = uVar14 + 1 & 0xffff;
    } while (uVar14 < 0x20);
    uVar14 = 0;
    do {
      (&DAT_20001046)[uVar14] = 0;
      uVar14 = uVar14 + 1 & 0xffff;
    } while (uVar14 < 0x20);
    DAT_200038a8 = '\0';
    DAT_20000141 = '\0';
  }
  uVar14 = 0;
  do {
    (&DAT_20001068)[uVar14] = (&DAT_08064800)[uVar14];
    uVar14 = uVar14 + 1 & 0xffff;
  } while (uVar14 < 0x24);
  DAT_200038a9 = 0;
  DAT_200038aa = 0;
  uVar14 = 0x2843;
  do {
    uVar10 = uVar14 + 1 & 0xffff;
    (&DAT_2000108c)[uVar14] = 0;
    uVar14 = uVar10;
  } while (uVar10 < 0x297c);
  uVar14 = 0x20;
  do {
    (&DAT_20004000)[uVar14] = 0;
    (&DAT_2000422c)[uVar14] = 0;
    uVar14 = uVar14 + 1 & 0xffff;
  } while (uVar14 < 0x40);
  uVar14 = 0x60;
  do {
    (&DAT_20004000)[uVar14] = 0;
    (&DAT_2000422c)[uVar14] = 0;
    uVar14 = uVar14 + 1 & 0xffff;
  } while (uVar14 < 0x100);
  uVar14 = 0;
  do {
    (&DAT_20005a4c)[uVar14] = (char)uVar14 + '\n';
    uVar14 = uVar14 + 1 & 0xffff;
  } while (uVar14 < 100);

  /* ==========================================================================
   * SECTION 6: Author Signature, Special Memory (SM) Defaults & Hardware UID Check
   *   - Writes author signature bytes to 0x20005A4C..0x20005A6B.
   *   - Initializes Special Memory (SMB0..SMB549 at 0x20005F28..) hardware defaults.
   *   - Calls FUN_0803bf24() and reads the 96-bit STM32 Unique Device ID from
   *     0x1FFFF7E8..0x1FFFF7F0 into 0x2000511C..0x20005128, computing UID hash
   *     fields (0x200038B6..0x200038BD).
   *   - Compiles/verifies user program OB1 via FUN_0803ece0().
   * ========================================================================== */
  DAT_20005a4c = 0x50;
  DAT_20005a4f = 0x47;
  DAT_20005a4d = 0x52;
  DAT_20005a4e = 0x4f;
  DAT_20005a52 = 0x4d;
  DAT_20005a50 = 0x52;
  DAT_20005a54 = 0x45;
  DAT_20005a51 = 0x41;
  DAT_20005a53 = 0x4d;
  DAT_20005a56 = 0x2d;
  DAT_20005a55 = 0x44;
  DAT_20005a59 = 0x55;
  DAT_20005a57 = 0x2d;
  DAT_20005a58 = 0x53;
  DAT_20005a5d = 0x49;
  DAT_20005a5a = 0x20;
  DAT_20005a5e = 0x4e;
  DAT_20005a5b = 0x41;
  DAT_20005a5c = 0x4d;
  DAT_20005a61 = 0x5f;
  DAT_20005a5f = 0x47;
  DAT_20005a63 = 0x61;
  DAT_20005a60 = 0x5f;
  DAT_20005a62 = 0x48;
  DAT_20005a65 = 0x4c;
  DAT_20005a64 = 0x69;
  DAT_20005a68 = 0x4d;
  DAT_20005a66 = 0x61;
  DAT_20005a67 = 0x6e;
  DAT_20005a6b = 0x67;
  DAT_20005a69 = 0x69;
  DAT_20005a6a = 0x6e;
  DAT_20005a6d = 0x75;
  DAT_20005a6c = 0x48;
  DAT_20005a70 = 0x75;
  DAT_20005a6e = 0x61;
  DAT_20005a6f = 0x59;
  DAT_20005a72 = 0x6e;
  DAT_20005a71 = 0x61;
  DAT_20005a75 = 0x61;
  DAT_20005a73 = 0x4a;
  DAT_20005a77 = 0x67;
  DAT_20005a74 = 0x69;
  DAT_20005a79 = 0x69;
  DAT_20005a76 = 0x6e;
  DAT_20005a7b = 0x43;
  DAT_20005a78 = 0x59;
  DAT_20005a7d = 0x74;
  DAT_20005a7a = 0x6e;
  DAT_20005a7c = 0x69;
  DAT_20005a7e = 0x79;
  DAT_20000284 = (DAT_200002b0 & 0xffffff) + 36000000;
  DAT_20005f28 = 8;
  DAT_20005f2b = 0;
  DAT_20005f2c = 0x60;
  DAT_20005f2e = 0x2a;
  DAT_20005f47 = 0;
  DAT_20005f48 = 0;
  DAT_20005f49 = 0;
  FUN_0803bf24();
  DAT_2000511c = DAT_1ffff7e8;
  DAT_20005120 = DAT_1ffff7ec;
  DAT_20005124 = DAT_1ffff7f0;
  DAT_20005128 = DAT_20005124 + (DAT_2000511c >> 2) + (DAT_20005120 >> 1);
  DAT_200038ba = (undefined1)((uint)DAT_20005128 >> 0x18);
  DAT_200038bb = (undefined1)((uint)DAT_20005128 >> 0x10);
  DAT_200038bc = (undefined1)((uint)DAT_20005128 >> 8);
  DAT_200038bd = (undefined1)DAT_20005128;
  DAT_2000510c = DAT_20005104 ^ 0xab232100;
  DAT_200038b6 = (undefined1)(DAT_20005104 >> 0x18);
  DAT_200038b7 = (undefined1)(DAT_20005104 >> 0x10);
  DAT_200038b8 = (undefined1)(DAT_20005104 >> 8);
  DAT_200038b9 = (undefined1)DAT_20005104;
  FUN_0803ece0();
  uVar14 = DAT_e000ed0c;
  uVar14 = (uVar14 & 0x7ff) >> 8;
  uVar10 = 7 - uVar14;
  if (4 < uVar10) {
    uVar10 = 4;
  }
  if (uVar14 + 4 < 7) {
    uVar14 = 0;

  /* ==========================================================================
   * SECTION 7: Backup Domain (BKP DR1 0xA5A5), RTC, Timers & SysTick Startup
   *   - Enables PWR backup domain write access (BB_PWR_CR_DBP = 1).
   *   - Checks BKP->DR1 (offset 4) for magic 0xA5A5; initializes LSE & RTC on
   *     first power-up, then starts TIM2 (100us), HSC EXTI, and SysTick (1ms).
   * ========================================================================== */
  }
  else {
    uVar14 = uVar14 - 3;
  }
  DAT_e000ed23 = (char)((((1 << (uVar10 & 0xff)) - 1U & 3) << (uVar14 & 0xff)) << 4);
  iVar8 = FUN_08000dec(4);
  if (iVar8 == 0xa5a5) {
    FUN_08036de2();
    FUN_08036dd2();
  }
  else {
    FUN_08036cde();
    FUN_08000dfc(4,0xa5a5);
  }
  FUN_08036bb8();
  FUN_08000b1a();
  FUN_08000b78();
  FUN_0800428c();
  DAT_20005f2f = 0;
  if (DAT_200001d0 == '\0') {
    DAT_20005100 = 0;
  }
  DAT_200038ae = (undefined1)((uint)DAT_20005100 >> 0x18);
  DAT_200038af = (undefined1)((uint)DAT_20005100 >> 0x10);
  DAT_200038b0 = (undefined1)((uint)DAT_20005100 >> 8);
  DAT_200038b1 = (undefined1)DAT_20005100;
  DAT_40003000 = 0x5555;
  DAT_40003004 = 5;
  DAT_40003008 = 0x2a;
  DAT_40003000 = 0xaaaa;
  DAT_40003000 = 0xcccc;
  FUN_0801452a(&DAT_40011800,0x4000);
  do {

  /* ==========================================================================
   * SECTION 8: Main PLC Cyclic Executive Scan Loop
   *   8.1 Timed Interrupts (SMB34/SMB35 via TIM3/TIM4), RTC & S7 Timer Updates
   * ========================================================================== */
    do {
      DAT_20000228 = DAT_20000220;
      DAT_200005f7 = 0;
      DAT_40003000 = 0xaaaa;
      DAT_20005f2d = 0;
      uVar14 = 0;
      do {
        if ((&DAT_20005f31)[uVar14 * 2] != '\0') {
          DAT_20005f2d = 1;
        }
        uVar14 = uVar14 + 1 & 0xffff;
      } while (uVar14 < 7);
      if (DAT_20005969 == -1) {
        DAT_20005969 = '\x01';
      }
      if (DAT_2000016f == '\x01') {
        DAT_20005f28 = DAT_20005f28 | 0x82;
      }
      else {
        DAT_20005f28 = DAT_20005f28 & 0x7d;
        DAT_20005f46 = 0;
        DAT_20005faa = 0;
      }
      if ((DAT_20005f28 & 1) != 0) {
        DAT_20005f28 = DAT_20005f28 & 0xfd;
      }
      if (DAT_2000016f == '\x01') {
        DAT_20005f28 = DAT_20005f28 | 1;
      }
      else {
        DAT_20005f28 = DAT_20005f28 & 0xfe;
        DAT_20005f2c = DAT_20005f2c & 0xe0;
        FUN_0803fc1e(&DAT_40000400);
        FUN_0803fbee(&DAT_40000400,1);
        FUN_0803ff08(&DAT_40000400,1,0);
        FUN_0803fbf4(&DAT_40000400,0);
        if (DAT_20000194 != '\0') {
          DAT_20000195 = 0;
          DAT_20000199 = 1;
          DAT_20000196 = 0;
          uVar7 = DAT_40013400;
          DAT_40013400 = uVar7 & 0xfffe;
          uVar7 = DAT_40013444;
          DAT_40013444 = (ushort)(((uint)uVar7 << 0x11) >> 0x11);
          uVar7 = DAT_40000c00;
          DAT_40000c00 = uVar7 & 0xfffe;
          DAT_40000c24 = 0;
          DAT_20005fdc = DAT_20005fdc & 0xfe;
          DAT_20005fdd = DAT_20005fdd | 0x80;
          DAT_2000009c = 0;
          DAT_20000388 = 0;
          DAT_2000038c = 0;
          DAT_20000198 = 0;
          DAT_200000a8 = 0;
          DAT_200002e4 = 0;
          DAT_200000b0 = 0;
          DAT_20000197 = 0;
        }
        if (DAT_2000018d != '\0') {
          DAT_2000018e = 0;
          DAT_20000192 = 1;
          DAT_2000018f = 0;
          uVar7 = DAT_40012c00;
          DAT_40012c00 = uVar7 & 0xfffe;
          uVar7 = DAT_40012c44;
          DAT_40012c44 = (ushort)(((uint)uVar7 << 0x11) >> 0x11);
          uVar7 = DAT_40000000;
          DAT_40000000 = uVar7 & 0xfffe;
          DAT_40000024 = 0;
          DAT_20005fd2 = DAT_20005fd2 & 0xfe;
          DAT_20005fd3 = DAT_20005fd3 | 0x80;
          DAT_20000084 = 0;
          DAT_20000380 = 0;
          DAT_20000384 = 0;
          DAT_20000191 = 0;
          DAT_20000090 = 0;
          DAT_200002d8 = 0;
          DAT_20000098 = 0;
          DAT_20000190 = 0;
        }
        uVar14 = 0x10;
        do {
          (&DAT_20001046)[uVar14] = 0;
          uVar14 = uVar14 + 1 & 0xffff;
        } while (uVar14 < 0x20);
      }
      DAT_200006d8 = 2;
      DAT_200006d6 = 0;
      DAT_200005ed = DAT_200007e9 | DAT_200007ea | DAT_200007eb;
      if (DAT_2000016f == '\x01') {
        DAT_200001c6 = 1;
        DAT_200001e8 = (undefined2)DAT_20000224;
        uVar10 = DAT_20000224 / 10 & 0xffff;
        uVar14 = 1;
LAB_080440a8:
        bVar5 = (&DAT_2000422c)[uVar14];
        uVar15 = bVar5 & 0xd0;
        if ((uVar15 == 0x80) || (uVar15 == 0x50)) {
          uVar19 = (uint)*(ushort *)(&DAT_20005bf0 + uVar14 * 2);
          if (uVar10 < uVar19) {
            uVar19 = 0xffff - uVar19;
            uVar21 = uVar19 + uVar10 & 0xffff;
            if (uVar21 != 0) {
              if (uVar21 < uVar19) goto LAB_08044102;
              *(short *)(&DAT_20005bf0 + uVar14 * 2) = (short)(uVar19 + uVar10) - (short)uVar19;
            }
          }
          else {
            uVar21 = uVar10 - uVar19 & 0xffff;
            if (uVar21 != 0) {
LAB_08044102:
              *(ushort *)(&DAT_20005bf0 + uVar14 * 2) =
                   *(ushort *)(&DAT_20005bf0 + uVar14 * 2) + (short)uVar21;
            }
          }
          if (uVar14 - 1 < 4) {
            uVar25 = (uint)*(ushort *)(uVar14 * 2 + 0x20005df0);
          }
          else if (uVar14 - 0x21 < 4) {
            uVar25 = (uint)*(ushort *)(&DAT_20005db8 + uVar14 * 2);
          }
          else if (uVar14 - 0x41 < 4) {
            uVar25 = (uint)*(ushort *)(uVar14 * 2 + 0x20005d80);
          }
          else if (uVar14 - 0x61 < 4) {
            uVar25 = (uint)*(ushort *)(&DAT_20005d48 + uVar14 * 2);
          }
          if (uVar15 == 0x80) {
            if ((uVar21 + (ushort)(&DAT_20004000)[uVar14] & 0xffff) < 0x7fff) {
              (&DAT_20004000)[uVar14] = (short)(uVar21 + (ushort)(&DAT_20004000)[uVar14]);
            }
            else {
              (&DAT_20004000)[uVar14] = 0x7fff;
            }
            if ((ushort)(&DAT_20004000)[uVar14] < uVar25) {
              bVar5 = bVar5 & 0xfd;
            }
            else {
              bVar5 = bVar5 | 2;
            }
LAB_080441e6:
            (&DAT_2000422c)[uVar14] = bVar5;
          }
          else if (-1 < (int)((uint)bVar5 << 0x1a)) {
            uVar15 = uVar21 + (ushort)(&DAT_20004000)[uVar14] & 0xffff;
            if (0x7ffe < uVar15) {
              uVar15 = 0x7fff;
            }
            if (uVar15 < uVar25) {
              (&DAT_20004000)[uVar14] = (short)uVar15;
            }
            else {
              (&DAT_20004000)[uVar14] = (short)uVar25;
              uVar15 = uVar25;
            }
            if (uVar25 <= uVar15) {
              bVar5 = bVar5 & 0xfd | 0x20;
              goto LAB_080441e6;
            }
          }
        }
        uVar14 = uVar14 + 1 & 0xffff;
        if (uVar14 < 0x65) {
          if (uVar14 == 5) {
            uVar14 = 0x21;
          }
          else if (uVar14 == 0x25) {
            uVar14 = 0x41;
          }
          else if (uVar14 == 0x45) {
            uVar14 = 0x61;
          }
          goto LAB_080440a8;
        }
        DAT_20000171 = '\0';
        iVar8 = DAT_200002c8;
        if (DAT_20000195 < 0x15) {
          if (9 < DAT_20000195) goto LAB_080442b6;
          DAT_20000388 = 0;
          DAT_2000038c = 0;
          DAT_200002e8 = 0;
        }
        else {
          DAT_200000a0 = DAT_2000009c;
          uVar7 = DAT_40000c24;
          DAT_200002e0 = (uint)uVar7 + DAT_2000009c * 0x10000;
          DAT_2000038c = 0;
          DAT_20000388 = DAT_200002e0;
LAB_080442b6:
          DAT_200002e8 = DAT_20000388 + DAT_200000b0;
          if (DAT_2000019a == '\x02') {
            iVar8 = DAT_200002e8;
            if (DAT_2000018a == '\0') {
              iVar8 = -DAT_200002e8;
            }
            DAT_2000615c = DAT_200002b8 + iVar8;
            iVar8 = DAT_2000615c;
            if (DAT_20000347 == '\x01') {
              DAT_200002d0 = DAT_200061b8;
              DAT_200002c0 = DAT_2000615c;
              if (((DAT_200061b8 <= DAT_2000615c) && (DAT_200002c8 < DAT_200061b8)) ||
                 ((DAT_2000615c <= DAT_200061b8 && (DAT_200061b8 < DAT_200002c8)))) {
                DAT_2000018c = 1;
                uVar14 = DAT_40010410;
                DAT_40010410 = uVar14 | 2;
              }
            }
          }
        }
        DAT_200002c8 = iVar8;
        iVar8 = DAT_200002c4;
        if (DAT_2000018e < 0x15) {
          if (9 < DAT_2000018e) goto LAB_080443c0;
          DAT_20000380 = 0;
          DAT_20000384 = 0;
          DAT_200002dc = 0;
        }
        else {
          DAT_20000088 = DAT_20000084;
          uVar7 = DAT_40000024;
          DAT_200002d4 = (uint)uVar7 + DAT_20000084 * 0x10000;
          DAT_20000384 = 0;
          DAT_20000380 = DAT_200002d4;
LAB_080443c0:
          DAT_200002dc = DAT_20000380 + DAT_20000098;
          if (DAT_20000193 == '\x02') {
            iVar8 = DAT_200002dc;
            if (DAT_20000189 == '\0') {
              iVar8 = -DAT_200002dc;
            }
            DAT_20006150 = DAT_200002b4 + iVar8;
            iVar8 = DAT_20006150;
            if (DAT_20000344 == '\x01') {
              DAT_200002cc = DAT_200061ac;
              DAT_200002bc = DAT_20006150;
              if ((((int)DAT_200061ac <= DAT_20006150) && (DAT_200002c4 < (int)DAT_200061ac)) ||
                 ((DAT_20006150 <= (int)DAT_200061ac && ((int)DAT_200061ac < DAT_200002c4)))) {
                DAT_2000018b = 1;
                uVar14 = DAT_40010410;
                DAT_40010410 = uVar14 | 2;
              }
            }
          }
        }
        DAT_200002c4 = iVar8;
        uVar14 = (uint)DAT_2000033a;
        if ((int)(uVar14 << 0x18) < 0) {
          bVar28 = false;
          uVar10 = DAT_40011808;
          uVar10 = uVar10 & 0x1000;
          DAT_200001f6 = (ushort)uVar10;
          uVar15 = DAT_40011808;
          uVar15 = uVar15 & 0x2000;
          DAT_200001fa = (ushort)uVar15;
          if ((((DAT_20000340 == '\x02') || (DAT_20000340 == '\x05')) || (DAT_20000340 == '\b')) ||
             (DAT_20000340 == '\v')) {
            if (uVar10 == 0) {
              if (uVar14 * -0x80000000 != 0) goto LAB_08044482;
            }
            else if (uVar14 * -0x80000000 == 0) {
LAB_08044482:
              DAT_20006158 = 0;
              bVar28 = true;
            }
          }
          if (((DAT_20000340 == '\x03') || (DAT_20000340 == '\x06')) ||
             ((DAT_20000340 == '\t' || (DAT_20000340 == '\f')))) {
            if (uVar10 == 0) {
              if (uVar14 * -0x80000000 != 0) goto LAB_080444aa;
              goto LAB_080444ca;
            }
            if (uVar14 * -0x80000000 != 0) goto LAB_080444ca;
LAB_080444aa:
            if (uVar15 == 0) {
              if ((int)(uVar14 * 0x40000000) < 0) goto LAB_080444b4;
              goto LAB_080444ca;
            }
            if ((int)(uVar14 * 0x40000000) < 0) goto LAB_080444ca;
LAB_080444b4:
            DAT_20006158 = 0;
LAB_080444cc:
            if ((DAT_2000034c == '\x01') && ((DAT_200001f4 != uVar10 || (DAT_200001f8 != uVar15))))
            {
              DAT_20000352 = 1;
              uVar14 = DAT_40010410;
              DAT_40010410 = uVar14 | 2;
              DAT_200001f4 = DAT_200001f6;
              DAT_200001f8 = DAT_200001fa;
              goto LAB_08044518;
            }
          }
          else {
LAB_080444ca:
            if (bVar28) goto LAB_080444cc;

  /* ==========================================================================
   *   8.2 User Program (OB1) Execution (via FUN_08013d96 -> plc_vm_execute 0x08021948),
   *       Scan Time Calculation (SMW22/SMW24/SMW26) & HSC Updates
   * ========================================================================== */
          }
          DAT_20000352 = 0;
          DAT_200001f4 = DAT_200001f6;
          DAT_200001f8 = DAT_200001fa;
        }
LAB_08044518:
        DAT_20005f29 = 0;
        FUN_08013d96();
        DAT_20005f28 = DAT_20005f28 & 0xf7;
      }
      uVar14 = (uint)DAT_20005f2c;
      if ((int)(uVar14 << 0x18) < 0) {
        uVar10 = 0;
        do {
          uVar15 = 0;
          do {
            if (((byte)(&DAT_20005ad0)[uVar10] >> uVar15 & 1) != 0) {
              if (((byte)(&DAT_20005ae0)[uVar10] >> uVar15 & 1) == 0) {
                (&DAT_20001046)[uVar10] = (&DAT_20001046)[uVar10] & ~(byte)(1 << uVar15);
              }
              else {
                (&DAT_20001046)[uVar10] = (&DAT_20001046)[uVar10] | (byte)(1 << uVar15);
              }
            }
            uVar15 = uVar15 + 1 & 0xff;
          } while (uVar15 < 8);
          uVar10 = uVar10 + 1 & 0xff;
        } while (uVar10 < 0x10);
      }
      if (DAT_200001ec == 0 && DAT_20000188 == '\0') {
        if ((-1 < (int)(uVar14 << 0x18)) && (DAT_20000171 == '\0')) {
          uRam422201b0 = 1;
          goto LAB_080445f4;
        }
        uRam4223018c = 0;
        uRam422201b0 = 1;
      }
      else {
        uRam422201b0 = 0;
LAB_080445f4:
        uRam4223018c = 1;
      }
      if (DAT_2000018d == '\0') {
        uRam422101a0 = DAT_20001046 & 1;
      }
      if (DAT_20000194 == '\0') {
        uRam4221019c = (DAT_20001046 & 3) >> 1;
      }
      uVar14 = (uint)DAT_20001046;
      _DAT_42228180 = (uVar14 & 7) >> 2;
      _DAT_42228184 = (uVar14 & 0xf) >> 3;
      _DAT_42228188 = (uVar14 & 0x1f) >> 4;
      _DAT_4222818c = (uVar14 & 0x3f) >> 5;
      _DAT_42228190 = (uVar14 & 0x7f) >> 6;
      _DAT_42228194 = (uint)(DAT_20001046 >> 7);
      uVar14 = (uint)DAT_20001047;
      uRam42220180 = uVar14 & 1;
      uRam42220184 = (uVar14 & 3) >> 1;
      uRam422101ac = (uVar14 & 7) >> 2;
      uRam422101b0 = (uVar14 & 0xf) >> 3;
      uRam422101bc = (uVar14 & 0x1f) >> 4;
      _DAT_4221818c = (uVar14 & 0x3f) >> 5;
      _DAT_42228198 = (uVar14 & 0x7f) >> 6;
      _DAT_4222819c = (uint)(DAT_20001047 >> 7);
      if (DAT_2000016b == '\x01') {
        DAT_200002ac = ((uint)DAT_200001e2 * 0x44c) / 0x221f;
      }
      else {
        DAT_200002ac = (((uint)DAT_20005e99 + (uint)DAT_20005e98 * 0x100) * 0x44c) / 0x221f;
        if (0xfff < DAT_200002ac) {
          DAT_200002ac = 0xfff;
        }
      }
      FUN_08003ea0(0,DAT_200002ac);
      if (DAT_2000016c == '\x01') {
        DAT_200002ac = ((uint)DAT_200001e2 * 0x44c) / 0x221f;
      }
      else {
        DAT_200002ac = (((uint)DAT_20005e9b + (uint)DAT_20005e9a * 0x100) * 0x44c) / 0x221f;
        if (0xfff < DAT_200002ac) {
          DAT_200002ac = 0xfff;
        }
      }
      FUN_08003eb4(0,DAT_200002ac);
      if ((DAT_20000142 == '\0') &&
         ((DAT_20000290 = DAT_20000220 & 0x7fffff, DAT_20000290 + 0xfa6bc6ff < 9999 ||
          (DAT_200038cd == '\x06')))) {
        DAT_40003000 = 0xaaaa;
        DAT_20000142 = '\x01';
        DAT_200038cd = '\0';
        DAT_40003000 = 0xaaaa;
      }
      if (((DAT_20005bd0 != 0) || (DAT_200038ce == '\x06')) &&
         (((DAT_2000028c = DAT_20000220 & 0x7fffff, DAT_2000028c - 0x700001 < 0x4ff &&
           (DAT_20000141 == '\0')) || (DAT_200038cf == '\a')))) {
        DAT_40003000 = 0xaaaa;
        DAT_20000141 = '\x01';
        DAT_40003000 = 0xaaaa;

  /* ==========================================================================
   *   8.3 Communication Port 0 / 1 / 2 Frame Servicing (FUN_08017450)
   * ========================================================================== */
        DAT_200038ce = '\0';
        DAT_200038cf = '\0';
      }
      if (DAT_200001ec != 0) {
        DAT_200001f0 = DAT_200001ee + -0x24;
        DAT_200001f2 = DAT_200001ec;
      }
      if (DAT_200001ba != '\0') {
        DAT_200001bb = '\x01';
        DAT_200007e9 = 0;
        in_stack_ffffffcc = &DAT_200055cc;
        in_stack_ffffffc0 = &DAT_20000039;
        in_stack_ffffffc8 = &DAT_200056cc;
        DAT_200001bd = FUN_08017450(DAT_20000172,1,DAT_200001bc,&DAT_200000bc,&DAT_20000039,
                                    &DAT_200000c0,&DAT_200056cc,&DAT_200055cc,DAT_200001ba,
                                    &DAT_200057cc);
        DAT_200001ba = '\0';
      }
      if (DAT_200001bf != '\0') {
        DAT_200001c0 = '\x01';
        DAT_200007ea = 0;
        in_stack_ffffffc8 = &DAT_20000c9c;
        in_stack_ffffffcc = &DAT_20000b9c;
        in_stack_ffffffc0 = &DAT_2000003c;
        DAT_200001be = FUN_08017450(DAT_20000173,0,DAT_200001c1,&DAT_200000c4,&DAT_2000003c,
                                    &DAT_200000c8,&DAT_20000c9c,&DAT_20000b9c,DAT_200001bf,
                                    &DAT_20003a08);
        DAT_200001bf = '\0';
      }
      if (DAT_20000040 != '\0') {
        DAT_20000100 = '\x01';
        in_stack_ffffffcc = &DAT_20000d9c;
        in_stack_ffffffc0 = &DAT_2000003d;
        in_stack_ffffffc8 = &DAT_20000e9c;
        DAT_2000047c = FUN_08017450(DAT_2000003f,2,DAT_20000041,&DAT_200000cc,&DAT_2000003d,
                                    &DAT_200000d0,&DAT_20000e9c,&DAT_20000d9c,DAT_20000040,
                                    &DAT_20003b07);
        DAT_20000040 = '\0';
      }
      if (DAT_200005ed == 1) {
        uVar14 = 0;
        if (2 < DAT_200006d8) {
          if ((uint)DAT_200006d6 < (uint)DAT_200006d4) {
            uVar14 = (uint)DAT_200006d4 - (uint)DAT_200006d6 & 0xffff;
            for (uVar10 = 0; uVar10 < uVar14; uVar10 = uVar10 + 1 & 0xffff) {
              uVar15 = (uint)DAT_200006d8;
              DAT_200006d8 = DAT_200006d8 + 3;
              (&DAT_2000b919)[uVar15] = 2;
              (&DAT_2000b91a)[uVar15] = 0;
              (&DAT_2000b91b)[uVar15] = 0;
            }
          }
        }
        uVar10 = (uint)DAT_200006d8;
        if (uVar10 == 0) {
          DAT_2000031c = DAT_20000220 - DAT_20000318;
          if (99 < DAT_2000031c) {
            DAT_200001c2 = 0;
          }
        }
        else {
          DAT_20000318 = DAT_20000220;
          DAT_200001c2 = 1;
          DAT_2000081d = 0;
          DAT_2000081e = 1;
          DAT_2000081f = 0x12;
          DAT_20000820 = 8;
          DAT_20000821 = 0x12;
          DAT_20000822 = 0x88;
          DAT_20000823 = 0x11;
          if (uVar10 < 3) {
            DAT_200007e8 = 0;
            uVar15 = 0;
          }
          else {
            uVar15 = uVar10;
            if ((0xd1 < uVar10 - 3) && (uVar15 = uVar14, 0xd4 < uVar10)) {
              uVar15 = 0xd4;
            }
          }
          cVar13 = (char)uVar15;
          DAT_2000080d = cVar13 + '\x1f';
          DAT_2000081c = cVar13 + '\x06';
          DAT_2000082c = cVar13 + '\x02';
          DAT_20000824 = 0xd4 < uVar10;
          if ((bool)DAT_20000824) {
            DAT_20000825 = 0xdd;
          }
          else {
            DAT_20000825 = 0;
          }
          DAT_20000827 = 0;
          DAT_20000828 = 0;
          DAT_20000829 = 0xff;
          DAT_2000082a = 9;
          DAT_2000082b = 0;
          DAT_2000082d = 0;
          if (uVar10 < 3) {
            DAT_2000082e = 4;
          }
          else {
            DAT_2000082e = 0;
          }
          DAT_2000b919 = (undefined1)((ushort)DAT_200006d2 >> 8);
          DAT_2000b91a = (undefined1)DAT_200006d2;
          DAT_2000080e = DAT_2000080d;
          DAT_20000826 = DAT_20000824;
          for (uVar14 = 1; uVar14 <= uVar15; uVar14 = uVar14 + 1 & 0xffff) {
            (&DAT_2000082e)[uVar14] = (&DAT_2000b918)[uVar14];
          }
          if (0xd4 < uVar10) {
            DAT_2000091c = 0;
            DAT_2000091d = 1;
            DAT_2000091e = 0x12;
            DAT_2000091f = 8;
            DAT_20000920 = 0x12;
            DAT_20000921 = 0x88;
            DAT_20000922 = 0x11;
            if (uVar10 < 0x1a9) {
              uVar14 = uVar10 - 0xd4 & 0xffff;
            }
            else {
              uVar14 = 0xd4;
            }
            cVar13 = (char)uVar14;
            DAT_2000090c = cVar13 + '\x1f';
            DAT_2000091b = cVar13 + '\x06';
            DAT_2000092b = cVar13 + '\x02';
            if (uVar10 < 0x1a9) {
              DAT_20000923 = 0;
              DAT_20000924 = 0;
            }
            else {
              DAT_20000923 = 2;
              DAT_20000924 = 0xdd;
            }
            DAT_20000925 = uVar10 >= 0x1a9;
            DAT_20000926 = 0;
            DAT_20000927 = 0;
            DAT_20000928 = 0xff;
            DAT_20000929 = 9;
            DAT_2000092a = 0;
            DAT_2000092c = 0;
            DAT_2000092d = 0;
            DAT_2000090d = DAT_2000090c;
            for (uVar15 = 1; uVar15 <= uVar14; uVar15 = uVar15 + 1 & 0xffff) {
              (&DAT_2000092d)[uVar15] = *(undefined1 *)(uVar15 + 0x2000b9ec);
            }
          }
          if (uVar10 - 0x1a9 < 0xd4) {
            DAT_20000a1b = 0;
            DAT_20000a1c = 1;
            DAT_20000a1d = 0x12;
            DAT_20000a1e = 8;
            DAT_20000a1f = 0x12;
            DAT_20000a20 = 0x88;
            DAT_20000a21 = 0x11;
            uVar14 = uVar10 - 0x1a8 & 0xffff;
            cVar13 = (char)uVar14;
            DAT_20000a0b = cVar13 + '\x1f';
            DAT_20000a0c = DAT_2000080d;
            DAT_20000a1a = cVar13 + '\x06';
            DAT_20000a2a = cVar13 + '\x02';
            DAT_20000a22 = 0;
            DAT_20000a23 = 0;
            DAT_20000a24 = 0;
            DAT_20000a25 = 0;
            DAT_20000a26 = 0;
            DAT_20000a27 = 0xff;
            DAT_20000a28 = 9;
            DAT_20000a29 = 0;
            DAT_20000a2b = 0;
            DAT_20000a2c = 0;
            for (uVar15 = 1; uVar15 <= uVar14; uVar15 = uVar15 + 1 & 0xffff) {
              (&DAT_20000a2c)[uVar15] = *(undefined1 *)(uVar15 + 0x2000bac0);
            }
          }
        }
        if ((5000 < DAT_20000220 - DAT_2000023c) || (2 < uVar10)) {
          DAT_200007e9 = 0;
          DAT_200007ea = 0;
          DAT_200005ed = 0;
        }
      }
      if (DAT_2000016f == '\0') {
        if (DAT_20000156 == '\x01') {
          DAT_20000156 = '\0';
        }
        if ((DAT_20000155 == '\x01') && (DAT_20000155 = '\0', DAT_20000486 == '\x02')) {
          DAT_20000486 = '\0';
          DAT_20000310 = DAT_20000220;
          DAT_200001cf = '\x01';
        }
        if (DAT_20000157 == '\x01') {
          if (DAT_2000016d == '\x01') {
            DAT_2000016d = '\0';
          }
          DAT_20000157 = '\0';
        }
      }
      if ((DAT_2000016e == '\x01') && (DAT_2000016f == '\0')) {
        DAT_2000016e = '\0';

  /* ==========================================================================
   *   8.4 UID Verification, Digital Input Filtering (I0.0..I2.7) & Analog Pot
   *       (SMB28/SMB29) / Temperature / Supply Voltage ADC Scaling
   * ========================================================================== */
        FUN_0803bf24();
      }
      uRam422201b4 = (uint)(DAT_2000016f == '\x01');
      DAT_2000027c = *(undefined4 *)(DAT_20000504 + 0x20004f94);
      DAT_20000280 = DAT_20000220 & 0xfffffff;
      if (DAT_200038be == '\x03') {
        DAT_200038be = '\0';
      }
      if (DAT_200001bb != '\0') {
        DAT_200001bb = '\0';
        if (DAT_200001bd == '\0') {
          DAT_20000186 = 1;
        }
        else {
          DAT_20000186 = 2;
        }
      }
      if (DAT_200001c0 != '\0') {
        DAT_200001c0 = '\0';
        if (DAT_200001be == '\0') {
          DAT_20000187 = 1;
        }
        else {
          DAT_20000187 = 2;
        }
      }
      if (DAT_20000100 != '\0') {
        DAT_20000100 = '\0';
        if (DAT_2000047c == '\0') {
          DAT_20000042 = 1;
        }
        else {
          DAT_20000042 = 2;
        }
      }
      if (0xf0 < DAT_200038a2 - 10) {
        DAT_200038a2 = 0x14;
      }
      DAT_20000210 = DAT_20000220;
      if (DAT_20000220 < DAT_20000214) {
        uVar14 = ~DAT_20000214;
      }
      else {
        uVar14 = -DAT_20000214;
      }
      DAT_20000218 = uVar14 + DAT_20000220;
      DAT_2000015a = DAT_200038a2 <= DAT_20000218;
      if ((bool)DAT_2000015a) {
        DAT_20000214 = DAT_20000220;
      }
      if (((int)((uint)DAT_20005f28 << 0x1a) < 0) && (DAT_20000159 == '\0')) {
        DAT_20000158 = '\x01';
        DAT_20000159 = '\x01';
      }
      else {
        DAT_20000158 = '\0';
        if (-1 < (int)((uint)DAT_20005f28 << 0x1a)) {
          DAT_20000159 = '\0';
        }
      }
      DAT_2000024c = DAT_2000024c + 1;
      DAT_2000022c = DAT_20000220;
      if (DAT_20000228 <= DAT_20000220) {
        DAT_20000230 = DAT_20000220 - DAT_20000228;
      }
      DAT_20005f3e = (undefined1)(DAT_20000230 >> 8);
      DAT_20005f3f = (undefined1)DAT_20000230;
      if ((DAT_20000234 < DAT_20000230) && (DAT_2000017b == '\0')) {
        DAT_20000234 = DAT_20000230;
      }
      DAT_20005f42 = (undefined1)(DAT_20000234 >> 8);
      DAT_20005f43 = (undefined1)DAT_20000234;
      if (DAT_20000230 < DAT_20000238) {
        DAT_20000238 = DAT_20000230;
      }
      DAT_20005f40 = (undefined1)(DAT_20000238 >> 8);
      DAT_20005f41 = (undefined1)DAT_20000238;
      uVar14 = FUN_08014520(&DAT_40011400);
      DAT_200001d9 = DAT_200001d9 & 0xfc | (byte)((uVar14 << 0x13) >> 0x1f) |
                     (byte)(uVar14 >> 0xc) & 2;
      DAT_200001dd = DAT_200001dd & 3 | (byte)(uVar14 >> 6) & 4 | (byte)(uVar14 >> 6) & 8 |
                     (byte)(uVar14 >> 6) & 0x10 | (byte)(uVar14 >> 6) & 0x20 |
                     (byte)(uVar14 >> 9) & 0x40 | (byte)(uVar14 >> 7) & 0x80;
      uVar14 = FUN_08014520(&DAT_40011800);
      DAT_200001d9 = DAT_200001d9 & 0xdb | (byte)(uVar14 << 2) & 4 | (byte)(uVar14 << 4) & 0x20;
      bVar5 = (byte)(uVar14 >> 8);
      DAT_200001db = DAT_200001db & 0xc0 | (byte)((uVar14 << 0x17) >> 0x1f) | bVar5 & 2 | bVar5 & 4
                     | bVar5 & 8 | bVar5 & 0x10 | bVar5 & 0x20;
      DAT_200001dd = DAT_200001dd & 0xfc | (byte)((uVar14 & 0xff) >> 7) | (byte)(uVar14 >> 0xe) & 2;
      uVar14 = FUN_08014520(&DAT_40011000);
      DAT_200001d9 = DAT_200001d9 & 0xe7 | (byte)(uVar14 >> 3) & 8 | (byte)(uVar14 >> 3) & 0x10;
      DAT_200001db = DAT_200001db & 0x3f | (byte)((uVar14 & 4) << 4) | (byte)((uVar14 & 8) << 4);
      bVar5 = FUN_08014520(&DAT_40011000);
      DAT_20000179 = bVar5 & 0x20;
      DAT_200038d1 = DAT_20000179;
      uVar14 = FUN_08014520(&DAT_40010c00);
      DAT_200001d9 = DAT_200001d9 & 0x3f | (byte)((uVar14 & 0x10) << 2) |
                     (byte)((uVar14 & 0x20) << 2);
      iVar8 = FUN_08014520(&DAT_40010c00);
      DAT_200001c3 = (byte)((uint)(iVar8 << 0x17) >> 0x1f);
      FUN_080043b8(10000);
      uVar14 = FUN_08014520(&DAT_40011400);
      DAT_200001d8 = DAT_200001d8 & 0xfc | (byte)((uVar14 << 0x13) >> 0x1f) |
                     (byte)(uVar14 >> 0xc) & 2;
      DAT_200001dc = DAT_200001dc & 3 | (byte)(uVar14 >> 6) & 4 | (byte)(uVar14 >> 6) & 8 |
                     (byte)(uVar14 >> 6) & 0x10 | (byte)(uVar14 >> 6) & 0x20 |
                     (byte)(uVar14 >> 9) & 0x40 | (byte)(uVar14 >> 7) & 0x80;
      uVar14 = FUN_08014520(&DAT_40011800);
      DAT_200001d8 = DAT_200001d8 & 0xdb | (byte)(uVar14 << 2) & 4 | (byte)(uVar14 << 4) & 0x20;
      bVar5 = (byte)(uVar14 >> 8);
      DAT_200001da = DAT_200001da & 0xc0 | (byte)((uVar14 << 0x17) >> 0x1f) | bVar5 & 2 | bVar5 & 4
                     | bVar5 & 8 | bVar5 & 0x10 | bVar5 & 0x20;
      DAT_200001dc = DAT_200001dc & 0xfc | (byte)((uVar14 & 0xff) >> 7) | (byte)(uVar14 >> 0xe) & 2;
      uVar14 = FUN_08014520(&DAT_40011000);
      DAT_200001d8 = DAT_200001d8 & 0xe7 | (byte)(uVar14 >> 3) & 8 | (byte)(uVar14 >> 3) & 0x10;
      DAT_200001da = DAT_200001da & 0x3f | (byte)((uVar14 & 4) << 4) | (byte)((uVar14 & 8) << 4);
      bVar5 = FUN_08014520(&DAT_40011000);
      DAT_20000179 = bVar5 & 0x20;
      DAT_200038d1 = DAT_20000179;
      iVar8 = FUN_08014520(&DAT_40010c00);
      DAT_200001c3 = (byte)((uint)(iVar8 << 0x17) >> 0x1f);
      bVar5 = FUN_08014520(&DAT_40010c00);
      bVar20 = (bVar5 & 0x10) << 2;
      bVar5 = (bVar5 & 0x20) << 2;
      if ((DAT_200001d8 & 1) == (DAT_200001d9 & 1)) {
        DAT_20005bd0 = DAT_20005bd0 & 0xfe | DAT_200001d9 & 1;
      }
      if ((DAT_200001d8 & 2) == (DAT_200001d9 & 2)) {
        DAT_20005bd0 = DAT_20005bd0 & 0xfd | DAT_200001d9 & 2;
      }
      if ((DAT_200001d8 & 4) == (DAT_200001d9 & 4)) {
        DAT_20005bd0 = DAT_20005bd0 & 0xfb | DAT_200001d9 & 4;
      }
      if ((DAT_200001d8 & 8) == (DAT_200001d9 & 8)) {
        DAT_20005bd0 = DAT_20005bd0 & 0xf7 | DAT_200001d9 & 8;
      }
      if ((DAT_200001d8 & 0x10) == (DAT_200001d9 & 0x10)) {
        DAT_20005bd0 = DAT_20005bd0 & 0xef | DAT_200001d9 & 0x10;
      }
      if ((DAT_200001d8 & 0x20) == (DAT_200001d9 & 0x20)) {
        DAT_20005bd0 = DAT_20005bd0 & 0xdf | DAT_200001d9 & 0x20;
      }
      if (bVar20 == (DAT_200001d9 & 0x40)) {
        DAT_20005bd0 = DAT_20005bd0 & 0xbf | DAT_200001d9 & 0x40;
      }
      if (bVar5 == (DAT_200001d9 & 0x80)) {
        DAT_20005bd0 = DAT_20005bd0 & 0x7f | DAT_200001d9 & 0x80;
      }
      if ((DAT_200001da & 1) == (DAT_200001db & 1)) {
        DAT_20005bd1 = DAT_20005bd1 & 0xfe | DAT_200001db & 1;
      }
      if ((DAT_200001da & 2) == (DAT_200001db & 2)) {
        DAT_20005bd1 = DAT_20005bd1 & 0xfd | DAT_200001db & 2;
      }
      if ((DAT_200001da & 4) == (DAT_200001db & 4)) {
        DAT_20005bd1 = DAT_20005bd1 & 0xfb | DAT_200001db & 4;
      }
      if ((DAT_200001da & 8) == (DAT_200001db & 8)) {
        DAT_20005bd1 = DAT_20005bd1 & 0xf7 | DAT_200001db & 8;
      }
      if ((DAT_200001da & 0x10) == (DAT_200001db & 0x10)) {
        DAT_20005bd1 = DAT_20005bd1 & 0xef | DAT_200001db & 0x10;
      }
      if ((DAT_200001da & 0x20) == (DAT_200001db & 0x20)) {
        DAT_20005bd1 = DAT_20005bd1 & 0xdf | DAT_200001db & 0x20;
      }
      if ((DAT_200001da & 0x40) == (DAT_200001db & 0x40)) {
        DAT_20005bd1 = DAT_20005bd1 & 0xbf | DAT_200001db & 0x40;
      }
      if ((DAT_200001da & 0x80) == (DAT_200001db & 0x80)) {
        DAT_20005bd1 = DAT_20005bd1 & 0x7f | DAT_200001db & 0x80;
      }
      if ((DAT_200001dc & 1) == (DAT_200001dd & 1)) {
        DAT_20005bd2 = DAT_20005bd2 & 0xfe | DAT_200001dd & 1;
      }
      if ((DAT_200001dc & 2) == (DAT_200001dd & 2)) {
        DAT_20005bd2 = DAT_20005bd2 & 0xfd | DAT_200001dd & 2;
      }
      if ((DAT_200001dc & 4) == (DAT_200001dd & 4)) {
        DAT_20005bd2 = DAT_20005bd2 & 0xfb | DAT_200001dd & 4;
      }
      if ((DAT_200001dc & 8) == (DAT_200001dd & 8)) {
        DAT_20005bd2 = DAT_20005bd2 & 0xf7 | DAT_200001dd & 8;
      }
      if ((DAT_200001dc & 0x10) == (DAT_200001dd & 0x10)) {
        DAT_20005bd2 = DAT_20005bd2 & 0xef | DAT_200001dd & 0x10;
      }
      if ((DAT_200001dc & 0x20) == (DAT_200001dd & 0x20)) {
        DAT_20005bd2 = DAT_20005bd2 & 0xdf | DAT_200001dd & 0x20;
      }
      if ((DAT_200001dc & 0x40) == (DAT_200001dd & 0x40)) {
        DAT_20005bd2 = DAT_20005bd2 & 0xbf | DAT_200001dd & 0x40;
      }
      if ((DAT_200001dc & 0x80) == (DAT_200001dd & 0x80)) {
        DAT_20005bd2 = DAT_20005bd2 & 0x7f | DAT_200001dd & 0x80;
      }
      DAT_2000512c = DAT_2000512c & 0xffffff00 | DAT_20005bd1 & 1 | (DAT_20005bd1 >> 2 & 1) << 1 |
                     (DAT_20005bd1 >> 1 & 1) << 2 | (DAT_20005bd1 >> 3 & 1) << 3 |
                     (DAT_20005bd1 >> 4 & 1) << 4 | (DAT_20005bd1 >> 5 & 1) << 5 |
                     (DAT_20005bd1 >> 6 & 1) << 6 | (uint)(DAT_20005bd1 >> 7) << 7;
      DAT_20005bd1 = (byte)DAT_2000512c;
      DAT_200001d8 = DAT_200001d8 & 0x3f | bVar20 | bVar5;
      iVar8 = FUN_08014520(&DAT_40011800);
      DAT_200001c4 = (byte)((uint)(iVar8 << 0x19) >> 0x1f);
      uVar14 = 0;
      do {
        iVar8 = 0;
        uVar10 = 0;
        do {
          iVar1 = uVar10 * 7;
          uVar10 = uVar10 + 1 & 0xffff;
          iVar8 = iVar8 + (uint)(ushort)(&DAT_200061e2)[iVar1 + uVar14];
        } while (uVar10 < 0x100);
        uVar31 = FUN_080006e0(iVar8);
        FUN_080005ce((int)uVar31,(int)((ulonglong)uVar31 >> 0x20),0x33333333,0x403d7333);
        uVar6 = FUN_0800072c();
        (&DAT_200061a0)[uVar14] = uVar6;
        uVar14 = uVar14 + 1 & 0xffff;
      } while (uVar14 < 7);
      DAT_2000029c = DAT_200061a0 - 0x28;
      DAT_200002a0 = DAT_200061a2 - 0x28;
      DAT_200002a4 = (uint)DAT_200061aa;
      DAT_200002a8 = DAT_200061ac & 0xffff;
      if (DAT_2000029c < 0x8000) {
        if (DAT_2000029c < 0x32) {
          DAT_2000029c = 0;
        }
      }
      else {
        DAT_2000029c = 0x7fff;
      }
      if (DAT_200002a0 < 0x8000) {
        if (DAT_200002a0 < 0x32) {
          DAT_200002a0 = 0;
        }
      }
      else {
        DAT_200002a0 = 0x7fff;
      }
      if (DAT_200002a4 < 0x8000) {
        if (DAT_200002a4 < 0x32) {
          DAT_200002a4 = 0;
        }
      }
      else {
        DAT_200002a4 = 0x7fff;
      }
      if (DAT_200002a8 < 0x8000) {
        if (DAT_200002a8 < 0x32) {
          DAT_200002a8 = 0;
        }
      }
      else {
        DAT_200002a8 = 0x7fff;
      }
      pbVar24 = &DAT_20005f28;
      DAT_20005f45 = (undefined1)(DAT_200061a4 / 0x86);
      DAT_20005f44 = (undefined1)(DAT_200061a6 / 0x86);
      DAT_200001ea = DAT_200061a8;
      if (DAT_2000016b == '\x01') {
        uVar6 = FUN_08042c44(DAT_200002a4);
        uVar31 = FUN_080006e0(uVar6);
        uVar31 = FUN_080004ea((int)uVar31,(int)((ulonglong)uVar31 >> 0x20),0x47ae147b,0x3f347ae1);
        uVar31 = FUN_080004ea((int)uVar31,(int)((ulonglong)uVar31 >> 0x20),0,0x40c38800);
        FUN_080005ce((int)uVar31,(int)((ulonglong)uVar31 >> 0x20),0x6e978d50,0x3fe08312);
        DAT_2000020c = FUN_0800072c();
        DAT_200001e4 = FUN_08042cc0();
      }
      if (DAT_2000016c == '\x01') {
        uVar6 = FUN_08042c82(DAT_200002a8);
        uVar31 = FUN_080006e0(uVar6);
        uVar31 = FUN_080004ea((int)uVar31,(int)((ulonglong)uVar31 >> 0x20),0x47ae147b,0x3f347ae1);
        uVar31 = FUN_080004ea((int)uVar31,(int)((ulonglong)uVar31 >> 0x20),0,0x40c38800);
        FUN_080005ce((int)uVar31,(int)((ulonglong)uVar31 >> 0x20),0x6e978d50,0x3fe08312);
        DAT_2000020a = FUN_0800072c();
        DAT_200001e6 = FUN_08042cc0();
      }
      bVar28 = DAT_200001a2 == 0;

  /* ==========================================================================
   *   8.5 Expansion Module (EM0..EM6) Bus Enumeration, Config & Input Refresh
   * ========================================================================== */
      do {
        if (bVar28) goto LAB_08047ba4;
        bVar28 = true;
      } while (DAT_20000145 == '\x01');
      DAT_20000294 = DAT_20000220 & 0x7fffff;
      if (((DAT_20000294 - 0x700001 < 0x4ff) && (DAT_20000144 == '\0')) || (DAT_200038cb == '\a')) {
        DAT_40003000 = 0xaaaa;
        DAT_200038cb = '\0';
        DAT_40003000 = 0xaaaa;
      }
      DAT_200001fc = 0;
LAB_080476a8:
      pbVar17 = (byte *)&DAT_20000140;
      uVar14 = (uint)DAT_200001fc;
      if (uVar14 < DAT_200001a2) {
        uVar10 = (uint)(byte)(&DAT_20000356)[uVar14];
        if (uVar10 - 1 < 0xf) {
          for (uVar14 = 0; uVar10 = (uint)DAT_200001fc, uVar14 < (byte)(&DAT_20000372)[uVar10];
              uVar14 = uVar14 + 1 & 0xffff) {
            uVar15 = uVar10 << 5 | 0x1c | uVar14;
            DAT_2000019b = (byte)uVar15;
            uVar10 = (uint)(&DAT_20001046)[(byte)(&DAT_20000364)[uVar10] + uVar14] + uVar15 * 0x100;
            DAT_20000200 = (ushort)uVar10;
            DAT_200001fe = FUN_08012aac(uVar10 & 0xffff,0x19);
          }
        }
        else if (uVar10 - 0x10 < 0x70) {
          for (uVar14 = 0; uVar14 < (byte)(&DAT_20000372)[DAT_200001fc];
              uVar14 = uVar14 + 2 & 0xffff) {
            uVar10 = (DAT_200001fc & 7) << 0xd | 0x1204;
            DAT_20000200 = (ushort)uVar10;
            iVar8 = FUN_08012aac(uVar10,0x19);
            DAT_200001fe = (undefined2)iVar8;
            if ((iVar8 != 1) && (iVar8 != 5)) break;
            uVar10 = (uint)DAT_200001fc << 5 | 0x18 | uVar14;
            DAT_2000019b = (byte)uVar10;
            bVar5 = (&DAT_20005e98)[(byte)(&DAT_20000364)[DAT_200001fc] + uVar14];
            DAT_2000019c = bVar5 & 0x80;
            if ((bVar5 & 0x80) == 0) {
              DAT_2000019d = bVar5 | 0x80;
            }
            else {
              DAT_2000019d = bVar5 & 0x7f;
            }
            iVar8 = (uVar10 & 0xff) * 0x100 + (uint)(DAT_2000019d >> 4);
            DAT_20000200 = (ushort)iVar8;
            DAT_200001fe = FUN_08012aac(iVar8,0x19);
            uVar15 = (uint)DAT_200001fc << 5 | 0x18 | uVar14 + 1;
            DAT_2000019b = (byte)uVar15;
            uVar10 = (uint)((&DAT_20005e98)[uVar14 + 1 + (uint)(byte)(&DAT_20000364)[DAT_200001fc]]
                           >> 4) | (DAT_2000019d & 0xf) << 4;
            DAT_2000019e = (byte)uVar10;
            uVar10 = uVar10 + uVar15 * 0x100;
            DAT_20000200 = (ushort)uVar10;
            DAT_200001fe = FUN_08012aac(uVar10 & 0xffff,0x19);
          }
        }
        else if (uVar10 == 0xa0) {
          uVar14 = (uVar14 & 7) << 5 | 4;
          DAT_2000019b = (byte)uVar14;
          if ((DAT_2000388a & 1) == 0) {
            uVar14 = FUN_08013346(uVar14,0x18);
            DAT_200001fe = (undefined2)uVar14;
            uVar10 = (uVar14 & 0x3ff) >> 2;
            DAT_2000019c = (byte)((uVar14 << 0x16) >> 0x18);
            DAT_2000019d = (byte)(uVar14 & 3);
            if ((uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3)) {
              uVar14 = (uint)DAT_200001fc;
              bVar28 = uVar10 == 0x10;
              pbVar24[uVar14 * 2 + 9] = pbVar24[uVar14 * 0x32 + 0xdd];
              pbVar23 = (byte *)&DAT_200001c0;
LAB_08045b78:
              if (bVar28) {
                uVar14 = (uVar14 & 7) << 0xd | 0x1400;
                *(short *)(pbVar23 + 0x40) = (short)uVar14;
                iVar8 = FUN_08012aac(uVar14,0x19);
                *(short *)(pbVar23 + 0x3e) = (short)iVar8;
                if ((iVar8 == 1) || (iVar8 == 5)) {
                  uVar7 = (ushort)pbVar23[0x3c] << 0xd | 0x1210;
                  *(ushort *)(pbVar23 + 0x40) = uVar7;
                  iVar8 = FUN_08012aac(uVar7,0x19);
                  *(short *)(pbVar23 + 0x3e) = (short)iVar8;
                  pbVar22 = pbVar23;
                  if (iVar8 == 1) goto LAB_08045bba;
                  bVar28 = iVar8 == 5;
LAB_08045bb8:
                  pbVar22 = pbVar23;
                  if (bVar28) {
LAB_08045bba:
                    bVar5 = pbVar22[0x3c] << 5 | 8;
                    pbVar17[0x5b] = bVar5;
                    uVar14 = FUN_08013346(bVar5,0x18);
                    *(short *)(pbVar22 + 0x3e) = (short)uVar14;
                    uVar10 = (uVar14 & 0x3ff) >> 2;
                    pbVar17[0x5d] = (byte)((uVar14 << 0x16) >> 0x18);
                    pbVar17[0x5e] = (byte)(uVar14 & 3);
                    bVar28 = (uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3);
LAB_08045bec:
                    if (bVar28) {
                      pbVar17[100] = (byte)uVar10;
                      uVar7 = (ushort)pbVar22[0x3c] << 0xd | 0x1210;
                      *(ushort *)(pbVar22 + 0x40) = uVar7;
                      iVar8 = FUN_08012aac(uVar7,0x19);
                      *(short *)(pbVar22 + 0x3e) = (short)iVar8;
                      if (iVar8 == 1) goto LAB_08045c12;
                      bVar28 = iVar8 == 5;
LAB_08045c10:
                      if (bVar28) {
LAB_08045c12:
                        pbVar22 = (byte *)&DAT_200001c0;
                        DAT_2000019b = (char)DAT_200001fc << 5 | 9;
                        uVar10 = FUN_08013346(DAT_2000019b,0x18);
                        DAT_200001fe = (undefined2)uVar10;
                        DAT_2000019d = (byte)((uVar10 << 0x16) >> 0x18);
                        uVar14 = uVar10 & 3;
                        DAT_2000019e = (byte)uVar14;
                        pbVar17 = &DAT_0804a2a2;
                        bVar30 = (byte)(&DAT_0804a2a2)[(uVar10 & 0x3ff) >> 2] == uVar14;
LAB_08045c54:
                        bVar28 = false;
                        pbVar23 = pbVar22;
                        if (bVar30) {
                          bVar5 = pbVar22[0x3c] << 5 | 10;
                          pbVar23 = pbVar22 + -0x25;
                          *pbVar23 = bVar5;
                          uVar14 = FUN_08013346(bVar5,0x18);
                          DAT_200001fe = (undefined2)uVar14;
                          pbVar22[-0x23] = (byte)((uVar14 << 0x16) >> 0x18);
                          pbVar22[-0x22] = (byte)(uVar14 & 3);
                          bVar30 = (uint)pbVar17[(uVar14 & 0x3ff) >> 2] == (uVar14 & 3);
LAB_08045c8e:
                          bVar28 = false;
                          if (bVar30) {
                            pbVar17 = (byte *)&DAT_200001c0;
                            DAT_2000019b = (char)DAT_200001fc << 5 | 0xb;
                            pbVar22 = &DAT_2000019b;
                            uVar14 = FUN_08013346(DAT_2000019b,0x18);
                            DAT_200001fe = (undefined2)uVar14;
                            DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                            DAT_2000019e = (byte)(uVar14 & 3);
                            uVar10 = (uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2];
                            bVar30 = uVar10 == (uVar14 & 3);
LAB_08045cce:
                            bVar28 = false;
                            if (bVar30) {
                              uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1210;
                              *(ushort *)(pbVar17 + 0x40) = uVar7;
                              iVar8 = FUN_08012aac(uVar7,0x19);
                              *(short *)(pbVar17 + 0x3e) = (short)iVar8;
                              if (iVar8 == 1) goto LAB_08045cf2;
                              bVar30 = iVar8 == 5;
LAB_08045cf0:
                              bVar28 = false;
                              if (bVar30) {
LAB_08045cf2:
                                DAT_2000019b = pbVar17[0x3c] << 5 | 0xc;
                                uVar10 = FUN_08013346(DAT_2000019b,0x18);
                                *(short *)(pbVar17 + 0x3e) = (short)uVar10;
                                DAT_2000019d = (byte)((uVar10 << 0x16) >> 0x18);
                                uVar14 = uVar10 & 3;
                                DAT_2000019e = (byte)uVar14;
                                pbVar22 = (byte *)&DAT_20000140;
                                bVar30 = false;
                                if ((byte)(&DAT_0804a2a2)[(uVar10 & 0x3ff) >> 2] == uVar14) {
                                  DAT_2000019b = pbVar17[0x3c] << 5 | 0xd;
                                  uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                  *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                  DAT_2000019e = (byte)(uVar14 & 3);
                                  pbVar23 = (byte *)&DAT_20000140;
                                  bVar28 = (uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] ==
                                           (uVar14 & 3);
LAB_08045d64:
                                  bVar30 = false;
                                  if (bVar28) {
                                    bVar5 = pbVar17[0x3c] << 5 | 0xe;
                                    pbVar23[0x5b] = bVar5;
                                    uVar14 = FUN_08013346(bVar5,0x18);
                                    *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                    pbVar23[0x5d] = (byte)((uVar14 << 0x16) >> 0x18);
                                    pbVar23[0x5e] = (byte)(uVar14 & 3);
                                    uVar10 = (uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2];
                                    bVar28 = uVar10 == (uVar14 & 3);
                                    pbVar22 = pbVar23;
                                    while (bVar30 = false, bVar28) {
                                      pbVar22[0x66] = pbVar22[100] & 1;
                                      pbVar17 = (byte *)&DAT_200001c0;
                                      bVar5 = (char)DAT_200001fc << 5 | 0xf;
                                      pbVar22[0x5b] = bVar5;
                                      uVar14 = FUN_08013346(bVar5,0x18);
                                      DAT_200001fe = (undefined2)uVar14;
                                      bVar5 = (byte)((uVar14 << 0x16) >> 0x18);
                                      pbVar22[0x5d] = bVar5;
                                      pbVar22[0x5e] = (byte)(uVar14 & 3);
                                      bVar30 = (uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] ==
                                               (uVar14 & 3);
                                      if (!bVar30) goto LAB_08045cf0;
                                      if ((pbVar22[100] - 0xb < 0x40) && (pbVar22[0x66] == 0)) {
                                        pbVar24[(uint)DAT_200001fc * 0x32 + 0xe2] = bVar5;
LAB_08045e1a:
                                        pbVar22[0x6d] = bVar5;
                                      }
                                      else if (pbVar22[100] == 10) goto LAB_08045e1a;
                                      pbVar23 = (byte *)&DAT_200001fc;
                                      DAT_20000200 = DAT_200001fc << 0xd | 0x1211;
                                      iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                      DAT_200001fe = (undefined2)iVar8;
                                      if (iVar8 == 1) goto LAB_08045e46;
                                      bVar28 = iVar8 == 5;
                                      pbVar17 = pbVar22;
LAB_08045e44:
                                      pbVar22 = pbVar17;
                                      if (!bVar28) goto LAB_08045d64;
LAB_08045e46:
                                      pbVar23 = pbVar22;
                                      pbVar17 = (byte *)&DAT_200001c0;
                                      bVar5 = (char)DAT_200001fc << 5 | 8;
                                      pbVar23[0x5b] = bVar5;
                                      uVar14 = FUN_08013346(bVar5,0x18);
                                      DAT_200001fe = (undefined2)uVar14;
                                      uVar10 = (uVar14 & 0x3ff) >> 2;
                                      pbVar23[0x5d] = (byte)((uVar14 << 0x16) >> 0x18);
                                      pbVar23[0x5e] = (byte)(uVar14 & 3);
                                      bVar30 = (uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3);
                                      while (bVar28 = false, pbVar22 = pbVar23, bVar30) {
                                        bVar5 = (byte)uVar10;
                                        pbVar23[0x65] = bVar5;
                                        if ((pbVar23[100] - 0xb < 0x40) && (pbVar23[0x66] == 0)) {
                                          uVar7 = *(ushort *)(pbVar17 + 0x3c);
                                          pbVar24[(uint)uVar7 * 0x32 + 0xe3] = bVar5;
                                          pbVar23[0x6e] = bVar5;
                                          bVar5 = pbVar24[(uint)uVar7 * 0x32 + 0xe2];
LAB_08045ec6:
                                          *(ushort *)(pbVar17 + 0x42) =
                                               (short)uVar10 + (ushort)bVar5 * 0x100;
                                        }
                                        else if (pbVar23[100] == 10) {
                                          pbVar23[0x6e] = bVar5;
                                          bVar5 = pbVar23[0x6d];
                                          goto LAB_08045ec6;
                                        }
                                        bVar5 = pbVar17[0x3c] << 5 | 9;
                                        pbVar23[0x5b] = bVar5;
                                        uVar14 = FUN_08013346(bVar5,0x18);
                                        *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                        uVar10 = (uVar14 & 0x3ff) >> 2;
                                        pbVar23[0x5d] = (byte)((uVar14 << 0x16) >> 0x18);
                                        pbVar23[0x5e] = (byte)(uVar14 & 3);
                                        bVar28 = (uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3)
                                        ;
                                        pbVar22 = pbVar23 + 0x5b;
LAB_08045f00:
                                        pbVar23 = pbVar22;
                                        if (!bVar28) goto LAB_08045e44;
                                        uVar14 = (uint)DAT_200001a4;
                                        bVar5 = (byte)uVar10;
                                        if ((uVar14 - 0xb < 0x40) && (DAT_200001a6 == '\0')) {
                                          pbVar23 = (byte *)((uint)*(ushort *)(pbVar17 + 0x3c) *
                                                            0x19);
                                          pbVar24[(uint)*(ushort *)(pbVar17 + 0x3c) * 0x32 + 0xe4] =
                                               bVar5;
                                          uVar32 = CONCAT44(uVar10 + 10,uVar14);
                                          bVar28 = uVar10 + 10 == uVar14;
                                          DAT_200001ab = bVar5;
                                          goto LAB_08045f4e;
                                        }
                                        if (uVar14 == 10) {
                                          bVar30 = false;
                                          DAT_200001ab = bVar5;
                                          if (uVar10 == 0) goto LAB_08045f70;
                                        }
                                        else {
                                          do {
                                            if (uVar14 == 0x13) {
                                              DAT_2000019b = pbVar17[0x3c] << 5 | 10;
                                              uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                              *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                              uVar10 = (uVar14 & 0x3ff) >> 2;
                                              DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                              DAT_2000019e = (byte)(uVar14 & 3);
                                              bVar28 = (uint)(byte)(&DAT_0804a2a2)[uVar10] ==
                                                       (uVar14 & 3);
                                              pbVar23 = pbVar24;
                                              goto LAB_08045fa8;
                                            }
                                            if (uVar14 != 10) {
                                              if ((DAT_200001a6 == '\0') && (10 < uVar14)) {
                                                DAT_2000019b = pbVar17[0x3c] << 5 | 10;
                                                uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                                *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                                uVar10 = (uVar14 & 0x3ff) >> 2;
                                                DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                                uVar14 = uVar14 & 3;
                                                DAT_2000019e = (byte)uVar14;
                                                pbVar23 = (byte *)&DAT_20000140;
                                                bVar28 = (byte)(&DAT_0804a2a2)[uVar10] == uVar14;
LAB_08046398:
                                                if (!bVar28) goto LAB_080462fa;
                                                uVar14 = (uint)*(ushort *)(pbVar17 + 0x3c);
                                                DAT_200001ac = (byte)uVar10;
                                                pbVar24[uVar14 * 0x32 + 0xe5] = DAT_200001ac;
                                                if (pbVar24[uVar14 * 0x32 + 0xe4] != 2) {
                                                  DAT_200001a9 = 0x11;
                                                  DAT_200001aa = 2;
                                                  DAT_200001a7 = pbVar24[uVar14 * 0x32 + 0xe4] >> 2;
                                                  for (DAT_200001a8 = 0; DAT_200001a8 < DAT_200001a7
                                                      ; DAT_200001a8 = DAT_200001a8 + 1) {
                                                    DAT_200001aa = DAT_200001aa + 1 & 7;
                                                    DAT_2000019b = pbVar17[0x3c] << 5 | 8 |
                                                                   DAT_200001aa;
                                                    uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                                    *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                                    DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                                    DAT_2000019e = (byte)(uVar14 & 3);
                                                    if ((uint)(byte)(&DAT_0804a2a2)
                                                                    [(uVar14 & 0x3ff) >> 2] ==
                                                        (uVar14 & 3)) {
                                                      (&DAT_2000108c)
                                                      [(uint)*(ushort *)(pbVar17 + 0x42) +
                                                       (uint)DAT_200001a8 * 4] = DAT_2000019d;
                                                      if (6 < DAT_200001aa) {
                                                        DAT_200001a9 = DAT_200001a9 + 1;
                                                      }
                                                      uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1200
                                                              | (ushort)DAT_200001a9;
                                                      *(ushort *)(pbVar17 + 0x40) = uVar7;
                                                      iVar8 = FUN_08012aac(uVar7,0x19);
                                                      *(short *)(pbVar17 + 0x3e) = (short)iVar8;
                                                      if ((iVar8 == 1) || (iVar8 == 5)) {
                                                        DAT_200001aa = DAT_200001aa + 1 & 7;
                                                        DAT_2000019b = pbVar17[0x3c] << 5 | 8 |
                                                                       DAT_200001aa;
                                                        uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                                        *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                                        DAT_2000019d = (byte)((uVar14 << 0x16) >>
                                                                             0x18);
                                                        DAT_2000019e = (byte)(uVar14 & 3);
                                                        if ((uint)(byte)(&DAT_0804a2a2)
                                                                        [(uVar14 & 0x3ff) >> 2] ==
                                                            (uVar14 & 3)) {
                                                          (&DAT_2000108d)
                                                          [(uint)*(ushort *)(pbVar17 + 0x42) +
                                                           (uint)DAT_200001a8 * 4] = DAT_2000019d;
                                                          DAT_200001aa = DAT_200001aa + 1 & 7;
                                                          DAT_2000019b = pbVar17[0x3c] << 5 | 8 |
                                                                         DAT_200001aa;
                                                          uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                                          *(short *)(pbVar17 + 0x3e) = (short)uVar14
                                                          ;
                                                          DAT_2000019d = (byte)((uVar14 << 0x16) >>
                                                                               0x18);
                                                          DAT_2000019e = (byte)(uVar14 & 3);
                                                          if ((uint)(byte)(&DAT_0804a2a2)
                                                                          [(uVar14 & 0x3ff) >> 2] ==
                                                              (uVar14 & 3)) {
                                                            (&DAT_2000108e)
                                                            [(uint)*(ushort *)(pbVar17 + 0x42) +
                                                             (uint)DAT_200001a8 * 4] = DAT_2000019d;
                                                            DAT_200001aa = DAT_200001aa + 1 & 7;
                                                            DAT_2000019b = pbVar17[0x3c] << 5 | 8 |
                                                                           DAT_200001aa;
                                                            uVar14 = FUN_08013346(DAT_2000019b,0x18)
                                                            ;
                                                            *(short *)(pbVar17 + 0x3e) =
                                                                 (short)uVar14;
                                                            DAT_2000019d = (byte)((uVar14 << 0x16)
                                                                                 >> 0x18);
                                                            DAT_2000019e = (byte)(uVar14 & 3);
                                                            if ((uint)(byte)(&DAT_0804a2a2)
                                                                            [(uVar14 & 0x3ff) >> 2]
                                                                == (uVar14 & 3)) {
                                                              (&DAT_2000108f)
                                                              [(uint)*(ushort *)(pbVar17 + 0x42) +
                                                               (uint)DAT_200001a8 * 4] =
                                                                   DAT_2000019d;
                                                            }
                                                          }
                                                        }
                                                      }
                                                    }
                                                  }
                                                  goto LAB_0804665e;
                                                }
                                                uVar14 = (uVar14 & 7) << 5 | 0xb;
                                                DAT_2000019b = (byte)uVar14;
                                                uVar14 = FUN_08013346(uVar14,0x18);
                                                *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                                uVar10 = (uVar14 & 0x3ff) >> 2;
                                                DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                                uVar32 = CONCAT44(uVar10,uVar14) &
                                                         0xffffffff00000003;
                                                DAT_2000019e = (byte)(uVar14 & 3);
                                                bVar28 = false;
                                                if ((uint)(byte)(&DAT_0804a2a2)[uVar10] ==
                                                    (uVar14 & 3)) {
                                                  (&DAT_2000108c)[*(ushort *)(pbVar17 + 0x42)] =
                                                       DAT_2000019d;
                                                  uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1211;
                                                  *(ushort *)(pbVar17 + 0x40) = uVar7;
                                                  uVar32 = FUN_08012aac(uVar7,0x19);
                                                  *(short *)(pbVar17 + 0x3e) = (short)uVar32;
                                                  if (((int)uVar32 == 1) ||
                                                     (bVar28 = false, (int)uVar32 == 5))
                                                  goto LAB_08046438;
                                                }
                                                goto LAB_0804634e;
                                              }
                                              goto LAB_0804665e;
                                            }
LAB_08045f70:
                                            DAT_2000019b = pbVar17[0x3c] << 5 | 10;
                                            uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                            *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                            uVar10 = (uVar14 & 0x3ff) >> 2;
                                            DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                            uVar32 = CONCAT44(uVar10,uVar14) & 0xffffffff00000003;
                                            DAT_2000019e = (byte)(uVar14 & 3);
                                            bVar28 = (uint)(byte)(&DAT_0804a2a2)[uVar10] ==
                                                     (uVar14 & 3);
LAB_0804634e:
                                            pbVar22 = (byte *)&DAT_20000140;
                                            bVar30 = false;
                                            pbVar23 = pbVar24;
                                            if (bVar28) goto code_r0x08046350;
LAB_080462a0:
                                            pbVar24 = pbVar23;
                                            if (!bVar30) goto LAB_080461b8;
                                            uVar7 = *(ushort *)(pbVar17 + 0x3c);
                                            pbVar23[(uint)uVar7 * 0x32 + 0xe4] =
                                                 (byte)(uVar32 >> 0x20);
                                            uVar14 = (uVar7 & 7) << 5 | 0xb;
                                            DAT_2000019b = (byte)uVar14;
                                            uVar14 = FUN_08013346(uVar14,0x18);
                                            *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                            uVar10 = (uVar14 & 0x3ff) >> 2;
                                            DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                            uVar14 = uVar14 & 3;
                                            DAT_2000019e = (byte)uVar14;
                                            bVar28 = (byte)(&DAT_0804a2a2)[uVar10] == uVar14;
LAB_080462fa:
                                            bVar29 = false;
                                            pbVar22 = pbVar17;
                                            if (bVar28) {
                                              pbVar23[(uint)*(ushort *)(pbVar17 + 0x3c) * 0x32 +
                                                      0xe5] = (byte)uVar10;
                                              goto LAB_0804665e;
                                            }
LAB_0804623c:
                                            bVar30 = false;
                                            pbVar17 = pbVar22;
                                            if (bVar29) goto code_r0x0804623e;
LAB_08046186:
                                            uVar32 = CONCAT44(uVar10,uVar14);
                                            if (!bVar30) goto LAB_080460d4;
                                            uVar7 = *(ushort *)(pbVar23 + 0x3c);
                                            pbVar24[(uint)uVar7 * 0x32 + 0xe1] = (byte)uVar10;
                                            uVar14 = (uVar7 & 7) << 0xd | 0x1212;
                                            *(short *)(pbVar23 + 0x40) = (short)uVar14;
                                            uVar32 = FUN_08012aac(uVar14,0x19);
                                            *(short *)(pbVar23 + 0x3e) = (short)uVar32;
                                            pbVar17 = pbVar23;
                                            if ((int)uVar32 != 1) {
                                              bVar30 = (int)uVar32 == 5;
                                              pbVar22 = pbVar23;
LAB_080461b8:
                                              pbVar23 = pbVar22;
                                              if (!bVar30) goto LAB_080460d4;
                                            }
                                            DAT_2000019b = pbVar17[0x3c] << 5 | 8;
                                            pbVar23 = &DAT_2000019b;
                                            uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                            *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                            uVar10 = (uVar14 & 0x3ff) >> 2;
                                            DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                            uVar14 = uVar14 & 3;
                                            DAT_2000019e = (byte)uVar14;
                                            bVar28 = false;
                                            pbVar22 = (byte *)&DAT_20000140;
                                            if ((byte)(&DAT_0804a2a2)[uVar10] == uVar14)
                                            goto code_r0x080461f6;
LAB_0804612e:
                                            pbVar23 = pbVar17;
                                            uVar32 = CONCAT44(uVar10,uVar14);
                                            bVar30 = false;
                                            pbVar17 = pbVar23;
                                            if (bVar28) goto code_r0x08046130;
                                            while (pbVar23 = pbVar22, bVar28 = false, bVar30) {
                                              do {
                                                pbVar23 = pbVar24;
                                                DAT_2000019b = pbVar17[0x3c] << 5 | 0xc;
                                                uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                                *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                                uVar10 = (uVar14 & 0x3ff) >> 2;
                                                DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                                DAT_2000019e = (byte)(uVar14 & 3);
                                                bVar28 = (uint)(byte)(&DAT_0804a2a2)[uVar10] ==
                                                         (uVar14 & 3);
                                                if (bVar28) {
                                                  uVar7 = *(ushort *)(pbVar17 + 0x3c);
                                                  pbVar23[(uint)uVar7 * 0x32 + 0xde] = DAT_2000019d;
                                                  uVar14 = (uVar7 & 7) << 5 | 0xd;
                                                  DAT_2000019b = (byte)uVar14;
                                                  uVar14 = FUN_08013346(uVar14,0x18);
                                                  *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                                  uVar10 = (uVar14 & 0x3ff) >> 2;
                                                  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                                  uVar32 = CONCAT44(uVar10,uVar14) &
                                                           0xffffffff00000003;
                                                  DAT_2000019e = (byte)(uVar14 & 3);
                                                  bVar30 = (uint)(byte)(&DAT_0804a2a2)[uVar10] ==
                                                           (uVar14 & 3);
                                                  pbVar24 = pbVar23;
LAB_080460d4:
                                                  bVar28 = false;
                                                  if (bVar30) {
                                                    uVar7 = *(ushort *)(pbVar17 + 0x3c);
                                                    pbVar23[(uint)uVar7 * 0x32 + 0xdf] =
                                                         (byte)(uVar32 >> 0x20);
                                                    uVar14 = (uVar7 & 7) << 5 | 0xe;
                                                    DAT_2000019b = (byte)uVar14;
                                                    uVar14 = FUN_08013346(uVar14,0x18);
                                                    *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                                    uVar10 = (uVar14 & 0x3ff) >> 2;
                                                    DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                                    uVar14 = uVar14 & 3;
                                                    DAT_2000019e = (byte)uVar14;
                                                    bVar28 = (byte)(&DAT_0804a2a2)[uVar10] == uVar14
                                                    ;
                                                    pbVar22 = pbVar23;
                                                    pbVar24 = pbVar23;
                                                    goto LAB_0804612e;
                                                  }
                                                }
                                                else {
LAB_08045fa8:
                                                  pbVar22 = &DAT_2000019b;
                                                  pbVar24 = pbVar23;
                                                  if (!bVar28) goto LAB_08045f00;
                                                  uVar7 = *(ushort *)(pbVar17 + 0x3c);
                                                  pbVar23[(uint)uVar7 * 0x32 + 0xdc] = (byte)uVar10;
                                                  uVar14 = (uVar7 & 7) << 5 | 0xb;
                                                  DAT_2000019b = (byte)uVar14;
                                                  uVar14 = FUN_08013346(uVar14,0x18);
                                                  *(short *)(pbVar17 + 0x3e) = (short)uVar14;
                                                  uVar10 = (uVar14 & 0x3ff) >> 2;
                                                  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                                  uVar32 = CONCAT44(uVar10,uVar14) &
                                                           0xffffffff00000003;
                                                  DAT_2000019e = (byte)(uVar14 & 3);
                                                  bVar28 = (uint)(byte)(&DAT_0804a2a2)[uVar10] ==
                                                           (uVar14 & 3);
                                                }
                                                if (!bVar28) goto LAB_08045f4e;
                                                uVar7 = *(ushort *)(pbVar17 + 0x3c);
                                                pbVar23[(uint)uVar7 * 0x32 + 0xdd] =
                                                     (byte)(uVar32 >> 0x20);
                                                uVar14 = (uVar7 & 7) << 0xd | 0x1211;
                                                *(short *)(pbVar17 + 0x40) = (short)uVar14;
                                                uVar32 = FUN_08012aac(uVar14,0x19);
                                                *(short *)(pbVar17 + 0x3e) = (short)uVar32;
                                                pbVar24 = pbVar23;
                                              } while ((int)uVar32 == 1);
                                              bVar30 = (int)uVar32 == 5;
                                              pbVar22 = pbVar23;
                                            }
LAB_08045f4e:
                                            uVar10 = (uint)(uVar32 >> 0x20);
                                            uVar14 = (uint)uVar32;
                                            bVar30 = false;
                                          } while (bVar28);
                                        }
                                      }
                                    }
                                    goto LAB_08045cce;
                                  }
                                  goto LAB_08045c8e;
                                }
                                goto LAB_08045c54;
                              }
                              goto LAB_08045c10;
                            }
                            goto LAB_08045bec;
                          }
                          goto LAB_08045bb8;
                        }
                        goto LAB_08045b78;
                      }
                    }
                  }
                }
              }
            }
          }
          else {
            uVar14 = FUN_08013346(uVar14,0x18);
            DAT_200001fe = (undefined2)uVar14;
            uVar10 = (uVar14 & 0x3ff) >> 2;
            DAT_2000019c = (byte)((uVar14 << 0x16) >> 0x18);
            DAT_2000019d = (byte)(uVar14 & 3);
            if ((uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3)) {
              uVar14 = (uint)DAT_200001fc;
              pbVar24[uVar14 * 2 + 9] = pbVar24[uVar14 * 0x32 + 0xdd];
              if (uVar10 == 0x10) {
                uVar14 = (uVar14 & 7) << 0xd | 0x1400;
                DAT_20000200 = (ushort)uVar14;
                iVar8 = FUN_08012aac(uVar14,0x19);
                DAT_200001fe = (undefined2)iVar8;
                if ((iVar8 == 1) || (iVar8 == 5)) {
                  DAT_20000200 = DAT_200001fc << 0xd | 0x1204;
                  iVar8 = FUN_08012aac(DAT_20000200,0x19);
                  DAT_200001fe = (undefined2)iVar8;
                  if ((iVar8 == 1) || (iVar8 == 5)) {
                    DAT_2000019b = (char)DAT_200001fc << 5 | 8;
                    uVar14 = FUN_08013346(DAT_2000019b,0x18);
                    DAT_200001fe = (undefined2)uVar14;
                    uVar10 = (uVar14 & 0x3ff) >> 2;
                    DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                    DAT_2000019c = (byte)(uVar14 & 3);
                    bVar28 = (uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3);
                    puVar18 = &DAT_20000140;
                    do {
                      puVar16 = puVar18;
                      if (!bVar28) goto LAB_0804769a;
                      puVar16[0x60] = (char)uVar10;
                      puVar16[10] = 0;
                      while ((byte)(uintptr_t)puVar16[10] < (byte)(uintptr_t)puVar16[0x60]) {
                        DAT_200051cf = puVar16[0x60] + 4;
                        bVar5 = (char)DAT_200001fc << 5 | 8;
                        puVar16[0x5b] = bVar5;
                        uVar14 = FUN_08013346(bVar5,0x18);
                        DAT_200001fe = (undefined2)uVar14;
                        uVar12 = (undefined1)((uVar14 << 0x16) >> 0x18);
                        puVar16[0x5d] = uVar12;
                        puVar16[0x5e] = (char)(uVar14 & 3);
                        if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] == (uVar14 & 3)) {
                          (&DAT_200051d0)[(byte)(uintptr_t)puVar16[10]] = uVar12;
                        }
                        puVar16[10] = puVar16[10] + '\x01';
                      }
                      DAT_200051cc = 3;
                      DAT_200051cd = 0;
                      DAT_200051ce = 0;
                      DAT_200051cf = puVar16[0x60] + 4;
                      DAT_200051d1 = 0xf0;
                      DAT_200051d2 = 0x80;
                      DAT_200001d2 = (ushort)DAT_200051cf;

  /* ==========================================================================
   *   8.6 Primary PPI / MPI / S7 Protocol Servicing (FUN_080047ac) & Online Edit
   * ========================================================================== */
                      uVar32 = FUN_080047ac();
                      uVar10 = (uint)(uVar32 >> 0x20);
                      bVar28 = (uVar32 & 0xff) == 0;
                      puVar16[0x61] = (char)uVar32;
                      puVar18 = &DAT_200051cc;
                    } while (!bVar28);
                    DAT_200052d2 = DAT_200052d0;
                    DAT_200052d1 = DAT_200052cf + -6;
                    puVar16[10] = 0;
                    for (; (int)(uint)DAT_2000014a < (int)(DAT_200001d4 - 4);
                        DAT_2000014a = DAT_2000014a + 1) {
                      DAT_20000200 = DAT_200001fc << 0xd | (ushort)DAT_200001aa << 8 |
                                     (byte)(&DAT_200052d1)[DAT_2000014a] | 0x1800;
                      DAT_200001fe = FUN_08012aac(DAT_20000200,0x19);
                    }
                  }
                }
              }
            }
          }
        }
        else if (uVar10 == 0xa1) {
          uVar14 = (uVar14 & 7) << 5 | 4;
          DAT_2000019b = (byte)uVar14;
          uVar14 = FUN_08013346(uVar14,0x18);
          DAT_200001fe = (undefined2)uVar14;
          uVar10 = (uVar14 & 0x3ff) >> 2;
          DAT_2000019c = (byte)((uVar14 << 0x16) >> 0x18);
          DAT_2000019d = (byte)(uVar14 & 3);
          if ((uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3)) {
            uVar14 = (uint)DAT_200001fc;
            pbVar24[uVar14 * 2 + 9] = pbVar24[uVar14 * 0x32 + 0xdd];
            if (uVar10 == 0x10) {
              uVar14 = (uVar14 & 7) << 0xd | 0x1400;
              DAT_20000200 = (ushort)uVar14;
              iVar8 = FUN_08012aac(uVar14,0x19);
              DAT_200001fe = (undefined2)iVar8;
              if ((iVar8 == 1) || (iVar8 == 5)) {
                DAT_20000200 = DAT_200001fc << 0xd | 0x1210;
                iVar8 = FUN_08012aac(DAT_20000200,0x19);
                DAT_200001fe = (undefined2)iVar8;
                if ((iVar8 == 1) || (iVar8 == 5)) {
                  DAT_2000019b = (char)DAT_200001fc << 5 | 8;
                  uVar14 = FUN_08013346(DAT_2000019b,0x18);
                  DAT_200001fe = (undefined2)uVar14;
                  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                  DAT_2000019e = (byte)(uVar14 & 3);
                  if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] == (uVar14 & 3)) {
                    DAT_20000200 = DAT_200001fc << 0xd | 0x1210;
                    DAT_200001b8 = DAT_2000019d;
                    iVar8 = FUN_08012aac(DAT_20000200,0x19);
                    DAT_200001fe = (undefined2)iVar8;
                    if ((iVar8 == 1) || (iVar8 == 5)) {
                      DAT_2000019b = (char)DAT_200001fc << 5 | 9;
                      uVar14 = FUN_08013346(DAT_2000019b,0x18);
                      DAT_200001fe = (undefined2)uVar14;
                      DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                      DAT_2000019e = (byte)(uVar14 & 3);
                      if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] == (uVar14 & 3)) {
                        DAT_2000019b = (char)DAT_200001fc << 5 | 10;
                        uVar14 = FUN_08013346(DAT_2000019b,0x18);
                        DAT_200001fe = (undefined2)uVar14;
                        DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                        DAT_2000019e = (byte)(uVar14 & 3);
                        if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] == (uVar14 & 3)) {
                          DAT_2000019b = (char)DAT_200001fc << 5 | 0xb;
                          uVar14 = FUN_08013346(DAT_2000019b,0x18);
                          DAT_200001fe = (undefined2)uVar14;
                          DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                          DAT_2000019e = (byte)(uVar14 & 3);
                          if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] == (uVar14 & 3)) {
                            DAT_20000200 = DAT_200001fc << 0xd | 0x1210;
                            iVar8 = FUN_08012aac(DAT_20000200,0x19);
                            DAT_200001fe = (undefined2)iVar8;
                            if ((iVar8 == 1) || (iVar8 == 5)) {
                              DAT_2000019b = (char)DAT_200001fc << 5 | 0xc;
                              uVar14 = FUN_08013346(DAT_2000019b,0x18);
                              DAT_200001fe = (undefined2)uVar14;
                              DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                              DAT_2000019e = (byte)(uVar14 & 3);
                              if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] == (uVar14 & 3)
                                 ) {
                                DAT_2000019b = (char)DAT_200001fc << 5 | 0xd;
                                DAT_200001b1 = DAT_2000019d;
                                uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                DAT_200001fe = (undefined2)uVar14;
                                DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                DAT_2000019e = (byte)(uVar14 & 3);
                                if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] ==
                                    (uVar14 & 3)) {
                                  DAT_2000019b = (char)DAT_200001fc << 5 | 0xe;
                                  DAT_200001b5 = DAT_2000019d;
                                  uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                  DAT_200001fe = (undefined2)uVar14;
                                  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                  DAT_2000019e = (byte)(uVar14 & 3);
                                  if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] ==
                                      (uVar14 & 3)) {
                                    DAT_2000019b = (char)DAT_200001fc << 5 | 0xf;
                                    DAT_200001b6 = DAT_2000019d;
                                    uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                    DAT_200001fe = (undefined2)uVar14;
                                    DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                    DAT_2000019e = (byte)(uVar14 & 3);
                                    if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] ==
                                        (uVar14 & 3)) {
                                      DAT_20000200 = DAT_200001fc << 0xd | 0x1211;
                                      DAT_200001b7 = DAT_2000019d;
                                      iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                      DAT_200001fe = (undefined2)iVar8;
                                      if ((iVar8 == 1) || (iVar8 == 5)) {
                                        DAT_2000019b = (char)DAT_200001fc << 5 | 8;
                                        uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                        DAT_200001fe = (undefined2)uVar14;
                                        uVar10 = (uVar14 & 0x3ff) >> 2;
                                        DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                        DAT_2000019e = (byte)(uVar14 & 3);
                                        if ((uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3)) {
                                          DAT_20000204 = (ushort)DAT_2000019d +
                                                         (ushort)DAT_200001b7 * 0x100;
                                          if ((uVar10 == 0x66) && (DAT_200001b5 == 0x1f)) {
                                            DAT_200001b4 = 1;
                                            bVar5 = DAT_200001b4;
                                          }
                                          else {
                                            bVar5 = DAT_200001b1;
                                            if ((DAT_200001b1 != 2) &&
                                               ((DAT_200001b5 == 8 ||
                                                ((bVar5 = DAT_200001b4, DAT_200001b5 == 4 &&
                                                 (DAT_200001b1 == 1)))))) {
                                              bVar5 = 3;
                                            }
                                          }
                                          DAT_200001b4 = bVar5;
                                          DAT_2000019b = (char)DAT_200001fc << 5 | 9;
                                          uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                          DAT_200001fe = (undefined2)uVar14;
                                          DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                          DAT_2000019e = (byte)(uVar14 & 3);
                                          if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] ==
                                              (uVar14 & 3)) {
                                            DAT_200001b9 = DAT_2000019d;
                                            if ((9 < DAT_200001b8) && (DAT_200001b1 == 2)) {
                                              DAT_200001b2 = DAT_200001b8 - 10;
                                              DAT_200001af = 0x11;
                                              DAT_200001b0 = 2;
                                              for (DAT_200001b3 = 0; DAT_200001b3 <= DAT_200001b2;
                                                  DAT_200001b3 = DAT_200001b3 + 1) {
                                                DAT_2000019b = (char)DAT_200001fc << 5 | 8U |
                                                               DAT_200001b0;
                                                uVar14 = FUN_08013346(DAT_2000019b,0x18);
                                                DAT_200001fe = (undefined2)uVar14;
                                                DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
                                                DAT_2000019e = (byte)(uVar14 & 3);
                                                if ((uint)(byte)(&DAT_0804a2a2)
                                                                [(uVar14 & 0x3ff) >> 2] ==
                                                    (uVar14 & 3)) {
                                                  if (DAT_200001b5 == 4) {
                                                    pbVar24[(uint)DAT_200001b3 + (uint)DAT_20000204]
                                                         = DAT_2000019d;
                                                  }
                                                  else if (DAT_200001b5 == 8) {
                                                    (&DAT_2000108c)
                                                    [(uint)DAT_200001b3 + (uint)DAT_20000204] =
                                                         DAT_2000019d;
                                                  }
                                                  if ((DAT_200001b0 < 7) ||
                                                     (DAT_200001b2 <= DAT_200001b3)) {
                                                    if ((DAT_200001b0 == 3) &&
                                                       (DAT_200001b3 < DAT_200001b2))
                                                    goto LAB_0804727a;
                                                  }
                                                  else {
                                                    DAT_200001af = DAT_200001af + 1;
LAB_0804727a:
                                                    DAT_20000200 = DAT_200001fc << 0xd | 0x1200 |
                                                                   (ushort)DAT_200001af;
                                                    iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                    DAT_200001fe = (undefined2)iVar8;
                                                    if ((iVar8 != 1) && (iVar8 != 5))
                                                    goto LAB_080472a8;
                                                  }
                                                  DAT_200001b0 = DAT_200001b0 + 1 & 7;
                                                }
LAB_080472a8:
                                              }
                                            }
                                            if (DAT_200001b4 == 1) {
                                              DAT_20000200 = DAT_200001fc << 0xd | 0x1218;
                                              iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                              DAT_200001fe = (undefined2)iVar8;
                                              if ((iVar8 == 1) || (iVar8 == 5)) {
                                                DAT_20000200 = DAT_200001fc << 0xd | 0x1803;
                                                iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                DAT_200001fe = (undefined2)iVar8;
                                                if ((iVar8 == 1) || (iVar8 == 5)) {
                                                  if (pbVar24[(uint)DAT_200001fc * 0x32 + 0xf6] == 0
                                                     ) {
                                                    uVar7 = 0x1900;
                                                  }
                                                  else {
                                                    uVar7 = 0x1980;
                                                  }
                                                  uVar14 = (uint)uVar7 | (DAT_200001fc & 7) << 0xd;
                                                  DAT_20000200 = (ushort)uVar14;
                                                  iVar8 = FUN_08012aac(uVar14,0x19);
                                                  DAT_200001fe = (undefined2)iVar8;
                                                  if ((iVar8 == 1) || (iVar8 == 5)) {
                                                    DAT_20000200 = DAT_200001fc << 0xd | 0x1a14;
                                                    iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                    DAT_200001fe = (undefined2)iVar8;
                                                    if ((iVar8 == 1) || (iVar8 == 5)) {
                                                      uVar7 = 0x1b00;
                                                      goto LAB_080473ca;
                                                    }
                                                  }
                                                }
                                              }
                                            }
                                            else if (DAT_200001b4 == 2) {
                                              DAT_20000200 = DAT_200001fc << 0xd | 0x1218;
                                              iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                              DAT_200001fe = (undefined2)iVar8;
                                              if ((iVar8 == 1) || (iVar8 == 5)) {
                                                DAT_20000200 = DAT_200001fc << 0xd | 0x1801;
                                                iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                DAT_200001fe = (undefined2)iVar8;
                                                if ((iVar8 == 1) || (iVar8 == 5)) {
                                                  uVar7 = 0x1900;
LAB_080473ca:
                                                  DAT_20000200 = uVar7 | DAT_200001fc << 0xd;
                                                  iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                  DAT_200001fe = (undefined2)iVar8;
                                                  if ((iVar8 == 1) || (iVar8 == 5)) {
LAB_080475d2:
                                                    DAT_20000200 = DAT_200001fc << 0xd | 0x1400;
                                                    iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                    DAT_200001fe = (undefined2)iVar8;
                                                    if ((iVar8 == 1) || (iVar8 == 5)) {
                                                      DAT_20000200 = DAT_200001fc << 0xd | 0x1581;
                                                      iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                      DAT_200001fe = (undefined2)iVar8;
                                                      if ((iVar8 == 1) || (iVar8 == 5))
                                                      goto LAB_08047610;
                                                    }
                                                  }
                                                }
                                              }
                                            }
                                            else {
                                              if (DAT_200001b4 != 3) goto LAB_08047610;
                                              DAT_20000200 = DAT_200001fc << 0xd | 0x1218;
                                              iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                              DAT_200001fe = (undefined2)iVar8;
                                              if ((iVar8 == 1) || (iVar8 == 5)) {
                                                DAT_20000200 = 0x1800;
                                                if ((DAT_200001b5 == 8) || (DAT_200001b5 == 4)) {
                                                  DAT_20000200 = DAT_200001fc << 0xd | 0x1800 |
                                                                 DAT_200001b9 + 1;
                                                }
                                                iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                DAT_200001fe = (undefined2)iVar8;
                                                if ((iVar8 == 1) || (iVar8 == 5)) {
                                                  DAT_20000200 = DAT_200001fc << 0xd | 0x1900;
                                                  iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                  DAT_200001fe = (undefined2)iVar8;
                                                  if ((iVar8 == 1) || (iVar8 == 5)) {
                                                    DAT_200001af = 0x18;
                                                    DAT_200001b0 = 2;
                                                    DAT_200001b2 = DAT_200001b9 - 1;
                                                    for (DAT_200001b3 = 0;
                                                        uVar14 = (uint)DAT_200001b3,
                                                        uVar14 <= DAT_200001b2;
                                                        DAT_200001b3 = DAT_200001b3 + 1) {
                                                      DAT_20000200 = (ushort)DAT_200001b0 * 0x100 +
                                                                     0x1800;
                                                      if (DAT_200001b5 == 8) {
                                                        bVar5 = (&DAT_2000108c)
                                                                [DAT_20000204 + uVar14];
LAB_080474ec:
                                                        DAT_20000200 = (ushort)bVar5 |
                                                                       DAT_20000200 |
                                                                       DAT_200001fc << 0xd;
                                                      }
                                                      else if (DAT_200001b5 == 4) {
                                                        bVar5 = pbVar24[DAT_20000204 + uVar14];
                                                        goto LAB_080474ec;
                                                      }
                                                      iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                      DAT_200001fe = (undefined2)iVar8;
                                                      if ((iVar8 == 1) || (iVar8 == 5)) {
                                                        if (DAT_200001b0 < 7) {
                                                          if ((DAT_200001b0 == 3) &&
                                                             (DAT_200001b3 < DAT_200001b2)) {
                                                            DAT_20000200 = DAT_200001fc << 0xd |
                                                                           0x1200 | (ushort)
                                                  DAT_200001af;
                                                  goto LAB_0804757c;
                                                  }
                                                  }
                                                  else if (DAT_200001b3 < DAT_200001b2) {
                                                    DAT_200001af = DAT_200001af + 1;
                                                    DAT_20000200 = DAT_200001fc << 0xd | 0x1200 |
                                                                   (ushort)DAT_200001af;
LAB_0804757c:
                                                    iVar8 = FUN_08012aac(DAT_20000200,0x19);
                                                    DAT_200001fe = (undefined2)iVar8;
                                                    if ((iVar8 != 1) && (iVar8 != 5))
                                                    goto LAB_080475ac;
                                                  }
                                                  DAT_200001b0 = DAT_200001b0 + 1 & 7;
                                                  }
LAB_080475ac:
                                                  }
                                                  goto LAB_080475d2;
                                                  }
                                                }
                                              }
                                            }
                                          }
                                        }
                                      }
                                    }
                                  }
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
            else {
LAB_08047610:
              DAT_20000200 = DAT_200001fc << 0xd | 0x1200;
              iVar8 = FUN_08012aac(DAT_20000200,0x19);
              DAT_200001fe = (undefined2)iVar8;
              if ((iVar8 == 1) || (iVar8 == 5)) {
                uVar14 = (uint)(&DAT_20001046)[pbVar24[(uint)DAT_200001fc * 0x32 + 0xf5]] |
                         (DAT_200001fc & 7) << 0xd | 0x1c00;
                DAT_20000200 = (ushort)uVar14;
                iVar8 = FUN_08012aac(uVar14,0x19);
                DAT_200001fe = (undefined2)iVar8;
                if (((iVar8 == 1) || (iVar8 == 5)) &&
                   ((&DAT_20001046)[pbVar24[(uint)DAT_200001fc * 0x32 + 0xf5]] == '\0')) {
                  pbVar24[(uint)DAT_200001fc * 0x32 + 0xea] =
                       pbVar24[(uint)DAT_200001fc * 0x32 + 0xea] | 0x80;
                }
              }
            }
          }
        }
        goto LAB_0804769a;
      }
      DAT_200001a3 = DAT_200001a3 + 1;
      if (DAT_200001a2 <= DAT_200001a3) {
        DAT_200001a3 = 0;
      }
      DAT_2000019b = DAT_200001a3 << 5;
      uVar14 = FUN_08013346(DAT_2000019b,0x18);
      DAT_200001fe = (undefined2)uVar14;
      DAT_2000019c = (byte)((uVar14 << 0x16) >> 0x18);
      DAT_2000019d = (byte)(uVar14 & 3);
      if ((uint)(byte)(&DAT_0804a2a2)[(uVar14 & 0x3ff) >> 2] == (uVar14 & 3)) {
        bVar5 = pbVar24[(uint)DAT_200001a3 * 2 + 9] & 0xef;
      }
      else {
        bVar5 = pbVar24[(uint)DAT_200001a3 * 2 + 9] | 0x10;
      }
      pbVar24[(uint)DAT_200001a3 * 2 + 9] = bVar5;
      DAT_2000019b = DAT_200001a3 << 5 | 1;
      uVar14 = FUN_08013346(DAT_2000019b,0x18);
      DAT_200001fe = (undefined2)uVar14;
      uVar10 = (uVar14 & 0x3ff) >> 2;
      DAT_2000019f = (undefined1)(uVar14 & 3);
      if ((uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3)) {
        bVar5 = pbVar24[(uint)DAT_200001a3 * 2 + 9] & 0xef;
      }
      else {
        bVar5 = pbVar24[(uint)DAT_200001a3 * 2 + 9] | 0x10;
      }
      pbVar24[(uint)DAT_200001a3 * 2 + 9] = bVar5;
      uVar10 = uVar10 & 4;
      DAT_2000019e = (byte)uVar10;
      if (uVar10 == 0) {
LAB_080477a6:
        pbVar24 = pbVar24 + (uint)DAT_200001a3 * 2;
        bVar5 = pbVar24[9] & 0xfb;
      }
      else {
        if (((byte)(&DAT_20000356)[DAT_200001a3] < 0x10) ||
           (0x7f < (byte)(&DAT_20000356)[DAT_200001a3])) goto LAB_080477a6;
        pbVar24 = pbVar24 + (uint)DAT_200001a3 * 2;
        bVar5 = pbVar24[9] | 4;
      }
      pbVar24[9] = bVar5;
      for (DAT_200001fc = 0; (uint)DAT_200001fc < (uint)DAT_200001a2;
          DAT_200001fc = DAT_200001fc + 1) {
        uVar14 = (uint)(byte)(&DAT_20000356)[DAT_200001fc];
        if (uVar14 - 1 < 0xf) {
          for (uVar14 = 0; uVar14 < (byte)(&DAT_2000036b)[DAT_200001fc];
              uVar14 = uVar14 + 1 & 0xffff) {
            uVar10 = (uint)DAT_200001fc << 5 | 8 | uVar14;
            DAT_2000019b = (byte)uVar10;
            uVar10 = FUN_08013346(uVar10 & 0xff,0x18);
            DAT_200001fe = (undefined2)uVar10;
            DAT_2000019c = (byte)((uVar10 << 0x16) >> 0x18);
            DAT_2000019d = (byte)(uVar10 & 3);
            if ((uint)(byte)(&DAT_0804a2a2)[(uVar10 & 0x3ff) >> 2] == (uVar10 & 3)) {
              (&DAT_20005bd0)[(byte)(&DAT_2000035d)[DAT_200001fc] + uVar14] = DAT_2000019c;
            }
          }
        }
        else if (uVar14 - 0x10 < 0x30) {
          for (uVar14 = 0; uVar14 < (byte)(&DAT_2000036b)[DAT_200001fc];
              uVar14 = uVar14 + 2 & 0xffff) {
            DAT_200001a0 = (byte)uVar14 & 7;
            if (uVar14 < 8) {
              uVar10 = 0x1200;
            }
            else {
              uVar10 = 0x1201;
            }
            uVar10 = uVar10 | (DAT_200001fc & 7) << 0xd;
            DAT_20000200 = (ushort)uVar10;
            iVar8 = FUN_08012aac(uVar10,0x19);
            DAT_200001fe = (undefined2)iVar8;
            if ((iVar8 != 1) && (iVar8 != 5)) break;
            if ((byte)(&DAT_20000356)[DAT_200001fc] < 0x21) {
              uVar10 = 0x18d3;
            }
            else {
              uVar10 = 0x1894;
            }
            uVar10 = uVar10 | (DAT_200001fc & 7) << 0xd | (uint)DAT_200001a0 << 8;
            DAT_20000200 = (ushort)uVar10;
            iVar8 = FUN_08012aac(uVar10,0x19);
            DAT_200001fe = (undefined2)iVar8;
            if ((iVar8 != 1) && (iVar8 != 5)) break;
            DAT_20000200 = DAT_200001fc << 0xd | 0x1900 | (ushort)DAT_200001a0 * 0x100 + 0x100;
            iVar8 = FUN_08012aac(DAT_20000200,0x19);
            DAT_200001fe = (undefined2)iVar8;
            if ((iVar8 != 1) && (iVar8 != 5)) break;
            FUN_080043b8(200);
            DAT_2000019b = (char)DAT_200001fc << 5 | 8U | DAT_200001a0;
            uVar10 = FUN_08013346(DAT_2000019b,0x18);
            DAT_200001fe = (undefined2)uVar10;
            DAT_2000019c = (byte)((uVar10 << 0x16) >> 0x18);
            DAT_2000019d = (byte)(uVar10 & 3);
            if ((uint)(byte)(&DAT_0804a2a2)[(uVar10 & 0x3ff) >> 2] != (uVar10 & 3)) break;
            DAT_2000019b = (char)DAT_200001fc << 5 | 9U | DAT_200001a0 + 1;
            uVar10 = FUN_08013346(DAT_2000019b,0x18);
            DAT_200001fe = (undefined2)uVar10;
            uVar15 = (uVar10 & 0x3ff) >> 2;
            DAT_2000019e = (byte)((uVar10 << 0x16) >> 0x18);
            DAT_2000019f = (undefined1)(uVar10 & 3);
            if ((uint)(byte)(&DAT_0804a2a2)[uVar15] != (uVar10 & 3)) break;
            uVar10 = (uint)DAT_200001fc;
            if ((byte)(&DAT_20000356)[uVar10] < 0x21) {
              uVar10 = (uint)(byte)(&DAT_2000035d)[uVar10];
              uVar12 = (undefined1)(uVar15 << 3);
              (&DAT_20005e1a)[uVar10 + uVar14] = DAT_2000019c << 3 | DAT_2000019e >> 5;
            }
            else {
              uVar10 = (uint)(byte)(&DAT_2000035d)[uVar10];
              uVar12 = (undefined1)(uVar15 << 4);
              (&DAT_20005e1a)[uVar10 + uVar14] = DAT_2000019c << 4 | DAT_2000019e >> 4;
            }
            (&DAT_20005e1b)[uVar10 + uVar14] = uVar12;
          }
        }
        else if (uVar14 - 0x41 < 0x3f) {
          for (uVar14 = 0; uVar14 < (byte)(&DAT_2000036b)[DAT_200001fc];
              uVar14 = uVar14 + 2 & 0xffff) {
            DAT_200001a0 = (byte)uVar14 & 7;
            if (uVar14 < 8) {
              uVar10 = 0x1200;
            }
            else {
              uVar10 = 0x1201;
            }
            uVar10 = uVar10 | (DAT_200001fc & 7) << 0xd;
            DAT_20000200 = (ushort)uVar10;
            iVar8 = FUN_08012aac(uVar10,0x19);
            DAT_200001fe = (undefined2)iVar8;
            if ((iVar8 != 1) && (iVar8 != 5)) break;
            DAT_2000019b = (char)DAT_200001fc << 5 | 8U | DAT_200001a0;
            uVar10 = FUN_08013346(DAT_2000019b,0x18);
            DAT_200001fe = (undefined2)uVar10;
            DAT_2000019c = (byte)((uVar10 << 0x16) >> 0x18);
            DAT_2000019d = (byte)(uVar10 & 3);
            if ((uint)(byte)(&DAT_0804a2a2)[(uVar10 & 0x3ff) >> 2] != (uVar10 & 3)) break;
            DAT_2000019b = (char)DAT_200001fc << 5 | 9U | DAT_200001a0 + 1;
            uVar10 = FUN_08013346(DAT_2000019b,0x18);
            DAT_200001fe = (undefined2)uVar10;
            bVar5 = (byte)((uVar10 << 0x16) >> 0x18);
            DAT_2000019f = (undefined1)(uVar10 & 3);
            DAT_2000019e = bVar5;
            if ((uint)(byte)(&DAT_0804a2a2)[(uVar10 & 0x3ff) >> 2] != (uVar10 & 3)) break;
            bVar20 = (&DAT_2000035d)[DAT_200001fc];
            (&DAT_20005e1a)[bVar20 + uVar14] = DAT_2000019c;
            (&DAT_20005e1b)[bVar20 + uVar14] = bVar5;
          }
        }
        else if ((uVar14 != 0xa0) && (uVar14 != 0xa1)) break;
      }
LAB_08047ba4:
      uVar14 = (uint)DAT_20005f2c;
      if ((int)(uVar14 << 0x18) < 0) {
        uVar10 = 0;
        do {
          uVar15 = 0;
          do {
            if (((byte)(&DAT_20005ab0)[uVar10] >> uVar15 & 1) != 0) {
              if (((byte)(&DAT_20005ac0)[uVar10] >> uVar15 & 1) == 0) {
                bVar5 = (&DAT_20005bd0)[uVar10] & ~(byte)(1 << uVar15);
              }
              else {
                bVar5 = (&DAT_20005bd0)[uVar10] | (byte)(1 << uVar15);
              }
              (&DAT_20005bd0)[uVar10] = bVar5;
            }
            uVar15 = uVar15 + 1 & 0xff;
          } while (uVar15 < 8);
          uVar10 = uVar10 + 1 & 0xff;
        } while (uVar10 < 0x10);
        if ((int)(uVar14 << 0x18) < 0) {
          uVar14 = 0;
          do {
            if ((&DAT_20005af0)[uVar14 * 0xe] == '\x10') {
              iVar8 = uVar14 * 0xe;
              cVar13 = (&DAT_20005af1)[iVar8];
              if (cVar13 == '\x02') {
                if ((&DAT_20005af6)[iVar8] == -0x7d) {
                  (&DAT_20001000)
                  [(uint)(byte)(&DAT_20005af9)[iVar8] +
                   ((uint)(byte)(&DAT_20005af8)[iVar8] + (uint)(byte)(&DAT_20005af7)[iVar8] * 0x100)
                   * 0x100 >> 3] = (&DAT_20005afd)[iVar8];
                }
                else if ((&DAT_20005af6)[iVar8] == -0x7c) {
                  (&DAT_2000108c)
                  [(uint)(byte)(&DAT_20005af9)[iVar8] +
                   ((uint)(byte)(&DAT_20005af8)[iVar8] + (uint)(byte)(&DAT_20005af7)[iVar8] * 0x100)
                   * 0x100 >> 3] = (&DAT_20005afd)[iVar8];
                }
              }
              else if (cVar13 == '\x04') {
                cVar13 = (&DAT_20005af6)[iVar8];
                if (cVar13 == '\x06') {
                  uVar12 = (&DAT_20005afc)[iVar8];
                  uVar10 = (uint)(byte)(&DAT_20005af9)[iVar8] +
                           ((uint)(byte)(&DAT_20005af8)[iVar8] +
                           (uint)(byte)(&DAT_20005af7)[iVar8] * 0x100) * 0x100 >> 3;
                  puVar18 = &DAT_20005e1a;
LAB_08047d14:
                  puVar18[uVar10] = uVar12;
                  uVar12 = (&DAT_20005afd)[iVar8];
                  puVar18 = puVar18 + uVar10;
                }
                else {
                  if (cVar13 == '\a') {
                    uVar12 = (&DAT_20005afc)[iVar8];
                    uVar10 = (uint)(byte)(&DAT_20005af9)[iVar8] +
                             ((uint)(byte)(&DAT_20005af8)[iVar8] +
                             (uint)(byte)(&DAT_20005af7)[iVar8] * 0x100) * 0x100 >> 3;
                    puVar18 = &DAT_20005e98;
                    goto LAB_08047d14;
                  }
                  if (cVar13 == -0x7d) {
                    uVar12 = (&DAT_20005afc)[iVar8];
                    uVar10 = (uint)(byte)(&DAT_20005af9)[iVar8] +
                             ((uint)(byte)(&DAT_20005af8)[iVar8] +
                             (uint)(byte)(&DAT_20005af7)[iVar8] * 0x100) * 0x100 >> 3;
                    puVar18 = &DAT_20001000;
                    goto LAB_08047d14;
                  }
                  if (cVar13 != -0x7c) goto LAB_08047d76;
                  puVar18 = &DAT_2000108c +
                            ((uint)(byte)(&DAT_20005af9)[iVar8] +
                             ((uint)(byte)(&DAT_20005af8)[iVar8] +
                             (uint)(byte)(&DAT_20005af7)[iVar8] * 0x100) * 0x100 >> 3);
                  *puVar18 = (&DAT_20005afc)[iVar8];
                  uVar12 = (&DAT_20005afd)[iVar8];
                }
                puVar18[1] = uVar12;
              }
              else if (cVar13 == '\x06') {
                if ((&DAT_20005af6)[iVar8] == -0x7d) {
                  uVar10 = (uint)(byte)(&DAT_20005af9)[iVar8] +
                           ((uint)(byte)(&DAT_20005af8)[iVar8] +
                           (uint)(byte)(&DAT_20005af7)[iVar8] * 0x100) * 0x100 >> 3;
                  (&DAT_20001000)[uVar10] = (&DAT_20005afa)[iVar8];
                  puVar18 = &DAT_20001000 + uVar10;
                  (&DAT_20001001)[uVar10] = (&DAT_20005afb)[iVar8];
                }
                else {
                  if ((&DAT_20005af6)[iVar8] != -0x7c) goto LAB_08047d76;
                  uVar10 = (uint)(byte)(&DAT_20005af9)[iVar8] +
                           ((uint)(byte)(&DAT_20005af8)[iVar8] +
                           (uint)(byte)(&DAT_20005af7)[iVar8] * 0x100) * 0x100 >> 3;
                  puVar18 = &DAT_2000108c + uVar10;
                  *puVar18 = (&DAT_20005afa)[iVar8];
                  (&DAT_2000108d)[uVar10] = (&DAT_20005afb)[iVar8];
                }
                puVar18[2] = (&DAT_20005afc)[iVar8];
                puVar18[3] = (&DAT_20005afd)[iVar8];
              }
            }
LAB_08047d76:
            uVar14 = uVar14 + 1 & 0xff;
          } while (uVar14 < 0x10);

  /* ==========================================================================
   *   8.7 Watchdog Service, Local Output Refresh (FUN_0801520e) & Power-Fail
   *       Emergency Retentive Flash Save (PE6 input / FUN_08037746)
   * ========================================================================== */
        }
      }
      DAT_200002b0 = FUN_08036d76();
      FUN_0801520e();
      DAT_200038d2 = (undefined1)(DAT_20000220 >> 0x18);
      DAT_200038d3 = (undefined1)(DAT_20000220 >> 0x10);
      DAT_200038d4 = (undefined1)(DAT_20000220 >> 8);
      DAT_200038d5 = (undefined1)DAT_20000220;
      DAT_20000240 = DAT_20000220;
      if (DAT_20000220 < DAT_20000244) {
        uVar14 = ~DAT_20000244;
      }
      else {
        uVar14 = -DAT_20000244;
      }
      DAT_20000248 = uVar14 + DAT_20000220;
      if (((499 < DAT_20000248) && (DAT_2000017b == '\0')) && (DAT_2000016f == '\x01')) {
        DAT_2000016f = '\0';
        DAT_20000188 = '\x03';
      }
      DAT_20000244 = DAT_20000220;
      DAT_2000017b = '\0';
      DAT_20000288 = DAT_20000220 & 0x7fffff;
      if (((DAT_20000288 + 0xdbf37bff < 99999) && (DAT_20000143 == '\0')) ||
         (DAT_200038cc == '\x04')) {
        DAT_40003000 = 0xaaaa;
        DAT_20000143 = '\x01';
        DAT_200038cc = '\0';
        DAT_40003000 = 0xaaaa;
      }
      DAT_20000320 = DAT_20000220;
      if (DAT_20000220 < DAT_20000324) {
        uVar14 = ~DAT_20000324;
      }
      else {
        uVar14 = -DAT_20000324;
      }
      DAT_20000328 = uVar14 + DAT_20000220;
      if (29999 < DAT_20000328) {
        if ((int)((uint)DAT_20005f28 << 0x1b) < 0) {
          DAT_20005f28 = DAT_20005f28 & 0xef;
        }
        else {
          DAT_20005f28 = DAT_20005f28 | 0x10;
        }
        DAT_20000324 = DAT_20000220;
      }
      DAT_2000032c = DAT_20000220;
      if (DAT_20000220 < DAT_20000330) {
        uVar14 = ~DAT_20000330;
      }
      else {
        uVar14 = -DAT_20000330;
      }
      DAT_20000334 = uVar14 + DAT_20000220;
      if (499 < DAT_20000334) {
        if ((int)((uint)DAT_20005f28 << 0x1a) < 0) {
          DAT_20005f28 = DAT_20005f28 & 0xdf;
        }
        else {
          DAT_20005f28 = DAT_20005f28 | 0x20;
        }
        DAT_20000330 = DAT_20000220 - DAT_20000334 % 500;
      }
      if ((int)((uint)DAT_20005f28 << 0x19) < 0) {
        DAT_20005f28 = DAT_20005f28 & 0xbf;
      }
      else {
        DAT_20005f28 = DAT_20005f28 | 0x40;
      }
      if (DAT_20000342 != '\0') {
        DAT_2000017c = DAT_20000298 <= DAT_20006160;
        DAT_20000298 = DAT_20006160;
      }
      if (DAT_200001c3 == 0) {
        DAT_2000016f = '\0';
      }
      else if (((DAT_200001c3 == 1) && (DAT_200001c5 == 0)) && (DAT_200001c7 == '\x01')) {
        FUN_0803bf24();
      }
      DAT_200001c5 = DAT_200001c3;
      DAT_200001c7 = '\x01';
      if (200 < DAT_20000240) {
        DAT_2000017a = '\x01';
      }
      if (((DAT_2000016f == '\0') && (DAT_20000176 == '\x01')) && (DAT_20000177 == '\0')) {
        uRam422201b0 = 0;
        if (DAT_2000017a == '\x01') {
          disableIRQinterrupts();
          if (DAT_20003895 == -1) {
            DAT_20003894 = DAT_20003894 + '\x01';
          }
          DAT_20003892 = (undefined1)(DAT_20000220 >> 8);
          DAT_20003893 = (undefined1)DAT_20000220;
          DAT_20003895 = DAT_20003895 + '\x01';
          FUN_08037746();
          uVar14 = DAT_e000ed0c;
          DAT_e000ed0c = uVar14 & 0x700 | 0x5fa0004;
          DataSynchronizationBarrier(0xf);
          do {
                    /* WARNING: Do nothing block with infinite loop */
          } while( true );
        }
        DAT_20000177 = '\x01';
        DAT_20000178 = '\0';
      }
      if (DAT_20000179 == 0) {
        DAT_20000176 = '\0';
      }
      else {
        uRam422201b0 = 0;
        DAT_20000176 = '\x01';
        DAT_2000016f = '\0';
        uVar14 = 0;
        do {
          (&DAT_20001046)[uVar14] = 0;
          uVar14 = uVar14 + 1 & 0xffff;
        } while (uVar14 < 0x20);
      }
      if (DAT_20000176 == '\0' && DAT_20000178 == '\0') {
        DAT_20000178 = '\x01';
        DAT_20000177 = '\0';
        uVar14 = 0;
        bVar5 = DAT_20000179;
        do {
          DAT_20000179 = bVar5;
          bVar5 = FUN_08014520(&DAT_40011000);
          DAT_20000179 = bVar5 & 0x20;
          DAT_40003000 = 0xaaaa;
          if (DAT_20000179 != 0) {
            DAT_20000178 = '\0';
            DAT_20000177 = '\x01';
            break;
          }
          uVar14 = uVar14 + 1;
          bVar5 = 0;
        } while (uVar14 < 6000000);
        if ((DAT_200001c3 == 1) && (DAT_20000178 == '\x01')) {
          FUN_0803bf24();
        }
      }
      if (((DAT_20000170 != '\0') && (DAT_2000016f == '\0')) &&
         ((DAT_20000176 == '\0' && DAT_20000177 == '\0' && (DAT_20005a39 == '\0')))) {
        uVar14 = 0;

  /* ==========================================================================
   *   8.8 Dynamic Port 0/1 Baudrate & Freeport Reconfiguration (SMB30 / SMB130),
   *       Pulse Catch / Edge Interrupt Polling & W5500 Ethernet Service
   * ========================================================================== */
        do {
          (&DAT_20001046)[uVar14] = (&DAT_20005a3a)[uVar14];
          uVar14 = uVar14 + 1 & 0xffff;
        } while (uVar14 < 0x10);
      }
      DAT_20000170 = DAT_2000016f;
      if (DAT_200001cf == '\x01') {
        if (DAT_20000220 < DAT_20000310) {
          uVar14 = ~DAT_20000310;
        }
        else {
          uVar14 = -DAT_20000310;
        }
        DAT_20000314 = DAT_20000220 + uVar14;
        if (800 < DAT_20000314) {
          DAT_200001cf = '\0';
          FUN_0804211c();
          FUN_080421f0();
        }
      }
      uVar14 = (uint)DAT_20005f46;
      if (((DAT_20005f46 & 3) == 0) && ((DAT_200001cb & 3) != 0)) {
        DAT_20000185 = '\0';
        DAT_20000175 = DAT_200058fb;
        if (DAT_200058fb == '\0') {
LAB_08048190:
          iVar8 = 0x2580;
          uVar12 = 9;
          cVar13 = '\x02';
        }
        else if (DAT_200058fb == '\x02') {
          iVar8 = 0x4b00;
          uVar12 = 9;
          cVar13 = '\x02';
        }
        else {
          if (DAT_200058fb != '\x04') {
            DAT_20000175 = '\0';
            goto LAB_08048190;
          }
          iVar8 = 0x2dc6c;
          uVar12 = 9;
          cVar13 = '\x02';
        }
LAB_080483da:
        FUN_08041b98(iVar8,cVar13,uVar12,1);
        FUN_08003fbe();
        FUN_08041fbe(&DAT_40004400,0x80,1);
        FUN_08041fbe(&DAT_40004400,0x40,1);
        FUN_08042042(&DAT_40004400,0x424,1);
        FUN_08042042(&DAT_40004400,0x626,1);
        FUN_08003ee2(&DAT_40020080,0);
        FUN_08003ee2(&DAT_4002006c,1);
      }
      else {
        if (((uVar14 & 3) == 1) && ((DAT_200001cb & 3) != 1)) {
          switch((uVar14 & 0x1f) >> 2) {
          case 0:
            DAT_20000764 = 0x9600;
            break;
          case 1:
            DAT_20000764 = 0x4b00;
            break;
          case 2:
            DAT_20000764 = 0x2580;
            break;
          case 3:
            DAT_20000764 = 0x12c0;
            break;
          case 4:
            DAT_20000764 = 0x960;
            break;
          case 5:
            DAT_20000764 = 0x4b0;
            break;
          case 6:
            DAT_20000764 = 0x1c200;
            break;
          case 7:
            DAT_20000764 = 0xe100;
          }
          bVar5 = DAT_20005f46 >> 6;
          if (bVar5 == 0) {
LAB_08048236:
            DAT_200005f4 = '\0';
          }
          else if (bVar5 == 1) {
            DAT_200005f4 = '\x02';
          }
          else {
            if (bVar5 == 2) goto LAB_08048236;
            if (bVar5 == 3) {
              DAT_200005f4 = '\x01';
            }
          }
          iVar8 = uVar14 << 0x1a;
          DAT_20000147 = (byte)((uint)iVar8 >> 0x1f);
          if (iVar8 < 0) {
            if (iVar8 < 0) {
              DAT_200005f5 = 8;
            }
          }
          else if (DAT_200005f4 == '\0') {
            DAT_200005f5 = 8;
          }
          else {
            DAT_200005f5 = 9;
          }
          DAT_20000185 = '\x01';
          iVar8 = DAT_20000764;
          uVar12 = DAT_200005f5;
          cVar13 = DAT_200005f4;
          goto LAB_080483da;
        }
        if (((uVar14 & 3) == 2) && ((DAT_200001cb & 3) != 2)) {
          DAT_20000185 = '\x02';
        }
        else if (((~uVar14 & 3) == 0) && ((~DAT_200001cb & 3) != 0)) {
          switch((uVar14 & 0x1f) >> 2) {
          case 0:
            break;
          case 1:
            break;
          case 2:
            break;
          case 3:
            break;
          case 4:
            break;
          case 5:
            break;
          case 6:
            break;
          case 7:
          }
          bVar5 = DAT_20005f46 >> 6;
          if (bVar5 == 0) {
LAB_0804834c:
            DAT_200005f4 = '\0';
          }
          else if (bVar5 == 1) {
            DAT_200005f4 = '\x02';
          }
          else {
            if (bVar5 == 2) goto LAB_0804834c;
            if (bVar5 == 3) {
              DAT_200005f4 = '\x01';
            }
          }
          iVar8 = uVar14 << 0x1a;
          DAT_20000147 = (byte)((uint)iVar8 >> 0x1f);
          if (iVar8 < 0) {
            if (iVar8 < 0) {
LAB_08048398:
              DAT_200005f5 = 8;
            }
          }
          else {
            if (DAT_200005f4 == '\0') goto LAB_08048398;
            DAT_200005f5 = 9;
          }
          iVar8 = ((uint)DAT_20005f83 + (uint)DAT_20005f82 * 0x100) * 0x10000 +
                  (uint)DAT_20005f84 * 0x100 + (uint)DAT_20005f85;
          DAT_20000185 = '\x03';
          DAT_20000764 = iVar8;
          uVar12 = DAT_200005f5;
          cVar13 = DAT_200005f4;
          goto LAB_080483da;
        }
      }
      DAT_200001cb = DAT_20005f46;
      uVar14 = (uint)DAT_20005faa;
      if (((DAT_20005faa & 3) == 0) && ((DAT_200001ca & 3) != 0)) {
        DAT_20000184 = '\0';
        DAT_20000174 = DAT_20005991;
        if (DAT_20005991 == '\0') {
LAB_0804847a:
          iVar8 = 0x2580;
          uVar12 = 9;
          cVar13 = '\x02';
        }
        else if (DAT_20005991 == '\x02') {
          iVar8 = 0x4b00;
          uVar12 = 9;
          cVar13 = '\x02';
        }
        else {
          if (DAT_20005991 != '\x04') {
            DAT_20000174 = '\0';
            goto LAB_0804847a;
          }
          iVar8 = 0x2dc6c;
          uVar12 = 9;
          cVar13 = '\x02';
        }
LAB_080486a8:
        FUN_08040e18(iVar8,cVar13,uVar12,1);
        FUN_08003ef8();
        FUN_08041fbe(&DAT_40013800,0x80,1);
        FUN_08041fbe(&DAT_40013800,0x40,1);
        FUN_08042042(&DAT_40013800,0x424,1);
        FUN_08042042(&DAT_40013800,0x626,1);
        FUN_08003ee2(&DAT_40020044,0);
        FUN_08003ee2(&DAT_40020058,1);
      }
      else {
        if (((uVar14 & 3) == 1) && ((DAT_200001ca & 3) != 1)) {
          switch((uVar14 & 0x1f) >> 2) {
          case 0:
            DAT_20000764 = 0x9600;
            break;
          case 1:
            DAT_20000764 = 0x4b00;
            break;
          case 2:
            DAT_20000764 = 0x2580;
            break;
          case 3:
            DAT_20000764 = 0x12c0;
            break;
          case 4:
            DAT_20000764 = 0x960;
            break;
          case 5:
            DAT_20000764 = 0x4b0;
            break;
          case 6:
            DAT_20000764 = 0x1c200;
            break;
          case 7:
            DAT_20000764 = 0xe100;
          }
          bVar5 = DAT_20005faa >> 6;
          if (bVar5 == 0) {
LAB_08048520:
            DAT_200005f4 = '\0';
          }
          else if (bVar5 == 1) {
            DAT_200005f4 = '\x02';
          }
          else {
            if (bVar5 == 2) goto LAB_08048520;
            if (bVar5 == 3) {
              DAT_200005f4 = '\x01';
            }
          }
          iVar8 = uVar14 << 0x1a;
          DAT_20000147 = (byte)((uint)iVar8 >> 0x1f);
          if (iVar8 < 0) {
            if (iVar8 < 0) {
              DAT_200005f5 = 8;
            }
          }
          else if (DAT_200005f4 == '\0') {
            DAT_200005f5 = 8;
          }
          else {
            DAT_200005f5 = 9;
          }
          DAT_20000184 = '\x01';
          iVar8 = DAT_20000764;
          uVar12 = DAT_200005f5;
          cVar13 = DAT_200005f4;
          goto LAB_080486a8;
        }
        if (((uVar14 & 3) == 2) && ((DAT_200001ca & 3) != 2)) {
          DAT_20000184 = '\x02';
        }
        else if (((~uVar14 & 3) == 0) && ((~DAT_200001ca & 3) != 0)) {
          switch((uVar14 & 0x1f) >> 2) {
          case 0:
            break;
          case 1:
            break;
          case 2:
            break;
          case 3:
            break;
          case 4:
            break;
          case 5:
            break;
          case 6:
            break;
          case 7:
          }
          bVar5 = DAT_20005faa >> 6;
          if (bVar5 == 0) {
LAB_08048636:
            DAT_200005f4 = '\0';
          }
          else if (bVar5 == 1) {
            DAT_200005f4 = '\x02';
          }
          else {
            if (bVar5 == 2) goto LAB_08048636;
            if (bVar5 == 3) {
              DAT_200005f4 = '\x01';
            }
          }
          iVar8 = uVar14 << 0x1a;
          DAT_20000147 = (byte)((uint)iVar8 >> 0x1f);
          if (iVar8 < 0) {
            if (iVar8 < 0) {
              DAT_200005f5 = 8;
            }
          }
          else if (DAT_200005f4 == '\0') {
            DAT_200005f5 = 8;
          }
          else {
            DAT_200005f5 = 9;
          }
          iVar8 = ((uint)DAT_20005fe7 + (uint)DAT_20005fe6 * 0x100) * 0x10000 +
                  (uint)DAT_20005fe8 * 0x100 + (uint)DAT_20005fe9;
          DAT_20000184 = '\x03';
          DAT_20000764 = iVar8;
          uVar12 = DAT_200005f5;
          cVar13 = DAT_200005f4;
          goto LAB_080486a8;
        }
      }
      DAT_200001ca = DAT_20005faa;
      if ((8 < DAT_200007ec) || (DAT_200007ec == 0)) {
        DAT_200007ec = 1;
      }
      if (DAT_200001cc == '\x01') {
        DAT_20000308 = DAT_20000220;
        if (DAT_20000220 < DAT_20000304) {
          uVar14 = ~DAT_20000304;
        }
        else {
          uVar14 = -DAT_20000304;
        }
        DAT_2000030c = DAT_20000220 + uVar14;
        if (((DAT_2000030c < 0x65) || (DAT_200001ce != '\0')) && (DAT_2000030c < 0x1f5)) {
          if (DAT_200001cd != '\x01') goto LAB_08048816;
LAB_080487c6:
          **(undefined1 **)(&DAT_2000bba4 + (uint)DAT_200007ec * 4) = 0x80;
        }
        else {
          if (DAT_200001cd == '\x01') goto LAB_080487c6;
          **(undefined1 **)(&DAT_2000bba4 + (uint)DAT_200007ec * 4) = 0x21;
          if (DAT_20000184 == '\x02') {
            DAT_20005faa = DAT_20005faa & 0xfd;
          }
          else if (DAT_20000185 == '\x02') {
            DAT_20005f46 = DAT_20005f46 & 0xfd;
          }
        }
        if (DAT_200007ec <= DAT_200005f1) {
          DAT_200007ec = DAT_200007ec + 1;
          DAT_200001cc = '\0';
        }
        DAT_200001cd = '\0';
      }
LAB_08048816:
      uVar14 = (uint)DAT_200007ec;
      if (uVar14 < 9) {
        if ((DAT_200005f1 != 0) && (uVar14 <= DAT_200005f1)) {
          if ((&DAT_2000bbc8)[uVar14] == '\0') {
            if (((DAT_20000185 == '\x02') && (DAT_200005f3 == '\0')) && (DAT_200001cc == '\0')) {
              DAT_20000304 = DAT_20000220;
              DAT_200001cc = '\x01';
              DAT_200001cd = '\0';
              DAT_20005f2c = DAT_20005f2c & 0xdf;
              DAT_200001ce = '\0';
              if ((&DAT_2000bbf5)[uVar14] == '\0') {
                DAT_20000b9c = 'h';
                DAT_20000b9d = 0x1b;
                DAT_20000b9e = 0x1b;
                DAT_20000b9f = 0x68;
                DAT_20000ba0 = *(byte *)(*(int *)(&DAT_2000bba4 + uVar14 * 4) + 1);
                DAT_20000ba1 = DAT_20000173;
                DAT_20000ba2 = 0x7c;
                DAT_20000ba3 = 0x32;
                DAT_20000ba4 = 1;
                DAT_20000ba5 = 0;
                DAT_20000ba6 = 0;
                DAT_20000ba7 = 0x17;
                DAT_20000ba8 = 0x17;
                DAT_20000ba9 = 0;
                DAT_20000baa = 0xe;
                DAT_20000bab = 0;
                DAT_20000bac = '\0';
                DAT_20000bad = 4;
                DAT_20000bae = 1;
                DAT_20000baf = 0x12;
                DAT_20000bb0 = 10;
                DAT_20000bb1 = 0x10;
                DAT_20000bb2 = 2;
                DAT_20000bb3 = 0;
                DAT_20000bb4 = *(undefined1 *)(*(int *)(&DAT_2000bba4 + uVar14 * 4) + 6);
                DAT_20000bb5 = 0;
                DAT_20000bb7 = (&DAT_2000bbd1)[uVar14];
                DAT_20000bb6 = DAT_20000bb7 == -0x7c;
                DAT_20000bb8 = (&DAT_2000bbda)[uVar14];
                DAT_20000bb9 = (&DAT_2000bbe3)[uVar14];
                DAT_20000bba = (&DAT_2000bbec)[uVar14];
                DAT_20000147 = 0;
                uVar14 = 4;
                do {
                  DAT_20000147 = (&DAT_20000b9c)[uVar14] + DAT_20000147;
                  uVar14 = uVar14 + 1 & 0xffff;
                } while (uVar14 < 0x1f);
                DAT_20000bbc = 0x16;
                DAT_200000c4 = (undefined1 *)0x21;
                DAT_20000bbb = DAT_20000147;
              }
              else {
                DAT_20000b9c = 'h';
                iVar8 = *(int *)(&DAT_2000bba4 + uVar14 * 4);
                DAT_20000b9d = *(char *)(iVar8 + 6) + 0x1f;
                DAT_20000b9f = 0x68;
                DAT_20000ba0 = *(byte *)(iVar8 + 1);
                DAT_20000ba1 = DAT_20000173;
                DAT_20000ba2 = 0x7c;
                DAT_20000ba3 = 0x32;
                DAT_20000ba4 = 1;
                DAT_20000ba5 = 0;
                DAT_20000ba6 = 0;
                DAT_20000ba7 = 4;
                DAT_20000ba8 = 4;
                DAT_20000ba9 = 0;
                DAT_20000baa = 0xe;
                DAT_20000bab = 0;
                DAT_20000bac = *(char *)(iVar8 + 6) + '\x04';
                DAT_20000bad = 5;
                DAT_20000bae = 1;
                DAT_20000baf = 0x12;
                DAT_20000bb0 = 10;
                DAT_20000bb1 = 0x10;
                DAT_20000bb2 = 2;
                DAT_20000bb3 = 0;
                DAT_20000bb4 = *(undefined1 *)(iVar8 + 6);
                DAT_20000bb5 = 0;
                DAT_20000bb7 = (&DAT_2000bbd1)[uVar14];
                DAT_20000bb6 = DAT_20000bb7 == -0x7c;
                DAT_20000bb8 = (&DAT_2000bbda)[uVar14];
                DAT_20000bb9 = (&DAT_2000bbe3)[uVar14];
                DAT_20000bba = (&DAT_2000bbec)[uVar14];
                DAT_20000bbb = 0;
                DAT_20000bbc = 4;
                DAT_20000bbd = 0;
                DAT_20000bbe = *(char *)(iVar8 + 6) << 3;
                bVar5 = *(byte *)(iVar8 + 6);
                DAT_20000b9e = DAT_20000b9d;
                for (uVar14 = 0; uVar14 < bVar5; uVar14 = uVar14 + 1 & 0xffff) {
                  (&DAT_20000bbf)[uVar14] = *(undefined1 *)(iVar8 + uVar14 + 7);
                }
                DAT_20000148 = *(char *)(iVar8 + 6) + 0x22;
                uVar10 = (uint)DAT_20000148;
                DAT_20000147 = 0;
                for (uVar14 = 4; uVar14 <= uVar10; uVar14 = uVar14 + 1 & 0xffff) {
                  DAT_20000147 = (&DAT_20000b9c)[uVar14] + DAT_20000147;
                }
                (&DAT_20000b9d)[uVar10] = DAT_20000147;
                (&DAT_20000b9e)[uVar10] = 0x16;
                DAT_200000c4 = (undefined1 *)(uVar10 + 3);
              }
              DAT_200005f3 = '\x01';
              FUN_08003ec8(0x2000000);
              FUN_08003ee2(&DAT_40020080,0);
              FUN_0801452a(&DAT_40011800,0x20);
              DAT_40001424 = 0;
              DAT_4000142c = DAT_200001de;
              FUN_0803fbee(&DAT_40001400,1);
              FUN_0803ff08(&DAT_40001400,1);
              FUN_0803fbf4(&DAT_40001400,1);
              puVar11 = &DAT_40004400;
              goto LAB_08048e44;
            }
          }
          else if (((((&DAT_2000bbc8)[uVar14] == '\x01') && (DAT_20000184 == '\x02')) &&
                   (DAT_200005f2 == '\0')) && (DAT_200001cc == '\0')) {
            DAT_20000304 = DAT_20000220;
            DAT_200001cc = '\x01';
            DAT_200001cd = '\0';
            DAT_20005f2c = DAT_20005f2c & 0xbf;
            DAT_200001ce = '\0';
            if ((&DAT_2000bbf5)[uVar14] == '\0') {
              DAT_200055cc = 'h';
              DAT_200055cd = 0x1b;
              DAT_200055ce = 0x1b;
              DAT_200055cf = 0x68;
              DAT_200055d0 = *(byte *)(*(int *)(&DAT_2000bba4 + uVar14 * 4) + 1);
              DAT_200055d1 = DAT_20000172;
              DAT_200055d2 = 0x7c;
              DAT_200055d3 = 0x32;
              DAT_200055d4 = 1;
              DAT_200055d5 = 0;
              DAT_200055d6 = 0;
              DAT_200055d7 = 0x19;
              DAT_200055d8 = 0x19;
              DAT_200055d9 = 0;
              DAT_200055da = 0xe;
              DAT_200055db = 0;
              DAT_200055dc = '\0';
              DAT_200055dd = 4;
              DAT_200055de = 1;
              DAT_200055df = 0x12;
              DAT_200055e0 = 10;
              DAT_200055e1 = 0x10;
              DAT_200055e2 = 2;
              DAT_200055e3 = 0;
              DAT_200055e4 = *(undefined1 *)(*(int *)(&DAT_2000bba4 + uVar14 * 4) + 6);
              DAT_200055e5 = 0;
              DAT_200055e7 = (&DAT_2000bbd1)[uVar14];
              DAT_200055e6 = DAT_200055e7 == -0x7c;
              DAT_200055e8 = (&DAT_2000bbda)[uVar14];
              DAT_200055e9 = (&DAT_2000bbe3)[uVar14];
              DAT_200055ea = (&DAT_2000bbec)[uVar14];
              DAT_20000147 = 0;
              uVar14 = 4;
              do {
                DAT_20000147 = (&DAT_200055cc)[uVar14] + DAT_20000147;
                uVar14 = uVar14 + 1 & 0xffff;
              } while (uVar14 < 0x1f);
              DAT_200055ec = 0x16;
              DAT_200000bc = (undefined1 *)0x21;
              DAT_200055eb = DAT_20000147;
            }
            else {
              DAT_200055cc = 'h';
              iVar8 = *(int *)(&DAT_2000bba4 + uVar14 * 4);
              DAT_200055cd = *(char *)(iVar8 + 6) + 0x1f;
              DAT_200055cf = 0x68;
              DAT_200055d0 = *(byte *)(iVar8 + 1);
              DAT_200055d1 = DAT_20000172;
              DAT_200055d2 = 0x7c;
              DAT_200055d3 = 0x32;
              DAT_200055d4 = 1;
              DAT_200055d5 = 0;
              DAT_200055d6 = 0;
              DAT_200055d7 = 4;
              DAT_200055d8 = 4;
              DAT_200055d9 = 0;
              DAT_200055da = 0xe;
              DAT_200055db = 0;
              DAT_200055dc = *(char *)(iVar8 + 6) + '\x04';
              DAT_200055dd = 5;
              DAT_200055de = 1;
              DAT_200055df = 0x12;
              DAT_200055e0 = 10;
              DAT_200055e1 = 0x10;
              DAT_200055e2 = 2;
              DAT_200055e3 = 0;
              DAT_200055e4 = *(undefined1 *)(iVar8 + 6);
              DAT_200055e5 = 0;
              DAT_200055e7 = (&DAT_2000bbd1)[uVar14];
              DAT_200055e6 = DAT_200055e7 == -0x7c;
              DAT_200055e8 = (&DAT_2000bbda)[uVar14];
              DAT_200055e9 = (&DAT_2000bbe3)[uVar14];
              DAT_200055ea = (&DAT_2000bbec)[uVar14];
              DAT_200055eb = 0;
              DAT_200055ec = 4;
              DAT_200055ed = 0;
              DAT_200055ee = *(char *)(iVar8 + 6) << 3;
              bVar5 = *(byte *)(iVar8 + 6);
              DAT_200055ce = DAT_200055cd;
              for (uVar14 = 0; uVar14 < bVar5; uVar14 = uVar14 + 1 & 0xffff) {
                (&DAT_200055ef)[uVar14] = *(undefined1 *)(iVar8 + uVar14 + 7);
              }
              DAT_20000148 = *(char *)(iVar8 + 6) + 0x22;
              uVar10 = (uint)DAT_20000148;
              DAT_20000147 = 0;
              for (uVar14 = 4; uVar14 <= uVar10; uVar14 = uVar14 + 1 & 0xffff) {
                DAT_20000147 = (&DAT_200055cc)[uVar14] + DAT_20000147;
              }
              (&DAT_200055cd)[uVar10] = DAT_20000147;
              (&DAT_200055ce)[uVar10] = 0x16;
              DAT_200000bc = (undefined1 *)(uVar10 + 3);
            }
            DAT_200005f2 = '\x01';
            FUN_08003ec8(0x2000);
            FUN_08003ee2(&DAT_40020044,0);
            DAT_40001024 = 0;
            DAT_4000102c = DAT_200001d6;
            FUN_0803fbee(&DAT_40001000,1);
            FUN_0803ff08(&DAT_40001000,1);
            FUN_0803fbf4(&DAT_40001000,1);
            puVar11 = &DAT_40013800;
LAB_08048e44:
            FUN_08042042(puVar11,0x424,1);
          }
        }
      }
      else {
        DAT_200005f1 = 0;
        DAT_200007ec = 1;
        DAT_200001cc = '\0';
      }
      if (DAT_200001c8 == '\x01') {
        DAT_200002f0 = DAT_20000220;
        if (DAT_20000220 < DAT_200002ec) {
          uVar14 = ~DAT_200002ec;
        }
        else {
          uVar14 = -DAT_200002ec;
        }
        DAT_200002f4 = DAT_20000220 + uVar14;
        if (DAT_20000206 <= DAT_200002f4) {
          DAT_20005fe2 = DAT_20005fe2 | 4;
        }
      }
      if (DAT_200001c9 == '\x01') {
        DAT_200002fc = DAT_20000220;
        if (DAT_20000220 < DAT_200002f8) {
          uVar14 = ~DAT_200002f8;
        }
        else {
          uVar14 = -DAT_200002f8;
        }
        DAT_20000300 = DAT_20000220 + uVar14;
        if (DAT_20000208 <= DAT_20000300) {
          DAT_20005f7e = DAT_20005f7e | 4;
        }
      }
      if (DAT_20000149 == '\x01') {
        DAT_200002fc = DAT_20000220;
        if (DAT_20000220 < DAT_200002f8) {
          uVar14 = ~DAT_200002f8;
        }
        else {
          uVar14 = -DAT_200002f8;
        }
        DAT_20000300 = DAT_20000220 + uVar14;
      }
      if ((DAT_2000048c != '\0') &&
         (DAT_20000518 = DAT_20000220 - DAT_200004e4, 4000 < DAT_20000518)) {
        DAT_2000048c = '\0';
      }
      if ((((0xcdfe6000 < DAT_20000220) && (DAT_20000220 < 0xcdfe6100)) && (DAT_20000146 == '\0'))
         || (DAT_200038cd == '\x05')) {
        DAT_20000146 = '\x01';
        DAT_200038cd = '\0';
      }
      iVar8 = FUN_08036f06(0);
      DAT_20000163 = (undefined1)iVar8;
      if (iVar8 == 1) {
        if (DAT_20000158 == '\x01') {
          DAT_2000015b = DAT_2000015b + 1;
        }
        if (9 < DAT_2000015b) goto LAB_08048f66;
      }
      else {
LAB_08048f66:
        DAT_2000015b = 0;
      }
      iVar8 = FUN_08036f06(1);
      DAT_20000164 = (undefined1)iVar8;
      if (iVar8 == 1) {
        if (DAT_20000158 == '\x01') {
          DAT_2000015c = DAT_2000015c + 1;
        }
        if (9 < DAT_2000015c) goto LAB_08048f88;
      }
      else {
LAB_08048f88:
        DAT_2000015c = 0;
      }
      iVar8 = FUN_08036f06(2);
      DAT_20000165 = (undefined1)iVar8;
      if (iVar8 == 1) {
        if (DAT_20000158 == '\x01') {
          DAT_2000015d = DAT_2000015d + 1;
        }
        if (9 < DAT_2000015d) goto LAB_08048faa;
      }
      else {
LAB_08048faa:
        DAT_2000015d = 0;
      }
      iVar8 = FUN_08036f06(3);
      DAT_20000166 = (undefined1)iVar8;
      if (iVar8 == 1) {
        if (DAT_20000158 == '\x01') {
          DAT_2000015e = DAT_2000015e + 1;
        }
        if (9 < DAT_2000015e) goto LAB_08048fcc;
      }
      else {
LAB_08048fcc:
        DAT_2000015e = 0;
      }
      iVar8 = FUN_08036f06(4);
      DAT_20000167 = (undefined1)iVar8;
      if (iVar8 == 1) {
        if (DAT_20000158 == '\x01') {
          DAT_2000015f = DAT_2000015f + 1;
        }
        if (9 < DAT_2000015f) goto LAB_08048fee;
      }
      else {
LAB_08048fee:
        DAT_2000015f = 0;
      }
      iVar8 = FUN_08036f06(5);
      DAT_20000168 = (undefined1)iVar8;
      if (iVar8 == 1) {
        if (DAT_20000158 == '\x01') {
          DAT_20000160 = DAT_20000160 + 1;
        }
        if (9 < DAT_20000160) goto LAB_08049016;
      }
      else {
LAB_08049016:
        DAT_20000160 = 0;
      }
      iVar8 = FUN_0801643a(6);
      DAT_20000169 = (undefined1)iVar8;
      if (iVar8 == 1) {
        if (DAT_20000158 == '\x01') {
          DAT_20000161 = DAT_20000161 + 1;
        }
        if (9 < DAT_20000161) goto LAB_08049040;
      }
      else {
LAB_08049040:
        DAT_20000161 = 0;
      }
      DAT_2000016a = FUN_0801643a(7);
      if (DAT_20000158 == '\x01') {
        DAT_20000162 = DAT_20000162 + 1;
      }
      if (9 < DAT_20000162) {
        DAT_20000162 = 0;
      }
      uVar14 = extraout_r3_00;
      puVar18 = extraout_r12;
      if (DAT_20000140 == '\x01') {
        FUN_0803f020();
        uVar14 = extraout_r3_01;
        puVar18 = extraout_r12_00;
      }

  /* ==========================================================================
   *   8.9 Built-in Modbus RTU Slave / Freeport Protocol Handler (Ports 0 & 1)
   * ========================================================================== */
      bVar5 = DAT_200054d1;
      cVar13 = DAT_200054cc;
      bVar26 = (byte)(uintptr_t)puVar18;
      bVar20 = (byte)uVar14;
      if (7 < DAT_2000014f) {
        DAT_200061c4 = (ushort)DAT_200060b0 * 0x100 + (ushort)DAT_200060b1;
        uVar10 = (uint)DAT_200061c4;
        puVar16 = &DAT_200054cc;
        if (DAT_200054cc != DAT_20005f80) {
          DAT_2000014f = 0;
          return 0;
        }
        if (DAT_200054cd == 5) {
          uVar14 = (uint)DAT_200054ce;
          bVar28 = uVar14 == 0;
LAB_08049490:
          bVar20 = (byte)uVar14;
          bVar30 = false;
          if (bVar28) {
            puVar18 = (undefined1 *)(uint)DAT_200054cf;
            bVar30 = (undefined1 *)0x7f < puVar18;
            goto LAB_08049496;
          }
LAB_080493c0:
          if (bVar30) {
            uVar19 = (uint)DAT_200054d1;
            bVar30 = 0x7b < uVar19;
            bVar28 = uVar19 == 0x7c;
            uVar15 = uVar19;
            if (0x7c < uVar19) goto LAB_08049318;
            bVar28 = true;
            if (uVar19 == 0) goto LAB_0804931e;
            puVar18 = (undefined1 *)(uint)DAT_200054cf;
            uVar14 = 0xffff;
            uVar15 = (uint)DAT_200054ce;
            puVar16 = (undefined1 *)((int)(puVar18 + uVar15 * 0x100 + uVar19) * 2 & 0xffff);
            bVar30 = (undefined1 *)0x27ff < puVar16;
            bVar28 = puVar16 == (undefined1 *)0x2800;
            if ((undefined1 *)0x2800 < puVar16) goto LAB_08049318;
            DAT_20000b9c = DAT_200054cc;
            DAT_20000b9d = 3;
            DAT_20000b9e = DAT_200054d1 << 1;
            for (uVar14 = 0; uVar14 < uVar19; uVar14 = uVar14 + 1 & 0xffff) {
              iVar8 = ((int)(puVar18 + uVar15 * 0x100) * 2 & 0xffffU) + uVar14 * 2 + uVar10;
              (&DAT_20000b9f)[uVar14 * 2] = (&DAT_2000108c)[iVar8];
              (&DAT_20000ba0)[uVar14 * 2] = (&DAT_2000108d)[iVar8];
            }
            uVar6 = FUN_08000e0e(&DAT_20000b9c,uVar19 * 2 + 3 & 0xff);
            uVar14 = (uint)DAT_200054d1;
            (&DAT_20000b9f)[uVar14 * 2] = (char)((ushort)uVar6 >> 8);
            (&DAT_20000ba0)[uVar14 * 2] = (char)uVar6;
          }
          else {
LAB_08049310:
            if (!bVar30) goto LAB_08049226;
            uVar15 = (uint)DAT_200054d1;
            bVar30 = 0x1f < uVar15;
            bVar28 = uVar15 == 0x20;
LAB_08049318:
            do {
              if (bVar30 && !bVar28) goto LAB_0804922e;
              bVar28 = DAT_200054d1 == 0;
LAB_0804931e:
              uVar10 = (uint)DAT_200054d1;
              bVar30 = true;
              uVar15 = uVar10;
              if (bVar28) goto LAB_08049234;
              uVar15 = (uint)DAT_200054cf;
              puVar16 = (undefined1 *)(uint)DAT_200054ce;
              uVar14 = 0xffff;
              uVar19 = (uVar15 + uVar10 + (int)puVar16 * 0x100) * 2 & 0xffff;
              bVar30 = 0x3f < uVar19;
              bVar28 = uVar19 == 0x40;
            } while (0x40 < uVar19);
            DAT_20000b9c = DAT_200054cc;
            DAT_20000b9d = 4;
            DAT_20000b9e = DAT_200054d1 << 1;
            in_stack_ffffffc0 = (undefined1 *)((uVar15 + (int)puVar16 * 0x100) * 2 & 0xffff);
            for (uVar14 = 0; uVar14 < uVar10; uVar14 = uVar14 + 1 & 0xffff) {
              (&DAT_20000b9f)[uVar14 * 2] = (&DAT_20005e1a)[(int)(in_stack_ffffffc0 + uVar14 * 2)];
              (&DAT_20000ba0)[uVar14 * 2] = (&DAT_20005e1b)[(int)(in_stack_ffffffc0 + uVar14 * 2)];
            }
            uVar6 = FUN_08000e0e(&DAT_20000b9c,uVar10 * 2 + 3 & 0xff);
LAB_080497fe:
            uVar14 = (uint)DAT_200054d1;
            (&DAT_20000b9f)[uVar14 * 2] = (char)((ushort)uVar6 >> 8);
            (&DAT_20000ba0)[uVar14 * 2] = (char)uVar6;
          }
          DAT_200000c4 = (undefined1 *)(uVar14 * 2 + 5);
LAB_0804964c:
          FUN_08036fb6(0);
          puVar18 = extraout_r12_01;
          bVar20 = extraout_r3;
LAB_08049652:
          bVar26 = (byte)(uintptr_t)puVar18;
        }
        else {
          if (5 < DAT_200054cd) {
            if (DAT_200054cd == 6) {
              puVar18 = (undefined1 *)
                        (((uint)DAT_200054cf + (uint)DAT_200054ce * 0x100) * 2 & 0xffff);
              bVar20 = 0xff;
              bVar28 = true;
              bVar26 = DAT_200054ce;
              if (puVar18 < (undefined1 *)0x27ff) {
                DAT_20000b9c = DAT_200054cc;
                DAT_20000b9d = 6;
                DAT_20000b9e = DAT_200054ce;
                DAT_20000b9f = DAT_200054cf;
                DAT_20000ba0 = DAT_200054d0;
                DAT_20000ba1 = DAT_200054d1;
                (&DAT_2000108c)[(int)(puVar18 + uVar10)] = DAT_200054d0;
                (&DAT_2000108d)[(int)(puVar18 + uVar10)] = bVar5;
                uVar6 = FUN_08000e0e(&DAT_20000b9c,6);
                DAT_20000ba2 = (undefined1)((ushort)uVar6 >> 8);
                DAT_20000ba3 = (undefined1)uVar6;
                DAT_200000c4 = (undefined1 *)0x8;
              }
              else {
LAB_08049522:
                bVar30 = true;
                if (bVar28) {
LAB_08049496:
                  if (bVar30) goto LAB_08049102;
                  DAT_20000b9c = DAT_200054cc;
                  DAT_20000b9d = 5;
                  DAT_20000b9f = (byte)(uintptr_t)puVar18;
                  DAT_20000ba0 = DAT_200054d0;
                  DAT_20000ba1 = DAT_200054d1;
                  bVar5 = (byte)(1 << ((uint)puVar18 & 7));
                  if (DAT_200054d0 == 0) {
                    bVar5 = (&DAT_20001046)[(uint)puVar18 >> 3] & ~bVar5;
                  }
                  else {
                    bVar5 = (&DAT_20001046)[(uint)puVar18 >> 3] | bVar5;
                  }
                  DAT_20000b9e = bVar20;
                  (&DAT_20001046)[(uint)puVar18 >> 3] = bVar5;
                  uVar9 = FUN_08000e0e(&DAT_20000b9c,6);
                  DAT_20000ba2 = (undefined1)((uint)uVar9 >> 8);
                }
                else {
                  puVar18 = puVar18 + uVar10;
                  bVar30 = true;
                  if ((undefined1 *)0x80 < puVar18) goto LAB_08049496;
                  for (uVar14 = 0; uVar14 < uVar10; uVar14 = uVar14 + 1 & 0xffff) {
                    uVar15 = ((uint)(in_stack_ffffffc0 + uVar14) & 0xff) >> 3;
                    bVar5 = (byte)(1 << ((uint)(in_stack_ffffffc0 + uVar14) & 7));
                    if (((uint)(&DAT_200054d3)[(uVar14 & 0x7ff) >> 3] & 1 << (uVar14 & 7)) == 0) {
                      bVar5 = (&DAT_20001046)[uVar15] & ~bVar5;
                    }
                    else {
                      bVar5 = (&DAT_20001046)[uVar15] | bVar5;
                    }
                    (&DAT_20001046)[uVar15] = bVar5;
                  }
                  DAT_20000b9c = cVar13;
                  DAT_20000b9d = 0xf;
                  DAT_20000b9e = (byte)(uintptr_t)in_stack_ffffffc8;
                  DAT_20000b9f = (byte)(uintptr_t)in_stack_ffffffc0;
                  DAT_20000ba0 = (byte)(uintptr_t)in_stack_ffffffcc;
                  DAT_20000ba1 = (byte)uVar10;
                  uVar9 = FUN_08000e0e(&DAT_20000b9c,6);
                  DAT_20000ba2 = (undefined1)((uint)uVar9 >> 8);
                }
                DAT_20000ba3 = (undefined1)uVar9;
LAB_080495be:
                DAT_200000c4 = (undefined1 *)0x8;
              }
              goto LAB_0804964c;
            }
            if (DAT_200054cd == 0xf) {
              in_stack_ffffffcc = (undefined1 *)(uint)DAT_200054d0;
              bVar28 = false;
              if (in_stack_ffffffcc == (undefined1 *)0x0) {
                uVar10 = (uint)DAT_200054d1;
                if ((0x80 < uVar10) || (uVar10 == 0)) goto LAB_08049102;
                in_stack_ffffffc8 = (undefined1 *)(uint)DAT_200054ce;
                bVar28 = false;
                if (in_stack_ffffffc8 == (undefined1 *)0x0) {
                  puVar18 = (undefined1 *)(uint)DAT_200054cf;
                  bVar28 = (undefined1 *)0x7f < puVar18;
                  in_stack_ffffffc0 = puVar18;
                  goto LAB_08049522;
                }
              }
              goto LAB_08049490;
            }
            if (DAT_200054cd != 0x10) {
              if (DAT_200054cd != 0x17) goto LAB_08049102;
              if (((DAT_200054d0 != 0) ||
                  (in_stack_ffffffc0 = (undefined1 *)(uint)DAT_200054d1,
                  (undefined1 *)0x7c < in_stack_ffffffc0)) ||
                 (in_stack_ffffffc0 == (undefined1 *)0x0)) goto LAB_08049652;
              in_stack_ffffffcc = (undefined1 *)(uint)DAT_200054cf;
              in_stack_ffffffc8 = (undefined1 *)(uint)DAT_200054ce;
              if (0x2800 < ((int)(in_stack_ffffffc0 + (int)in_stack_ffffffcc +
                                 (int)in_stack_ffffffc8 * 0x100) * 2 & 0xffffU)) goto LAB_08049652;
              DAT_20000b9c = DAT_200054cc;
              DAT_20000b9d = 0x17;
              DAT_20000b9e = DAT_200054d1 << 1;
              uVar14 = (uint)DAT_200054d3;
              uVar19 = (uint)DAT_200054d2;
              uVar21 = (uint)DAT_200054d6;
              for (uVar15 = 0; uVar15 < uVar21; uVar15 = uVar15 + 1 & 0xffff) {
                (&DAT_2000108c)[uVar10 + ((uVar14 + uVar19 * 0x100) * 2 & 0xffff) + uVar15] =
                     (&DAT_200054d7)[uVar15];
              }
              DAT_40003000 = 0xaaaa;
              for (puVar18 = (undefined1 *)0x0; puVar18 < in_stack_ffffffc0;
                  puVar18 = (undefined1 *)((uint)(puVar18 + 1) & 0xffff)) {
                iVar8 = ((int)in_stack_ffffffc8 * 0x100 + (int)in_stack_ffffffcc * 2 & 0xffffU) +
                        (int)puVar18 * 2;
                (&DAT_20000b9f)[(int)puVar18 * 2] = (&DAT_2000108c)[iVar8];
                (&DAT_20000ba0)[(int)puVar18 * 2] = (&DAT_2000108d)[iVar8];
              }
              uVar6 = FUN_08000e0e(&DAT_20000b9c,(int)in_stack_ffffffc0 * 2 + 3U & 0xff);
              goto LAB_080497fe;
            }
            bVar20 = DAT_200054d0;
            if (((DAT_200054d0 == 0) &&
                (in_stack_ffffffc0 = (undefined1 *)(uint)DAT_200054d1,
                in_stack_ffffffc0 < (undefined1 *)0x7d)) &&
               ((in_stack_ffffffc0 != (undefined1 *)0x0 &&
                ((uint)DAT_200054d2 == (int)in_stack_ffffffc0 * 2)))) {
              uVar15 = (uint)DAT_200054cf;
              uVar14 = (uint)DAT_200054ce;
              puVar18 = in_stack_ffffffc0 + uVar14 * 0x100 + uVar15;
              if (0x2800 < ((int)puVar18 * 2 & 0xffffU)) goto LAB_08049652;
              DAT_20000b9c = DAT_200054cc;
              DAT_20000b9d = 0x10;
              DAT_20000b9e = DAT_200054ce;
              DAT_20000b9f = DAT_200054cf;
              DAT_20000ba0 = DAT_200054d0;
              DAT_20000ba1 = DAT_200054d1;
              for (uVar19 = 0; uVar19 < (uint)((int)in_stack_ffffffc0 << 1);
                  uVar19 = uVar19 + 1 & 0xffff) {
                (&DAT_2000108c)[((uVar15 + uVar14 * 0x100) * 2 & 0xffff) + uVar19 + uVar10] =
                     (&DAT_200054d3)[uVar19];
              }
              uVar6 = FUN_08000e0e(&DAT_20000b9c,6);
              DAT_20000ba2 = (undefined1)((ushort)uVar6 >> 8);
              DAT_20000ba3 = (undefined1)uVar6;
              goto LAB_080495be;
            }
            goto LAB_08049652;
          }
          if (DAT_200054cd == 1) {
            if (((DAT_200054d0 == 0) && (uVar15 = (uint)DAT_200054d1, uVar15 < 0x81)) &&
               ((uVar15 != 0 && (DAT_200054ce == 0)))) {
              uVar14 = (uint)DAT_200054cf;
              bVar28 = 0x7f < uVar14;
LAB_08049162:
              if ((!bVar28) && (uVar14 + uVar15 < 0x81)) {
                in_stack_ffffffc0 = (undefined1 *)(uVar15 >> 3);
                if ((uVar15 & 7) != 0) {
                  in_stack_ffffffc0 = in_stack_ffffffc0 + 1;
                }
                uVar14 = 0;
                for (puVar18 = (undefined1 *)0x0; puVar18 < in_stack_ffffffc0;
                    puVar18 = (undefined1 *)((uint)(puVar18 + 1) & 0xffff)) {
                  (&DAT_20000b9f)[(int)puVar18] = (&DAT_20001046)[uVar14];
                  uVar14 = uVar14 + 1 & 0xff;
                }
                DAT_20000b9c = cVar13;
                DAT_20000b9d = 1;
                DAT_20000b9e = (byte)(uintptr_t)in_stack_ffffffc0;
                uVar9 = FUN_08000e0e(&DAT_20000b9c,(uintptr_t)(in_stack_ffffffc0 + 3));
                (&DAT_20000b9f)[(int)in_stack_ffffffc0] = (char)((uint)uVar9 >> 8);
LAB_080492fa:
                (&DAT_20000ba0)[(int)in_stack_ffffffc0] = (char)uVar9;
                DAT_200000c4 = (undefined1 *)(in_stack_ffffffc0 + 5);
                goto LAB_0804964c;
              }
            }
          }
          else {
            if (DAT_200054cd == 2) {
              bVar30 = DAT_200054d0 == 0;
              goto LAB_08049226;
            }
            if (DAT_200054cd == 3) {
              bVar30 = DAT_200054d0 == 0;
              goto LAB_080493c0;
            }
            if (DAT_200054cd == 4) {
              bVar30 = DAT_200054d0 == 0;
              goto LAB_08049310;
            }
          }
        }
LAB_08049102:
        DAT_2000014f = 0;
      }
      bVar4 = DAT_200053d1;
      bVar3 = DAT_200053d0;
      bVar2 = DAT_200053cf;
      bVar5 = DAT_200053ce;
      cVar13 = DAT_200053cc;
    } while (DAT_20000150 < 8);
    DAT_200061c6 = (ushort)DAT_200060b4 * 0x100 + (ushort)DAT_200060b5;
    in_stack_ffffffc0 = (undefined1 *)(uint)DAT_200061c6;
    puVar18 = &DAT_200053cc;
    if (DAT_200053cc != DAT_20005fe4) {
      DAT_20000150 = 0;
      return 0;
    }
    if (DAT_200053cd == 5) {
      bVar28 = DAT_200053ce == 0;
      puVar16 = (undefined1 *)0x5;
      bVar20 = DAT_200053ce;
      goto LAB_08049b60;
    }
    if (DAT_200053cd < 6) {
      if (DAT_200053cd == 1) {
        if ((((DAT_200053d0 != 0) || (uVar14 = (uint)DAT_200053d1, 0x80 < uVar14)) || (uVar14 == 0))
           || (((DAT_200053ce != 0 || (0x7f < DAT_200053cf)) || (0x80 < DAT_200053cf + uVar14))))
        goto LAB_08049854;
        puVar18 = (undefined1 *)(uint)(DAT_200053d1 >> 3);
        if ((DAT_200053d1 & 7) != 0) {
          puVar18 = puVar18 + 1;
        }
        uVar14 = 0;
        for (puVar16 = (undefined1 *)0x0; puVar16 < puVar18;
            puVar16 = (undefined1 *)((uint)(puVar16 + 1) & 0xffff)) {
          (&DAT_200055cf)[(int)puVar16] = (&DAT_20001046)[uVar14];
          uVar14 = uVar14 + 1 & 0xff;
        }
        DAT_200055cc = cVar13;
        DAT_200055cd = 1;
        DAT_200055ce = (byte)(uintptr_t)puVar18;
        uVar6 = FUN_08000e0e(&DAT_200055cc,(uintptr_t)(puVar18 + 3));
        (&DAT_200055cf)[(int)puVar18] = (char)((ushort)uVar6 >> 8);
        (&DAT_200055d0)[(int)puVar18] = (char)uVar6;
        in_stack_ffffffc0 = puVar18;
LAB_08049a00:
        DAT_200000bc = (undefined1 *)(puVar18 + 5);
        goto LAB_08049ce0;
      }
      if (DAT_200053cd != 2) {
        if (DAT_200053cd == 3) {
          puVar16 = (undefined1 *)(uint)DAT_200053d0;
          bVar28 = puVar16 == (undefined1 *)0x0;
LAB_08049a94:
          if (!bVar28) goto LAB_08049a08;
          uVar14 = (uint)DAT_200053d1;
          bVar30 = 0x7b < uVar14;
          bVar28 = uVar14 == 0x7c;
          if (0x7c < uVar14) goto LAB_08049a0e;
          bVar28 = true;
          if (uVar14 == 0) goto LAB_08049a12;
          puVar16 = (undefined1 *)(uint)DAT_200053cf;
          uVar10 = (uint)DAT_200053ce;
          puVar18 = (undefined1 *)((int)(puVar16 + uVar10 * 0x100 + uVar14) * 2 & 0xffff);
          bVar30 = (undefined1 *)0x27ff < puVar18;
          bVar28 = puVar18 == (undefined1 *)0x2800;
          if ((undefined1 *)0x2800 < puVar18) goto LAB_08049a0e;
          DAT_200055cc = DAT_200053cc;
          DAT_200055cd = 3;
          DAT_200055ce = DAT_200053d1 << 1;
          for (uVar15 = 0; uVar15 < uVar14; uVar15 = uVar15 + 1 & 0xffff) {
            (&DAT_200055cf)[uVar15 * 2] =
                 (&DAT_2000108c)
                 [(int)(in_stack_ffffffc0 +
                       ((int)(puVar16 + uVar10 * 0x100) * 2 & 0xffffU) + uVar15 * 2)];
            (&DAT_200055d0)[uVar15 * 2] =
                 (&DAT_2000108d)
                 [(int)(in_stack_ffffffc0 +
                       ((int)(puVar16 + uVar10 * 0x100) * 2 & 0xffffU) + uVar15 * 2)];
          }
        }
        else {
          if (DAT_200053cd != 4) goto LAB_08049854;
          puVar16 = (undefined1 *)(uint)DAT_200053d0;
          bVar28 = puVar16 == (undefined1 *)0x0;
LAB_08049a08:
          if (!bVar28) goto LAB_08049926;
          bVar30 = 0x1f < DAT_200053d1;
          bVar28 = DAT_200053d1 == 0x20;
LAB_08049a0e:
          do {
            if (bVar30 && !bVar28) goto LAB_0804992c;
            bVar28 = DAT_200053d1 == 0;
LAB_08049a12:
            uVar14 = (uint)DAT_200053d1;
            bVar30 = true;
            if (bVar28) goto LAB_08049930;
            puVar16 = (undefined1 *)(uint)(byte)(uintptr_t)puVar18[3];
            puVar18 = (undefined1 *)(uint)(byte)(uintptr_t)puVar18[2];
            uVar10 = (int)(puVar16 + (int)puVar18 * 0x100 + uVar14) * 2 & 0xffff;
            bVar30 = 0x3f < uVar10;
            bVar28 = uVar10 == 0x40;
          } while (0x40 < uVar10);
          DAT_200055cc = DAT_200053cc;
          DAT_200055cd = 4;
          DAT_200055ce = DAT_200053d1 << 1;
          in_stack_ffffffc0 = (undefined1 *)((int)(puVar16 + (int)puVar18 * 0x100) * 2 & 0xffff);
          for (uVar10 = 0; uVar10 < uVar14; uVar10 = uVar10 + 1 & 0xffff) {
            (&DAT_200055cf)[uVar10 * 2] = (&DAT_20005e1a)[(int)(in_stack_ffffffc0 + uVar10 * 2)];
            (&DAT_200055d0)[uVar10 * 2] = (&DAT_20005e1b)[(int)(in_stack_ffffffc0 + uVar10 * 2)];
          }
        }
        uVar6 = FUN_08000e0e(&DAT_200055cc,bVar4 * '\x02' + '\x03');
        uVar14 = (uint)DAT_200053d1;
        (&DAT_200055cf)[uVar14 * 2] = (char)((ushort)uVar6 >> 8);
        (&DAT_200055d0)[uVar14 * 2] = (char)uVar6;
LAB_08049b50:
        DAT_200000bc = (undefined1 *)(uVar14 * 2 + 5);
        goto LAB_08049ce0;
      }
      bVar28 = DAT_200053d0 == 0;
LAB_08049926:
      if (bVar28) {
        puVar16 = (undefined1 *)(uint)DAT_200053d1;
        bVar30 = (undefined1 *)0x7f < puVar16;
        bVar28 = puVar16 == (undefined1 *)0x80;
LAB_0804992c:
        if (!bVar30 || bVar28) {
          bVar30 = puVar16 == (undefined1 *)0x0;
LAB_08049930:
          if (((!bVar30) && (puVar18[2] == '\0')) &&
             (((byte)(uintptr_t)puVar18[3] < 0x80 && (puVar16 + (byte)(uintptr_t)puVar18[3] < (undefined1 *)0x81)))) {
            puVar18 = (undefined1 *)((uint)puVar16 >> 3);
            if (((uint)puVar16 & 7) != 0) {
              puVar18 = puVar18 + 1;
            }
            uVar14 = 0;
            for (puVar16 = (undefined1 *)0x0; puVar16 < puVar18;
                puVar16 = (undefined1 *)((uint)(puVar16 + 1) & 0xffff)) {
              (&DAT_200055cf)[(int)puVar16] = (&DAT_20005bd0)[uVar14];
              uVar14 = uVar14 + 1 & 0xff;
            }
            DAT_200055cc = cVar13;
            DAT_200055cd = 2;
            DAT_200055ce = (byte)(uintptr_t)puVar18;
            uVar6 = FUN_08000e0e(&DAT_200055cc,(uintptr_t)(puVar18 + 3));
            (&DAT_200055cf)[(int)puVar18] = (char)((ushort)uVar6 >> 8);
            (&DAT_200055d0)[(int)puVar18] = (char)uVar6;
            goto LAB_08049a00;
          }
        }
      }
    }
    else {
      if (DAT_200053cd == 6) {
        uVar14 = ((uint)DAT_200053cf + (uint)DAT_200053ce * 0x100) * 2 & 0xffff;
        if (0x27fe < uVar14) goto LAB_08049854;
        DAT_200055cc = DAT_200053cc;
        DAT_200055cd = 6;
        DAT_200055ce = DAT_200053ce;
        DAT_200055cf = DAT_200053cf;
        DAT_200055d0 = DAT_200053d0;
        DAT_200055d1 = DAT_200053d1;
        (&DAT_2000108c)[(int)(in_stack_ffffffc0 + uVar14)] = DAT_200053d0;
        (&DAT_2000108d)[(int)(in_stack_ffffffc0 + uVar14)] = bVar4;
        uVar6 = FUN_08000e0e(&DAT_200055cc,6);
        DAT_200055d2 = (undefined1)((ushort)uVar6 >> 8);
        DAT_200055d3 = (undefined1)uVar6;
      }
      else {
        if (DAT_200053cd != 0xf) {
          bVar5 = DAT_200053d0;
          if (DAT_200053cd == 0x10) {
LAB_08049d5a:
            bVar26 = bVar5;
            if (DAT_200053d0 != 0) goto LAB_08049854;
            uVar14 = (uint)DAT_200053d1;
            bVar30 = 0x7b < uVar14;
            bVar28 = uVar14 == 0x7c;
LAB_08049d60:
            if (bVar30 && !bVar28) goto LAB_08049854;
            bVar28 = uVar14 == 0;
LAB_08049d64:
            if (!bVar28) {
              if ((uint)DAT_200053d2 != uVar14 * 2) {
                return 1;
              }
              uVar10 = (uint)DAT_200053cf;
              uVar15 = (uint)DAT_200053ce;
              if (((uVar10 + uVar14 + uVar15 * 0x100) * 2 & 0xffff) < 0x2801) {
                DAT_200055cc = DAT_200053cc;
                DAT_200055cd = 0x10;
                DAT_200055ce = DAT_200053ce;
                DAT_200055cf = DAT_200053cf;
                DAT_200055d1 = (byte)uVar14;
                DAT_200055d0 = bVar26;
                for (uVar19 = 0; uVar19 < uVar14 << 1; uVar19 = uVar19 + 1 & 0xffff) {
                  (&DAT_2000108c)[(int)(in_stack_ffffffc0 + uVar10 + uVar15 * 0x100 + uVar19)] =
                       (&DAT_200054d3)[uVar19];
                }
                uVar6 = FUN_08000e0e(&DAT_200055cc,6);
                DAT_200055d2 = (undefined1)((ushort)uVar6 >> 8);
                DAT_200055d3 = (undefined1)uVar6;
                goto LAB_08049cd4;
              }
            }
            goto LAB_08049854;
          }
          if (DAT_200053cd != 0x17) goto LAB_08049854;
          bVar5 = bVar26;
          if (DAT_200053d0 != 0) goto LAB_08049d5a;
          uVar10 = (uint)DAT_200053d1;
          bVar30 = 0x7b < uVar10;
          bVar28 = uVar10 == 0x7c;
          uVar14 = uVar10;
          if (0x7c < uVar10) goto LAB_08049d60;
          bVar28 = true;
          if (uVar10 == 0) goto LAB_08049d64;
          uVar15 = (uint)DAT_200053cf;
          in_stack_ffffffc8 = (undefined1 *)(uint)DAT_200053ce;
          uVar14 = (uVar15 + uVar10 + (int)in_stack_ffffffc8 * 0x100) * 2 & 0xffff;
          bVar30 = 0x27ff < uVar14;
          bVar28 = uVar14 == 0x2800;
          if (0x2800 < uVar14) goto LAB_08049d60;
          DAT_200055cc = DAT_200053cc;
          DAT_200055cd = 0x17;
          DAT_200055ce = DAT_200053d1 << 1;
          uVar14 = (uint)DAT_200053d3;
          uVar21 = (uint)DAT_200053d2;
          uVar27 = (uint)DAT_200053d6;
          for (uVar19 = 0; uVar19 < uVar27 << 1; uVar19 = uVar19 + 1 & 0xffff) {
            (&DAT_2000108c)
            [(int)(in_stack_ffffffc0 + ((uVar14 + uVar21 * 0x100) * 2 & 0xffff) + uVar19)] =
                 (&DAT_200053d7)[uVar19];
          }
          DAT_40003000 = 0xaaaa;
          for (uVar14 = 0; uVar14 < uVar10 << 1; uVar14 = uVar14 + 1 & 0xffff) {
            (&DAT_200055cf)[uVar14 * 2] =
                 (&DAT_2000108c)
                 [(int)(in_stack_ffffffc0 +
                       ((uVar15 + (int)in_stack_ffffffc8 * 0x100) * 2 & 0xffff) + uVar14 * 2)];
            (&DAT_200055d0)[uVar14 * 2] =
                 (&DAT_2000108d)
                 [(int)(in_stack_ffffffc0 +
                       ((uVar15 + (int)in_stack_ffffffc8 * 0x100) * 2 & 0xffff) + uVar14 * 2)];
          }
          uVar6 = FUN_08000e0e(&DAT_200055cc,uVar10 * 2 + 3 & 0xff);
          uVar14 = (uint)DAT_200053d1;
          (&DAT_200055cf)[uVar14 * 2] = (char)((ushort)uVar6 >> 8);
          (&DAT_200055d0)[uVar14 * 2] = (char)uVar6;
          goto LAB_08049b50;
        }
        in_stack_ffffffc8 = (undefined1 *)(uint)DAT_200053d0;
        bVar28 = in_stack_ffffffc8 == (undefined1 *)0x0;
        puVar16 = in_stack_ffffffc8;
        if (!bVar28) {
LAB_08049b60:
          if (!bVar28) goto LAB_08049a94;
          puVar18 = (undefined1 *)(uint)DAT_200053cf;
          bVar28 = (undefined1 *)0x7f < puVar18;
          puVar16 = in_stack_ffffffcc;
          goto LAB_08049b66;
        }
        uVar14 = (uint)DAT_200053d1;
        if ((0x80 < uVar14) || (uVar14 == 0)) goto LAB_08049854;
        puVar16 = (undefined1 *)(uint)DAT_200053ce;
        bVar28 = puVar16 == (undefined1 *)0x0;
        in_stack_ffffffcc = puVar16;
        if (!bVar28) goto LAB_08049b60;
        in_stack_ffffffc0 = (undefined1 *)(uint)DAT_200053cf;
        bVar28 = true;
        puVar18 = in_stack_ffffffc0;
        if (in_stack_ffffffc0 < (undefined1 *)0x80) {
          puVar18 = in_stack_ffffffc0 + uVar14;
          bVar28 = true;
          if ((undefined1 *)0x80 < puVar18) goto LAB_08049b66;
          for (uVar10 = 0; uVar10 < uVar14; uVar10 = uVar10 + 1 & 0xffff) {
            uVar15 = ((uint)(in_stack_ffffffc0 + uVar10) & 0xff) >> 3;
            uVar19 = (uint)(in_stack_ffffffc0 + uVar10) & 7;
            if (((uint)(&DAT_200053d3)[(uVar10 & 0x7ff) >> 3] & 1 << (uVar10 & 7)) == 0) {
              (&DAT_20001046)[uVar15] = (&DAT_20001046)[uVar15] & ~(byte)(1 << uVar19);
            }
            else {
              (&DAT_20001046)[uVar15] = (&DAT_20001046)[uVar15] | (byte)(1 << uVar19);
            }
          }
          DAT_200055cc = cVar13;
          DAT_200055cd = 0xf;
          DAT_200055ce = bVar5;
          DAT_200055cf = bVar2;
          DAT_200055d0 = bVar3;
          DAT_200055d1 = bVar4;
          (&DAT_2000108c)[(int)in_stack_ffffffc0 * 2] = bVar3;
          (&DAT_2000108d)[(int)in_stack_ffffffc0 * 2] = bVar4;
          uVar9 = FUN_08000e0e(&DAT_200055cc,6);
          DAT_200055d2 = (undefined1)((uint)uVar9 >> 8);
        }
        else {
LAB_08049b66:
          in_stack_ffffffcc = puVar16;
          if (bVar28) goto LAB_08049854;
          DAT_200055cc = DAT_200053cc;
          DAT_200055cd = 5;
          DAT_200055cf = (byte)(uintptr_t)puVar18;
          DAT_200055d0 = DAT_200053d0;
          DAT_200055d1 = DAT_200053d1;
          uVar14 = (uint)puVar18 >> 3;
          DAT_200055ce = bVar20;
          if (DAT_200053d0 == 0) {
            (&DAT_20001046)[uVar14] = (&DAT_20001046)[uVar14] & ~(byte)(1 << ((uint)puVar18 & 7));
          }
          else {
            (&DAT_20001046)[uVar14] = (&DAT_20001046)[uVar14] | (byte)(1 << ((uint)puVar18 & 7));
          }
          uVar9 = FUN_08000e0e(&DAT_200055cc,6);
          DAT_200055d2 = (undefined1)((uint)uVar9 >> 8);
        }
        DAT_200055d3 = (undefined1)uVar9;
        in_stack_ffffffcc = puVar16;
      }
LAB_08049cd4:
      DAT_200000bc = (undefined1 *)0x8;
LAB_08049ce0:
      FUN_08036fb6(1);
    }
LAB_08049854:
    DAT_20000150 = 0;

  /* ==========================================================================
   *   8.10 Expansion Module Output Refresh, Force Masks & RUN/STOP/SF LED Control
   * ========================================================================== */
  } while( true );
LAB_08046438:
  DAT_2000019b = pbVar17[0x3c] << 5 | 0xc;
  uVar14 = FUN_08013346(DAT_2000019b,0x18);
  *(short *)(pbVar17 + 0x3e) = (short)uVar14;
  uVar10 = (uVar14 & 0x3ff) >> 2;
  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
  uVar14 = uVar14 & 3;
  DAT_2000019e = (byte)uVar14;
  bVar28 = false;
  if ((byte)(&DAT_0804a2a2)[uVar10] == uVar14) {
    (&DAT_2000108d)[*(ushort *)(pbVar17 + 0x42)] = DAT_2000019d;
    DAT_200001a9 = 0x11;
    DAT_200001aa = 4;
    goto LAB_0804665e;
  }
  goto LAB_08046398;
code_r0x08046350:
  DAT_200001ac = (byte)(uVar32 >> 0x20);
  DAT_200001a9 = 0x11;
  DAT_200001aa = 2;
LAB_0804665e:
  pbVar23 = pbVar17;
  if (DAT_200001a4 == 9) {
    uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1211;
    *(ushort *)(pbVar17 + 0x40) = uVar7;
    iVar8 = FUN_08012aac(uVar7,0x19);
    *(short *)(pbVar17 + 0x3e) = (short)iVar8;
    if (iVar8 == 1) goto LAB_080466a6;
    bVar28 = iVar8 == 5;
    goto LAB_080466a4;
  }
  pbVar23 = (byte *)&DAT_20000140;
  if (DAT_200001a4 == 0x13) {
    uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1212;
    *(ushort *)(pbVar17 + 0x40) = uVar7;
    iVar8 = FUN_08012aac(uVar7,0x19);
    *(short *)(pbVar17 + 0x3e) = (short)iVar8;
    pbVar23 = pbVar17;
    if (iVar8 == 1) goto LAB_080467ac;
    bVar28 = iVar8 == 5;
    goto LAB_080467aa;
  }
  if ((DAT_200001a6 == '\0') && (9 < DAT_200001a4)) {
    if (DAT_200001ac == 2) {
      if (6 < DAT_200001aa) {
        DAT_200001a9 = DAT_200001a9 + 1;
      }
      uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1200 | (ushort)DAT_200001a9;
      *(ushort *)(pbVar17 + 0x40) = uVar7;
      iVar8 = FUN_08012aac(uVar7,0x19);
      *(short *)(pbVar17 + 0x3e) = (short)iVar8;
      pbVar22 = pbVar23;
      if (iVar8 == 1) goto LAB_0804686a;
      bVar28 = iVar8 == 5;
      goto LAB_08046868;
    }
    if (6 < DAT_200001aa) {
      DAT_200001a9 = DAT_200001a9 + 1;
    }
    uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1200 | (ushort)DAT_200001a9;
    *(ushort *)(pbVar17 + 0x40) = uVar7;
    iVar8 = FUN_08012aac(uVar7,0x19);
    *(short *)(pbVar17 + 0x3e) = (short)iVar8;
    pbVar22 = pbVar23;
    if (iVar8 == 1) goto LAB_08046830;
    bVar28 = iVar8 == 5;
LAB_0804682e:
    pbVar22 = pbVar23;
    if (!bVar28) goto LAB_08046754;
LAB_08046830:
    pbVar23 = pbVar22 + 0x6a;
    bVar5 = *pbVar23 + 1 & 7;
    *pbVar23 = bVar5;
    uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1800 | (ushort)bVar5 << 8 | pbVar22[0x6c] + 1;
    *(ushort *)(pbVar17 + 0x40) = uVar7;
    iVar8 = FUN_08012aac(uVar7,0x19);
    *(short *)(pbVar17 + 0x3e) = (short)iVar8;
    if ((iVar8 != 1) && (bVar28 = iVar8 == 5, !bVar28)) goto LAB_080469c2;
    if (6 < DAT_200001aa) {
      DAT_200001a9 = DAT_200001a9 + 1;
      pbVar23 = pbVar17 + 0x3c;
      uVar7 = (ushort)*pbVar23 << 0xd | 0x1200 | (ushort)DAT_200001a9;
      *(ushort *)(pbVar17 + 0x40) = uVar7;
      iVar8 = FUN_08012aac(uVar7,0x19);
      *(short *)(pbVar17 + 0x3e) = (short)iVar8;
      if ((iVar8 != 1) && (bVar30 = iVar8 == 5, !bVar30)) goto LAB_080469fc;
    }
    DAT_200001aa = DAT_200001aa + 1 & 7;
    uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1800 | (ushort)DAT_200001aa << 8;
    *(ushort *)(pbVar17 + 0x40) = uVar7;
    iVar8 = FUN_08012aac(uVar7,0x19);
    *(short *)(pbVar17 + 0x3e) = (short)iVar8;
    if ((iVar8 != 1) && (bVar28 = iVar8 == 5, pbVar23 = pbVar17, !bVar28)) {
LAB_08046a4a:
      if (!bVar28) {
        do {
          if (!bVar28) goto LAB_080468a2;
          do {
            do {
              pbVar23 = (byte *)&DAT_20000140;
              DAT_200001aa = DAT_200001aa + 1 & 7;
              uVar7 = (ushort)pbVar17[0x3c] << 0xd | (ushort)DAT_200001aa << 8 |
                      (byte)(&DAT_2000108c)[(uint)DAT_200001ab + (uint)*(ushort *)(pbVar17 + 0x42)]
                      | 0x1800;
              *(ushort *)(pbVar17 + 0x40) = uVar7;
              iVar8 = FUN_08012aac(uVar7,0x19);
              *(short *)(pbVar17 + 0x3e) = (short)iVar8;
              if (iVar8 != 1) {
                bVar28 = iVar8 == 5;
LAB_080469c2:
                if (!bVar28) goto LAB_080468ec;
              }
              if (pbVar23[0x6a] < 7) {
LAB_080469fe:
                pbVar22 = pbVar23 + 0x6a;
                bVar5 = *pbVar22 + 1 & 7;
                *pbVar22 = bVar5;
                uVar7 = (ushort)pbVar17[0x3c] << 0xd | (ushort)bVar5 << 8 |
                        (byte)(&DAT_2000108d)
                              [(uint)pbVar23[0x6b] + (uint)*(ushort *)(pbVar17 + 0x42)] | 0x1800;
                *(ushort *)(pbVar17 + 0x40) = uVar7;
                iVar8 = FUN_08012aac(uVar7,0x19);
                *(short *)(pbVar17 + 0x3e) = (short)iVar8;
                pbVar23 = pbVar22;
                goto LAB_08046a44;
              }
              bVar5 = pbVar23[0x69];
              pbVar23[0x69] = bVar5 + 1;
              uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1200 | (ushort)(byte)(bVar5 + 1);
              *(ushort *)(pbVar17 + 0x40) = uVar7;
              iVar8 = FUN_08012aac(uVar7,0x19);
              *(short *)(pbVar17 + 0x3e) = (short)iVar8;
              if (iVar8 == 1) goto LAB_080469fe;
              bVar30 = iVar8 == 5;
LAB_080469fc:
              bVar28 = false;
              if (bVar30) goto LAB_080469fe;
              while (!bVar28) {
LAB_08046868:
                pbVar22 = pbVar23;
                if (!bVar28) goto LAB_080467aa;
LAB_0804686a:
                pbVar23 = pbVar22 + 0x6a;
                bVar5 = *pbVar23 + 1 & 7;
                *pbVar23 = bVar5;
                uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1800 | (ushort)bVar5 << 8 |
                        pbVar22[0x6c] + 1;
                *(ushort *)(pbVar17 + 0x40) = uVar7;
                iVar8 = FUN_08012aac(uVar7,0x19);
                *(short *)(pbVar17 + 0x3e) = (short)iVar8;
                if (iVar8 != 1) {
                  bVar28 = iVar8 == 5;
LAB_080468a2:
                  if (!bVar28) goto LAB_080467cc;
                }
                if (6 < DAT_200001aa) {
                  DAT_200001a9 = DAT_200001a9 + 1;
                  pbVar23 = pbVar17 + 0x3c;
                  uVar7 = (ushort)*pbVar23 << 0xd | 0x1200 | (ushort)DAT_200001a9;
                  *(ushort *)(pbVar17 + 0x40) = uVar7;
                  iVar8 = FUN_08012aac(uVar7,0x19);
                  *(short *)(pbVar17 + 0x3e) = (short)iVar8;
                  if (iVar8 != 1) {
                    bVar28 = iVar8 == 5;
LAB_080468ec:
                    if (!bVar28) goto LAB_0804682e;
                  }
                }
                DAT_200001aa = DAT_200001aa + 1 & 7;
                pbVar23 = pbVar17 + 0x3c;
                uVar7 = (ushort)*pbVar23 << 0xd | 0x1800 | (ushort)DAT_200001aa << 8;
                *(ushort *)(pbVar17 + 0x40) = uVar7;
                iVar8 = FUN_08012aac(uVar7,0x19);
                *(short *)(pbVar17 + 0x3e) = (short)iVar8;
                if (iVar8 == 1) break;
                bVar28 = iVar8 == 5;
              }
            } while (DAT_200001aa < 7);
            DAT_200001a9 = DAT_200001a9 + 1;
            pbVar23 = pbVar17 + 0x3c;
            uVar7 = (ushort)*pbVar23 << 0xd | 0x1200 | (ushort)DAT_200001a9;
            *(ushort *)(pbVar17 + 0x40) = uVar7;
            iVar8 = FUN_08012aac(uVar7,0x19);
            *(short *)(pbVar17 + 0x3e) = (short)iVar8;
          } while (iVar8 == 1);
          bVar28 = iVar8 == 5;
        } while( true );
      }
      goto LAB_08046dbc;
    }
    DAT_200001a7 = DAT_200001ac >> 2;
    for (DAT_200001a8 = 0; DAT_200001a8 < DAT_200001a7; DAT_200001a8 = DAT_200001a8 + 1) {
      if (DAT_200001aa < 7) {
LAB_08046b68:
        DAT_200001aa = DAT_200001aa + 1 & 7;
        pbVar23 = pbVar17 + 0x3c;
        uVar7 = (byte)(&DAT_2000108c)
                      [(uint)*(ushort *)(pbVar17 + 0x42) + (uint)DAT_200001ab +
                       (uint)DAT_200001a8 * 4] | 0x1800 |
                (ushort)*pbVar23 << 0xd | (ushort)DAT_200001aa << 8;
        *(ushort *)(pbVar17 + 0x40) = uVar7;
        iVar8 = FUN_08012aac(uVar7,0x19);
        *(short *)(pbVar17 + 0x3e) = (short)iVar8;
        if ((iVar8 == 1) || (iVar8 == 5)) {
          if (6 < DAT_200001aa) {
            DAT_200001a9 = DAT_200001a9 + 1;
            uVar7 = (ushort)*pbVar23 << 0xd | 0x1200 | (ushort)DAT_200001a9;
            *(ushort *)(pbVar17 + 0x40) = uVar7;
            iVar8 = FUN_08012aac(uVar7,0x19);
            *(short *)(pbVar17 + 0x3e) = (short)iVar8;
            if ((iVar8 != 1) && (iVar8 != 5)) goto LAB_08046d96;
          }
          DAT_200001aa = DAT_200001aa + 1 & 7;
          uVar7 = (byte)(&DAT_2000108d)
                        [(uint)*(ushort *)(pbVar17 + 0x42) + (uint)DAT_200001ab +
                         (uint)DAT_200001a8 * 4] | 0x1800 |
                  (ushort)*pbVar23 << 0xd | (ushort)DAT_200001aa << 8;
          *(ushort *)(pbVar17 + 0x40) = uVar7;
          iVar8 = FUN_08012aac(uVar7,0x19);
          *(short *)(pbVar17 + 0x3e) = (short)iVar8;
          if ((iVar8 == 1) || (iVar8 == 5)) {
            if (6 < DAT_200001aa) {
              DAT_200001a9 = DAT_200001a9 + 1;
              uVar7 = (ushort)*pbVar23 << 0xd | 0x1200 | (ushort)DAT_200001a9;
              *(ushort *)(pbVar17 + 0x40) = uVar7;
              iVar8 = FUN_08012aac(uVar7,0x19);
              *(short *)(pbVar17 + 0x3e) = (short)iVar8;
              if ((iVar8 != 1) && (iVar8 != 5)) goto LAB_08046d96;
            }
            DAT_200001aa = DAT_200001aa + 1 & 7;
            uVar7 = (byte)(&DAT_2000108e)
                          [(uint)*(ushort *)(pbVar17 + 0x42) + (uint)DAT_200001ab +
                           (uint)DAT_200001a8 * 4] | 0x1800 |
                    (ushort)*pbVar23 << 0xd | (ushort)DAT_200001aa << 8;
            *(ushort *)(pbVar17 + 0x40) = uVar7;
            iVar8 = FUN_08012aac(uVar7,0x19);
            *(short *)(pbVar17 + 0x3e) = (short)iVar8;
            if ((iVar8 == 1) || (iVar8 == 5)) {
              if (6 < DAT_200001aa) {
                DAT_200001a9 = DAT_200001a9 + 1;
                uVar7 = (ushort)*pbVar23 << 0xd | 0x1200 | (ushort)DAT_200001a9;
                *(ushort *)(pbVar17 + 0x40) = uVar7;
                iVar8 = FUN_08012aac(uVar7,0x19);
                *(short *)(pbVar17 + 0x3e) = (short)iVar8;
                if ((iVar8 != 1) && (iVar8 != 5)) goto LAB_08046d96;
              }
              DAT_200001aa = DAT_200001aa + 1 & 7;
              uVar7 = (byte)(&DAT_2000108f)
                            [(uint)*(ushort *)(pbVar17 + 0x42) + (uint)DAT_200001ab +
                             (uint)DAT_200001a8 * 4] | 0x1800 |
                      (ushort)*pbVar23 << 0xd | (ushort)DAT_200001aa << 8;
              *(ushort *)(pbVar17 + 0x40) = uVar7;
              uVar6 = FUN_08012aac(uVar7,0x19);
              *(undefined2 *)(pbVar17 + 0x3e) = uVar6;
            }
          }
        }
      }
      else {
        DAT_200001a9 = DAT_200001a9 + 1;
        uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1200 | (ushort)DAT_200001a9;
        *(ushort *)(pbVar17 + 0x40) = uVar7;
        iVar8 = FUN_08012aac(uVar7,0x19);
        *(short *)(pbVar17 + 0x3e) = (short)iVar8;
        if ((iVar8 == 1) || (iVar8 == 5)) goto LAB_08046b68;
      }
LAB_08046d96:
    }
  }
LAB_08046dbc:
  uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1400;
  *(ushort *)(pbVar17 + 0x40) = uVar7;
  iVar8 = FUN_08012aac(uVar7,0x19);
  *(short *)(pbVar17 + 0x3e) = (short)iVar8;
  if ((iVar8 == 1) || (iVar8 == 5)) {
    uVar7 = (ushort)pbVar17[0x3c] << 0xd | 0x1581;
    *(ushort *)(pbVar17 + 0x40) = uVar7;
    uVar6 = FUN_08012aac(uVar7,0x19);
    *(undefined2 *)(pbVar17 + 0x3e) = uVar6;
  }
LAB_0804769a:
  DAT_200001fc = DAT_200001fc + 1;
  goto LAB_080476a8;
LAB_080467aa:
  if (!bVar28) goto LAB_080466c6;
LAB_080467ac:
  uVar7 = (ushort)pbVar23[0x3c] << 0xd | 0x1c01;
  *(ushort *)(pbVar23 + 0x40) = uVar7;
  iVar8 = FUN_08012aac(uVar7,0x19);
  *(short *)(pbVar23 + 0x3e) = (short)iVar8;
  if (iVar8 != 1) {
    bVar28 = iVar8 == 5;
LAB_080467cc:
    if (!bVar28) {
      do {
        if (!bVar28) goto LAB_080466a4;
        do {
          uVar7 = (ushort)pbVar23[0x3c] << 0xd | 0x1211;
          *(ushort *)(pbVar23 + 0x40) = uVar7;
          iVar8 = FUN_08012aac(uVar7,0x19);
          *(short *)(pbVar23 + 0x3e) = (short)iVar8;
          if ((iVar8 == 1) || (bVar28 = false, iVar8 == 5)) {
            if (DAT_200001a5 == 'f') {
              uVar6 = 0x1c14;
LAB_08046730:
              *(undefined2 *)(pbVar23 + 0x40) = uVar6;
            }
            else if (DAT_200001a5 == 'P') {
              uVar6 = 0x1c00;
              goto LAB_08046730;
            }
            uVar7 = *(ushort *)(pbVar23 + 0x40) | (ushort)pbVar23[0x3c] << 0xd;
            *(ushort *)(pbVar23 + 0x40) = uVar7;
            iVar8 = FUN_08012aac(uVar7,0x19);
            *(short *)(pbVar23 + 0x3e) = (short)iVar8;
            if (iVar8 != 1) {
              bVar28 = iVar8 == 5;
LAB_08046754:
              if (!bVar28) goto LAB_080466a4;
            }
            if (DAT_200001a5 == 'f') {
              uVar6 = 0x1d00;
LAB_0804677a:
              *(undefined2 *)(pbVar23 + 0x40) = uVar6;
            }
            else if (DAT_200001a5 == 'P') {
              uVar6 = 0x1d10;
              goto LAB_0804677a;
            }
            uVar7 = *(ushort *)(pbVar23 + 0x40);
            goto LAB_080467d2;
          }
LAB_080466a4:
          do {
            if (!bVar28) goto LAB_0804769a;
LAB_080466a6:
            uVar7 = (ushort)pbVar23[0x3c] << 0xd | 0x1a03;
            *(ushort *)(pbVar23 + 0x40) = uVar7;
            iVar8 = FUN_08012aac(uVar7,0x19);
            *(short *)(pbVar23 + 0x3e) = (short)iVar8;
            if (iVar8 == 1) break;
            bVar28 = iVar8 == 5;
LAB_080466c6:
          } while (!bVar28);
          uVar7 = (ushort)pbVar23[0x3c] << 0xd | 0x1b00;
          *(ushort *)(pbVar23 + 0x40) = uVar7;
          iVar8 = FUN_08012aac(uVar7,0x19);
          *(short *)(pbVar23 + 0x3e) = (short)iVar8;
        } while (iVar8 == 1);
        bVar28 = iVar8 == 5;
      } while( true );
    }
  }
  uVar7 = 0x1d00;
LAB_080467d2:
  uVar7 = uVar7 | (ushort)pbVar23[0x3c] << 0xd;
  *(ushort *)(pbVar23 + 0x40) = uVar7;
  iVar8 = FUN_08012aac(uVar7,0x19);
  *(short *)(pbVar23 + 0x3e) = (short)iVar8;
  pbVar23 = pbVar23 + 0x3c;
LAB_08046a44:
  if (iVar8 == 1) goto LAB_08046dbc;
  bVar28 = iVar8 == 5;
  goto LAB_08046a4a;
code_r0x0804623e:
  uVar14 = (uint)*(ushort *)(pbVar22 + 0x3c);
  pbVar24[uVar14 * 0x32 + 0xe3] = (byte)uVar10;
  *(ushort *)(pbVar22 + 0x42) = (short)uVar10 + (ushort)pbVar24[uVar14 * 0x32 + 0xe2] * 0x100;
  uVar14 = (uVar14 & 7) << 5 | 10;
  DAT_2000019b = (byte)uVar14;
  uVar14 = FUN_08013346(uVar14,0x18);
  *(short *)(pbVar22 + 0x3e) = (short)uVar14;
  uVar10 = (uVar14 & 0x3ff) >> 2;
  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
  uVar32 = CONCAT44(uVar10,uVar14) & 0xffffffff00000003;
  DAT_2000019e = (byte)(uVar14 & 3);
  bVar30 = (uint)(byte)(&DAT_0804a2a2)[uVar10] == (uVar14 & 3);
  pbVar23 = pbVar24;
  goto LAB_080462a0;
code_r0x080461f6:
  uVar7 = *(ushort *)(pbVar17 + 0x3c);
  pbVar24[(uint)uVar7 * 0x32 + 0xe2] = DAT_2000019d;
  uVar14 = (uVar7 & 7) << 5 | 9;
  DAT_2000019b = (byte)uVar14;
  uVar14 = FUN_08013346(uVar14,0x18);
  *(short *)(pbVar17 + 0x3e) = (short)uVar14;
  uVar10 = (uVar14 & 0x3ff) >> 2;
  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
  uVar14 = uVar14 & 3;
  DAT_2000019e = (byte)uVar14;
  bVar29 = (byte)(&DAT_0804a2a2)[uVar10] == uVar14;
  pbVar22 = pbVar17;
  goto LAB_0804623c;
code_r0x08046130:
  uVar7 = *(ushort *)(pbVar23 + 0x3c);
  pbVar24[(uint)uVar7 * 0x32 + 0xe0] = (byte)uVar10;
  uVar14 = (uVar7 & 7) << 5 | 0xf;
  DAT_2000019b = (byte)uVar14;
  uVar14 = FUN_08013346(uVar14,0x18);
  *(short *)(pbVar23 + 0x3e) = (short)uVar14;
  uVar10 = (uVar14 & 0x3ff) >> 2;
  DAT_2000019d = (byte)((uVar14 << 0x16) >> 0x18);
  uVar14 = uVar14 & 3;
  DAT_2000019e = (byte)uVar14;
  bVar30 = (byte)(&DAT_0804a2a2)[uVar10] == uVar14;
  goto LAB_08046186;
LAB_08049234:
  bVar26 = (byte)(uintptr_t)puVar18;
  bVar20 = (byte)uVar14;
  if (bVar30) goto LAB_08049102;
  bVar30 = false;
  if (puVar16[2] == '\0') {
    uVar14 = (uint)(byte)(uintptr_t)puVar16[3];
    bVar28 = 0x7f < uVar14;
    if (!bVar28) {
      uVar14 = uVar14 + uVar15;
      bVar28 = 0x80 < uVar14;
      if (!bVar28) {
        in_stack_ffffffc0 = (undefined1 *)(uVar15 >> 3);
        if ((uVar15 & 7) != 0) {
          in_stack_ffffffc0 = in_stack_ffffffc0 + 1;
        }
        uVar14 = 0;
        for (puVar18 = (undefined1 *)0x0; puVar18 < in_stack_ffffffc0;
            puVar18 = (undefined1 *)((uint)(puVar18 + 1) & 0xffff)) {
          (&DAT_20000b9f)[(int)puVar18] = (&DAT_20005bd0)[uVar14];
          uVar14 = uVar14 + 1 & 0xff;
        }
        DAT_20000b9c = cVar13;
        DAT_20000b9d = 2;
        DAT_20000b9e = (byte)(uintptr_t)in_stack_ffffffc0;
        uVar9 = FUN_08000e0e(&DAT_20000b9c,(uintptr_t)(in_stack_ffffffc0 + 3));
        (&DAT_20000b9f)[(int)in_stack_ffffffc0] = (char)((uint)uVar9 >> 8);
        goto LAB_080492fa;
      }
    }
    goto LAB_08049162;
  }
LAB_08049226:
  bVar26 = (byte)(uintptr_t)puVar18;
  bVar20 = (byte)uVar14;
  if (!bVar30) goto LAB_08049102;
  uVar15 = (uint)(byte)(uintptr_t)puVar16[5];
  bVar30 = 0x7f < uVar15;
  bVar28 = uVar15 == 0x80;
LAB_0804922e:
  bVar26 = (byte)(uintptr_t)puVar18;
  bVar20 = (byte)uVar14;
  if (bVar30 && !bVar28) goto LAB_08049102;
  bVar30 = uVar15 == 0;
  goto LAB_08049234;
}




/**
 * @brief Standard C entry point invoked by `__main` (0x08000130) after `__scatterload`.
 *        Dispatches directly to `FUN_080433e4` (0x080433E4).
 */
int main(void)
{
    return (int)FUN_080433e4();
}

