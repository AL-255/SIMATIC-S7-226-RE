# SIMATIC S7-226 (APM32E103) Firmware Reverse Engineering

Complete C reconstruction of the 512 KB firmware image (`raw/APM32E103_backup.bin`) from an APM32E103 (STM32F103HD-compatible ARM Cortex-M3) SIMATIC S7-200 CPU 226 CN compatible PLC with integrated W5500 Ethernet and dual RS-485 ports.

## 1. Language, Compiler & Runtime Inference

| Property | Inferred Value | Evidence in Binary |
| :--- | :--- | :--- |
| **Source Language** | **C (C99 / Keil C)** | Zero C++ artifacts: no `.init_array` / `__cpp_initialize__aeabi_` in startup, no vtables, no RTTI, no `this`-pointer member call conventions, no exception handling frames (`__cxa_*`). |
| **Compiler / Linker** | **Keil MDK-ARM (`armcc` v5)** | Standard Keil startup sequence at `__main` (`0x08000130`) $\rightarrow$ `__scatterload_rt2` (`0x08000AA0`) reading `Region$$Table` (`0x0804A960..0x0804A980`), using Keil's LZ77+RLE `.data` decompressor `__decompress1` (`0x08000AC4`) and `__scatterload_zeroinit` (`0x08042C28`), plus Keil `armcc` software FP/64-bit runtime (`0x0800016C..0x08000A9E`) and sequential `.rodata` `tbb`/`tbh`/`ldr pc` switch tables (`0x0804A980..0x0804B41C`). |
| **SDK / HAL** | **STM32F10x Standard Peripheral Library (SPL)** | Direct 1:1 matches for `SystemInit` (`0x0803EE2C`), `SetSysClock` (`0x0803BB26`), `GPIO_Init` (`0x080143E4`), `USART_Init` (`0x08041FCE`), `TIM_TimeBaseInit` (`0x0803FC1E`), `ADC_Init` (`0x0800414A`), `FLASH_ErasePage` (`0x08013D20`), `FLASH_ProgramHalfWord` (`0x08013D0E`), `RCC_APB2PeriphClockCmd` (`0x08036B7C`), etc. |
| **Execution Model** | **Bare-Metal Superloop + ISR-Driven** | No RTOS (`PendSV_Handler` at `0x08021792` is a 2-byte `bx lr` stub). Real-time PLC tasks (1 ms timers, HSC quadrature counting, PTO/PWM pulse generation, UART framing) run in hardware ISRs with software-triggered deferred interrupt dispatch (`EXTI->SWIER`). |

---

## 2. Reconstructed Source Tree

All 11 reconstructed `.c` files and 3 `.h` headers (28,404 lines total) compile cleanly with `arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -std=c11 -O2`:

| File | Lines | Flash Address Range | Description |
| :--- | ---: | :--- | :--- |
| [`include/s7_226_plc.h`](include/s7_226_plc.h) | 3,022 | — | Master hardware register definitions, Flash/SRAM memory map (`0x20000000..0x2000CE38`), bit-band aliases, and function prototypes. |
| [`include/plc_vm.h`](include/plc_vm.h) | 693 | — | S7-200 MC7/STL bytecode opcodes (`0x00..0xFE`), 142 extended `0xFE` sub-opcodes (`0x07..0x94`), operand descriptors, accumulators (`AC0..AC3`), and VM state structures. |
| [`include/s7_ppi_comm.h`](include/s7_ppi_comm.h) | 413 | — | S7-200 PPI (`0x10`/`0x68`/`0xE5`), RFC 1006 ISO-on-TCP TPKT/COTP, and S7comm (`0x32`) protocol structures and area codes. |
| [`src/startup_apm32e103.c`](src/startup_apm32e103.c) | 506 | `0x08000000` – `0x08000B19` | 76-entry Cortex-M3 vector table (`Initial SP = 0x2000CE38`), `Reset_Handler`, `__main`, `__scatterload`, `__decompress1`, and `.data` initializers (`SystemCoreClock = 96 MHz`). |
| [`src/main.c`](src/main.c) | 4,676 | `0x080433E4` – `0x08049F1D` | Complete PLC executive (`FUN_080433e4`, 27.4 KB): cold/warm reset handling, memory descriptor init, hardware/W5500/SDB setup, 96-bit UID license check, and the cyclic PLC scan loop. |
| [`src/plc_prog_parser.c`](src/plc_prog_parser.c) | 1,185 | `0x08000E0E` – `0x08003E5B` | User program (`OB1`/`SBR_n`/`INT_n`) structural scanner, virtual-to-physical pointer translator, and `CALL SBR_n` `LB0..LB59` parameter marshaling. |
| [`src/plc_vm_interpreter.c`](src/plc_vm_interpreter.c) | 3,175 | `0x08021948` – `0x08036B1F` | Complete S7-200 MC7/STL Virtual Machine bytecode interpreter (`FUN_08021948`, 86.5 KB), including all bit logic, timers, counters, math, PID, string/table ops, and the 142-entry dispatch table at `0x0804AEC8`. |
| [`src/s7_comm_eth_port.c`](src/s7_comm_eth_port.c) | 1,728 | `0x080047AC` – `0x0800FB4D` | Primary S7comm / ISO-on-TCP (W5500 port 102) & bridged PPI protocol engine (`FUN_080047ac`, 46.0 KB), block upload/download, `OB1` nibble-XOR de-obfuscation (`0x08004300`), and I/O forcing. |
| [`src/s7_ppi_serial.c`](src/s7_ppi_serial.c) | 1,322 | `0x08017450` – `0x08021947` | RS-485 PPI multi-drop serial protocol engine (`FUN_08017450`, 41.8 KB) for Port 0 (`USART1`), Port 1 (`USART2`), and Port 2, plus Fletcher/weighted block checksum verifiers. |
| [`src/plc_hsc_exti.c`](src/plc_hsc_exti.c) | 2,151 | `0x0800FB4E` – `0x08013CA7` | `EXTI0..EXTI15_10` ISRs for High-Speed Counters (`HSC0..HSC5`), input edge interrupts (`I0.0..I0.3`), PTO envelope stepping (`EXTI2`), and `PC8..PC11` expansion bus bit-banging. |
| [`src/plc_int_dispatch.c`](src/plc_int_dispatch.c) | 3,172 | `0x08013CA8` – `0x0801744F`<br>`0x08037928` – `0x0803B100` | PLC interrupt event queues, priority arbitration, `OB1` scan wrapper (`0x08013D96`), `INT_n` wrapper (`0x08015402`), 11 nested `SBR_n` depth wrappers, and Modbus RTU handlers. |
| [`src/plc_sdb_expansion.c`](src/plc_sdb_expansion.c) | 1,516 | `0x0803B100` – `0x0803DFDD` | System Data Block (`SDB`) parser (`FUN_0803bf24`, 8.4 KB), expansion module (`EM0..EM6`) bus discovery and `SMB8..SMB21` address allocation, and operand write helpers. |
| [`src/bsp_peripherals.c`](src/bsp_peripherals.c) | 2,465 | `0x08000B1A` – `0x08000E0D`<br>`0x08036B20` – `0x08037927` | 96 MHz PLL setup, SPL peripheral drivers (`RCC`, `GPIO`, `ADC1`+DMA, `DAC`, `CAN1`, `RTC`, `BKP`), `SPI2` W5500 Ethernet driver, and internal Flash block/retentive storage manager. |
| [`src/plc_timers_pto.c`](src/plc_timers_pto.c) | 1,343 | `0x0803DFDE` – `0x080401B3` | 1 ms `SysTick_Handler` (`0x0803DFDE`: 1 ms timers, `SMB34`/`35` timed interrupts, `HSC0`/`1` polling, PTO ramp engine) and `TIM1..TIM8` PTO/PWM/turnaround ISRs. |
| [`src/plc_usart_rs485.c`](src/plc_usart_rs485.c) | 1,037 | `0x080401B4` – `0x08042317` | `USART1` (Port 0 RS-485), `USART2` (Port 1 RS-485), and `USART3` (Expansion Bus) DMA/IRQ drivers supporting PPI, Freeport (`RCV`/`XMT`), and Modbus RTU framing. |

---

## 3. Building / Checking the Reconstructed Code

Run `make` (requires `arm-none-eabi-gcc`):

```bash
make
```
