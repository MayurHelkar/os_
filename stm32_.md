Sure — let’s make a **comprehensive and in-depth set of notes on STM32 microcontrollers**, covering everything from architecture to peripherals, programming, and debugging.
I’ll organize this into clear sections so you can use it as a full reference or study guide.

---

# 🧠 **STM32 Microcontroller — In-Depth Notes**

---

## 1. 🧩 **Overview**

**STM32** is a family of 32-bit microcontrollers (MCUs) developed by **STMicroelectronics**, based on **ARM Cortex-M cores**.
They are widely used in embedded systems, IoT, robotics, and industrial control due to their balance of performance, power efficiency, and rich peripherals.

### STM32 Series Families

| Family            | Core              | Performance | Flash/RAM          | Specialty                   |
| ----------------- | ----------------- | ----------- | ------------------ | --------------------------- |
| **STM32F0**       | Cortex-M0         | Basic       | Up to 256 KB Flash | Low-cost, simple tasks      |
| **STM32F1**       | Cortex-M3         | Medium      | Up to 1 MB Flash   | General-purpose             |
| **STM32F3**       | Cortex-M4         | Medium-High | Up to 512 KB Flash | Mixed-signal (ADC/DAC)      |
| **STM32F4**       | Cortex-M4         | High        | Up to 2 MB Flash   | DSP & floating point        |
| **STM32F7**       | Cortex-M7         | Very High   | Up to 2 MB Flash   | High-performance apps       |
| **STM32H7**       | Cortex-M7         | Ultra High  | Up to 2 MB Flash   | Dual-core, advanced DSP     |
| **STM32L0/L4/L5** | Cortex-M0+/M4/M33 | Low Power   | Up to 1 MB Flash   | Battery/IoT optimized       |
| **STM32G0/G4**    | Cortex-M0+/M4     | Balanced    | Up to 512 KB Flash | General-purpose + low power |
| **STM32WB/WL**    | Cortex-M4/M0+     | Medium      | Varies             | Wireless (Bluetooth, LoRa)  |

---

## 2. ⚙️ **Architecture Overview**

### 2.1 ARM Cortex-M Core

* **Harvard architecture**: separate buses for instructions and data.
* **Load/Store architecture**: operations occur on registers, not directly on memory.
* **Pipeline**: typically 3 to 6 stages depending on the model (fetch, decode, execute...).
* **Interrupts**: NVIC (Nested Vectored Interrupt Controller) supports priorities and nesting.
* **SysTick timer**: system timer for RTOS or timebase.
* **Memory Protection Unit (MPU)**: available on higher-end cores for memory access control.

### 2.2 Bus Matrix

STM32 MCUs use an **AHB/APB** bus architecture:

* **AHB (Advanced High-performance Bus)** – used for CPU, DMA, Flash, SRAM, high-speed peripherals (e.g., USB, Ethernet).
* **APB (Advanced Peripheral Bus)** – used for slower peripherals (GPIO, UART, SPI, I2C, etc.).
* **Bus Bridges** (AHB1 → APB1/APB2) connect different speed domains.

---

## 3. 💾 **Memory Organization**

| Type                         | Description                                                           |
| ---------------------------- | --------------------------------------------------------------------- |
| **Flash Memory**             | Non-volatile memory storing program code.                             |
| **SRAM**                     | Volatile memory for variables, stacks, buffers.                       |
| **EEPROM / Emulated EEPROM** | Non-volatile user data storage (some STM32s emulate EEPROM in Flash). |
| **Peripheral Registers**     | Mapped to memory space (MMIO) for direct access.                      |
| **Bootloader Memory**        | Built-in ROM bootloader supports USART, USB, CAN, etc.                |

### Memory Map (Typical)

```
0x0000 0000 - 0x1FFF FFFF : Code (Flash, Bootloader, etc.)
0x2000 0000 - 0x3FFF FFFF : SRAM
0x4000 0000 - 0x5FFF FFFF : Peripherals
0x6000 0000 - 0x9FFF FFFF : External memories (FSMC/QUADSPI)
0xE000 0000 - 0xE00F FFFF : System control space (NVIC, SysTick)
```

---

## 4. ⚡ **Clocks and Reset System (RCC)**

### 4.1 Clock Sources

* **HSI**: High-Speed Internal oscillator (default, ~8 MHz or 16 MHz)
* **HSE**: High-Speed External oscillator (crystal)
* **LSI**: Low-Speed Internal (~32 kHz)
* **LSE**: Low-Speed External (~32.768 kHz crystal)
* **PLL**: Phase-Locked Loop for frequency multiplication/division.

### 4.2 Clock Tree

The RCC controls:

* System Clock (SYSCLK)
* AHB, APB1, APB2 bus clocks
* Peripheral clocks (USARTx, TIMx, GPIOx, etc.)
* Optional PLL-based high-frequency clock generation.

---

## 5. 🔌 **GPIO (General Purpose I/O)**

### Configuration

Each GPIO pin can function as:

* **Input (floating, pull-up/down)**
* **Output (push-pull, open-drain)**
* **Alternate Function (AF)** – for peripherals like SPI, USART, I2C, etc.
* **Analog Mode** – for ADC/DAC.

### Registers (Example: GPIOA)

* `GPIOx_MODER` – mode register
* `GPIOx_OTYPER` – output type
* `GPIOx_OSPEEDR` – speed
* `GPIOx_PUPDR` – pull-up/pull-down
* `GPIOx_IDR` / `GPIOx_ODR` – input/output data
* `GPIOx_BSRR` – bit set/reset (atomic operations)

---

## 6. ⏱️ **Timers**

STM32 MCUs feature multiple timers:

| Timer Type                             | Features                                                            |
| -------------------------------------- | ------------------------------------------------------------------- |
| **Basic Timers (TIM6, TIM7)**          | Simple up/down counters, trigger sources.                           |
| **General-Purpose Timers (TIM2–TIM5)** | PWM, Input Capture, Output Compare.                                 |
| **Advanced Timers (TIM1, TIM8)**       | Dead-time insertion, complementary outputs (used in motor control). |
| **SysTick Timer**                      | System tick for RTOS or delay routines.                             |

Timers can trigger:

* PWM generation
* Timebase interrupts
* ADC sampling synchronization
* Encoder interface reading

---

## 7. 🔄 **Communication Peripherals**

| Peripheral        | Description                                                          |
| ----------------- | -------------------------------------------------------------------- |
| **USART/UART**    | Serial communication; supports asynchronous, synchronous, LIN, IrDA. |
| **SPI**           | Synchronous serial; full-duplex master/slave.                        |
| **I²C**           | Two-wire serial; multi-master/slave.                                 |
| **CAN / FD-CAN**  | Automotive and industrial communication.                             |
| **USB OTG FS/HS** | USB host/device support.                                             |
| **SDIO / SDMMC**  | SD card interface.                                                   |
| **QSPI**          | Quad-SPI for external Flash.                                         |
| **Ethernet MAC**  | On STM32F7/H7 devices.                                               |

---

## 8. 🎛️ **Analog Peripherals**

### ADC (Analog to Digital Converter)

* 12-bit / 16-bit resolution
* Single-ended or differential input
* Multi-channel scanning
* DMA integration for high-speed sampling
* Triggered by timers or software

### DAC (Digital to Analog Converter)

* 12-bit output resolution
* Buffered/unbuffered output
* Can generate waveforms (sine, triangle)

### Comparator / OPAMP / PGA

* Available on STM32F3/G4 series for mixed-signal applications.

---

## 9. 🧮 **DMA (Direct Memory Access)**

* Moves data between peripherals and memory **without CPU intervention**.
* Multiple DMA channels/streams available.
* Reduces CPU load for high-speed data operations (e.g., ADC sampling → memory).

---

## 10. 🧰 **Programming and Development Tools**

### Toolchains

* **STM32CubeIDE** (official, Eclipse-based)
* **Keil uVision** (ARM MDK)
* **IAR Embedded Workbench**
* **PlatformIO** (VSCode-based)
* **Makefile + GCC (arm-none-eabi-gcc)**

### Firmware Libraries

* **STM32Cube HAL (Hardware Abstraction Layer)**

  * High-level, easy to use.
* **LL (Low Layer API)**

  * Faster, lower-level register access.
* **Standard Peripheral Library (SPL)** — legacy but still used in STM32F1.

---

## 11. 🧑‍💻 **Boot Modes**

Boot options are configured via **BOOT0/BOOT1 pins**:

| Boot Mode         | Description                                      |
| ----------------- | ------------------------------------------------ |
| **Main Flash**    | Normal user program boot.                        |
| **System Memory** | Built-in bootloader mode (UART, USB, CAN, etc.). |
| **SRAM**          | Used for debugging/testing.                      |

---

## 12. 🐞 **Debugging and Programming Interfaces**

* **SWD (Serial Wire Debug)** – 2-pin (SWDIO, SWCLK)
* **JTAG** – 4-pin (TDI, TDO, TMS, TCK)
* **ST-Link** – ST’s in-circuit debugger/programmer.
* **OpenOCD / pyOCD / DAPLink** – open-source debugging tools.

---

## 13. 🔐 **Security Features (Advanced Models)**

* **Readout Protection (RDP)** levels 0–2
* **Write protection** for Flash sectors
* **TrustZone (Cortex-M33)** for secure/non-secure code partitioning
* **Crypto accelerators** (AES, SHA, RNG)
* **Secure Boot / Firmware Update (SB/SFU)**

---

## 14. ⚡ **Power Modes**

STM32 supports several power-saving modes:

| Mode         | Description                               |
| ------------ | ----------------------------------------- |
| **Run**      | Full speed operation.                     |
| **Sleep**    | CPU halted; peripherals active.           |
| **Stop**     | Most clocks off; SRAM retained.           |
| **Standby**  | Minimal power; wake-up via reset/pin/RTC. |
| **Shutdown** | Deepest low-power mode (on L-series).     |

---

## 15. 🧭 **RTOS Integration**

* FreeRTOS and Zephyr are commonly used.
* SysTick timer used as OS tick source.
* STM32CubeMX auto-generates RTOS integration code.

---

## 16. 🧱 **Common Application Examples**

| Application          | Key Peripherals                   |
| -------------------- | --------------------------------- |
| **Motor Control**    | PWM, Timers, ADC, DMA             |
| **Sensor Interface** | I2C, SPI, ADC                     |
| **IoT Device**       | UART, LoRa/BLE (WB/WL), Low Power |
| **Audio Processing** | I2S, DAC, DMA                     |
| **Data Logging**     | SDIO, FATFS, RTC                  |

---

## 17. 🧾 **Tips for Development**

* Always configure **clock tree** properly before peripheral init.
* Use **HAL or LL drivers** instead of raw register unless performance-critical.
* Enable **DMA and interrupts** to offload CPU.
* Use **CubeMX** to generate base code, but learn register-level details for control.
* Always check **errata sheets** for known hardware issues.

---

Perfect ✅ — let’s continue expanding your **STM32 in-depth theoretical reference**, moving into **interrupt handling, NVIC, RTOS integration**, and other advanced core concepts.

This will give you a **complete theoretical understanding** of how STM32 systems behave at the architecture and OS level, which is essential before writing optimized code.

---

# 🧠 **STM32 In-Depth Notes — Part 2**

---

## 18. ⚡ **Interrupts and Exception Handling**

STM32 uses the **Nested Vectored Interrupt Controller (NVIC)**, part of the ARM Cortex-M core.
It manages **external interrupts (IRQs)** and **system exceptions**.

---

### 18.1 NVIC Overview

The NVIC provides:

* Up to **240 interrupt sources** (depending on the MCU).
* **Programmable priority levels** (4–16 bits depending on implementation).
* **Preemption and subpriority** (for nested interrupts).
* **Dynamic enabling/disabling** of interrupts.

**Key Registers (NVIC):**

* `NVIC_ISER` – Interrupt Set-Enable Register
* `NVIC_ICER` – Interrupt Clear-Enable Register
* `NVIC_ISPR` – Set-Pending Register
* `NVIC_ICPR` – Clear-Pending Register
* `NVIC_IPR` – Priority Registers

Each interrupt vector corresponds to a function pointer stored in the **vector table** (in Flash or relocated in RAM).

---

### 18.2 Vector Table Layout

At address `0x00000000` (or remapped), we have:

| Address | Description                                              |
| ------- | -------------------------------------------------------- |
| 0x00    | Initial Stack Pointer                                    |
| 0x04    | Reset Handler                                            |
| 0x08    | NMI Handler                                              |
| 0x0C    | HardFault Handler                                        |
| ...     | Other system exceptions                                  |
| ...     | Peripheral Interrupt Handlers (TIMx, USARTx, GPIO, etc.) |

**Example:**

```c
void (* const g_pfnVectors[])(void) __attribute__((section(".isr_vector"))) = {
    (void *)&_estack,       // Initial Stack Pointer
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,             // Reserved
    SVC_Handler,
    DebugMon_Handler,
    0,                      // Reserved
    PendSV_Handler,
    SysTick_Handler,
    // Peripheral interrupts:
    WWDG_IRQHandler,
    PVD_IRQHandler,
    ...
};
```

---

### 18.3 Interrupt Priority and Grouping

Interrupt priority in STM32 is divided into:

* **Preemption Priority** – determines which interrupt can interrupt another.
* **Subpriority** – used to resolve multiple pending interrupts with same preemption priority.

Configured via:

```c
HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
HAL_NVIC_EnableIRQ(USART1_IRQn);
```

Groupings:

| Group                | Preempt bits | Subpriority bits |
| -------------------- | ------------ | ---------------- |
| NVIC_PRIORITYGROUP_0 | 0            | 4                |
| NVIC_PRIORITYGROUP_1 | 1            | 3                |
| NVIC_PRIORITYGROUP_2 | 2            | 2                |
| NVIC_PRIORITYGROUP_3 | 3            | 1                |
| NVIC_PRIORITYGROUP_4 | 4            | 0                |

---

### 18.4 Exception Types

| Exception                        | Description                                     |
| -------------------------------- | ----------------------------------------------- |
| **Reset**                        | Occurs on system start-up or reset event.       |
| **NMI (Non-Maskable Interrupt)** | Cannot be disabled, highest priority.           |
| **HardFault**                    | On invalid memory access, divide-by-zero, etc.  |
| **MemManage**                    | Memory protection fault (if MPU used).          |
| **BusFault**                     | Bus error (invalid memory, peripheral access).  |
| **UsageFault**                   | Invalid instruction or state.                   |
| **SVC (Supervisor Call)**        | Used by RTOS or system services.                |
| **PendSV**                       | Context switching (RTOS uses this).             |
| **SysTick**                      | Periodic tick interrupt (used in HAL and RTOS). |

---

## 19. 🧩 **System Tick Timer (SysTick)**

### 19.1 Overview

SysTick is a **24-bit down counter** built into every Cortex-M core.
It can:

* Generate a periodic interrupt.
* Serve as the **system tick** for delays, HAL, or RTOS.

**Registers:**

* `SYST_CSR` – Control and Status
* `SYST_RVR` – Reload Value
* `SYST_CVR` – Current Value
* `SYST_CALIB` – Calibration Value

**Example (1ms tick):**

```c
SysTick_Config(SystemCoreClock / 1000);
```

This causes SysTick_Handler() to execute every 1 ms.

---

### 19.2 SysTick in RTOS

In **FreeRTOS**, SysTick drives the **system tick interrupt** used for task scheduling.
In **STM32 HAL**, it drives:

* `HAL_Delay()`
* Timeouts in peripheral drivers
* Millisecond-based timing for polling loops

---

## 20. 🧠 **RTOS Integration (FreeRTOS Example)**

### 20.1 What is an RTOS?

A **Real-Time Operating System (RTOS)** manages multiple concurrent tasks with deterministic scheduling.
It handles:

* **Multitasking**
* **Inter-task communication (queues, semaphores)**
* **Timing and delays**
* **Synchronization**

---

### 20.2 FreeRTOS Kernel Basics

| Concept               | Description                        |
| --------------------- | ---------------------------------- |
| **Task**              | Independent thread of execution    |
| **Scheduler**         | Determines which task runs next    |
| **Tick**              | Periodic interrupt (often SysTick) |
| **Queue**             | FIFO data exchange between tasks   |
| **Semaphore / Mutex** | Synchronization tools              |
| **Event Group**       | Flag-based synchronization         |
| **Timer**             | Software timer for delayed actions |

---

### 20.3 Context Switching

RTOS uses **PendSV** exception to perform context switches.

* SysTick → triggers RTOS tick interrupt
* Scheduler decides next task → sets **PendSV**
* PendSV handler saves/restores task context (registers, stack pointer)

Sequence:

```
SysTick_Handler() --> xTaskIncrementTick() --> PendSV_Handler() --> Switch Context
```

---

### 20.4 Memory Allocation in FreeRTOS

FreeRTOS offers several **heap allocation schemes** (`heap_1.c` → `heap_5.c`):

* **heap_1:** static, no freeing
* **heap_2:** free/realloc support
* **heap_3:** wraps `malloc/free`
* **heap_4:** coalescing allocator (best for embedded)
* **heap_5:** multiple memory regions

Configured in **FreeRTOSConfig.h**.

---

### 20.5 Task States

| State         | Description                 |
| ------------- | --------------------------- |
| **Running**   | Task currently executing    |
| **Ready**     | Waiting for CPU time        |
| **Blocked**   | Waiting for event/timeout   |
| **Suspended** | Manually suspended          |
| **Deleted**   | Terminated, waiting cleanup |

---

## 21. 🕹️ **Nested Interrupts and RTOS**

* NVIC supports **nested interrupts**, but RTOS masks priorities below a threshold.
* **FreeRTOS config parameter**:
  `configMAX_SYSCALL_INTERRUPT_PRIORITY`
  defines the highest priority allowed to call RTOS APIs from ISR.
* Higher priority interrupts (above this value) **cannot use RTOS functions**.

---

## 22. 🧱 **System Boot and Reset Flow**

### 22.1 Reset Sources

STM32 reset can be triggered by:

* Power-on reset
* System reset (software)
* Watchdog reset (IWDG/WWDG)
* Brown-out reset
* Pin reset (NRST)
* Clock failure (if configured)

### 22.2 Reset Sequence

1. CPU fetches **initial SP** from vector table.
2. CPU fetches **Reset_Handler** address.
3. Executes **startup code**:

   * Initialize data/BSS sections
   * Configure clocks, PLL
   * Call `SystemInit()`
   * Jump to `main()`

---

## 23. 🔐 **Memory Protection Unit (MPU)**

### 23.1 Purpose

* Isolate tasks or memory regions
* Prevent accidental data corruption
* Enable privilege separation (secure vs non-secure)

### 23.2 MPU Features

* Up to **8 configurable regions**
* Each with attributes: read/write/execute, cacheable, bufferable
* Enabled by setting **PRIVDEFENA** and **ENABLE** bits

---

## 24. ⏳ **Watchdogs**

### 24.1 Independent Watchdog (IWDG)

* Runs on **LSI** (independent from main clock).
* Once started, **cannot be stopped**.
* Must periodically "kick" the watchdog (`IWDG->KR = 0xAAAA`).

### 24.2 Window Watchdog (WWDG)

* Based on **APB1** clock.
* Requires refresh **within a time window** (not too early/late).
* Useful for detecting stuck code loops.

---

## 25. 🧮 **Real-Time Clock (RTC)**

* Runs independently (LSI or LSE clock).
* Keeps time and date even in low-power modes.
* Can generate **alarms**, **wake-ups**, or **timestamp events**.
* Has backup registers for small non-volatile data storage.

---

## 26. 🪄 **Bootloader and System Memory**

### 26.1 Built-in Bootloader

Located in **system memory (ROM)**.
Supports boot via:

* USART
* USB DFU
* CAN
* I2C
* SPI

### 26.2 Boot Configuration

Controlled by **BOOT0** pin and **BOOT1** option bit.

| BOOT0 | BOOT1 | Boot Source   |
| ----- | ----- | ------------- |
| 0     | X     | Main Flash    |
| 1     | 0     | System Memory |
| 1     | 1     | SRAM          |

---

## 27. 🧾 **Error and Fault Handling**

Common fault handlers:

* `HardFault_Handler`
* `BusFault_Handler`
* `UsageFault_Handler`
* `MemManage_Handler`

To debug a **HardFault**, examine:

* **SCB registers** (`HFSR`, `CFSR`, `BFAR`, `MMFAR`)
* The **stack frame** to identify PC/LR at crash.

---

## 28. ⚙️ **System Control Block (SCB)**

SCB is a core Cortex-M system register block.

| Register | Description                             |
| -------- | --------------------------------------- |
| `CPUID`  | CPU identification                      |
| `ICSR`   | Interrupt Control and State             |
| `AIRCR`  | Application Interrupt and Reset Control |
| `SCR`    | System Control Register                 |
| `CCR`    | Configuration and Control               |
| `SHCSR`  | System Handler Control and State        |
| `CFSR`   | Configurable Fault Status               |
| `HFSR`   | HardFault Status                        |

---

## 29. 🪫 **Low Power and Sleep Control**

* The **System Control Register (SCR)** defines sleep behavior.
* Key instructions:

  * `__WFI()` — Wait For Interrupt
  * `__WFE()` — Wait For Event
* Used for power saving in Sleep/Stop/Standby modes.

---

## 30. 🔋 **Clock and Power Optimization Tips**

* Lower AHB/APB prescalers when not needed.
* Use **Stop Mode** between active periods.
* Disable unused peripheral clocks via RCC.
* Use **DMA** + **interrupts** instead of polling.
* On L-series, use **VREFINT**, **ULP wake-up**, **shutdown** for extreme savings.

---

That covers the **theoretical foundation** up to the RTOS and interrupt level — you now have the **complete picture of STM32 architecture and operation** from hardware to OS interface.

---

Would you like me to continue with **Part 3**, focusing on **peripheral operation theory** (e.g., UART, SPI, ADC, DMA, timers — how they work internally and how to synchronize them theoretically)?
That would complete the full STM32 theoretical series.


Would you like me to continue this into a **part 2** (deep dive into programming — e.g., how to configure peripherals in C with examples), or keep expanding this theoretical reference (like interrupt handling, RTOS setup, etc.)?
