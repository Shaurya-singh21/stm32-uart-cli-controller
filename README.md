🧰 STM32 UART Command Line Interface (CLI) Project

A lightweight UART-based Command Line Interface built on STM32 (Nucleo F446RE) that allows controlling LED modes, PWM brightness, and timer-based blinking directly from a serial terminal.
This project strengthens understanding of GPIO, PWM, Timers, Interrupts, and UART RX (Interrupt Mode) while demonstrating real embedded firmware structuring and parsing.

✅ Features

UART-based command interface

Non-blocking UART reception using interrupts

Supports:

LED PWM brightness control

LED blinking using timers

Idle mode

System status reporting

Input validation and error handling

Clean command parsing

Beginner-friendly but industry-style logic

🖥️ CLI Commands
HELP
    Show available commands

SET PWM <10,20,...,100>
    Set LED brightness (%) using PWM
    Only multiples of 10 allowed

SET BLINK <ms>
    Blink LED using timer
    Range: 10ms – 2000ms

STATUS
    Show current operating mode + parameters

IDLE
    Stop PWM / Timer and turn LED OFF


Example Usage:

> HELP
> SET PWM 50
> STATUS
> SET BLINK 500
> IDLE


Wrong command?

Error: Unknown command Type HELP


Invalid values?

wrong values

⚙️ Hardware & Tools

STM32 Nucleo-F446RE (or compatible STM32 board)

USB connection

Serial terminal (115200 baud)

PuTTY / TeraTerm / Serial Monitor

🧩 Core Concepts Practiced

GPIO Output

Timer Base + Interrupt

PWM Generation

UART RX Interrupt Mode

ISR discipline + state handling

Embedded parsing logic

HAL APIs:

HAL_UART_Receive_IT

HAL_UART_RxCpltCallback

HAL_TIM_Base_Start_IT

HAL_TIM_PWM_Start

__HAL_TIM_SET_COMPARE

🏗️ Build & Run

1️⃣ Flash code via STM32CubeIDE
2️⃣ Open Serial Terminal at 115200 baud
3️⃣ Reset board
4️⃣ Type HELP to begin

📦 Memory Footprint

Approx Firmware Size:

~21 KB Flash (depends slightly on build config)


Comfortably fits STM32F4 flash.
