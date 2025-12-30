# 🧰 STM32 UART Command Line Interface (CLI)

![STM32](https://img.shields.io/badge/Hardware-STM32_Nucleo_F446RE-blue)
![Language](https://img.shields.io/badge/Language-Embedded_C-green)
![IDE](https://img.shields.io/badge/IDE-STM32CubeIDE-orange)

A lightweight, non-blocking **Command Line Interface (CLI)** built on the STM32 Nucleo-F446RE. This project demonstrates how to control hardware peripherals (PWM, Timers, GPIO) dynamically via a serial terminal, using interrupt-driven architecture to ensure the CPU remains efficient.

## 📸 Screenshots
| **CLI in Action** | **Pinout Configuration** |
|:---:|:---:|
| <img src="cli_output.png" width="400" alt="CLI Output"> | <img src="pinout.png" width="400" alt="STM32 Pinout"> |
*(Replace `cli_output.png` and `pinout.png` with your actual image files)*

---

## ✅ Features
* **Real-time UART Parsing:** Custom string parser handles commands without blocking the main loop.
* **Interrupt-Driven RX:** Uses `HAL_UART_Receive_IT` to capture data asynchronously.
* **Dual Mode LED Control:**
    * **PWM Mode:** Adjusts brightness (0-100%) using hardware Timer PWM (TIM2).
    * **Blink Mode:** Toggles LED with precise millisecond periods using Timer Interrupts (TIM7).
* **Robust Error Handling:** Detects invalid commands, out-of-range values, and parsing errors.

---

## ⚙️ Hardware & Tools
* **Board:** STM32 Nucleo-F446RE (ARM Cortex-M4)
* **Actuators:** User LED (Connected to PA5)
* **Communication:** UART2 (Connected via USB ST-Link)
* **Software:** STM32CubeIDE, HAL Library
* **Terminal:** PuTTY / TeraTerm / Serial Monitor

---

## 🖥️ CLI Commands
Connect your serial terminal at **115200 baud** to interact with the board.

| Command | Arguments | Description | Example |
| :--- | :--- | :--- | :--- |
| **`HELP`** | None | Displays the list of available commands. | `HELP` |
| **`SET PWM`** | `10` - `100` | Sets LED brightness %. Only multiples of 10 allowed. | `SET PWM 50` |
| **`SET BLINK`** | `10` - `2000` | Sets LED blinking period in milliseconds. | `SET BLINK 500` |
| **`STATUS`** | None | Prints current system mode (PWM/Blink) and active values. | `STATUS` |
| **`IDLE`** | None | Stops all timers and turns the LED off. | `IDLE` |

### Example Session
```text
> HELP
... (Command List) ...

> SET PWM 80
PWM Set to 80%

> SET BLINK 100
Blinking at 100ms

> STATUS
System: BLINKING (Period: 100 ms)
