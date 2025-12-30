# STM32 UART CLI — LED Control, PWM & Timer Blinking

This project implements a fully functional Command Line Interface (CLI) on an STM32 Nucleo-F446RE board using UART communication.  
The CLI allows controlling the on-board LED using PWM brightness control and timer-based blinking — all in real-time through a serial terminal.

---

## ✨ Features
- Interactive UART CLI (115200 baud)
- Non-blocking UART receive using interrupts
- PWM-based LED brightness control (Timer2 Channel1)
- Timer interrupt based LED blinking (Timer7)
- Mode switching between:
  - IDLE Mode
  - PWM Mode
  - BLINK Mode
- Proper input validation + error messages
- Clean architecture + readable code

---

## 🧑‍💻 Supported Commands

| Command | Description |
|--------|-------------|
| `HELP` | Show command list |
| `SET PWM <10..100>` | Set LED brightness in % (multiples of 10) |
| `SET BLINK <10..2000>` | Blink LED with specified period (ms) |
| `STATUS` | Show current system mode and parameters |
| `IDLE` | Stop everything and turn LED OFF |

---

## 🛠️ Technical Highlights
- UART: `USART2`
- PWM Timer: `TIM2` (PA5 - LED pin using AF1)
- Blink Timer: `TIM7` periodic interrupt
- Uses:
  - `HAL_UART_Receive_IT`
  - `HAL_GPIO_EXTI_Callback`
  - `HAL_TIM_PeriodElapsedCallback`
  - `HAL_TIM_PWM_Start`
- Fully interrupt-driven (no blocking loops)

---

---

## 📦 Memory Footprint
Approx Flash Usage:
- ~20–25 KB (depends on build)

---

## ✔️ Target Board
- STM32 Nucleo F446RE

---

## 🎯 Purpose
This project builds confidence in:
- UART
- Timers
- Interrupts
- PWM
- Embedded CLI design
- Real firmware structuring

Perfect for learning + interviews + lab submission.

---

## 📜 License
Free to use for learning and development.


## 🖥️ How To Use
1️⃣ Connect board  
2️⃣ Open serial terminal (115200 baud)  
3️⃣ Type commands like:
