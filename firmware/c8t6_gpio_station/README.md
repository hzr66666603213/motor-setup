# C8T6 GPIO Station

Minimal STM32F103C8T6 firmware and Web Serial GUI for GPIO inspection and
control.

## Connections

- SWD flashing: PA13/SWDIO, PA14/SWCLK, GND, target 3.3 V.
- GUI serial: USART1 remapped to `PB6/PB7` at `115200 8N1`.
  - Board PB6/TX1 -> USB-UART RX
  - Board PB7/RX1 -> USB-UART TX
  - Board GND -> USB-UART GND

## Commands

- `HELLO`
- `PINS`
- `MODE PA0 IN`
- `MODE PA0 OUT`
- `MODE PA0 ANALOG`
- `READ PA0`
- `WRITE PA0 1`
- `WRITE PA0 0`
- `TOGGLE PC13`

PB6/PB7 are reserved for USART1. PA13/PA14 are reserved for SWD.
