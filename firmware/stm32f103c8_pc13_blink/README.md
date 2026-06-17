# STM32F103C8 PC13 Blink

This is a minimal HAL example for blinking the LED on `PC13` of an
`STM32F103C8T6`, such as a Blue Pill board.

## Usage

1. Create an STM32CubeIDE or Keil project for `STM32F103C8Tx`.
2. Enable HAL drivers for STM32F1.
3. Replace the generated `Core/Src/main.c` with `main.c` in this folder, or
   copy the `main`, `SystemClock_Config`, and `MX_GPIO_Init` functions into your
   generated file.
4. Build and flash.

## Notes

- Common Blue Pill boards connect the onboard LED to `PC13` as active low.
  `GPIO_PIN_RESET` turns it on, and `GPIO_PIN_SET` turns it off.
- `SystemClock_Config` assumes an 8 MHz external crystal and configures the MCU
  to run at 72 MHz.
- If your board has no external crystal, generate a clock config from CubeMX
  using HSI instead.
