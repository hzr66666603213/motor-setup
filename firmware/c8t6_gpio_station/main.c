#include <stdbool.h>
#include <stdint.h>

#define RCC_BASE        0x40021000u
#define GPIOA_BASE      0x40010800u
#define GPIOB_BASE      0x40010C00u
#define GPIOC_BASE      0x40011000u
#define USART1_BASE     0x40013800u
#define SYSTICK_BASE    0xE000E010u

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define RCC_CR          REG32(RCC_BASE + 0x00u)
#define RCC_CFGR        REG32(RCC_BASE + 0x04u)
#define RCC_APB2ENR     REG32(RCC_BASE + 0x18u)

#define AFIO_BASE       0x40010000u
#define AFIO_MAPR       REG32(AFIO_BASE + 0x04u)

#define GPIO_CRL(base)  REG32((base) + 0x00u)
#define GPIO_CRH(base)  REG32((base) + 0x04u)
#define GPIO_IDR(base)  REG32((base) + 0x08u)
#define GPIO_ODR(base)  REG32((base) + 0x0Cu)
#define GPIO_BSRR(base) REG32((base) + 0x10u)
#define GPIO_BRR(base)  REG32((base) + 0x14u)

#define USART_SR        REG32(USART1_BASE + 0x00u)
#define USART_DR        REG32(USART1_BASE + 0x04u)
#define USART_BRR       REG32(USART1_BASE + 0x08u)
#define USART_CR1       REG32(USART1_BASE + 0x0Cu)

#define SYST_CSR        REG32(SYSTICK_BASE + 0x00u)
#define SYST_RVR        REG32(SYSTICK_BASE + 0x04u)
#define SYST_CVR        REG32(SYSTICK_BASE + 0x08u)

static volatile uint32_t g_ms;

void SysTick_Handler(void)
{
  g_ms++;
}

static void delay_ms(uint32_t ms)
{
  const uint32_t end = g_ms + ms;
  while ((int32_t)(end - g_ms) > 0) {
  }
}

static void uart_putc(char c)
{
  while ((USART_SR & (1u << 7)) == 0u) {
  }
  USART_DR = (uint32_t)c;
}

static void uart_write(const char *s)
{
  while (*s != '\0') {
    uart_putc(*s++);
  }
}

static void uart_writeln(const char *s)
{
  uart_write(s);
  uart_write("\r\n");
}

static int uart_getc_nonblock(void)
{
  if ((USART_SR & (1u << 5)) == 0u) {
    return -1;
  }
  return (int)(USART_DR & 0xFFu);
}

static uint32_t gpio_base(char port)
{
  if (port == 'A') {
    return GPIOA_BASE;
  }
  if (port == 'B') {
    return GPIOB_BASE;
  }
  if (port == 'C') {
    return GPIOC_BASE;
  }
  return 0u;
}

static bool parse_pin(const char *s, char *port, uint8_t *pin)
{
  if (s[0] != 'P') {
    return false;
  }

  const char p = s[1];
  if (p != 'A' && p != 'B' && p != 'C') {
    return false;
  }

  uint8_t n = 0u;
  uint8_t i = 2u;
  if (s[i] < '0' || s[i] > '9') {
    return false;
  }
  while (s[i] >= '0' && s[i] <= '9') {
    n = (uint8_t)(n * 10u + (uint8_t)(s[i] - '0'));
    i++;
  }
  if (s[i] != '\0' || n > 15u) {
    return false;
  }

  *port = p;
  *pin = n;
  return true;
}

static void gpio_set_mode(char port, uint8_t pin, uint32_t mode)
{
  const uint32_t base = gpio_base(port);
  if (base == 0u) {
    return;
  }

  volatile uint32_t *cr = (pin < 8u) ? &GPIO_CRL(base) : &GPIO_CRH(base);
  const uint8_t shift = (uint8_t)((pin & 7u) * 4u);
  uint32_t value = *cr;
  value &= ~(0xFu << shift);
  value |= (mode & 0xFu) << shift;
  *cr = value;
}

static void gpio_write_pin(char port, uint8_t pin, bool high)
{
  const uint32_t base = gpio_base(port);
  if (base == 0u) {
    return;
  }

  if (high) {
    GPIO_BSRR(base) = 1u << pin;
  } else {
    GPIO_BRR(base) = 1u << pin;
  }
}

static bool gpio_read_pin(char port, uint8_t pin)
{
  const uint32_t base = gpio_base(port);
  if (base == 0u) {
    return false;
  }
  return (GPIO_IDR(base) & (1u << pin)) != 0u;
}

static bool pin_is_reserved(char port, uint8_t pin)
{
  if (port == 'A' && (pin == 13u || pin == 14u)) {
    return true;
  }
  if (port == 'B' && (pin == 6u || pin == 7u)) {
    return true;
  }
  return false;
}

static bool streq(const char *a, const char *b)
{
  while (*a != '\0' && *b != '\0') {
    if (*a++ != *b++) {
      return false;
    }
  }
  return *a == *b;
}

static char *next_token(char **cursor)
{
  char *s = *cursor;
  while (*s == ' ') {
    s++;
  }
  if (*s == '\0') {
    *cursor = s;
    return 0;
  }

  char *token = s;
  while (*s != '\0' && *s != ' ') {
    s++;
  }
  if (*s == ' ') {
    *s++ = '\0';
  }
  *cursor = s;
  return token;
}

static void handle_line(char *line)
{
  char *cursor = line;
  char *cmd = next_token(&cursor);
  if (cmd == 0) {
    return;
  }

  if (streq(cmd, "HELLO")) {
    uart_writeln("OK C8T6-GPIO-STATION 1 USART1=PB6/PB7 BAUD=115200");
    return;
  }

  if (streq(cmd, "PINS")) {
    uart_writeln("OK PA0 PA1 PA2 PA3 PA4 PA5 PA6 PA7 PA8 PA11 PA12 PA15 PB0 PB1 PB3 PB4 PB5 PB6 PB7 PB8 PB9 PB10 PB11 PB12 PB13 PB14 PB15 PC13 PC14 PC15");
    return;
  }

  char *pin_name = next_token(&cursor);
  char port = 0;
  uint8_t pin = 0;
  if (pin_name == 0 || !parse_pin(pin_name, &port, &pin)) {
    uart_writeln("ERR BAD_PIN");
    return;
  }
  if (pin_is_reserved(port, pin)) {
    uart_writeln("ERR RESERVED_PIN");
    return;
  }

  if (streq(cmd, "MODE")) {
    char *mode = next_token(&cursor);
    if (mode == 0) {
      uart_writeln("ERR BAD_MODE");
      return;
    }
    if (streq(mode, "OUT")) {
      gpio_set_mode(port, pin, 0x2u);
      uart_writeln("OK");
    } else if (streq(mode, "IN")) {
      gpio_set_mode(port, pin, 0x4u);
      uart_writeln("OK");
    } else if (streq(mode, "ANALOG")) {
      gpio_set_mode(port, pin, 0x0u);
      uart_writeln("OK");
    } else {
      uart_writeln("ERR BAD_MODE");
    }
    return;
  }

  if (streq(cmd, "WRITE")) {
    char *value = next_token(&cursor);
    if (value == 0) {
      uart_writeln("ERR BAD_VALUE");
      return;
    }
    gpio_write_pin(port, pin, streq(value, "1") || streq(value, "HIGH"));
    uart_writeln("OK");
    return;
  }

  if (streq(cmd, "READ")) {
    uart_write("OK ");
    uart_putc(gpio_read_pin(port, pin) ? '1' : '0');
    uart_write("\r\n");
    return;
  }

  if (streq(cmd, "TOGGLE")) {
    const bool high = gpio_read_pin(port, pin);
    gpio_write_pin(port, pin, !high);
    uart_writeln("OK");
    return;
  }

  uart_writeln("ERR BAD_CMD");
}

static void clock_init(void)
{
  RCC_CR |= 1u;
  while ((RCC_CR & (1u << 1)) == 0u) {
  }
  RCC_CFGR = 0u;
}

static void gpio_init(void)
{
  RCC_APB2ENR |= (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 14);

  AFIO_MAPR = (1u << 2) | (2u << 24);

  gpio_set_mode('B', 6u, 0xAu);
  gpio_set_mode('B', 7u, 0x4u);
  gpio_set_mode('C', 13u, 0x2u);
  gpio_write_pin('C', 13u, true);
}

static void uart_init(void)
{
  USART_BRR = 0x0045u;
  USART_CR1 = (1u << 13) | (1u << 3) | (1u << 2);
}

int main(void)
{
  clock_init();
  gpio_init();
  uart_init();

  SYST_RVR = 8000u - 1u;
  SYST_CVR = 0u;
  SYST_CSR = 0x7u;

  uart_writeln("BOOT C8T6-GPIO-STATION 1");

  char line[96];
  uint8_t pos = 0u;
  uint32_t last_blink = g_ms;

  while (1) {
    const int c = uart_getc_nonblock();
    if (c >= 0) {
      if (c == '\r' || c == '\n') {
        if (pos > 0u) {
          line[pos] = '\0';
          handle_line(line);
          pos = 0u;
        }
      } else if (pos < (uint8_t)(sizeof(line) - 1u)) {
        line[pos++] = (char)c;
      }
    }

    if ((g_ms - last_blink) >= 1000u) {
      last_blink = g_ms;
      gpio_write_pin('C', 13u, !gpio_read_pin('C', 13u));
    }

    delay_ms(1u);
  }
}
