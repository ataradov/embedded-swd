/*
 * Copyright (c) 2015, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd21.h"
#include "hal_gpio.h"
#include "dap.h"

//-----------------------------------------------------------------------------
#define PERIOD_FAST     100
#define PERIOD_SLOW     500

HAL_GPIO_PIN(LED,      B, 30)
HAL_GPIO_PIN(BUTTON,   A, 15)
HAL_GPIO_PIN(UART_TX,  A, 22)
HAL_GPIO_PIN(UART_RX,  A, 23)

//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
  uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;

  HAL_GPIO_UART_TX_out();
  HAL_GPIO_UART_TX_pmuxen(PORT_PMUX_PMUXE_C_Val);
  HAL_GPIO_UART_RX_in();
  HAL_GPIO_UART_RX_pmuxen(PORT_PMUX_PMUXE_C_Val);

  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM3;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM3_GCLK_ID_CORE) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  SERCOM3->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
      SERCOM_USART_CTRLA_RXPO(1/*PAD1*/) | SERCOM_USART_CTRLA_TXPO(0/*PAD0*/);

  SERCOM3->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);

  SERCOM3->USART.BAUD.reg = (uint16_t)br;

  SERCOM3->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}

//-----------------------------------------------------------------------------
void uart_putc(char c)
{
  while (!(SERCOM3->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
  SERCOM3->USART.DATA.reg = c;
}

//-----------------------------------------------------------------------------
void uart_puts(char *s)
{
  while (*s)
    uart_putc(*s++);
}

//-----------------------------------------------------------------------------
void uart_puthex(uint32_t v, int size)
{
  char hex[] = "0123456789abcdef";

  for (int i = 0; i < size; i++)
  {
    int offs = ((size - 1) - i) * 4;
    uart_putc(hex[(v >> offs) & 0xf]);
  }
}


//-----------------------------------------------------------------------------
static void sys_init(void)
{
  // Switch to 8MHz clock (disable prescaler)
  SYSCTRL->OSC8M.bit.PRESC = 0;
}

//-----------------------------------------------------------------------------
static void test_swd(void)
{
  dap_init();
  dap_connect();
  dap_reset_link();
  dap_target_prepare();
  dap_target_select();

  uint32_t id = dap_read_idcode();
  uart_puts("\r\nIDCODE = ");
  uart_puthex(id, 8);

  uart_puts("\r\nDSU_DID = ");
  uint32_t dsu_did = dap_read_word(0x41002118/*DSU_DID*/);
  uart_puthex(dsu_did, 8);
  uart_puts("\r\n");

  uart_puts("Erasing entire device... ");
  dap_target_erase();
  uart_puts("done\r\n");


  uint32_t addr = 0x8000;
  uint8_t page_buf[TARGET_PAGE_SIZE];

  uart_puts("Erasing a row device... ");
  dap_target_erase_row(addr);
  uart_puts("done\r\n");

  for (int i = 0; i < TARGET_PAGE_SIZE; i++)
    page_buf[i] = i;

  uart_puts("Writing a page... ");
  dap_target_write_page(addr, page_buf);
  uart_puts("done\r\n");

  for (int i = 0; i < TARGET_PAGE_SIZE; i++) // Clear the buffer for readback test
    page_buf[i] = 0;

  uart_puts("Reading a page... ");
  dap_target_read_page(addr, page_buf);
  uart_puts("done\r\n");

  for (int i = 0; i < TARGET_PAGE_SIZE; i++)
  {
    if (page_buf[i] != i)
    {
      uart_puts("\r\nERROR @ ");
      uart_puthex(i, 2);
      uart_puts("\r\n");
    }
  }

  dap_target_deselect();

  dap_disconnect();
}

//-----------------------------------------------------------------------------
int main(void)
{
  int cnt = 0;

  sys_init();
  uart_init(115200);

  uart_puts("\r\n--- start ---\r\n");

  HAL_GPIO_LED_out();
  HAL_GPIO_LED_clr();

  HAL_GPIO_BUTTON_in();
  HAL_GPIO_BUTTON_pullup();

  while (1)
  {
    if (HAL_GPIO_BUTTON_read())
      cnt = 0;
    else if (cnt < 5001)
      cnt++;

    if (5000 == cnt)
    {
      uart_puts("Test SWD: ");
      test_swd();
      uart_puts("\r\ndone.\r\n");
    }
  }

  return 0;
}
