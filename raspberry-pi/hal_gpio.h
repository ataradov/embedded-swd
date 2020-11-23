/*
 * Copyright (c) 2020, Alex Taradov <alex@taradov.com>
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

#ifndef _HAL_GPIO_H_
#define _HAL_GPIO_H_

/*- Includes ----------------------------------------------------------------*/
#include <err.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

/*- Definitions -------------------------------------------------------------*/
// Note: 0x3f000000 for RPi 2B+ and above, 0x20000000 for older models
#define BCM_GPIO_BASE   0x3f200000
#define BCM_GPIO_SIZE   4096

enum
{
  HAL_GPIO_ALT_0 = 4,
  HAL_GPIO_ALT_1 = 5,
  HAL_GPIO_ALT_2 = 6,
  HAL_GPIO_ALT_3 = 7,
  HAL_GPIO_ALT_4 = 3,
  HAL_GPIO_ALT_5 = 2,
};

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  uint32_t GPFSEL[6];
  uint32_t reserved0;
  uint32_t GPSET[2];
  uint32_t reserved1;
  uint32_t GPCLR[2];
  uint32_t reserved2;
  uint32_t GPLEV[2];
  uint32_t reserved3;
  uint32_t GPEDS[2];
  uint32_t reserved4;
  uint32_t GPREN[2];
  uint32_t reserved5;
  uint32_t GPFEN[2];
  uint32_t reserved6;
  uint32_t GPHEN[2];
  uint32_t reserved7;
  uint32_t GPLEN[2];
  uint32_t reserved8;
  uint32_t GPAREN[2];
  uint32_t reserved9;
  uint32_t GPAFEN[2];
  uint32_t reserved10;
  uint32_t GPPUD;
  uint32_t GPPUDCLK[2];
} bcm_gpio_t;

/*- Variables ---------------------------------------------------------------*/
extern volatile bcm_gpio_t *BCM_GPIO;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline void bcm_gpio_init(void)
{
  int fd;

  fd = open("/dev/mem", O_RDWR | O_SYNC);

  if (fd < 0)
    err(0, "can't open /dev/mem");

  BCM_GPIO = (volatile bcm_gpio_t *)mmap(NULL, BCM_GPIO_SIZE, PROT_READ | PROT_WRITE,
      MAP_SHARED, fd, BCM_GPIO_BASE);

  close(fd);

  if (MAP_FAILED == BCM_GPIO)
    err(0, "mmap() error");

  (void)bcm_gpio_init;
}

//-----------------------------------------------------------------------------
static inline void bcm_gpio_close(void)
{
  munmap((void *)BCM_GPIO, BCM_GPIO_SIZE);
  (void)bcm_gpio_close;
}

//-----------------------------------------------------------------------------
// Note: The code manipulating GPPUDCLK[0] is not atomic, bad things may happen
// if it is interrupted by some other code manipulating the same registers.

#define HAL_GPIO_PIN(name, index)						\
  _Static_assert(index < 28, "GPIO index must be 0-27");			\
										\
  static inline void HAL_GPIO_##name##_set(void)				\
  {										\
    BCM_GPIO->GPSET[0] = (1 << index);						\
    (void)HAL_GPIO_##name##_set;						\
  }										\
										\
  static inline void HAL_GPIO_##name##_clr(void)				\
  {										\
    BCM_GPIO->GPCLR[0] = (1 << index);						\
    (void)HAL_GPIO_##name##_clr;						\
  }										\
										\
  static inline void HAL_GPIO_##name##_toggle(void)				\
  {										\
    if (BCM_GPIO->GPLEV[0] & (1 << index))					\
      BCM_GPIO->GPCLR[0] = (1 << index);					\
    else									\
      BCM_GPIO->GPSET[0] = (1 << index);					\
    (void)HAL_GPIO_##name##_toggle;						\
  }										\
										\
  static inline void HAL_GPIO_##name##_write(int value)				\
  {										\
    if (value)									\
      BCM_GPIO->GPSET[0] = (1 << index);					\
    else									\
      BCM_GPIO->GPCLR[0] = (1 << index);					\
    (void)HAL_GPIO_##name##_write;						\
  }										\
										\
  static inline void HAL_GPIO_##name##_in(void)					\
  {										\
    int idx = index / 10;							\
    int pos = (index % 10) * 3;							\
    BCM_GPIO->GPFSEL[idx] &= ~(7 << pos);					\
    (void)HAL_GPIO_##name##_in;							\
  }										\
										\
  static inline void HAL_GPIO_##name##_out(void)				\
  {										\
    int idx = index / 10;							\
    int pos = (index % 10) * 3;							\
    BCM_GPIO->GPFSEL[idx] &= ~(7 << pos);					\
    BCM_GPIO->GPFSEL[idx] |= (1 << pos);					\
    (void)HAL_GPIO_##name##_out;						\
  }										\
										\
  static inline void HAL_GPIO_##name##_alt(int alt)				\
  {										\
    int idx = index / 10;							\
    int pos = (index % 10) * 3;							\
    BCM_GPIO->GPFSEL[idx] &= ~(7 << pos);					\
    BCM_GPIO->GPFSEL[idx] |= (alt << pos);					\
    (void)HAL_GPIO_##name##_alt;						\
  }										\
										\
  static inline void HAL_GPIO_##name##_pullup(void)				\
  {										\
    BCM_GPIO->GPPUD = 2;							\
    for (int i = 0; i < 150; i++) asm("nop");					\
    BCM_GPIO->GPPUDCLK[0] |= (1 << index);					\
    for (int i = 0; i < 150; i++) asm("nop");					\
    BCM_GPIO->GPPUDCLK[0] &= ~(1 << index);					\
    (void)HAL_GPIO_##name##_pullup;						\
  }										\
										\
  static inline void HAL_GPIO_##name##_pulldown(void)				\
  {										\
    BCM_GPIO->GPPUD = 1;							\
    for (int i = 0; i < 150; i++) asm("nop");					\
    BCM_GPIO->GPPUDCLK[0] |= (1 << index);					\
    for (int i = 0; i < 150; i++) asm("nop");					\
    BCM_GPIO->GPPUDCLK[0] &= ~(1 << index);					\
    (void)HAL_GPIO_##name##_pulldown;						\
  }										\
										\
  static inline void HAL_GPIO_##name##_pulloff(void)				\
  {										\
    BCM_GPIO->GPPUD = 0;							\
    for (int i = 0; i < 150; i++) asm("nop");					\
    BCM_GPIO->GPPUDCLK[0] |= (1 << index);					\
    for (int i = 0; i < 150; i++) asm("nop");					\
    BCM_GPIO->GPPUDCLK[0] &= ~(1 << index);					\
    (void)HAL_GPIO_##name##_pulloff;						\
  }										\
										\
  static inline int HAL_GPIO_##name##_read(void)				\
  {										\
    return ((BCM_GPIO->GPLEV[0] & (1 << index)) > 0);				\
    (void)HAL_GPIO_##name##_read;						\
  }										\

#endif // _HAL_GPIO_H_

