/*
 * Copyright (c) 2014-2016, Alex Taradov <alex@taradov.com>
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

/*- Includes ----------------------------------------------------------------*/
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "dap.h"

/*- Definitions -------------------------------------------------------------*/
#define DHCSR                  0xe000edf0
#define DEMCR                  0xe000edfc
#define AIRCR                  0xe000ed0c

#define DSU_CTRL_STATUS        0x41002100
#define DSU_DID                0x41002118

#define NVMCTRL_CTRLA          0x41004000
#define NVMCTRL_CTRLB          0x41004004
#define NVMCTRL_PARAM          0x41004008
#define NVMCTRL_INTFLAG        0x41004014
#define NVMCTRL_STATUS         0x41004018
#define NVMCTRL_ADDR           0x4100401c

#define NVMCTRL_CMD_ER         0xa502
#define NVMCTRL_CMD_WP         0xa504
#define NVMCTRL_CMD_EAR        0xa505
#define NVMCTRL_CMD_WAP        0xa506
#define NVMCTRL_CMD_WL         0xa50f
#define NVMCTRL_CMD_UR         0xa541
#define NVMCTRL_CMD_PBC        0xa544
#define NVMCTRL_CMD_SSB        0xa545

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void dap_target_select(void)
{
  // Stop the core
  dap_write_word(DHCSR, 0xa05f0003);
  dap_write_word(DEMCR, 0x00000001);
  dap_write_word(AIRCR, 0x05fa0004);

  // Enable automatic writes
  dap_write_word(NVMCTRL_CTRLB, 0);
}

//-----------------------------------------------------------------------------
void dap_target_deselect(void)
{
  dap_write_word(DEMCR, 0x00000000);
  dap_write_word(AIRCR, 0x05fa0004);
}

//-----------------------------------------------------------------------------
void dap_target_erase(void)
{
  dap_write_word(DSU_CTRL_STATUS, 0x00001f00); // Clear flags
  dap_write_word(DSU_CTRL_STATUS, 0x00000010); // Chip erase
  while (0 == (dap_read_word(DSU_CTRL_STATUS) & 0x00000100));
}

//-----------------------------------------------------------------------------
void dap_target_lock(void)
{
  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_SSB); // Set Security Bit
}

//-----------------------------------------------------------------------------
void dap_target_erase_row(uint32_t addr)
{
  dap_write_word(NVMCTRL_ADDR, addr >> 1);

  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_UR); // Unlock Region
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_ER); // Erase Row
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));
}

//-----------------------------------------------------------------------------
void dap_target_write_page(uint32_t addr, uint8_t *data)
{
  dap_write_word(NVMCTRL_ADDR, addr >> 1);

  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_UR); // Unlock Region
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

  for (int offs = 0; offs < TARGET_PAGE_SIZE; offs += sizeof(uint32_t))
  {
    uint32_t value =
        ((uint32_t)data[offs + 3] << 24) | ((uint32_t)data[offs + 2] << 16) |
        ((uint32_t)data[offs + 1] << 8) | (uint32_t)data[offs];

    dap_write_word(addr + offs, value);
  }
}

//-----------------------------------------------------------------------------
void dap_target_read_page(uint32_t addr, uint8_t *data)
{
  for (int offs = 0; offs < TARGET_PAGE_SIZE; offs += sizeof(uint32_t))
  {
    uint32_t value = dap_read_word(addr + offs);

    data[offs + 0] = (value >> 0) & 0xff;
    data[offs + 1] = (value >> 8) & 0xff;
    data[offs + 2] = (value >> 16) & 0xff;
    data[offs + 3] = (value >> 24) & 0xff;
  }
}

