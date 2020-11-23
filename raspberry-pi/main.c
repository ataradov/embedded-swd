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

//-----------------------------------------------------------------------------
#include <err.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal_gpio.h"
#include "dap.h"

//-----------------------------------------------------------------------------
volatile bcm_gpio_t *BCM_GPIO;

//-----------------------------------------------------------------------------
static uint8_t buffer[TARGET_FLASH_SIZE];

//-----------------------------------------------------------------------------
static void target_connect(void)
{
  dap_init();
  dap_connect();
  dap_reset_link();
  dap_target_prepare();
  dap_target_select();
}

//-----------------------------------------------------------------------------
static void target_id(void)
{
  uint32_t idcode  = dap_read_idcode();
  uint32_t dsu_did = dap_read_word(0x41002118/*DSU_DID*/);

  printf("IDCODE  = 0x%08x\n", idcode);
  printf("DSU_DID = 0x%08x\n", dsu_did);

  if (idcode != 0x0bc11477)
  {
    printf("Error: unexpected IDCODE value\n");
    exit(1);
  }
}

//-----------------------------------------------------------------------------
static void target_erase(void)
{
  dap_target_erase();
}

//-----------------------------------------------------------------------------
static void target_read(char *name)
{
  int ptr = 0;
  int addr = 0;
  int fd, rsize;

  while (ptr < TARGET_FLASH_SIZE)
  {
    dap_target_read_page(addr, &buffer[ptr]);

    addr += TARGET_PAGE_SIZE;
    ptr += TARGET_PAGE_SIZE;

    printf(".");
    fflush(stdout);
  }

  fd = open(name, O_WRONLY | O_TRUNC | O_CREAT, 0644);
  if (fd < 0)
    err(1, "open()");

  rsize = write(fd, buffer, sizeof(buffer));
  if (rsize < 0)
    err(1, "write()");

  close(fd);
}

//-----------------------------------------------------------------------------
static void target_write(char *name)
{
  int ptr = 0;
  int addr = 0;
  int fd, rsize;
  uint8_t tmp[TARGET_PAGE_SIZE];

  fd = open(name, O_RDONLY);
  if (fd < 0)
    err(1, "open()");

  rsize = read(fd, buffer, sizeof(buffer));
  if (rsize < 0)
    err(1, "read()");

  close(fd);

  while (rsize)
  {
    int size = (rsize < TARGET_PAGE_SIZE) ? rsize : TARGET_PAGE_SIZE;

    memset(tmp, 0xff, sizeof(tmp));
    memcpy(tmp, &buffer[ptr], size);

    dap_target_write_page(addr, tmp);

    rsize -= size;
    addr += size;
    ptr += size;

    printf(".");
    fflush(stdout);
  }
}

//-----------------------------------------------------------------------------
static void target_verify(char *name)
{
  int ptr = 0;
  int addr = 0;
  int fd, rsize;
  uint8_t tmp[TARGET_PAGE_SIZE];

  fd = open(name, O_RDONLY);
  if (fd < 0)
    err(1, "open()");

  rsize = read(fd, buffer, sizeof(buffer));
  if (rsize < 0)
    err(1, "read()");

  close(fd);

  while (rsize)
  {
    int size = (rsize < TARGET_PAGE_SIZE) ? rsize : TARGET_PAGE_SIZE;

    dap_target_read_page(addr, tmp);

    for (int i = 0; i < size; i++)
    {
      if (tmp[i] != buffer[ptr + i])
      {
        printf("Verification failed at 0x%x: expected 0x%02x, got 0x%02x\n", ptr + i, buffer[ptr + i], tmp[i]);
        exit(1);
      }
    }

    rsize -= size;
    addr += size;
    ptr += size;

    printf(".");
    fflush(stdout);
  }
}

//-----------------------------------------------------------------------------
static void target_disconnect(void)
{
  dap_target_deselect();
  dap_disconnect();
}

//-----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  bool cmd_erase  = false;
  bool cmd_read   = false;
  bool cmd_write  = false;
  bool cmd_verify = false;
  char *file_name;

  if (argc < 2)
  {
    printf("Usage: dap_test [erase | read | write | verify] [file_name]\n");
    exit(1);
  }

  if (0 == strcmp(argv[1], "erase"))
    cmd_erase = true;
  else if (0 == strcmp(argv[1], "read"))
    cmd_read = true;
  else if (0 == strcmp(argv[1], "write"))
    cmd_write = true;
  else if (0 == strcmp(argv[1], "verify"))
    cmd_verify = true;
  else
  {
    printf("Error: invalid command '%s'\n", argv[1]);
    exit(1);
  }

  if ((cmd_read | cmd_write | cmd_verify) && (argc < 3))
  {
    printf("Error: file name is expected\n");
    exit(1);
  }

  file_name = argv[2];

  bcm_gpio_init();

  target_connect();
  target_id();

  if (cmd_erase)
    target_erase();

  if (cmd_read)
    target_read(file_name);

  if (cmd_write)
    target_write(file_name);

  if (cmd_verify)
    target_verify(file_name);

  target_disconnect();

  bcm_gpio_close();

  printf("\n");

  return 0;
}


