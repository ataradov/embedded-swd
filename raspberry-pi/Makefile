CFLAGS += -W -Wall -Wextra -O3 -std=gnu11
CFLAGS += -fno-diagnostics-show-caret

SRC = \
  main.c \
  dap.c \
  dap_target.c \

HDR = \
  hal_gpio.h \
  dap_config.h \
  dap.h \

all: $(SRC) $(HDR)
	gcc $(CFLAGS) $(SRC) -o dap_test


