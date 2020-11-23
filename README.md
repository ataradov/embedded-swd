# embedded-swd
Embedded implementation of the SWD interface. This is just an example of implementing
SWD interface in the MCU and programming another MCU using it.

### Raspberry Pi Example

Example for the Raspberry Pi is a complete code that can be compiled into a running
executable (just run 'make'). Root privileges are requred to run the resulting executable,
since it uses direct access to the GPIO controller.

By default target connections are SWCLK - GPIO2 (pin 3), SWDIO - GPIO3 (pin 5).
This can be adjusted in the dap_config.h file.

Note that the code will run unmodified on RPi 2B+, for older models you will need
to ajdust BCM_GPIO_BASE definition in the hal_gpio.h file.


