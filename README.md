# Real-Time Controller

## This firmware is designed for handling following tasks:
- If the User button is pressed and kept pressed for 5 seconds on power-on, boot into the bootloader and wait for firmware update.

- If the User button is clicked (pressed and released) 3 times in (250ms to 1s), then read the ADC value (could be any value) and write it into UART

- Every 600ms toggle the User LED

- If the User button is pressed and kept pressed for 5 seconds during run-time, the device goes to deep sleep mode (shutdown mode). Before going to sleep mode, the User LED blinks 5 times at 5Hz, then the User LED turns off and the MCU goes to sleep.

- If the MCU is in sleep mode, it should be possible to wake it up by clicking the User button.
When MCU wakes up User LED blinks 5 times at 10Hz

- If the MCU is in sleep mode and no User button is pressed, it should wake up every 10 seconds, blink the User LED 3 times at 5Hz, and go back to sleep

The project was successfully tested on STs NUCLEO-F446 board.
