# Real-Time Controller

## This firmware designed for handling followin tasks:
- If User button is pressed and kept pressed for 5 seconds on power-on, boot into bootloader and wait for firmware update.

- If User button is clicked (pressed and released) 3 times in (250ms to 1s), then read ADC value (could be any value) and write it into UART

- Every 600ms toggle the User LED

- If User button is pressed and kept pressed for 5 seconds during run-time, device goes to deep sleep mode (shutdown mode). Before going to sleep mode, User LED blinks 5 times at 5Hz, then User LED turns off and MCU goes to sleep.

- If MCU is in sleep mode, it should be possible to wake it up by once clicking the User button.
When MCU wakes up User LED blinks 5 times at 10Hz

- If MCU is in sleep mode and no User button is pressed, it should wake up every 10 seconds, blink the User LED 3 times at 5Hz, and go back to sleep

The project successfully tested on STs NUCLEO-F446 board.