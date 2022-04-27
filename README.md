# GhettoKeyboard

For when your Logitech keyboard decides to randomly die.

This code is written for the stm32f411CE black pill, but it should be easily portable to other MCUs which have the DWC2 USB hardware and a U(S)ART.

The stm32 usb stack is configured as a generic FS HID keyboard. To use this, flash the firmware to the stm32 and connect with a terminal (I used putty) to the uart port at baud 115200. All the incoming uart data is converted to keystrokes and sent over usb.
