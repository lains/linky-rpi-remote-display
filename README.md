# Introduction

This repository contains python software dedicated to Raspberry Pi for Linky TIC decoding.

For this, I used an LCD display shield for Raspberry Pi. My selection was a Nokia-based monochrome shield like the one described here: https://www.tomshardware.com/news/nokia-cell-phone-screen-finds-second-life-as-raspberry-pi-shield
but other similar shields would do as well. The current python code is using Pillow as a graphical display library, and the Adafruit_Nokia_LCD library to drive the LCD display via an SPI link.

Also I home-made a Linky serial adapter including optocoupler and transitor-based level shifter. Examples can be found here: https://faire-ca-soi-meme.fr/domotique/2016/09/12/module-teleinformation-tic/

Once the serial adapter is plugged into the Raspberry Pi, a new serial device will be accessible from Linux (like /dev/ttyUSB0), this is how we will get the TIC information from the Linky meter.

We will then use the LCD to display the withdrawn power in real-time, together with a history of the last few minutes of measurements. To get more exhaustive data, I asked my electricity provider to have my Linky meter switched to _TIC standard_ mode instead of _TIC historique_, this mode is expected by the python code, as it opens the serial port in 9600 bauds mode.
