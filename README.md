# Software S/PDIF audio output for Raspberry Pi

## Description

This is a Linux kernel module that outputs an audio stream in the S/PDIF format. The module is an ALSA sound card driver. It includes a software encoder to generate the S/PDIF stream and uses the I2S interface present in the BCM2708 SOC to transmit the S/PDIF stream.

The code is still quite experimental. Currently, a sampling rate of 44100 Hz is supported and a resolution of 16 bits or 24 bits. No sub code is inserted into the S/PDIF stream.

## Hardware

Since the S/PDIF stream is generated in software, no special encoder chip is needed. Just connect a S/PDIF transmitter (electrical or optical) to the PCM_DOUT pin (Header 5, pin 6: GPIO31).

## Using the analog video output RCA connector

As shown below, the Raspberry Pi can be modified to reuse the analog video connector as an electrical S/PDIF output.

First, I have cut off the PCB track between the center connector and the Soc.

![RPi Mod Top](https://raw.githubusercontent.com/kiffie/rpi-i2s-spdif/master/doc/rpi_mod_top.jpg)

Maybe, cutting of this PCB track is not needed. However, I do not know how the inactive analog video output behaves.

Second, a 100 nF capacitor should be connected between the PCM_DOUT pin and the center connector of the video RCA connector. This is shown in the image below.

![RPi Mod Bottom](https://raw.githubusercontent.com/kiffie/rpi-i2s-spdif/master/doc/rpi_mod_bottom.jpg)

Instead of performing this modification, an optical transmitter module can be connected to the PCM_DOUT pin in order connect the Raspberry Pi with a optical fiber.

## Status

The drivers works on a Raspberry Pi Model B having the P5 header when connected to a simple audio DAC with an S/PDIF input. Not clear if more complex devices like AV receivers expect the sub code be present in the S/PDIF stream.

Adaptation of the driver to the newer models having a 40 pin IO connector should be possible. It appears that the I2S interface is still available but the driver software must probably map it to different GPIO pins. The 40 pin header has GPIO21 available on pin 40. Thus, PCM_DOUT shoule be mapped to GPIO21 (rather than GPIO31) for the newer models.

## Installation

The module can be compiled (either on the PC or on the RPI) using the Makefile after adapting the path to the kernel headers.

The compiled module `bcm2708-i2s-spdif.ko` may be copied e.g. to `/lib/modules/$(uname -r)/updates/bcm2708-i2s-spdif.ko`


