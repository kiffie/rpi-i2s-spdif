# Software S/PDIF audio output for Raspberry Pi

## Description

This is a Linux kernel module that outputs an audio stream in the S/PDIF format. The module is an ALSA sound card driver. It includes a software encoder to generate the S/PDIF stream and uses the I2S interface present in the BCM2708 SOC to transmit the S/PDIF stream.

The driver supports multiple sampling rates: 44100, 48000, 96000 and 192000. The channel status bits in the S/PDIF blocks are set according to the sampling rate.

The driver has been successfully tried on a Raspberry Pi Model B (with P5 header), Raspberry Pi Zero W and Raspberry Pi 3 Model B when connected to a simple audio DAC with an S/PDIF input. The driver has been reported to work on a Raspberry Pi 1 Model B combined with a SONY STR-DB840 AV receiver.

## Installation

### Manual compiling and installing

The module can be compiled (either on the PC or on the RPI) using the Makefile after adapting the path to the kernel headers.

The compiled module `bcm2708-i2s-spdif.ko` may be copied e.g. to `/lib/modules/$(uname -r)/updates/bcm2708-i2s-spdif.ko`. The original driver must be blacklisted. The Makefile has targets for both installing and blacklisting.

In `/boot/config.txt`, the i2s part of the device tree must be enabled.

```
dtparam=i2s=on
```

The above steps can be done like so:

```sh
sudo apt install raspberrypi-kernel-headers
make
sudo make install
sudo make blacklist
sudo vi /boot/config.txt # edit to enable i2s
sudo reboot
```

### DKMS

The module can be intalled with DKMS, too. The following commands must be executed from a root shell.

```sh
apt install dkms raspberrypi-kernel-headers
cd /usr/src
git clone https://github.com/kiffie/rpi-i2s-spdif.git rpi_i2s_spdif-1
dkms install rpi_i2s_spdif/1
vi /boot/config.txt # edit to enable i2s
# blacklist the original driver
echo "blacklist snd_soc_bcm2835_i2s" > /etc/modprobe.d/blacklist-snd_soc_bcm2835_i2s.conf
```

## Pinout

Since the S/PDIF stream is generated in software, no special encoder chip is needed. Just connect an S/PDIF transmitter (electrical or optical) to the PCM_DOUT pin.

| Model | SPDIF output pin |
|-|-|
| Raspberry Pi 1 Model B <br> (28-pin header + P5 header) | header P5, pin 6: GPIO 31 |
| Raspberry Pi 1 Model B+ and later <br> (40-pin header) | pin 40: GPIO 21|

## Continuous SPDIF signal output

There are module parameters to control whether an SPDIF signal containing zero
samples (silence) is outputted when there is no ALSA audio stream (continuous
SPDIF signal).

| Parameter | Default | Description |
|-|-|-|
| continuous | N | Enable continuous SPDIF signal output. Modifications of this parameter via the /sys file system will have an effect the next time when an ASLA stream is stopped. |
| initial_rate | 44100 | Sample rate of the continuous SPDIF signal after loading the module before the first ALSA stream is outputted. Modification of this parameter via the /sys filesystem does not have an effect. |

The sample rate of the continuous SPDIF signal corresponds to the sample rate of the previously outputted ALSA stream.

An example configuration is in file `bcm2708-i2s-spdif.conf`, which can be
copied to `/etc/modprobe.d`.

## Hardware hack for Pi 1 Model B: using the analog video output RCA connector

As shown below, old models of the Raspberry Pi can be modified to reuse the analog video connector as an electrical S/PDIF output.

First, I cut off the PCB track between the center connector and the Soc.

![RPi Mod Top](https://raw.githubusercontent.com/kiffie/rpi-i2s-spdif/master/doc/rpi_mod_top.jpg)

Maybe, cutting of this PCB track is not needed. However, I do not know how the inactive analog video output behaves.

Second, a 100 nF capacitor should be connected between the PCM_DOUT pin and the center connector of the video RCA connector. This is shown in the image below.

![RPi Mod Bottom](https://raw.githubusercontent.com/kiffie/rpi-i2s-spdif/master/doc/rpi_mod_bottom.jpg)

Instead of performing this modification, an optical transmitter module can be connected to the PCM_DOUT pin in order connect the Raspberry Pi to an optical fiber.
