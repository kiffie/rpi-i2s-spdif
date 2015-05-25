
obj-m = bcm2708-i2s-spdif.o
bcm2708-i2s-spdif-objs = bcm2708-i2s-spdif-driver.o spdif-encoder.o

#MY_BUILDDIR=/lib/modules/$(shell uname -r)/build
MY_BUILDDIR=/data/scratch/raspberry_pi/linux-rpi-3.18.13

all:
	make -C $(MY_BUILDDIR) M=$(PWD) EXTRA_CFLAGS+=-I$(PWD)/include modules

clean:
	make -C $(MY_BUILDDIR) M=$(PWD) clean
	rm -f Module.markers modules.order

