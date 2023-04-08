
obj-m = bcm2708-i2s-spdif.o
bcm2708-i2s-spdif-objs = bcm2708-i2s-spdif-driver.o spdif-encoder.o

MY_BUILDDIR=/lib/modules/$(shell uname -r)/build
BLACKLIST_FILE=/etc/modprobe.d/blacklist-snd_soc_bcm2835_i2s.conf

all:
	make -C $(MY_BUILDDIR) M=$(PWD) EXTRA_CFLAGS+=-I$(PWD)/include modules

clean:
	make -C $(MY_BUILDDIR) M=$(PWD) clean
	rm -f Module.markers modules.order

install:
	mkdir -p -m755 /lib/modules/$(shell uname -r)/updates
	install -m644 bcm2708-i2s-spdif.ko /lib/modules/$(shell uname -r)/updates
	depmod -a

blacklist:
	echo "blacklist snd_soc_bcm2835_i2s" > $(BLACKLIST_FILE)
	chmod 644 $(BLACKLIST_FILE)
