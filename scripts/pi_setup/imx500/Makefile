# Ubuntu 24.04 raspi 커널(6.8)용 IMX500(AI 카메라) 드라이버 외부 빌드.
# 소스: raspberrypi/linux rpi-6.6.y (imx500.c는 imx500-ubuntu-6.8.patch 적용본)
obj-m += imx500.o spi-rp2040-gpio-bridge.o
KDIR ?= /lib/modules/$(shell uname -r)/build
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules
	cpp -nostdinc -undef -x assembler-with-cpp -I. -I$(KDIR)/include imx500-pi5-overlay.dts > imx500-pi5.pp.dts
	dtc -q -@ -I dts -O dtb -o imx500-pi5.dtbo imx500-pi5.pp.dts
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	rm -f imx500-pi5.pp.dts imx500-pi5.dtbo
