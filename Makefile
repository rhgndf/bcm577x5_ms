obj-m := bcm577x5_ms.o

CFLAGS_bcm577x5_ms.o := -g
#LDFLAGS_bcm577x5_ms.ko := -Map=bcm.map
#ldflags-m := -Map=bcm.map
KBUILD_LDFLAGS_MODULE := -Map=/home/user/bcmms/bcmms/bcm.map
 
all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
