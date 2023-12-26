sudo modprobe memstick
sudo rmmod bcm577x5_ms
make clean
make -j $(nprocs)
sudo insmod ./bcm577x5_ms.ko

