#sudo rmmod sdhci-pci
sudo modprobe memstick
sudo rmmod bcm577x5_ms
make clean
make -j
sudo insmod ./bcm577x5_ms.ko debug=5

