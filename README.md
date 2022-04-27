# BMP280_Linux_Driver

Linux Device Driver for BMP280 Preassure sensor using I2C interface. Driver is tested on Raspberry Pi 3B running ubuntu.

# Compiling and loading the driver

Run the following commands

```
make all

sudo insmod driver.ko

```
Reading sensor value

```
sudo cat /dev/etx_device
```
