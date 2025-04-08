<!-- The markdown-toc utilitity is used to generate the Table of Contents -->
<!-- Installation: npm install -g markdown-toc -->
<!-- Usage: markdown-toc -i README.md -->

<!-- toc -->
  * [Install Ubuntu Server LTS 24.04.02](#install-ubuntu-server-lts-240402)
  * [Connect to the WiFi network](#connect-to-the-wifi-network)
- [RPI Performance Testing](#rpi-performance-testing)
  - [I/O Stress Test](#io-stress-test)
  - [Stress Test the CPU and RAM](#stress-test-the-cpu-and-ram)
  - [Network Stress Test](#network-stress-test)
  - [Combined Stress Test](#combined-stress-test)
  - [Monitor System Performance](#monitor-system-performance)

<!-- tocstop -->

## Install Ubuntu Server LTS 24.04.02

### Prepare SD card
1. Install on SD card and boot from SD
1. Enable ssh:
    ```bash
    sudo nano /etc/ssh/sshd_config.d/50-cloud-init.conf
    PasswordAuthentication yes
    ```
1. ssh-copy-id on PC
2. nano ~/.ssh/config on PC
3. Install Docker
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sh get-docker.sh

   sudo usermod -aG docker $USER
   newgrp docker
   docker run hello-world
   sudo systemctl enable --now docker
   ```
4. This power supply is not capable of supplying 5A; power to peripherals will be restricted
   ```bash
   sudo rpi-eeprom-config --edit
   # add at the end of the file:  PSU_MAX_CURRENT=5000
   
   sudo nano /boot/firmware/config.txt
   # add at the end of the file: usb_max_current_enable=1

   sudo reboot
   ```
5. udev rules for LIDARs
   ```bash
   # RPI LIDAR
   sudo cp kpi-rover.rules /etc/udev/rules.d/
   sudo service udev reload
   sudo service udev restart
   sudo udevadm control --reload && sudo udevadm trigger
   ```
6. sudo apt install fastdds-tools
   

### Clone SD card on SSD
1. Clone SD card to ssd
    ```bash
    sudo dd if=/dev/mmcblk0 of=/dev/nvme0n1 bs=4M status=progress
    ```
1. Resize ssd
    ```bash
    sudo growpart /dev/nvme0n1 2
    sudo resize2fs /dev/nvme0n1p2
    ```
1. Make sure boot from ssd is enabled
    ```bash
    sudo apt install raspi-config
    sudo raspi-config
    ```
1. Install Docker
    ```bash

    ```

## Connect to the WiFi network

**Install the connman utility:**
```bash
sudo apt install connman
```

**Configure access to your WiFi network:**
```bash
connmanctl
connmanctl> tether wifi off
connmanctl> enable wifi
connmanctl> scan wifi
connmanctl> services
connmanctl> agent on
connmanctl> connect wifi_f45eab2f1ee1_6372797774616c_managed_psk (Replace with your network)
connmanctl> quit
```

# RPI Performance Testing
##  I/O Stress Test

**Install fio:**
```bash
sudo apt install fio
```

**Run the test:**
```bash
fio --name=write_test --ioengine=libaio --iodepth=1 --rw=write --bs=4k --direct=1 --size=512M --numjobs=4 --runtime=60 --group_reporting

fio --name=read_test --ioengine=libaio --iodepth=1 --rw=read --bs=4k --direct=1 --size=512M --numjobs=4 --runtime=60 --group_reporting
```
This writes and reads 512MB of data in 4KB blocks to test the I/O system.

## Stress Test the CPU and RAM

**Install the tool:**
```bash
sudo apt update
sudo apt install stress-ng -y
```

**Run the CPU and RAM stress test:**
```bash
stress-ng --cpu 4 --vm 2 --vm-bytes 512M --timeout 300s
```
This tests the CPU with 4 cores and allocates 2 virtual memory workers using 512MB RAM each for 5 minutes.

## Network Stress Test

Use iperf3 to measure network bandwidth.

**Install iperf3:**
```bash
sudo apt install iperf3
```
**Run the test:**
* Set up a server on another machine:
  ```bash
  iperf3 -s
  ```
* Run a client test on the Raspberry Pi:
  ```bash
  iperf3 -c <server-ip> -t 60
  ```

This tests the network throughput for 60 seconds.

## Combined Stress Test

For a full system load, run all tests simultaneously for 1 hour. For example:
```bash
stress-ng --cpu 4 --vm 2 --vm-bytes 75% --timeout 1h &
fio --name=write_test --ioengine=libaio --iodepth=1 --rw=write --bs=4k --direct=1 --size=1G --numjobs=4 --runtime=3600 --time_based --group_reporting &

iperf3 -c <server-ip> -t 3600 &
```

## Monitor System Performance

While running these tests, monitor the Raspberry Pi’s performance using htop.

**Install htop:**
```bash
sudo apt install htop
```
**Monitor system resources:**
```bash
htop
```

**Enable CPU Temperature Display in htop**

By default, htop does not show CPU temperature, but you can add it manually:

1. Open htop:
    ```bash
    htop
    ```
2. Press `F2` (Setup).
3. Use the arrow keys to navigate to Display options -> Global Options -> Allow show CPU temperature
4. Press `Space` to activate.
5. Press `F10` to save and exit.

Now, htop should display the CPU temperature in the process list.



