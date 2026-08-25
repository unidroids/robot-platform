# HMI - Human Machine Interface

HMI je implementováno na dotykovém displeji mobilního telefonu s Androidem, který je připojený k robotu USB kabelem. Pro komunikaci se vyuížívá adb (Android Debug Bridge).

## Instalace

sudo apt install adb

user@robot:~$ apt policy adb
adb:
  Installed: (none)
  Candidate: 1:10.0.0+r36-9
  Version table:
     1:10.0.0+r36-9 500
        500 http://ports.ubuntu.com/ubuntu-ports jammy/universe arm64 Packages

user@robot:~$ adb version
Android Debug Bridge version 1.0.41
Version 28.0.2-debian
Installed as /usr/lib/android-sdk/platform-tools/adb

user@robot:~$ adb devices
* daemon not running; starting now at tcp:5037
* daemon started successfully
List of devices attached
120453749J000566        no permissions (user in plugdev group; are your udev rules wrong?); see [http://developer.android.com/tools/device.html]

lsusb

Bus 001 Device 004: ID 1782:5d31 Spreadtrum Communications Inc. Infinix SMART 8


echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1782", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-robot-hmi.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
adb kill-server


## Identifikace

user@robot:/etc/udev/rules.d$ adb devices
List of devices attached
120453749J000566        device

user@robot:/etc/udev/rules.d$

## Připojení

adb -s 120453749J000566 reverse tcp:9000 tcp:9000

adb reverse --list

adb -s 120453749J000566 reverse --remove tcp:9000

