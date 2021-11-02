# UCI cBP (continuous Blood Pressure) demo
Link : https://pypi.org/project/uci-cbp-demo/0.2.5/

This project uses [MetaWear Python SDK](https://github.com/mbientlab/MetaWear-SDK-Python) and works in only Linux environment.

# How to install

The API metawear depends on [Warble](https://github.com/mbientlab/Warble) and Warble requires 3 installations

[bluez](www.bluez.org).   
Boost headers.   
GNU with a C++ compiler that support C++ 14.  

To use `bluez`, the device files should be installed and we have to install bluez. You can also use `apt` to get bluez-util as [Metawear tutorial on Linux](https://mbientlab.com/tutorials/Linux.html), but this way can cause some error. So, you have to use a makefile in order to install bluez.

## On Ubuntu
    $ sudo apt-get update
    $ sudo apt-get install -y libusb-dev libdbus-1-dev libglib2.0-dev libudev-dev libical-dev libreadline-dev
    $ tar xvf bluez-5.40.tar.xz
    $ cd bluez-5.40
    $ ./configure
    $ make
    $ sudo make install
    $ sudo apt-get install build-essential
    $ sudo apt-get install libboost-all-dev
    $ sudo apt-get install libbluetooth-dev
    $ pip -r install requirements.txt
    
## On Arch Linux
```shell script
    $ yay -S boost
    $ yay -S bluez
    $ yay -S bluez-util
    # systemctl enable bluetooth
    # systemctl start bluetooth
    $ bluetoothctl power on
    $ pip -r install requirements.txt
    $ sudo setcap 'cap_net_raw,cap_net_admin+eip' `which python`
    $ sudo setcap 'cap_net_raw,cap_net_admin+eip' `which hciconfig`
    $ sudo setcap 'cap_net_raw,cap_net_admin+eip' `which hcitool`
```

## PyGATTLib
```shell script
pip3 download gattlib
tar xvzf ./gattlib-0.20150805.tar.gz
cd gattlib-0.20150805/
sed -ie 's/boost_python-py34/boost_python3/' setup.py
pip3 install .
```
    
# Usage
If you execute this code, after finding the bluetooth device near our device, you can choose a device which you want to connect.

![original](https://media.github.uci.edu/user/1629/files/80d5cf80-b2ed-11e9-9451-93a25e901b81)
