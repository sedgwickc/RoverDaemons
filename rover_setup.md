BeagleBone Blue Setup
=====================

Connect to WiFi
---------------
To connect to your Wireless network type the following command in the terminal
window:

```
sudo connmanctl

connmanctl> enable wifi

Enabled wifi

connmanctl> scan wifi

Scan completed for wifi

connmanctl> services
*** list of available networks by name ****
connmanctl> agent on

Agent registered

connmanctl> connect $NETWORK_NAME

Passphrase? xxxxxxxxxxx

connected $NETWROK_NAME

connmanctl> quit
```

You should now be connected to your local wifi. You can check that you have an
IP address by typing the following in the terminal window:

```
ifconfig –a
```

Create rover user
-----------------
 - create a new user called rover 
```
 adduser --home /home/rover --shell /bin/bash rover
```
 - add new rover user to required groups
```
sudo usermod -a -G debian,adm,kmem,dialout,cdrom,floppy,audio,dip,video,plugdev,users,systemd-journal,i2c,bluetooth,netdev,cloud9ide,xenomai,weston-launch,tisdk,spi,admin,pwm,gpio rover
```

