## Table of Contents

TBD


## Hardware
Pinout and connections for this to work. I personally use a Raspberry pi zero 2 w with zram enabled (not only swapfile). Compilation of ROS is made on an rpi4 and then rsync over to the rpi zero w 2.
I also have a 4g modem for connectivity. I have put the INA226 on the thick red wire to the board (cut).

![DB504](Worx_db504_pcb.png)

General schematic, work in progress. Made from memory - no guarantee it is working!
![Schematic](Worx_db504_schematic.png)

Note code for setting up INA226 is missing at the moment, this needs to be programmed every start and should be done in a i2c module we need to write. This module should also handle the MPU9255 for the IMU data to ROS. TBD


## ROS enviroment
Follow and install https://wiki.openmower.de/index.php?title=System_Image 
But run the following instead of ClemensElflein repo, also note the branch.
```
git clone --recursive -b worx_comms https://github.com/Damme/open_mower_ros
```  
Edit ~/mower_config.sh, set OM_MOWER="Worx" and uncomment last line in file.

## Worx firmware
install arm-none-eabi-gcc https://developer.arm.com/downloads/-/gnu-rm
Note that some older versions does compile but generates a binary that does not start. I have no idea why.
```
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-aarch64-linux.tar.bz2
sudo tar -xvf gcc-arm-none-eabi-10.3-2021.10-aarch64-linux.tar.bz2 -C /usr/share/
sudo ln -s /usr/share/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi* /usr/bin/

cd LandLord
make cpu=1788 #for db504 (the only board that is working atm)
# flash with openocd, openocd-flash.cfg
# note you will need to hold down power button to flash with openocd.
# it is also possible to flash with usb, the original boot loader is kept intact so rollback and usb flashing is possilbe.
# use create_bin.py to generate binary to put on usb stick.
```


## TODOOOOOOO
Known things that needs implemtentation
* Working emegancy stop and communication to ROS and reset emergancy stop
* internal tilt sensor emergancy stop if z val < threashold (mower leaning too much, upside down, standing on its side/back)

* xSensorQueue is now replaced with global struct sensorMsg instead. We need to add Semaphores before updating sensorMsg! (read should be ok though...?)

* BUG - Investigate the relationship with CHARGER_CHECK, CHARGER_CONNECTED, CHARGER_ENABLE, MOTOR_MOSFET. It seems we need both MOTOR_MOSFET and CHARGER_CHECK enabled to get a reading on CHARGER_CONNECTED. Make powermgmt.c more reliable. I still want to keep the "keep alive charger" function. since the mower standby voltage is so much higher I still want to re-enable charging. Pulse the CHARGER_ENABLE for 10ms every 500ms is enough to make the charger being kept in "red" state.

* investigate what sensor actually does what function naming might be wrong - SENSOR_STUCK, SENSOR_STUCK2, SENSOR_LIFT, SENSOR_COLLISION
* Full stop when collision detected. Block forward motion - and then? ROS has no way to handle this event yet.
* Lift sensor - no function

* Better readme - The first person setting up the enviroment PLEASE take notes! I am available on discord in the worx-landroid channel. https://discord.com/channels/958476543846412329/1092896540907032696