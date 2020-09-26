## RPI - Romi 32U4 Control Board -  I²C bridge test

With the Raspberry Pi 3 or Zero W, Wi-Fi is built-in, and you should not have to do anything to configure it other than selecting a Wi-Fi network. Otherwise, you’ll need a Wi-Fi dongle. If you are using an Edimax or similar Wi-Fi adapter with the 8192cu module, you will notice frustrating delays or dropped connections unless you disable power management. Create a module configuration file:

```
sudo nano /etc/modprobe.d/8192cu.conf
```


and add the following lines:

##### Disable power management
options 8192cu rtw_power_mgnt=0 rtw_enusbss=0
After setting up Wi-Fi, enable I²C on your Raspberry Pi by running

```
sudo raspi-config
```


and selecting the option within the “Advanced” or “Interfacing Options” menu. By default, I²C runs at 100 kHz, but you can safely increase that rate to 400 kHz and get a much faster communications channel between the boards. To increase the speed, edit the configuration file:

```
sudo nano /boot/config.txt
```


At the end, add the line:

dtparam=i2c_arm_baudrate=400000
Note that the Raspberry Pi 3 performs frequency scaling (reducing the speed of the processor when there are no intensive computations running) that affects the I²C clock, so even when you increase the frequency to 400 kHz, it will not always run that fast. In our tests it usually ran at half of the specified speed, only jumping up to the full speed occasionally or when under significant CPU load.

If you want to log in as a user other than the default pi, give yourself access to the I²C devices with this command, replacing <user> with your username:

```
sudo usermod -a -G i2c,dialout <user>
```

<user> should be replaced by pi in your/most case. One further configuration step that will come in handy is to allow your user to safely shut down or reboot the Pi without typing a password. On Raspbian Jessie, the user pi can already do this. To make this possible for another user, run visudo and add the following lines at the end, again replacing <user> with your username:

<user> ALL = (root) NOPASSWD: /sbin/halt
<user> ALL = (root) NOPASSWD: /sbin/shutdown
<user> ALL = (root) NOPASSWD: /sbin/reboot

<user> should be replaced by pi in your/most case. Try it out now with sudo reboot. After the reboot you should see the device /dev/i2c-1 on your Raspberry Pi, indicating that I²C is available. Then shut down the Pi and remove the USB cable before going on to the next step.

Next, install the Pololu Raspberry Pi I2C Slave Arduino library. Assuming you are using version 1.6.2 or later of the Arduino software (IDE), you can use the Library Manager to install this library:

In the Arduino IDE, open the “Sketch” menu, select “Include Library”, then “Manage Libraries…”.
Search for “Pololu RPi Slave”.
Click the “Pololu Raspberry Pi I2C Slave Arduino library” entry in the list.
Click “Install”.
If this does not work, you can manually install the library:

Download the latest release archive from GitHub and decompress it.
Rename the folder “pololu-rpi-slave-arduino-library” to “PololuRPiSlave”.
Move the “PololuRPiSlave” folder into the “libraries” directory inside your Arduino sketchbook directory. You can view your sketchbook location by opening the “File” menu and selecting “Preferences” in the Arduino IDE. If there is not already a “libraries” folder in that location, you should make the folder yourself.
After installing the library, restart the Arduino IDE.
From the examples menu under PololuRPiSlave, select RomiRPiSlaveDemo, and load this example onto your control board. It is now ready to receive I²C commands from a Raspberry Pi.

At this point you will need to temporarily connect the Raspberry Pi to the Romi 32U4 Control Board. You don’t have to screw them all the way in, but install standoffs in at least a couple of the locations provided to prevent components from touching. (If you are using the Zero, you probably don’t need them at this point.)

The Romi 32U4 Control Board is configured to power the Raspberry Pi from its 5 V supply by default. This means that if you plug a power source into the control board’s USB port, you will turn on both boards. Do that now so you can install the Raspberry Pi software.

To run our Raspberry Pi example code, you should make sure Python 3 and a couple of required libraries are installed. Install them with:

sudo apt-get install python3 python3-flask python3-smbus
Next, download the pololu-rpi-slave-arduino-library code from our GitHub repository. You can do this a number of ways, but if you are unfamiliar with Git, the simplest is to do the following, replacing <version> with the version of the library that you installed for your Arduino IDE (e.g. “2.0.0”).

```
wget https://github.com/pololu/pololu-rpi-slave-arduino-library/archive/<version>.tar.gz
tar -xzf <version>.tar.gz
mv pololu-rpi-slave-arduino-library-master pololu-rpi-slave-arduino-library
```


In the folder pololu-rpi-slave-arduino-library/pi is some example Python code for controlling the Romi 32U4 Control Board. Since you loaded RomiRPiSlaveDemo earlier, the Romi 32U4 Control Board should be ready to respond to these commands. Try running

```
python3 pololu-rpi-slave-arduino-library/pi/blink.py
```

If everything goes well, you should see the LEDs on the Romi 32U4 Control Board flash a sequential pattern.

