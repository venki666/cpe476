### Install and test Raspbian on Raspberri Pi W/4/3

##### Installing operating system images

This resource explains how to install a Raspberry Pi operating system image on an SD card. You will need another computer with an SD card reader to install the image.

Before you start, don't forget to check the SD card requirements.

##### Using Raspberry Pi Imager

Raspberry Pi have developed a graphical SD card writing tool that works on Mac OS, Ubuntu 18.04 and Windows, and is the easiest option for most users as it will download the image and install it automatically to the SD card.

- Download the latest version of Raspberry Pi Imager and install it.
- If you want to use Raspberry Pi Imager on the Raspberry Pi itself, you can install it from a terminal using sudo apt install rpi-imager.
- Connect an SD card reader with the SD card inside.
- Open Raspberry Pi Imager and choose the required OS from the list presented.
- Choose the SD card you wish to write your image to.
- Review your selections and click 'WRITE' to begin writing data to the SD card.
  Note: if using the Raspberry Pi Imager on Windows 10 with Controlled Folder Access enabled, you will need to explicitly allow the Raspberry Pi Imager permission to write the SD card. If this is not done, Raspberry Pi Imager will fail with a "failed to write" error.

##### Using other tools

Most other tools require you to download the image first, then use the tool to write it to your SD card.

###### Download the image

Official images for recommended operating systems are available to download from the Raspberry Pi website downloads page.

Alternative distributions are available from third-party vendors.

You may need to unzip .zip downloads to get the image file (.img) to write to your SD card.

Note: the Raspberry Pi OS with desktop image contained in the ZIP archive is over 4GB in size and uses the ZIP64 format. To uncompress the archive, a unzip tool that supports ZIP64 is required. The following zip tools support ZIP64:

7-Zip (Windows)
The Unarchiver (Mac)
Unzip (Linux)

##### Writing the image

How you write the image to the SD card will depend on the operating system you are using.

###### balenaEtcher

- Download the Windows installer from balena.io
- Run balenaEtcher and select the unzipped Raspberry Pi OS image file
- Select the SD card drive
- Finally, click Burn to write the Raspberry Pi OS image to the SD card
- You'll see a progress bar. Once complete, the utility will automatically unmount the SD card so it's safe to remove it from your computer.

#### Boot Raspberry Pi with wifi on first boot

Insert the micro SD card using USB adapter or SD Card adapter to your PC. The boot partition will be visible on your file explorer. In the boot partition complete the following.

##### Enable SSH on boot

By default, SSH access is disabled for security reason as many user often forgot to disabled it when using Raspberry Pi as a web server. We will need to enable the SSH when Raspberry Pi is booted up for the first time. To do that, create a file called ssh (with no file extension) and copy it to the SD card. The content of the file doesn’t matter.

##### Add wifi configuration

Create a file name wpa_supplicant.conf and copy it to SD card, the content of wpa_supplicant.conf looks like this:

country=SG
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="wifi_ssid"
    psk="wifi_password"
}
Replace wifi_ssid and wifi_password with your actual wifi network information. The wpa_supplicant.conf file will get copy to /etc/wpa_supplicant/ directory automatically once the Raspberry Pi is booted up.

##### Login via SSH

Run terminal command from your computer to login to Raspberry Pi like this:

ssh pi@raspberry.local
or in case you are using the IP address:

ssh pi@192.168.0.10
Replace 192.168.0.10 with the IP address that you identified in step 3.

That’s all. You have successfully boot up your Raspberry Pi at first boot with wifi only, no keyboard, no monitor, and no ethernet cable required.





