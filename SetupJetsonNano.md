In this article I show you how to perform a headless WiFi setup of a Jetson Nano. No monitor, keyboard or mouse required. This article is duplicated from https://desertbot.io/blog/jetson-nano-headless-wifi-setup

In order for these steps to work, you should be using a recent version of JetPack.

## Step 1. Gather the parts

For the steps in this article you will need the following:

-   Jetson Nano
-   5V 4A Barrel Jack Power Supply
-   Micro USB cable
-   SD Card (64GB or 128GB)
-   Network card or Edimax EW-7811Un
-   Header jumper - which should be supplied with the most recent versions

## Step 2. Download the Nano SD card image

This article was tested on a **Jetson Nano (B01)** using this image:

-   **Jetson Nano Developer Kit SD Card Image**
-   **JetPack 4.4**
-   **2020/07/07**

You can download it from here:

-   [https://developer.nvidia.com/embedded/downloads](https://developer.nvidia.com/embedded/downloads)

## Step 3. Flash the image to the SD card

Flash the image to the SD card using [balenaEtcher](https://www.balena.io/etcher/).

I recommend using at least a 64GB card, such as this one:

-   [SanDisk 64GB Extreme microSDXC UHS-I Memory Card with Adapter](https://www.amazon.com/gp/product/B07FCMBLV6/ref=as_li_tl?ie=UTF8&camp=1789&creative=9325&creativeASIN=B07FCMBLV6&linkCode=as2&tag=desertbot-20&linkId=211f426bed3161732afc105213883cc6)![](https://ir-na.amazon-adsystem.com/e/ir?t=desertbot-20&l=am2&o=1&a=B07FCMBLV6)

Once the image is flashed to the hard-drive, you may see a message like I did on my Mac that says **The disk you inserted was not readable by this computer**. In that case, click **Eject**.

Remember that on a Mac this isn't a physical eject. It's a software eject. Which means it's safe to pull the SD card and its adapter out of your computer.

## Step 4. Install your network card or dongle

To test this article I'm using this WiFi adapter plugged into one of the USB ports:

-   [Edimax EW-7811Un 150Mbps 11n Wi-Fi USB Adapter](https://www.amazon.com/gp/product/B003MTTJOY/ref=as_li_tl?ie=UTF8&camp=1789&creative=9325&creativeASIN=B003MTTJOY&linkCode=as2&tag=desertbot-20&linkId=dcbc9ce7779445d4b29a4f99b31a8959)![](https://ir-na.amazon-adsystem.com/e/ir?t=desertbot-20&l=am2&o=1&a=B003MTTJOY) (Amazon)

You also have the option of setting up with no network card.

## Step 5. Insert the microSD card

-   Insert the microSD card into the Jetson Nano.

The microSD card goes in the back, just under the heatsink.

If you are looking down on the Nano the contacts must be face up.

The card reader has a spring so you have to gently push the card info the device.

On a **B01** especially, make sure you don't bend the nearby header pins.

## Step 6. Set the power jumper

-   Set the jumper on **J48** (it's the jumper behind the barrel jack)

The jumper may be there, but it may not be set on both posts.

If for some reason the jumper is missing you can buy more here:

-   [Gikfun 200pcs 2.54mm Standard Computer Jumper Caps Short Circuit Cap Mini Micro Jumper Bridge Plug DIY Kit for Arduino (Pack of 200pcs) EK1928](https://www.amazon.com/gp/product/B07F8R36ZW/ref=as_li_tl?ie=UTF8&camp=1789&creative=9325&creativeASIN=B07F8R36ZW&linkCode=as2&tag=desertbot-20&linkId=05073bdd9be95004901a34ad9e49af99)![](https://ir-na.amazon-adsystem.com/e/ir?t=desertbot-20&l=am2&o=1&a=B07F8R36ZW)

## Step 7. Connect the Micro-USB cable

-   Use a cable from a Raspberry Pi or an Android phone
-   Plug the Micro-USB cable into the Jetson Nano
-   Plug the other end into your computer or laptop

## Step 8. Plug in the power jack

-   Plug one end of the 5V 4A power supply into the barrel jack on the Jetson Nano
-   Plug the other end into a wall socket or power strip

## Step 9. Wait for the Jetson Nano to boot up

-   Wait for a drive to appear on your desktop
-   It _may_ be labeled **LT4-README**.

## Step 10. Look for the new device

-   On a Mac in a terminal window type the following:

```
ls -ls /dev/cu.*
```

You should see a device like this:

```
/dev/cu.usbmodem14231190531643
```

If you see multiple devices that start with **/dev/cu.usbmodem**, unplug the Micro-USB cable from the Nano so you can figure out which one it is.

## Step 11. Connect over USB

If you have only one device you can connect to the Jetson Nano from the terminal using this command:

```
screen /dev/cu.usbmode* 115200 -L
```

If you had multiple devices, replace the wildcard with the full name.

## Step 12. Fill in the config info

You should now see a screen like this:

```
System Configuration

 ┌────┤ License For Customer Use of NVIDIA Software ├─────┐
 │                                                        │
 │ Welcome to Jetson Initial Configuration                │
 │                                                        │
 │                         <Ok>                           │
 │                                                        │
 └────────────────────────────────────────────────────────┘
```

-   If **Ok** isn't highlighted, press **Tab**
-   Press **Enter**
-   Accept the license agreement (Press **Tab** / **Enter**)
-   Select your language (in my case **English** / **Tab** / **Enter**)
-   Select your country (in my case **United States** / **Tab** / **Enter**)
-   Select your timezone (in my case **Eastern** / **Tab** / **Enter**)
-   Select **UTC** (the recommendation is to select **Yes** / **Enter**)
-   Select a new user name (in my case **mitch** / **Enter**)
-   Select a password (enter something unique then **Enter**)
-   Confirm the password (enter it again, then press **Enter**)
-   APP Partition Size (I use the recommended default)

**Network configuration**

-   If you aren't concerned about networking right now, select **dummy0: Unknown interface** (note that DHCP will fail later, but you can skip that)
-   If you are using a network card (in this case I'm using the Edimax) select **wlan0: Wireless ethernet (802.11x)**
-   For **Wireless ESSID for wlan0** enter your network name
-   In my case for **Network configuration** I selected **WPA/WPA2 PSK**
-   When prompted enter the pass phrase for your network
-   For hostname, backout the default string and enter your desired hostname (I chose **jet2**)

At this point you will be prompted to login with your username. But the connection may be in a weird state after you are done with the configuration. The easiest thing to do is to just kill the current Terminal window and open up a new one.

**Note:** When trying to setup networking on one attempt I ran into a failure. I tried again with the exact same credentials and it succeeded.

## Step 13. Login over USB

-   Open up a new Terminal window
-   Connect over USB again:

```
screen /dev/cu.usbmode* 115200 -L
```

-   Login using your new username and password
-   Make sure you can see some files:

```
ls -ls
```

## Step 14. Shutdown the Jetson Nano

-   Run this command to shutdown the Jetson Nano:

```
sudo shutdown -h now
```

-   Enter your password for the Nano
-   Wait for the lights on the Nano to go out
-   Disconnect the barrel jack power connector
-   Disconnect the micro USB cable

A this point you can put away the micro-USB connector. You won't need it for the rest of this article.

## Step 15. Test over WiFi

You may find that after the last step your Terminal window is again in a weird state.

Just close it and open up a new one.

-   Plug the barrel power jack back into the Nano
-   Wait a minute or two for it to boot up
-   From a laptop **on the same network** login over WiFi via ssh

Since my user name is **userer** and my Jetson Nano hostname is **jet2**, I would login like this (adjust for your username and hostname):

```
ssh userer@jet2.local
```

**If you get a warning**

Because I run through these steps multiple times using the same hostname, I sometimes get a warning like this:

-   **WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!**

You may see it too if you are trying again with the same hostname.

If you see that error, try this (substituting YOUR-HOSTNAME):

```
ssh-keygen -R YOUR-HOSTNAME.local
```

Then try the login again.

## Step 16. Run some updates

At this point it’s a good idea to run some updates. You can do that by entering the commands below on the Nano.

```
sudo apt-get update
sudo apt-get upgrade
```

## Step 17. Create a swap file

I borrowed these swap file steps from the JetBot [wiki](https://github.com/NVIDIA-AI-IOT/jetbot/wiki/Create-SD-Card-Image-From-Scratch).

```
sudo fallocate -l 4G /var/swapfile
sudo chmod 600 /var/swapfile
sudo mkswap /var/swapfile
sudo swapon /var/swapfile
sudo bash -c &#039;echo "/var/swapfile swap swap defaults 0 0" >> /etc/fstab&#039;
```

## Step 18. Turn on the fan

To see the current status:

```
sudo /usr/bin/jetson_clocks --show
```

To run the fan:

```
sudo /usr/bin/jetson_clocks --fan
```

The `--fan` flag is now required to turn the fan on.

## Step 19. Shutdown properly

Once you are done, you can shut down the Jetson Nano with this command:

```
sudo shutdown -h now
```

## Conclusion

Tasks completed:

-   what hardware is required to perform a headless WiFi setup
-   how to connect from your computer to a brand new Jetson Nano
-   how to remotely login to a Jetson Nano over WiFi
-   how to create a swap file
-   how to turn on the fan


## References

-   **screen** utility manual for OSX \[[1](https://ss64.com/osx/screen.html)\]
-   Jetson Nano – Headless Setup \[[2](https://www.jetsonhacks.com/2019/08/21/jetson-nano-headless-setup/)\]