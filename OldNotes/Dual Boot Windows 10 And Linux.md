## Dual Boot Windows 10 And Linux (Ubuntu — 20.04 LTS)

Adapted from the website @ https://towardsdatascience.com/how-to-dual-boot-windows-10-and-linux-ubuntu-20-04-lts-in-a-few-hassle-free-steps-f0e465c3aafd

Last week I installed the latest version of [Ubuntu](https://ubuntu.com/) (20.04 LTS) on my PC and I had to do it again on one more. So I thought I might as well write about how I did it here so that all readers on Medium can find it helpful. This is how I did it.

-   Windows 10 PC (should work in the same way for Win 7, 8, Vista with a few modifications in the steps)
-   Ubuntu 20.04 ([Link](https://ubuntu.com/#download) to versions page)
-   Around 20 GB of disk space to spare
-   USB Flash Drive (8–16 GB)
-   Rufus — a software to create bootable USB ([link](https://rufus.ie/))

Once you have these, let us begin.

Insert your USB drive and check if the connection is without any errors. You should not have any data in the USB drive, as any data will be overwritten. Please back up any important data.

Now start the Rufus software that you downloaded (I used version 3.13 from [here](https://github.com/pbatard/rufus/releases/download/v3.13/rufus-3.13.exe)).

![](https://miro.medium.com/max/1348/1*OqKTi-YesTYCp1YwLNAGKg.jpeg)

It will automatically detect the USB drive inserted. Once you click on select, you will get a browse window to go to the Ubuntu ISO file that you downloaded.

![](https://miro.medium.com/max/60/1*TIKQ-jZN9Qs9y9UrRW-3qQ.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*TIKQ-jZN9Qs9y9UrRW-3qQ.jpeg)

After you select the file from your downloads, you may get a few of the following options based on your initial settings. Select as red-outlined in the image below.

![](https://miro.medium.com/max/60/1*-wV5AtO7WOF8KPwKVpmrjQ.png?q=20)

![](https://miro.medium.com/max/1400/1*-wV5AtO7WOF8KPwKVpmrjQ.png)

After this, you are ready to go. Click on start and wait, this should take a couple of minutes based on the drive writing speed.

![](https://miro.medium.com/max/46/1*LzKVAy3kJLwphsWLSNz3tQ.png?q=20)

![](https://miro.medium.com/max/1264/1*LzKVAy3kJLwphsWLSNz3tQ.png)

Once you are finished with this, safely unmount the USB drive and mount it again to check if everything works fine. The drive would have been labeled now as UBUNTU 20\_0 (\[_your drive letter_:\]) by default.

To see your disk space, press Window key + R and type _diskmgmt.msc_ in the application. This will get you to your Disk Management.

![](https://miro.medium.com/max/1242/1*RvPG5lvvLcqT6x0KCCdYwQ.png)

I have the Windows Installation on Disk 0 — OS (C:) and also some amount of data. Right-click on it and select _shrink volume._

![](https://miro.medium.com/max/60/1*3AnkH1eHKf3mGHH2Wln81Q.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*3AnkH1eHKf3mGHH2Wln81Q.jpeg)

I want approximately 20 GB of free space for the Ubuntu installation, so I have entered the amount to shrink in MB as 20,000. You can vary it depending on your space availability. After you shrink, you should be able to see this as unallocated space as seen below.

![](https://miro.medium.com/max/60/1*m5-LoyK-lzoql4WOeEzhWg.jpeg?q=20)

![](https://miro.medium.com/max/1240/1*m5-LoyK-lzoql4WOeEzhWg.jpeg)

That is it, you have a new unallocated partition for your Linux installation.

As always, it is a good practice to back up any data that is there on the drive.

**Also a very important note of caution here:** _To install any Linux distribution, the disk type should be Basic as only this type is supported. However, trying to create too many partitions(4 or more) will convert the disk type inadvertently to Dynamic. If you have any data in Disk 0 (especially a Windows Recovery Drive), converting back to Basic type is very tedious. You will have to format the disk (in the case of C-Drive, you would have to reinstall Windows), and you will lose the recovery partition in most cases._

_The only way I found to convert Dynamic to Basic without formatting is with a software called_ [_Paragon_](https://www.paragon-software.com/free/pm-express/)_. The community version is free to use and works surprisingly well. You can alternatively use this software to create a partition as well._

Now that you have successfully allocated free space in your drive with partitioning, we can start the installation process.

Here is the complete walkthrough with screenshots.

Connect the USB Flash, make sure it is detected and that you can see it as a drive. Restart your PC. Before it boots, press _Esc_ or _F2_ repeatedly. You will see a screen as follows:

![](https://miro.medium.com/max/1400/1*774aRfh6hkVSgtci-UjPRw.jpeg)

Select the boot device as your USB Drive and press _Enter._ You should see a screen as seen here:

![](https://miro.medium.com/max/60/1*LJtosLIcOf7vlvFbti8kjA.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*LJtosLIcOf7vlvFbti8kjA.jpeg)

From here, it is just a few clicks for the basic setup which are shown below:

Select Install Ubuntu option (if you want to try, you can actually use the Try Ubuntu option which still gives you the full capabilities. The only hassle is that you would have to connect your USB flash drive every time you want to use it).

![](https://miro.medium.com/max/60/1*EYbkj1eUo8eQ9jh9ag7ptA.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*EYbkj1eUo8eQ9jh9ag7ptA.jpeg)

Select language.

![](https://miro.medium.com/max/60/1*GdehgJPTEkkKV9tc-TowVw.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*GdehgJPTEkkKV9tc-TowVw.jpeg)

Connect to your Wi-Fi (optional but recommended as the packages and updates will be installed).

![](https://miro.medium.com/max/60/1*LbaIJYepZl4CHnIKUXGsXw.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*LbaIJYepZl4CHnIKUXGsXw.jpeg)

![](https://miro.medium.com/max/60/1*o5M0NiGWPH0sEsPbvt2h9A.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*o5M0NiGWPH0sEsPbvt2h9A.jpeg)

When you are connected, you will see the screen as seen below. Also tick the option for install third-party software (includes codecs e.g. MP3, Flash, etc.)

![](https://miro.medium.com/max/60/1*4HPU_NmY1imxA9ewmr_X2w.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*4HPU_NmY1imxA9ewmr_X2w.jpeg)

Next, you will be asked to create a password for secure boot. This is different from the actual login password that will come later. This password is not needed much, but please keep it handy as it is required if you install anything that might change the boot loader.

![](https://miro.medium.com/max/60/1*lJIAMJHh94pAy7EUIpNT7A.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*lJIAMJHh94pAy7EUIpNT7A.jpeg)

From here on begins the tricky part. You have 3 Options:

1.  Install alongside Windows Boot Manager — This will automatically do the partition for you, but you do not get an option for personalized partitioning, nor the option of where to install the boot loader.
2.  Erase disk and install Ubuntu — You probably don’t want to do this as it will erase your windows installation.
3.  Something else — This is what we will go for, as we have created a free space partition for this purpose.

![](https://miro.medium.com/max/60/1*ljMD8dqB7Hmz3GXQ5BMUEQ.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*ljMD8dqB7Hmz3GXQ5BMUEQ.jpeg)

You will get a window as follows. As you can see, there is a _free space_ of around 20 GB listed there. This is what we will use.

![](https://miro.medium.com/max/60/1*lk3fFHNRmcGeREG5SidqDQ.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*lk3fFHNRmcGeREG5SidqDQ.jpeg)

Select the free space.

![](https://miro.medium.com/max/60/1*ixPvEr3iDUeOSH5Pnb4d5A.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*ixPvEr3iDUeOSH5Pnb4d5A.jpeg)

Click on the + at the bottom left corner, as indicated by the mouse pointer in the picture below. You will get the options for the partition.

![](https://miro.medium.com/max/60/1*gszctXlAMcwXa_cLli5m6g.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*gszctXlAMcwXa_cLli5m6g.jpeg)

We have about 20 GB available. From that, I have used 13 GB for use as the ext4 file system. Select the partition type as Logical and the mount point as a ‘/’ as indicated in the image. Click Ok.

![](https://miro.medium.com/max/60/1*1huk_V9oARMv8R0X4YkL1g.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*1huk_V9oARMv8R0X4YkL1g.jpeg)

After that, go back to the free space seen again, this is approx. 7 GB in this case and click on + again to create a new partition. This one will be used as the swap (a kind of backup RAM).

![](https://miro.medium.com/max/60/1*iqWwq8qm1F4U7wLDcVqu2w.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*iqWwq8qm1F4U7wLDcVqu2w.jpeg)

Once done, click ok.

![](https://miro.medium.com/max/60/1*M21d1sPtAJmL3LEzhV1OSQ.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*M21d1sPtAJmL3LEzhV1OSQ.jpeg)

Now, in the option at the bottom, which says _Device for Boot Loader Installation,_ select the ext4 partition that was created, in this case, it is /dev/sda5.

![](https://miro.medium.com/max/60/1*pcu14dyGUK3x-AOo74qauw.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*pcu14dyGUK3x-AOo74qauw.jpeg)

After this selection. Click _install now_ and continue.

![](https://miro.medium.com/max/60/1*XAKO_6MW53Sr7ZkbteRDcg.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*XAKO_6MW53Sr7ZkbteRDcg.jpeg)

You will have the option to select the approximate region.

![](https://miro.medium.com/max/60/1*iJAoBaO3y-PF-7nzCkwuUg.jpeg?q=20)

![](https://miro.medium.com/max/1400/1*iJAoBaO3y-PF-7nzCkwuUg.jpeg)

Next, you can create a login and password. This password will be used for login and also when you have to install new applications.

![](https://miro.medium.com/max/1400/1*K82LRAoin8rU51y_hAo2xg.jpeg)

Continue and the installation will begin. This will take a couple of minutes.

![](https://miro.medium.com/max/1400/1*32Nj7ZCcfmvLf18FiYC23g.jpeg)

Once done, click _Restart Now_.

![](https://miro.medium.com/max/1400/1*MCPRLWwZQIkv8pUxu_c1jA.jpeg)

After this, you may also see a window saying remove disk and press Enter. In this case remove the USB Flash drive, press Enter.

![](https://miro.medium.com/max/1400/1*pNRpZ7rgK3eQYgM3R6LD1A.jpeg)

Once done, you press Enter for each of the following.

![](https://miro.medium.com/max/1400/1*-xqlicvi6DxrZ1S4uv5Vng.jpeg)

![](https://miro.medium.com/max/1400/1*CizHPGMM4VGNyb1yZAztfA.jpeg)

There you go, almost done. Log in with your password.

![](https://miro.medium.com/max/1400/1*JFvC00xUXhOJJEoVf5gcFQ.jpeg)

Done! Welcome to your brand new Ubuntu installation! This is the window you should see. You can create accounts and customize them the way you like.

![](https://miro.medium.com/max/1400/1*H96TAwKUkGy_5RGva4jgrA.jpeg)

Once you restart your PC, you will automatically go into the Windows boot by default. If you want to start Ubuntu, press escape (followed by F9 to go into the boot device) at startup and you will get to select what you want to boot.

![](https://miro.medium.com/max/1400/1*9XoV7nI2O_nDlaWXkSwQTQ.jpeg)

I hope you had a successful installation! If you are new to Linux, be open to the wide range of learning opportunities it offers. Here are some start-up resources:

You can also get used to common shortcuts and commands from the link below:

And as a bit advanced level, you can unleash the power of _bash_ and type commands like a pro using the cheat-sheet in the link below:

That’s it! Tinker around with your new installation, and keep a watch for my posts about configuring Ubuntu securely and other interesting tips and tricks!

Cheers!

