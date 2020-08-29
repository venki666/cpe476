## Install Virtualmachine (Host System)

### Virtualmachine

- If you have a laptop and can dual boot with Linux operating system, that’s great! This is the best way to work for this class. If you cannot do this, then you can either try parallels in mac or virtualbox in windows. You can also use vmware for windows to setup your virtual machine. 

### Installing Virtualbox:

1. To install VirtualBox in Windows, go to the download page of VirtualBox and download the installer. [Download VirtualBox for Windows](https://www.virtualbox.org/wiki/Downloads) or 

2. To install VirtualBox in mac, go to the download page of VirtualBox and download the installer. [Download the mac version](https://www.virtualbox.org/wiki/Downloads).

3. Install the software.

   a. Navigate the folder where you have downloaded your VirtualBox and double-click on the downloaded "VirtualBox" file to run it.

   b. "Oracle VM VirtualBox X.X.X Setup" window will appear on the screen and click on the "Next" button to proceed.

   c. Choose the location where you want to install the VirtualBox and click on the "Next" button to proceed. 

   d. Choose the options as per your choice and click on the "Next" button. 

   e. Click on the Yes button and then the "Install" button.

### Installing Linux machine from scratch on Virtualbox:

1. To download the latest version of Ubuntu, i.e. Ubuntu 20.04 LTS, visit the official [Ubuntu](https://ubuntu.com/download/desktop) website in your web browser.
2. By clicking on the "Download" button, you can download the latest version of Ubuntu, i.e. Ubuntu 20.04 LTS (long term support).
3. Now, it is time to create a Virtual Machine. Follow the instructions below to proceed.
4. Open VirtualBox and click on the "New" button.
5. Choose a name for your virtual machine with its location. Based on the name you entered, VirtualBox will try to predict the "Type" and "Version". Otherwise, from the drop-down menu, select "Linux" as the type and "Ubuntu" as the version and click on the "Next" button.
6. With the help of the slider, choose the amount of memory (RAM) to be allocated to the virtual machine. (The recommended memory size is 3 GB. Please note that this memory will only be used while using a virtual machine).
7. Select "Create a virtual hard disk now" option and click on the "Create" button to proceed.
8. Choose the "VDI (VirtualBox Disk Image)" option and click "Next". 
9. Again, click on the "Next" button. 
10. Select the amount of space for your virtual machine and click the "Create" button. (This will be used for your operating system which is going to be installed, so give as much space as possible).

### Installing Ubuntu

1.  Now, your virtual machine has been successfully created and it is time to install Ubuntu on it. Since the latest version of Ubuntu is 20.04 LTS, but I am showing you the installation of Ubuntu 18.04 on VirtualBox and my aim is to show you the process of installing Ubuntu. I hope you understand the process and install the latest version of Ubuntu as you want. Follow the instructions below to proceed.
2.  The name of your virtual machine will now appear on the left side of the VirtualBox Manager. Click on the "Start" button in the toolbar to launch your VM.
3. This time, you have to select your Ubuntu ISO file that you downloaded earlier. Now, click on the folder icon and then click on the "Add" button. Then, select your Ubuntu ISO file. Click on the "Start" button to proceed.
4. If you get the following error: 
   1. The native API DLL was not found (C:\WINDOWS\system32\WinHvPlatform.dll) (VERR_NEM_NOT_AVAILABLE).
   2. VT-x is not available (VERR_VMX_NO_VMX).
   3. To resolve this above error, use the following command in Command Prompt (Admin), "dism.exe /Online /Disable-Feature:Microsoft-Hyper-V".
   4. After removing this error, your Ubuntu installation will be ready to start on your VirtualBox.
5. Now, click on the "Install Ubuntu" button to proceed.
6. Select your desired "keyboard layout" and click on the "Continue" button to proceed.
7. Use the default option as "Normal Installation" with the "Download updates while installing Ubuntu" and click on the "Continue" button.
8. Select the default option as the "Erase disk and Install Ubuntu" and click on the "Install Now" option to proceed.
9. A warning prompt will appear on the screen and click on the "Continue" button to ignore this warning.
10. Choose your time zone on the map and click Continue.
11. Now, set your user account here by filling the necessary details and click on the "Continue" button to proceed.
12. Now, the installation process will begin.
13. Now, restart your system by clicking on the "Restart Now" option.



### Alternate - Installing VMware: (note there is no free version for MAC osx)

1. Download  and install [VMware Workstation Player](https://www.vmware.com/products/workstation-player.html) for windows. It’s free!

### Installing Linux machine from scratch on VMware:

Once you have your VMware installed, let’s create a new VM and install Ubuntu 18.04.

- Download Ubuntu 18.04 disc image from [official website](http://releases.ubuntu.com/18.04/) (64-bit PC Desktop).
- You can also download the latest version of Ubuntu. To download the latest version of Ubuntu, i.e. Ubuntu 20.04 LTS, visit the official [Ubuntu](https://ubuntu.com/download/desktop) website in your web browser.
- In VMware, create a new VM.
  - Typical configuration
  - Choose the disc image you download
  - Enter some information about this VM
  - Again, enter name
  - Please allocate at least 30GB (preferred 50GB)
  - Store virtual disk as a single file
  - Customize Hardware: Please allocate more memory and CPU processors for better performance
  - Finish
- Great. Now you have a Linux (virtual) computer. Take your time and play with it!

Note that the disk size 30GB/50GB will not be allocated instantly, but will dynamically grow as you are adding more stuff.

