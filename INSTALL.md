# Installation instructions

Installation instructions based on: `https://www.gamingonlinux.com/articles/first-steps-with-openvr-and-the-vive-on-linux.7229`


## Download and build Christoph's OpenVR SDK fork:

      cd ~
      mkdir libraries
      cd libraries
      git clone https://github.com/ChristophHaag/openvr.git
      cd openvr
      mkdir build
      cd build
      cmake -DCMAKE_BUILD_TYPE=Release ../
      make

## Allow hardware access
Then plug-in VIVE to your computer and make sure you can see the devices on `/dev/hidraw[1-6]`.

Copy the file `88-vive.rules` to the folder `/etc/udev/rules.d`. Then run:

      sudo /etc/init.d/udev restart

## Install Steam and SteamVR

Go to `http://store.steampowered.com/` and download Steam for Linux.
After successfully installing and running Steam, it should store its files on: `~/.local/share/Steam`

Install SteamVR by using this URL `steam://install/250820`.
Files should be located on: `~/.local/share/Steam/steamapps/common/SteamVR`

## Configure display.

Go to your OS display options to enable HMD's display.
