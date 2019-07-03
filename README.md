# vive_ros

Video example: [https://youtu.be/1hiX0f6UAew](https://youtu.be/1hiX0f6UAew)

## Installation instructions

### Download and build Valve's OpenVR SDK (most recently tested version):

      cd ~
      mkdir libraries
      cd libraries
      git clone https://github.com/ValveSoftware/openvr.git -b v1.3.22
      cd openvr
      mkdir build
      cd build
      cmake -DCMAKE_BUILD_TYPE=Release ../
      make

### Allow hardware access
Then plug-in VIVE to your computer and make sure you can see the devices on `/dev/hidraw[1-6]`.

Copy the file `60-HTC-Vive-perms.rules` to the folder `/etc/udev/rules.d`. Then run:

      sudo udevadm control --reload-rules && sudo udevadm trigger

### Install Steam and SteamVR

Download latest steam version at `https://store.steampowered.com/`. You should get the file `steam_latest.deb` in your `~/Downloads` folder

Install Steam:
      
      sudo dpkg --install ~/Downloads/steam_latest.deb
      
__Note:__ If it gives a dependency error run `sudo apt install --f` and re-run the install steam command (sometimes you may need to repeat this proccess multiple times)

Run Steam:
      
      steam

Setup or log in into your Steam account and install SteamVR from the Steam store.

Steam files should be located in: `~/.steam/steam`

SteamVR files should be located in: `~/.steam/steam/steamapps/common/SteamVR`

### Configure display.

Go to your OS display options to enable HMD's display.

## Usage

Before start:

* Make sure VIVE is present as several `/dev/hidraw*` and you have access permissions.
* Make sure VIVE display is enabled as extended view.
* Libraries and Steam are present on the folders described by `INSTALL.md`.

Procedure:
1. Start a `roscore`
2. Launch the SteamVR's `vrserver` by launching the file: `roslaunch vive_ros server_vr.launch`
3. Launch the node: `roslaunch vive_ros vive.launch`
4. To close the node you can `Ctrl+C`. To close the vr server you have to kill the process. For convenience: `rosrun vive_ros close_servervr.sh`

Update
1. Hardware Safety code added in order to shutdown of hardware. This will prevent hardware from any damage caused due to abrupt shutdown.

2. It has code updated for publishing HTC Vive Component (HTC Vive Headset and 2 HTC Controllers) data.
