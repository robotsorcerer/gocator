
### ROS Bridge to the GoCator Line Scanner Windows SDK/Library
##### Drivers for retrieving the Infra-red profile and surface map data from the Gocator 2370 Line Scanner

#### Author: [Lekan Molu](http://lakehanne.github.io)
#### [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
	- [X-Resolution](#x-resolution)
	- [Z-Resolution](#z-resolution)
	- [Y-Resolution](#z-resolution)
	- [Profile Output](#profile-output)
	- [Coodinate Systems](#coodinate-systems)
	- [Sensor Coodinates](#sensor-coodinates)
- [Using Gocator on Linux](#using-gocator-on-linux)
	- [Running a Standalone System](#runing-a-standalone-system)
- [Piplines 1](#pipeline-1)
- [Pipeline 2](#pipeline-2)
- [Dependencies](#dependencies)
	- [Optional Dependencies](#optional dependencies)
	-[GOSDK FIRMWARE VERSION 4.3.3.167](#gosdk-firmware-version-4.3.3.167)
	- [GOSDK VERSION 4.3.3](#gosdk-version-4.3.3)
- [Sensor Profile](#sensor-profile)
	- [Streaming profiles of line scans](#streaming-profiles-of-line-scans)
	- [STREAM PROFILE DATA MAP](#stream-profile-data-map)
	- [STREAM SURFACE DATA MAP](#stream-surface-data-map)
	- [STREAM VIDEO POINT CLOUD MAP](#stream-video-point-cloud-map)
	- [STREAMING RANGE PROFILES](#streaming-range-profiles)
- [FAQS](#faqs)

#### Introduction

The sensor captures a single 3D profile for each camera exposure. The sensor projects a laser line to the target and the sensor's camera views the laser from an angle capturing the reflection of the light off the target. Due to the triangulation angle, the laser line appears in different positions on the camera depending on the 3D shape of the target.

##### X-Resolution
X-Resolution is the horizontal distance between each measurement point along the laser line.

Default XResolution: 0.275mm.

##### Z-Resolution

Z Resolution is the variability in height measurement. The Z resolution is also closer at the close range and worse at the far range just as the X-resolution. It is best to position an object at no more than 35mm from an object in the z-direction

Z-linearity is the difference between actual distance to the target and the measured distance to the target, throughout the measurement range.

Default Z-Resolution from profile map is 0.0088 millimeters.


##### Y-Resolution
This is obtained as the sensor is moved across a surface. 

##### Profile Output

Measures the height of an object from the laser triangulation and reports a series of ranges along the laser line, with each range representing the distance from the sensor's origin plane (z is in the range of 0 to 700mm or -350mm to 350mm). Each range contains a height and a position in the sensor's field of view. Alignmnet is performed in the Alignment panel on the Scan page of the web UI.

##### Coordinate Systems

The sensors are calibrated in engineering units (mm) and are ready to deliver profiles right out of box but alignment procedures are required to account for sensor mounting inaccuracies. 
Before alignment, individual sensors use the coordinate system below.

##### Sensor Coordnates
The Z-axis represents the sensor's measurement range with the values increasing towards the sensor. The X axis represents the sensor's FOV with +x pointing to the right (it's a left-handed system). The origin is at the center of the measurement range and field-of-view (FOV). 
In surface data, the Y-axis represents the relative position of the part in the direction of travel. Y position increases as the object moves forward increasing encoder position.

#### Using Gocator Scanner on Linux
##### Running a Standalone system

1.	Power up the sensor. Connect the Power/LAN cable to the sensor and the other end to the Power and Ethernet ports on the Master 100

2. In network tab from your Ubuntu search tab, you want to create a new wired connection with the following settings:

- IP address: 192.168.1.5

- Subnet Mask: 255.255.255.0

- Default Gateway: 0.0.0.0

3. Then in a browser, launch the following ip address

```bash
192.168.1.10
```

#### Pipeline 1: January 14, 2016 : Synchronous operation [master branch]
This code was developed in `ROS Hydro` on the an `armsdev` jail environment. It's been tested on  Ubuntu 12.04 and Ububntu 14.04 +. You are welcome to leave comments or add a PR if you can verify it works in Jade.

To work without an encoder (in `Time` Trigger Mode), the laser strobes the camera in an infinite while loop and I push acquired point clouds to the visualizer window. Press `r` on your keyboard to zero-center the displayed point clouds. For additional help,
 click within the console where the executables are launched and press `h` for additional help with the `vtk` window.

 To run in synchronous mode, check out the master branch from this `git repo` and compile e.g. `catkin_make --source src/gocator`, assuming the code is cloned into a folder called `src` within your `catkin workspace`

 @TODO

 *	Gather profile triggers into a buffer after every end-to-end motion of the sensor and display the 3D points on the visualizer

 *	Implement the `OverFeat` classifier for generic objects detection and classification?


#### PIPELINE 2: May 2016 [async branch]

 Program is now integrated to work with maxon encoder. There is a a launch script within the gocator_profile and gocator_surface
 package directories. Doing 

 ```
 roslaunch gocator_porfile profile.launch
 ``` 

 or 

 ```
 roslaunch gocator_surface surface.launch
 ``` 

 would start the gocator sensor on the rail. The launch file launches two executables:

 1. EPOS CONTROLLER EXEC

	The Epos controller executable which moves back and forth in velocity mode at a constant speed of 1500rpm. I have set it up
 	to do a 1500rpm motion consecutively for four iterations in the forward and backward directions respectively. I added a `3 seconds`
 	delay in-between each velocity motion to avoid race conditions. It is important to change the triggering mode to "Encoder" on the web UI.
 	Technically, you shouldn't be required to do this but I have found that the API's provided doesn't automatically change the 
 	triggering mode

 2. GOCATOR_PROFILE EXEC

 	The "gocator_profile" and "gocator_surface" executables listen asynchronously for encoder triggers. Whenever the motor moves, 
 	the encoder triggers and a message is sent to the sensor to acquire a frame. After each frame acquisition, the laser points are 
 	pushed into a point cloud container and an update is made on the point cloud visualizer window.
 
 3. @TODO

 *	Gather profile triggers into a buffer after every end-to-end motion of the sensor and display the 3D points on the visualizer
	
 *	Implement the `OverFeat` classifier for generic objects detection and classification?

#### Dependencies

* OpenCV

  For visualizing the profile map if you do not want to use the web interface. But the web interface are just good enough if you do not care for a separate UI development.

*   PCL (Point Clouds Library)

*   Maxon Motor libraries
	Because the encoder I used in testing the rail motion interfaces with the maxon motor, we are going to need the at least `libftd2xx.so.1.3.6` and `libEposCmd.so.5.0.1.0` to link 
	the compiled codes against the maxon libraries. The shared object files we would need are in the [Gocator_Bridge/lib folder](https://git.kivasystems.com/projects/HWA/repos/gocator/browse/gocator_bridge/lib). Sometimes, `catkin` has issues 
	reading the byte symbols of used librraies. To be run-time linking safe, go to the maxon website, grab these files, copy them to your `/usr/local/lib/` folder and do a symbolic link 
	to the /usr/lib` folder as follows:

```bash
	code@cracker$   cd /usr/local/lib
code@cracker:/usr/local/lib$	sudo cp /path/to/libftd2xx.so.1.3.6 `pwd`
	code@cracker:/usr/local/lib$	sudo cp /path/to/libEposCmd.so.5.0.1.0 `pwd`	
	code@cracker:/usr/local/lib$	sudo ln -s libEposCmd.so.5.0.1.0  /usr/lib/libEposCmd.so
	code@cracker:/usr/local/lib$	sudo ln -s libftd2xx.so.1.3.6 /usr/lib/libftd2xx.so	
	code@cracker:/usr/local/lib$    sudo ldconfig
```

##### Optional Dependencies

  *	 VTK LIBRARY (Optional)
  *	 Boost Library (Optional)

  Most of these dependencies are already part of the native ROS install except for GLEW from OpenGL (you can pull this from the khronos repo should you need one). 

##### GoSDK Firmware version 4.3.3.167

This is in the Data folder. Simply do the network configuration as advised in the client set-up page of the manual in the Data/Manual_User.pdf file (pg 35), navigate to the `Manage` page within the web gui and click on `upgrade`. For ease of use, I am providing the firware version I used in the Data directory to reduce set-up complexity. Choose the file `Firmware_4.3.3.167_SOFTWARE_Gocator2x00.dat` within the data folder to upgrade the firmware version.

Installing the firmware took between 20 to 30 minutes on my system. Your system set-up time might vary. 

After you are done with the firmware upgrade, you would want to refresh your web UI page at the ip address, `192.168.1.10`. Again, you do not need to choose a password at this time. Feel free to do so later when you get a hang of the set-up.

##### GoSDK version 4.3.3

The default SDK from LMI3D ships as a C API library and is default to windows and MAC OS. To get the SDK running on LINUX (and Ubuntu 12.04 in particular), I had to compile all intermediary object files associated with the GOSDK into two separate shared object files: one for the Gocator API named `libGoSdk.so` and the other for the so-called kAPI library, named, `libkAPI.so`. Both of these are in the `gocator_bridge/GoSDK/lib` directory. The provided CMakeLists.txt file nicely links to both at run time and you should not have to go through the trouble of recompiling these.

It is important to download the latest SDK files from the Gocator repositories online if you want to make changes to the source files. I spent needless hours trying to debug my code when it was a simple matter of upgrading the firmware. The SDK files (as of January 2016) are provided in the Data folder should you want to do further development and testing of the code. The header files you might need are all nicely included within the `gocator_bridge`'s' include folder. You do not have to reinvent the wheel :) .

## Sensor Profile

The default sensor index for a single scanner gocator is `0`. It seems the number increases progressively as you add more sensors to your set-up.

#### Stream profiles of line scans

The `gocator_bridge` is the standard ros interface to the GoSDK API Library while the `gocator_receiver.h` contains all the classes that enables laser map retrieval of the data context from the scanner.

To run, do the following:

##### Run the GoSDK bridge

Now that we are all set, you can clone this repo, compile and start playing with the executables.

- git clone from the Kiva repo database into your armsdev `catkin workspace` folder:

```bash
	code@cracker$ cd /path_to_your_catkin/workspace && jail
   (armsinstall)root@cracker:/home/local/armsdev/catkin_ws# git clone https://$User@git.kivasystems.com/scm/hwa/gocator.git
```

- cd into the cloned repo and install the ros dependencies:	

```bash
	cd src/gocator
	rosdep --install --from-paths .
```

- Compile the source codes:  

```bash
catkin_make -DCMAKE_BUILD_TYPE="Release"
rosrun gocator_bridge gocator_bridge
```

##### Stream profile Data Map

In a separate terminal, begin streaming profile images:

```bash
	rosrun gocator_profile gocator_profile
```

To run in asynchronous mode while sensor is being triggered by motor motion, 

```bash
	roslaunch gocator_profile profile.launch
```
This will launch the maxon controller executable as well as profiler.
To center the point cloud in the window, click on the point cloud visualizer and press `r` on the keyboard.

##### Stream Surface Data Map

The surface map gives the 3D coordinates of the visual scene.

```bash
	rosrun gocator_surface gocator_surface
```

To obtain continuous stream of surface map with moving motor, 

```bash
	roslaunch gocator_surface surface.launch
```

This will launch the maxon controller executable as well as surfacer. To center the point cloud in the window, click on the point cloud visualizer and press `r` on the keyboard.

##### Stream Video Point Cloud Map

The 2370A has an imager that enables the laser scanner to capture 3D snapshots when the camera is strobed. To retrieve the video map,

```bash
	rosrun gocator_video gocator_video
```

To obtain continuous stream of surface map with moving motor, 

```bash
	roslaunch gocator_video video.launch
```

This will launch the maxon controller executable as well as point cloud viewer for the video. 

To center the point cloud in the window, click on the point cloud visualizer and press `r` on the keyboard.

##### Streaming range profiles
Ethernet output for range data must be enabled. Ethernet/IP is protocol used for PLC, Gocator 1x00 single point sensors only. In  a separate termninal, do,

```bash
	rosrun gocator_range gocator_ranger
```

If you run into issues, feel free to open an issues ticket or ping me [@patmeansnoble](https://twitter.com/patmeansnoble).

## FAQS

### I am having issues upgrading my firmware?

If you are having trouble doing a straight upgrade, I would recommend restoring your firmware to factory version first before attempting an upgrade. Simply fire up your web gui, navigate to `Manage -> Maintenance -> Factory Restore ` and click on `Factory Restore`.

I had problems similar to this and I conjecture this has a magical way of fixing the problem.


### I am having a GoSystem_Connect Error -997

You need to fix your firmware. Please refer to the firmware section above.
