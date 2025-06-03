# How to Image a Raspberry Pi for GWUAV

## 1. Use Raspberry Pi Imager
- Install **Raspberry Pi OS (64-bit, latest version)** onto your microSD card using the Raspberry Pi Imager.

## 2. Set OS Customization Settings

### General
- **Set Hostname, Username, and Password**  
  Write these down for future reference.
- **Configure Wireless LAN**
  - Set the **SSID** and **Password** to match your wireless network.
  - Set **Wireless LAN country** to `US`.
- **Set locale settings**
  - Choose the correct **time zone** and **keyboard layout**.

### Services
- **Enable SSH**: Yes
  - Use **password authentication**.

### Options
- These settings **don’t matter**, leave them as is.

- **Run the Imager** and wait for it to finish.

- Install the **microSD card** into the Raspberry Pi and **power it on**.

---

## 3. Post-Boot Setup

### Software & Features
- Install **VS Code**
- Enable **VNC** and **GPIO**
- Install **Miniforge** and **conda (version 3.9.21)**
- Install required **Python packages**
- Update `.bashrc` as needed
- Upload any **code** to the desktop

### Network & System
- Configure **network priority**
- Run `sudo apt update && sudo apt upgrade` to update the system

---


# Pre-Flight Dependency Checks

Before running `challenge1.py`, ensure your system has the following **software, files, devices, and packages** installed or connected.

## Python Version
- Must use **Python 3.9 or below**
- Run: `python3 --version`

## Required Files & Directories
The following must exist in your working directory:
- `aruco_library.py`
- `calibration_logi.npz`
- `challenge1.py`
- `preflight-c1.py`
- `Logs/` (folder)

## Required Python Packages
Ensure the following Python libraries are installed:
- `numpy`
- `cv2` (OpenCV)
- `dronekit`
- `pymavlink`

> Optionally needed for Challenge 2 depending on hardware:
> - `Jetson.GPIO` *(for Jetson devices)*
> - `RPi.GPIO` *(for Raspberry Pi)*

Run:
```bash
ls -l /dev/ttyACM0 /dev/ttyUSB0

if permissions are denied then run:

sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0
```

## Run Pre-Flight Script
To validate everything above is ready:

```bash
python3 preflight-c1.py
```



## USB Camera Connection
- Check with: `lsusb`
- The system will fail if the camera is not connected

## Serial Device Permissions
Ensure correct permissions (`666`) for:
- **Flight Controller** at `/dev/ttyACM0`
- **Telemetry Module** at `/dev/ttyUSB0`

# Simulations
<p>Navigation simulations with SITL, MAVProxy, Gazebo, MAVLINK, DroneKit, and pymavlink.</p>

---

## Shell script simstart.sh
<p> Running, just the shell script in the terminal should start up SITL with MAVProxy, Gazebo virtual world, and the autonomous mission scripts. You can specify different scripts from the Auto Scipts folder. </p>

## How to use capstone.world

### Explanation

<p>Capstone.world is a Gazebo simulation environment. It is supposed to emulate the real settings of the drone flight. You can make this as specific and realistic as you want by defining the gravity, time, wind conditions, etc. Gravity is default 9.8 m/s^2,
but think about simulations on the moon. Specifically for capstone.world the time is defined as early morning with clouds and natural lighting. Morning because the competition starts then and clouds because of a typical cloudy day at Northern Virginia. <br>

The .world file uses XML and SDF standards. Models were used to include drones and ArUco markers. <br> 

Also, the ArUco markers positions can be changed by editing the *<pose> </pose>* first two points which are x,y coordinates. This will give random locations for later tests with the drone. <br>

Now, other simulations and programs can be written to make the drone model fly within this .world (think of .launch scripts). Run this command in one terminal:</p>
```
gazebo --verbose ~/ardupilot_gazebo/worlds/capstone.world
```
<p>In the other terminal, you will run the .launch script.</p>

![capstone world_directory](https://github.com/user-attachments/assets/2255394d-d981-472e-938d-cdabfe288d9c)


![capstone world_sim](https://github.com/user-attachments/assets/65682ce2-08d3-47d4-a100-cc2029a0106d)


![Tutorial Video For Running Drone-To-Drone Communication + Flight](https://drive.google.com/file/d/17-rLpAumjmc_p_EiKA09Fj9I2nQ5LNkR/view?usp=drive_link)


