# SAVARI OBU Node

**A ROS driver (server) for SAVARI DSRC on-board-unit (OBU)**

This package publishes ROS topics related to the content of basic-safety-messages (BSMs) including:
 
- vehicle general information (length, height, mass, class, etc.)
- vehicle live sensory information (light/wiper/signal status, brake/throttle status, etc.)
- vehicle path history information (if provided)
- Message interval rate (beaconing rate -Hz-)
- vehicle GPS information (lattitude, longitude, GPS time, speed, etc.)

The information can be for self vehicle (the OBU which the ROS client node runs on) and/or the ego vehicle (the OBU which is installed on other vehicle from which BSM messages are periodically received by self vehicle). 

<p align="center">
  <img src="/image/Header.jpg">
</p>

**What is DSRC?**
DSRC is is “a two-way short-to-medium-range wireless communications capability that permits very high data transmission critical in communications-based active safety applications,” according to the U.S. [Department of Transportation’s Intelligent Transportation][https://www.its.dot.gov/factsheets/dsrc_factsheet.htm] Systems Joint Program Office. DSRC is provided with 75 MHz of spectrum around the 5.9 GHz band (5.850-5.925 GHz) band to be used for vehicle-related safety and mobility systems. A number of DSRC applications have been envisioned:

* Collision avoidance
* Transit vehicle refueling management
* Personalized taxi dispatch services
* Integrated transportation financial transactions:
* Toll collection
* Parking payment
* Rental car payments and processing
* Pedestrian safety at intersections

[Read more here!][https://www.rcrwireless.com/20151020/featured/what-is-dsrc-for-the-connected-car-tag6]

## Dependencies

* Supports Ubuntu 16.04 with ROS Kinetic
* Robot Operating System ([http://wiki.ros.org/kinetic/Installation/Ubuntu][ROS Installation])
* sensor_msgs/NavSatFix ([http://wiki.ros.org/sensor_msgs][link])
* savari_msgs ([https://github.com/zlg9folira/savari_msgs][link])

## Publishes
```sh
- /savari_ros/self/fix (sensor_msgs/NavSatFix)
- /savari_ros/self/gps (savari_msgs/Gps)
- /savari_ros/self/vehicle (savari_msgs/Vehicle)
- /savari_ros/self/path (savari_msgs/Path)

- /savari_ros/ego/fix (sensor_msgs/NavSatFix)
- /savari_ros/ego/gps (savari_msgs/Gps)
- /savari_ros/ego/vehicle (savari_msgs/Vehicle)
- /savari_ros/ego/path (savari_msgs/Path)
```

## Subscribes to
```sh
- /*   None  */
```

## Building the ROS Node

* Change directory to `/src` of your catkin workspace and clone:
```sh
cd catkin_ws/src
git clone https://github.com/zlg9folira/savari.git
```
* Be sure to source the ROS setup script before building
```sh
source /opt/ros/kinetic/setup.bash
```
* Build: 
```sh
cd ..
catkin_make --only-pkg-with-deps savari
```

## Testing & Usage 

```sh
rosrun savari savari_ros -h
```

The command above should return the command/argument structure supported by the ROS node:

```sh
[Usage]		rosrun savari savari_ros -p [PORT]
[Example]	rosrun savari savari_ros -p 1111
[Default]	rosrun savari savari_ros
```

## Development

Package will be subject to change.


## Meta

Foad Hajiaghajani

[CONNECTED AND AUTONOMOUS VEHICLE APPLICATIONS AND SYSTEMS](https://www.linkedin.com/in/foadhajiaghajani)


