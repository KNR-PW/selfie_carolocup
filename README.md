# Selfie-autonomous-car
![pic](/uploads/62db4f1809915f5a58a848fd77c937ac/overview.jpg)

*Selfie* is the student project of autonomous cars. Vehicles based on 1:10 scale RC cars are customized to be able
to operate autonomously in simulated road environments. They are equipped with camera, computer vision computing unit,
controller and set of sensors like magnetic encoders, distance sensors and IMU.

# Selfie Carolo-Cup2021

This repository contains the [catkin workspace](http://wiki.ros.org/catkin/workspaces) for Selfie Autonomous Car
at [Carolo-Cup 2021](https://wiki.ifr.ing.tu-bs.de/carolocup/news) competition.

## Build instructions

The project is targetting [ROS Noetic](http://wiki.ros.org/noetic) distribution. Make
sure you have it [installed](http://wiki.ros.org/noetic/Installation) on your development machine.

Clone the repository to a convenient location using:

```bash
git clone https://gitlab.com/KNR-Selfie/carolocup-2021/carolocup_2021.git
```

Navigate to the main directory with:

```bash
cd carolocup_2021
```

The following set of commands will in turn download all external dependencies, build the packages in
`src` directory and include them in your environment.

```bash
catkin_make
source ./devel/setup.bash
