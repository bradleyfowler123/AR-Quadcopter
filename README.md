# AR Controlled Drone

This repository contains all the source code used in the custom built AR Drone. It communicates the a mobile device via bluetooth and the device's accelerometer is used to controlled the movement of the drone.

## Device_Brain

The drone uses an Arduino microcontroller. The source code is contained within this directory with the libaries separated from the main scripts as per the Arduino docs.

## Unity_App

This folder contains the Unity app that is used to communicate with the drone's Arduino. It should be launch as a Unity project.
