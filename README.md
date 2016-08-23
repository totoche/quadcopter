# Quadcopter

arduino code of my quadcopter

## Equipments

* Controller : Arduino UNO
* IMU : 9DOF sparkfun stick stick (follow this tutorial https://github.com/ptrbrtz/razor-9dof-ahrs/wiki/Tutorial for calibrations values)
* Radio : Turnigy 9X 9Ch Transmitter w/ Module & 8ch Receiver (v2 Firmware)
* ESC : TURNIGY Plush 18amp Speed Controller w/BEC- modified version - 2ESC connected as 1 unit
* Motors : hexTronik DT750 Brushless Outrunner 750kv
* Battery : 2200mah 11.1v 3 cell
* Propeller : 11x4.7
* Base : X525 V3 Quadcopter X-copter Foldable Frame Kit Fiberglass Multirotor

## How to setup

* Find calibration value of 9DOF sparkfun stick
* Download and import arduino PID library (http://playground.arduino.cc/Code/PIDLibrary)
* Calibrate PID for stabilization ( think to Calibrate each ESC before )
* Have fun !

## license

GPL v3.0
