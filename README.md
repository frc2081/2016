# FIRST Robotics 2016 Code

#### Controller Mapping
* Controller 1

![alt tag](https://raw.githubusercontent.com/matthewknecht/RealRoboCode2016/master/2016%20Controller%201.png)
* Controller 2

![alt tag](https://raw.githubusercontent.com/matthewknecht/RealRoboCode2016/master/2016%20Controller%202.png)

#### Incorrect Pin-Out Record
```
Left Drive Encoder: DIO 0, 1
Right Drive Encoder: DIO 2, 3
Winch Encoder: DIO 4, 5
Photo-Eye: DIO 6
Gypo: SPI 

Left Motors: PWM 0
	Left Motor 1: Motor Port 0
	Left Motor 2: Motor Port 1
Right Motors: PWM 1
	Right Motor 1: Motor Port 2
	Right Motor 2: Motor Port 3
Winch Left CIM: PWM 2
	Winch Left CIM : Motor Port 4
Winch Right CIM: PWM 2
	Winch Right CIM: Motor Port 5 
Winch Motor Mini-CIM: PWM 2
	Winch Motor Mini-CIM: Motor Port 6

Solenoid Arm Closer: PCM0 0, 1
Solenoid Lifter: PCM0 6, 7
Solenoid Poker: PCM0 2, 3
Solenoid Arm Lever: PCM0 4, 5
Solenoid Winch: PCM1 0, 1
```
#### To Do
* Label Controllers physically
* Fix pressure sensor
* Install Robot Signal Light (RSL)
* Label all pneumatic solenoids
* Label all PWM connectors with their port numbers
* Put retention clips on all electrical connectors
* Test Classmate PC with the robot
