# CS467-400 Quadruped Project

Our team created a 3D-printed, quadruped robot using C++ and an Arduino Uno. The quadruped has 3 degrees of freedom on each leg and is capable of walking, turning in place,  body rotation, and body translation. The user interface was created using Qt and sends commands to the quadruped via bluetooth.

## Software
- Qt
- Arduino IDE

## Hardware
- Arduino Uno
- Adafruit 16 PWM servo shield
- HC-05 or HC-06 bluetooth module
- 9V battery with barrel plug adapter
- 7.4V Lipo battery with 5V UBEC 
- 12 SG90 or equivalent servos
- 3D printer
- M2, M3 screws, ball bearings, wire, resistors

## Deployment
1. Print part files, assemble and wire
2. Upload movement sketch to Uno
3. Load and compile Qt project file

## Usage
Connect to the quadruped via the connect button. Translate the body while the legs remain planted using the translate mode. Pitch, roll and yaw the body while the legs remain planted using the rotate mode. Walk and turn using the walk mode. The sit, stand and center commands are available in all modes. 

## Authors
- Michael/Yau Chan
- Daniel Jarc
- Jacob Powers
