# Folkrace
Folkrace is a robotics competition in which multiple autonomous robots race against each other in a track.
They have to complete as many laps as they can under a given time limit.
The winner is the robot which completes the most laps in the correct direction.
The driving direction is determined by the judges before the competition.

## Rules
- The robot must be autonomous.
- The maximum dimensions of the robot are 15x20 cm.
- The maximum weight of the robot is 1 kg.
- Each lap in the right direction scores 1 point, while each lap in the wrong direction scores -1 point.
- The width of the track varies between 100 and 120 cm.
- The course has a number of obstacles, such as hills, holes, loose material, obstructing walls or poles and uneven surfaces.

## Robots
### sizedoesntmatter
A relative small robot compared to other participants (as the name suggests), which performs quite well.
All of the architectural design work was done using AutoCAD software.
The robot was 3D printed at our JBG Robotics club with Creality's Ender 5 Pro.

For navigation in the track it uses 3 Sharp Distance Sensors from Pololu, which detect in 10 cm - 80 cm range.
All of the calculations are done by the microcontroller - Arduino Nano.
The robot uses skid steering mechanism, in which the car wheels are fixed in the same position.
So in order to turn, the robot changes the motors' speed.

The code for the robot was written in Arduino IDE, using Arduino language, which is nearly identical to C++.
To determine the motor speed, necessary for the robot to stay in the middle of the track, PID algorithm was used.
To easily change the variable values (PID tuning parameters, maximum speed, etc), I created a phone application on MIT App Inventor.
The application connects to the HC-06 bluetooth module mounted on the robot, which transfers the information.

#### Achievements
- 1st place Robotų Intelektas 2021
- 2nd place Robotex International 2021

#### Gallery
<div style="display: flex">
  <img src="https://github.com/arminasbrazenas/folkrace/blob/master/Assets/sizedoesntmatter/pic_2.jpg" width="45%" />
  <img src="https://github.com/arminasbrazenas/folkrace/blob/master/Assets/sizedoesntmatter/pic_1.jpg" width="45%" />
</div>

<div style="display: flex">
  <img src="https://github.com/arminasbrazenas/folkrace/blob/master/Assets/sizedoesntmatter/video_1.gif" width="45%" />
  <img src="https://github.com/arminasbrazenas/folkrace/blob/master/Assets/sizedoesntmatter/pic_3.jpg" width="45%" />
</div>

