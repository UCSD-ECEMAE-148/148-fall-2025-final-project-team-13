
<h1 align = "center"> Motion to Drive Robot </h1>
<p align = "center">
<img width="650" height="300" alt="image" src="https://github.com/user-attachments/assets/fae75be7-b48a-49da-8780-113694382b2e" />
</p>
<h2 align = "center"> MAE/ECE 148 Final Project Fall 26 </h2>
<h3 align = "center"> Team 13 </h2>

<p align = "center">
<img width="592" height="486" alt="image" src="https://github.com/user-attachments/assets/ea4d738b-65de-4c2c-98a2-80c0440bbb76" />
</p>

# Table of Contents
1. [Team Members](#Team-Members)
2. [Abstract](#Abstract)
3. [What We Promised](#What-We-Promised)
4. [Accomplishments](#Accomplishments)
5. [Demonstration](#Demonstration)
6. [Challenges](#Challenges)
7. [Robot Design](#Robot-Design)
8. [Circuit Diagram](#Circuit-Diagram)
9. [References](#References)
10. [Contacts](#Contacts)

# Team Members
- Evelyn M. - Computer Engineering
- Joseph W. - Computer Engineering
- Aden W. - Mechanical and Aerospace Engineering
- Omar C. - Extended Studies

# Abstract
The project's goal is to implement head-tracking gestures as directional movements and throttle control on our vehicle. We utilized the ROS2 framework with open-source head-tracking software to turn numerical head movement data into car movement commands. Our project hopes to create a fun interactive game experience.

# What We Promised
**Must Haves**
- A direct coorelation between head movement data (yaw and pitch) and the cars throttle and streering

**Nice to Have** 
- Integration of Lidar sensors for emergency stopping near obstacles
- Integration of our groups hand gesture detection model as alternatives for car movement control
- Real time camera feed through the cars OAKD camera

# Accomplishments
- We successfully used UDP to connect AITrack, Opentrack and our pi to efficiently deliver numerical head movement data.
- We mapped pitch values to throttle and forward/backward driving direction.
  - We coorelated a high positive pitch value with the backward direction and full throttle.
  - We coorelated a mid positive pitch value with the backward direction and low throtle.
  - We coorelated a high negative pitch value with the forward direction and full throttle.
  - We coorelated a mid negative pitch value with the forward direction and low throttle.
  - We coorelated mid pitch values (when the head is centered) to give no throttle nor direction
- We mapped yaw values to steering direction
  - We coorelated a high positive yaw value with sharp left turn.
  - We coorelated a mid positive yaw value with a slight left turn.
  - We coorelated a high negative yaw value with sharp right turn.
  - We coorelated a mid negative yaw value with a slight right turn.
  - We coorelated mid yaw values (when the head is centered) to give no steer
# Demonstration

# Challenges
- Our first approach to steering had only two directions hard left and hard right making driving unnatural and incredibly difficult
   - We solved this problem by creating a system that mapped ranges of raw yaw degrees to specific servo steering values

# Robot Design
# Circuit Diagram
<img width="4800" height="2400" alt="Visual Circuit Diagram (3)" src="https://github.com/user-attachments/assets/bb94c57c-3bd9-449d-8f68-8ec0568eca5d" />

# References
# Contacts
- Evelyn M. - emaresmoreno@ucsd.edu
- Joseph W. - jwarzybokmckenney@ucsd.edu
- Aden W. - 
- Omar C. - omarchoy@gmail.com


