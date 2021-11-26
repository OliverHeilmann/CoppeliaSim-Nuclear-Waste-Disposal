# Introduction
This project demonstrates how four robotic arms can be used to extract nuclear fuel rods from a reactor and pass them through a series of obstacles. As well as graphically displaying torque loads on the robotic arms (a feature which a digital twin might use), the system also employs computer vision to locate and select which specific fuel rods for extraction. This repository contains all the necessary files and information to build and run the project described above.

# Description
<p align="center">
   <img width="750" src="https://github.com/OliverHeilmann/CoppeliaSim-Nuclear-Waste-Disposal/blob/main/pictures/pic1_Main.png">
</p>

1) Main view where the user may navigate through the environment to inspect their chosen features. The model design was influenced by a nuclear reactor; here, a cross section is presented such that all four robots can be seen simultaneously. After the final robot recieves the fuel rod, it is then placed on the conveyor belt and subsequently dropped into a hollow cylindar (representing the way that spent fuel is burried underground in lead/ concrete structures).
2) Shows the computer vision view as seen by the first robots onboard camera. This system filters for colour, then searches for key features, determines their position in the frame and then assigns the fuel rod to be collected first (note this is always displayed in red). Using inverse kinematics, the robot arm is dynamically adjusted based on the results from the computer vision sensor until the arm is on top of the pre-selected fuel rod.
3) A top down view of the nuclear reactor and fuel rods. Sections of the robot are intentionally ommitted from the view so as to better see the fuel rods during extraction.
4) Each robotic arm has an onboard camera. Users can see the frames if they wish to get a better view of the gripper and fuel rod. Note, in a real system where a user may wish to control a robotic arm, they may also wish to use the camera for sensing/ control.
5) Popup window is used to allow users to control some actions of the robotic arm e.g. joint angles, computer vision frame etc.
6) Joint torques are displayed on a graph to illustrate how the forces vary during operation. Note that all 6 joints (for robot 1, closes to the reactor) are displayed on the same graph to conserve space. Ideally, they would be presented on separate scales to allow users to see the joints with lower maximum torques.

# Exporting From Unity
1) Install FBX Exporter --> GameObject --> Export to XBF (note that you should select top level to export)
2) Install Probuilder --> tools --> ProBuilder --> ProBuilder Window --> ProBuilderize
3) Tools --> ProBuilder --> Export --> Export Obj

# Importing to CoppeliaSim
1) Note: CoppeliaSim can import objects with textures but the textures MUST be as JPEG (or similar)
2) Your files will be tif --> convert them to JPEG using [this!](https://cloudconvert.com/tif-to-jpg)
3) Your Mat file will reference the old .tif file --> change it to .jpeg file instead.

# Notes on Syntax
use "" for referring to objects in self
use '' for referring to objects not in self i.e. world objects

consider ur5WithRg2Grasping demo scene

# Useful Links
1) [Standard Sim API](https://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm)
2) [Vision Sim API](https://www.coppeliarobotics.com/helpFiles/en/simVision.htm?view=category)
3) [IM Sim API](https://www.coppeliarobotics.com/helpFiles/en/simIM.htm?view=alphabetical)
4) [Vision Callback Functions](https://www.coppeliarobotics.com/helpFiles/en/visionCallbackFunctions.htm)
5) [Messagining/ Interfacing/ Connectivity](https://www.coppeliarobotics.com/helpFiles/en/meansOfCommunication.htm#signals)
6) [Accessing Objects Programmatically](https://www.coppeliarobotics.com/helpFiles/en/accessingGeneralObjects.htm)
7) [Bubble Robot Demo](https://www.coppeliarobotics.com/helpFiles/en/bubbleRobTutorial.htm)