# Introduction
This project demonstrates how four robotic arms can be used to take nuclear fuel rods out of a reactor and pass them through a series of obstacles. As well as graphically plotting torque loads on the robotic arms (a feature which a digital twin might employ), the system also employs computer vision to locate and select which specific fuel rods should be selected. This repository contains all the necessary files and information to build and run the project described above.

# Description


<p align="center">
   <img width="750" src="https://github.com/OliverHeilmann/CoppeliaSim-Nuclear-Waste-Disposal/blob/main/pictures/DemoView.png">
</p>

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