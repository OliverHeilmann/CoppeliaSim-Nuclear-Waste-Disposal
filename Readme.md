# Description
Words to be added...


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