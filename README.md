# DroneAutoCapture
Image Acquisition for High Quality Architectural Reconstruction

## Dependencies and requirements
This Matlab code depends on the following toolkits and libraries
1. gptoolbox <https://github.com/alecjacobson/gptoolbox>
2. libigl <https://github.com/libigl/libigl> (tag: v1.3.1)
3. clipper <http://www.angusj.com/delphi/clipper.php>
4. fm2 <https://github.com/jvgomez/fm2_matlab>
5. toolbox fast marching <https://github.com/gpeyre/matlab-toolboxes/tree/master/toolbox_fast_marching>

Additionally, the following Matlab toolboxes need to be installed
1. geom2d <https://www.mathworks.com/matlabcentral/fileexchange/7844-geom2d>
2. geom3d <https://www.mathworks.com/matlabcentral/fileexchange/24484-geom3d>


## Code organization:
The entire code is organized as multiple stages that are organized in folders prepended with the stage number. The stages are pipelined and the output of the previous stage feeds to the input of the next stage, except for the 5th stage of roof waypoint generation (which is an independent stage). For every stage the main Matlab script is named after the stage name. Following are various stages of the techniqe that need to be run in this order:
1. **MeshCut**: Cut out a building of interest from a larger nadir mesh  
*Input*: Georefernced boundaries for the building in DXF format (polygon), Nadir mesh in OBJ format  
*Output*: A cut mesh of the required building in OBJ format
2. **MeshSections**: Create cross-sections of the cut mesh at certain altitudes in order to create drone path  
*Input*: Cut mesh  
*Output*: Obstacle maps as polygon files (text)
3. **ObstacleAwarePath**: Create an obstacle aware path from cross-section at an altitude (that serves as an obstacle map) and the building boundary  
*Input*: Obstacle maps, building boundaries (polygon) in DXF format  
*Output*: Obstacle aware path for the drone as polygon files (text)
4. **Waypoints**: Optimize and generate waypoints along the obstacle aware path  
*Input*: Obstacle aware paths, obstacle maps, building boundaries  
*Output*: Drone waypoints for walls
5. **WaypointsRoof**: Generate waypoints for the building roof  
*Input*: Roof polygon in DXF format  
*Output*: Drone waypoints for roof

**Note**  
*Nadir Mesh*: This is the mesh generated by a set of downward looking photographs taken at a high altitude (with waypoints arranged in a grid). An SFM software such as Agisoft Photoscan, OpenMVG/OpenMVS, COLMAP, and Pix4DMapper may be used. The generated nadir mesh must be georeferenced for processing. Since the coordinates are usually stored in the UTM coordinate space, an offset is substracted from all coordinate values so that the mesh coordinates are small values. Such a UTM_offset is used in the code at some places and an appropriate value must be used.


---
The code corrsponds to the following Computers & Graphics publication and is allowed to be used for academic and research purpose (non-commercial):  
*[O. Sharma N. Arora, and H. Sagar, "Image acquisition for high quality architectural reconstruction." In Proceedings of Graphics Interface, 2019. (To appear)](http://graphicsinterface.org/conference/2019/)*  
Please cite the paper in case you choose to use this code.
