# ros_brain
Here is the instruction for ros_brain installation. Please make sure ``turtle_ws`` has been installed and clone this repository into a different workspace.  
if you haven't installed ``turtle_ws``, you can follow [turtle_ws installation](https://github.com/HuanyuL/bulding_instintics)
1. Create a new catkin workspace  
```
mkdir -p brain_ws/src
```
2. Clone this repository
```
git clone https://github.com/HuanyuL/ros_brain
```
3. Before build the workspace you need to source your ``bash`` from ``turtle_ws``
```
source /YOUR_TURTLE_WS/devel/setup.bash
```
4 catkin build
```
catkin build
```
5. To test the installation, launch the example file
```
source devel/setup.bash
roslaunch workshop3.2_bringup.launch
```
