
# Robutler (Turtlebot as butler)

PSR course, asssignemnt 3, porpouse of this assignment is to use turtlebot waffle pi as a butler in a given scenario. The robutler should be able to navigate inside the apartment, to detect some objects and to compute some 'mission'.


## Requiremets
This repository contains a ROS package and therefore requires ROS to be installed (tested with ROS Noetic on Ubuntu 20.04).

Other requirements:
- [yolov7](https://github.com/lukazso/yolov7-ros)
- turtlebot simulation with ros, how to set up at: https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/?fbclid=IwAR2l6Jd1E4GLdlt_ClWRMoyh8Rl8SNGxdPIyjIvgg1mOeBn-Xn4si9XrP80
- to correctly visualize the apartment: 
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/aws-robotics/aws-robomaker-small-house-world
    git clone https://github.com/aws-robotics/aws-robomaker-hospital-world
    ```
    and maybe you'll need also to add some new models to your gazebo models, usefull repository: https://github.com/oguran/models
## Usage
Run first of all gazebo using :

```bash
roslaunch robutler_bringup gazebo.launch
```
Then to initialize the robot use
```bash
roslaunch robutler_bringup bringup.launch
```
this command will spawn the robot in gazebo and open rviz.

Now you have different packages for different pourpouse:

- robutler_navigation: ```roslaunch robutler_navigation navigation.launch```
    this will initialize the localization procedure and enable the movement of the robot with the command ```rosrun robutler_navigation go_to_specific_point_on_map.py```which take as argument or a pair of coordinates xy or a room name in which you want to move(run with -h to see the possible room).

- robutler_detection: ```roslaunch robutler_detection detection.launch```
    with this command you will launch yolov7 for object detection and the detection by color too. In this package there are also the file ```take_photo.py``` and ```color_segmentation.py``` which are usefull to choose the color to detect from an image: first take a photo using ```rosrun robutler_detection take_photo.py``` then use ```color_segmenter.py``` to find the values of hsv to detect the color that you prefer.Note: there is also present the launch file for start individually yolo, and the python script for the color detection can be used with rosrun command

- psr_apartment_description: this package contain the information about the apartment for gazebo, moreover it contain the ```spawn_object.py``` which can be used to spawn objects in the apartment either randomly or choosing a desired position and the desired object. Use:```rosrun psr_apartment_description spawn_object.py -object [object to spawn] -room [room location]```use -h to see the possible spawn configuration.

- robutler_menu: contain the interactive menu that allow you to run all the above command from rviz using the menu on the robutler and also contain some composed mission like search a purple sphere. Use: ```roslaunch robutler_menu launch.launch```
## Authors

- [@Contin1999](https://github.com/Contin1999)
- [@MojganGhanbari](https://github.com/MojganGhanbari)


## License

[MIT](https://choosealicense.com/licenses/mit/)

