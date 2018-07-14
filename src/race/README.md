**Initial Setup**
- Suppose you haven't created a catkin workspace yet. Then, do the following:
    ```
    source /opt/ros/kinetic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
- Copy `race` directory into your catkin workspace
    ```
    cp -r ~/selfdrivingcar/code/ROS/race/ ~/catkin_ws/src/
    ```
- Make
    ```
    cd ~/catkin_ws
    catkin_make
    ```
- Change the permissions of the python files so that they are executable
    ```
    chmod +x src/race/src/PYTHON_FILE
    ```

**How to run**
- You need to execute xboxdrv in order to use an xbox controller. 
  ```
  sudo xboxdrv
  ```
  Sometime it crashes. If it happens, kill it (or use https://github.com/CertiKOS/selfdrivingcar/blob/master/code/ROS/new_race/src/kill_xboxdrv.sh) and then relaunch it.
- race_launch.launch will launch some core ROS nodes (including roscore) necessary for driving the car
  ```
  roslaunch race race_launch.launch
  ```
- After launching the script, you should be able to see `joy` topic giving you some messages. Try `rostopic echo joy`.
- In case the xbox controller is not recognized (i.e., `joy` topic does not show you any message), kill `race_launch.launch` script (Ctrl+C) and then change `/dev/input/js0` to `/dev/input/js1` (or vice versa) in `race_launch.launch`. Then, re-launch it. Don't forget to relaunch any other nodes that have been executing since the previous `race_launch.launch`.
- By just executing the script and `xbox_control.py`, you should be able to drive the car. 
- To use camera images, subscribe `camera` topic. That means, you need to run `camera_feeder` node. 
- To use IMU, 
    ```
    roslaunch race razor-pub.launch
    ```
    then you can use `imu` topic.
  
