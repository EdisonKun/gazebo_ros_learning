# gazebo_ros_learning
Tips: This file is to upload the basic controller on the quadruped model.

1. The namespace of the controller should be attentioned.
   In the simpledog_control.launch file

```cpp
   <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
 	output="screen" ns="/simpledog" args="joint_state_controller"/>
  ```
Which corresponds to the controller load yaml file: simpledog_control.yaml

```cpp
  simpledog:
    # Publish all joint states -----------------------------------
    joint_state_controller:
      type: joint_state_controller/JointStateController
      publish_rate: 50
  ```
  And the gazebo tag in the urdf.xacro file
```cpp
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
      <robotNamespace>/simpledog</robotNamespace>
      <legacyModeNS>true</legacyModeNS>
    </plugin>
  </gazebo>
  ```
  Otherwise,it will causes the following error:
  ![controller_ns_error](images/2019/07/controller-ns-error.png)
  And the following picture shows the correct running:
  ![Correct_running](images/2019/07/correct-running.png)    
2.Send the control command, using the following code:
```c++
rostopic pub -1 /simpledog/joint1_position_controller/command std_msgs/Float64 "data: 2.5"
```
You will get the following response:     
![gazebo_response](images/2019/07/gazebo-response.png)    

_**Attention**_: In the yaml file,when you set the PID, you have a space between the P: and the _P number_: P:(SPACE)100.0.
