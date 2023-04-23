# rl2023_ex5_P38000151

This repository contains the implementation for the simulation of a periodic trajectory in gazebo for the KUKA IIWA. The gazebo enviroment can be run with
```
$ roslaunch rl2023_ex5 gazebo_ctrl.launch
```

After the simulation enviroment has been loaded the program that computes the inverse kinematic and publish the joints command can be run with

```
$ rosrun rl2023_ex5 kuka_invkin_ctrl_ex5
```

The program reads the forward and backward trajectory from the two text files provided and moves back and forth periodically (T = 8s) between two given points.
Joint commands are logged in the file q_out.txt.
