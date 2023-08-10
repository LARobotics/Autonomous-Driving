README

This project was developed with:
- Ubuntu 20.04
- ROS noetic V1.15.14
- CoppeliaSim Educational V4.3.0 
- Pycharm Comunity with python 3.8

create/enter workspace (catkin_ws) and 
source devel/setup.bash

initiate ROS (roscore &)
after Ros initiated open Coppelia simulation track_environment.ttt 

lastly go to PycharmProjects/aut_driving_competition_ready folder
it contains the main codes, the dataset and robotic car results
(*run init.sh to do all these steps)

- CV_simulation_ros.py - uses the computer vision algorithm
- network_model_simulation_real.py - imports one of the models created to test in simulation or robotic car
- test_img_by_img.py - imports one of the models created, selects a folder and returns the images result using the neural network model

lap6, lap14, lap17, lap27, lap38, lap39 and lap41 folders contain the dataset images
lap10, lap11, lap28, lap30, lap108, and lap110 are additional images extracted from the first tests of each model to be used for image by image test or other tests
models folder contains the different neural network models created

model2 was trained with 1289 (lap6)
model3 was trained with 3490 (+lap14)
model4 was trained with 4366 (+lap17+lap27)
model5 was trained with 5906 (+lap38+lap39+lap41)

