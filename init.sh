source ~/.bashrc
cd ~/catkin_ws
source devel/setup.bash
roscore &
sleep 3
cd ~/Programs/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04
./coppeliaSim.sh ~/simulation/track_environment.ttt &
cd ~/PycharmProjects/aut_driving_competition_ready
pycharm-community . &
