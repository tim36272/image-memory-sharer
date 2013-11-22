catkin_make
source devel/setup.bash
rosbag play sample.bag -l &
rosrun image_memory_sharer RosReceiver image:=/camera/rgb/image_color &
sleep 2
roslaunch three_windows.launch
