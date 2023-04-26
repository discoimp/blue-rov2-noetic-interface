source /opt/ros/noetic/setup.bash
echo "ROS Noetic sourced"
source ~/catkin_ws/devel/setup.bash
echo -e "catkin_ws sourced\n"
source ~/uslam_ws/devel/setup.bash
echo -e "uslam_ws sourced\n"

virtualhere=false
roscore=false
davis_ros_driver=false
dv_runtime=false

# Check and set VirtualHere status
pgrep -x "vhuit64" >/dev/null 2>&1 && virtualhere=true

# Check and set ROS core status
pgrep -x "roscore" >/dev/null && roscore=true

# Check and set Davis ROS driver status if both ROS core and VirtualHere are true
$roscore && $virtualhere && (rosnode ping -c 1 /davis_ros_driver | grep "time=") >/dev/null 2>&1 && davis_ros_driver=true

# Check and set DV-Runtime status if VirtualHere is true
$virtualhere && pgrep -x "dv-runtime" >/dev/null 2>&1 && dv_runtime=true

# Print status table
printf "\e[%sm%-25s\e[0m\n" $([[ $virtualhere = true ]] && echo "32" || echo "31") "VirtualHere USB"
printf "\e[%sm%-25s\e[0m\n" $([[ $roscore = true ]] && echo "32" || echo "31") "ROS core"
$roscore && $virtualhere && printf "\e[%sm%-25s\e[0m\n" $([[ $davis_ros_driver = true ]] && echo "32" || echo "31") "Davis ROS driver"
$virtualhere && printf "\e[%sm%-25s\e[0m\n" $([[ $dv_runtime = true ]] && echo "32" || echo "31") "DV-Runtime"
