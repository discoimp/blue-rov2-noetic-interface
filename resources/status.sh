source /opt/ros/noetic/setup.bash
echo "ROS Noetic sourced"
source ~/catkin_ws/devel/setup.bash
echo "catkin_ws sourced"
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

alias launch_vhuit64='sudo ~/catkin_ws/src/blue-rov2-noetic-interface/resources/vhuit64'
alias launch_QGC='~/catkin_ws/src/blue-rov2-noetic-interface/resources/QGroundControl.AppImage'
echo -e "\nAdded aliases for vhuit64 and QGroundControl:\nlaunch_vhuit64\nlaunch_QGC"

# This part only runs once to set up github for you

#!/bin/bash

CONFIG_TOKEN="$HOME/.git_token_config"
CONFIG_GLOBAL="$HOME/.gitconfig"

if [ ! -f "$CONFIG_GLOBAL" ]; then
  echo
  echo "Github initial setup:"
  read -p "Enter your Git user name: " GIT_USER_NAME
  read -p "Enter your Git email: " GIT_USER_EMAIL

  git config --global user.name "$GIT_USER_NAME"
  git config --global user.email "$GIT_USER_EMAIL"

  echo "Git global user name and email have been set."
fi

if [ ! -f "$CONFIG_TOKEN" ]; then
  echo "Leave blank if you don't wish to configure access tokens now"
  read -p "Enter your GitHub repository URL (e.g., https://github.com/username/repo.git): " REPO_URL
  read -s -p "Enter your access token (leave empty to skip): " ACCESS_TOKEN
  echo

  if [ -z "$ACCESS_TOKEN" ]; then
    echo "No access token provided. Skipping."
    echo "To rerun delete:"
    echo $CONFIG_TOKEN
    echo "and open a new terminal session"
    touch "$CONFIG_TOKEN"
    return 0
  fi
  # disassemble URL into parts to build the new variable
  BASE_URL=$(echo "$REPO_URL" | awk -F/ '{print $3}')
  USERNAME=$(echo "$REPO_URL" | awk -F/ '{print $4}')
  REPO_NAME=$(echo "$REPO_URL" | awk -F/ '{print $5}')
  REPO_URL_WITH_TOKEN="https://${USERNAME}:${ACCESS_TOKEN}@${BASE_URL}/$USERNAME/${REPO_NAME}"

  # Change to the repository directory if it exists
  REPO_DIR="$HOME/catkin_ws/src/$REPO_NAME"
  if [ -d "$REPO_DIR" ]; then
    cd "$REPO_DIR"
  else
    echo "The repository directory $REPO_DIR does not exist. Please ensure the path is correct."
    echo "You can manually run the following command after changing to the repository directory:"
    echo "git remote set-url origin \"$REPO_URL_WITH_TOKEN\""
    return 1
  fi

  git remote set-url origin "$REPO_URL_WITH_TOKEN" 2>/dev/null

