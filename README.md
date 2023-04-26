# BlueROV2-Noetic-interface
Mavlink to rosbag including Event camera. Using BlueOS (ADD VERSION).

## Install (assuming you have a catkin_ws initialised)
```
cd ~/catkin_ws/src
git clone https://github.com/discoimp/blue-rov2-noetic-interface.git
# if you don't have curl, download files manually or install with sudo apt install curl
curl -o ~/catkin_ws/blue-rov2-noetic-interface/resources/vhuit64 -LO https://www.virtualhere.com/sites/default/files/usbclient/vhuit64
curl -o ~/catkin_ws/blue-rov2-noetic-interface/resources/QGroundControl.AppImage -LO https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
```

If running with QGroundControl in parallel, make sure to configure a different endpoint. More information [here](https://docs.bluerobotics.com/ardusub-zola/software/onboard/BlueOS-1.0/advanced-usage/#mavlink-endpoints)

#testbag:
https://drive.google.com/file/d/1icbxs7BnQ-Fbpe4AsNEASfXFrdqn45gh/view?usp=sharing
