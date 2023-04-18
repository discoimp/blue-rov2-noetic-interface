// include all libraries referenced in the code




using mavros_msgs::Mavlink;
using mavconn::MAVConnInterface;

const uint16_t UDP_PORT = 14552;
const std::string ROS_TOPIC = "mavlink_all_messages";

void mavlink_to_ros_msg(const mavlink_message_t &mavlink_msg, Mavlink &ros_msg)
{
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.header.frame_id = "mavlink";
    ros_msg.framing_status = Mavlink::FRAMING_OK;
    ros_msg.magic = mavlink_msg.magic;
    ros_msg.len = mavlink_msg.len;
    ros_msg.seq = mavlink_msg.seq;
    ros_msg.sysid = mavlink_msg.sysid;
    ros_msg.compid = mavlink_msg.compid;
    ros_msg.msgid = mavlink_msg.msgid;

    size_t payload64_len = (mavlink_msg.len + 7) / 8;
    ros_msg.payload64.resize(payload64_len);
    memcpy(ros_msg.payload64.data(), mavlink_msg.payload64, payload64_len * sizeof(uint64_t));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavlink_to_ros_node");
    ros::NodeHandle nh;

    ros::Publisher mavlink_pub = nh.advertise<Mavlink>(ROS_TOPIC, 10);

    auto conn = MAVConnInterface::open_url("udpin:0.0.0.0:" + std::to_string(UDP_PORT));
    conn->message_received_cb = [&](const mavlink_message_t *msg, const mavconn::Framing framing) {
        Mavlink ros_msg;
        mavlink_to_ros_msg(*msg, ros_msg);
        mavlink_pub.publish(ros_msg);
    };

    ROS_INFO_STREAM("Publishing all MAVLink messages on " << ROS_TOPIC << " topic...");

    ros::spin();

    return 0;
}
