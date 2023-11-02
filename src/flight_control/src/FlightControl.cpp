#include <flight_control/FlightControl.hpp>

FlightControlNode::FlightControlNode(){
    this->state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, bind(&state_cb, _1, this));
    this->odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("mavros/odometry/in", 10, bind(&odom_cb, _1, this));
    this->local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    this->vel_pub = nh.advertise<geometry_msgs::TwistStamped>
        ("mavros/setpoint_velocity/cmd_vel", 10);
    this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
}

void FlightControlNode::odom_cb(
    const nav_msgs::Odometry::ConstPtr &msg,
    FlightControlNode *node){
    node->pos.x = msg->pose.pose.position.x;
    node->pos.y = msg->pose.pose.position.y;
    node->pos.z = msg->pose.pose.position.z;
    node->vel.x = msg->twist.twist.linear.x;
    node->vel.y = msg->twist.twist.linear.y;
    node->vel.z = msg->twist.twist.linear.z;
    return;
}

void FlightControlNode::state_cb(
    const mavros_msgs::State::ConstPtr& msg,
    FlightControlNode *node){
    node->current_state = *msg;
}

void FlightControlNode::set_pos(double x, double y, double z){
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    msg.header.stamp = ros::Time::now();
    local_pos_pub.publish(msg);
    return;
}

void FlightControlNode::set_pos(const Point& pos){
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = pos.x;
    msg.pose.position.y = pos.y;
    msg.pose.position.z = pos.z;
    msg.header.stamp = ros::Time::now();
    local_pos_pub.publish(msg);
    return;
}

void FlightControlNode::set_vel(double vx, double vy, double vz){
    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    msg.header.stamp = ros::Time::now();
    vel_pub.publish(msg);
    return;
}

void FlightControlNode::set_vel(const Point& vel){
    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = vel.x;
    msg.twist.linear.y = vel.y;
    msg.twist.linear.z = vel.z;
    msg.header.stamp = ros::Time::now();
    vel_pub.publish(msg);
    return;
}

bool FlightControlNode::arm(){
    arm_cmd.request.value = true;
    ROS_INFO("Vehicle Armed.");
    return this->arming_client.call(arm_cmd);
}

bool FlightControlNode::disarm(){
    arm_cmd.request.value = false;
    ROS_INFO("Vehicle Disarmed.");
    return this->arming_client.call(arm_cmd);
}

bool FlightControlNode::set_mode(std::string mode="OFFBOARD"){
    offb_set_mode.request.custom_mode = mode;
    ROS_INFO("Vihicle mode set to %s.", mode.c_str());
    return this->set_mode_client.call(offb_set_mode);
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "offb_node");
    FlightControlNode node;
    
    ros::Rate rate(20.0);
    while(ros::ok() && !node.current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        node.set_pos(0., 0., 2.);
        ros::spinOnce();
        rate.sleep();
    }
    
    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        if( node.current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(node.set_mode() && node.offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !node.current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( node.arm() && node.arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        node.set_pos(0., 0., 2.);

        ros::spinOnce();
        rate.sleep();
        std::cout << node.pos.z << std::endl;
    }
}