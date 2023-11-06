#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <queue>
#include <map>
#include <phone_finder/phone_finder.hpp>
#include "nlink_parser/LinktrackNodeframe2.h"
#include "nlink_parser/LinktrackNode2.h"

const double GD_STEP_LEN = 1e-2; // 梯度下降步长
const int N_GD_STEPS = 1000; // 梯度下降步数
const double MAX_DIST = 1e10; // 最大距离
const int MAX_N_SAMPLES = 100; // 最大采样点个数
const int MAX_N_SOLUTIONS = 20; // 最大解算位置个数
const double CRUISING_ALTITUDE = 5; // 巡航高度（m）
const double STD_THRES = 0.25; // 判断距离收敛的标准差阈值

using namespace std;

class FindTheTargetHardware: public FlightControlNode{
public:
    int offboard_setpoint_counter; // 为系统提供初始化时间
    int trajectory_counter; // 负责画圈的计数器
    int detect_target_counter; // 用来判断是原地绕圈还是去寻找目标
    ros::Timer pos_ctrl_timer; // 位置控制
    ros::Timer locate_timer; // 定位目标
    ros::Timer guidance_timer; // 为位置控制指定目标点
    ros::Subscriber locator_sub;

    const Point home; // 起始点
    Point destination; // 目标点
    bool return_home;

    map<id_type, int> target_map; // 从目标id到Target对象在targets向量中的下标的映射
    vector<Target> targets; // 存储目标信息的列表

    FindTheTargetHardware(): home(0, 0, 2), destination(home)
    {
        offboard_setpoint_counter = 0;
        trajectory_counter = 0;
        detect_target_counter = 0;
        pos_ctrl_timer = nh.createTimer(ros::Duration(0.1), bind(&FindTheTargetHardware::pos_ctrl_cb, this), false);
        locate_timer   = nh.createTimer(ros::Duration(1.0), bind(&FindTheTargetHardware::locate_cb, this), false);
        guidance_timer = nh.createTimer(ros::Duration(1.0), bind(&FindTheTargetHardware::guidance_cb, this), false);
        // 订阅距离信息的Subscriber
        locator_sub = nh.subscribe("nlink_linktrack_nodeframe2", 1000, &FindTheTargetHardware::distCallback, this);

        return_home = false;
        targets.clear();
        target_map.clear();
    }
    static void pos_ctrl_cb(FindTheTargetHardware*);
    static void locate_cb(FindTheTargetHardware*);
    static void guidance_cb(FindTheTargetHardware*);
    void distCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg); // 订阅距离信息的回调函数
    int decide_current_target();
};

// 位置控制回调函数，每0.1秒发出一个位置指令
// 把无人机指引到home或destination的位置
void FindTheTargetHardware::pos_ctrl_cb(FindTheTargetHardware* node){
    if (node->offboard_setpoint_counter < 10){
        node->offboard_setpoint_counter++;
    }
    else if (node->offboard_setpoint_counter == 10){
        if (!node->set_mode("OFFBOARD") || !node->arm()){
            ROS_FATAL("Failed to switch to OFFBOARD mode and arm the vehicle.");
            exit(EXIT_FAILURE);
        }
        node->offboard_setpoint_counter++;
    }
    node->trajectory_counter++;
    if (node->return_home){
        node->set_pos(node->home);
    }else{
        double radius = 5;
        double theta = double(node->trajectory_counter) / 30.;
        node->set_pos(
            node->destination.x + radius * sin(theta),
            node->destination.y + radius * cos(theta),
            CRUISING_ALTITUDE
        );
    }
}

// 下面这段程序是加入了定位器信息的，原先的程序在上面被注释掉了
// 修改的部分主要是将id换成了target_id中的id，距离换成了locator_dist
void FindTheTargetHardware::locate_cb(FindTheTargetHardware* node){
    for (int i = 0; i < node->targets.size(); i++){
        if (!node->targets[i].solved){
            bool success = node->targets[i].solve(N_GD_STEPS, GD_STEP_LEN);
            if (success && node->targets[i].converged(STD_THRES)){
                ROS_WARN("Target(id=%s) located at x=%.3lf, y=%.3lf\n", 
                    node->targets[i].id.c_str(),
                    node->targets[i].solutions.mean().x,
                    node->targets[i].solutions.mean().y);
                node->targets[i].solved = true;
            }
        }
    }
}

// 用于订阅距离信息的回调函数
void FindTheTargetHardware::distCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg){
    // 接收到的信息msg中的nodes是LinktrackNode2类型的，距离信息dis和id是nodes的属性
    // nodes是一个vector，两个定位器的情况下nodes只有一个，因此距离信息存储在nodes[0]中
    if(msg->nodes.size() != 0){  // 判断是否有数据丢失，若数据丢失，nodes的size为0
        for(int i = 0; i < msg->nodes.size(); i++){
            id_type id = to_string(msg->nodes[i].id);
            if (target_map.find(id) == target_map.end()){
                target_map.insert(pair<id_type, int>(id, targets.size()));
                targets.push_back(Target(id, MAX_N_SAMPLES, MAX_N_SOLUTIONS));
                targets.back().sample(pos, msg->nodes[i].dis);
            }
            else{
                Target &target = targets[target_map[id]];
                if (!target.solved){
                    target.sample(pos, msg->nodes[i].dis);
                }
            }
        }
    }   
    else
        ROS_WARN("Data loss.");
}

// 判断哪个目标距离当前无人机的位置最近
int FindTheTargetHardware::decide_current_target(){
    double min_dist = MAX_DIST;
    int next_target_index = -1;
    for (int i = 0; i < targets.size(); i++){
        double dist = distance(this->pos, targets[i].solutions.mean());
        if (!targets[i].solved && dist < min_dist){
            min_dist = dist;
            next_target_index = i;
        }
    }
    return next_target_index;
}

// 将目标点设定为距离当前无人机位置最近的，距离还没被解算成功的目标点（贪心）
void FindTheTargetHardware::guidance_cb(FindTheTargetHardware* node){
    node->detect_target_counter ++;
    // 起飞，在home点悬停
    if (node->detect_target_counter < 10){
        node->return_home = true;
    }
    // 围绕home点画圈，收集数据
    else if (node->detect_target_counter < 20){
        node->return_home = false;
        node->destination = node->home;
    }
    // 开始逐个收集目标点数据，解算其位置
    else{
        int current_target = node->decide_current_target();
        if (current_target != -1){
            // ! 实验室太小，为了防止飞机乱跑，就先不让飞机跟踪目标了，起飞之后原地转圈就行
            // node->destination = node->targets[current_target].solutions.queue.back();
            // node->destination.z = 5;
            node->return_home = false;
            node->destination = node->home;
            ROS_INFO("Locating Target(id=%s): x=%.3lf, y=%.3lf, d=%.3f",
                node->targets[current_target].id.c_str(),
                node->targets[current_target].solutions.mean().x,
                node->targets[current_target].solutions.mean().y,
                node->targets[current_target].distances.back());
        }
        else{
            ROS_WARN("All targets located, returning home.");
            node->return_home = true;
        }
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "phone_detector");
    FindTheTargetHardware node;
    ros::spin();
    return 0;
}