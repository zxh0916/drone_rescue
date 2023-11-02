#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <queue>
#include <map>
#include <phone_finder/phone_finder.hpp>

const double GD_STEP_LEN = 1e-2; // 梯度下降步长
const int N_GD_STEPS = 1000; // 梯度下降步数
const double MAX_DIST = 1e10; // 最大距离
const int MAX_N_SAMPLES = 100; // 最大采样点个数
const int MAX_N_SOLUTIONS = 20; // 最大解算位置个数
const double CRUISING_ALTITUDE = 5; // 巡航高度（m）
const double STD_THRES = 0.1; // 判断距离收敛的标准差阈值

using namespace std;

class FindTheTarget: public FlightControlNode{
public:
    int offboard_setpoint_counter; // 为系统提供初始化时间
    int trajectory_counter; // 负责画圈的计数器
    int detect_target_counter; // 用来判断是原地绕圈还是去寻找目标
    ros::Timer pos_ctrl_timer; // 位置控制
    ros::Timer locate_timer; // 定位目标
    ros::Timer guidance_timer; // 为位置控制指定目标点

    const Point home; // 起始点
    Point destination; // 目标点
    bool return_home;

    vector<SignalSource> signal_sources; // 定位器所接收到的目标列表
    map<id_type, int> target_map; // 从目标id到Target对象在targets向量中的下标的映射
    vector<Target> targets; // 存储目标信息的列表

    FindTheTarget(): home(0, 0, 5), destination(home)
    {
        offboard_setpoint_counter = 0;
        trajectory_counter = 0;
        detect_target_counter = 0;
        pos_ctrl_timer = nh.createTimer(ros::Duration(0.1), bind(&FindTheTarget::pos_ctrl_cb, this), false);
        locate_timer = nh.createTimer(ros::Duration(1.), bind(&FindTheTarget::locate_cb, this), false);
        guidance_timer = nh.createTimer(ros::Duration(1.), bind(&FindTheTarget::guidance_cb, this), false);

        return_home = false;
        targets.clear();
        target_map.clear();
        signal_sources.clear();
        signal_sources.push_back(SignalSource("a",  100., -150.,  -5.));
        signal_sources.push_back(SignalSource("b", -200.,  150., -10.));
        signal_sources.push_back(SignalSource("c",  500.,  300.,   0.));
    }
    static void pos_ctrl_cb(FindTheTarget*);
    static void locate_cb(FindTheTarget*);
    static void guidance_cb(FindTheTarget*);
    int decide_current_target();
};

// 位置控制回调函数，每0.1秒发出一个未知指令
// 把无人机指引到home或destination的位置
void FindTheTarget::pos_ctrl_cb(FindTheTarget* node){
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
        double radius = 10;
        double theta = double(node->trajectory_counter) / 30.;
        node->set_pos(
            node->destination.x + radius * sin(theta),
            node->destination.y + radius * cos(theta),
            CRUISING_ALTITUDE
        );
    }
}

// 根据当前无人机自身位置和到目标的距离
// 利用梯度下降法解算目标位置
void FindTheTarget::locate_cb(FindTheTarget* node){
    // TODO: signal_sources是一个包含了虚拟目标点的列表，
    // TODO: 进行硬件实验之前应该将其替换成包含真实定位目标信息的列表
    for (int i = 0; i < node->signal_sources.size(); i++){
        // 如果某个信号源还没有在targets中注册，就在targets列表中追加一个Target实例
        // 用以存储信号源的信息
        // 并在target_map中增加条目(key, value) = (信号源的id, 信号源所对应Target实例在targets列表中的下标)
        if (node->target_map.find(node->signal_sources[i].id) == node->target_map.end()){
            node->target_map.insert(
                pair<id_type, int>(node->signal_sources[i].id, node->targets.size()));
            node->targets.push_back(Target(
                node->signal_sources[i].id, MAX_N_SAMPLES, MAX_N_SOLUTIONS));
            node->targets.back().sample(node->pos, node->signal_sources[i](node->pos));
        }
        // 如果某个信号源已有与之对应的Target实例，则进行无人机位置和与信号源之间距离的采样
        // 如果已经收集了足够多的数据，则解算目标位置并进行记录
        else {
            Target &target = node->targets[node->target_map[node->signal_sources[i].id]];
            if (!target.solved){
                // TODO: 进行硬件实验前，把下面sample函数的第二项距离换成从无人机到信号源的真实距离
                target.sample(node->pos, node->signal_sources[i](node->pos));
                bool success = target.solve(N_GD_STEPS, GD_STEP_LEN);
                if (success && target.converged(STD_THRES)){
                    ROS_WARN("Target(id=%s) located at x=%.3lf, y=%.3lf\n", 
                        target.id.c_str(),
                        target.solutions.mean().x,
                        target.solutions.mean().y);
                    target.solved = true;
                }
            }
        }
    }
}

// 判断哪个目标距离当前无人机的位置最近
int FindTheTarget::decide_current_target(){
    double min_dist = MAX_DIST;
    int next_target_index = -1;
    for (int i = 0; i < targets.size(); i++){
        double dist = distance(this->pos, targets[i].solutions.mean());
        if (dist < min_dist && !targets[i].solved){
            min_dist = dist;
            next_target_index = i;
        }
    }
    return next_target_index;
}

// 将目标点设定为距离当前无人机位置最近的，距离还没被解算成功的目标点（贪心）
void FindTheTarget::guidance_cb(FindTheTarget* node){
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
            node->destination = node->targets[current_target].solutions.queue.back();
            node->destination.z = 5;
            ROS_INFO("Locating Target(id=%s): x=%.3lf, y=%.3lf",
                node->targets[current_target].id.c_str(),
                node->targets[current_target].solutions.mean().x,
                node->targets[current_target].solutions.mean().y);
        }
        else{
            ROS_WARN("All targets located, returning home.");
            node->return_home = true;
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "phone_detector");
    FindTheTarget node;
    ros::spin();
    return 0;
}