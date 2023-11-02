#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <queue>
#include <map>
#include <phone_finder/phone_finder.hpp>

const double GD_STEP_LEN = 1e-2;
const int N_GD_STEPS = 1000;
const double MAX_DIST = 1e10;
const int MAX_N_SAMPLES = 100;
const int MAX_N_SOLUTIONS = 10;
const double EMA_STEP_SIZE = 0.1;
const double CRUISING_ALTITUDE = 5;

using namespace std;

class FindTheTarget: public FlightControlNode{
public:
    int offboard_setpoint_counter;
    int trajectory_counter;
    int detect_target_counter;
    ros::Timer pos_ctrl_timer;
    ros::Timer locate_timer;
    ros::Timer guidance_timer;

    const Point home;
    Point destination;
    bool return_home;

    vector<SignalSource> signal_sources;
    map<id_type, int> target_map;
    vector<Target> targets;

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
    int decide_next_target();
};

void FindTheTarget::pos_ctrl_cb(FindTheTarget* node){
    if (node->offboard_setpoint_counter < 10){
        node->offboard_setpoint_counter++;
    }
    else if (node->offboard_setpoint_counter == 10){
        node->set_mode("OFFBOARD");
        node->arm();
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

void FindTheTarget::locate_cb(FindTheTarget* node){
    // TODO: replace vector signal_sources by a list of real signal sources
    for (int i = 0; i < node->signal_sources.size(); i++){
        if (node->target_map.find(node->signal_sources[i].id) == node->target_map.end()){
            node->target_map.insert(
                pair<id_type, int>(node->signal_sources[i].id, node->targets.size()));
            node->targets.push_back(Target(
                node->signal_sources[i].id, MAX_N_SAMPLES, MAX_N_SOLUTIONS, EMA_STEP_SIZE));
            node->targets.back().sample(node->pos, node->signal_sources[i](node->pos));
        }
        else {
            Target &target = node->targets[node->target_map[node->signal_sources[i].id]];
            if (!target.solved){
                target.sample(node->pos, node->signal_sources[i](node->pos));
                if (target.sampled_postions.size() > target.max_n_samples / 10){
                    bool success = target.solve(N_GD_STEPS, GD_STEP_LEN);
                    if (success && target.converged()){
                        ROS_WARN("Target(id=%s) found at x=%.3lf, y=%.3lf\n", 
                            target.id.c_str(),
                            target.solutions.mean().x,
                            target.solutions.mean().y);
                        target.solved = true;
                    }
                }
            }
        }
    }
}

int FindTheTarget::decide_next_target(){
    double min_dist = MAX_DIST;
    int next_target_index = -1;
    for (int i = 0; i < targets.size(); i++){
        double dist = distance(this->pos, targets[i].solutions.queue.back());
        if (dist < min_dist && !targets[i].solved){
            min_dist = dist;
            next_target_index = i;
        }
    }
    return next_target_index;
}

void FindTheTarget::guidance_cb(FindTheTarget* node){
    node->detect_target_counter ++;
    if (node->detect_target_counter < 10){
        node->return_home = true;
    }
    else if (node->detect_target_counter < 20){
        node->return_home = false;
        node->destination = node->home;
    }
    else{
        int current_target = node->decide_next_target();
        if (current_target != -1){
            node->destination = node->targets[current_target].solutions.queue.back();
            node->destination.z = 5;
            ROS_INFO("Finding Target(id=%s): x=%.3lf, y=%.3lf",
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