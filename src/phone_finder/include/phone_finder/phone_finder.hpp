#include <vector>
#include <math.h>
#include <iostream>
#include <flight_control/FlightControl.hpp>
#include <random>
#include <time.h>

using namespace std;

typedef string id_type;

class MovingStatistics{
    public:
        Point mean_data;
        Point std_data;
        deque<Point> queue;
        const int queue_size;
        const double step_size;
        MovingStatistics(double step_size, int queue_size):
            step_size(step_size), queue_size(queue_size), 
            mean_data(), std_data(){
                queue.clear();
            }
        void update(const Point& x){
            // 把x存在queue里
            queue.push_back(x);
            while (queue.size() > queue_size){
                queue.pop_front();
            }
            if (!queue.empty()){
                mean_data = Point();
                for (int i = 0; i < queue.size(); i++){
                    mean_data += queue[i] / queue.size();
                }
                std_data = Point();
                for (int i = 0; i < queue.size(); i++){
                    std_data += power(queue[i] - mean_data, 2) / queue.size();
                }
                std_data.x = sqrt(std_data.x);
                std_data.y = sqrt(std_data.y);
                std_data.z = sqrt(std_data.z);
            }
        }
        const Point& mean(){
            return mean_data;
        }
        const Point& std(){
            return std_data;
        }
};

class Target{
    public:
        id_type id;
        MovingStatistics solutions;
        deque<Point> sampled_postions;
        deque<double> distances;
        int max_n_samples;
        int max_n_solutions;
        bool solved;

        Target(
            id_type id,
            int max_n_samples,
            int max_n_solutions,
            double ema_step_size
        ):
            solutions(ema_step_size, max_n_solutions)
        {
            this->id = id;
            sampled_postions.clear();
            distances.clear();
            this->max_n_samples = max_n_samples;
            solved = false;
        }
        void sample(Point& sample_pos, double sampled_dist){
            sampled_postions.push_back(sample_pos);
            distances.push_back(sampled_dist);
            while (sampled_postions.size() > max_n_samples)
                sampled_postions.pop_front();
            while (distances.size() > max_n_samples)
                distances.pop_front();
        }
        bool solve(int n_steps, double step_len){
            Point pos;
            vd mse;
            mse.clear();
            double dist = 1e10;
            int init_id = -1;
            for (int i = 0; i < distances.size(); i++){
                if (distances[i] < dist){
                    pos = sampled_postions[i];
                    dist = distances[i];
                    init_id = i;
                }
            }
            for (int step = 0; step < n_steps; step++){
                Point grad;
                mse.push_back(0.);
                double pred_dist, num_x, num_y, num_z, error;
                for (int i = 0; i < distances.size(); i++){
                    if (i == init_id) continue;
                    pred_dist = sqrt(pow(pos.x - sampled_postions[i].x, 2)
                                    +pow(pos.y - sampled_postions[i].y, 2)
                                    +pow(pos.z - sampled_postions[i].z, 2));
                    error = distances[i] - pred_dist;
                    num_x = -2 * (pos.x - sampled_postions[i].x) * error;
                    num_y = -2 * (pos.y - sampled_postions[i].y) * error;
                    num_z = -2 * (pos.z - sampled_postions[i].z) * error;
                    grad.x += num_x / pred_dist;
                    grad.y += num_y / pred_dist;
                    grad.z += num_z / pred_dist;
                    mse.back() += pow(error, 2) / distances.size();
                }
                pos -= grad * step_len;
            }
            bool success = mse[0] > mse.back();
            if (success){
                solutions.update(pos);
            }
            return success;
        }
        bool converged(){
            if (solutions.queue.size() < 10)
                return false;
            Point last_std = solutions.std();
            double std = (last_std.x + last_std.y + last_std.z) / 3.;
            return (std < 0.1);
        }
};

class SignalSource{
private:
    Point pos;
public:
    id_type id;
    SignalSource(id_type id, double x, double y, double z):
        pos(x, y, z), id(id) {}
    double operator()(const Point &dest){
        mt19937 gen((unsigned int) time(nullptr));
        normal_distribution<double> dis(0., 1.);
        double dist = sqrt(pow(pos.x - dest.x, 2)
                         + pow(pos.y - dest.y, 2)
                         + pow(pos.z - dest.z, 2));
        double additive_noise = dis(gen) * 0.01 * dist;
        return dist + additive_noise;
    }
};
