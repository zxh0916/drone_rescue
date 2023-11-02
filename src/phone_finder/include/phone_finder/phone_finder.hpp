#include <vector>
#include <math.h>
#include <iostream>
#include <flight_control/FlightControl.hpp>
#include <random>
#include <time.h>

using namespace std;

typedef string id_type;

// 用来记录目标坐标解的统计信息的数据结构
class MovingStatistics{
    public:
        Point mean_data; // 平均坐标
        Point std_data; // 三坐标方差
        deque<Point> queue; // 记录历史坐标的队列
        const int queue_size;
        MovingStatistics(int queue_size):
            queue_size(queue_size), mean_data(), std_data(){
                queue.clear();
            }
        // 更新数据
        void update(const Point& x){
            // 将新增数据入队
            queue.push_back(x);
            while (queue.size() > queue_size){
                queue.pop_front(); // 将最老的历史数据出队
            }
            // 计算统计信息
            if (!queue.empty()){
                // 计算均值
                mean_data = Point();
                for (int i = 0; i < queue.size(); i++){
                    mean_data += queue[i] / queue.size();
                }
                // 计算方差
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

// 用于在类FindTheTarget中存储目标信息的数据结构
class Target{
    public:
        id_type id;
        MovingStatistics solutions;
        deque<Point> sampled_positions; // 采样与目标点距离时无人机自身位置
        deque<double> distances; // 采样到的与目标点的距离
        int max_n_samples;
        int max_n_solutions;
        bool solved; // 目标点的位置是否已经解算成功

        Target(
            id_type id,
            int max_n_samples,
            int max_n_solutions
        ):
            solutions(max_n_solutions)
        {
            this->id = id;
            sampled_positions.clear();
            distances.clear();
            this->max_n_samples = max_n_samples;
            this->max_n_solutions = max_n_solutions;
            solved = false;
        }

        // 记录无人机当前时刻的坐标和其到目标点的距离
        void sample(Point& sample_pos, double sampled_dist){
            sampled_positions.push_back(sample_pos);
            distances.push_back(sampled_dist);
            while (sampled_positions.size() > max_n_samples)
                sampled_positions.pop_front();
            while (distances.size() > max_n_samples)
                distances.pop_front();
        }

        // 根据过去收集到的坐标和距离信息解算目标位置
        bool solve(int n_steps, double step_len){
            // 至少收集10条数据才进行解算
            if (sampled_positions.size() < 10) return false;
            Point pos;
            vd mse; // 平均误差平方
            mse.clear();
            // 用与目标距离最近的采样点位置初始化目标位置
            double dist = 1e10;
            int init_id = -1;
            for (int i = 0; i < distances.size(); i++){
                if (distances[i] < dist){
                    pos = sampled_positions[i];
                    dist = distances[i];
                    init_id = i;
                }
            }
            // 运行梯度下降法
            for (int step = 0; step < n_steps; step++){
                Point grad;
                mse.push_back(0.);
                double pred_dist, error;
                for (int i = 0; i < distances.size(); i++){
                    if (i == init_id) continue;
                    pred_dist = distance(pos, sampled_positions[i]);
                    error = distances[i] - pred_dist;
                    Point numerator = -2 * (pos - sampled_positions[i]) * error;
                    grad += numerator / pred_dist;
                    mse.back() += pow(error, 2) / distances.size();
                }
                // cout << grad.x << " " << grad.y << " " << grad.z << endl;
                pos -= grad * step_len;
            }
            bool success = mse.back() < mse[0] / 5.;
            if (success){
                solutions.update(pos);
            }
            return success;
        }
        // 用x y两个坐标历史解的方差平均值判断解是否已经收敛
        bool converged(const double std_thres){
            // 解过少时，不判断为已收敛
            if (solutions.queue.size() != max_n_solutions)
                return false;
            Point last_std = solutions.std();
            double std = (last_std.x + last_std.y) / 2.;
            return (std < std_thres);
        }
};

// TODO: 目前SignalSource类的实现仅用于软件仿真！
// TODO: 在结合定位器进行实验后，需要对该类进行修改，
// TODO: 把operator()函数修改为调用定位器的库函数来计算距离。
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
