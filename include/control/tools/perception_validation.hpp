//
// Created by haavard on 17.02.18.
//

#ifndef CONTROL_FSM_PERCEPTION_VALIDATION_HPP
#define CONTROL_FSM_PERCEPTION_VALIDATION_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace control {
namespace validation {

struct Vec2 {
    double x = 0;
    double y = 0;
};

class PoseStampedCorrelation {
    static constexpr double NANO_PREFIX = std::pow(10, -9);
    static constexpr double SAMPLE_TIME = 0.1;
    static constexpr double SAMPLE_DT = 0.02;
    static constexpr size_t NUM_SAMPLES = static_cast<const size_t>(std::ceil(SAMPLE_TIME / SAMPLE_DT));
    static constexpr size_t NUM_DELAYS = 2;
    using SignalType = std::array<Vec2, NUM_SAMPLES>;
private:
    bool running_ = false;
    ros::NodeHandle n_;
    ros::Subscriber main_signal_sub_;
    ros::Subscriber delayed_signal_sub_;

    bool main_overflow_ = false;
    bool delayed_overflow_ = false;

    unsigned int last_main_index_ = 0;
    unsigned int last_delayed_index_ = 0;

    ros::Time start_time_;
    ros::Time last_calculated_;

    std::array<Vec2, NUM_SAMPLES> main_signal_;
    std::array<Vec2, NUM_SAMPLES> delayed_signal_;
    Vec2 best_correlation_;
    double delay_ = -1;

    void poseRecievedCB(geometry_msgs::PoseStamped::ConstPtr msg_p, SignalType* signal, bool* overflow, unsigned int* last_index);

    void calculateCrossCorrelation();

public:
    PoseStampedCorrelation(const std::string& main_topic, const std::string& delayed_topic);

    ///Start repeated calculations
    void start() { running_ = true; start_time_ = ros::Time::now(); };
    ///Stop repeated calculations
    void stop() { running_ = false; };

    ///Struct containing result, delay and timestamp
    struct Result {
        Vec2 best_corr = Vec2();
        double signal_delay{};
        ros::Time stamp = ros::Time();
    };

    ///Returns best calculated correlation
    Result getBestCorr() {
        Result r;
        r.best_corr = best_correlation_;
        r.signal_delay = delay_;
        r.stamp = last_calculated_;
        return r;
    }

};
}
}
#endif //CONTROL_FSM_PERCEPTION_VALIDATION_HPP
