//
// Created by haavard on 17.02.18.
//

#include "control/tools/perception_validation.hpp"

using geometry_msgs::PoseStamped;
using namespace control::validation;

PoseStampedCorrelation::PoseStampedCorrelation(const std::string& main_topic, const std::string& delayed_topic) {
    auto cb = &PoseStampedCorrelation::poseRecievedCB;
    auto main_cb = boost::bind(cb, this, _1, &main_signal_, &main_overflow_, &last_main_index_);
    auto delayed_cb = boost::bind(cb, this, _1, &delayed_signal_, &delayed_overflow_, &last_delayed_index_);
    main_signal_sub_ = n_.subscribe<PoseStamped>(main_topic, 10, main_cb);
    delayed_signal_sub_ = n_.subscribe<PoseStamped>(delayed_topic, 10, delayed_cb);
}

void PoseStampedCorrelation::poseRecievedCB(PoseStamped::ConstPtr msg_p, SignalType* signal, bool* overflow, unsigned int* last_index) {
    if(!running_) return;

    auto& msg = *msg_p;

    auto elapsed_since_start = msg.header.stamp - start_time_;
    if(elapsed_since_start > ros::Duration(SAMPLE_TIME)) {
        *overflow = true;
    }

    if(main_overflow_ && delayed_overflow_) {
        calculateCrossCorrelation();
        start_time_ = msg.header.stamp;
        main_overflow_ = delayed_overflow_ = false;
    }
    elapsed_since_start = msg.header.stamp - start_time_;
    constexpr double SCALE = SAMPLE_DT / NANO_PREFIX;
    double scaled_time = static_cast<double>(elapsed_since_start.toNSec()) / SCALE;
    auto index = static_cast<unsigned int>(std::round(scaled_time));
    *overflow = (index >= signal->size());

    int diff = index - *last_index;
    if(!(*overflow)) {
        signal->at(index).x = msg.pose.position.x;
        signal->at(index).y = msg.pose.position.y;
        if(diff > 1) {
            double dx = (signal->at(index).x - signal->at(*last_index).x) / static_cast<double>(diff);
            double dy = (signal->at(index).y - signal->at(*last_index).y) / static_cast<double>(diff);
            for(unsigned int i = *last_index + 1; i < index; ++i) {
                signal->at(i).x = dx * i + signal->at(*last_index).x;
                signal->at(i).y = dy * i + signal->at(*last_index).y;
            }
        }
        *last_index = index;
    } else if(*overflow) {
        //Clear last part of signal
        for(unsigned int i = *last_index + 1; i < signal->size(); ++i) {
            signal->at(i).x = 0.0;
            signal->at(i).y = 0.0;
        }
        *last_index = static_cast<unsigned int>(NUM_SAMPLES - 1);
    }

}

double getErrorSquared(Vec2 corr) {
    return std::pow(1 - corr.x, 2) + std::pow(1 - corr.y, 2);
}

void PoseStampedCorrelation::calculateCrossCorrelation() {
    if(!(main_overflow_ && delayed_overflow_)) {
        ROS_ERROR_NAMED("Perception Validation", "Signals not valid!");
        return;
    }

    for(unsigned int n = 0; n < NUM_DELAYS; ++n) {
        double sum_squared_x_f = 0;
        double sum_squared_y_f = 0;
        double sum_squared_x_p = 0;
        double sum_squared_y_p = 0;
        double correlation_x = 0;
        double correlation_y = 0;
        for(unsigned int i = 0; i < NUM_SAMPLES - n; ++i) {
            auto& filter = delayed_signal_[i + n];
            auto& perception = main_signal_[i];
            correlation_x += filter.x * perception.x;
            correlation_y += filter.y * perception.y;
            sum_squared_x_f += filter.x * filter.x; //Squared
            sum_squared_y_f += filter.y * filter.y; //Squared
            sum_squared_x_p += perception.x * perception.x; //Squared
            sum_squared_y_p += perception.y * perception.y;
        }
        Vec2 corr;
        corr.x = correlation_x / std::sqrt(sum_squared_x_f * sum_squared_x_p);
        corr.y = correlation_y / std::sqrt(sum_squared_y_f * sum_squared_y_p);
        if(getErrorSquared(corr) < getErrorSquared(best_correlation_) || n == 0) {
            best_correlation_ = corr;
            delay_ = n * SAMPLE_DT;
        }
    }
    last_calculated_ = ros::Time::now();
    std::cout << "Best correlation, X: " << best_correlation_.x << " Y: " << best_correlation_.y << " at delay " << delay_ << std::endl;


}
