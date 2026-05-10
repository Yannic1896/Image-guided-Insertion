//
// Created by Gerlach on 7/14/21.
//

#ifndef SRC_TRAJECTORY_H
#define SRC_TRAJECTORY_H
#include <queue>
#include <cstdint>

namespace franka_example_controllers {
class Trajectory {

public:
    explicit Trajectory(bool should_send_completed=false) noexcept :
            id_(trajectories),
            should_send_completed_(should_send_completed) {
        if(should_send_completed) ++trajectories;
    }

    explicit Trajectory(std::queue<double> trajectory, bool should_send_completed=false) noexcept :
        trajectory_(std::move(trajectory)),
        id_(trajectories++),
        should_send_completed_(should_send_completed) {
        if(should_send_completed) ++trajectories;
    }

    [[nodiscard]] std::queue<double>& get() {return trajectory_;}
    [[nodiscard]] uint64_t getId() const { return id_;}
    void setId(uint64_t id) {id_ = id;}
    [[nodiscard]] bool getShouldSendCompleted() const {return should_send_completed_;}
    void setShouldSendCompleted(bool should_set_completed) {should_send_completed_=should_set_completed;}

private:
    std::queue<double> trajectory_;
    uint64_t id_;
    bool should_send_completed_;

    static uint64_t trajectories;

};

}

#endif //SRC_TRAJECTORY_H

