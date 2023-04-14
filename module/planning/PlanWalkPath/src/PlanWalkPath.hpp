#ifndef MODULE_PLANNING_PLANWALKPATH_HPP
#define MODULE_PLANNING_PLANWALKPATH_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class PlanWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Maximum walk command velocity for walking to ball
            float max_forward_speed = 0;
            /// @brief Minimum walk command velocity for walking to ball
            float min_forward_speed = 0;
            /// @brief Crude acceleration, the maximum increment/decrease in walk command velocity per update
            float acceleration = 0;
            /// @brief Region around ball to begin decelerating in
            float approach_radius = 0;
            /// @brief Maximum angular velocity command for walking to ball
            float max_turn_speed = 0;
            /// @brief Minimum angular velocity command for walking to ball
            float min_turn_speed = 0;
            /// @brief Rotate on spot walk command angular velocity
            float rotate_speed = 0;
            /// @brief Rotate on spot walk command forward velocity
            float rotate_speed_x = 0;
            /// @brief Rotate on spot walk command side velocity
            float rotate_speed_y = 0;
            /// @brief Pivot ball command angular velocity
            float pivot_ball_speed = 0;
            /// @brief Pivot ball forward velocity
            float pivot_ball_speed_x = 0;
            /// @brief Pivot ball side velocity
            float pivot_ball_speed_y = 0;
        } cfg;

        /// @brief The current speed of the walk command
        float speed    = 0;
        bool is_stable = true;

    public:
        /// @brief Called by the powerplant to build and setup the PlanWalkPath reactor.
        explicit PlanWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANWALKPATH_HPP
