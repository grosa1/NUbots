#ifndef MODULE_PLANNING_PLANWALKPATH_HPP
#define MODULE_PLANNING_PLANWALKPATH_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class PlanWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The maximum speed we can walk forwards
            float speed = 0.0;
            /// @brief The maximum speed we can turn
            float max_turn_speed = 0.0;
            /// @brief The minimum speed we can turn
            float min_turn_speed = 0.0;
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

    public:
        /// @brief Called by the powerplant to build and setup the PlanWalkPath reactor.
        explicit PlanWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANWALKPATH_HPP