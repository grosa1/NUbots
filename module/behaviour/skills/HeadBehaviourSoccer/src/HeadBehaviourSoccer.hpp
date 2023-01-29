/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_HPP
#define MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::behaviour::skills {

    /**
     * Executes a HeadBehaviourSoccer action.
     *
     * @author Thomas O'Brien
     */
    class HeadBehaviourSoccer : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief Walk path planner priority in the subsumption system
            // Time before starting to search for ball after its lost
            NUClear::clock::duration search_timeout{};
            /// @brief Time lingering at each position in lost ballsearch
            float fixation_time = 0.0f;
            float pitch_offset  = 0.0f;
            /// @brief List of positions for search
            std::vector<Eigen::Vector2d> search_positions;

            /// @brief Distance away from the ball where we should look up for a goal measurement
            float goal_search_distance_threshold = 0.0f;
            /// @brief Time between glancing up to get a goal measurement
            float goal_search_timeout = 0.0f;
        } cfg;

        /// @brief Index in the list of search positions
        long unsigned int search_idx = 0;

        /// @brief Flag for if the robot is currently getting up
        bool is_getting_up = false;

        /// @brief  Time since last search position transition
        NUClear::clock::time_point search_last_moved = NUClear::clock::now();

        /// @brief Time since last ball seen
        NUClear::clock::time_point ball_last_measured = NUClear::clock::now();

        /// @brief Time since last glance at the goals, used when lining up the ball
        NUClear::clock::time_point goal_last_measured = NUClear::clock::now();

        /// @brief brief description
        float last_look_for_goal = 0;

        /// @brief distance to the ball
        float distance_to_ball = 0;


    public:
        explicit HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::behaviour::skills

#endif  // MODULES_BEHAVIOURS_REFLEX_HEADBEHAVIOURSOCCER_HPP
