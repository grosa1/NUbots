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

#include "HeadBehaviourSoccer.hpp"

#include <utility>

#include "extension/Configuration.hpp"

#include "message/localisation/FilteredBall.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/HeadCommand.hpp"
#include "message/vision/Goal.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::behaviour::skills {

    using extension::Configuration;

    using message::motion::ExecuteGetup;
    using message::motion::HeadCommand;
    using message::motion::KillGetup;
    using FilteredBall = message::localisation::FilteredBall;
    using VisionGoals  = message::vision::Goal;


    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::support::Expression;

    /**
     * Calculates angles to a 3d vector in world
     *
     * @return Eigen::Vector2d of {yaw, pitch}
     */
    inline Eigen::Vector2d screenAngularFromObjectDirection(const Eigen::Vector3d& v) {
        return {std::atan2(v.y(), v.x()), std::atan2(v.z(), v.x())};
    }

    HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("HeadBehaviourSoccer.yaml")
            .then("Head Behaviour Soccer Config", [this](const Configuration& config) {
                log_level          = config["log_level"].as<NUClear::LogLevel>();
                cfg.search_timeout = duration_cast<NUClear::clock::duration>(
                    std::chrono::duration<double>(config["search_timeout"].as<double>()));
                cfg.fixation_time = config["fixation_time"].as<float>();
                cfg.pitch_offset  = config["pitch_offset"].as<float>();
                // Create vector of search positions
                for (const auto& position : config["positions"].config) {
                    cfg.search_positions.push_back(position.as<Expression>());
                }
                cfg.goal_search_distance_threshold = config["goal_search_distance_threshold"].as<float>();
                cfg.goal_search_timeout            = config["goal_search_timeout"].as<float>();
            });

        // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>().then([this] { is_getting_up = true; });

        // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>().then([this] { is_getting_up = false; });

        on<Every<90, Per<std::chrono::seconds>>,
           Optional<With<FilteredBall>>,
           Optional<With<VisionGoals>>,
           Sync<HeadBehaviourSoccer>>()
            .then("Head Behaviour Main Loop",
                  [this](const std::shared_ptr<const FilteredBall>& ball,
                         const std::shared_ptr<const VisionGoals>& goals) {
                      // Only look for ball or goal if not getting up
                      if (!is_getting_up) {
                          // If we see a ball
                          if (ball && NUClear::clock::now() - ball->time_of_measurement < cfg.search_timeout) {
                              NUClear::log<NUClear::DEBUG>("HB - Looking at ball");
                              // We can see the ball, lets look at it
                              Eigen::Vector3d rBCc                 = ball->rBCc.cast<double>();
                              Eigen::Vector2d angles               = screenAngularFromObjectDirection(rBCc);
                              std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                              command->yaw                         = angles[0];
                              command->pitch                       = angles[1] + cfg.pitch_offset;
                              command->robot_space                 = true;
                              command->smooth                      = true;
                              emit(std::move(command));
                          }
                          //   NUClear::log<NUClear::DEBUG>("HB - standing and sees ball");
                          // If we can't see the goals
                          //   else if (ball && !goals && ball->rBTt.norm() < cfg.goal_search_distance_threshold) {
                          //     NUClear::log<NUClear::DEBUG>("HB - Looking for a goal");
                          //       //   NUClear::log<NUClear::DEBUG>("HB - Has goals");
                          //       //   NUClear::log<NUClear::DEBUG>("HB - goal last measured: ", goal_last_measured);
                          //       //   NUClear::log<NUClear::DEBUG>("HB - Last look for goal: ", last_look_for_goal);
                          //       //   NUClear::log<NUClear::DEBUG>("HB - Search timeout: ", cfg.goal_search_timeout);
                          //       last_look_for_goal = std::chrono::duration_cast<std::chrono::duration<float>>(
                          //                                NUClear::clock::now() - goal_last_measured)
                          //                                .count();
                          //       // If we haven't looked up for a goal measurement for a certain time and
                          //       // if close to the ball but out of kick distance, look up and try for a goal
                          //       // measurement.
                          //       if (last_look_for_goal > cfg.goal_search_timeout) {
                          //           NUClear::log<NUClear::DEBUG>("HB - Looking for a goal");
                          //           goal_last_measured                   = NUClear::clock::now();
                          //           std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                          //           command->yaw                         = 0.7;
                          //           command->pitch                       = 0.4;
                          //           command->robot_space                 = true;
                          //           command->smooth                      = true;
                          //           emit(std::move(command));
                          //       }
                          //   }
                          // Ball hasn't been seen in a while. Look around using search positions
                          else {
                              NUClear::log<NUClear::DEBUG>("HB - Searching for ball");
                              float time_since_last_search_moved =
                                  std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now()
                                                                                           - search_last_moved)
                                      .count();

                              // Robot will move through the search positions, and linger for fixation_time. Once
                              // fixation_time time has passed, send a new head command for the next position in the
                              // list of cfg.search_positions
                              if (time_since_last_search_moved > cfg.fixation_time) {
                                  // Move to next search position in list
                                  search_last_moved                    = NUClear::clock::now();
                                  std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                                  command->yaw                         = cfg.search_positions[search_idx][0];
                                  command->pitch                       = cfg.search_positions[search_idx][1];
                                  command->robot_space                 = true;
                                  command->smooth                      = false;
                                  search_idx++;
                                  emit(std::move(command));
                              }

                              // Reset the search position index if at end of list
                              if (search_idx == cfg.search_positions.size()) {
                                  search_idx = 0;
                              }
                          }
                      }
                  });
    }

}  // namespace module::behaviour::skills
