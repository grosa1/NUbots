#include "AlignBallToGoal.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::localisation::Field;
    using message::localisation::FilteredBall;
    using message::planning::TurnAroundBall;
    using AlignBallToGoalTask = message::strategy::AlignBallToGoal;

    AlignBallToGoal::AlignBallToGoal(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("AlignBallToGoal.yaml").then([this](const Configuration& config) {
            // Use configuration here from file AlignBallToGoal.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        //
        on<Provide<AlignBallToGoalTask>, With<FilteredBall>, With<Field>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const FilteredBall& ball, const Field& field) {
                // Get the robot's position (pose) on the field
                Eigen::Vector3d robot_pos = field.Hfw.block<3, 1>(0, 0);
                // Compute the angle from the robot's forward direction to the x axis (opponent goal)
                NUClear::log("robot_pos x:", robot_pos.x());
                NUClear::log("robot_pos y:", robot_pos.y());
                // NOTE: field x axis is towards the opponent's goal
                // Do stuff
                // field has the robots location
                // make a vector from the robot to the goal and compare it to
                // the robots forward vector
                emit<Task>(std::make_unique<TurnAroundBall>(true));
            });
    }

}  // namespace module::strategy
