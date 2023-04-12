#include "AlignBallToGoal.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/AlignBallToGoal.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
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

        on<Provide<AlignBallToGoalTask>,
           With<FilteredBall>,
           With<Field>,
           With<Sensors>,
           Every<30, Per<std::chrono::seconds>>>()
            .then([this](const FilteredBall& ball, const Field& field, const Sensors& sensors) {
                // if the ball is close
                float distance_to_ball = ball.rBTt.head(2).norm();
                NUClear::log<NUClear::DEBUG>("distance to ball: ", distance_to_ball);
                if (distance_to_ball < 1.0f) {
                    // Get the robot's position (pose) on the field
                    Eigen::Isometry3d Htf = Eigen::Isometry3d(sensors.Htw) * Eigen::Isometry3d(field.Hfw.inverse());
                    // Goal position relative to field
                    Eigen::Vector3d rGFf(4.5, 0.0, 0.0);
                    // convert to torso space
                    Eigen::Vector3d rGTt = Htf * rGFf;
                    // angle to the goal
                    float kick_angle = std::fabs(std::atan2(rGTt.y(), rGTt.x()));
                    NUClear::log<NUClear::DEBUG>("kick angle to the goal: ", kick_angle);
                    if (kick_angle > 0) {
                        NUClear::log<NUClear::DEBUG>("Rotate clockwise");
                        emit<Task>(std::make_unique<TurnAroundBall>(true));
                    }
                    else {
                        NUClear::log<NUClear::DEBUG>("Rotate clockwise");
                        emit<Task>(std::make_unique<TurnAroundBall>(false));
                    }
                    // NOTE: Will probs need to ensure alignment in the KickTo provider
                    // Compute the angle from the robot's forward direction to the x axis (opponent goal)
                    // NOTE: field x axis is towards the opponent's goal
                    // Do stuff
                }
            });
    }

}  // namespace module::strategy
