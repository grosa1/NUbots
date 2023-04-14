#include "KickToGoal.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/KickTo.hpp"
#include "message/strategy/KickToGoal.hpp"

namespace module::strategy {

    using extension::Configuration;
    using KickToGoalTask = message::strategy::KickToGoal;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::planning::KickTo;

    KickToGoal::KickToGoal(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("KickToGoal.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KickToGoal.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<KickToGoalTask>, With<Field>, With<Sensors>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const Field& field, const Sensors& sensors) {
                // Get the robot's position (pose) on the field
                Eigen::Isometry3d Htf = Eigen::Isometry3d(sensors.Htw) * Eigen::Isometry3d(field.Hfw.inverse());
                // Goal position relative to field
                Eigen::Vector3d rGFf(-4.5, 0.0, 0.0);
                // convert to torso space
                Eigen::Vector3d rGTt = Htf * rGFf;
                emit<Task>(std::make_unique<KickTo>(rGTt));  // kick the ball if possible
            });
    }

}  // namespace module::strategy
