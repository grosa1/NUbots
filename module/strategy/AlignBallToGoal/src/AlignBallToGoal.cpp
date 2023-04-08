#include "AlignBallToGoal.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::localisation::Field;
    using message::localisation::FilteredBall;
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
                // Do stuff
                // field has the robots location
                // make a vector from the robot to the goal and compare it to
                // the robots forward vector
                // NOTE: How to use the field message for the robots position?
                // NOTE: How to use the field description
            });
    }

}  // namespace module::strategy
