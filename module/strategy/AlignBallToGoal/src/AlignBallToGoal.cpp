#include "AlignBallToGoal.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module::strategy {

using extension::Configuration;

AlignBallToGoal::AlignBallToGoal(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

    on<Configuration>("AlignBallToGoal.yaml").then([this](const Configuration& config) {
        // Use configuration here from file AlignBallToGoal.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::strategy
