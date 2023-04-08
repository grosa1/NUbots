#ifndef MODULE_STRATEGY_ALIGNBALLTOGOAL_HPP
#define MODULE_STRATEGY_ALIGNBALLTOGOAL_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

class AlignBallToGoal : public ::extension::behaviour::BehaviourReactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the AlignBallToGoal reactor.
    explicit AlignBallToGoal(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_ALIGNBALLTOGOAL_HPP
