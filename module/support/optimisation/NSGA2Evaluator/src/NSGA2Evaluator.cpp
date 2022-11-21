#include "NSGA2Evaluator.hpp"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "tasks/RotationEvaluator.hpp"
#include "tasks/StandEvaluator.hpp"
#include "tasks/StrafeEvaluator.hpp"
#include "tasks/WalkEvaluator.hpp"

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/WebotsResetDone.hpp"
#include "message/platform/webots/WebotsTimeUpdate.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {
    namespace support {
        namespace optimisation {

            using extension::Configuration;
            using extension::ExecuteScriptByName;

            using message::behaviour::MotionCommand;
            using message::motion::DisableWalkEngineCommand;
            using message::motion::EnableWalkEngineCommand;
            using message::motion::StopCommand;
            using message::motion::WalkCommand;
            using RawSensorsMsg = message::platform::RawSensors;
            using message::platform::webots::OptimisationCommand;
            using message::platform::webots::OptimisationRobotPosition;
            using message::platform::webots::WebotsResetDone;
            using message::platform::webots::WebotsTimeUpdate;
            using message::support::optimisation::NSGA2EvaluationRequest;
            using message::support::optimisation::NSGA2EvaluatorReadinessQuery;
            using message::support::optimisation::NSGA2EvaluatorReady;
            using message::support::optimisation::NSGA2FitnessScores;
            using message::support::optimisation::NSGA2Terminate;
            using message::support::optimisation::NSGA2TrialExpired;

            using utility::behaviour::RegisterAction;
            using utility::input::LimbID;
            using utility::input::ServoID;
            using utility::support::Expression;

            NSGA2Evaluator::NSGA2Evaluator(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {
                    log<NUClear::INFO>("Setting up the NSGA2 evaluator");

                emit<Scope::DIRECT>(std::make_unique<RegisterAction>(RegisterAction{
                    subsumptionId,
                    "NSGA2 Evaluator",
                    {std::pair<float, std::set<LimbID>>(
                        1,
                        {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                    [this](const std::set<LimbID>& given_limbs) {
                        if (given_limbs.find(LimbID::LEFT_LEG) != given_limbs.end()) {
                            // Enable the walk engine.},
                            emit<Scope::DIRECT>(std::make_unique<EnableWalkEngineCommand>(subsumptionId));
                        }
                    },
                    [this](const std::set<LimbID>& taken_limbs) {
                        if (taken_limbs.find(LimbID::LEFT_LEG) != taken_limbs.end()) {
                            // Shut down the walk engine, since we don't need it right now.
                            emit<Scope::DIRECT>(std::make_unique<DisableWalkEngineCommand>(subsumptionId));
                        }
                    },
                    [this](const std::set<ServoID>&) {}}));

                // Handle a state transition event
                on<Trigger<Event>, Sync<NSGA2Evaluator>>().then([this](const Event& event) {
                    State old_state = current_state;
                    State new_state = HandleTransition(current_state, event);

                    log<NUClear::DEBUG>("Transitioning on", event, ", from state", old_state, "to state", new_state);

                    switch (new_state) {
                        case State::WAITING_FOR_REQUEST:
                            current_state = new_state;
                            WaitingForRequest();
                            break;
                        case State::SETTING_UP_TRIAL:
                            current_state = new_state;
                            SettingUpTrial();
                            break;
                        case State::RESETTING_SIMULATION:
                            current_state = new_state;
                            ResettingSimulation();
                            break;
                        case State::EVALUATING:
                            current_state = new_state;
                            Evaluating(event);
                            break;
                        case State::TERMINATING_EARLY:
                            current_state = new_state;
                            TerminatingEarly();
                            break;
                        case State::TERMINATING_GRACEFULLY:
                            current_state = new_state;
                            TerminatingGracefully();
                            break;
                        case State::FINISHED:
                            current_state = new_state;
                            Finished();
                            break;
                        default:
                            log<NUClear::WARN>("Unable to transition to unknown state from", current_state, "on", event);
                    }
                });

                on<Trigger<NSGA2EvaluationRequest>, Single>().then([this](const NSGA2EvaluationRequest& request) {
                    lastEvalRequestMsg = request;
                    emit(std::make_unique<Event>(Event::EvaluateRequest));
                });

                on<Trigger<NSGA2TrialExpired>, Single>().then([this](const NSGA2TrialExpired& message) {
                    // Only start terminating gracefully if the trial that just expired is the current one
                    // (and not previous ones that have terminated early)
                    if (message.generation == generation && message.individual == individual) {
                        emit(std::make_unique<Event>(Event::TrialCompleted));
                    }
                });

                on<Trigger<RawSensorsMsg>, Single>().then([this](const RawSensorsMsg& sensors) {
                    if (current_state == State::EVALUATING) {
                        task->processRawSensorMsg(sensors, this);
                    }
                });

                on<Trigger<WebotsResetDone>, Single>().then([this](const WebotsResetDone&) {
                    log<NUClear::INFO>("Reset done");
                    emit(std::make_unique<Event>(Event::ResetDone));
                });

                on<Trigger<WebotsTimeUpdate>, Single>().then(
                    [this](const WebotsTimeUpdate& update) { sim_time = update.sim_time; });

                on<Trigger<NSGA2Terminate>, Single>().then([this]() {
                    // NSGA2Terminate is emitted when we've finished all generations and all individuals
                    emit(std::make_unique<Event>(Event::TerminateEvaluation));
                });

                on<Trigger<NSGA2EvaluatorReadinessQuery>, Single>().then([this]() {
                    // NSGA2EvaluatorReadinessQuery is the optimiser checking if we're ready
                    emit(std::make_unique<Event>(Event::CheckReady));
                });

                on<Trigger<OptimisationRobotPosition>, Single>().then(
                    [this](const OptimisationRobotPosition& position) {
                        if (current_state == State::EVALUATING) {
                            task->processOptimisationRobotPosition(position);
                        }
                });
            }

            NSGA2Evaluator::State NSGA2Evaluator::HandleTransition(NSGA2Evaluator::State current_state,
                                                                   NSGA2Evaluator::Event event) {
                switch (current_state) {
                    case State::WAITING_FOR_REQUEST:
                        return TransitionEvents(event);

                    case State::SETTING_UP_TRIAL:
                        return TransitionEvents(event);

                    case State::RESETTING_SIMULATION:
                        return TransitionEvents(event);

                    case State::EVALUATING:
                        return TransitionEvents(event);

                    case State::TERMINATING_EARLY:
                        return TransitionEvents(event);

                    case State::TERMINATING_GRACEFULLY:
                        return TransitionEvents(event);

                    case State::FINISHED:
                        // Arguably this should return FINISHED regardless of event, unless we want to be able to
                        // reset
                        if (event == Event::FitnessScoresSent) {
                            return State::FINISHED;
                        }
                        else {
                            return TransitionEvents(event);
                        }

                    default: return State::UNKNOWN;
                }
            }

            NSGA2Evaluator::State NSGA2Evaluator::TransitionEvents(NSGA2Evaluator::Event event) {
                switch (event) {
                    case Event::EvaluateRequest     : return State::SETTING_UP_TRIAL;
                    case Event::CheckReady          : return State::WAITING_FOR_REQUEST;
                    case Event::TerminateEvaluation : return State::FINISHED;
                    case Event::TrialSetupDone      : return State::RESETTING_SIMULATION;
                    case Event::ResetDone           : return State::EVALUATING;
                    case Event::TerminateEarly      : return State::TERMINATING_EARLY;
                    case Event::TrialCompleted      : return State::TERMINATING_GRACEFULLY;
                    case Event::FitnessScoresSent   : return State::WAITING_FOR_REQUEST;

                    default: return State::UNKNOWN;
                }
            }

            /// @brief Handle the WAITING_FOR_REQUEST state
            void NSGA2Evaluator::WaitingForRequest() {
                log<NUClear::DEBUG>("WaitingForRequest");
                emit(std::make_unique<NSGA2EvaluatorReady>());  // Let the optimiser know we're ready
            }

            /// @brief Handle the SETTING_UP_TRIAL state
            void NSGA2Evaluator::SettingUpTrial() {
                log<NUClear::DEBUG>("SettingUpTrial");

                generation = lastEvalRequestMsg.generation;
                individual = lastEvalRequestMsg.id;

                if (lastEvalRequestMsg.task == "walk") {
                    task = std::make_unique<WalkEvaluator>();
                }
                else if (lastEvalRequestMsg.task == "strafe") {
                    task = std::make_unique<StrafeEvaluator>();
                }
                else if (lastEvalRequestMsg.task == "rotation") {
                    task = std::make_unique<RotationEvaluator>();
                }
                else if (lastEvalRequestMsg.task == "stand") {
                    task = std::make_unique<StandEvaluator>();
                }
                else {
                    log<NUClear::ERROR>("Unhandled task type:", lastEvalRequestMsg.task);
                }

                task->setUpTrial(lastEvalRequestMsg);

                emit(std::make_unique<Event>(Event::TrialSetupDone));
            }

            /// @brief Handle the RESETTING_SIMULATION state
            void NSGA2Evaluator::ResettingSimulation() {
                log<NUClear::DEBUG>("ResettingSimulation");

                task->resetSimulation();

                // Tell Webots to reset the world
                std::unique_ptr<OptimisationCommand> reset = std::make_unique<OptimisationCommand>();
                reset->command                             = OptimisationCommand::CommandType::RESET_WORLD;
                emit(reset);
            }

            /// @brief Handle the EVALUATING state
            void NSGA2Evaluator::Evaluating(NSGA2Evaluator::Event event) {
                log<NUClear::DEBUG>("Evaluating");

                if (event == Event::ResetDone) {
                    if (lastEvalRequestMsg.task == "walk"    ||
                        lastEvalRequestMsg.task == "stand"   ||
                        lastEvalRequestMsg.task == "strafe"  ||
                        lastEvalRequestMsg.task == "rotation") {
                        task->evaluatingState(subsumptionId, this);
                    }
                    else {
                        log<NUClear::ERROR>("Unhandled task type:", lastEvalRequestMsg.task);
                    }
                }
            }

            void NSGA2Evaluator::ScheduleTrialExpiredMessage(const int trial_stage,
                                                             const std::chrono::seconds delay_time) {
                // Prepare the trial expired message
                std::unique_ptr<NSGA2TrialExpired> message = std::make_unique<NSGA2TrialExpired>();
                message->time_started                      = sim_time;
                message->generation                        = generation;
                message->individual                        = individual;
                message->trial_stage                       = trial_stage;

                // Schedule the end of the walk trial after the duration limit
                log<NUClear::DEBUG>("Scheduling expired message with time ", delay_time.count());
                emit<Scope::DELAY>(message, delay_time);
            }

            /// @brief Handle the TERMINATING_EARLY state
            void NSGA2Evaluator::TerminatingEarly() {
                log<NUClear::DEBUG>("TerminatingEarly");

                // Send a zero walk command to stop walking
                emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(0.0, 0.0, 0.0)));
                bool early_termination = true;
                auto fitness_scores    = task->calculateFitnessScores(early_termination, sim_time, generation, individual);
                emit(fitness_scores);

                emit(std::make_unique<Event>(Event::FitnessScoresSent));  // Go back to waiting for the next request
            }

            /// @brief Handle the TERMINATING_GRACEFULLY state
            void NSGA2Evaluator::TerminatingGracefully() {
                log<NUClear::DEBUG>("TerminatingGracefully");

                // Send a zero walk command to stop walking
                emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(0.0, 0.0, 0.0)));
                bool early_termination = false;
                auto fitness_scores    = task->calculateFitnessScores(early_termination, sim_time, generation, individual);
                emit(fitness_scores);

                emit(std::make_unique<Event>(Event::FitnessScoresSent));  // Go back to waiting for the next request
            }

            void NSGA2Evaluator::Finished() {
                log<NUClear::INFO>("Finished");
            }
        }  // namespace optimisation
    }      // namespace support
}  // namespace module
