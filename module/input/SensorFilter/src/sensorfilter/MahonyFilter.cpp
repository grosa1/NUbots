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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#include "utility/math/filter/MahonyFilter.hpp"

#include "SensorFilter.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;

    using extension::Configuration;

    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::math::filter::MahonyUpdate;
    using utility::support::Expression;

    void SensorFilter::integrate_walkcommand(const double dt) {
        // Check if we are not currently falling and walking
        if (!falling && walk_engine_enabled) {
            // Integrate the walk command to estimate the change in position and yaw orientation
            double dx = walk_command.x() * dt * cfg.deadreckoning_scale.x();
            double dy = walk_command.y() * dt * cfg.deadreckoning_scale.y();
            yaw += walk_command.z() * dt * cfg.deadreckoning_scale.z();
            // Rotate the change in position into world coordinates before adding it to the current position
            Hwt.translation().x() += dx * cos(yaw) - dy * sin(yaw);
            Hwt.translation().y() += dy * cos(yaw) + dx * sin(yaw);
        }
    }

    void SensorFilter::configure_mahony(const Configuration& config) {
        // Mahony Filter Config
        cfg.Ki            = config["mahony"]["Ki"].as<Expression>();
        cfg.Kp            = config["mahony"]["Kp"].as<Expression>();
        cfg.bias          = Eigen::Vector3d(config["mahony"]["bias"].as<Expression>());
        Hwt.translation() = Eigen::VectorXd(config["mahony"]["initial_rTWw"].as<Expression>());
    }

    void SensorFilter::update_odometry_mahony(std::unique_ptr<Sensors>& sensors,
                                              const std::shared_ptr<const Sensors>& previous_sensors,
                                              const RawSensors& raw_sensors) {
        // **************** Time Update ****************
        // Calculate our time offset from the last read then update the filter's time
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Integrate the walk command to estimate the change in position (x,y) and yaw orientation
        integrate_walkcommand(dt);

        // **************** Roll/Pitch Orientation Measurement Update ****************
        utility::math::filter::MahonyUpdate(sensors->accelerometer,
                                            sensors->gyroscope,
                                            Hwt,
                                            dt,
                                            cfg.Ki,
                                            cfg.Kp,
                                            cfg.bias);
        // Extract the roll and pitch from the orientation quaternion
        Eigen::Vector3d rpy = MatrixToEulerIntrinsic(Hwt.rotation());

        // Compute the height of the torso using the kinematics from a foot which is on the ground
        if (sensors->feet[BodySide::LEFT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::L_ANKLE_ROLL]).inverse().translation().z();
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::R_ANKLE_ROLL].inverse()).translation().z();
        }

        // **************** Construct Odometry Output (Htw) ****************
        // Use the roll and pitch from the Mahony filter and the yaw from the dead reckoning of walk command
        const double roll  = rpy(0);
        const double pitch = rpy(1);
        Hwt.linear()       = EulerIntrinsicToMatrix(Eigen::Vector3d(roll, pitch, yaw));
        sensors->Htw       = Hwt.inverse().matrix();
        update_loop.enable();
    }
}  // namespace module::input
