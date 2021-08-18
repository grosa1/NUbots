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

#include "NLoptInverseKinematics.hpp"


namespace utility::motion::nloptinversekinematics {

    using message::input::Sensors;
    using message::motion::KinematicsModel;

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    typedef struct {
        Eigen::Affine3d target;
        LimbID limb;
    } Data;

    Eigen::Affine3d forwardKinematics(const std::vector<double>& q) {
        Eigen::Affine3d runningTransform = Eigen::Affine3d::Identity();
        double hip_yaw                   = 0.055;
        double hip_roll                  = 0.06;
        double hip_pitch                 = 0;
        double upper_leg                 = 0.2;
        double lower_leg                 = 0.2;
        double ankle                     = 0;
        int negativeIfRight              = -1;
        // Hip yaw
        runningTransform = runningTransform.translate(Eigen::Vector3d(0.0, negativeIfRight * hip_yaw, 0.0));
        // Hip roll
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ()));
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
        runningTransform = runningTransform.translate(Eigen::Vector3d(hip_roll, 0.0, 0.0));
        // Hip pitch
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitZ()));
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()));
        runningTransform = runningTransform.translate(Eigen::Vector3d(hip_pitch, 0.0, 0.0));
        // Upper leg
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitZ()));
        runningTransform = runningTransform.translate(Eigen::Vector3d(upper_leg, 0.0, 0.0));
        // Lower leg
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitZ()));
        runningTransform = runningTransform.translate(Eigen::Vector3d(lower_leg, 0.0, 0.0));
        // Ankle
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitZ()));
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
        runningTransform = runningTransform.translate(Eigen::Vector3d(ankle, 0.0, 0.0));
        // Foot
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitZ()));
        return runningTransform;
    }

    double cost(const std::vector<double>& q, std::vector<double>& grad, void* data) {
        Data* d            = reinterpret_cast<Data*>(data);
        Eigen::Vector3d xe = forwardKinematics(q).translation();
        Eigen::Vector3d xd;  //(0, -0.055, -0.4);
        xd << d->target.translation();
        Eigen::Vector3d cost = xe - xd;
        return cost.norm();
    }

    std::vector<std::pair<ServoID, float>> inverseKinematics(const message::motion::KinematicsModel& model,
                                                             const Eigen::Affine3d& target,
                                                             const LimbID& limb) {
        Data *data, d;
        d.target = target;
        d.limb   = limb;
        data     = &d;

        nlopt::opt opt(nlopt::LN_COBYLA, 6);
        opt.set_min_objective(cost, data);
        opt.set_xtol_rel(1e-2);
        std::vector<double> x(6, 0.0);
        double minf;
        // Run the optimizer
        std::cout << opt.optimize(x, minf) << std::endl;
        float hipYaw     = (float) x[0];
        float hipRoll    = (float) x[1];
        float hipPitch   = (float) x[2];
        float knee       = (float) x[3];
        float anklePitch = (float) x[4];
        float ankleRoll  = (float) x[5];

        std::cout << "NLopt Hip yaw" << hipYaw << std::endl;
        std::cout << "NLopt Hip roll" << hipRoll << std::endl;
        std::cout << "NLopt Hip pitch" << hipPitch << std::endl;
        std::cout << "NLopt Knee pitch" << knee << std::endl;
        std::cout << "NLopt Ankle pitch" << anklePitch << std::endl;
        std::cout << "NLopt Ankle Roll" << ankleRoll << std::endl;

        std::vector<std::pair<ServoID, float>> positions;
        if (limb == LimbID::LEFT_LEG) {
            positions.push_back(std::make_pair(ServoID::L_HIP_YAW, -hipYaw));
            positions.push_back(std::make_pair(ServoID::L_HIP_ROLL, hipRoll));
            positions.push_back(std::make_pair(ServoID::L_HIP_PITCH, -hipPitch));
            positions.push_back(std::make_pair(ServoID::L_KNEE, M_PI - knee));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_PITCH, -anklePitch));
            positions.push_back(std::make_pair(ServoID::L_ANKLE_ROLL, ankleRoll));
        }
        else {
            positions.push_back(std::make_pair(ServoID::R_HIP_YAW, (model.leg.LEFT_TO_RIGHT_HIP_YAW) * -hipYaw));
            positions.push_back(std::make_pair(ServoID::R_HIP_ROLL, (model.leg.LEFT_TO_RIGHT_HIP_ROLL) * hipRoll));
            positions.push_back(std::make_pair(ServoID::R_HIP_PITCH, (model.leg.LEFT_TO_RIGHT_HIP_PITCH) * -hipPitch));
            positions.push_back(std::make_pair(ServoID::R_KNEE, (model.leg.LEFT_TO_RIGHT_KNEE) * (M_PI - knee)));
            positions.push_back(
                std::make_pair(ServoID::R_ANKLE_PITCH, (model.leg.LEFT_TO_RIGHT_ANKLE_PITCH) * -anklePitch));
            positions.push_back(
                std::make_pair(ServoID::R_ANKLE_ROLL, (model.leg.LEFT_TO_RIGHT_ANKLE_ROLL) * ankleRoll));
        }

        // delete data;
        return positions;
    }

    std::vector<std::pair<ServoID, float>> inverseKinematics(const message::motion::KinematicsModel& model,
                                                             const Eigen::Affine3d& leftTarget,
                                                             const Eigen::Affine3d& rightTarget) {
        auto joints  = inverseKinematics(model, leftTarget, LimbID::LEFT_LEG);
        auto joints2 = inverseKinematics(model, rightTarget, LimbID::RIGHT_LEG);
        joints.insert(joints.end(), joints2.begin(), joints2.end());
        return joints;
    }


}  // namespace utility::motion::nloptinversekinematics
