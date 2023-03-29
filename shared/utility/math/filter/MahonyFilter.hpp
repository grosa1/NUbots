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
 * Copyright 2020 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_MAHONYFILTER_HPP
#define UTILITY_MATH_FILTER_MAHONYFILTER_HPP

#include <Eigen/Core>

namespace utility {
    namespace math {
        namespace filter {

            /**
             * @brief Compute skew of a vector
             * @param v: input vector
             * @return skew of v
             */
            template <typename T>
            Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& v) {
                Eigen::Matrix<T, 3, 3> skew;
                skew << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
                return skew;
            }

            /**
             * @brief The Mahony update uses the IMU data to compute the attitude. To see report on this, go to
             * "public/Projects/Final Year Projects/NUgus Pose Estimation" folder on the NAS.
             * @param acc:     accelerometer reading as a column vector, [a_x, a_y, a_z]'. g force
             * @param gyro:    gyroscope reading as  column vector, [w_x, w_y, w_z]'. angular velocities in rad/s.
             * @param ts:      Sample rate of this update.
             * @param Ki:      integral gain
             * @param Kp:      proportional gain
             * @param quat:    quaternion representing the rotation of the torso. Hwt.
             * @param bias:    gyroscope bias
             */
            void MahonyUpdate(Eigen::Vector3d acc,
                              Eigen::Vector3d gyro,
                              const double ts,
                              const double Ki,
                              const double Kp,
                              Eigen::Quaterniond& quat,
                              Eigen::Vector3d& bias) {
                // Normalise the acceleration vector
                acc.normalize();
                Eigen::Vector3d rGTt = acc;

                // Convert the quaternion to a rotation matrix
                Eigen::Matrix3d Rtw = quat.toRotationMatrix().inverse();

                // Calculate the error between the accelerometer gravity and state estimated gravity vector
                // World vector for gravity (gravity is positive)
                Eigen::Vector3d rGWw(0, 0, 1);
                // Calculate estimated accelerometer reading
                Eigen::Vector3d est_rGTt = Rtw * rGWw;
                // Calculate error between estimate and real
                Eigen::Matrix3d a_corr = rGTt * est_rGTt.transpose() - est_rGTt * rGTt.transpose();
                // Vex (inverse function of skew-symmetric function) the error
                Eigen::Vector3d omega_mes = -1 * Eigen::Vector3d(a_corr(2, 1), a_corr(0, 2), a_corr(1, 0));

                // Estimate the bias
                bias += Ki * omega_mes * ts;

                // Depolarizing the gyroscope bias
                Eigen::Vector3d l_omega = gyro + Kp * omega_mes + bias;

                // Find the quaternions rate of change
                Eigen::Matrix3d omega_x = skew(l_omega);
                Eigen::Matrix4d ome     = Eigen::Matrix4d::Zero();
                ome.block<3, 3>(0, 0)   = -omega_x;
                ome.block<1, 3>(3, 0)   = -l_omega.transpose();
                ome.block<3, 1>(0, 3)   = l_omega;
                ome(3, 3)               = 0;

                Eigen::Vector4d quat_vec(quat.x(), quat.y(), quat.z(), quat.w());
                Eigen::Vector4d q_d(0.5 * ome * quat_vec);

                // Calculate integral to find the attitude quaternion
                quat_vec += ts * q_d;

                // Set quat (Rwt) and normalise
                quat = Eigen::Quaterniond(quat_vec(3), quat_vec(0), quat_vec(1), quat_vec(2));
                quat.normalize();
            }

        }  // namespace filter
    }      // namespace math
}  // namespace utility

#endif
