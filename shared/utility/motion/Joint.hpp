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

#ifndef UTILITY_MOTION_JOINT_HPP
#define UTILITY_MOTION_JOINT_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utility::motion {

    /// @brief The types of the joints
    enum class JointType { FIXED, REVOLUTE, PRISMATIC };

    /**
     * @brief Defines how a link moves relative to an attachment point
     * @details
     * @param Scalar The scalar type of the joint
     */
    template <typename Scalar>
    class Joint {

    private:
        /// @brief The unique ID of the joint
        const int JOINT_ID;

        /// @brief The name of the joint
        const std::string JOINT_NAME;

        /// @brief The type of the joint
        const JointType JOINT_TYPE;

        /// @brief The axis of motion for the joint
        const Eigen::Matrix<Scalar, 3, 1> JOINT_AXIS;

        /// @brief The transform to the parent link
        const Eigen::Transform<Scalar, 3, Eigen::Affine> JOINT_TRANSFORM_TO_PARENT;

        /// @brief The transform to the child link
        const Eigen::Transform<Scalar, 3, Eigen::Affine> JOINT_TRANSFORM_TO_CHILD;


    public:
        /**
         * @brief Construct a new Joint object
         * @details
         * @param id The unique ID of the joint
         * @param name The name of the joint
         * @param joint_type The type of the joint
         * @param axis The axis of motion for the joint
         * @param transform_to_parent The transform to the parent link
         * @param transform_to_child The transform to the child link
         */
        Joint(const int id,
              const std::string& name,
              const JointType joint_type,
              const Eigen::Matrix<Scalar, 3, 1>& axis,
              const Eigen::Transform<Scalar, 3, Eigen::Affine>& transform_to_parent,
              const Eigen::Transform<Scalar, 3, Eigen::Affine>& transform_to_child) {
            JOINT_ID                  = id;
            JOINT_NAME                = name;
            JOINT_TYPE                = joint_type;
            JOINT_AXIS                = axis;
            JOINT_TRANSFORM_TO_PARENT = transform_to_parent;
            JOINT_TRANSFORM_TO_CHILD  = transform_to_child;
        }
    };
}  // namespace utility::motion

#endif
