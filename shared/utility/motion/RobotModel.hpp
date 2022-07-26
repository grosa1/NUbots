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

#ifndef UTILITY_MOTION_ROBOTMODEL_HPP
#define UTILITY_MOTION_ROBOTMODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include "utility/motion/Joint.hpp"
#include "utility/motion/Link.hpp"
#include "utility/motion/urdf_parser/link.h"
#include "utility/motion/urdf_parser/model.h"
#include "utility/motion/urdf_parser/txml.h"

namespace utility::motion {

    /**
     * @brief Represents a robot model.
     * @details
     * @param Scalar The scalar type for the robot model
     */
    template <typename Scalar>
    class RobotModel {

    private:
        /// @brief The name of the robot model.
        std::string name;

        /// @brief The number of links in the robot model.
        int n_links;

        /// @brief The number of joints in the robot model.
        int n_joints;

        /// @brief The base link of the robot model.
        std::shared_ptr<Link<Scalar>> base_link;

        /// @brief Map of links in the robot model.
        std::map<std::string, std::shared_ptr<Link<Scalar>>> links;

        /// @brief Map of joints in the robot model.
        std::map<std::string, std::shared_ptr<Joint<Scalar>>> joints;

        /// @brief The gravitational acceleration experienced by robot.
        Eigen::Matrix<Scalar, 3, 1> gravity = {0, 0, -9.81};

    public:
        /**
         * @brief Construct a new RobotModel from a URDF file description.
         * @param path_to_urdf The path to the URDF file
         */
        RobotModel(const std::string& path_to_urdf, const bool floating_base = false) {

            // Parse the URDF file
            std::cout << "Loading URDF from: " << path_to_urdf << std::endl;
            ifstream input_file(path_to_urdf);
            if (!input_file.is_open()) {
                cerr << "Could not open the file - '" << path_to_urdf << "'" << endl;
                exit(EXIT_FAILURE);
            }

            std::string urdf_string =
                string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());

            std::shared_ptr<urdf::UrdfModel> model;
            model = urdf::UrdfModel::fromUrdfStr(urdf_string);

            std::vector<std::shared_ptr<urdf::Link>> links;
            std::vector<std::shared_ptr<urdf::Joint>> joints;

            model->getLinks(links);
            // TODO: Add all the link and joints to the robot model, with option of a floating base
            for (auto link : links) {
                std::cout << "Name of link: " << link->name << std::endl;
                // Check if inertial exists
                if (link->inertial != nullopt) {
                    auto inertial = &link->inertial.value();
                    std::cout << "Mass: " << inertial->mass << std::endl;
                    std::cout << "ixx : " << inertial->ixx << std::endl;
                    std::cout << "ixy : " << inertial->ixy << std::endl;
                    std::cout << "ixz : " << inertial->ixz << std::endl;
                    std::cout << "iyy : " << inertial->iyy << std::endl;
                    std::cout << "iyz : " << inertial->iyz << std::endl;
                    std::cout << "izz : " << inertial->izz << std::endl;
                    std::cout << "inertial->origin.rotation.x : " << inertial->origin.rotation.x << std::endl;
                    std::cout << "inertial->origin.rotation.y : " << inertial->origin.rotation.y << std::endl;
                    std::cout << "inertial->origin.rotation.z : " << inertial->origin.rotation.z << std::endl;
                    std::cout << "origin.rotation.w : " << inertial->origin.rotation.w << std::endl;
                    std::cout << "origin.position.x : " << inertial->origin.position.x << std::endl;
                    std::cout << "origin.position.y : " << inertial->origin.position.y << std::endl;
                    std::cout << "origin.position.z : " << inertial->origin.position.z << std::endl << std::endl;
                }
            }


            // TODO: Get the number of links and joints
        }

        /**
         * @brief Adds a joint to the robot model.
         *
         * @param joint The joint to add to the robot model.
         */
        void addJoint(const Joint<Scalar>& joint);


        /**
         * @brief Get the number of links in the robot model.
         *
         */
        int get_n_links() const {
            return n_links;
        }

        /**
         * @brief Get the number of joints in the robot model.
         *
         */
        int get_n_joints() const {
            return n_joints;
        }

        /**
         * @brief Get the links in the robot model.
         *
         */
        const std::vector<std::shared_ptr<Link<Scalar>>>& get_links() const {
            return links;
        }

        /**
         * @brief Get the gravitational acceleration experienced by robot.
         *
         */
        const Eigen::Matrix<Scalar, 3, 1>& get_gravity() const {
            return gravity;
        }

        /**
         * @brief Set the gravitational acceleration experienced by robot
         *
         */
        void set_gravity(const Eigen::Matrix<Scalar, 3, 1>& gravity) {
            this->gravity = gravity;
        }

        /**
         * @brief Computes the transform between two links.
         * @param target_link_id {t} The link to which the transform is computed.
         * @param source_link_id {s} The link from which the transform is computed.
         * @return The transform between the two links.
         */
        Eigen::Transform<Scalar, 3, Eigen::Affine> forward_kinematics(int target_link_id, int source_link_id) const;

        /**
         * @brief Solves the inverse kinematics problem between two links.
         * @param target_link_id {t} The link to which the transform is computed.
         * @param source_link_id {s} The link from which the transform is computed.
         * @param desired_pose {d} The desired pose of the target link in the source link frame.
         * @return The configuration vector of the robot model which achieves the desired pose.
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> inverse_kinematics(
            int target_link_id,
            int source_link_id,
            const Eigen::Transform<Scalar, 3, Eigen::Affine>& desired_pose) const;

        /**
         * @brief Computes the lagrangian dynamics of the robot model.
         * @param q The configuration vector of the robot model.
         * @param qdot The velocity vector of the robot model.
         *
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> lagrangian_dynamics(
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& qdot) const;

        /**
         * @brief Computes the hamiltonian dynamics of the robot model.
         * @param q The configuration vector of the robot model.
         * @param p The momentum vector of the robot model.
         *
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> hamiltonian_dynamics(
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& p) const;

        /**
         * @brief Computes the mass matrix of the robot model.
         *
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> mass_matrix() const;

        /**
         * @brief Computes the coriolis matrix of the robot model.
         *
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> coriolis_matrix() const;

        /**
         * @brief Computes the gravity torque vector of the robot model.
         *
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> gravity_torque() const;

        /**
         * @brief Computes the geometric jacobian of the robot model.
         * @param target_link_id The link to which the jacobian is computed.
         * @param source_link_id The link from which the jacobian is computed.
         * @param q The configuration vector of the robot model.
         * @return The geometric jacobian to the target link from the source link frame.
         */
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> geometric_jacobian(
            int target_link_id,
            int source_link_id,
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q) const;

        /**
         * @brief Computes the centre of mass expressed in source link frame.
         *
         * @param source_link_id The link from which the centre of mass position is computed.
         * @param q The configuration vector of the robot model.
         *
         * @return The centre of mass position expressed in source link frame.
         */
        Eigen::Matrix<Scalar, 3, 1> centre_of_mass(int source_link_id,
                                                   const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q) const;
    };
}  // namespace utility::motion

#endif
