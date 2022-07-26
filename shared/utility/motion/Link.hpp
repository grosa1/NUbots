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

#ifndef UTILITY_MOTION_LINK_HPP
#define UTILITY_MOTION_LINK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utility/motion/Joint.hpp"

namespace utility::motion {

    /**
     * @brief Represents a link in a robot model.
     * @details
     * @param Scalar Is the scalar type of the link
     */
    template <typename Scalar>
    class Link {

    private:
        /// @brief Name of the link
        const std::string LINK_NAME;

        /// @brief Unique ID of the link
        const int LINK_ID;

        /// @brief The links joint.
        const Joint<Scalar> JOINT;

        /// @brief The links centre of mass in the link's frame.
        const Eigen::Matrix<Scalar, 3, 1> CENTRE_OF_MASS;

        /// @brief The links inertia matrix [kg m^2].
        const Eigen::Matrix<Scalar, 3, 3> INERTIA;

        /// @brief The links mass [kg].
        const Scalar MASS;

        /// @brief The links parent link.
        std::shared_ptr<Link<Scalar>> parent_link;

        /// @brief The list of child links.
        std::vector<std::shared_ptr<Link<Scalar>>> child_links;

    public:
        /**
         * @brief Construct a new Link from a URDF file description.
         * @param link_name The name of the link
         * @param link_id The unique ID of the link
         * @param joint The joint of the link
         * @param centre_of_mass The centre of mass of the link
         * @param inertia The inertia matrix of the link
         * @param mass The mass of the link
         * @param parent_link The parent link of the link
         * @param child_links The child links of the link
         */
        Link(const std::string& link_name,
             const int link_id,
             const Joint<Scalar>& joint,
             const Eigen::Matrix<Scalar, 3, 1>& centre_of_mass,
             const Eigen::Matrix<Scalar, 3, 3>& inertia,
             const Scalar mass,
             std::shared_ptr<Link<Scalar>> parent_link,
             std::vector<std::shared_ptr<Link<Scalar>>> child_links) {
            this->LINK_NAME      = link_name;
            this->LINK_ID        = link_id;
            this->JOINT          = joint;
            this->CENTRE_OF_MASS = centre_of_mass;
            this->INERTIA        = inertia;
            this->MASS           = mass;
            this->parent_link    = parent_link;
            this->child_links    = child_links;
        }

        /**
         * @brief Get the name of the link.
         * @return The name of the link.
         */
        const std::string& get_name() const {
            return LINK_NAME;
        }

        /**
         * @brief Get the unique ID of the link.
         * @return The unique ID of the link.
         */
        int get_id() const {
            return LINK_ID;
        }

        /**
         * @brief Get the joint of the link.
         * @return The joint of the link.
         */
        const Joint<Scalar>& get_joint() const {
            return JOINT;
        }

        /**
         * @brief Add a joint to the link.
         * @param joint The joint to add to the link.
         */
        void add_joint(const Joint<Scalar>& joint) {
            JOINT = joint;
        }
    };
}  // namespace utility::motion

#endif
