/*
    Copyright (c) 2024 Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#include <rbdl/addons/urdfreader/urdfreader.h>
//
#include "rbdl/rbdl_mathutils.h"
//
#include <gafro_benchmarks/gafro_benchmark_package_config.hpp>
//
#include <gafro_benchmarks/BenchmarkRBDL.hpp>

namespace gafro_benchmark
{

    BenchmarkRBDL::BenchmarkRBDL()
    {
        RigidBodyDynamics::Addons::URDFReadFromFile((GAFRO_ROBOT_DESCRIPTIONS + "panda_rbdl.urdf").c_str(), &panda_, false);
    }

    BenchmarkRBDL::~BenchmarkRBDL() {}

    void BenchmarkRBDL::computeForwardKinematics(const Vector &position)
    {
        Eigen::Vector3d point = RigidBodyDynamics::CalcBaseToBodyCoordinates(panda_, position, 7, Eigen::Vector3d::Zero());
        Eigen::Matrix3d orientation = RigidBodyDynamics::CalcBodyWorldOrientation(panda_, position, 7);
    }

    void BenchmarkRBDL::computeJacobian(const Vector &position)
    {
        Eigen::Vector3d point = RigidBodyDynamics::CalcBaseToBodyCoordinates(panda_, position, 7, Eigen::Vector3d::Zero());
        RigidBodyDynamics::Math::MatrixNd jacobian = RigidBodyDynamics::Math::MatrixNd::Zero(6, 7);
        RigidBodyDynamics::CalcPointJacobian6D(panda_, position, 7, point, jacobian, false);
    }

    Benchmark::Vector BenchmarkRBDL::computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration)
    {
        RigidBodyDynamics::Math::VectorNd torque(7);
        RigidBodyDynamics::InverseDynamics(panda_, position, velocity, acceleration, torque);

        return torque;
    }

    Benchmark::Vector BenchmarkRBDL::computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque)
    {
        RigidBodyDynamics::Math::VectorNd acceleration(7);
        std::vector<RigidBodyDynamics::Math::SpatialVector> external_forces;

        RigidBodyDynamics::ForwardDynamics(panda_, position, velocity, torque, acceleration);

        return acceleration;
    }

    const std::string &BenchmarkRBDL::getName() const
    {
        static const std::string name = "RBDL";
        return name;
    }

    bool BenchmarkRBDL::hasForwardKinematics()
    {
        return true;
    }

    bool BenchmarkRBDL::hasJacobian()
    {
        return true;
    }

    bool BenchmarkRBDL::hasInverseDynamics()
    {
        return true;
    }

    bool BenchmarkRBDL::hasForwardDynamics()
    {
        return true;
    }

    REGISTER_CLASS(Benchmark, BenchmarkRBDL, "gafro_benchmark_rbdl");

}  // namespace gafro_benchmark