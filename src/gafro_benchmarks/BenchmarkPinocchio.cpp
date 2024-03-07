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

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hxx>
#include <pinocchio/parsers/urdf.hpp>
//
#include <gafro_benchmarks/gafro_benchmark_package_config.hpp>
//
#include <gafro_benchmarks/BenchmarkPinocchio.hpp>

namespace gafro_benchmark
{

    BenchmarkPinocchio::BenchmarkPinocchio()
    {
        pinocchio::urdf::buildModel(GAFRO_ROBOT_DESCRIPTIONS + "panda_pinocchio.urdf", panda_model_);
        panda_data_ = pinocchio::Data(panda_model_);
    }

    BenchmarkPinocchio::~BenchmarkPinocchio() {}

    void BenchmarkPinocchio::computeForwardKinematics(const Vector &position)
    {
        pinocchio::forwardKinematics(panda_model_, panda_data_, position);
    }

    void BenchmarkPinocchio::computeJacobian(const Vector &position)
    {
        pinocchio::computeJointJacobians(panda_model_, panda_data_, position);
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
        pinocchio::getJointJacobian(panda_model_, panda_data_, 7, pinocchio::ReferenceFrame(), jacobian);
    }

    Benchmark::Vector BenchmarkPinocchio::computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration)
    {
        return pinocchio::rnea(panda_model_, panda_data_, position, velocity, acceleration);
    }

    Benchmark::Vector BenchmarkPinocchio::computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque)
    {
        return pinocchio::aba(panda_model_, panda_data_, position, velocity, torque);
    }

    const std::string &BenchmarkPinocchio::getName() const
    {
        static const std::string name = "pinocchio";
        return name;
    }

    REGISTER_CLASS(Benchmark, BenchmarkPinocchio, "gafro_benchmark_pinocchio");

}  // namespace gafro_benchmark