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

#include <gafro_benchmarks/BenchmarkDQRobotics.hpp>

namespace gafro_benchmark
{

    BenchmarkDQRobotics::BenchmarkDQRobotics() : panda_(DQ_robotics::FrankaEmikaPandaRobot::kinematics()) {}

    BenchmarkDQRobotics::~BenchmarkDQRobotics() {}

    void BenchmarkDQRobotics::computeForwardKinematics(const Vector &position)
    {
        panda_.fkm(position);
    }

    void BenchmarkDQRobotics::computeJacobian(const Vector &position)
    {
        Eigen::MatrixXd jacobian = panda_.pose_jacobian(position);
    }

    Benchmark::Vector BenchmarkDQRobotics::computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration)
    {
        return Vector::Zero();
    }

    Benchmark::Vector BenchmarkDQRobotics::computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque)
    {
        return Vector::Zero();
    }

    const std::string &BenchmarkDQRobotics::getName() const
    {
        static const std::string name = "DQ Robotics";
        return name;
    }

    bool BenchmarkDQRobotics::hasForwardKinematics()
    {
        return true;
    }

    bool BenchmarkDQRobotics::hasJacobian()
    {
        return true;
    }

    bool BenchmarkDQRobotics::hasInverseDynamics()
    {
        return false;
    }

    bool BenchmarkDQRobotics::hasForwardDynamics()
    {
        return false;
    }

    REGISTER_CLASS(Benchmark, BenchmarkDQRobotics, "gafro_benchmark_dqrobotics");

}  // namespace gafro_benchmark