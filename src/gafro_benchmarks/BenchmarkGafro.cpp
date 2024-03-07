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

#include <gafro_benchmarks/BenchmarkGafro.hpp>

namespace gafro_benchmark
{

    BenchmarkGafro::BenchmarkGafro() {}

    BenchmarkGafro::~BenchmarkGafro() {}

    void BenchmarkGafro::computeForwardKinematics(const Vector &position)
    {
        panda_.getEEMotor(position);
    }

    void BenchmarkGafro::computeJacobian(const Vector &position)
    {
        panda_.getEEGeometricJacobian(position);
    }

    Benchmark::Vector BenchmarkGafro::computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration)
    {
        return panda_.getJointTorques(position, velocity, acceleration, 9.81);
    }

    Benchmark::Vector BenchmarkGafro::computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque)
    {
        return panda_.getJointAccelerations(position, velocity, torque);
    }

    const std::string &BenchmarkGafro::getName() const
    {
        static const std::string name = "gafro";
        return name;
    }

    REGISTER_CLASS(Benchmark, BenchmarkGafro, "gafro_benchmark_gafro");

}  // namespace gafro_benchmark