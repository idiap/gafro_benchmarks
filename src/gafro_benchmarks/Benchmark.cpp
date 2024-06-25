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

#include <gafro_benchmarks/Benchmark.hpp>

namespace gafro_benchmark
{

    Benchmark::Benchmark() = default;

    Benchmark::~Benchmark() = default;

    double Benchmark::runBenchmark(const std::function<void()> &function)
    {
        double average_time = 0.0;

        for (unsigned i = 0; i < 20; ++i)
        {
            average_time += timer_.computeAverageTime([&]() { function(); }) * 1e6;
        }

        average_time /= 20.0;

        return average_time;
    }

    double Benchmark::benchmarkForwardKinematics()
    {
        Vector position = Vector::Random();

        return runBenchmark([&]() { computeForwardKinematics(position); });
    }

    double Benchmark::benchmarkJacobian()
    {
        Vector position = Vector::Random();

        return runBenchmark([&]() { computeJacobian(position); });
    }

    double Benchmark::benchmarkInverseDynamics()
    {
        Vector position = Vector::Random();
        Vector velocity = Vector::Random();
        Vector acceleration = Vector::Random();

        return runBenchmark([&]() { computeInverseDynamics(position, velocity, acceleration); });
    }

    double Benchmark::benchmarkForwardDynamics()
    {
        Vector position = Vector::Random();
        Vector velocity = Vector::Random();
        Vector torque = Vector::Random();

        return runBenchmark([&]() { computeForwardDynamics(position, velocity, torque); });
    }

}  // namespace gafro_benchmark