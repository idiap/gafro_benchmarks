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
#include <gafro_benchmarks/gafro_benchmark_package_config.hpp>
#include <scatter/scatter.hpp>

using namespace gafro_benchmark;

int main(int argc, char **argv)
{
    auto gafro = Benchmark::getFactory()->createUnique("gafro_benchmark_gafro");

    std::vector<std::unique_ptr<Benchmark>> benchmarks;

    benchmarks.push_back(Benchmark::getFactory()->createUnique("gafro_benchmark_pinocchio"));
    benchmarks.push_back(Benchmark::getFactory()->createUnique("gafro_benchmark_kdl"));
    benchmarks.push_back(Benchmark::getFactory()->createUnique("gafro_benchmark_rbdl"));

    std::array<double, 3> error({ 0.0, 0.0, 0.0 });

    for (unsigned i = 0; i < 1000; ++i)
    {
        Eigen::Vector<double, 7> position = Eigen::Vector<double, 7>::Random();
        Eigen::Vector<double, 7> velocity = Eigen::Vector<double, 7>::Random();
        Eigen::Vector<double, 7> torque = Eigen::Vector<double, 7>::Random();

        Eigen::Vector<double, 7> gafro_acceleration = gafro->computeForwardDynamics(position, velocity, torque);

        for (unsigned k = 0; k < 3; k++)
        {
            Eigen::Vector<double, 7> acceleration = benchmarks[k]->computeForwardDynamics(position, velocity, torque);

            error[k] += (gafro_acceleration - acceleration).norm();
        }
    }

    for (unsigned k = 0; k < 3; k++)
    {
        std::cout << benchmarks[k]->getName() << " " << error[k] / 1000.0 << std::endl;
    }

    return 0;
}
