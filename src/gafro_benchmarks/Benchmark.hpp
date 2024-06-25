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

#pragma once

#include <Eigen/Core>
#include <sackmesser/FactoryClass.hpp>
#include <sackmesser/Timer.hpp>

namespace gafro_benchmark
{

    class Benchmark : public sackmesser::FactoryClass<Benchmark>
    {
      public:
        using Vector = Eigen::Vector<double, 7>;

        Benchmark();

        virtual ~Benchmark();

        virtual bool hasForwardKinematics() = 0;

        virtual bool hasJacobian() = 0;

        virtual bool hasInverseDynamics() = 0;

        virtual bool hasForwardDynamics() = 0;

        double benchmarkForwardKinematics();

        double benchmarkJacobian();

        double benchmarkInverseDynamics();

        double benchmarkForwardDynamics();

        virtual const std::string &getName() const = 0;

        double runBenchmark(const std::function<void()> &function);

        virtual void computeForwardKinematics(const Vector &position) = 0;

        virtual void computeJacobian(const Vector &position) = 0;

        virtual Vector computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration) = 0;

        virtual Vector computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque) = 0;

      private:
        sackmesser::Timer timer_;
    };

}  // namespace gafro_benchmark