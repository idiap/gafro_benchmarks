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

#include <raisim/World.hpp>
//
#include <gafro_benchmarks/Benchmark.hpp>

namespace gafro_benchmark
{

    class BenchmarkRaisim : public Benchmark
    {
      public:
        BenchmarkRaisim();

        virtual ~BenchmarkRaisim();

        void computeForwardKinematics(const Vector &position);

        void computeJacobian(const Vector &position);

        Vector computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration);

        Vector computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque);

        const std::string &getName() const;

        bool hasForwardKinematics();

        bool hasJacobian();

        bool hasInverseDynamics();

        bool hasForwardDynamics();

      protected:
      private:
        raisim::World world_;
        raisim::ArticulatedSystem *panda_;
    };

}  // namespace gafro_benchmark