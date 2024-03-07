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

#include <kdl/chain.hpp>
#include <kdl/chainfdsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
//
#include <gafro_benchmarks/Benchmark.hpp>

namespace gafro_benchmark
{

    class BenchmarkKDL : public Benchmark
    {
      public:
        BenchmarkKDL();

        virtual ~BenchmarkKDL();

        void computeForwardKinematics(const Vector &position);

        void computeJacobian(const Vector &position);

        Vector computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration);

        Vector computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque);

        const std::string &getName() const;

      protected:
      private:
        KDL::Chain panda_;
        KDL::Tree tree_;

        std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        std::shared_ptr<KDL::ChainIdSolver_RNE> id_solver_;
        std::shared_ptr<KDL::ChainFdSolver_RNE> fd_solver_;
    };

}  // namespace gafro_benchmark