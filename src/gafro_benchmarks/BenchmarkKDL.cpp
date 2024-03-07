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

#include <kdl_parser/kdl_parser.hpp>
//
#include <gafro_benchmarks/gafro_benchmark_package_config.hpp>
//
#include <gafro_benchmarks/BenchmarkKDL.hpp>

namespace gafro_benchmark
{

    BenchmarkKDL::BenchmarkKDL()
    {
        kdl_parser::treeFromFile(GAFRO_ROBOT_DESCRIPTIONS + "panda_kdl.urdf", tree_);

        if (!tree_.getChain("panda_link0", "panda_hand_tcp", panda_))
        {
            throw std::runtime_error("no chain");
        }

        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(panda_);
        jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(panda_);
        id_solver_ = std::make_shared<KDL::ChainIdSolver_RNE>(panda_, KDL::Vector(0.0, 0.0, -9.81));
        fd_solver_ = std::make_shared<KDL::ChainFdSolver_RNE>(panda_, KDL::Vector(0.0, 0.0, -9.81));
    }

    BenchmarkKDL::~BenchmarkKDL() {}

    void BenchmarkKDL::computeForwardKinematics(const Vector &position)
    {
        KDL::JntArray joint_position = KDL::JntArray(7);
        joint_position.data = position;

        KDL::Frame end_frame;

        fk_solver_->JntToCart(joint_position, end_frame);
    }

    void BenchmarkKDL::computeJacobian(const Vector &position)
    {
        KDL::JntArray joint_position = KDL::JntArray(7);
        joint_position.data = position;

        KDL::Jacobian jacobian(7);

        jac_solver_->JntToJac(joint_position, jacobian);
    }

    Benchmark::Vector BenchmarkKDL::computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration)
    {
        KDL::JntArray kdl_q(7);
        KDL::JntArray kdl_q_dot(7);
        KDL::JntArray kdl_q_dotdot(7);

        kdl_q.data = position;
        kdl_q_dot.data = velocity;
        kdl_q_dotdot.data = acceleration;

        KDL::Wrenches wrenches(panda_.getNrOfSegments());
        for (unsigned i = 0; i < panda_.getNrOfSegments(); ++i)
        {
            wrenches[i] = KDL::Wrench::Zero();
        }

        KDL::JntArray torques(7);
        id_solver_->CartToJnt(kdl_q, kdl_q_dot, kdl_q_dotdot, wrenches, torques);

        return torques.data;
    }

    Benchmark::Vector BenchmarkKDL::computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque)
    {
        KDL::JntArray kdl_q(7);
        KDL::JntArray kdl_q_dot(7);
        KDL::JntArray kdl_tau(7);

        kdl_q.data = position;
        kdl_q_dot.data = velocity;
        kdl_tau.data = torque;

        KDL::Wrenches wrenches(panda_.getNrOfSegments());
        for (unsigned i = 0; i < panda_.getNrOfSegments(); ++i)
        {
            wrenches[i] = KDL::Wrench::Zero();
        }

        KDL::JntArray acceleration(7);
        id_solver_->CartToJnt(kdl_q, kdl_q_dot, kdl_tau, wrenches, acceleration);

        return acceleration.data;
    }

    const std::string &BenchmarkKDL::getName() const
    {
        static const std::string name = "KDL";
        return name;
    }

    REGISTER_CLASS(Benchmark, BenchmarkKDL, "gafro_benchmark_kdl");

}  // namespace gafro_benchmark