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

#include <gafro_benchmarks/gafro_benchmark_package_config.hpp>
//
#include <gafro_benchmarks/BenchmarkRaisim.hpp>

namespace gafro_benchmark
{

    BenchmarkRaisim::BenchmarkRaisim()
    {
        world_.setTimeStep(0.001);
        world_.addGround(0, "gnd");
        panda_ = world_.addArticulatedSystem(GAFRO_ROBOT_DESCRIPTIONS + "panda_raisim.urdf");
    }

    BenchmarkRaisim::~BenchmarkRaisim() {}

    void BenchmarkRaisim::computeForwardKinematics(const Vector &position)
    {
        panda_->setGeneralizedCoordinate(position);

        raisim::Vec<3> ee_position;
        raisim::Mat<3, 3> ee_rotation;

        panda_->getPosition(panda_->getBodyIdx("panda_link7"), ee_position);
        panda_->getOrientation(panda_->getBodyIdx("panda_link7"), ee_rotation);
    }

    void BenchmarkRaisim::computeJacobian(const Vector &position)
    {
        panda_->setGeneralizedCoordinate(position);

        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
        Eigen::MatrixXd jacobian_pos = Eigen::MatrixXd::Zero(3, 7);
        Eigen::MatrixXd jacobian_rot = Eigen::MatrixXd::Zero(3, 7);

        panda_->getDenseFrameJacobian(panda_->getBodyIdx("panda_link7"), jacobian_pos);
        panda_->getDenseRotationalJacobian(panda_->getBodyIdx("panda_link7"), jacobian_rot);

        jacobian.topRows(3) = jacobian_rot;
        jacobian.bottomRows(3) = jacobian_pos;
    }

    Benchmark::Vector BenchmarkRaisim::computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration)
    {
        panda_->setState(position, velocity);

        return panda_->getMassMatrix().e() * acceleration + panda_->getNonlinearities(world_.getGravity().e()).e();
    }

    Benchmark::Vector BenchmarkRaisim::computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque)
    {
        panda_->setState(position, velocity);
        panda_->setGeneralizedForce(torque);
        panda_->getMassMatrix();

        return panda_->getInverseMassMatrix().e() * (torque - panda_->getNonlinearities(world_.getGravity().e()).e());
    }

    const std::string &BenchmarkRaisim::getName() const
    {
        static const std::string name = "Raisim";
        return name;
    }

    REGISTER_CLASS(Benchmark, BenchmarkRaisim, "gafro_benchmark_raisim");

}  // namespace gafro_benchmark