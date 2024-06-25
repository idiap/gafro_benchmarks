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
#include <gafro_benchmarks/BenchmarkMujoco.hpp>

namespace gafro_benchmark
{
    BenchmarkMujoco::BenchmarkMujoco()
    {
        panda_model_ = mj_loadXML((GAFRO_ROBOT_DESCRIPTIONS + "panda_mujoco.xml").c_str(), nullptr, nullptr, 0);
        panda_data_ = mj_makeData(panda_model_);
    }

    BenchmarkMujoco::~BenchmarkMujoco() {}

    void BenchmarkMujoco::computeForwardKinematics(const Vector &position)
    {
        panda_data_->qpos = const_cast<Vector &>(position).data();

        mj_kinematics(panda_model_, panda_data_);
    }

    void BenchmarkMujoco::computeJacobian(const Vector &position)
    {
        mjtNum jacp[21];
        mjtNum jacr[21];

        mj_kinematics(panda_model_, panda_data_);
        mjtNum *point = panda_data_->site_xpos;

        mj_jac(panda_model_, panda_data_, jacp, jacr, point, 6);
    }

    Benchmark::Vector BenchmarkMujoco::computeInverseDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration)
    {
        panda_data_->qpos = const_cast<Vector &>(position).data();
        panda_data_->qvel = const_cast<Vector &>(velocity).data();
        panda_data_->qacc = const_cast<Vector &>(acceleration).data();

        mj_inverse(panda_model_, panda_data_);

        return Benchmark::Vector::Zero();
    }

    Benchmark::Vector BenchmarkMujoco::computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &torque)
    {
        panda_data_->qpos = const_cast<Vector &>(position).data();
        panda_data_->qvel = const_cast<Vector &>(velocity).data();
        panda_data_->qfrc_applied = const_cast<Vector &>(torque).data();

        mj_forward(panda_model_, panda_data_);

        return Vector(panda_data_->qacc);
    }

    const std::string &BenchmarkMujoco::getName() const
    {
        static const std::string name = "MuJoCo";
        return name;
    }

    bool BenchmarkMujoco::hasForwardKinematics()
    {
        return true;
    }

    bool BenchmarkMujoco::hasJacobian()
    {
        return true;
    }

    bool BenchmarkMujoco::hasInverseDynamics()
    {
        return true;
    }

    bool BenchmarkMujoco::hasForwardDynamics()
    {
        return true;
    }

    REGISTER_CLASS(Benchmark, BenchmarkMujoco, "gafro_benchmark_mujoco");

}  // namespace gafro_benchmark