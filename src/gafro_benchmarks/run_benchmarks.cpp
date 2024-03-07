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

namespace sackmesser
{
    std::vector<std::string> split_string(const std::string &string, const char &delimiter)
    {
        std::vector<std::string> split;

        std::size_t p1 = 0, p2 = 0;
        while (p2 != std::string::npos)
        {
            p2 = string.find(delimiter, p1);
            split.push_back(string.substr(static_cast<unsigned>(p1), ((p2 != std::string::npos) ? p2 : string.size()) - static_cast<unsigned>(p1)));
            p1 = p2 + 1;
        }

        return split;
    }
}  // namespace sackmesser

int main(int argc, char **argv)
{
    std::vector<std::unique_ptr<Benchmark>> benchmarks;

    for (const std::string &library : sackmesser::split_string(GAFRO_BENCHMARK_LIBRARIES, ';'))
    {
        benchmarks.push_back(Benchmark::getFactory()->createUnique(library));

        std::cout << "added benchmark: " << benchmarks.back()->getName() << std::endl;
    }

    std::cout << "---------" << std::endl;

    scatter::FigureOptions figure_options;
    figure_options.setWidth(2000.0);
    figure_options.setHeight(1000.0);

    scatter::Figure figure(figure_options, 2, 2);

    {
        scatter::Plot::Ptr plot = scatter::Plot::create("Forward Kinematics");

        double x = 0.5;

        std::cout << "FORWARD KINEMATICS" << std::endl;

        for (const auto &benchmark : benchmarks)
        {
            std::cout << "run " << benchmark->getName() << std::endl;

            plot
              ->add<scatter::BarPlot>(benchmark->getName(),
                                      std::vector<scatter::Point>({ scatter::Point(x, benchmark->benchmarkForwardKinematics()) }))
              ->setWidth(0.25);

            x += 1.0;
        }

        plot->options().getAxisOptions().setXlabel("");
        plot->options().getAxisOptions().setYlabel("time [ns]");

        plot->options().getAxisOptions().setXmin(0.0);
        plot->options().getAxisOptions().setXmax(x - 0.5);
        plot->options().getAxisOptions().setYticks(5);
        plot->options().getAxisOptions().setYmax(2500);
        plot->options().getAxisOptions().setYPrecision(4);

        plot->options().getAxisOptions().setShowXticks(false);
        plot->options().getAxisOptions().setShowYticks(true);
        plot->options().getAxisOptions().setShowGrid(true);
        plot->options().getAxisOptions().setShow(true);

        plot->options().getLegendOptions().setShow(false);
        plot->options().getLegendOptions().setAnchor(scatter::Anchor::NORTH_WEST);

        figure.add(plot, 0, 0);
    }

    std::cout << "---------" << std::endl;

    {
        scatter::Plot::Ptr plot = scatter::Plot::create("Jacobian");

        double x = 0.5;

        std::cout << "JACOBIAN" << std::endl;

        for (const auto &benchmark : benchmarks)
        {
            std::cout << "run " << benchmark->getName() << std::endl;

            plot->add<scatter::BarPlot>(benchmark->getName(), std::vector<scatter::Point>({ scatter::Point(x, benchmark->benchmarkJacobian()) }))
              ->setWidth(0.25);

            x += 1.0;
        }

        plot->options().getAxisOptions().setXlabel("");
        plot->options().getAxisOptions().setYlabel("time [ns]");

        plot->options().getAxisOptions().setXmin(0.0);
        plot->options().getAxisOptions().setXmax(x + 0.5);
        plot->options().getAxisOptions().setYticks(5);
        plot->options().getAxisOptions().setYmax(4000);
        plot->options().getAxisOptions().setYPrecision(4);

        plot->options().getAxisOptions().setShowXticks(false);
        plot->options().getAxisOptions().setShowYticks(true);
        plot->options().getAxisOptions().setShowGrid(true);
        plot->options().getAxisOptions().setShow(true);

        plot->options().getLegendOptions().setShow(false);
        plot->options().getLegendOptions().setAnchor(scatter::Anchor::NORTH_WEST);

        figure.add(plot, 0, 1);
    }

    std::cout << "---------" << std::endl;

    {
        scatter::Plot::Ptr plot = scatter::Plot::create("Inverse Dynamics");

        double x = 0.5;

        std::cout << "INVERSE DYNAMICS" << std::endl;

        for (const auto &benchmark : benchmarks)
        {
            std::cout << "run " << benchmark->getName() << std::endl;

            plot
              ->add<scatter::BarPlot>(benchmark->getName(), std::vector<scatter::Point>({ scatter::Point(x, benchmark->benchmarkInverseDynamics()) }))
              ->setWidth(0.25);

            x += 1.0;
        }

        plot->options().getAxisOptions().setXlabel("");
        plot->options().getAxisOptions().setYlabel("time [ns]");

        plot->options().getAxisOptions().setXmin(0.0);
        plot->options().getAxisOptions().setXmax(x + 0.5);
        plot->options().getAxisOptions().setYticks(5);
        plot->options().getAxisOptions().setYmax(25000);
        plot->options().getAxisOptions().setYPrecision(4);

        plot->options().getAxisOptions().setShowXticks(false);
        plot->options().getAxisOptions().setShowYticks(true);
        plot->options().getAxisOptions().setShowGrid(true);
        plot->options().getAxisOptions().setShow(true);

        plot->options().getLegendOptions().setShow(false);
        plot->options().getLegendOptions().setAnchor(scatter::Anchor::NORTH_WEST);

        figure.add(plot, 1, 0);
    }

    std::cout << "---------" << std::endl;

    {
        scatter::Plot::Ptr plot = scatter::Plot::create("Forward Dynamics");

        double x = 0.5;

        std::cout << "FORWARD DYNAMICS" << std::endl;

        for (const auto &benchmark : benchmarks)
        {
            std::cout << "run " << benchmark->getName() << std::endl;

            plot
              ->add<scatter::BarPlot>(benchmark->getName(), std::vector<scatter::Point>({ scatter::Point(x, benchmark->benchmarkForwardDynamics()) }))
              ->setWidth(0.25);

            x += 1.0;
        }

        plot->options().getAxisOptions().setXlabel("");
        plot->options().getAxisOptions().setYlabel("time [ns]");

        plot->options().getAxisOptions().setXmin(0.0);
        plot->options().getAxisOptions().setXmax(x + 0.5);
        plot->options().getAxisOptions().setYmax(75000);
        plot->options().getAxisOptions().setYticks(10);
        plot->options().getAxisOptions().setYPrecision(5);

        plot->options().getAxisOptions().setShowXticks(false);
        plot->options().getAxisOptions().setShowYticks(true);
        plot->options().getAxisOptions().setShowGrid(true);
        plot->options().getAxisOptions().setShow(true);

        plot->options().getLegendOptions().setShow(true);
        plot->options().getLegendOptions().setAnchor(scatter::Anchor::NORTH_EAST);

        figure.add(plot, 1, 1);
    }

    figure.save("benchmarks.pdf");

    return 0;
}
