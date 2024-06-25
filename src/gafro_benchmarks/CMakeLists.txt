find_package(sackmesser REQUIRED)
find_package(scatter REQUIRED)
find_package(gafro_robot_descriptions REQUIRED)

add_library(gafro_benchmark SHARED ${CMAKE_CURRENT_LIST_DIR}/Benchmark.cpp)
target_include_directories(gafro_benchmark PUBLIC "${CMAKE_CURRENT_LIST_DIR}/..")
target_include_directories(gafro_benchmark PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>/..)
target_link_libraries(gafro_benchmark sackmesser::sackmesser scatter::scatter)
target_compile_options(gafro_benchmark PUBLIC "-O3" "-march=native")

LIST(APPEND GAFRO_BENCHMARK_LIBRARIES "")

### GAFRO #############################################################################################
    SET(gafro_FOUND FALSE)

    find_package(gafro)

    if(${gafro_FOUND})
        target_sources(gafro_benchmark PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BenchmarkGafro.cpp)
        target_link_libraries(gafro_benchmark gafro::gafro gafro_robot_descriptions::gafro_robot_descriptions)

        LIST(APPEND GAFRO_BENCHMARK_LIBRARIES "gafro_benchmark_gafro")

        message("adding gafro to Benchmark")
    endif()
### GAFRO #############################################################################################

### RAISIM #############################################################################################
    SET(raisim_FOUND FALSE)

    find_package(raisim)

    if(${raisim_FOUND})
        target_sources(gafro_benchmark PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BenchmarkRaisim.cpp)
        target_link_libraries(gafro_benchmark raisim::raisim)

        LIST(APPEND GAFRO_BENCHMARK_LIBRARIES "gafro_benchmark_raisim")

        message("adding raisim to Benchmark")
    endif()
### RAISIM #############################################################################################

### PINOCCHIO ##########################################################################################
    SET(pinocchio_FOUND FALSE)

    find_package(pinocchio)

    if(${pinocchio_FOUND})
        target_sources(gafro_benchmark PRIVATE BenchmarkPinocchio.cpp)
        target_link_libraries(gafro_benchmark pinocchio::pinocchio)

        LIST(APPEND GAFRO_BENCHMARK_LIBRARIES "gafro_benchmark_pinocchio")

        message("adding pinocchio to Benchmark")
    endif()
### PINOCCHIO ##########################################################################################

### KDL ###########################################################################################
    SET(orocos_kdl_FOUND FALSE)

    find_package(catkin COMPONENTS orocos_kdl kdl_parser)

    if(${catkin_FOUND})
        target_sources(gafro_benchmark PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BenchmarkKDL.cpp)
        target_include_directories(gafro_benchmark PUBLIC 
                ${catkin_INCLUDE_DIRS}
            )
        target_link_libraries(gafro_benchmark 
                ${catkin_LIBRARIES}
            )

        LIST(APPEND GAFRO_BENCHMARK_LIBRARIES "gafro_benchmark_kdl")

        message("adding kdl to Benchmark")
    endif()
### KDL ###########################################################################################

### MUJOCO ###########################################################################################
    SET(mujoco_FOUND FALSE)

    find_package(mujoco)

    if(${mujoco_FOUND})
        target_sources(gafro_benchmark PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BenchmarkMujoco.cpp)
        
        target_link_libraries(gafro_benchmark mujoco)

        LIST(APPEND GAFRO_BENCHMARK_LIBRARIES "gafro_benchmark_mujoco")

        message("adding mujoco to Benchmark")
    endif()
### MUJOCO ###########################################################################################

### RBDL ###########################################################################################
    SET(RBDL_FOUND FALSE)

    find_package(RBDL)

    if(${RBDL_FOUND})
        target_sources(gafro_benchmark PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BenchmarkRBDL.cpp)
        
        target_link_libraries(gafro_benchmark ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})
        target_include_directories(gafro_benchmark PUBLIC ${RBDL_INCLUDE_DIRS})

        LIST(APPEND GAFRO_BENCHMARK_LIBRARIES "gafro_benchmark_rbdl")

        message("adding RBDL to Benchmark")
    endif()
### RBDL ###########################################################################################

### DQROBOTICS ###########################################################################################
    # SET(dqrobotics_FOUND TRUE)

    find_library(DQROBOTICSLIB dqrobotics "/usr/local/lib")

    if(DQROBOTICSLIB)
        target_sources(gafro_benchmark PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BenchmarkDQRobotics.cpp)
        target_link_libraries(gafro_benchmark dqrobotics)

        LIST(APPEND GAFRO_BENCHMARK_LIBRARIES "gafro_benchmark_dqrobotics")

        message("adding dqrobotics to Benchmark")
    endif()
### DQROBOTICS ###########################################################################################

configure_file(${CMAKE_CURRENT_LIST_DIR}/gafro_benchmark_package_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/gafro_benchmark_package_config.hpp")

add_executable(run_benchmarks run_benchmarks.cpp)
target_include_directories(run_benchmarks PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>/..)
target_link_libraries(run_benchmarks PUBLIC gafro_benchmark)
target_compile_options(run_benchmarks PUBLIC "-O3" "-march=native" "-msse4.1")

add_executable(compare_results compare_results.cpp)
target_include_directories(compare_results PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>/..)
target_link_libraries(compare_results PUBLIC gafro_benchmark)
target_compile_options(compare_results PUBLIC "-O3" "-march=native")