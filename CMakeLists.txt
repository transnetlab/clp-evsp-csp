cmake_minimum_required(VERSION 3.22)
project(ebus_vns)

set(CMAKE_CXX_STANDARD 23)

add_executable(ebus_vns main.cpp
        vehicle.cpp
        vehicle.h
        helpers.cpp
        helpers.h
        operators.cpp
        operators.h
        constants.h
        logger.h
        csp.cpp
        csp.h
)
