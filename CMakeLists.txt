cmake_minimum_required(VERSION 3.22)
project(ebus_vns)

set(CMAKE_CXX_STANDARD 23)

# Compiler options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# CPLEX and Concert directories
set(CPLEXDIR "/opt/ibm/ILOG/CPLEX_Studio2211/cplex")
set(CONCERTDIR "/opt/ibm/ILOG/CPLEX_Studio2211/concert")

# CPLEX and Concert library directories
set(CPLEXLIBDIR "${CPLEXDIR}/lib/x86-64_linux/static_pic")
set(CONCERTLIBDIR "${CONCERTDIR}/lib/x86-64_linux/static_pic")

# Link options and libraries
link_directories(${CPLEXLIBDIR} ${CONCERTLIBDIR})
set(CCLNFLAGS "-lilocplex -lcplex -lconcert -lm -pthread")

# Include directories
include_directories(
        ${CPLEXDIR}/include
        ${CONCERTDIR}/include
        ./include
)

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

# Link the executable with required libraries
target_link_libraries(ebus_vns PRIVATE ${CCLNFLAGS})