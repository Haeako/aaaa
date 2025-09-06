cmake_minimum_required(VERSION 2.8)
project(PiD_ballance_ball CXX)

add_library(AccelStepper/
        lib/AccelStepper.h
        lib/AccelStepper.h
)
add_library(MultiStepper
        lib/MultiStepper.h
        lib/MultiStepper.cpp
)
add_library(comunicate
        lib/comunicate.h
        lib/comunicate.cpp
)
add_library(InverseKinematics
        lib/InverseKinematics.h
        lib/InverseKinematics.cpp
)
add_executable(main main.cpp)