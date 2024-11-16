cmake_minimum_required(VERSION 3.24)

PROJECT( FitLine )

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# Compile core.cpp into a shared library
ADD_LIBRARY(core SHARED core.cpp)
TARGET_LINK_LIBRARIES(core ${OpenCV_LIBS})

# Compile main.cpp into an executable
# ADD_EXECUTABLE(Unit_test main.cpp)
# TARGET_LINK_LIBRARIES(Unit_test core ${OpenCV_LIBS})
