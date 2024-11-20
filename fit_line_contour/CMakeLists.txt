# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# Compile core.cpp into a shared library
ADD_LIBRARY(core SHARED core.cpp)
TARGET_LINK_LIBRARIES(core ${OpenCV_LIBS})

# Compile main.cpp into an executable
# ADD_EXECUTABLE(Unit_test main.cpp)
# TARGET_LINK_LIBRARIES(Unit_test core ${OpenCV_LIBS})
