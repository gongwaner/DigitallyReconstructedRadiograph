cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

project(DRR)

find_package(VTK COMPONENTS
        CommonColor
        CommonCore
        IOImage
        InteractionImage
        InteractionStyle
        RenderingContextOpenGL2
        RenderingCore
        RenderingFreeType
        RenderingGL2PSOpenGL2
        RenderingOpenGL2
)

if (NOT VTK_FOUND)
    message(FATAL_ERROR "ReadDICOMSeries: Unable to find the VTK build folder.")
endif ()

# Prevent a "command line is too long" failure in Windows.
set(CMAKE_NINJA_FORCE_RESPONSE_FILE "ON" CACHE BOOL "Force Ninja to use response files.")

add_executable(${PROJECT_NAME} MACOSX_BUNDLE main.cxx
        Algorithm/DigitallyReconstructedRadiograph.cpp
        Algorithm/DigitallyReconstructedRadiograph.h
        Algorithm/RayCastHelper.cpp
        Algorithm/RayCastHelper.h
        Algorithm/RayCastInterpolateImageFunction.cpp
        Algorithm/RayCastInterpolateImageFunction.h
        Utility/TestUtil.cpp
        Utility/TestUtil.h
        Utility/Utility.cpp
        Utility/Utility.h
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${VTK_LIBRARIES})

# vtk_module_autoinit is needed
vtk_module_autoinit(
        TARGETS ${PROJECT_NAME}
        MODULES ${VTK_LIBRARIES}
)