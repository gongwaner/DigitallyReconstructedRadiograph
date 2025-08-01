cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

project(DigitallyReconstructedRadiograph)

find_package(VTK COMPONENTS
        CommonColor
        CommonCore
        CommonDataModel
        IOGeometry
        IOImage
        IOPLY
        InteractionImage
        InteractionStyle
        InteractionWidgets
        RenderingContextOpenGL2
        RenderingCore
        RenderingFreeType
        RenderingGL2PSOpenGL2
        RenderingOpenGL2
)

if (NOT VTK_FOUND)
    message(FATAL_ERROR "DRR: Unable to find the VTK build folder.")
endif ()

# Prevent a "command line is too long" failure in Windows.
set(CMAKE_NINJA_FORCE_RESPONSE_FILE "ON" CACHE BOOL "Force Ninja to use response files.")

# Add the submodule as a subdirectory
add_subdirectory(CommonUtility)

set(SOURCES_FILES
        src/main.cpp
        src/Algorithm/VolumeDRR.cpp
        src/Algorithm/VolumeRayCastHelper.cpp
        src/Algorithm/MeshDRR.cpp
)

add_executable(${PROJECT_NAME} MACOSX_BUNDLE
        ${SOURCES_FILES}
)

target_compile_definitions(${PROJECT_NAME}
        PRIVATE
        CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ${VTK_LIBRARIES}
        CommonUtility
)

# vtk_module_autoinit is needed
vtk_module_autoinit(
        TARGETS ${PROJECT_NAME}
        MODULES ${VTK_LIBRARIES}
)
