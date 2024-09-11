cmake_minimum_required(VERSION 3.5)

project(viewer VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)


set(VTK_DIR "/usr/local/lib/cmake/vtk-6.3")
find_package(VTK REQUIRED)
find_package(calib_plugin REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

add_executable(viewer
    ${PROJECT_SOURCES}
)

target_include_directories(viewer PUBLIC
    ${VTK_INCLUDE_DIRS}
    ${calib_plugin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(viewer PRIVATE
    Qt5::Widgets
    ${VTK_LIBRARIES}
    calib_plugin
    yaml-cpp
    ${EIGEN3_LIBRARY_DIRS}
    ${PCL_LIBRARIES}
    )

set_target_properties(viewer PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(viewer)
endif()

#install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml DESTINATION ${CMAKE_BINARY_DIR})
add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_BINARY_DIR}
)
