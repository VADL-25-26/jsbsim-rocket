cmake_minimum_required(VERSION 3.12)
project(RocketSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_HITL_SIMULATIONS
    "Build the Linux/serial hardware-in-the-loop programs"
    OFF
)

#-----------------------------
# JSBSim library configuration
#-----------------------------
set(JSBSIM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(JSBSIM_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")

find_library(JSBSIM_LIBRARY
    NAMES JSBSim
    PATHS "${JSBSIM_LIBRARY_DIR}"
)

if(NOT JSBSIM_LIBRARY)
    message(FATAL_ERROR "JSBSim library not found. Build JSBSim first.")
endif()

message(STATUS "JSBSim library: ${JSBSIM_LIBRARY}")
message(STATUS "JSBSim include dir: ${JSBSIM_INCLUDE_DIR}")

function(add_jsbsim_program target_name)
    add_executable(${target_name} ${ARGN})
    target_include_directories(${target_name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${JSBSIM_INCLUDE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/legacy_files"
    )
    target_link_libraries(${target_name} PRIVATE "${JSBSIM_LIBRARY}")
endfunction()

# -----------------------------
# Build the program under active development in src/
# -----------------------------
add_jsbsim_program(rocket_main
    src/main.cpp
    src/rocket_simulation.cpp
)

# -----------------------------
# Build rocket_sim_subscale
# -----------------------------
add_jsbsim_program(rocket_sim_subscale rocket_sim_subscale.cpp)

# -----------------------------
# Build rocket_sim_fullscale
# -----------------------------
add_jsbsim_program(rocket_sim_fullscale rocket_sim_fullscale.cpp)

# -----------------------------
# Build Midscale sim
# -----------------------------
add_jsbsim_program(rocket_sim_midscale rocket_sim_midscale.cpp)

# -----------------------------
# Build Summer Subscale sim
# -----------------------------
add_jsbsim_program(rocket_sim_summer_subscale
    rocket_sim_summer_subscale_2026.cpp
)

# -----------------------------
# Build HIL programs only when explicitly requested. They require standalone
# Asio and Linux serial headers that are not needed for the normal simulations.
# -----------------------------
set(HITL_TARGETS)
if(BUILD_HITL_SIMULATIONS)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
        message(FATAL_ERROR "HIL simulations currently require Linux serial headers.")
    endif()

    find_path(ASIO_INCLUDE_DIR asio.hpp
        HINTS "$ENV{ASIO_ROOT}"
        PATH_SUFFIXES include
    )
    if(NOT ASIO_INCLUDE_DIR)
        message(FATAL_ERROR
            "Standalone Asio was not found. Set ASIO_ROOT or install Asio."
        )
    endif()

    add_jsbsim_program(hitl_sim_fullscale
        legacy_files/hitl_sim_fullscale.cpp
    )
    add_jsbsim_program(hitl_sim_midscale
        legacy_files/hitl_sim_midscale.cpp
    )
    add_jsbsim_program(hitl_sim_summer_subscale
        hitl_sim_summer_subscale_2026.cpp
    )

    set(HITL_TARGETS
        hitl_sim_fullscale
        hitl_sim_midscale
        hitl_sim_summer_subscale
    )
    foreach(target_name IN LISTS HITL_TARGETS)
        target_include_directories(${target_name} PRIVATE "${ASIO_INCLUDE_DIR}")
        target_compile_definitions(${target_name} PRIVATE ASIO_STANDALONE)
    endforeach()
endif()

# -----------------------------
# Copy aircraft config
# -----------------------------
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/aircraft
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# -----------------------------
# Runtime path for shared libraries
# -----------------------------
if(UNIX AND NOT APPLE)
    set(JSBSIM_RUNTIME_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/jsbsim/build/src")
    foreach(target_name
        rocket_main
        rocket_sim_subscale
        rocket_sim_fullscale
        rocket_sim_midscale
        rocket_sim_summer_subscale
        ${HITL_TARGETS}
    )
        set_target_properties(${target_name} PROPERTIES
            BUILD_RPATH "${JSBSIM_RUNTIME_LIBRARY_DIR};${JSBSIM_LIBRARY_DIR}"
            INSTALL_RPATH "${JSBSIM_RUNTIME_LIBRARY_DIR};${JSBSIM_LIBRARY_DIR}"
            INSTALL_RPATH_USE_LINK_PATH TRUE
        )
    endforeach()
endif()
