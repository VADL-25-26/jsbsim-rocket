cmake_minimum_required(VERSION 3.12)
project(RocketSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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


# -----------------------------
# Build rocket_sim
# -----------------------------
add_executable(rocket_sim rocket_sim.cpp)
target_include_directories(rocket_sim PRIVATE 
${JSBSIM_INCLUDE_DIR}
"/home/altari/asio/asio/include"
)
target_compile_definitions(rocket_sim PRIVATE ASIO_STANDALONE)
target_link_libraries(rocket_sim 
${JSBSIM_LIBRARY}
)

# -----------------------------
# Build rocket_sim_subscale
# -----------------------------
add_executable(rocket_sim_subscale rocket_sim_subscale.cpp)
target_include_directories(rocket_sim_subscale PRIVATE 
${JSBSIM_INCLUDE_DIR}
"/home/altari/asio/asio/include"
)
target_compile_definitions(rocket_sim_subscale PRIVATE ASIO_STANDALONE)
target_link_libraries(rocket_sim_subscale 
${JSBSIM_LIBRARY}
)

# -----------------------------
# Build rocket_sim_fullscale
# -----------------------------
add_executable(rocket_sim_fullscale rocket_sim_fullscale.cpp)
target_include_directories(rocket_sim_fullscale PRIVATE 
${JSBSIM_INCLUDE_DIR}
"/home/altari/asio/asio/include"
)
target_compile_definitions(rocket_sim_fullscale PRIVATE ASIO_STANDALONE)
target_link_libraries(rocket_sim_fullscale 
${JSBSIM_LIBRARY}
)

# -----------------------------
# Copy aircraft config
# -----------------------------
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/aircraft
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# -----------------------------
# Runtime path for shared libraries
# -----------------------------
if(UNIX AND NOT APPLE)
    set_target_properties(rocket_sim PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
    set_target_properties(rocket_sim_subscale PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()
