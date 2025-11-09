cmake_minimum_required(VERSION 3.16)
project(mujoco_delphi_wrapper LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_policy(SET CMP0135 NEW)
set(DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

# --- pas dit pad aan ---
set(MUJOCO_DIR "C:/Users/Roelof/Desktop/DESKTOP_2_NOV2025/MuJoCo_WRAPPER")

include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/bin)

# --- GLFW statisch binnenhalen ---
include(FetchContent)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw URL https://github.com/glfw/glfw/archive/refs/tags/3.4.zip)
FetchContent_MakeAvailable(glfw)

add_library(mujoco_delphi_wrapper SHARED mujoco_delphi_wrapper.cpp)
target_link_libraries(mujoco_delphi_wrapper
  "${MUJOCO_DIR}/bin/mujoco.lib"
  glfw opengl32 gdi32 user32 shell32)

add_custom_command(TARGET mujoco_delphi_wrapper POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${MUJOCO_DIR}/bin/mujoco.dll
          $<TARGET_FILE_DIR:mujoco_delphi_wrapper>)