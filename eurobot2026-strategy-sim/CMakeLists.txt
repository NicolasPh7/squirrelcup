cmake_minimum_required(VERSION 3.16)
project(eurobot2026-strategy-sim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SFML_STATIC_LIBRARIES FALSE)  # garde FALSE pour du dynamique
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(ai_decision_demo "src/ai_decision_demo.cpp")
target_link_libraries(ai_decision_demo PRIVATE sfml-graphics sfml-window sfml-system)
