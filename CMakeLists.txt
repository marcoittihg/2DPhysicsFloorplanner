cmake_minimum_required(VERSION 3.9)
project(BubbleRegionsFloorplanner)

set(CMAKE_CXX_STANDARD 11)

add_executable(BubbleRegionsFloorplanner main.cpp BaseUtils/Point2D.cpp BaseUtils/Point2D.h Data/FPGAData/Block.h Data/FPGAData/Board.cpp Data/FPGAData/Board.h Data/ProblemData/Problem.cpp Data/ProblemData/Problem.h Data/ProblemData/ProblemRegion.cpp Data/ProblemData/ProblemRegion.h Data/ProblemData/RegionIOData.cpp Data/ProblemData/RegionIOData.h Data/ProblemData/RegionType.h FileManagement/FileManager.cpp FileManagement/FileManager.h Solving/Phyisics/Physics.cpp Solving/Phyisics/Physics.h Solving/Render.cpp Solving/Render.h Solving/PhysicsRegion.cpp Solving/PhysicsRegion.h BaseUtils/Vector2.cpp BaseUtils/Vector2.h Solving/Phyisics/Rigidbody.cpp Solving/Phyisics/Rigidbody.h Solving/FeasiblePlacement.cpp Solving/FeasiblePlacement.h Solving/FeasiblePlacementState.h Solving/MainLoopManager.cpp Solving/MainLoopManager.h Solving/PhysicsRegionState.h Solving/PhysicsRegionType.h Solving/FloorplanningManager.cpp Solving/FloorplanningManager.h Solving/FloortplanningMangerState.h)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(glfw-3.2.1)

target_link_libraries(BubbleRegionsFloorplanner glfw)
find_package(OpenGL REQUIRED)
target_include_directories(BubbleRegionsFloorplanner PUBLIC ${OPENGL_INCLUDE_DIR})
target_link_libraries(BubbleRegionsFloorplanner ${OPENGL_gl_LIBRARY})
