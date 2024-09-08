cmake_minimum_required(VERSION 3.17.0)
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake") 


include(CMake/externalLibs.cmake)
include(CMake/eigen.cmake)

project(SimLib_Core)
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src_core")
file(GLOB_RECURSE source_files
	${PROJECT_SRC_DIR}/*.h
	${PROJECT_SRC_DIR}/*.cpp
	)
add_library(${PROJECT_NAME} ${source_files})

target_precompile_headers(${PROJECT_NAME}
  PRIVATE
  "${PROJECT_SRC_DIR}/SimLib/Precompiled.h"
)

#mkl
if(NOT APPLE)
	#oneapi mkl
	list(APPEND CMAKE_PREFIX_PATH
		"$ENV{MKL_ROOT}/lib/cmake/mkl")
	SET(MKL_INTERFACE lp64)
	find_package(MKL CONFIG REQUIRED)
endif()
# Find OpenMP
find_package(OpenMP REQUIRED)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
		Eigen3::Eigen
		nlohmann_json::nlohmann_json
		OpenMP::OpenMP_CXX
		igl::core
		igl::opengl
		igl::glfw
		igl::imgui
		igl::stb
		ipc::toolkit
		glad::glad
		Spectra::Spectra
		tinyobjloader
		TBB::tbb
		CLI11::CLI11
		spdlog::spdlog
	)

if(NOT APPLE)
	target_link_libraries(${PROJECT_NAME}
	PUBLIC
	MKL::MKL)
	
	target_compile_definitions(${PROJECT_NAME} PUBLIC "SIM_OPT_WITH_INTEL_MKL")
endif(NOT APPLE)

target_compile_definitions(${PROJECT_NAME} PUBLIC "SIM_OPT_RES_FOLDER=\"${CMAKE_CURRENT_LIST_DIR}/res/\"")
target_compile_definitions(${PROJECT_NAME} PUBLIC "SIM_OPT_SHADER_FOLDER=\"${CMAKE_CURRENT_LIST_DIR}/res/shader/\"")


source_group(TREE ${PROJECT_SRC_DIR} FILES ${source_files})


project(SimLib)

set(PROJECT_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")

file(GLOB_RECURSE source_files
	${PROJECT_SRC_DIR}/*.h
	${PROJECT_SRC_DIR}/*.cpp
	)

#put lib into the project
add_library(${PROJECT_NAME} ${source_files})
target_link_libraries(${PROJECT_NAME} SimLib_Core)
add_library(SimLib::core ALIAS SimLib)