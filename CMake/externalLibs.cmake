include(FetchContent)

#add our glad first, libigl is then gonna use it
if(NOT TARGET glad)
message(STATUS "Third-party: creating target 'glad'")
	FetchContent_Declare(
	  glad
	  GIT_REPOSITORY https://github.com/libigl/libigl-glad.git
	)
	FetchContent_MakeAvailable(glad)
	add_library(glad::glad ALIAS glad)
endif()

if(NOT TARGET Eigen3::Eigen)
	include(CMake/eigen.cmake)
endif()

#igl
if(NOT TARGET igl)
message(STATUS "Third-party: creating target 'igl'")
	FetchContent_Declare(
	  libigl
	  GIT_REPOSITORY https://github.com/libigl/libigl.git
	  GIT_TAG  v2.5.0
	)
	
	set(LIBIGL_PREDICATES ON CACHE BOOL "Use exact predicates" FORCE)
	option(LIBIGL_GLFW "Build target igl::glfw" ON)
	option(LIBIGL_IMGUI "Build target igl::imgui" ON)
	option(LIBIGL_OPENGL "Build target igl::opengl" ON)
	option(LIBIGL_STB "Build target igl::stb" ON)
	FetchContent_MakeAvailable(libigl)
endif()

# spdlog
if(NOT TARGET spdlog)
message(STATUS "Third-party: creating target 'spdlog'")
  FetchContent_Declare(
	  spdlog
	  GIT_REPOSITORY https://github.com/gabime/spdlog.git
      GIT_TAG        v1.10.0
  )
  
  FetchContent_MakeAvailable(spdlog)
  add_library(spdlog::spdlog ALIAS spdlog)
endif()


# CLI11
if(NOT TARGET CLI11::CLI11)
message(STATUS "Third-party: creating target 'CLI11'")
  FetchContent_Declare(
	  cli11
	  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
	  GIT_TAG  b979d3a37098da7ad6291d9bd3c4fdec3a705043
	)
	FetchContent_MakeAvailable(cli11)
endif()

if(NOT TARGET nlohmann_json::nlohmann_json)
message(STATUS "Third-party: creating target 'nlohmann_json'")
  FetchContent_Declare(
	  nlohmann_json
	  GIT_REPOSITORY https://github.com/nlohmann/json.git
	  GIT_TAG  v3.11.3
	)
	FetchContent_MakeAvailable(nlohmann_json)
	add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
endif()

if(NOT TARGET tinyobjloader)
message(STATUS "Third-party: creating target 'tinyobjloader'")
  FetchContent_Declare(
	  tinyobjloader
	  GIT_REPOSITORY https://github.com/tinyobjloader/tinyobjloader.git
	  GIT_TAG  v2.0.0rc13
	)
	FetchContent_MakeAvailable(tinyobjloader)
endif()

#include ipc toolkit

if(NOT TARGET ipc_toolkit)
	message(STATUS "Third-party: creating target 'ipc::toolkit'")
	FetchContent_Declare(
    ipc_toolkit
    GIT_REPOSITORY https://github.com/ipc-sim/ipc-toolkit.git
    GIT_TAG v1.2.0
    GIT_SHALLOW FALSE
	)
	FetchContent_MakeAvailable(ipc_toolkit)
endif()

if(NOT TARGET Spectra)
	message(STATUS "Third-party: creating target 'Spectra'")
	FetchContent_Declare(
		Spectra
	GIT_REPOSITORY https://github.com/yixuan/spectra.git
	GIT_TAG v1.0.1
	)
	FetchContent_MakeAvailable(Spectra)

	add_library(Spectra::Spectra ALIAS Spectra)
	
endif()
