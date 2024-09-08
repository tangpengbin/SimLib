
if(MSVC)
	#message(WARNING "Adding openmp and multi threaded compilation as default flags")
	set(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} /openmp /MP")
	set(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} /openmp /MP")
endif()
