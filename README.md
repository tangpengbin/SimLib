# SimLib
This is a base library for simulation and visualization. 

# Build
Our code can be built on Windows, MacOS, Unbuntu systems via CMake by adding the following lines in your CMake file.
```
FetchContent_Declare(
	SimLib
	GIT_REPOSITORY https://github.com/tangpengbin/SimLib.git
	GIT_TAG main
)
FetchContent_MakeAvailable(SimLib)
```

After that, you can link the library to your project.
```
target_link_libraries(${PROJECT_NAME}
		PUBLIC
		SimLib
)
```

# Citation

```
@software{SimLib,
  title = {SimLib},
  author = {Pengbin Tang, Jonas Zehnder, Bernhard Thomaszewski},
  url = {https://github.com/tangpengbin/SimLib},
  year = {2024}
}
```