cmake_minimum_required(VERSION 3.8)
project(aruco_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# The dog3 ROS image was built in a sysroot and exports that build-time Python
# library path from several imported message targets. At runtime the same
# library is available under /usr/lib. Rewrite only the missing exported path
# so custom Python messages can be linked without changing the system install.
set(_exported_python_library
	"/sysroot/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/libpython3.10.so")
set(_runtime_python_library
	"/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/libpython3.10.so")
if(NOT EXISTS "${_exported_python_library}"
	AND EXISTS "${_runtime_python_library}")
	get_property(_imported_targets DIRECTORY PROPERTY IMPORTED_TARGETS)
	foreach(_target IN LISTS _imported_targets)
		get_target_property(_interface_libraries
			"${_target}" INTERFACE_LINK_LIBRARIES)
		if(_interface_libraries
			AND NOT _interface_libraries MATCHES "-NOTFOUND$")
			string(REPLACE
				"${_exported_python_library}"
				"${_runtime_python_library}"
				_interface_libraries
				"${_interface_libraries}")
			set_property(TARGET "${_target}" PROPERTY
				INTERFACE_LINK_LIBRARIES "${_interface_libraries}")
		endif()
	endforeach()
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/ArucoMarkers.msg"
	DEPENDENCIES geometry_msgs
	DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)

	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	# set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	# set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
