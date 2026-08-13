add_library(odrive_lib_cpp STATIC
    odrive_lib_cpp.cpp
)

target_include_directories(odrive_lib_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(odrive_lib_cpp PUBLIC serialworker)
