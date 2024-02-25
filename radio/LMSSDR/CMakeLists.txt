# find_package(LimeSuite2 REQUIRED)

add_library(oai_lmssdrdevif MODULE lms_lib.cpp common.cpp)
target_link_libraries(oai_lmssdrdevif LimeSuite2 )
set_target_properties(oai_lmssdrdevif PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
add_custom_command(TARGET oai_lmssdrdevif POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink liboai_lmssdrdevif.so liboai_device.so
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
