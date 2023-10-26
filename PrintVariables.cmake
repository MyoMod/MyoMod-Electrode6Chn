function( print_variables)
    message( STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
    message( STATUS "CMAKE_CXX_COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")
    message( STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")

    message( STATUS "The CMake Source Dir: ${CMAKE_SOURCE_DIR}")
    message( STATUS "The CMake Binary Dir: ${CMAKE_BINARY_DIR}")
    message( STATUS "The CMake Current Source Dir: ${CMAKE_CURRENT_SOURCE_DIR}")
    message( STATUS "The CMake Current Binary Dir: ${CMAKE_CURRENT_BINARY_DIR}")
    
endfunction( print_variables)
