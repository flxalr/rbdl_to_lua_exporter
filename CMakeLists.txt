cmake_minimum_required(VERSION 3.15)
project(rbdl_urdf_lua CXX)

set(CMAKE_CXX_STANDARD 11)

#for FindRBDL.cmake
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

FIND_PACKAGE(RBDL REQUIRED)
FIND_PACKAGE(Eigen3 3.3 REQUIRED)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} )


add_executable(rbdl_urdf_lua main.cpp Model_builder.cpp)

target_precompile_headers(rbdl_urdf_lua PRIVATE Model_builder.h)

SET(RBDL_LIBRARY "/home/al3xst/proggen/rbdl-orb/cmake-build-release/librbdl.so")
SET(RBDL_URDFREADER_LIBRARY "/home/al3xst/proggen/rbdl-orb/cmake-build-release/addons/urdfreader/librbdl_urdfreader.so")

message("rbdl_lib: \t\t" ${RBDL_LIBRARY})
message("rbdl_lua_lib: \t" ${RBDL_LUAMODEL_LIBRARY})
message("rbdl_urdf_lib: \t" ${RBDL_URDFREADER_LIBRARY})

TARGET_LINK_LIBRARIES ( rbdl_urdf_lua
        Eigen3::Eigen
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
        ${RBDL_LUAMODEL_LIBRARY} )