cmake_minimum_required(VERSION 3.16.3)

project( TaskPool VERSION 1.0.1)

option(BUILD_SHARED_LIBS "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)" ON)

include_directories(
    include
)

add_library( taskpool
    TaskPool.cc
)

set_target_properties(taskpool PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(taskpool PROPERTIES SOVERSION 1 )

target_include_directories(taskpool PUBLIC include)

include(GNUInstallDirs)

install(TARGETS taskpool
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
