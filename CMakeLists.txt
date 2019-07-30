cmake_minimum_required (VERSION 3.0.0)
project (G4TURTLE
    LANGUAGES CXX
    VERSION 0.7
)

include (ExternalProject)


if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING
                "Choose the type of build, options are: Debug Release
                 RelWithDebInfo MinSizeRel." FORCE)
endif ()

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set (CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH
                "Install path prefix." FORCE)
endif ()

set (CMAKE_MACOSX_RPATH 1)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")


# Geant4 includes
find_package (Geant4 REQUIRED)
include (${Geant4_USE_FILE})
include_directories (${Geant4_INCLUDE_DIR})


# Configure or install TURTLE
set (TURTLE_PREFIX "" CACHE STRING "Installation prefix of TURTLE")
if (TURTLE_PREFIX STREQUAL "")
        # INSTALL the TURTLE library
        message ("PREFIX=${CMAKE_INSTALL_PREFIX}")
        ExternalProject_Add (TURTLE
            URL https://github.com/niess/turtle/archive/v${G4TURTLE_VERSION}.tar.gz
            CONFIGURE_COMMAND cmake ../TURTLE -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
            BUILD_COMMAND make
            INSTALL_COMMAND make install
        )
        link_directories (${CMAKE_INSTALL_PREFIX}/lib)
        include_directories (${CMAKE_INSTALL_PREFIX}/include)
else ()
        link_directories (${TURTLE_PREFIX}/lib)
        include_directories (${TURTLE_PREFIX}/include)
endif ()


# G4Turtle wrapper
add_library (G4turtle SHARED src/G4Turtle.cc src/G4Turtle.hh)
set_target_properties (G4turtle PROPERTIES VERSION ${G4TURTLE_VERSION})
target_link_libraries (G4turtle turtle)
if (TURTLE_PREFIX STREQUAL "")
        add_dependencies (G4turtle TURTLE)
endif ()

install (TARGETS G4turtle DESTINATION lib)
install (FILES src/G4Turtle.hh DESTINATION include)


# Test executable
add_executable(run-g4turtle EXCLUDE_FROM_ALL test/run.cc test/run.hh)
target_link_libraries (run-g4turtle G4turtle ${Geant4_LIBRARIES})
