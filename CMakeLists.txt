# Copyright 2012-2015 Free Software Foundation, Inc.
# Copyright 2015-2016 Josh Blum
#
# This file is part of SoapyUHD support modules
#
# SoapyUHD is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# SoapyUHD is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8.12...3.10)
project(SoapyUHD CXX C)
enable_testing()

set(CMAKE_CXX_STANDARD 14)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

########################################################################
# Setup SoapySDR and set default modules location
########################################################################
find_package(SoapySDR "0.6" NO_MODULE REQUIRED)

if (NOT SOAPYSDR_ROOT)
    get_filename_component(SOAPYSDR_ROOT "${SoapySDR_INCLUDE_DIRS}/.." ABSOLUTE)
endif()
message(STATUS "SoapySDR root directory: ${SOAPYSDR_ROOT}")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set_property(CACHE CMAKE_INSTALL_PREFIX PROPERTY VALUE ${SOAPYSDR_ROOT})
endif()

########################################################################
# Dependencies
########################################################################
find_package(UHD NO_MODULE)

#try old-style find script
if (NOT UHD_FOUND)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(UHD)
endif()

if (NOT UHD_FOUND)
    message(FATAL_ERROR "UHD not found -- required for Soapy UHD support")
endif()

if (NOT UHD_ROOT)
    get_filename_component(UHD_ROOT "${UHD_INCLUDE_DIRS}/.." ABSOLUTE)
endif()
message(STATUS "UHD root directory: ${UHD_ROOT}")
message(STATUS "UHD include directories: ${UHD_INCLUDE_DIRS}")
message(STATUS "UHD libraries: ${UHD_LIBRARIES}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${SoapySDR_INCLUDE_DIRS})
include_directories(${UHD_INCLUDE_DIRS})

message(STATUS "Checking uhd::device::register_device() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/device.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/device.hpp device_hpp)
string(FIND "${device_hpp}" "device_filter_t" has_device_filter)
if ("${has_device_filter}" STREQUAL "-1")
    message(STATUS "  has original API")
else()
    add_definitions(-DUHD_HAS_DEVICE_FILTER)
    message(STATUS "  has filter API")
endif()

message(STATUS "Checking uhd::usrp::multi_usrp::set_rx_agc() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp multi_usrp_hpp)
string(FIND "${multi_usrp_hpp}" "set_rx_agc" has_set_rx_agc)
if ("${has_set_rx_agc}" STREQUAL "-1")
    message(STATUS "  missing set_rx_agc() API")
else()
    add_definitions(-DUHD_HAS_SET_RX_AGC)
    message(STATUS "  has set_rx_agc() API")
endif()

message(STATUS "Checking uhd::property::set_publisher() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp property_tree_hpp)
string(FIND "${property_tree_hpp}" "set_publisher" has_set_publisher)
if ("${has_set_publisher}" STREQUAL "-1")
    message(STATUS "  missing set_publisher() API")
else()
    add_definitions(-DUHD_HAS_SET_PUBLISHER)
    message(STATUS "  has set_publisher() API")
endif()

if (EXISTS "${UHD_INCLUDE_DIRS}/uhd/utils/msg.hpp")
    add_definitions(-DUHD_HAS_MSG_HPP)
    message(STATUS "  use msg.hpp for logging")
else()
    message(STATUS "  use log.hpp for logging")
endif()

########################################################################
# Setup Boost
########################################################################
MESSAGE(STATUS "Configuring Boost C++ Libraries...")

if(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
endif(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")

find_package(Boost)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Boost not found -- required for Soapy UHD support")
endif()

ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

message(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost library directories: ${Boost_LIBRARY_DIRS}")
message(STATUS "Boost libraries: ${Boost_LIBRARIES}")

########################################################################
# Build a Soapy module to support UHD devices
########################################################################
SOAPY_SDR_MODULE_UTIL(
    TARGET uhdSupport
    SOURCES SoapyUHDDevice.cpp
    LIBRARIES
        ${UHD_LIBRARIES}
        ${Boost_LIBRARIES}
)

########################################################################
# Build a UHD module to support Soapy devices
########################################################################
add_library(soapySupport MODULE UHDSoapyDevice.cpp)
target_link_libraries(soapySupport
    ${UHD_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    ${Boost_LIBRARIES})
install(TARGETS soapySupport
    DESTINATION ${UHD_ROOT}/lib${LIB_SUFFIX}/uhd/modules
)

########################################################################
# rpath setup - http://www.cmake.org/Wiki/CMake_RPATH_handling
########################################################################
# use, i.e. don't skip the full RPATH for the build tree
option(CMAKE_SKIP_BUILD_RPATH "skip rpath build" FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
option(CMAKE_BUILD_WITH_INSTALL_RPATH "build with install rpath" FALSE)

# the RPATH to be used when installing, but only if it's not a system directory
option(CMAKE_AUTOSET_INSTALL_RPATH TRUE)
if(CMAKE_AUTOSET_INSTALL_RPATH)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")
ENDIF("${isSystemDir}" STREQUAL "-1")
endif(CMAKE_AUTOSET_INSTALL_RPATH)

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
option(CMAKE_INSTALL_RPATH_USE_LINK_PATH "build with automatic rpath" TRUE)

if(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
endif()

########################################################################
# Print Summary
########################################################################
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")

########################################################################
# uninstall target
########################################################################
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

