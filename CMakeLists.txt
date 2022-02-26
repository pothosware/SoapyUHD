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
cmake_minimum_required(VERSION 2.8.7)
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
# Dependencies
########################################################################
find_package(SoapySDR "0.6" NO_MODULE REQUIRED)
find_package(UHD NO_MODULE)

#try old-style find script
if (NOT UHD_FOUND)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(UHD)
endif()

if (NOT UHD_FOUND)
    message(WARNING "UHD not found -- required for Soapy UHD support")
    return()
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

# UHD has a single "LO config" API define, but the TX LO config
# functions were added later, so it doesn't mean much. So we check
# here.
message(STATUS "Checking uhd::usrp::multi_usrp::get_tx_lo_names() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp multi_usrp_hpp)
string(FIND "${multi_usrp_hpp}" "get_tx_lo_names" has_tx_lo_config)
if ("${has_tx_lo_config}" STREQUAL "-1")
    message(STATUS "  missing TX LO config API")
else()
    add_definitions(-DUHD_HAS_TX_LO_CONFIG)
    message(STATUS "  has TX LO config API")
endif()

message(STATUS "Checking uhd::usrp::multi_usrp::set_clock_source_out() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp multi_usrp_hpp)
string(FIND "${multi_usrp_hpp}" "set_clock_source_out" has_sync_source_out)
if ("${has_sync_source_out}" STREQUAL "-1")
    message(STATUS "  missing sync source out API")
else()
    add_definitions(-DUHD_HAS_SYNC_SOURCE_OUT)
    message(STATUS "  has sync source out API")
endif()

message(STATUS "Checking uhd::usrp::multi_usrp::get_tx_gain_profile_names() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp multi_usrp_hpp)
string(FIND "${multi_usrp_hpp}" "get_tx_gain_profile_names" has_gain_profile)
if ("${has_gain_profile}" STREQUAL "-1")
    message(STATUS "  missing gain profile API")
else()
    add_definitions(-DUHD_HAS_GAIN_PROFILE)
    message(STATUS "  has gain profile API")
endif()

# UHD has an API define for this, but it only correponds to when the first half
# of the API was added...
message(STATUS "Checking uhd::usrp::multi_usrp::get_tx_power_range() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/usrp/multi_usrp.hpp multi_usrp_hpp)
string(FIND "${multi_usrp_hpp}" "get_tx_power_range" has_power_level)
if ("${has_power_level}" STREQUAL "-1")
    message(STATUS "  missing power level API")
else()
    add_definitions(-DUHD_HAS_POWER_LEVEL)
    message(STATUS "  has power level API")
endif()

########################################################################
# Setup boost
########################################################################
MESSAGE(STATUS "Configuring Boost C++ Libraries...")

# Although not required on my system, some users have linking issues without
SET(BOOST_REQUIRED_COMPONENTS
    thread
    system
)

if(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
endif(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")

set(Boost_ADDITIONAL_VERSIONS
    "1.35.0" "1.35" "1.36.0" "1.36" "1.37.0" "1.37" "1.38.0" "1.38" "1.39.0" "1.39"
    "1.40.0" "1.40" "1.41.0" "1.41" "1.42.0" "1.42" "1.43.0" "1.43" "1.44.0" "1.44"
    "1.45.0" "1.45" "1.46.0" "1.46" "1.47.0" "1.47" "1.48.0" "1.48" "1.49.0" "1.49"
    "1.50.0" "1.50" "1.51.0" "1.51" "1.52.0" "1.52" "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
)

find_package(Boost COMPONENTS ${BOOST_REQUIRED_COMPONENTS})

if(NOT Boost_FOUND)
    message(WARNING "Boost required to build -- ignoring uhd support")
    return()
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

