# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(predicates-download NONE)

include(ExternalProject)
ExternalProject_Add(predicates-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/libigl-predicates.git;GIT_TAG;5a1d2194ec114bff51d5a33230586cafb83adc86
                    SOURCE_DIR          "/usr/include/libigl/cmake/../external/predicates"
                    BINARY_DIR          "/usr/include/libigl/build/predicates-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
