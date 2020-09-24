# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(test_data-download NONE)

include(ExternalProject)
ExternalProject_Add(test_data-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/libigl-tests-data;GIT_TAG;b5dddf45e329af685cd107e38770a28cfc18eb15
                    SOURCE_DIR          "/usr/include/libigl/cmake/../external/../tests/data"
                    BINARY_DIR          "/usr/include/libigl/build/test_data-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
