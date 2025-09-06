cmake_minimum_required(VERSION 3.16.0)

# Set project version manually to avoid git dependency issues
set(PROJECT_VER "1.0.0")
set(IDF_VER "v5.5.0")

# Override ESP-IDF git functions with empty implementations
macro(__build_get_idf_git_revision)
    # Do nothing - disable git revision detection
endmacro()

macro(git_describe VAR)
    set(${VAR} "1.0.0")
endmacro()

# Set environment variables before including ESP-IDF
set(ENV{PROJECT_VER} "1.0.0")
set(ENV{IDF_VER} "v5.5.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(testtesttest VERSION 1.0.0)
