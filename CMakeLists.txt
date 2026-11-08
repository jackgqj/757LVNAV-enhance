cmake_minimum_required(VERSION 3.20)
project(757LVNAVPlus VERSION 1.2.36 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep the Windows plugin self-contained.  X-Plane calls only C entry points,
# so a static CRT does not cross a C++ ABI boundary.
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

set(XPLANE_SDK_DIR "" CACHE PATH "Path to the X-Plane SDK (xpsdk) checkout")

if(NOT XPLANE_SDK_DIR)
    include(FetchContent)
    FetchContent_Declare(
        xpsdk
        GIT_REPOSITORY https://github.com/X-Plane/xpsdk.git
        GIT_TAG master
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(xpsdk)
    set(XPLANE_SDK_DIR "${xpsdk_SOURCE_DIR}")
endif()

if(NOT EXISTS "${XPLANE_SDK_DIR}/CHeaders/XPLM/XPLMPlugin.h")
    message(FATAL_ERROR
        "X-Plane SDK not found at '${XPLANE_SDK_DIR}'. "
        "Set -DXPLANE_SDK_DIR=/path/to/xpsdk or let CMake fetch it from GitHub.")
endif()

set(XPLM_INCLUDE_DIRS
    "${XPLANE_SDK_DIR}/CHeaders/XPLM"
    "${XPLANE_SDK_DIR}/CHeaders/Widgets"
)

set(LVNV_SOURCES
    src/plugin.cpp
    src/config.cpp
    src/route_reader.cpp
    src/guidance.cpp
    src/cdu_parser.cpp
    src/cdu_reader.cpp
    src/nav_database.cpp
    src/cifp_parser.cpp
)

add_library(757LVNAVPlus MODULE ${LVNV_SOURCES})
target_include_directories(757LVNAVPlus PRIVATE
    ${XPLM_INCLUDE_DIRS}
    src
)
target_compile_definitions(757LVNAVPlus PRIVATE
    XPLM200=1
    XPLM210=1
    XPLM300=1
    XPLM301=1
    XPLM303=1
)

if(MSVC)
    target_compile_options(757LVNAVPlus PRIVATE /W4 /permissive-)
else()
    target_compile_options(757LVNAVPlus PRIVATE -Wall -Wextra)
endif()

set(LVNV_PLATFORM_NAME "")
if(WIN32)
    set(LVNV_PLATFORM_NAME "win")
    target_compile_definitions(757LVNAVPlus PRIVATE IBM=1)
    target_link_libraries(757LVNAVPlus PRIVATE
        "${XPLANE_SDK_DIR}/Libraries/Win/XPLM_64.lib")
elseif(APPLE)
    set(LVNV_PLATFORM_NAME "mac")
    target_compile_definitions(757LVNAVPlus PRIVATE APL=1)
    set(CMAKE_FRAMEWORK_PATH "${XPLANE_SDK_DIR}/Libraries/Mac")
    target_link_libraries(757LVNAVPlus PRIVATE
        "-framework XPLM"
        "-framework XPWidgets")
    target_link_options(757LVNAVPlus PRIVATE
        "-F${XPLANE_SDK_DIR}/Libraries/Mac"
        "-Wl,-undefined,dynamic_lookup")
else()
    set(LVNV_PLATFORM_NAME "lin")
    target_compile_definitions(757LVNAVPlus PRIVATE LIN=1)
endif()

set_target_properties(757LVNAVPlus PROPERTIES
    PREFIX ""
    SUFFIX ".xpl"
    OUTPUT_NAME "${LVNV_PLATFORM_NAME}"
    # The $<0:> generator expression prevents multi-config generators from
    # appending a per-config subdirectory, so CI always finds the binary at
    # build/dist/64/<platform>.xpl.
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist/64/$<0:>"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist/64/$<0:>"
)

include(CTest)
if(BUILD_TESTING)
    add_executable(757lvnv_tests
        tests/guidance_tests.cpp
        src/guidance.cpp
        src/config.cpp
    )
    target_include_directories(757lvnv_tests PRIVATE src)
    if(MSVC)
        target_compile_options(757lvnv_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(757lvnv_tests PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME 757lvnv_guidance COMMAND 757lvnv_tests)

    add_executable(757lvnv_cdu_tests
        tests/cdu_tests.cpp
        src/cdu_parser.cpp
    )
    target_include_directories(757lvnv_cdu_tests PRIVATE src)
    if(MSVC)
        target_compile_options(757lvnv_cdu_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(757lvnv_cdu_tests PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME 757lvnv_cdu COMMAND 757lvnv_cdu_tests)

    add_executable(757lvnv_cifp_tests
        tests/cifp_tests.cpp
        src/cifp_parser.cpp
        src/nav_database.cpp
    )
    target_include_directories(757lvnv_cifp_tests PRIVATE src)
    if(MSVC)
        target_compile_options(757lvnv_cifp_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(757lvnv_cifp_tests PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME 757lvnv_cifp COMMAND 757lvnv_cifp_tests)
endif()

install(TARGETS 757LVNAVPlus
    LIBRARY DESTINATION "64"
    RUNTIME DESTINATION "64"
)
install(FILES
    README.md
    LICENSE
    CHANGELOG.md
    THIRD_PARTY_NOTICES.md
    NOTICE.md
    DESTINATION "."
)
install(FILES
    docs/DATAREFS.md
    docs/CONFIG.md
    docs/AIRCRAFT_STRUCTURE.md
    docs/USAGE.md
    DESTINATION "docs"
)
install(FILES
    licenses/libcxx-LICENSE.txt
    DESTINATION "licenses"
)
