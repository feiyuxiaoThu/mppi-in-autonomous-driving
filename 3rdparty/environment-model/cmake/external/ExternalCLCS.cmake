include(FetchContent)

# we explicitly set build s11n to ON since it is deactivated by default in the CMake file of the CLCS if it is not top level
set(CR_CLCS_BUILD_S11N ON)

FetchContent_Declare(
        CommonRoadCLCS
        SYSTEM
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../external/commonroad-clcs
)

FetchContent_MakeAvailable(CommonRoadCLCS)

set_property(DIRECTORY ${CommonRoadCLCS_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
        CR_CLCS_BUILD_S11N
)
