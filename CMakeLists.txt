cmake_minimum_required(VERSION 3.28)
project(bridging_mvh_dr)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options log thread filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

# Enable profiling
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include_directories("include")

# All library sources, excluding the program entry point
file(GLOB_RECURSE LIB_SRC_FILES "src/*.cpp" "src/**/*.cpp" "include/**/*.h" "include/*.h")
list(REMOVE_ITEM LIB_SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

# Main executable
add_executable(MultivaluedHeuristicSearch ${LIB_SRC_FILES} src/main.cpp)
target_link_libraries(MultivaluedHeuristicSearch ${Boost_LIBRARIES})

# Solver comparison tests on small fixtures.
add_executable(BoaComparisonTests ${LIB_SRC_FILES} tests/compare_against_boa.cpp)
target_link_libraries(BoaComparisonTests ${Boost_LIBRARIES})

add_executable(NamoaDr3ObjectiveComparisonTests ${LIB_SRC_FILES} tests/compare_against_namoa_dr_3obj.cpp)
target_link_libraries(NamoaDr3ObjectiveComparisonTests ${Boost_LIBRARIES})

add_executable(ApexMvhMultiObjectiveTests ${LIB_SRC_FILES} tests/compare_apex_mvh_multiobj.cpp)
target_link_libraries(ApexMvhMultiObjectiveTests ${Boost_LIBRARIES})

enable_testing()

add_test(
    NAME smoke_synthetic_graph
    COMMAND MultivaluedHeuristicSearch
        --map ${CMAKE_CURRENT_SOURCE_DIR}/resources/synthetic_graph
        --start 1
        --goal 20
        --algorithm NAMOA_DR
        --objectives 0 1 2 3 4
        --cutoffTime 5
)
set_tests_properties(smoke_synthetic_graph PROPERTIES
    PASS_REGULAR_EXPRESSION "algorithm=NAMOA_DR"
)

add_test(
    NAME synthetic_graph_boa_comparisons
    COMMAND BoaComparisonTests ${CMAKE_CURRENT_SOURCE_DIR}/resources/synthetic_graph
)

add_test(
    NAME synthetic_graph_namoa_dr_3obj_comparisons
    COMMAND NamoaDr3ObjectiveComparisonTests ${CMAKE_CURRENT_SOURCE_DIR}/resources/synthetic_graph
)

add_test(
    NAME synthetic_graph_apex_mvh_multiobj
    COMMAND ApexMvhMultiObjectiveTests ${CMAKE_CURRENT_SOURCE_DIR}/resources/synthetic_graph
)
