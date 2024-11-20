cmake_minimum_required(VERSION 3.15)
project(pov0 VERSION 0.1)

set(FAST_TEST 1)

set(SKIP_TO_V0 0)
set(TEST_FIRST "")
set(SKIP_TO_FILE "")
set(TEST_EXTENDED 0)

if (NOT DEFINED ONLY_V0)
	set(TEST_SKIA 1)
	set(TEST_REGION 1)
else()
	set(TEST_SKIA 0)
	set(TEST_REGION 0)
endif()

configure_file(../tests/OpTestDrive.h.in ../tests/OpTestDrive.h)
configure_file(../tests/OpSkiaTests.h.in ../tests/OpSkiaTests.h)

# Add source to this project's executable.
add_executable (skiatest "skiatest.cpp" 
	"../debug/OpDebug.cpp"
	"../debug/OpDebugColor.cpp"
	"../debug/OpDebugCompare.cpp"
	"../debug/OpDebugDouble.cpp"
	"../debug/OpDebugDump.cpp"
	"../debug/OpDebugGenerator.cpp"
	"../debug/OpDebugImage.cpp"
	"../debug/OpDebugRecord.cpp"
	"../src/OpContour.cpp"
	"../src/OpCurve.cpp"
	"../src/OpCurveCurve.cpp"
	"../src/OpEdge.cpp"
	"../src/OpIntersection.cpp"
	"../src/OpJoiner.cpp"
	"../src/OpMath.cpp"
	"../src/OpSegment.cpp"
	"../src/OpSegments.cpp"
    "../src/OpWinder.cpp"
    "../src/OpWinding.cpp"
	"../skia/SkiaPaths.cpp"
    "../tests/OpSkiaTests.cpp"
    "../tests/OpV0Tests.cpp"
    "../tests/PathOpsBattles_shim.cpp"
    "../tests/PathOpsChalkboardTest_shim.cpp"
    "../tests/PathOpsFuzz763Test_shim.cpp"
    "../tests/PathOpsInverseTest_shim.cpp"
    "../tests/PathOpsIssue3651_shim.cpp"
    "../tests/PathOpsOpCircleThreadedTest_shim.cpp"
    "../tests/PathOpsOpCubicThreadedTest_shim.cpp"
    "../tests/PathOpsOpLoopThreadedTest_shim.cpp"
    "../tests/PathOpsOpRectThreadedTest_shim.cpp"
    "../tests/PathOpsOpTest_shim.cpp"
    "../tests/PathOpsSimplifyDegenerateThreadedTest_shim.cpp"
    "../tests/PathOpsSimplifyFailTest_shim.cpp"
    "../tests/PathOpsSimplifyQuadThreadedTest_shim.cpp"
    "../tests/PathOpsSimplifyQuadralateralsThreadedTest_shim.cpp"
    "../tests/PathOpsSimplifyRectThreadedTest_shim.cpp"
    "../tests/PathOpsSimplifyTest_shim.cpp"
    "../tests/PathOpsSimplifyTrianglesThreadedTest_shim.cpp"
    "../tests/PathOpsTigerTest_shim.cpp"
    "../tests/TinySkia.cpp"
    "../PathOps.cpp"
)

add_executable (tinytest "tinytest.cpp"
	"../debug/OpDebug.cpp"
	"../src/OpContour.cpp"
	"../src/OpCurve.cpp"
	"../src/OpCurveCurve.cpp"
	"../src/OpEdge.cpp"
	"../src/OpIntersection.cpp"
	"../src/OpJoiner.cpp"
	"../src/OpMath.cpp"
	"../src/OpSegment.cpp"
	"../src/OpSegments.cpp"
    "../src/OpWinder.cpp"
    "../src/OpWinding.cpp"
	"../tests/TestNewInterface.cpp"
    "../PathOps.cpp"
)

include_directories(
	"../debug"
	"../src"
	"../tests"
	".."
	"../.."
)

include(CodeCoverage.cmake)
append_coverage_compiler_flags()

target_compile_definitions(skiatest PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS)
target_compile_definitions(tinytest PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(skiatest PUBLIC /wd4200 /wd4244 /wd4267)
    target_compile_options(tinytest PUBLIC /wd4200 /wd4244 /wd4267)
else()
    target_compile_options(skiatest PUBLIC -Wall -Wextra -Wpedantic -Wno-unused-parameter -fprofile-update=atomic)
    target_compile_options(tinytest PUBLIC -Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET skiatest PROPERTY CXX_STANDARD 17)
  set_property(TARGET tinytest PROPERTY CXX_STANDARD 17)
endif()

# TODO: Add tests and install targets if needed.

# (remove all built files, remove lines to build tinytest?)
# cmake -DCMAKE_BUILD_TYPE=Debug ../cmake
# make
# ./skiatest
# lcov -c -d  . -o skiatest.info
# genhtml --ignore-errors inconsistent skiatest.info --output-directory skiatestout
# file:///D:/gerrit/skia/example/cmake/skiatestout/example/