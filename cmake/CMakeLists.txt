# (c) 2024, Cary Clark cclark2@gmail.com

cmake_minimum_required (VERSION 3.15)
project (pov0 VERSION 0.1)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

if (DEFINED COVERAGE)
	include (CodeCoverage.cmake)
	append_coverage_compiler_flags ()
endif()

if (DEFINED TSAN)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
	set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=thread")
endif ()

if (DEFINED ASAN)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
endif ()

if (DEFINED MSAN)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=memory")
endif ()

if (DEFINED LSAN)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=leak")
	set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=leak")
endif ()

set (ignoreMe "${TSAN}${ASAN}${MSAN}${LSAN}${COVERAGE}")

set (FAST_TEST 1)
set (SKIP_TO_V0 0)
set (TEST_FIRST "")
set (SKIP_TO_FILE "")
set (TEST_EXTENDED 0)

if (NOT DEFINED ONLY_V0)
	set (TEST_SKIA 1)
	set (TEST_REGION 1)
else ()
	set (TEST_SKIA 0)
	set (TEST_REGION 0)
endif ()

include_directories(
	"../debug"
	"../src"
	"../tests"
	".."
	"../.."
)

set (COMMON_SOURCES
	"../debug/OpDebug.cpp"
	"../src/OpContext.cpp"
	"../src/OpContour.cpp"
	"../src/OpCubic.cpp"
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
    "../PathOps.cpp"
)

# Add source to this project's executable.
add_executable (skia_test
	"../debug/OpDebugColor.cpp"
	"../debug/OpDebugCompare.cpp"
	"../debug/OpDebugDouble.cpp"
	"../debug/OpDebugDump.cpp"
	"../debug/OpDebugGenerator.cpp"
	"../debug/OpDebugImage.cpp"
	"../debug/OpDebugRaster.cpp"
	"../debug/OpDebugRecord.cpp"
	"../debug/OpDebugTags.cpp"
	"../port/SkiaPaths.cpp"
	"../debug/SkiaPathsDebug.cpp"
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
    "../tests/UnitTestPathOps.cpp"
)

add_executable (contains_example "../example/ContainsExample.cpp" )
add_executable (frame_example "../example/FrameExample.cpp" )
add_executable (html_canvas_example "../port/Path2D.cpp" "../example/HTMLCanvasExample.cpp" )
add_executable (simplify_example "../example/SimplifyExample.cpp" )
add_executable (skia_intersect_example "../port/SkiaPaths.cpp" "../debug/SkiaPathsDebug.cpp" "../tests/TinySkia.cpp" "../example/SkiaIntersectExample.cpp" )
add_executable (skia_simplify_example "../port/SkiaPaths.cpp" "../debug/SkiaPathsDebug.cpp" "../tests/TinySkia.cpp" "../example/SkiaSimplifyExample.cpp" )
add_executable (svg_example "../port/Path2D.cpp" "../example/SVGExample.cpp" )
add_executable (cut_test "../tests/CutTest.cpp" )
add_executable (frame_test "../tests/FrameTest.cpp" )
add_executable (path_2d_test "../port/Path2D.cpp" "../tests/Path2DTest.cpp" )

target_sources (skia_test PUBLIC ${COMMON_SOURCES})
target_sources (contains_example PUBLIC ${COMMON_SOURCES})
target_sources (frame_example PUBLIC ${COMMON_SOURCES})
target_sources (html_canvas_example PUBLIC ${COMMON_SOURCES})
target_sources (simplify_example PUBLIC ${COMMON_SOURCES})
target_sources (skia_intersect_example PUBLIC ${COMMON_SOURCES})
target_sources (skia_simplify_example PUBLIC ${COMMON_SOURCES})
target_sources (svg_example PUBLIC ${COMMON_SOURCES})
target_sources (cut_test PUBLIC ${COMMON_SOURCES})
target_sources (frame_test PUBLIC ${COMMON_SOURCES})
target_sources (path_2d_test PUBLIC ${COMMON_SOURCES})

target_compile_definitions (skia_test PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS)
target_compile_definitions (contains_example PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (frame_example PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (html_canvas_example PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (simplify_example PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (skia_intersect_example PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (skia_simplify_example PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (svg_example PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (cut_test PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (frame_test PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)
target_compile_definitions (path_2d_test PUBLIC OP_TINY_SKIA=1 NOMINMAX _CRT_SECURE_NO_WARNINGS OP_TINY_TEST=1)

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options (skia_test PUBLIC)
    target_compile_options (contains_example PUBLIC)
    target_compile_options (frame_example PUBLIC)
    target_compile_options (html_canvas_example PUBLIC)
    target_compile_options (simplify_example PUBLIC)
    target_compile_options (skia_intersect_example PUBLIC)
    target_compile_options (skia_simplify_example PUBLIC)
    target_compile_options (svg_example PUBLIC)
    target_compile_options (cut_test PUBLIC)
    target_compile_options (frame_test PUBLIC)
    target_compile_options (path_2d_test PUBLIC)
elseif (CMAKE_BUILD_TYPE MATCHES "Debug")
    target_compile_options (skia_test PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter -Wno-invalid-offsetof -fprofile-update=atomic)
    target_compile_options (contains_example PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (frame_example PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (html_canvas_example PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (simplify_example PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (skia_intersect_example PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (skia_simplify_example PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (svg_example PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (cut_test PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (frame_test PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
    target_compile_options (path_2d_test PUBLIC -g -Wall -Wpedantic -Wno-unused-parameter )
elseif (CMAKE_BUILD_TYPE MATCHES "Release")
    target_compile_options (skia_test PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra -fprofile-update=atomic)
    target_compile_options (contains_example PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (frame_example PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (html_canvas_example PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (simplify_example PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (skia_intersect_example PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (skia_simplify_example PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (svg_example PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (cut_test PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (frame_test PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
    target_compile_options (path_2d_test PUBLIC -g -Wall -Wno-unused-parameter -Wno-extra)
endif()

# (remove all built files)
# cmake -S . -B destDir/ -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang
# cmake --build destDir/
# destDir/skia_test
# lcov -c -d  . -o skiatest.info
# genhtml --ignore-errors inconsistent skiatest.info --output-directory skiatestout
# file:///D:/gerrit/skia/example/cmake/skiatestout/example/

# cmake -S . -B release/ -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang

# to add new files to Visual Studio builds:
# add build C++ line, and add build link reference; to Debug/obj/HelloWorld.ninja and Release/obj/HelloWorld.ninja

# pathopsv0-wasm/compile-wasm.sh

# to run html
# npm install -g http-server
# ~/emsdk/node/20.18.0_64bit/bin/http-server ./pathopsv0-wasm -p 8080
# http://127.0.0.1:8080
#  click examples > es > basic.html

# git checkout -b my-feature
# git commit
# git push
# https://github.com/ErikSom/Pathopsv0-WASM

# https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork
