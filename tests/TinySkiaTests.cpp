// (c) 2026, Cary Clark cclark2@gmail.com

// The original Skia PathOps tests are streamlined to make them smaller,
// faster, and random access. The actual tests run are identical.

#include <atomic>
#include <mutex>
#include <thread>
#include "OpContext.h"
#include "OpDebugRaster.h"
#include "OpSkiaTests.h"
#include "PathOps.h"
#include "TinySkia.h"
#include "TinySkiaTests.h"
#include "SkiaPathsDebug.h"
#include "SkiaTestCommon.h"
#include "curves/BinaryWinding.h"
#include "port/SkiaPaths.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

#define THREAD_DEBUG 01

struct TinySuite {
    void (*func)(TestTrack* );
    std::string name;
    float maxError = 0.12f;
};

std::vector<TinySuite> tinySuites = {
    { V0Battles, "battle" },
    { V0Chalkboard, "chalkboard" },
    { V0Fuzz763, "fuzz763" },
    { V0Inverse, "inverse" },
    { V0Issue3651, "issue3651" },
    { V0Op, "op" },
    { V0OpCircles, "circle" },
    { V0OpCubics, "cubic" },
    { V0OpFail, "opFail" },
    { V0OpLoops, "loop" },
    { V0OpRects, "opRect" },
    { V0Simplify, "simplify" },
    { V0SimplifyDegenerates, "degenerate" },
    { V0SimplifyFail, "simplifyFail" },
    { V0SimplifyQuadralaterals, "quadralateral" },
    { V0SimplifyQuads, "quad" },
    { V0SimplifyRects, "simplifyRect" },
    { V0SimplifyTriangles, "triangle" },
    { V0Tiger, "tiger", .21f },
    { V0Tests, "v0" },
};

thread_local std::string currentTest;  // can't be in a struct

struct TinyError {
    std::string testname;
    float error = 0;
};

struct TinyState {
    void addADot(const OpDebugData& );
    void test();
    void trackError(PathOpsV0Lib::ContextError contextError);
    std::string stats();

    std::vector<std::string> skipFiles = { TEST_PATH_OP_SKIP_FILES };
    std::string skipTo = SKIP_TO_FILE;
    std::string largestError;
    std::array<TinyError, 20> tinyErrors;
    std::atomic<float> pixelError = 0.f;
    std::atomic_int runIndex = 0; 
    std::atomic_int gapError = 0;
    std::atomic_int testsError = 0;
    std::atomic_int testsRun = 0;
    std::atomic_int testsDot = 0;
    std::atomic_int testsLine = 0;
    std::atomic_int testsSkipped = 0;
    std::atomic_int testsToSkip = TESTS_TO_SKIP;
    std::atomic_int treeError = 0;
    std::atomic_int silentError = 0;
    #if OP_DEBUG_FAST_TEST
    std::mutex out_mutex;
    #endif
    float baseError = 0.00247338437f;
    float maxError = baseError;  // testQuads1411840
    int maxThreads = std::thread::hardware_concurrency();
    bool checkForDuplicateNames = false;
    bool defeatBreak = TEST_DEFEAT_BREAK || !strlen(TEST_FIRST);
    bool json = false;
    bool showName = OP_SHOW_TEST_NAME;
    // both false if before first; start false end true if no first; both true if after first
    bool startFirstTest = OP_DEBUG_FAST_TEST || !strlen(TEST_FIRST);
    bool endFirstTest = false;
} tinyState;

void TestOptions::checkTestCount(int testCount) {
    if (!testCountCheck || TESTS_TO_SKIP || OP_DEBUG_FAST_TEST || strlen(TEST_FIRST))
        return;
    OP_ASSERT(testTrack.run == testCount);
    testCountCheck = false;
}

void TinyState::addADot(const OpDebugData& debugData) {
#if OP_DEBUG_FAST_TEST
    std::lock_guard<std::mutex> guard(out_mutex);
#endif
    ++testsRun;
	if (debugData.error >= debugData.maxError && debugData.showError) {
	    std::string testname = debugData.testname;
	    OpDebugOut(testname + " raster errors:" + STR(debugData.error) + "\n");
    }
    if (debugData.error > tinyErrors[0].error) {
        tinyErrors[0].testname = debugData.testname;
        tinyErrors[0].error = debugData.error;
        std::sort(tinyErrors.begin(), tinyErrors.end(), [](const TinyError& a, const TinyError& b) {
                return a.error < b.error; } );
    }
    pixelError += debugData.error; 
    if (debugData.error > maxError)
        largestError = debugData.testname;
    maxError = std::max(debugData.error, maxError);
    if (!OP_SHOW_ERRORS_ONLY && !showName && testsRun && testsRun % 1000000 == 0)
        OpDebugOut(STR(testsRun / 1000000) + "M");
    ++testsDot;
    ++testsLine;
    if (OP_SHOW_ERRORS_ONLY || showName || testsDot <= (TEST_EXTENDED ? 5000 : 500))
        return;
    OpDebugOut(".");
    testsDot -= TEST_EXTENDED ? 5000 : 500;
    if (testsLine <= (TEST_EXTENDED ? 500000 : 50000))
        return;
    OpDebugOut("\n");
    testsLine -= TEST_EXTENDED ? 500000 : 50000;
}

std::string TinyState::stats() {
    std::string s;
    s += "testsRun:" + STR(testsRun) + " testsSkipped:" + STR(testsSkipped) + " ";
    if (gapError)
        s += "gapError:" + STR(gapError) + " ";
    if (testsError)
        s += "testsError:" + STR(testsError) + " ";
    if (treeError)
        s += "treeError:" + STR(treeError) + " ";
    if (silentError)
        s += "silentError:" + STR(silentError) + " ";
    if (pixelError && testsRun)
        s += " avg pixelError:" + STR(pixelError / testsRun) + " ";
    if (baseError < maxError)
        s += "maxError:" + STR(maxError) + " largestError:" + largestError + " ";
    s.pop_back();
    s += "\n";
    for (size_t index = tinyErrors.size(); index != 0; ) {
        const TinyError& e = tinyErrors[--index];
        if (0 == e.error)
            break;
        s += e.testname + " error:" + STR(e.error) + "\n";
    }
    s.pop_back();
    return s;
}

void TinyState::trackError(PathOpsV0Lib::ContextError contextError) {
	if (PathOpsV0Lib::ContextError::none != contextError)
		++testsError;
	switch (contextError) {
		case PathOpsV0Lib::ContextError::none:
			break;
		case PathOpsV0Lib::ContextError::finite:  // input was not finite
			++silentError; 
			break;
		case PathOpsV0Lib::ContextError::toVertical:  // skewing curve exceeds float range
			++silentError; 
			break;
		case PathOpsV0Lib::ContextError::tree:
			++treeError;
			break;
		case PathOpsV0Lib::ContextError::gap:
			++gapError;
			break;
        case PathOpsV0Lib::ContextError::root:
            ++silentError;
            break;
		default:
			OP_ASSERT(0);
	}
}

#if OP_DEBUG_SERIALIZE
static std::string debugOpTest(std::string testname, const SkPath& pathA, const SkPath& pathB, TinyOps op) {
	std::string s;
    s += "void " + testname + "(TestOptions* options) {\n";
    s += "    SkPath left, right;\n";
    s += dumpSkPath(&pathA, true, "    left.") + "\n";
    s += dumpSkPath(&pathB, true, "    right.") + "\n";
    std::string opStr;
    switch(op) {
        case TinyOps::difference: opStr = "TinyOps::difference"; break;
        case TinyOps::intersect: opStr = "TinyOps::intersect"; break;
        case TinyOps::unite: opStr = "TinyOps::unite"; break;
        case TinyOps::exclusiveOr: opStr = "TinyOps::exclusiveOr"; break;
        case TinyOps::reverseDifference: opStr = "TinyOps::reverseDifference"; break;
        default: OP_ASSERT(0);
    }
    s += "    options->testOne(left, right, " + opStr + ");\n";
    s += "}\n\n";
    return s;
}

// char* so it can be called from immediate window
static void dumpOpTest(std::string testname, const SkPath& pathA, const SkPath& pathB, TinyOps op, 
            std::string filename) {
    std::string filePath = dmpFileToPath(filename);
    FILE* file = fopen(filePath.c_str(), "w");
    if (!file) {
        OpDebugOut("could not open " + filePath + " to write\n");
        return;
    }
    std::string s = debugOpTest(testname, pathA, pathB, op);
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
}

static std::string debugSimplifyTest(std::string testname, const SkPath& path) {
	std::string s;
    s += "void " + testname + "(TestOptions* options) {\n";
    s += "    SkPath path;\n";
	s += dumpSkPath(&path, true, "    path.") + "\n";
	s += "    options->testOne(path);\n";
    s += "}\n\n";
	return s;
}

// char* so it can be called from immediate window
static void dumpSimplifyTest(std::string testname, const SkPath& path, std::string filename) {
    std::string filePath = dmpFileToPath(filename);
    FILE* file = fopen(filePath.c_str(), "w");
    if (!file) {
        OpDebugOut("could not open " + filePath + " to write\n");
        return;
    }
	std::string s = debugSimplifyTest(testname, path);
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
}
#endif

TestDone TestOptions::testOne(SkPath& left, SkPath& right, TinyOps op) {
    SkPath out;
    return testSetup(left, right, op, &out);
}

TestDone TestOptions::testLast(SkPath& left, SkPath& right, TinyOps op) {
    SkPath out;
    return testPart(left, right, op, &out);
}

TestDone TestOptions::testSetup(SkPath& left, SkPath& right, TinyOps op, SkPath* result) {     
    TestDone testDone = testTrack.skipInner(1);
    if (TestDone::run != testDone)
        return testDone;
#if THREAD_DEBUG
    if (testTrack.runNamedTest)
        OpDebugOut("test:" + testTrack.testName + "\n");
#endif
    debugData = OpDebugData(testTrack.testName, 
            testFunc.mayFail ? OpDebugExpect::fail : OpDebugExpect::success, 
            testTrack.maxError, CURVE_CURVE_1, CURVE_CURVE_2, CURVE_CURVE_DEPTH, CURVE_CURVE_DUMP,
            tinyState.defeatBreak, TEST_DEFEAT_DUMPS, testTrack.runNamedTest, !ignoreRaster);
    return testPart(left, right, op, result);
}

TestDone TestOptions::testPart(SkPath& a, SkPath& b, TinyOps op, SkPath* outPtr) {
    // !!! add support for TEST_PATH_SKIP_TESTS
    using namespace PathOpsV0Lib;
    Context* context = CreateContext();
    ContextUserData outPathData { outPtr, sizeof(outPtr), UserDataType::outPath };
    bool useDoublesInConics = USE_DOUBLE_CONICS;
    AddUserData(context, outPathData);
    ContextUserData doubleConicData { &useDoublesInConics, sizeof(useDoublesInConics), 
            (UserDataType) SkiaUserData::useDoubleConics };
    AddUserData(context, doubleConicData);
    OP_DEBUG_CODE(SetDebugData(context, debugData));
    OP_DEBUG_CODE(OpDebugData& debugRef = GetDebugData(context));
    SetSkiaContextCallbacks(context);
    SetSkiaCurveCallbacks(context);
    auto isWindingFill = [](const SkPath& path) {
        return SkPathFillType::kWinding == path.getFillType()
                || SkPathFillType::kInverseWinding == path.getFillType();
    }; 
    if (TinyOps::simplify == op) {
#if OP_DEBUG_SERIALIZE
        if (testTrack.runNamedTest && !TEST_DEFEAT_DUMPS)
            dumpSimplifyTest(testTrack.testName, a, TestInFile);
#endif
        int simpleData[] = { 1 };
        Contour* simple = SetSkiaSimplifyCallbacks(context, simpleData, sizeof(simpleData), 
                isWindingFill(a)  OP_DEBUG_PARAMS(&a));
        AddSkiaPath(context, simple, a);
    } else {
        TinyOps mappedOp = MapInvertedSkPathOp(op, a.isInverseFillType(), b.isInverseFillType());
#if OP_DEBUG_SERIALIZE
        if (testTrack.runNamedTest && !TEST_DEFEAT_DUMPS)
            dumpOpTest(testTrack.testName, a, b, mappedOp, TestInFile);
#endif
        bool aIsWinding = isWindingFill(a);
        bool bIsWinding = isWindingFill(b);
        BinaryWindType windType = aIsWinding && bIsWinding ? BinaryWindType::windBoth
                : aIsWinding ? BinaryWindType::windLeft : bIsWinding ? BinaryWindType::windRight
                : BinaryWindType::evenOdd;
        SetSkiaOpContextCallbacks(context, mappedOp, windType);
        int leftData[] = { 1, 0 };
        Contour* left = SetSkiaOpContourCallbacks(context, leftData, sizeof(leftData), 
                BinaryOperand::left  OP_DEBUG_PARAMS(&a));
        AddSkiaPath(context, left, a);
        int rightData[] = { 0, 1 };
        Contour* right = SetSkiaOpContourCallbacks(context, rightData, sizeof(rightData), 
                BinaryOperand::right  OP_DEBUG_PARAMS(&b));
        AddSkiaPath(context, right, b);
    }
#if OP_TEST_RASTER
    DebugRaster debugRaster((OpContext*) context);
#endif
	ContextError contextError = Error(context);
	if (ContextError::none == contextError) {
#if OP_TEST_RASTER
//        OP_DEBUG_SERIALIZE_CODE(debugRaster.deleteOld());
        if (OpDebugExpect::success == debugRef.expect)    
            debugRaster.in();
#endif
		Resolve(context);
#if OP_DEBUG_SERIALIZE
        ((OpContext*) context)->dumpFile("testOp resolved");
#endif
	}
    contextError = Error(context);
	tinyState.trackError(contextError);
#if OP_TEST_RASTER
    if (ContextError::none == contextError && OpDebugExpect::success == debugRef.expect)
        debugRef.error = debugRaster.out();
#endif
    OP_DEBUG_CODE(debugData = debugRef);
    DeleteContext(context);
    tinyState.addADot(debugData);
    ++testTrack.run;
    --testTrack.toRun;
    return testTrack.toRun <= 0 ? TestDone::yes : TestDone::no;
}

static void threadTest(TinySuite tinySuite, TestTrack track) {
    (*tinySuite.func)(&track);
}

void TinyState::test() {
    TestTrack track;
    track.extended = TEST_EXTENDED;
    track.runNamedTest = !OP_DEBUG_FAST_TEST && strlen(TEST_FIRST);
    if (track.runNamedTest) {
        track.testMatch = TEST_FIRST;
        int tens = 1;
        track.hasDigits = isdigit(track.testMatch.back());
        while (isdigit(track.testMatch.back())) {
            track.testSuffix += (track.testMatch.back() - '0') * tens;
            tens *= 10;
            track.testMatch.pop_back();
        }
        track.toRun = 1;
    #if THREAD_DEBUG
        if (tinyState.testsToSkip)
            OpDebugOut("(skip " + STR(tinyState.testsToSkip) + " ignored because TEST_FIRST is set\n");
    #endif
        track.skip = 0;  // can't set tests to skip if we are only running one test
    } else {
        track.skip = tinyState.testsToSkip;
        track.toRun = TESTS_TO_RUN ? TESTS_TO_RUN : INT_MAX;
    }
    #if THREAD_DEBUG
        std::string s = OP_DEBUG_FAST_TEST ? "(threaded)" : "(unthreaded)";
        if (track.skip)
            s += " skip:" + STR(track.skip);
        OpDebugOut(s + "\n");
    #endif
    for (const TinySuite& tinySuite : tinySuites) {
        if (!tinyState.skipTo.empty() && tinyState.skipTo != tinySuite.name)
            continue;
        track.maxError = tinySuite.maxError;
        if (!OP_DEBUG_FAST_TEST) {
    #if THREAD_DEBUG
            OpDebugOut("suite:" + tinySuite.name + "\n");
    #endif
            (*tinySuite.func)(&track);
            continue;
        }
        // split tests into threads
        track.skip = INT_MAX;
        (*tinySuite.func)(&track);  // track.indexOffset is set here
        int totalTests = INT_MAX - track.skip;
    #if THREAD_DEBUG
        std::string s = tinySuite.name + " totalTests:" + STR(totalTests);
        if (tinyState.testsToSkip)
            s += " skip:" + STR(tinyState.testsToSkip);
        OpDebugOut(s + "\n");
    #endif
        if (tinyState.testsToSkip > totalTests) {
            tinyState.testsToSkip -= totalTests;
            continue;
        }
        track.skip = tinyState.testsToSkip;
        tinyState.testsToSkip = 0;
        track.toRun = std::max((totalTests - track.skip) / maxThreads, 0);
        if (TESTS_TO_RUN)
            track.toRun = std::min(track.toRun, TESTS_TO_RUN);
        std::vector<std::thread> t;
        for (int index = 0; index < maxThreads; ) {
    #if THREAD_DEBUG
            OpDebugOut("thread:" + STR(index) + " " + tinySuite.name 
                    + " skip:" + STR(track.skip) + " toRun:" + STR(track.toRun) + "\n");
    #endif
            t.push_back(std::thread(threadTest, tinySuite, track));
            track.skip += track.toRun;
            ++index;
            track.toRun = totalTests * (index + 1) / maxThreads - track.skip;
        }
        for (unsigned index = 0; index < maxThreads; ++index)
            t[index].join();
    }
    s = stats();
    OpDebugOut(s + "\n");
}

bool TestTrack::runTests(const std::vector<TestFunc>& tests) {
    auto testOne = [this](const TestFunc& test) {
        for (std::string skipName : TEST_PATH_SKIP_TESTS) {
            if (test.name == skipName)
                return false;
        }
        TestOptions options(*this, test);
        (*test.func)(&options);
        return true;
    };
    for (const TestFunc& test : tests) {
        testFunc = &test;
        runIndex = 0;
        if (!test.numbered && skipTests(1))
            continue;
        if (!testOne(test))
            continue;
        if (0 == toRun)
            return true;
    }
    return false;
}

TestDone TestTrack::skipInner(int count) {
    if (0 == toRun)
        return TestDone::yes;
    if (hasDigits) {  // non-threaded
        OP_ASSERT(runNamedTest);
        if (testFunc->numbered) {
            if (testFunc->name != testMatch)
                return TestDone::no;
            if (testSuffix + indexOffset > runIndex + count) {
                runIndex += count;
                return TestDone::no;
            }
        } else if (testFunc->name != TEST_FIRST)
            return TestDone::no;
    } else if (runNamedTest) {
        if (testFunc->numbered)
            return TestDone::no;
        if (testFunc->name != TEST_FIRST)
            return TestDone::no;
    }
    if (skip >= count) {
        skip -= count;
        runIndex += count;
        return TestDone::no;
    }
    if (skip)
        return TestDone::skip;
    testName = testFunc->name;
    if (testFunc->numbered)
        testName += STR(++runIndex - indexOffset);
    return TestDone::run;
}

bool TestTrack::skipTests(int count) {
    TestDone testDone = skipInner(count);
    return TestDone::yes == testDone || TestDone::no == testDone;
}

void runTinyTests() {
    tinyState.test();
}
