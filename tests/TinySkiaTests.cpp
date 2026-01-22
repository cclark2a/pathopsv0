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
#include "SkiaTestCommon.h"
#include "port/SkiaPaths.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

enum class Skippable {
    no,
    yes
};

struct TinySuite {
    void (*func)(TestOptions* );
    void (*testFunc)(SkPath& path , TestOptions* options);
    std::string name;
    std::string baseName;
    Skippable skippable;
};

std::vector<TinySuite> tinySuites = {
    { V0SimplifyQuads, TestSimplify, "quad", "testQuads", Skippable::yes },
};

std::vector<std::string> highError = {
"testQuads1868284", // "0.00301069394"
"testQuads2441981", // "0.00984116644"
"testQuads2449734", // "0.00984228961"
"testQuads2465237", // "0.00986487791"
"testQuads2472989", // "0.00987935439"
"testQuads2558547", // "0.0110246753"
"testQuads2558548", // "0.0111738387"
"testQuads2559908", // "0.0112013761"
"testQuads2566300", // "0.0112189064"
"testQuads2567659", // "0.0112196235"
"testQuads2567660", // "0.0112479953"
"testQuads5109541", // "0.0114621641"
"testQuads5109542", // "0.0121286092"    
};

thread_local std::string currentTest;  // can't be in a struct

struct TinyState {
    void addADot(const OpDebugData& );
    void test();
    void trackError(PathOpsV0Lib::ContextError contextError);
    std::string stats();

    std::vector<std::string> skipFiles = { TEST_PATH_OP_SKIP_FILES };
    std::string testFirst = OP_DEBUG_FAST_TEST ? "" : TEST_FIRST;
    std::string skipTo = SKIP_TO_FILE;
    std::string largestError;
    std::atomic<float> pixelError = 0.f;
    std::atomic_int testIndex = 0; 
    std::atomic_int gapError = 0;
    std::atomic_int testsError = 0;
    std::atomic_int testsRun = 0;
    std::atomic_int testsDot = 0;
    std::atomic_int testsLine = 0;
    std::atomic_int testsSkipped = 0;
    std::atomic_int testsToRun = TESTS_TO_RUN;
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
    bool runOne = !OP_DEBUG_FAST_TEST && strlen(TEST_FIRST);
    bool showName = OP_SHOW_TEST_NAME;
    // both false if before first; start false end true if no first; both true if after first
    bool startFirstTest = OP_DEBUG_FAST_TEST || !strlen(TEST_FIRST);
    bool endFirstTest = false;
} tinyState;

void TinyState::addADot(const OpDebugData& debugData) {
#if OP_DEBUG_FAST_TEST
    std::lock_guard<std::mutex> guard(out_mutex);
#endif
    ++testsRun;
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

void TestSimplify(SkPath& path , TestOptions* options) {
    using namespace PathOpsV0Lib;
    Context* context = CreateContext();
    SkPath out;
    ContextUserData data { &out, sizeof(&out), UserDataType::outPath };
    AddUserData(context, data);
    OpDebugData debugData(options->testName, 
            options->v0MayFail ? OpDebugExpect::fail : OpDebugExpect::success, 
            CURVE_CURVE_1, CURVE_CURVE_2, CURVE_CURVE_DEPTH, 
            tinyState.defeatBreak, TEST_DEFEAT_DUMPS, tinyState.runOne
        );
    OP_DEBUG_CODE(SetDebugData(context, debugData));
    SetSkiaContextCallbacks(context);
    SetSkiaCurveCallbacks(context);
    auto isWindingFill = [](const SkPath& path) {
        return SkPathFillType::kWinding == path.getFillType()
                || SkPathFillType::kInverseWinding == path.getFillType();
    }; 
    int simpleData[] = { 1 };
    Contour* simple = SetSkiaSimplifyCallbacks(context, simpleData, sizeof(simpleData), 
            isWindingFill(path)  OP_DEBUG_PARAMS(&path));
    AddSkiaPath(context, simple, path);
#if OP_TEST_RASTER
    DebugRaster debugRaster((OpContext*) context);
    debugRaster.deleteOld();
    if (OpDebugExpect::success == debugData.expect)    
        debugRaster.in();
#endif
	ContextError contextError = Error(context);
	if (ContextError::none == contextError) {
		Resolve(context);
        ((OpContext*) context)->dumpFile("SimplifyV0 resolved");
	}
    contextError = Error(context);
	tinyState.trackError(contextError);
#if OP_TEST_RASTER
    if (OpDebugExpect::success == debugData.expect)
        debugData.error = debugRaster.out();
#endif
    DeleteContext(context);
    tinyState.addADot(debugData);
}

static void threadTest(TinySuite tinySuite, TestOptions options) {
    (*tinySuite.func)(&options);
}

void TinyState::test() {
    TestOptions options;
    options.extended = TEST_EXTENDED;
    for (const TinySuite& tinySuite : tinySuites) {
        if (!tinyState.skipTo.empty() && tinyState.skipTo != tinySuite.name)
            continue;
        options.testFunc = tinySuite.testFunc;
        options.baseName = tinySuite.baseName;
        if (!OP_DEBUG_FAST_TEST) {
            if (!testFirst.empty()) {
                if (Skippable::yes == tinySuite.skippable) {
                    const char* firstStr = testFirst.c_str();
                    options.skip = OpDebugReadNamedInt(firstStr, tinySuite.baseName.c_str());
//                    OP_ASSERT(options.skip > 0);
                    options.skip -= 1;
                } else
                    options.testFirst = testFirst;
                options.toRun = 1;
            } else {
                options.skip = tinyState.testsToSkip;
                options.toRun = tinyState.testsToRun;
            }
            (*tinySuite.func)(&options);
            continue;
        }
        // split tests into threads
        options.skip = INT_MAX;
        (*tinySuite.func)(&options);
        int totalTests = options.index + options.indexOffset;
        std::vector<std::thread> t;
        options.skip = 0;
        options.toRun = std::max(totalTests / maxThreads, 1);
        for (int index = 0; index < maxThreads; ) {
            t.push_back(std::thread(threadTest, tinySuite, options));
            options.skip += options.toRun;
            ++index;
            options.toRun = totalTests * (index + 1) / maxThreads - options.skip;
        }
        for (unsigned index = 0; index < maxThreads; ++index)
            t[index].join();
    }
    std::string s = stats();
    OpDebugOut(s + "\n");
}

void runTinyTests() {
    tinyState.test();
}
