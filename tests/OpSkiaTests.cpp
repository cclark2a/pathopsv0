// (c) 2023, Cary Clark cclark2@gmail.com
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkRegion.h"
#include "OpSkiaTests.h"
#include "OpContour.h"
#include "OpCurve.h"
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
#include "CTPL_old/ctpl_stl.h"
#endif
#include <atomic>
#include <vector>

struct testPair {
    void (*func)();
    std::string name;
};

std::vector<testPair> testPairs = {
    { run_battle_tests, "battle" },
    { run_chalkboard_tests, "chalkboard" },
    { run_fuzz763_tests, "fuzz763" },
    { run_inverse_tests, "inverse" },
    { run_issue3651_tests, "issue3651" },
    { run_op_tests, "op" },
    { run_op_circle_tests, "circle" },
    { run_op_cubic_tests, "cubic" },
    { run_op_loop_tests, "loop" },
    { run_op_rect_tests, "rect" },
    { run_simplify_tests, "simplify" },
    { run_simplify_degenerate_tests, "simplifyDegenerate" },
    { run_simplify_fail_tests, "simplifyFail" },
    { run_simplify_quadralaterals_tests, "simplifyQuadralaterals" },
    { run_simplify_quads_tests, "simplifyQuads" },
    { run_simplify_rect_tests, "simplifyRect" },
    { run_simplify_triangles_tests, "simplifyTriangles" },
    { run_tiger_tests, "tiger" },
    { run_v0_tests, "v0" },
};

// particulars that failed on 11/10/2023
std::vector<std::string> circleFails = {
"thread_circles315825",
"thread_circles315826",
"thread_circles315827",
"thread_circles315829",
"thread_circles315830",
"thread_circles315831",
"thread_circles315833",
"thread_circles315834",
"thread_circles315835",
"thread_circles315837",
"thread_circles315838",
"thread_circles315839",
"thread_circles392738",
"thread_circles392742",
"thread_circles392746",
"thread_circles392750",
"thread_circles392754",
"thread_circles392758",
"thread_circles392762",
"thread_circles392766",
"thread_circles401474",
"thread_circles401475",
"thread_circles401478",
"thread_circles401479",
"thread_circles401482",
"thread_circles401483",
"thread_circles401486",
"thread_circles401487",
"thread_circles401490",
"thread_circles401491",
"thread_circles401494",
"thread_circles401495",
"thread_circles401498",
"thread_circles401499",
"thread_circles401502",
"thread_circles401503",
"thread_circles414609",
"thread_circles414610",
"thread_circles414611",
"thread_circles414613",
"thread_circles414614",
"thread_circles414615",
"thread_circles414617",
"thread_circles414618",
"thread_circles414619",
"thread_circles414621",
"thread_circles414622",
"thread_circles414623",
"thread_circles1066753",
"thread_circles1066754",
"thread_circles1066755",
"thread_circles1066757",
"thread_circles1066758",
"thread_circles1066759",
"thread_circles1066761",
"thread_circles1066762",
"thread_circles1066763",
"thread_circles1066765",
"thread_circles1066766",
"thread_circles1066767",
"thread_circles1066769",
"thread_circles1066770",
"thread_circles1066771",
"thread_circles1066773",
"thread_circles1066774",
"thread_circles1066775",
"thread_circles1066777",
"thread_circles1066778",
"thread_circles1066779",
"thread_circles1066781",
"thread_circles1066782",
"thread_circles1066783",
"thread_circles1162849",
"thread_circles1162850",
"thread_circles1162851",
"thread_circles1162853",
"thread_circles1162854",
"thread_circles1162855",
"thread_circles1162857",
"thread_circles1162858",
"thread_circles1162859",
"thread_circles1162861",
"thread_circles1162862",
"thread_circles1162863",
"thread_circles1162865",
"thread_circles1162866",
"thread_circles1162867",
"thread_circles1162869",
"thread_circles1162870",
"thread_circles1162871",
"thread_circles1162873",
"thread_circles1162874",
"thread_circles1162875",
"thread_circles1162877",
"thread_circles1162878",
"thread_circles1162879",
"thread_circles1333682",
"thread_circles1333686",
"thread_circles1333690",
"thread_circles1333694",
"thread_circles1333698",
"thread_circles1333702",
"thread_circles1333706",
"thread_circles1333710",
"thread_circles1333714",
"thread_circles1333718",
"thread_circles1333722",
"thread_circles1333726",
"thread_circles1333730",
"thread_circles1333734",
"thread_circles1333738",
"thread_circles1333742",
"thread_circles1333746",
"thread_circles1333750",
"thread_circles1333754",
"thread_circles1333758",
"thread_circles1341937",
"thread_circles1341938",
"thread_circles1341939",
"thread_circles1341941",
"thread_circles1341942",
"thread_circles1341943",
"thread_circles1341945",
"thread_circles1341946",
"thread_circles1341947",
"thread_circles1341949",
"thread_circles1341950",
"thread_circles1341951",
"thread_circles1341953",
"thread_circles1341954",
"thread_circles1341955",
"thread_circles1341957",
"thread_circles1341958",
"thread_circles1341959",
"thread_circles1341961",
"thread_circles1341962",
"thread_circles1341963",
"thread_circles1341965",
"thread_circles1341966",
"thread_circles1341967",
"thread_circles1342305",
"thread_circles1342306",
"thread_circles1342307",
"thread_circles1342309",
"thread_circles1342310",
"thread_circles1342311",
"thread_circles1342313",
"thread_circles1342314",
"thread_circles1342315",
"thread_circles1342317",
"thread_circles1342318",
"thread_circles1342319",
"thread_circles1415857",
"thread_circles1415858",
"thread_circles1415859",
"thread_circles1415861",
"thread_circles1415862",
"thread_circles1415863",
"thread_circles1415865",
"thread_circles1415866",
"thread_circles1415867",
"thread_circles1415869",
"thread_circles1415870",
"thread_circles1415871",
"thread_circles1415873",
"thread_circles1415874",
"thread_circles1415875",
"thread_circles1415877",
"thread_circles1415878",
"thread_circles1415879",
"thread_circles1415881",
"thread_circles1415882",
"thread_circles1415883",
"thread_circles1415885",
"thread_circles1415886",
"thread_circles1415887",
"thread_circles1582178",
"thread_circles1582179",
"thread_circles1582181",
"thread_circles1582182",
"thread_circles1582183",
"thread_circles1582186",
"thread_circles1582187",
"thread_circles1582189",
"thread_circles1582190",
"thread_circles1582191",
"thread_circles1582194",
"thread_circles1582195",
"thread_circles1582197",
"thread_circles1582198",
"thread_circles1582199",
"thread_circles1582202",
"thread_circles1582203",
"thread_circles1582205",
"thread_circles1582206",
"thread_circles1582207",

};


std::vector<std::string> cubicFails = {
"thread_cubics157377",
"thread_cubics157378",
"thread_cubics157379",
"thread_cubics157381",
"thread_cubics157382",
"thread_cubics157383",
"thread_cubics157385",
"thread_cubics157386",
"thread_cubics157387",
"thread_cubics157389",
"thread_cubics157390",
"thread_cubics157391",
"thread_cubics184163",
"thread_cubics184167",
"thread_cubics184171",
"thread_cubics184175",
"thread_cubics184226",
"thread_cubics184230",
"thread_cubics184234",
"thread_cubics184238",
"thread_cubics228708",
"thread_cubics228712",
"thread_cubics228716",
"thread_cubics228720",
"thread_cubics298449",
"thread_cubics298450",
"thread_cubics298451",
"thread_cubics298453",
"thread_cubics298454",
"thread_cubics298455",
"thread_cubics298457",
"thread_cubics298458",
"thread_cubics298459",
"thread_cubics298461",
"thread_cubics298462",
"thread_cubics298463",
"thread_cubics303843",
"thread_cubics303847",
"thread_cubics303851",
"thread_cubics303855",
"thread_cubics332066",
"thread_cubics332070",
"thread_cubics332074",
"thread_cubics332078",
"thread_cubics354770",
"thread_cubics354774",
"thread_cubics354778",
"thread_cubics354782",
"thread_cubics482612",
"thread_cubics482616",
"thread_cubics482620",
"thread_cubics482624",
"thread_cubics488945",
"thread_cubics488946",
"thread_cubics488947",
"thread_cubics488949",
"thread_cubics488950",
"thread_cubics488951",
"thread_cubics488953",
"thread_cubics488954",
"thread_cubics488955",
"thread_cubics488957",
"thread_cubics488958",
"thread_cubics488959",
"thread_cubics523907",
"thread_cubics523911",
"thread_cubics523915",
"thread_cubics523919",
"thread_cubics524018",
"thread_cubics524022",
"thread_cubics524026",
"thread_cubics524030",
"thread_cubics673155",
"thread_cubics673159",
"thread_cubics673163",
"thread_cubics673167",
"thread_cubics770307",
"thread_cubics770311",
"thread_cubics770315",
"thread_cubics770319",
"thread_cubics771987",
"thread_cubics771991",
"thread_cubics771995",
"thread_cubics771999",
"thread_cubics779937",
"thread_cubics779939",
"thread_cubics779941",
"thread_cubics779943",
"thread_cubics779945",
"thread_cubics779947",
"thread_cubics779949",
"thread_cubics779951",
"thread_cubics794049",
"thread_cubics794050",
"thread_cubics794053",
"thread_cubics794054",
"thread_cubics794057",
"thread_cubics794058",
"thread_cubics794061",
"thread_cubics794062",
"thread_cubics814996",
"thread_cubics815000",
"thread_cubics815004",
"thread_cubics815008",
"thread_cubics819698",
"thread_cubics819702",
"thread_cubics819706",
"thread_cubics819710",
"thread_cubics861650",
"thread_cubics861654",
"thread_cubics861658",
"thread_cubics861662",
"thread_cubics899937",
"thread_cubics899938",
"thread_cubics899939",
"thread_cubics899941",
"thread_cubics899942",
"thread_cubics899943",
"thread_cubics899945",
"thread_cubics899946",
"thread_cubics899947",
"thread_cubics899949",
"thread_cubics899950",
"thread_cubics899951",
"thread_cubics911345",
"thread_cubics911346",
"thread_cubics911347",
"thread_cubics911349",
"thread_cubics911350",
"thread_cubics911351",
"thread_cubics911353",
"thread_cubics911354",
"thread_cubics911355",
"thread_cubics911357",
"thread_cubics911358",
"thread_cubics911359",
"thread_cubics917969",
"thread_cubics917970",
"thread_cubics917971",
"thread_cubics917973",
"thread_cubics917974",
"thread_cubics917975",
"thread_cubics917977",
"thread_cubics917978",
"thread_cubics917979",
"thread_cubics917981",
"thread_cubics917982",
"thread_cubics917983",
"thread_cubics935793",
"thread_cubics935794",
"thread_cubics935795",
"thread_cubics935797",
"thread_cubics935798",
"thread_cubics935799",
"thread_cubics935801",
"thread_cubics935802",
"thread_cubics935803",
"thread_cubics935805",
"thread_cubics935806",
"thread_cubics935807",
"thread_cubics938817",
"thread_cubics938818",
"thread_cubics938819",
"thread_cubics938821",
"thread_cubics938822",
"thread_cubics938823",
"thread_cubics938825",
"thread_cubics938826",
"thread_cubics938827",
"thread_cubics938829",
"thread_cubics938830",
"thread_cubics938831",
"thread_cubics945217",
"thread_cubics945218",
"thread_cubics945221",
"thread_cubics945222",
"thread_cubics945225",
"thread_cubics945226",
"thread_cubics945229",
"thread_cubics945230",
"thread_cubics952305",
"thread_cubics952306",
"thread_cubics952309",
"thread_cubics952310",
"thread_cubics952313",
"thread_cubics952314",
"thread_cubics952317",
"thread_cubics952318",
"thread_cubics1068931",
"thread_cubics1068935",
"thread_cubics1068939",
"thread_cubics1068943",
"thread_cubics1070291",
"thread_cubics1070295",
"thread_cubics1070299",
"thread_cubics1070303",
"thread_cubics1090529",
"thread_cubics1090530",
"thread_cubics1090531",
"thread_cubics1090533",
"thread_cubics1090534",
"thread_cubics1090535",
"thread_cubics1090537",
"thread_cubics1090538",
"thread_cubics1090539",
"thread_cubics1090541",
"thread_cubics1090542",
"thread_cubics1090543",
"thread_cubics1110611",
"thread_cubics1110615",
"thread_cubics1110619",
"thread_cubics1110623",
"thread_cubics1110658",
"thread_cubics1110662",
"thread_cubics1110666",
"thread_cubics1110670",
"thread_cubics1199124",
"thread_cubics1199132",
"thread_cubics1210338",
"thread_cubics1210339",
"thread_cubics1210342",
"thread_cubics1210343",
"thread_cubics1210346",
"thread_cubics1210347",
"thread_cubics1210350",
"thread_cubics1210351",
"thread_cubics1228852",
"thread_cubics1228856",
"thread_cubics1228860",
"thread_cubics1228864",
"thread_cubics1231490",
"thread_cubics1231494",
"thread_cubics1231498",
"thread_cubics1231502",
"thread_cubics1237331",
"thread_cubics1237335",
"thread_cubics1237339",
"thread_cubics1237343",
"thread_cubics1243395",
"thread_cubics1243399",
"thread_cubics1243403",
"thread_cubics1243407",
"thread_cubics1258498",
"thread_cubics1258502",
"thread_cubics1258506",
"thread_cubics1258510",
"thread_cubics1264787",
"thread_cubics1264791",
"thread_cubics1264795",
"thread_cubics1264799",
"thread_cubics1359208",
"thread_cubics1359216",
"thread_cubics1365922",
"thread_cubics1365923",
"thread_cubics1365926",
"thread_cubics1365927",
"thread_cubics1365930",
"thread_cubics1365931",
"thread_cubics1365934",
"thread_cubics1365935",
"thread_cubics1413027",
"thread_cubics1413035",
"thread_cubics1415057",
"thread_cubics1415058",
"thread_cubics1415059",
"thread_cubics1415061",
"thread_cubics1415062",
"thread_cubics1415063",
"thread_cubics1415065",
"thread_cubics1415066",
"thread_cubics1415067",
"thread_cubics1415069",
"thread_cubics1415070",
"thread_cubics1415071",
"thread_cubics1437601",
"thread_cubics1437602",
"thread_cubics1437603",
"thread_cubics1437605",
"thread_cubics1437606",
"thread_cubics1437607",
"thread_cubics1437609",
"thread_cubics1437610",
"thread_cubics1437611",
"thread_cubics1437613",
"thread_cubics1437614",
"thread_cubics1437615",
"thread_cubics1478465",
"thread_cubics1478467",
"thread_cubics1478469",
"thread_cubics1478471",
"thread_cubics1478473",
"thread_cubics1478475",
"thread_cubics1478477",
"thread_cubics1478479",
"thread_cubics1520801",
"thread_cubics1520803",
"thread_cubics1520805",
"thread_cubics1520807",
"thread_cubics1520809",
"thread_cubics1520811",
"thread_cubics1520813",
"thread_cubics1520815",
"thread_cubics1526531",
"thread_cubics1526535",
"thread_cubics1526539",
"thread_cubics1526543",
"thread_cubics1592195",
"thread_cubics1592199",
"thread_cubics1592203",
"thread_cubics1592207",
"thread_cubics1646850",
"thread_cubics1646854",
"thread_cubics1646858",
"thread_cubics1646862",
};

// skip tests by filename
std::vector<std::string> skipTestFiles = { TEST_PATH_OP_SKIP_FILES };
std::vector<std::string> skipRestFiles = { TEST_PATH_OP_SKIP_REST };
std::string currentTestFile;
std::string testFirst = OP_DEBUG_FAST_TEST ? "" : TEST_PATH_OP_FIRST;
std::string skipToFile = TEST_PATH_OP_SKIP_TO_FILE;
bool skiatest::Reporter::allowExtendedTest() { return OP_TEST_ALLOW_EXTENDED 
        | TEST_FAILS
        | (OP_DEBUG_FAST_TEST ? 0 : strlen(TEST_PATH_OP_FIRST)); }
bool testFailsOnly = TEST_FAILS;
std::atomic_int testIndex; 
bool showTestName = OP_SHOW_TEST_NAME;
std::atomic_int testsRun;
std::atomic_int totalRun;
std::atomic_int testsSkipped;
std::atomic_int totalSkipped;
std::atomic_int testsError;
std::atomic_int totalError;
extern std::atomic_int testsWarn;
std::atomic_int totalWarn;
std::atomic_int testsFailSkiaPass;
std::atomic_int totalFailSkiaPass;
std::atomic_int testsPassSkiaFail;
std::atomic_int totalPassSkiaFail;

bool PathOpsDebug::gCheckForDuplicateNames = false;
bool PathOpsDebug::gJson = false;

#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
ctpl::thread_pool threadpool(OP_MAX_THREADS);
#endif

// If code is built with the right architecture, the numerics use float multiply-add, which is
// signficantly more accurate that separate multiply and add instructions. Both should work.
// Since many tests may fail with one or the other, detect it, and use it to filter the test list
// accordingly.
bool runningWithFMA() {
    static bool calculated = false;
    static bool runWithFMA = false;
    if (calculated)
        return runWithFMA;
    // this is fragile, but as of 10/04/23, detects FMA with these values.
    LinePts line = { OpPoint(OpDebugBitsToFloat(0x4308f83e), OpDebugBitsToFloat(0x4326aaab)),  // {136.97, 166.667}
        OpPoint(OpDebugBitsToFloat(0x42c55d28), OpDebugBitsToFloat(0x430c5806)) };  // {98.68195, 140.344}
    OpCurve c = { { OpDebugBitsToFloat(0x42f16017), OpDebugBitsToFloat(0x431b7908) }, // {120.688, 155.473}
        { OpDebugBitsToFloat(0x42ed5d28), OpDebugBitsToFloat(0x43205806) } }; // {118.682, 160.344}
    OpCurve rotated = c.toVertical(line);
    OP_ASSERT(rotated.pts[0].x == OpDebugBitsToFloat(0x390713e0)     // 0.00012882007
            || rotated.pts[0].x == OpDebugBitsToFloat(0x39000000));  // 0.00012207031
    runWithFMA = rotated.pts[0].x == OpDebugBitsToFloat(0x390713e0);
    calculated = true;
    return runWithFMA;
}

void initializeTests(skiatest::Reporter* r, const char* name) {
    if (r)
        r->testname = name;
}

void initTests(std::string filename) {
    totalRun += testsRun;
    totalSkipped += testsSkipped;
    totalError += testsError;
    totalWarn += testsWarn;
    totalFailSkiaPass += testsFailSkiaPass;
    totalPassSkiaFail += testsPassSkiaFail;
    OpDebugOut(currentTestFile + " run:" + STR(testsRun) + " skipped:" + STR(testsSkipped)
            + " err:" + STR(testsError) + " warn:" + STR(testsWarn)
            + " v0:" + STR(testsPassSkiaFail) + " sk:" + STR(testsFailSkiaPass)
            + " total run:" + STR(totalRun) + " skipped:" + STR(totalSkipped)
            + " err:" + STR(totalError) + " warn:" + STR(totalWarn)
            + " v0:" + STR(totalPassSkiaFail) + " sk:" + STR(totalFailSkiaPass) + "\n");
    currentTestFile = filename;
    testsRun = 0;
    testsSkipped = 0;
    testsError = 0;
    testsWarn = 0;
    testsFailSkiaPass = 0;
    testsPassSkiaFail = 0;
    OpDebugOut(currentTestFile + "\n");
}

bool skipTest(std::string name) {
    if (testFailsOnly) {
        OP_ASSERT(currentTestFile == "circle" || currentTestFile == "cubic");
        if (currentTestFile == "circle") {
            if (testIndex >= (int) circleFails.size() || circleFails[testIndex] != name)
                return (void) ++testsSkipped, true;
        } else if (testIndex >= (int) cubicFails.size() || cubicFails[testIndex] != name)
            return (void) ++testsSkipped, true;
        ++testIndex;
    } else if (name != testFirst) {
        if ("" != testFirst)
            return (void) ++testsSkipped, true;
        if (skipRestFiles.end() != std::find(skipRestFiles.begin(), skipRestFiles.end(),
                name))
            skipTestFiles.push_back(currentTestFile);
        if ((skipToFile.size() && currentTestFile != skipToFile) 
                || skipTestFiles.end() != std::find(skipTestFiles.begin(), skipTestFiles.end(), 
                currentTestFile))
            return (void) ++testsSkipped, true;
    }
    ++testsRun;
    if (showTestName)
        OpDebugOut(name + "\n");
#if !OP_SHOW_ERRORS_ONLY    
    else if (testsRun % 500 == 0) {
        OpDebugOut(".");
        if (testsRun % 50000 == 0)
            OpDebugOut("\n");
    }
#endif
    return false;
}

uint64_t timerFrequency;
uint64_t timerStart;

void runTests() {
    auto runTest = [](std::string s) {
        for (auto pair : testPairs) {
            if (pair.name == s) {
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
#if !OP_SHOW_ERRORS_ONLY
                OpDebugOut(pair.name + " started\n");
#endif
                currentTestFile = pair.name;
#else
                initTests(pair.name);
#endif
                (*pair.func)();
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS && !OP_SHOW_ERRORS_ONLY
                OpDebugOut(pair.name + " finished\n");
#endif
                return;
            }
        }
    };
	OpDebugOut("\n");
    timerFrequency = OpInitTimer();
    timerStart = OpReadTimer();
    if (testFailsOnly) {
        testIndex = 0;
        runTest("circle");
        testIndex = 0;
        runTest("cubic");
    } else if (skipToFile.size()) {
        runTest(skipToFile);
    } else {
        runTest("v0");  // check for these failures first
        runTest("op");
        for (auto pair : testPairs) {
            if (pair.name != "v0" && pair.name != "op")
                runTest(pair.name);
        }
    }
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
    threadpool.stop(true);
#endif
    uint64_t end = OpReadTimer();
    float elapsed = OpTicksToSeconds(end - timerStart, timerFrequency);
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
    OpDebugOut("skia tests done: " + STR(elapsed) + "s\n");
#else
    initTests("skia tests done: " + STR(elapsed) + "s\n");
#endif
    OpDebugOut("total run:" + STR(testsRun) + " skipped:" + STR(testsSkipped) 
            + " errors:" + STR(testsError) +  " warnings:" + STR(testsWarn) 
            + " v0 only:" + STR(testsPassSkiaFail) + " skia only:" + STR(testsFailSkiaPass) + "\n");
}

const int bitWidth = 64;
const int bitHeight = 64;

static SkRect debug_scale_matrix(const SkPath& one, const SkPath* two, SkMatrix& scale) {
    SkRect larger = one.getBounds();
    if (two) {
        larger.join(two->getBounds());
    }
    SkScalar largerWidth = larger.width();
    if (largerWidth < 4) {
        largerWidth = 4;
    }
    SkScalar largerHeight = larger.height();
    if (largerHeight < 4) {
        largerHeight = 4;
    }
    SkScalar hScale = (bitWidth - 2) / largerWidth;
    SkScalar vScale = (bitHeight - 2) / largerHeight;
    scale.reset();
    scale.preScale(hScale, vScale);
    larger.fLeft *= hScale;
    larger.fRight *= hScale;
    larger.fTop *= vScale;
    larger.fBottom *= vScale;
    SkScalar dx = -16000 > larger.fLeft ? -16000 - larger.fLeft
        : 16000 < larger.fRight ? 16000 - larger.fRight : 0;
    SkScalar dy = -16000 > larger.fTop ? -16000 - larger.fTop
        : 16000 < larger.fBottom ? 16000 - larger.fBottom : 0;
    scale.preTranslate(dx, dy);
    return larger;
}

static int debug_paths_draw_the_same(const SkPath& one, const SkPath& two, SkBitmap& bits) {
    if (bits.width() == 0) {
        bits.allocN32Pixels(bitWidth * 2, bitHeight);
    }
    SkCanvas canvas(bits);
    canvas.drawColor(SK_ColorWHITE);
    SkPaint paint;
    canvas.save();
    const SkRect& bounds1 = one.getBounds();
    canvas.translate(-bounds1.fLeft + 1, -bounds1.fTop + 1);
    canvas.drawPath(one, paint);
    canvas.restore();
    canvas.save();
    canvas.translate(-bounds1.fLeft + 1 + bitWidth, -bounds1.fTop + 1);
    canvas.drawPath(two, paint);
    canvas.restore();
    int errors = 0;
    for (int y = 0; y < bitHeight - 1; ++y) {
        uint32_t* addr1 = bits.getAddr32(0, y);
        uint32_t* addr2 = bits.getAddr32(0, y + 1);
        uint32_t* addr3 = bits.getAddr32(bitWidth, y);
        uint32_t* addr4 = bits.getAddr32(bitWidth, y + 1);
        for (int x = 0; x < bitWidth - 1; ++x) {
            // count 2x2 blocks
            bool err = addr1[x] != addr3[x];
            if (err) {
                errors += addr1[x + 1] != addr3[x + 1]
                    && addr2[x] != addr4[x] && addr2[x + 1] != addr4[x + 1];
            }
        }
    }
    return errors;
}

std::vector<std::string> warningStrings = { "no edge found: last, last resort" };

void ReportError(std::string testname, int errors, std::vector<OpDebugWarning>& warnings) {
    std::string s = testname;
    // !!! when there's more than one warning, put this in a loop or lambda or something
    int count = 0;
    OpDebugWarning test = OpDebugWarning::lastResort;
    for (OpDebugWarning w : warnings) {
        if (test == w)
            ++count;
    }
    if (count) {
        s += " " + warningStrings[(int) test];
        if (count > 1)
            s += " (x" + STR(count) + ")";
    }
    if (errors)
        s += " had errors=" + STR(errors);
    OpDebugOut(s + "\n");
    OpDebugOut("");  // for setting a breakpoint
}

int VerifyOpNoRegion(const SkPath& left, const SkPath& right, SkPathOp op, const SkPath& result) {
    SkMatrix scale;
    SkRect bounds = debug_scale_matrix(left, &right, scale);
    SkPath scaledLeft, scaledRight, scaledResult;
    scaledLeft.addPath(left, scale);
    scaledLeft.setFillType(left.getFillType());
    scaledRight.addPath(right, scale);
    scaledRight.setFillType(right.getFillType());
    scaledResult.addPath(result, scale);
    scaledResult.setFillType(result.getFillType());
    SkBitmap bitmap;
    bitmap.allocN32Pixels(bitWidth, bitHeight);
    SkCanvas canvas(bitmap);
    canvas.drawColor(SK_ColorBLACK);
    SkPaint paint;
    paint.setBlendMode(SkBlendMode::kPlus);
    constexpr uint32_t leftColor = SK_ColorRED;
    constexpr uint32_t rightColor = SK_ColorBLUE;
    constexpr uint32_t resultColor = SK_ColorGREEN;
    canvas.translate(-bounds.fLeft, -bounds.fTop);
    paint.setColor(leftColor);
    canvas.drawPath(scaledLeft, paint);
    paint.setColor(rightColor);
    canvas.drawPath(scaledRight, paint);
    paint.setColor(resultColor);
    canvas.drawPath(scaledResult, paint);
    std::vector<uint32_t> okColors = { SK_ColorBLACK };
    switch (op) {
        case kDifference_SkPathOp:
            okColors.push_back(leftColor | resultColor);
            okColors.push_back(rightColor);
            okColors.push_back(leftColor | rightColor);
        break;
        case kIntersect_SkPathOp:
            okColors.push_back(leftColor);
            okColors.push_back(rightColor);
            okColors.push_back(leftColor | rightColor | resultColor);
        break;
        case kUnion_SkPathOp:
            okColors.push_back(leftColor | resultColor);
            okColors.push_back(rightColor | resultColor);
            okColors.push_back(leftColor | rightColor | resultColor);
        break;
        case kXOR_SkPathOp:
            okColors.push_back(leftColor | resultColor);
            okColors.push_back(rightColor | resultColor);
            okColors.push_back(leftColor | rightColor);
        break;
        case kReverseDifference_SkPathOp:
            okColors.push_back(leftColor);
            okColors.push_back(rightColor | resultColor);
            okColors.push_back(leftColor | rightColor);
        break;
        default:
            OP_ASSERT(0);
    }
    int errors = 0;
    for (int y = 0; y < bitHeight - 1; ++y) {
        uint32_t* addr1 = bitmap.getAddr32(0, y);
        for (int x = 0; x < bitWidth - 1; ++x) {
            if (okColors.end() != std::find(okColors.begin(), okColors.end(), addr1[x]))
                continue;
            ++errors;
        }
    }
    if (errors > 9)
        OpDebugOut("");
    return errors;
}

int VerifyOp(const SkPath& one, const SkPath& two, SkPathOp op, std::string testname,
        const SkPath& result) {
    SkPath pathOut, scaledPathOut;
    SkRegion rgnA, rgnB, openClip, rgnOut;
    openClip.setRect({ -16000, -16000, 16000, 16000 });
    rgnA.setPath(one, openClip);
    rgnB.setPath(two, openClip);
    rgnOut.op(rgnA, rgnB, (SkRegion::Op)op);
    rgnOut.getBoundaryPath(&pathOut);
    SkMatrix scale;
    debug_scale_matrix(one, &two, scale);
    SkRegion scaledRgnA, scaledRgnB, scaledRgnOut;
    SkPath scaledA, scaledB;
    scaledA.addPath(one, scale);
    scaledA.setFillType(one.getFillType());
    scaledB.addPath(two, scale);
    scaledB.setFillType(two.getFillType());
    scaledRgnA.setPath(scaledA, openClip);
    scaledRgnB.setPath(scaledB, openClip);
    scaledRgnOut.op(scaledRgnA, scaledRgnB, (SkRegion::Op)op);
    scaledRgnOut.getBoundaryPath(&scaledPathOut);
    SkBitmap bitmap;
    SkPath scaledOut;
    scaledOut.addPath(result, scale);
    scaledOut.setFillType(result.getFillType());
    int errors = debug_paths_draw_the_same(scaledPathOut, scaledOut, bitmap);
    return errors;
}

void threadablePathOpTest(int id, const SkPath& a, const SkPath& b, 
        SkPathOp op, std::string testname, bool v0MayFail, bool skiaMayFail) {
#if OP_TEST_V0
    SkPath result;
	OpInPath op1(&a);
	OpInPath op2(&b);
	OpOutPath opOut(&result);
    std::vector<OpDebugWarning> warnings;
#if OP_DEBUG || OP_TEST_REGION
    bool success = 
#endif
#if OP_DEBUG
        DebugPathOps(op1, op2, (OpOperator) op, opOut, v0MayFail ? OpDebugExpect::unknown :
                OpDebugExpect::success, testname, warnings);
#else
        PathOps(op1, op2, (OpOperator) op, opOut);
#endif
    OP_ASSERT(success || v0MayFail);
#endif
#if OP_TEST_SKIA
    SkPath skresult;
    bool skSuccess = Op(a, b, op, &skresult);
    OP_ASSERT(skSuccess || skiaMayFail);
#if OP_TEST_V0
    if (success && !skSuccess)
        testsPassSkiaFail++;
    else if (!success && skSuccess)
        testsFailSkiaPass++;
#else
    if (skiaMayFail && skSuccess)
        testsFailSkiaPass++;
#endif
#elif OP_TEST_REGION
    bool skSuccess = true;
#endif
#if OP_TEST_V0 && OP_TEST_REGION
    if (!success || !skSuccess || v0MayFail || skiaMayFail)
        return;
    int errors = VerifyOp(a, b, op, testname, result);
//  int altErrors = VerifyOpNoRegion(a, b, op, result);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS || warnings.size()) {
#if !defined(NDEBUG) || OP_RELEASE_TEST
        ReportError(testname, errors, warnings);
        if (errors > MAX_ERRORS)
            testsError++;
#endif
    }

#endif
}

bool testPathOpBase(skiatest::Reporter* r, const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* name, bool v0MayFail, bool skiaMayFail) {
    if (skipTest(name))
        return true;
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
    std::string testname(name);
    a.updateBoundsCache();
    b.updateBoundsCache();
    threadpool.push(threadablePathOpTest, a, b, op, testname, v0MayFail, skiaMayFail);
#else
    threadablePathOpTest(0, a, b, op, name, v0MayFail, skiaMayFail);
#endif
    return true;
}

bool ranFirstTest = false;

bool testPathOp(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* testname) {
    if (ranFirstTest)
        return true;
    std::string s = std::string(testname);
    if (s == testFirst)
        ranFirstTest = true;
    std::vector<std::string> skip = { TEST_PATH_OP_EXCEPTIONS };  // see OpTestDrive.h
    if (skip.end() != std::find(skip.begin(), skip.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> lap = { LAPTOP_PATH_OP_EXCEPTIONS };  // see OpTestDrive.h
    if (!runningWithFMA() && lap.end() != std::find(lap.begin(), lap.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> fuzz = { TEST_PATH_OP_MAP_TO_FUZZ };  // see OpTestDrive.h
    if (fuzz.end() != std::find(fuzz.begin(), fuzz.end(), s) && s != testFirst)
        return (void) testPathOpFuzz(r, a, b, op, testname), true;
    std::vector<std::string> lapz = { LAPTOP_PATH_OP_MAP_TO_FUZZ };  // see OpTestDrive.h
    if (!runningWithFMA() && lapz.end() != std::find(lapz.begin(), lapz.end(), s) && s != testFirst)
        return (void) testPathOpFuzz(r, a, b, op, testname), true;
    return testPathOpBase(r, a, b, op, testname, false, false);
}

void testPathOpCheck(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname, bool checkFail) {
    testPathOpBase(r, a, b, op, testname, false, false);
}

void testPathOpFuzz(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname) {
    testPathOpBase(r, a, b, op, testname, true, true);
}

bool testPathOpFail(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        const SkPathOp op, const char* testName) {
    std::string s = std::string(testName);
    std::vector<std::string> fail = { TEST_PATH_OP_FAIL_EXCEPTIONS };  // see OpTestDrive.h
    if (fail.end() != std::find(fail.begin(), fail.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    return testPathOpBase(r, a, b, op, testName, false, true);
}

void RunTestSet(skiatest::Reporter* r, TestDesc tests[], size_t count,
        void (*firstTest)(skiatest::Reporter* , const char* testName),
        void (*skipTest)(skiatest::Reporter* , const char* testName),
        void (*stopTest)(skiatest::Reporter* , const char* testName), bool reverse) {
    for (size_t i = 0; i < count; ++i)
        (*tests[i].fun)(r, tests[i].str);
}


int VerifySimplify(const SkPath& one, std::string testname, const SkPath& result) {
    SkPath pathOut, scaledPathOut;
    SkRegion rgnA, openClip, rgnOut;
    openClip.setRect({ -16000, -16000, 16000, 16000 });
    rgnA.setPath(one, openClip);
    rgnA.getBoundaryPath(&pathOut);
    SkMatrix scale;
    debug_scale_matrix(one, nullptr, scale);
    SkRegion scaledRgnA, scaledRgnB, scaledRgnOut;
    SkPath scaledA;
    scaledA.addPath(one, scale);
    scaledA.setFillType(one.getFillType());
    scaledRgnA.setPath(scaledA, openClip);
    scaledRgnA.getBoundaryPath(&scaledPathOut);
    SkBitmap bitmap;
    SkPath scaledOut;
    scaledOut.addPath(result, scale);
    scaledOut.setFillType(result.getFillType());
    int errors = debug_paths_draw_the_same(scaledPathOut, scaledOut, bitmap);
    return errors;
}

void threadableSimplifyTest(int id, const SkPath& path, std::string testname, 
            SkPath& out, bool v0MayFail, bool skiaMayFail) {
#if OP_TEST_V0
	OpInPath op1(&path);
    out.reset();
	OpOutPath opOut(&out);
    std::vector<OpDebugWarning> warnings;
#if OP_DEBUG || OP_TEST_REGION
    bool success =
#endif
#if OP_DEBUG
        DebugPathSimplify(op1, opOut, v0MayFail ? OpDebugExpect::unknown :
                OpDebugExpect::success, testname, warnings);
#else
        PathSimplify(op1, opOut);
#endif
    OP_ASSERT(v0MayFail || success);
#endif
#if OP_TEST_SKIA
    SkPath skOut;
    bool skSuccess = Simplify(path, &skOut);
    OP_ASSERT(skiaMayFail || skSuccess);
#if OP_TEST_V0
    if (success && !skSuccess)
        testsPassSkiaFail++;
    else if (!success && skSuccess)
        testsFailSkiaPass++;
#else
    if (skiaMayFail && skSuccess)
        testsFailSkiaPass++;
#endif
#endif
#if OP_TEST_V0 && OP_TEST_REGION
    if (!success)
        return;
    int errors = VerifySimplify(path, testname, out);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS) {
    #if !defined(NDEBUG) || OP_RELEASE_TEST
        ReportError(testname, errors, warnings);
        testsError++;
    }
    #endif
#endif
}

bool testSimplify(SkPath& path, bool useXor, SkPath& out, PathOpsThreadState& state, 
        const char* name) {
    std::string testname(name);
    if ("" == testname) {
        testname = state.fReporter->testname + STR(++state.fReporter->unnamedCount);
    }
    if (skipTest(testname.c_str()))
        return true;
    path.setFillType(useXor ? SkPathFillType::kEvenOdd : SkPathFillType::kWinding);
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
    path.updateBoundsCache();
    threadpool.push(threadableSimplifyTest, path, testname, out, false, false);
#else
    threadableSimplifyTest(0, path, testname.c_str(), out, false, false);
#endif
    return true;
}

bool testSimplifyBase(skiatest::Reporter* r, const SkPath& path, const char* name, 
        bool v0MayFail, bool skiaMayFail) {
    if (skipTest(name))
        return true;
    SkPath out;
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
    std::string testname(name);
    path.updateBoundsCache();
    threadpool.push(threadableSimplifyTest, path, testname, out, v0MayFail, skiaMayFail);
#else
    threadableSimplifyTest(0, path, name, out, v0MayFail, skiaMayFail);
#endif
    return true;
}

bool testSimplify(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    std::string s = std::string(testname);
    std::vector<std::string> fail = { TEST_PATH_SIMPLIFY_EXCEPTIONS };  // see OpTestDrive.h
    if (fail.end() != std::find(fail.begin(), fail.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> lap = { LAPTOP_SIMPLIFY_EXCEPTIONS };  // see OpTestDrive.h
    if (!runningWithFMA() && lap.end() != std::find(lap.begin(), lap.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> fuzz = { TEST_PATH_SIMPLIFY_MAP_TO_FUZZ };  // see OpTestDrive.h
    if (fuzz.end() != std::find(fuzz.begin(), fuzz.end(), s) && s != testFirst)
        return (void) testSimplifyFuzz(r, path, testname), true;
    return testSimplifyBase(r, path, testname, false, false);
}

bool testSimplifyFail(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    std::string s = std::string(testname);
    std::vector<std::string> fail = { TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS };  // see OpTestDrive.h
    if (fail.end() != std::find(fail.begin(), fail.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    return testSimplifyBase(r, path, testname, true, true);
}

bool testSimplifyFuzz(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    std::string s = std::string(testname);
    std::vector<std::string> fuzz = { TEST_PATH_SIMPLIFY_FUZZ_EXCEPTIONS };  // see OpTestDrive.h
    if (fuzz.end() != std::find(fuzz.begin(), fuzz.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    return testSimplifyBase(r, path, testname, true, true);
}

PathOpsThreadedRunnable** DebugOneShot::append() {
    if (slot)
        (*slot->fun)(&data);
    delete slot;
    return &slot;
}

PathOpsThreadedRunnable::PathOpsThreadedRunnable(void (*f)(PathOpsThreadState *),
            int a, int b, int c, int d, PathOpsThreadedTestRunner* runner) {
    fun = f;
    runner->fRunnables.data.fA = a;
    runner->fRunnables.data.fB = b;
    runner->fRunnables.data.fC = c;
    runner->fRunnables.data.fD = d;
}

PathOpsThreadedTestRunner::PathOpsThreadedTestRunner(skiatest::Reporter* r) {
    fRunnables.slot = nullptr;
    fRunnables.data.fReporter = r;
}

void PathOpsThreadedTestRunner::render() {
    fRunnables.append();
}
