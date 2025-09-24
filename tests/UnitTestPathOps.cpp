// (c) 2023, Cary Clark cclark2@gmail.com
//       1         2         3         4         5         6         7         8         9         0
//34567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#include "OpDebug.h"

std::string native_debugDump(size_t index) { return ""; }
size_t native_addText(std::string str, uint32_t color)  { return 0; }
struct NativeTextCache { int d = 0; } dummy;
const NativeTextCache& native_cache(size_t index) { return dummy; }

#define TEST_SMALL_EXAMPLES 0
#define TEST_SMALL_TESTS 0

extern void ContainsExample();
extern void CutExample();
extern void FrameExample();
extern void HTMLCanvasExample();
extern void SimplifyExample();
extern void SkiaIntersectExample();
extern void SkiaSimplifyExample();
extern void SVGExample();
extern void TestCut();
extern void TestFrame();
extern void TestPath2D(bool debugIt);
extern void runTests();

void OpTest() {
#if 0 && OP_DEBUG_IMAGE
    if (GENERATE_COLOR_FILES) {
        OpDebugGenerateColorFiles();
        return;
    }
#endif
#if TEST_SMALL_EXAMPLES
    CutExample();
    SimplifyExample();
    ContainsExample();
    HTMLCanvasExample();
    SVGExample();
    SkiaIntersectExample();
    SkiaSimplifyExample();
  	FrameExample();
#endif
#if TEST_SMALL_TESTS
    TestCut();
 	TestPath2D(true);
    TestFrame();
#endif
	runTests();
}
