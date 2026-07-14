// (c) 2023, Cary Clark cclark2@gmail.com
//       1         2         3         4         5         6         7         8         9         0
//34567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890

#define TEST_SMALL_EXAMPLES 0
#define TEST_SMALL_TESTS 0
#define TINY_TESTS 1

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
extern void runTinyTests();

int main() {
#if 0
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
#if TINY_TESTS
    runTinyTests();
#else
	runTests();
#endif
    return 0;
}
