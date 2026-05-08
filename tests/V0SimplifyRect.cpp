// (c) 2026, Cary Clark cclark2@gmail.com
// originally Skia's PathOpsSimplifyRectThreadedTest.cpp;
// the test below is optimized for speed, reduced memory, and random access
#include "TinySkiaTests.h"

// four rects, of four sizes
// for 3 smaller sizes, tall, wide
    // top upper mid lower bottom aligned (3 bits, 5 values)
    // same with x (3 bits, 5 values)
// not included, square, tall, wide (2 bits)
// cw or ccw (1 bit)

#define CONSTRUCT_SKIPPER_DATA 0

static void simplifyRects(TestOptions* options) {
    auto test = [options](int oA, int oB, int oC, int oD) {
#if CONSTRUCT_SKIPPER_DATA
        options->skip = 100000;
#endif
        int aShape = oA & 0x03;
        SkPathDirection aCW = oA >> 2 ? SkPathDirection::kCCW : SkPathDirection::kCW;
        int bShape = oB & 0x03;
        SkPathDirection bCW = oB >> 2 ? SkPathDirection::kCCW : SkPathDirection::kCW;
        int cShape = oC & 0x03;
        SkPathDirection cCW = oC >> 2 ? SkPathDirection::kCCW : SkPathDirection::kCW;
        int dShape = oD & 0x03;
        SkPathDirection dCW = oD >> 2 ? SkPathDirection::kCCW : SkPathDirection::kCW;
        for (float aXAlign = 0; aXAlign < 5; ++aXAlign) {
            for (float aYAlign = 0; aYAlign < 5; ++aYAlign) {
                for (float bXAlign = 0; bXAlign < 5; ++bXAlign) {
                    for (float bYAlign = 0; bYAlign < 5; ++bYAlign) {
                        for (float cXAlign = 0; cXAlign < 5; ++cXAlign) {
                            for (float cYAlign = 0; cYAlign < 5; ++cYAlign) {
                                for (float dXAlign = 0; dXAlign < 5; ++dXAlign) {
    for (float dYAlign = 0; dYAlign < 5; ++dYAlign) {
        SkPath path;
        float l  OP_DEBUG_INIT_FLOAT();
        float t  OP_DEBUG_INIT_FLOAT();
        float r  OP_DEBUG_INIT_FLOAT();
        float b  OP_DEBUG_INIT_FLOAT();
        if (aShape) {
            switch (aShape) {
                case 1: l = 0; r = 60; t = 0; b = 60; aXAlign = 5; aYAlign = 5; break; // square
                case 2: l = aXAlign * 12; r = l + 30; t = 0; b = 60; aYAlign = 5; break;
                case 3: l = 0; r = 60; t = aYAlign * 12; b = l + 30; aXAlign = 5; break;
            }
            path.addRect(l, r, t, b, aCW);
        } else {
            aXAlign = 5; aYAlign = 5;
        }
        if (bShape) {
            switch (bShape) {
                case 1: l = bXAlign * 10; r = l + 20; t = bYAlign * 10; b = l + 20; break; // square
                case 2: l = bXAlign * 10; r = l + 20; t = 10; b = 40; bYAlign = 5; break;
                case 3: l = 10; r = 40; t = bYAlign * 10; b = l + 20; bXAlign = 5; break;
            }
            path.addRect(l, r, t, b, bCW);
        } else {
            bXAlign = 5; bYAlign = 5;
        }
        if (cShape) {
            switch (cShape) {
                case 1: l = cXAlign * 6; r = l + 12; t = cYAlign * 6; b = l + 12; break;  // square
                case 2: l = cXAlign * 6; r = l + 12; t = 20; b = 30; cYAlign = 5; break;
                case 3: l = 20; r = 30; t = cYAlign * 6; b = l + 20; cXAlign = 5; break;
            }
            path.addRect(l, r, t, b, cCW);
        } else {
            cXAlign = 5; cYAlign = 5;
        }
        if (dShape) {
            switch (dShape) {
                case 1: l = dXAlign * 4; r = l + 9; t = dYAlign * 4; b = l + 9; break;  // square
                case 2: l = dXAlign * 6; r = l + 9; t = 32; b = 36; dYAlign = 5; break;
                case 3: l = 32; r = 36; t = dYAlign * 6; b = l + 9; dXAlign = 5; break;
            }
            path.addRect(l, r, t, b, dCW);
        } else {
            dXAlign = 5; dYAlign = 5;
        }
        path.close();
        if (options->skipTests(1))
            continue;
        if (TestDone::yes == options->testOne(path))
            return TestDone::yes;
        if (options->skipTests(1))
            continue;
        if (TestDone::yes == options->testOne(path))
            return TestDone::yes;
    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return TestDone::no;
    };
    int skipOuter = 0;
#if CONSTRUCT_SKIPPER_DATA
    std::string skipTable;
    std::string skipLine = "        ";
#else
    const int skipper[] = {
        1, 25, 5, 5, 1, 25, 5, 5, 625, 125, 125, 25, 625, 125, 125, 25, 25, 5, 125, 25, 25, 25, 5, 
        125, 25, 25, 1, 25, 5, 5, 625, 125, 125, 25, 25, 25, 15625, 3125, 3125, 625, 15625, 3125, 
        3125, 625, 625, 125, 3125, 625, 625, 625, 125, 3125, 625, 625, 25, 625, 125, 125, 15625, 
        3125, 3125, 625, 625, 625, 125, 125, 25, 625, 125, 125, 125, 25, 625, 125, 125, 5, 125, 25, 
        25, 3125, 625, 625, 125, 125, 125, 125, 25, 625, 125, 125, 5, 125, 25, 25, 3125, 625, 625, 
        125, 125, 125, 1, 25, 5, 5, 625, 125, 125, 25, 25, 25, 15625, 3125, 3125, 625, 625, 625, 
        125, 125, 125, 125, 15625, 3125, 3125, 625, 15625, 3125, 3125, 625, 625, 125, 3125, 625, 
        625, 625, 125, 3125, 625, 625, 25, 625, 125, 125, 15625, 3125, 3125, 625, 625, 625, 125, 
        125, 25, 625, 125, 125, 125, 25, 625, 125, 125, 5, 125, 25, 25, 3125, 625, 625, 125, 125, 
        125, 125, 25, 625, 125, 125, 5, 125, 25, 25, 3125, 625, 625, 125, 125, 125, 1, 25, 5, 5, 
        625, 125, 125, 25, 25, 25, 15625, 3125, 3125, 625, 625, 625, 125, 125, 125, 125, 625, 625, 
        125, 3125, 625, 625, 625, 125, 3125, 625, 625, 25, 625, 125, 125, 15625, 3125, 3125, 625, 
        625, 625, 625, 125, 3125, 625, 625, 25, 625, 125, 125, 15625, 3125, 3125, 625, 625, 625, 5, 
        125, 25, 25, 3125, 625, 625, 125, 125, 125, 78125, 15625, 15625, 3125, 3125, 3125, 625, 
        625, 625, 625, 625, 125, 3125, 625, 625, 25, 625, 125, 125, 15625, 3125, 3125, 625, 625, 
        625, 5, 125, 25, 25, 3125, 625, 625, 125, 125, 125, 78125, 15625, 15625, 3125, 3125, 3125, 
        625, 625, 625, 625, 1, 25, 5, 5, 625, 125, 125, 25, 25, 25, 15625, 3125, 3125, 625, 625, 
        625, 125, 125, 125, 125, 15625, 3125, 3125, 625, 625, 625, 125, 125, 125, 125, 625, 625, 
        625, 625, 625, 
    };
    int innerTest = 0;
#endif
    for (int a = 0; a < 8; ++a) {  // outermost
        for (int b = a ; b < 8; ++b) {
            for (int c = b ; c < 8; ++c) {
                for (int d = c; d < 8; ++d) {
#if CONSTRUCT_SKIPPER_DATA
                    options->skip = -1;
#else
                    skipOuter = skipper[innerTest++];
#endif
                    if (!options->skipTests(skipOuter) && TestDone::yes == test(a, b, c, d))
                        return;
#if CONSTRUCT_SKIPPER_DATA
                    std::string skipWord = STR(100000 - options->skip) + ", ";
                    if (skipWord.size() + skipLine.size() > 100) {
                        skipTable += skipLine + "\n";
                        skipLine = "        ";
                    }
                    skipLine += skipWord;
#endif
                }
                if (!options->extended()) 
                    return;
            }
        }
    }
#if CONSTRUCT_SKIPPER_DATA
    OpDebugOut(skipTable + skipLine + "\n");
#endif
}

void V0SimplifyRects(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC_NUMBERED(simplifyRects),
    };
    track->runTests(tests);
}
