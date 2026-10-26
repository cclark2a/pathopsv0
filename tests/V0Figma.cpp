// (c) 2026, Cary Clark cclark2@gmail.com
#include "TinySkiaTests.h"

#include "pathopsv0-perf-cases/FigmaPerfCases.cpp"

void V0Figma(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC(figma_zigzag32_offset_0),
        TEST_FUNC(figma_zigzag64_offset_0),
        TEST_FUNC(figma_zigzag128_offset_0),
        TEST_FUNC(figma_spiral1_outline_0),
        TEST_FUNC(figma_spiral2_outline_0),
        TEST_FUNC(figma_wobble_circle_offset_0),
        TEST_FUNC(figma_circle_grid_offset_0),
        TEST_FUNC(figma_glyph_s_0),
        TEST_FUNC(figma_glyph_G_0),
        TEST_FUNC(figma_alphabet_w12_0),
    };
    track->runTests(tests);
}
