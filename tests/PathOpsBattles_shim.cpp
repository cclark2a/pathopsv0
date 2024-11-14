// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsBattles.cpp"

void run_battle_tests(skiatest::Reporter* reporter) {
    test_PathOpsBattle(reporter);
}
