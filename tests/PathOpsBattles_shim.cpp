// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsBattles.cpp"

void run_all_battle_tests() {
    initializeTests(nullptr, "battle");
    test_PathOpsBattle(nullptr);
}
