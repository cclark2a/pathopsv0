// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsTigerTest.cpp"

void run_all_tiger_tests() {
    initializeTests(nullptr, "tiger");
    test_PathOpsTiger(nullptr);
}
