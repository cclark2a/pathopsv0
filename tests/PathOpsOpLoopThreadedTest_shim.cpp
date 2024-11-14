// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsOpLoopThreadedTest.cpp"

// !!! mistake in dumptest() accidentally isolated individual tests as:
//     op(a, a+b, operator)
//     the resultant coincidence exposes many failures
//     eventually, circle back and run (all?) tests like this on purpose (once everything works!)
void run_op_loop_tests(skiatest::Reporter* reporter) {
    test_PathOpsOpLoopsThreaded(reporter);
}
