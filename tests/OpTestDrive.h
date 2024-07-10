// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
// (c) 2023, Cary Clark cclark2@gmail.com
#define OpTestDrive_DEFINED

// this puts all switches that decide which tests to run and how to run them in one place
// these switches should migrate to make file options
#define OP_DEBUG_FAST_TEST 0  // in a debug build: set to zero to enable debug dump, debug image
#define TEST_PATH_OP_FIRST "cubic142801"  // e.g., "cubic140721" (ignored by fast test)
#define TEST_PATH_OP_SKIP_TO_V0 0 // if 1 & not fast test, ignore skip to file; run first "v0" test
#define TEST_PATH_OP_SKIP_TO_FILE "cubic" // e.g., "quad" tests only (see testSuites in OpSkiaTests)
#define OP_TEST_NEW_INTERFACE 1  // use context-free engine design; test hard-coded for now

#define OP_DEBUG_RECORD 0  // track some statistic or other while running 
#define OP_DEBUG_VERBOSE (1 && !OP_DEBUG_FAST_TEST)
#define OP_RELEASE_TEST 1	// !!! set to zero to remove tests from release build (untested)

#define OP_DEBUG_COMPARE 0	// set to one to compare successive runs
#if OP_DEBUG_COMPARE
#include "OpDebugCompare.h" // this hasn't been used in a long time and is likely broken
#endif

// an alternative to runningWithFMA()
// only used to attempt to fix debugging on laptop (unsuccessful)
#if defined(__llvm__)
#define OP_HAS_FMA
#else
#define OP_NO_FMA
#endif

#endif
