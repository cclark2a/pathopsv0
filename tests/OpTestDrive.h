// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

#define OP_DEBUG_FAST_TEST 0

#if defined OP_TINY_TEST && OP_TINY_TEST
#define TEST_RASTER 0
#else
#define TEST_RASTER 1  // work-in-progress to verify correctness (buggy)
#endif

#endif
