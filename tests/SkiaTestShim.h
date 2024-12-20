// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef SkiaTestShim_DEFINED
#define SkiaTestShim_DEFINED

#include "SkiaEnumSkPathOp.h"

// below redirections are only visible to consumed tests
#define SkPathOps_DEFINED

#if OP_TINY_SKIA
#include "TinySkia.h"
#define SkString_DEFINED
#define SkFloatBits_DEFINED
#define SkGeometry_DEFINED
#define SkParsePath_DEFINED
#define SkPath_DEFINED
#define SkPoint_DEFINED
#define SkRandom_DEFINED
#define SkPathOpsCubic_DEFINED
#define SkPathOpsQuad_DEFINED
#define SkPathOpsPoint_DEFINED


#else
#include "include/core/SkString.h"
#include "src/pathops/SkPathOpsDebug.h"
#include "src/pathops/SkPathOpsTypes.h"
#endif
#include "OpDebug.h"

// since we're defining our own test harness, don't allow this to be included
#define PathOpsDebug_DEFINED
#define PathOpsExtendedTest_DEFINED
#define PathOpsTestCommon_DEFINED
#define PathOpsThreadedCommon_DEFINED
#define skiatest_Test_DEFINED

#include "SkiaTestCommon.h"  // visible to consumed tests and test handler

// map calls to Op and Simplify within tests to v0
#define Op(a, b, op, result) OpV0(a, b, op, result, nullptr)
#define Simplify(a, result) SimplifyV0(a, result, nullptr)

extern bool OpV0(const SkPath& one, const SkPath& two, SkPathOp op, SkPath* result, OpDebugData* optional); 
extern bool SimplifyV0(const SkPath& path, SkPath* result, OpDebugData* optional);

#endif
