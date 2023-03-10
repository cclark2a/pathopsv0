#ifndef OpDebug_DEFINED
#define OpDebug_DEFINED

#include <assert.h>

#define PATH_OPS_V0_FOR_SKIA 1
#define PATH_OPS_V0_FOR_PENTEK 2

// set targeted platform here
#define PATH_OPS_V0_TARGET PATH_OPS_V0_FOR_SKIA

#define OP_RELEASE_TEST 1	// !!! set to zero to remove tests from release build

#define OP_DEBUG_COMPARE 0	// set to one to compare successive runs
#if OP_DEBUG_COMPARE
#include "OpDebugCompare.h"
#endif

#define OP_DEBUG_EDGE_INTERSECT 0	// set to analyze intersection crossings

#if !defined(NDEBUG) || OP_RELEASE_TEST
#include <string>

void OpPrintOut(const std::string& );
uint64_t OpInitTimer();
uint64_t OpReadTimer();
float OpTicksToSeconds(uint64_t ticks, uint64_t frequency);
#endif

#ifdef NDEBUG
#define OP_DEBUG_PARAMS(...)
#define OP_DEBUG_CODE(x)
#define OP_DEBUG 0
#define OP_DEBUG_DUMP 0
#define OP_DEBUG_IMAGE 0
#define OpDebugBreak(edge, id, doBreak)
#define OpDebugOut(string)
#else

#ifdef _WIN32
#include <intrin.h>
#define OP_DEBUG_BREAK() __debugbreak()
#else
#error "add included needed for __builtin_trap()"
#define OP_DEBUG_BREAK() __builtin_trap()
#endif

#define OP_DEBUG_PARAMS(...) , __VA_ARGS__
#define OP_DEBUG_CODE(x) x
#define OP_DEBUG 1
#define OP_DEBUG_DUMP 1
#define OP_DEBUG_IMAGE (PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA)

#include <string>

struct OpContours;

enum class OpDebugIntersect {
	segment,
	edge
};

// debug as written only works when testing uses a single thread
extern OpContours* debugGlobalContours;
extern OpDebugIntersect debugGlobalIntersect;


struct OpDebugIntersectSave {
	OpDebugIntersectSave(OpDebugIntersect tempState) {
		saved = debugGlobalIntersect;
		debugGlobalIntersect = tempState;
	}

	~OpDebugIntersectSave() {
		debugGlobalIntersect = saved;
	}

	OpDebugIntersect saved;
};

struct OpDebugPathOpsEnable {
	OpDebugPathOpsEnable() {
		inPathOps = true;
	}
	~OpDebugPathOpsEnable() {
		inPathOps = false;
	}

	static bool inPathOps;
};

float OpDebugBitsToFloat(int32_t);
std::string OpDebugDump(float);
std::string OpDebugDumpHex(float);
std::string OpDebugDumpHexToFloat(float);
int32_t OpDebugFloatToBits(float);
int32_t OpDebugHexToInt(const char*& str);
float OpDebugHexToFloat(const char*& str);
void OpDebugOut(const std::string&);
void OpDebugSkip(const char*& str, const char* match);
std::string OpDebugToString(float value, int precision);
inline std::string OpDebugStr(int32_t x) { return std::to_string(x); }
inline std::string OpDebugStr(size_t x) { return std::to_string(x); }
inline std::string OpDebugStr(float x) { return std::to_string(x); }
#define STR(x) OpDebugStr(x)

#define OpDebugBreak(opObject, ID, doBreak) \
		do { if (doBreak && ID == (opObject)->id) OP_DEBUG_BREAK(); } while (false)

#endif


// if identical runs produce different results, use this to help determine where 
// the successive runs first differ
#if OP_DEBUG_COMPARE
#define OP_DEBUG_DUMP_COUNT(contours, label) contours.dumpCount(#label)

#else
#define OP_DEBUG_DUMP_COUNT(contours, label)
#endif

#endif
