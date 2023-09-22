// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebug_DEFINED
#define OpDebug_DEFINED

#ifndef _WIN32
#include <assert.h>
#endif

#include "OpTestDrive.h"  // set test specific settings here

#if !defined(NDEBUG) || OP_RELEASE_TEST
#include <string>
void OpDebugOut(const std::string& );
void OpPrintOut(const std::string& );
uint64_t OpInitTimer();
uint64_t OpReadTimer();
float OpTicksToSeconds(uint64_t ticks, uint64_t frequency);
#endif

enum class OpDebugIntersect {
	segment,
	edge
};

struct OpContours;

#ifdef NDEBUG

#define OP_DEBUG_PARAMS(...)
#define OP_DEBUG_CODE(...)
#define OP_DEBUG_VALIDATE_CODE(...)
#define OP_DEBUG 0
#define OP_DEBUG_DUMP 0
#define OP_DEBUG_IMAGE 0
#define OP_DEBUG_INITIALIZE_TO_SILENCE_WARNING
#define OP_ASSERT(expr)
#define OP_DEBUG_FAIL(object, returnValue) return returnValue
#define OP_DEBUG_SUCCESS(object, returnValue) return returnValue

#else

#ifdef _WIN32
#include <intrin.h>
#define OP_DEBUG_BREAK() __debugbreak()
#define OP_ASSERT(expr) do { if (!(expr)) __debugbreak(); } while (false)

#else
#define OP_DEBUG_BREAK() __builtin_trap()
#define OP_ASSERT(expr) assert(expr)
#endif

#if OP_DEBUG_FAST_TEST
	#define OP_DEBUG_DUMP 0
	#define OP_DEBUG_IMAGE 0
	#define OP_DEBUG_VALIDATE 0
#else
	#define OP_DEBUG_DUMP 1
	#define OP_DEBUG_IMAGE 1
	#define OP_DEBUG_VALIDATE 1
#endif
#define OP_DEBUG_PARAMS(...) , __VA_ARGS__
#define OP_DEBUG_CODE(...) __VA_ARGS__
#if OP_DEBUG_VALIDATE
	#define OP_DEBUG_VALIDATE_CODE(...) __VA_ARGS__
#else
	#define OP_DEBUG_VALIDATE_CODE(...)
#endif
#define OP_DEBUG 1
#define OP_DEBUG_INITIALIZE_TO_SILENCE_WARNING = 0

// Use these defines where failure or success is logical, but we want it to break to verify
// that the decision is correct. Once verified, the macro is replaced with an error return.
#define OP_DEBUG_FAIL(object, returnValue) \
	do { if (!(object).debugFail()) OP_DEBUG_BREAK(); return returnValue; } while (false)
#define OP_DEBUG_SUCCESS(object, returnValue) \
	do { if (!(object).debugSuccess()) OP_DEBUG_BREAK(); return returnValue; } while (false)

#include <string>

// debug compare, debug dump, and debug image as written only work when testing uses a single thread
extern OpDebugIntersect debugGlobalIntersect;

enum class OpDebugExpect {
	unknown,
	fail,
	success,
};

#define OpDebugBreak(opObject, ID) \
	do { if ((ID) == (opObject)->id) OP_DEBUG_BREAK(); } while (false)

#define OpDebugBreakIf(opObject, ID, doBreak) \
	do { if ((doBreak) && (ID) == (opObject)->id) OP_DEBUG_BREAK(); } while (false)

#define OpDebugPlayback(opObject, ID) \
	do { if ((ID) == (opObject)->id) { playback(); OP_DEBUG_BREAK(); } } while (false)

#define OpDebugPlaybackIf(opObject, ID, doBreak) \
	do { if ((doBreak) && (ID) == (opObject)->id) { playback(); OP_DEBUG_BREAK(); } } while (false)

#endif

#if OP_DEBUG || OP_DEBUG_DUMP || OP_DEBUG_IMAGE
class SkPath;

#define STR(x) OpDebugStr(x)
#if OP_DEBUG
#define OP_DEBUG_STR_ID(x) OpDebugStr(x->id)
#else
#define OP_DEBUG_STR_ID(x) OpDebugStr(x)
#endif

struct OpDebugMaker {
	OpDebugMaker()
		: line(0) {
	}

	OpDebugMaker(std::string f, int l)
		: file(f)
		, line(l) {
	}

#if OP_DEBUG_DUMP
	std::string debugDump() const;
#endif
	std::string file;
	int line;
};

extern OpContours* debugGlobalContours;

float OpDebugBitsToFloat(int32_t);
std::string OpDebugDump(float);
std::string OpDebugDumpHex(float);
std::string OpDebugDumpHexToFloat(float);
int32_t OpDebugFloatToBits(float);
float OpDebugHexToFloat(const char*& str);
int32_t OpDebugHexToInt(const char*& str);
void OpDebugSkip(const char*& str, const char* match);
inline std::string OpDebugStr(void* x) { return std::to_string((unsigned long long)(void**)x); }
inline std::string OpDebugStr(int32_t x) { return std::to_string(x); }
inline std::string OpDebugStr(size_t x) { return std::to_string(x); }
inline std::string OpDebugStr(float x) { return std::to_string(x); }
std::string OpDebugToString(float value, int precision);
#endif


// if identical runs produce different results, use this to help determine where 
// the successive runs first differ
#if OP_DEBUG_COMPARE
#define OP_DEBUG_DUMP_COUNT(contours, label) contours.dumpCount(#label)

#else
#define OP_DEBUG_DUMP_COUNT(contours, label)
#endif

#endif
