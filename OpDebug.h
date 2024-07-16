// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebug_DEFINED
#define OpDebug_DEFINED

#ifndef _WIN32
#include <assert.h>
#endif

#if OP_TINY_TEST
#define OP_DEBUG_FAST_TEST 1
#else
#include "OpTestDrive.h"  // set test specific settings here
#endif
#define OP_TEST_NEW_INTERFACE 1  // use context-free engine design; test hard-coded for now

#if !defined(NDEBUG) || OP_RELEASE_TEST
#include <string>
#include <vector>

enum class OpDebugExpect {
	unknown,
	fail,
	success,
};

enum class OpDebugWarning {
	lastResort
};

float OpDebugBitsToFloat(int32_t);
void OpDebugOut(const std::string& );
uint64_t OpInitTimer();
uint64_t OpReadTimer();
float OpTicksToSeconds(uint64_t ticks, uint64_t frequency);

#define STR(x) OpDebugStr(x)
#define STR_E(x) OpDebugStr((int) (x))  // use with enums
inline std::string OpDebugStr(void* x) { return std::to_string((unsigned long long)(void**)x); }
inline std::string OpDebugStr(int32_t x) { return std::to_string(x); }
inline std::string OpDebugStr(size_t x) { return std::to_string(x); }
std::string OpDebugStr(float value);
extern int debugPrecision;		// minus one means unset

#endif

enum class OpDebugIntersect {
	segment,
	edge
};

struct OpContours;

struct OpDebugData {
	OpDebugData(bool mayFail) 
		: debugExpect(mayFail ? OpDebugExpect::fail : OpDebugExpect::success)
		, debugSuccess(true) {
	}

	std::vector<OpDebugWarning> debugWarnings;
	std::string debugTestname;
	OpDebugExpect debugExpect;
	int debugCurveCurve1;
	int debugCurveCurve2;
	int debugCurveCurveDepth;
	bool debugSuccess;
};

#define OP_DEBUG_CONTEXT(...)
#define OP_DEBUG_DUMP_CODE(...)
#define OP_DEBUG_DUMP_PARAMS(...)
#define OP_DEBUG_IMAGE_CODE(...)
#define OP_DEBUG_IMAGE_PARAMS(...)

#ifdef NDEBUG

#define OP_ASSERT(expr)
#define OP_DEBUG_PARAMS(...)
#define OP_DEBUG_CODE(...)
#define OP_DEBUG_VALIDATE_CODE(...)
#define OP_DEBUG 0
#define OP_DEBUG_DUMP 0
#define OP_DEBUG_IMAGE 0
#define OP_DEBUG_FAIL(object, returnValue) return returnValue
#define OP_DEBUG_SUCCESS(object, returnValue) return returnValue
#define OP_EXECUTE_AND_ASSERT(expr) (expr)
#define OP_LINE_FILE_PARAMS(...)
#define OP_LINE_FILE_NPARAMS(...)
#define OP_LINE_FILE_STRUCT(...)
#define OP_LINE_FILE_CALLER(...)
#define OP_LINE_FILE_NP_CALLER(...)
#define OP_LINE_FILE_SCALLER(...)
#define OP_LINE_FILE_DEF(...)
#define OP_LINE_FILE_NP_DEF(...)
#define OP_TRACK(vector)
#define OP_WARNING(contours, str)

#else


#ifdef _WIN32
#include <intrin.h>
#define OP_DEBUG_BREAK() __debugbreak()
#define OP_ASSERT(expr) do { if (!(expr)) __debugbreak(); } while (false)
#else
#define OP_DEBUG_BREAK() __builtin_trap()
#define OP_ASSERT(expr) assert(expr)
#endif

#define OP_EXECUTE_AND_ASSERT(expr) OP_ASSERT(expr)
#define OP_WARNING(contours, warn) contours->addDebugWarning(OpDebugWarning::warn)

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
#if OP_DEBUG_IMAGE
#undef OP_DEBUG_IMAGE_CODE
#define OP_DEBUG_IMAGE_CODE(...) __VA_ARGS__
#undef OP_DEBUG_IMAGE_PARAMS
#define OP_DEBUG_IMAGE_PARAMS(...) , __VA_ARGS__
#endif
#if OP_DEBUG_VALIDATE
	#define OP_DEBUG_VALIDATE_CODE(...) __VA_ARGS__
#else
	#define OP_DEBUG_VALIDATE_CODE(...)
#endif
#define OP_DEBUG 1

// Use these defines where failure or success is logical, but we want it to break to verify
// that the decision is correct. Once verified, the macro is replaced with an error return.
#define OP_DEBUG_FAIL(object, returnValue) \
	do { if (!(object).debugFail()) OP_DEBUG_BREAK(); return returnValue; } while (false)
#define OP_DEBUG_SUCCESS(object, returnValue) \
	do { if (!(object).debugSuccess()) OP_DEBUG_BREAK(); return returnValue; } while (false)

#include <string>

#define OP_LINE_FILE_PARAMS(...) , __LINE__, std::string(__FILE__), __VA_ARGS__
#define OP_LINE_FILE_NPARAMS(...) __LINE__, std::string(__FILE__), __VA_ARGS__
#define OP_LINE_FILE_STRUCT(...) , { __FILE__, __LINE__ }, __VA_ARGS__
#define OP_LINE_FILE_CALLER(...) , lineNo, fileName, __VA_ARGS__
#define OP_LINE_FILE_NP_CALLER(...) lineNo, fileName, __VA_ARGS__
#define OP_LINE_FILE_SCALLER(...) , { fileName, lineNo }, __VA_ARGS__
#define OP_LINE_FILE_DEF(...) , int lineNo, std::string fileName, __VA_ARGS__
#define OP_LINE_FILE_NP_DEF(...) int lineNo, std::string fileName, __VA_ARGS__

// keep track of vector size to find reserve  !!! haven't decided whether or not to build this out
#define OP_TRACK(v)

// debug compare, debug dump, and debug image as written only work when testing uses a single thread
#if !OP_DEBUG_FAST_TEST
#define OpDebugBreak(opObject, ID) \
	do { if ((ID) == (opObject)->id) OP_DEBUG_BREAK(); } while (false)

#define OpDebugBreak2(o1, o2, i1, i2) \
	do { if (((i1) == (o1)->id || (i2) == (o1)->id)) && \
             ((i1) == (o2)->id || (i2) == (o2)->id))) OP_DEBUG_BREAK(); } while (false)

#define OpDebugBreakIf(opObject, ID, doBreak) \
	do { if ((doBreak) && (ID) == (opObject)->id) OP_DEBUG_BREAK(); } while (false)

#if OP_DEBUG_DUMP 
#undef OP_DEBUG_DUMP_CODE
#define OP_DEBUG_DUMP_CODE(...) __VA_ARGS__
#undef OP_DEBUG_DUMP_PARAMS
#define OP_DEBUG_DUMP_PARAMS(...) , __VA_ARGS__

#undef OP_DEBUG_CONTEXT
#define OP_DEBUG_CONTEXT() \
	debugContext = __func__

#define OpDebugPlayback(opObject, ID) \
	do { if ((ID) == (opObject)->id) { playback(); OP_DEBUG_BREAK(); } } while (false)

#define OpDebugPlaybackIf(opObject, ID, doBreak) \
	do { if ((doBreak) && (ID) == (opObject)->id) { playback(); OP_DEBUG_BREAK(); } } while (false)
#endif
#endif
#endif

#if OP_DEBUG || OP_DEBUG_DUMP || OP_DEBUG_IMAGE

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
	void dumpSet(const char*& );
	std::string debugDump() const;
#endif
	std::string file;
	int line;
};

#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
extern OpContours* debugGlobalContours;
extern bool debugHexFloat;
extern void playback();
extern void record();
#endif

int OpDebugCountDelimiters(const char* str, char delimiter, char openBracket, char closeBracket);
void OpDebugExit(std::string);
void OpDebugExitOnFail(std::string, bool );
std::vector<uint8_t> OpDebugByteArray(const char*& str);
std::string OpDebugByteToHex(uint8_t);
std::string OpDebugIntToHex(int32_t);
std::string OpDebugDumpHex(float);
std::string OpDebugDumpByteArray(const char* bytes, size_t size);
int32_t OpDebugFloatToBits(float);
float OpDebugHexToFloat(const char*& str);
uint8_t OpDebugByteToInt(const char*& str);
int32_t OpDebugHexToInt(const char*& str);
int OpDebugReadNamedInt(const char*& str, const char* label);
std::string OpDebugLabel(const char*& str);
bool OpDebugOptional(const char*& str, const char* match);
float OpDebugReadNamedFloat(const char*& str, const char* label);
size_t OpDebugReadSizeT(const char*& str);
void OpDebugRequired(const char*& str, const char* match);
extern int debugPrecision;		// minus one means unset
#endif


// if identical runs produce different results, use this to help determine where 
// the successive runs first differ
#if OP_DEBUG_COMPARE
#define OP_DEBUG_DUMP_COUNT(contours, label) contours.dumpCount(#label)

#else
#define OP_DEBUG_DUMP_COUNT(contours, label)
#endif

#if OP_DEBUG_DUMP
enum class DebugLevel;
enum class DebugBase;

void OpDebugFormat(std::string );
#endif

#endif
