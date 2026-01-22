// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebug_DEFINED
#define OpDebug_DEFINED

#ifdef _WIN32
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <assert.h>
#include <cstdint>
#include <cstring>

#ifndef NDEBUG
#define OP_DEBUG 1
#define _GLIBCXX_DEBUG 1
#else
#define OP_DEBUG 0
#endif
#include <vector>

#ifndef OP_DEBUG_FAST_TEST
#define OP_DEBUG_FAST_TEST 0
#endif

#ifndef OP_TINY_TEST
#define OP_TINY_TEST 0
#endif

#ifndef OP_DEBUG_ALT
#define OP_DEBUG_ALT 0
#endif

#ifndef OP_RELEASE_TEST
#define OP_RELEASE_TEST 1	// !!! set to zero to remove tests from release build (untested)
#endif

// OP_DEBUG_FAST_TEST uses threads; all code must be thread-safe
#define OP_DEBUG_VERBOSE (OP_DEBUGGER || !OP_DEBUG_FAST_TEST)
#define OP_DEBUG_GLOBALS (!OP_DEBUG_FAST_TEST)  // globals available while debugging single-threaded
#define OP_TEST (OP_DEBUG || OP_RELEASE_TEST)  // check test results (e.g., scanline compare)

#ifndef OP_TEST_RASTER
#define OP_TEST_RASTER (!OP_TINY_TEST && OP_TEST)
#endif

#define OP_ENUM_BASE(member, value) member = value
#define OP_ENUM_MEMBER(member) member

#if OP_TEST
#include <string>

#define OpDebugExpect_Enums \
	OP_ENUM_MEMBER(unknown), \
	OP_ENUM_MEMBER(fail), \
	OP_ENUM_MEMBER(success),

enum class OpDebugExpect {
    OpDebugExpect_Enums
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
inline std::string OpDebugStr(const char* x) { return std::string(x); }
std::string OpDebugStr(float value);
inline std::string OpDebugStr(double value) { return OpDebugStr((float) value); }

#define OpNop() \
	OpDebugOut("")

#endif

enum class OpDebugIntersect {
	segment,
	edge
};

#if OP_TEST_RASTER
struct OpDebugRaster;
#endif

struct OpDebugData {
	OpDebugData() {}
	OpDebugData(std::string tn, OpDebugExpect expected, int cc1, int cc2, int ccDepth, 
			bool noBreaks, bool noDumps, bool runOne) 
		: testname(tn)
		, expect(expected)
        , curveCurve1(cc1)
        , curveCurve2(cc2)
        , curveCurveDepth(ccDepth)
		, defeatBreak(noBreaks)
		, defeatDumps(noDumps)
        , runOneFile(runOne) {
	}

	std::string testname;
	OpDebugExpect expect = OpDebugExpect::success;
	int curveCurve1 = 0;
	int curveCurve2 = 0;
	int curveCurveDepth = -1;
	int limitContours = 0;
	float error = 0;
	bool defeatBreak = false;
	bool defeatDumps = false;
	bool limitReached = false;
	bool runOneFile = false;
	bool success = true;
};

#define OP_DEBUG_DUMP_CODE(...)
#define OP_DEBUG_DUMP_PARAMS(...)
#define OP_DEBUG_IMAGE_CODE(...)
#define OP_DEBUG_IMAGE_CODE_OLD(...)
#define OP_DEBUG_IMAGE_PARAMS(...)
#define OP_DEBUG_IMAGE_PARAMS_OLD(...)
#define OP_DEBUGGER_CODE(...)
#define OP_DEBUGGER_PARAMS(...)

#if defined OP_TINY_TEST && OP_TINY_TEST
    #define OP_TINY_MAIN(func) int main() { func(); return 0; }
#else
    #define OP_TINY_MAIN(func)
#endif

#if !OP_DEBUG

#define OP_ASSERT(expr)
#define OP_DEBUG_PARAMS(...)
#define OP_LINE_FILE_PARGS()
#define OP_DEBUG_CODE(...)
#define OP_DEBUG_VALIDATE_CODE(...)
#define OP_DEBUG_BREAK()
#define OP_DEBUG_DUMP 0
#define OP_DEBUG_ENUM()
#define OP_DEBUG_IMAGE 0
#define OP_DEBUG_INIT(enum_name)
#define OP_DEBUG_INIT_BOOL()
#define OP_DEBUG_INIT_FLOAT()
#define OP_DEBUG_INIT_INT()
#define OP_DEBUG_INIT_PTR(ptr_type)
#define OP_DEBUG_INITED_PTR(ptr)  error("should never be in release code")
#define OP_DEBUG_INIT_SIZE()
#define OP_DEBUG_INIT_UINT()
#define OP_DEBUG_MAKER 0
#define OP_DEBUG_FAIL(object, returnValue) return returnValue
#define OP_DEBUG_SERIALIZE 0
#define OP_DEBUG_SUCCESS(object, returnValue) return returnValue
#define OP_LINE_FILE_PARAMS(...)
#define OP_LINE_FILE_PARGS()
#define OP_LINE_FILE_NPARAMS(...)
#define OP_LINE_FILE_NPARGS()
#define OP_LINE_FILE_STRUCT(...)
#define OP_LINE_FILE_CALLER(...)
#define OP_LINE_FILE_CARGS()
#define OP_LINE_FILE_NP_CALLER(...)
#define OP_LINE_FILE_NP_CARGS()
#define OP_LINE_FILE_SCALLER(...)
#define OP_LINE_FILE_DEF(...)
#define OP_LINE_FILE_ARGS()
#define OP_LINE_FILE_NP_DEF(...)
#define OP_LINE_FILE_NP_ARGS()
#define OP_LINE_FILE_DECLARE(debugMaker)
#define OP_LINE_FILE_SET(debugMaker)
#define OP_LINE_FILE_SET_IMMED(debugMaker)

#else

#ifdef _WIN32
#include <intrin.h>
#define OP_DEBUG_BREAK() __debugbreak()
#elif __clang__
#define OP_DEBUG_BREAK() __builtin_debugtrap()
#else
#include <signal.h>
#define OP_DEBUG_BREAK() raise(SIGTRAP)
#endif
#define OP_ASSERT(expr) do { if (!(expr)) OP_DEBUG_BREAK(); } while (false)

#define OP_DEBUG_SERIALIZE 1  // !!! migrate debug dump so threaded test can write to debugger

#if (!OP_DEBUGGER && OP_DEBUG_FAST_TEST) || (defined OP_TINY_TEST && OP_TINY_TEST)
	#define OP_DEBUG_DUMP 0
	#define OP_DEBUG_IMAGE 0
	#define OP_DEBUG_MAKER 0
	#define OP_DEBUG_VALIDATE 01
#else
	#define OP_DEBUG_DUMP 1
	#define OP_DEBUG_IMAGE 1
	#define OP_DEBUG_MAKER 1
	#define OP_DEBUG_VALIDATE 1
#endif
#ifndef OP_DEBUGGER
    #define OP_DEBUGGER 0
#endif
#define OP_DEBUG_PARAMS(...) , __VA_ARGS__
#define OP_DEBUG_CODE(...) __VA_ARGS__
#define OP_DEBUG_ENUM() uninitialized = -1,
#define OP_DEBUG_INIT(enum_name) = enum_name::uninitialized
#define OP_DEBUG_INIT_BOOL() = -1
#define OP_DEBUG_INIT_FLOAT() = OpDebugNaN
#define OP_DEBUG_INIT_INT() = INT_MAX
#define OP_DEBUG_INIT_PTR(ptr_type) = (ptr_type*) 0xDEAD0ABEDEADBEEF
#define OP_DEBUG_INITED_PTR(ptr) (!!ptr && (decltype(ptr)) 0xDEAD0ABEDEADBEEF)
#define OP_DEBUG_INIT_SIZE() = SIZE_MAX
#define OP_DEBUG_INIT_UINT() = UINT_MAX

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

// Use these defines where failure or success is logical, but we want it to break to verify
// that the decision is correct. Once verified, the macro is replaced with an error return.
#define OP_DEBUG_FAIL(object, returnValue) \
	do { if (!(object).debugFail()) OP_DEBUG_BREAK(); return returnValue; } while (false)
#define OP_DEBUG_SUCCESS(object, returnValue) \
	do { if (!(object).debugSuccess()) OP_DEBUG_BREAK(); return returnValue; } while (false)

#if OP_DEBUG_MAKER
struct OpDebugMaker {
	OpDebugMaker()
		: line(0) {
	}

	OpDebugMaker(std::string f, int l)
		: file(f)
		, line(l) {
	}

#if OP_DEBUG_DUMP
    bool valid() const { return line > 0; }
	void dumpSet(const char*& );
	std::string debugDump() const;
#endif
	std::string file = "uninitialized";
	int line = -1;
};

#define OP_LINE_FILE_PARAMS(...) , __LINE__, std::string(__FILE__), __VA_ARGS__
#define OP_LINE_FILE_PARGS() , __LINE__, std::string(__FILE__)
#define OP_LINE_FILE_NPARAMS(...) __LINE__, std::string(__FILE__), __VA_ARGS__
#define OP_LINE_FILE_NPARGS() __LINE__, std::string(__FILE__)
#define OP_LINE_FILE_STRUCT(...) , { __FILE__, __LINE__ }, __VA_ARGS__
#define OP_LINE_FILE_CALLER(...) , lineNo, fileName, __VA_ARGS__
#define OP_LINE_FILE_CARGS() , lineNo, fileName
#define OP_LINE_FILE_NP_CALLER(...) lineNo, fileName, __VA_ARGS__
#define OP_LINE_FILE_NP_CARGS() lineNo, fileName
#define OP_LINE_FILE_SCALLER(...) , { fileName, lineNo }, __VA_ARGS__
#define OP_LINE_FILE_DEF(...) , int lineNo, std::string fileName, __VA_ARGS__
#define OP_LINE_FILE_ARGS() , int lineNo, std::string fileName
#define OP_LINE_FILE_NP_DEF(...) int lineNo, std::string fileName, __VA_ARGS__
#define OP_LINE_FILE_NP_ARGS() int lineNo, std::string fileName
#define OP_LINE_FILE_DECLARE(debugMaker) OpDebugMaker debugMaker;
#define OP_LINE_FILE_SET(debugMaker) debugMaker = { fileName, lineNo }
#define OP_LINE_FILE_SET_IMMED(debugMaker) debugMaker = { __FILE__, __LINE__ }

#else
#define OP_LINE_FILE_PARAMS(...) , __VA_ARGS__
#define OP_LINE_FILE_PARGS()
#define OP_LINE_FILE_NPARAMS(...) __VA_ARGS__
#define OP_LINE_FILE_NPARGS()
#define OP_LINE_FILE_STRUCT(...) , __VA_ARGS__
#define OP_LINE_FILE_CALLER(...) , __VA_ARGS__
#define OP_LINE_FILE_CARGS()
#define OP_LINE_FILE_NP_CALLER(...) __VA_ARGS__
#define OP_LINE_FILE_NP_CARGS()
#define OP_LINE_FILE_SCALLER(...) , __VA_ARGS__
#define OP_LINE_FILE_DEF(...) , __VA_ARGS__
#define OP_LINE_FILE_ARGS()
#define OP_LINE_FILE_NP_DEF(...) __VA_ARGS__
#define OP_LINE_FILE_NP_ARGS()
#define OP_LINE_FILE_DECLARE(debugMaker)
#define OP_LINE_FILE_SET(debugMaker)
#define OP_LINE_FILE_SET_IMMED(debugMaker)
#endif

// debug compare, debug dump, and debug image as written only work when testing uses a single thread
#if OP_DEBUGGER || !OP_DEBUG_FAST_TEST
// conditionalize the following to fast test so they don't end up in committed code by accident
#define OpBreak(opObject, ID) \
	do { if ((opObject) && (ID) == (opObject)->id) OP_DEBUG_BREAK(); } while (false)

#define OpBreak2(o1, o2, i1, i2) \
	do { if ((o1) && (o2) && (o1)->id != (o2)->id && ((i1) == (o1)->id || (i2) == (o1)->id) && \
            ((i1) == (o2)->id || (i2) == (o2)->id)) OP_DEBUG_BREAK(); } while (false)

#define OpBreakIf(opObject, ID, doBreak) \
	do { if ((doBreak) && (opObject) && (ID) == (opObject)->id) OP_DEBUG_BREAK(); } while (false)

#define OpAssert(doBreak) \
    do { if (!(doBreak)) OP_DEBUG_BREAK(); } while (false)

#endif

#if OP_DEBUG_DUMP 
#undef OP_DEBUG_DUMP_CODE
#define OP_DEBUG_DUMP_CODE(...) __VA_ARGS__
#undef OP_DEBUG_DUMP_PARAMS
#define OP_DEBUG_DUMP_PARAMS(...) , __VA_ARGS__
#endif

#if OP_DEBUGGER
#undef OP_DEBUGGER_CODE
#define OP_DEBUGGER_CODE(...) __VA_ARGS__
#undef OP_DEBUGGER_PARAMS
#define OP_DEBUGGER_PARAMS(...) , __VA_ARGS__
#endif

#endif

#if OP_DEBUG

#if OP_DEBUG_GLOBALS && OP_DEBUG_DUMP
extern struct OpContext* debugGlobalContext;

struct OpDebugContourIter {
	OpDebugContourIter(bool start);

	bool operator!=(OpDebugContourIter rhs) { 
		return contourIndex != rhs.contourIndex; 
	}

	struct OpContour* operator*();

	void operator++() { 
		++contourIndex;
	}

	size_t contourIndex;
};

struct OpDebugContourIterator {
	OpDebugContourIter begin() { return OpDebugContourIter(true); }
	OpDebugContourIter end() { return OpDebugContourIter(false); }
	bool empty() { return !(begin() != end()); }
};

struct OpDebugSegmentIter {
	OpDebugSegmentIter(bool start);

	bool operator!=(OpDebugSegmentIter rhs) { 
		return segmentIndex != rhs.segmentIndex; 
	}

	struct OpSegment* operator*();

	void operator++() { 
		++segmentIndex;
	}

	size_t segmentIndex;
};

struct OpDebugSegmentIterator {
	OpDebugSegmentIter begin() { return OpDebugSegmentIter(true); }
	OpDebugSegmentIter end() { return OpDebugSegmentIter(false); }
	bool empty() { return !(begin() != end()); }
};

struct OpDebugEdgeIter {
	OpDebugEdgeIter(bool start);

	bool operator!=(OpDebugEdgeIter rhs) { 
		return edgeIndex != rhs.edgeIndex; 
	}

	struct OpEdge* operator*();

	void operator++() { 
		++edgeIndex;
	}

	bool isCurveCurve;
	bool isFiller;
	bool isLine;
	int edgeIndex;
};

struct OpDebugEdgeIterator {
	OpDebugEdgeIter begin() { return OpDebugEdgeIter(true); }
	OpDebugEdgeIter end() { return OpDebugEdgeIter(false); }
	bool empty() { return !(begin() != end()); }
};

struct OpDebugIntersectionIter {
	OpDebugIntersectionIter(bool start);

	bool operator!=(OpDebugIntersectionIter rhs) { 
		return localIntersectionIndex != rhs.localIntersectionIndex; 
	}

	const struct OpIntersection* operator*();

	void operator++() { 
		++localIntersectionIndex;
	}

	size_t localIntersectionIndex;
};

struct OpDebugIntersectionIterator {
	OpDebugIntersectionIter begin() { return OpDebugIntersectionIter(true); }
	OpDebugIntersectionIter end() { return OpDebugIntersectionIter(false); }
	bool empty() { return !(begin() != end()); }
};

// !!! eventually, move iterators into op context
extern OpDebugContourIterator contourIterator;
extern OpDebugSegmentIterator segmentIterator;
extern OpDebugEdgeIterator edgeIterator;
extern OpDebugIntersectionIterator intersectionIterator;

#endif

extern void v0(const char* testname, class SkPath& );  // immediate command to capture failing test

int OpDebugCountDelimiters(const char* str, char delimiter, char openBracket, char closeBracket);
void OpDebugExit(std::string);
void OpDebugExitOnFail(std::string, bool );
void OpDebugByteArray(const char*& str, size_t size, uint8_t* );
std::string OpDebugByteToHex(uint8_t);
std::string OpDebugIntToHex(int32_t);
std::string OpDebugPtrToHex(void*);
std::string OpDebugDumpHex(float);

#endif


#if OP_DEBUG || OP_DEBUG_DUMP || OP_DEBUG_IMAGE

std::string OpDebugDumpByteArray(const uint8_t* bytes, size_t size);
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

#endif

#if OP_DEBUG_SERIALIZE
enum class DebugLevel;
enum class DebugBase;

void OpDebugFormat(std::string );
#else
#define DUMP_DECLARATIONS
#endif

namespace PathOpsV0Lib {
	struct Curve;

	OP_DEBUG_VALIDATE_CODE(void OpDebugValidate(Curve ));

    void debugLineScale(PathOpsV0Lib::Curve curve, double sX, double SY, double dX, double dY);
    void debugQuadScale(PathOpsV0Lib::Curve curve, double sX, double SY, double dX, double dY);
    void debugConicScale(PathOpsV0Lib::Curve curve, double sX, double SY, double dX, double dY);
    void debugCubicScale(PathOpsV0Lib::Curve curve, double sX, double SY, double dX, double dY);

#define DEBUG_SCALE_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(debugLineScale), \
    OP_TAGGED_FUNCTION(debugQuadScale), \
    OP_TAGGED_FUNCTION(debugConicScale), \
    OP_TAGGED_FUNCTION(debugCubicScale), \

}

#if OP_TEST_RASTER
#define OP_DEBUG_RASTER_CODE(...) __VA_ARGS__
#define OP_DEBUG_RASTER_PARAMS(params) , params
#else
#define OP_DEBUG_RASTER_CODE(...)
#define OP_DEBUG_RASTER_PARAMS(params)
#endif

#endif
