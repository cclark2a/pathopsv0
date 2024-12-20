// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOpsTypes_DEFINED
#define DebugOpsTypes_DEFINED

#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

#if OP_DEBUG
typedef void (*DebugScale)(Curve , double scale, double offsetX, double offsetY);

#if OP_DEBUG_DUMP
// returns string name of curve type
typedef std::string (*DebugDumpCurveName)();

// describes caller data for debugging (does not include points: e.g., a rational Bezier weight)
typedef std::string (*DebugDumpCurveExtra)(Curve , DebugLevel , DebugBase);
#endif

#if OP_DEBUG_IMAGE
// !!! documentation comment missing
typedef void (*DebugAddToPath)(Curve , class SkPath& );
#endif

struct DebugCurveCallBacks {
	DebugScale scaleFuncPtr;
	OP_DEBUG_DUMP_CODE(DebugDumpCurveName curveNameFuncPtr;)
	OP_DEBUG_DUMP_CODE(DebugDumpCurveExtra curveExtraFuncPtr;)
	OP_DEBUG_IMAGE_CODE(DebugAddToPath addToPathFuncPtr;)
};

// caller defined contour data; curves that share the same winding and fill rules
typedef void* DebugContourData;

// for transport of contour data to callbacks
struct DebugCallerData {
	DebugContourData data;
	size_t size;
};

typedef uint8_t (*DebugBitOper)(DebugCallerData , uint8_t , uint8_t);

#if OP_DEBUG_DUMP
typedef void (*DebugDumpContourIn)(const char*& str , Winding );
typedef std::string (*DebugDumpContourOut)(Winding );
typedef std::string (*DebugDumpContourExtra)(DebugCallerData , DebugLevel , DebugBase );
#endif
#if OP_DEBUG_IMAGE
typedef std::string (*DebugImageOut)(Winding , int index);
typedef uint32_t (*DebugCCOverlapsColor)(DebugCallerData);
typedef uint32_t (*DebugCurveCurveColor)(DebugCallerData);
typedef uint32_t (*DebugNativeFillColor)(DebugCallerData);
typedef uint32_t (*DebugNativeInColor)(DebugCallerData);
typedef void* (*DebugNativePath)(DebugCallerData);
typedef bool (*DebugGetDraw)(DebugCallerData);
typedef void (*DebugSetDraw)(DebugCallerData, bool);
typedef bool (*DebugIsOpp)(DebugCallerData);
#endif

struct DebugContourCallBacks {
	DebugBitOper debugBitOperFuncPtr;

#if OP_DEBUG_DUMP
	DebugDumpContourIn debugDumpContourInFuncPtr;
	DebugDumpContourOut debugDumpContourOutFuncPtr;
	DebugDumpContourExtra debugDumpContourExtraFuncPtr;
#endif
#if OP_DEBUG_IMAGE
	DebugImageOut debugImageOutFuncPtr;
	DebugNativePath debugNativePathFuncPtr;
	DebugGetDraw debugGetDrawFuncPtr;
	DebugSetDraw debugSetDrawFuncPtr;
	DebugIsOpp debugIsOppFuncPtr;
#endif
};

#if OP_DEBUG_DUMP
inline std::string noDumpFunc(DebugCallerData caller, DebugLevel , DebugBase ) {
    OP_ASSERT(!caller.size);
    return "";
}
#endif

#if OP_DEBUG_IMAGE
inline void noAddToSkPathFunc(Curve , SkPath& ) {
}

inline std::string noWindingImageOutFunc(Winding , int index) {
	return "";
}

inline void* noNativePathFunc(DebugCallerData ) {
	return nullptr;
}

inline bool noDebugGetDrawFunc(DebugCallerData ) {
	return false;
}

inline void noDebugSetDrawFunc(DebugCallerData , bool ) {
}

inline bool noIsOppFunc(DebugCallerData ) {
	return false;
}
#endif

#endif

}

#endif

