// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOpsTypes_DEFINED
#define DebugOpsTypes_DEFINED

#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

// caller defined contour data (e.g., a pointer to the native path)
typedef void* DebugContour;

enum class DebugContourType {
    windingUserData,
    Count
};

// for transport of contour data to callbacks
struct DebugContourData {
	DebugContour data;
	size_t size;
};

// caller defined context data (e.g., the path operation)
typedef void* DebugContext;

enum class DebugContextType {
    windingUserData,
    addRaster,
    Count
};

// for transport of context data to callbacks
struct DebugContextData {
	DebugContext data;
	size_t size;
};

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

#if TEST_RASTER
typedef void (*DebugAddRaster)(DebugContextData , Curve , int parentID);
#endif

struct DebugCurveCallbacks {
	DebugScale scaleFuncPtr;
	OP_DEBUG_DUMP_CODE(DebugDumpCurveName curveNameFuncPtr;)
	OP_DEBUG_DUMP_CODE(DebugDumpCurveExtra curveExtraFuncPtr;)
	OP_DEBUG_IMAGE_CODE(DebugAddToPath addToPathFuncPtr;)
#if TEST_RASTER
    DebugAddRaster addRasterFuncPtr;
#endif
};

typedef uint8_t (*DebugBitOper)(DebugContourData , uint8_t , uint8_t);
#if OP_DEBUG_DUMP
typedef std::string (*DebugDumpWindingOut)(Winding );
typedef void (*DebugDumpWindingSet)(const char*& , Winding& );
typedef std::string (*DebugDumpContextExtra)(DebugContextData , DebugLevel , DebugBase );
#endif
#if OP_DEBUG_IMAGE
typedef std::string (*DebugImageWindingOut)(Winding , int index);
#endif

struct DebugContextCallbacks {
#if OP_DEBUG_DUMP
    DebugDumpContextExtra debugDumpContextExtraFuncPtr = nullptr;
	DebugDumpWindingOut debugDumpWindingOutFuncPtr = nullptr;
	DebugDumpWindingSet debugDumpWindingSetFuncPtr = nullptr;
#endif
#if OP_DEBUG_IMAGE
	DebugImageWindingOut debugImageWindingOutFuncPtr = nullptr;
#endif
};

#if OP_DEBUG_DUMP
typedef std::string (*DebugDumpContourExtra)(DebugContourData , DebugLevel , DebugBase );
#endif
#if OP_DEBUG_IMAGE
typedef void* (*DebugNativePath)(DebugContourData );
typedef bool (*DebugGetDraw)(DebugContourData );
typedef void (*DebugSetDraw)(DebugContourData , bool);
typedef bool (*DebugOperand)(DebugContourData , int );
#endif

struct DebugContourCallbacks {
#if OP_DEBUG_DUMP
    DebugDumpContourExtra debugDumpContourExtraFuncPtr = nullptr;
#endif
#if OP_DEBUG_IMAGE
	DebugNativePath debugNativePathFuncPtr = nullptr;
	DebugGetDraw debugGetDrawFuncPtr = nullptr;
	DebugSetDraw debugSetDrawFuncPtr = nullptr;
	DebugOperand debugOperandFuncPtr = nullptr;
#endif
};


#if OP_DEBUG_IMAGE
struct ColorCurve {
    Curve curve;
    uint32_t color;
};
#endif

#if 0  // not (yet) implemented
typedef uint32_t (*DebugCCOverlapsColor)(DebugContextData );  
typedef uint32_t (*DebugCurveCurveColor)(DebugContextData );
typedef uint32_t (*DebugNativeFillColor)(DebugContextData );
typedef uint32_t (*DebugNativeInColor)(DebugContextData );
#endif

#endif

}

#endif
