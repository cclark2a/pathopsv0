// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOpsTypes_DEFINED
#define DebugOpsTypes_DEFINED

#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

// caller defined curve data (e.g., a pointer to the native path)
struct DebugCurve {
    CurveType curveType;
    size_t curveSize;  // size of curve data and optional curve data
    CurveData curveData;
    // ... optional curve data 
    // ... extrema t values
};

// for transport of curve data to callbacks
struct DebugCurveData {
	DebugCurve* data = nullptr;
	size_t size = 0;  // size of debug curve including extrema values
};

// caller defined context data (e.g., the path operation)
typedef void* DebugContext;

// for transport of context data to callbacks
struct DebugContextData {
	DebugContext data;
	size_t size;
};

#if OP_DEBUG
typedef void (*DebugScale)(Curve , double scaleX, double scaleY, double offsetX, double offsetY);

#if OP_DEBUG_SERIALIZE
// returns string name of curve type
typedef std::string (*DebugDumpCurveName)();

// describes caller data for debugging (does not include points: e.g., a rational Bezier weight)
typedef std::string (*DebugDumpCurveExtra)(Curve , DebugLevel , DebugBase);

typedef void (*DebugSubDivide)(Curve , float t1, float t2, Curve* result);
#endif

#if 0
typedef void (*DebugAddToPath)(Curve , class SkPath& );
#endif

#if 0 && OP_TEST_RASTER
typedef void (*DebugAddRaster)(DebugContextData , Curve , int parentID);
#endif

struct DebugCurveCallbacks {
	DebugScale scaleFuncPtr;
#if OP_DEBUG_SERIALIZE
	DebugDumpCurveName curveNameFuncPtr;
	DebugDumpCurveExtra curveExtraFuncPtr = nullptr;
    DebugSubDivide debugSubDivideFuncPtr = nullptr;
#endif
//    OP_DEBUG_RASTER_CODE(DebugAddRaster addRasterFuncPtr = nullptr;)
};

#if OP_DEBUG_SERIALIZE
struct DebugEdgeType {
    bool disabled;
    bool inOutput;
    bool unsortable;
    bool curveCurve;
    bool ccOverlaps;
};
#endif

// typedef uint8_t (*DebugBitOper)(DebugContourData , uint8_t , uint8_t);
typedef bool (*DebugIsFill)(Winding );
#if OP_DEBUG_SERIALIZE
typedef std::string (*DebugDumpWindingOut)(Winding );
typedef void (*DebugDumpWindingSet)(const char*& , Winding& );
typedef std::string (*DebugDumpOut)(Context* );
typedef std::string (*DebugImageWindingOut)(Winding );
typedef std::vector<std::string> (*DebugImageWindingNames)();
typedef uint32_t (*DebugEdgeColor)(Winding , DebugEdgeType );
#endif

struct DebugContextCallbacks {
    DebugIsFill debugIsFillFuncPtr = nullptr;
#if OP_DEBUG_SERIALIZE
	DebugDumpWindingOut debugDumpWindingOutFuncPtr = nullptr;
	DebugDumpWindingSet debugDumpWindingSetFuncPtr = nullptr;
    DebugDumpOut debugDumpOutFuncPtr = nullptr;
	DebugImageWindingOut debugImageWindingOutFuncPtr = nullptr;
    DebugImageWindingNames debugImageWindingNamesFuncPtr = nullptr;
    DebugEdgeColor debugEdgeColorFuncPtr = nullptr;
    WindingKeep debugWindingVisibleFuncPtr = nullptr;  // if winding effects fill for one contour
#endif
};

#if 0 && OP_DEBUG_SERIALIZE  // disable until we need it
typedef std::string (*DebugDumpContourExtra)(DebugContourData , DebugLevel , DebugBase );
typedef void* (*DebugNativePath)(DebugContourData );
typedef bool (*DebugGetDraw)(DebugContourData );
typedef void (*DebugSetDraw)(DebugContourData , bool);
typedef bool (*DebugOperand)(DebugContourData , int );  // deprecated
#endif

struct DebugContourCallbacks {
#if 0 && OP_DEBUG_SERIALIZE
    DebugDumpContourExtra debugDumpContourExtraFuncPtr = nullptr;
	DebugNativePath debugNativePathFuncPtr = nullptr;
	DebugGetDraw debugGetDrawFuncPtr = nullptr;
	DebugSetDraw debugSetDrawFuncPtr = nullptr;
	DebugOperand debugOperandFuncPtr = nullptr;
    DebugEdgeColor debugEdgeColorFuncPtr = nullptr;
#endif
};

#endif

}

#endif
