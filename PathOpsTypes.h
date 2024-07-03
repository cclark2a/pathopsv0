// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOpsTypes_DEFINED
#define PathOpsTypes_DEFINED

#include "OpMath.h"

enum class OpType;
enum class WindKeep;

namespace PathOpsV0Lib {

/* think about:
data is clumped into points, which may have 2, 3, or more components
pathops only uses first two: x, y
caller may carry other data, such as conic weight or z

the data could be int or double (or long double etc.) instead of float
allow this?

the type describes what the data is, and how many floats it uses
the intent is to do this through a callback

*/

// An instance of the pathops engine.
struct Context;

// caller defined optional data
typedef void* ContextData;

// convenience for adding caller defined data to context
struct AddContext {
	ContextData data;
	size_t size;
};

// A collection of curves in an operand that share the same fill rules.
struct Contour;

// caller defined optional data
typedef void* ContourData;

// convenience for adding caller defined data to contour
struct AddContour {
	ContourData data;
	size_t size;
};

/* Curve describes a set of continuous points from start to end.
   Curve's points must be contained by a rectangle bounded by start and end.
   Curve's points must monotonically vary from start to end in x and y.
 */
struct CurveData {
	OpPoint start;
	OpPoint end;
	char optionalAdditionalData[];
};

struct Curve {
	CurveData* data;
	size_t size;  // total size in bytes, including additional data, if any
	OpType type;
};

// convenience for adding curves (e.g., Beziers) to contour
struct AddCurve {
	OpPoint* points;
	size_t size;	// size of points in bytes
	OpType type;
};

// caller defined data representing how curves in a contour cover area
typedef void* WindingData;

struct Winding {
	WindingData data;
	size_t size;
};

// convenience for adding winding data to contour
struct AddWinding {
	Contour* contour;
	int* windings;
	size_t size;
};

// output path provided by caller
typedef void* PathOutput;

// callbacks

// intersects the curve and axis at the axis intercept
typedef OpRoots (*AxisRawHit)(Curve , Axis , float axisIntercept, MatchEnds);

// returns number of points in curve, including start and end (minimum 2)
typedef size_t (*PtCount)();

typedef bool (*ControlNearlyEnd)(Curve );

typedef bool (*CurveIsFinite)(Curve );

// returns true if curve is line
typedef bool (*CurveIsLine)(Curve ); 

// returns true if curve degenerates to line
typedef bool (*CurveIsLinear)(Curve ); 

// returns OpPoint at parameter t, where: t=0 is start, t=1 is end
typedef OpPoint (*PtAtT)(Curve , float t);

// returns high precision OpPoint at parameter t, where: t=0 is start, t=1 is end
typedef OpPoint (*DoublePtAtT)(Curve , float t);

// returns true if additional data is equal (e.g., lines are always equal) 
typedef bool (*CurvesEqual)(Curve , Curve );

// returns point constructs curve's hull; the curve is tightly contained by the hull's polygon
typedef OpPoint (*CurveHull)(Curve, int index);

typedef void (*CurveOutput)(Curve, bool firstPt, bool lastPt, PathOutput );

// returns normal vector at parameter t, where: t=0 is start, t=1 is end
typedef OpVector (*CurveNormal)(Curve, float t);

// reverses order of control points, if there is more than one
typedef void (*CurveReverse)(Curve);

typedef void (*Rotate)(Curve , const LinePts& , float adj, float opp, Curve result);

// computes part of Curve from parameter t1 to t2, both from zero to one
typedef void (*SubDivide)(Curve , OpPtT t1, OpPtT t2, Curve result);

// returns tangent vector at parameter t, where: t=0 is start, t=1 is end
typedef OpVector (*CurveTangent)(Curve, float t);

typedef void (*SetBounds)(Curve , OpRect& );

// returns either x or y pair at parameter t, where: t=0 is start, t=1 is end
typedef OpPair (*XYAtT)(Curve , OpPair t, XyChoice );

#if OP_DEBUG_DUMP
typedef std::string (*DebugDumpExtra)(Curve , DebugLevel , DebugBase);
typedef void (*DumpSet)(Curve , const char*& str);
typedef void (*DumpSetExtra)(Curve , const char*& str);
#endif

#if OP_DEBUG_IMAGE
// !!! documentation comment missing
typedef void (*DebugAddToPath)(Curve , class SkPath& );
#endif

struct CurveCallBacks {
	AxisRawHit axisRawHitFuncPtr;
	ControlNearlyEnd controlNearlyEndFuncPtr;
	CurveHull curveHullFuncPtr;
	CurveIsFinite curveIsFiniteFuncPtr;
	CurveIsLine curveIsLineFuncPtr;
	CurveIsLinear curveIsLinearFuncPtr;
	SetBounds setBoundsFuncPtr;
	CurveNormal curveNormalFuncPtr;
	CurveOutput curveOutputFuncPtr;
	CurveReverse curveReverseFuncPtr;
	CurveTangent curveTangentFuncPtr;
	CurvesEqual curvesEqualFuncPtr;
	PtAtT  ptAtTFuncPtr;
	DoublePtAtT  doublePtAtTFuncPtr;
	PtCount ptCountFuncPtr;
	Rotate rotateFuncPtr;
	SubDivide subDivideFuncPtr;
	XYAtT xyAtTFuncPtr;
#if OP_DEBUG_DUMP
	DebugDumpExtra debugDumpExtraFuncPtr;
	DumpSet dumpSetFuncPtr;
	DumpSetExtra dumpSetExtraFuncPtr;
#endif
#if OP_DEBUG_IMAGE
	DebugAddToPath debugAddToPathFuncPtr;
#endif
};

typedef Winding (*WindingAdd)(Winding winding, Winding toAdd);
typedef Winding (*WindingSubtract)(Winding winding, Winding toSubtract);
typedef bool (*WindingVisible)(Winding winding);
typedef void (*WindingZero)(Winding toZero);

// returns if curve transitions to a filled area and is kept; or if curve is discarded
typedef WindKeep (*WindingKeep)(Winding winding, Winding sum);

#if OP_DEBUG_DUMP
typedef void (*WindingDumpIn)(const char*& str , Winding );
typedef std::string (*WindingDumpOut)(Winding );
typedef std::string (*ContourDumpExtra)(AddContour , DebugLevel , DebugBase );
#endif
#if OP_DEBUG_IMAGE
typedef std::string (*WindingImageOut)(Winding , int index);
typedef uint32_t (*WindingDebugColor)(AddContour , DebugImage );
typedef void* (*ContourNativePath)(AddContour);
typedef bool (*ContourDebugDraw)(AddContour);
#endif

struct ContourCallBacks {
	WindingAdd windingAddFuncPtr;
	WindingKeep windingKeepFuncPtr;
	WindingSubtract windingSubtractFuncPtr;
	WindingVisible windingVisibleFuncPtr;
	WindingZero windingZeroFuncPtr;
#if OP_DEBUG_DUMP
	WindingDumpIn windingDumpInFuncPtr;
	WindingDumpOut windingDumpOutFuncPtr;
	ContourDumpExtra debugDumpFuncPtr;
#endif
#if OP_DEBUG_IMAGE
	WindingImageOut windingImageOutFuncPtr;
	WindingDebugColor debugColorFuncPtr;
	ContourNativePath debugNativePath;
	ContourDebugDraw debugDrawFuncPtr;
#endif
};

#if 0
#if OP_DEBUG_IMAGE
typedef void* (*OutputNativePath)(AddContext);
#endif

struct ContextCallBacks {
#if OP_DEBUG_IMAGE
	OutputNativePath debugOutputPath;
#endif
};
#endif

#if OP_DEBUG_DUMP
inline std::string noDumpFunc(AddContour caller, DebugLevel , DebugBase ) {
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

inline uint32_t noDebugColorFunc(AddContour , DebugImage ) {
	return 0;
}

inline void* noNativePathFunc(AddContour) {
	return nullptr;
}

inline bool noDebugDrawFunc(AddContour) {
	return false;
}
#endif

} // namespace PathOpsV0Lib

#endif