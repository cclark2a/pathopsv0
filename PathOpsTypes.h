// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOpsTypes_DEFINED
#define PathOpsTypes_DEFINED

#include "OpMath.h"

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
struct Context {
	int dummy;
};

// caller defined optional data
typedef void* ContextData;

enum class ContextError {
    none,
	finite,
	intersection,
    segmentBounds,
	toVertical,
	tree
};

// convenience for adding caller defined data to context
struct AddContext {
	ContextData data;
	size_t size;
};

// A collection of curves in an operand that share the same fill rules.
struct Contour;

// caller defined operator (e.g., union, intersect, difference, ...)
enum class Operation : int;

// caller defined operand (e.g., left-of-operator, right-of-operator, ...)
enum class Operand : int;

// caller defined curve type (e.g., line, arc, cubic, ...)  Value zero is reserved.
enum class CurveType : int;

// caller defined contour data; curves that share the same winding and fill rules
typedef void* ContourData;

// for transport of contour data to callbacks
struct CallerData {
	ContourData data;
	size_t size;
};

// convenience for adding contours
struct AddContour {
	Context* context;
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
	CurveType type;
};

// convenience for adding curves (e.g., Beziers) to contour
struct AddCurve {
	OpPoint* points;
	size_t size;	// size of points in bytes
	CurveType type;
};

// caller defined data representing how curves in a contour cover area
typedef void* WindingData;

// returns if an edge starts a fill, ends a fill, or does neither and should be discarded
enum class WindKeep {
    Discard,	// must be equal to zero
    End,
    Start,
};

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

// typedef bool (*ControlNearlyEnd)(Curve );

typedef bool (*CurveIsFinite)(Curve );

// returns true if curve is line
typedef bool (*CurveIsLine)(Curve ); 

// returns true if curve degenerates to line
// typedef bool (*CurveIsLinear)(Curve ); 

// returns OpPoint at parameter t, where: t=0 is start, t=1 is end
typedef OpPoint (*PtAtT)(Curve , float t);

// returns high precision OpPoint at parameter t, where: t=0 is start, t=1 is end
typedef OpPoint (*DoublePtAtT)(Curve , float t);

// returns true if additional data is equal (e.g., lines are always equal) 
typedef bool (*CurvesEqual)(Curve , Curve );

// returns point constructs curve's hull; the curve is tightly contained by the hull's polygon
typedef OpPoint (*CurveHull)(Curve, int index);

// returns normal vector at parameter t, where: t=0 is start, t=1 is end
typedef OpVector (*CurveNormal)(Curve, float t);

// adds curve to output
typedef void (*CurveOutput)(Curve, bool firstPt, bool lastPt, PathOutput );

// map the control points (if any) to lie inside the bounds of the curve (e.g., start and end)
typedef void (*CurvePinCtrl)(Curve);

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
// returns string name of curve type
typedef std::string (*DebugDumpCurveName)();

// describes caller data for debugging (does not include points: e.g., a rational Bezier weight)
typedef std::string (*DebugDumpCurveExtra)(Curve , DebugLevel , DebugBase);
#endif

#if OP_DEBUG_IMAGE
// !!! documentation comment missing
typedef void (*DebugAddToPath)(Curve , class SkPath& );
#endif

struct CurveCallBacks {
	AxisRawHit axisRawHitFuncPtr;
	CurveHull curveHullFuncPtr;
	CurveIsFinite curveIsFiniteFuncPtr;
	CurveIsLine curveIsLineFuncPtr;
	SetBounds setBoundsFuncPtr;
	CurveNormal curveNormalFuncPtr;
	CurveOutput curveOutputFuncPtr;
	CurvePinCtrl curvePinCtrlFuncPtr;
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
	DebugDumpCurveName debugDumpCurveNameFuncPtr;
	DebugDumpCurveExtra debugDumpCurveExtraFuncPtr;
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
typedef void (*DebugDumpContourIn)(const char*& str , Winding );
typedef std::string (*DebugDumpContourOut)(Winding );
typedef std::string (*DebugDumpContourExtra)(CallerData , DebugLevel , DebugBase );
#endif
#if OP_DEBUG_IMAGE
typedef std::string (*DebugImageOut)(Winding , int index);
typedef uint32_t (*DebugCCOverlapsColor)(CallerData);
typedef uint32_t (*DebugCurveCurveColor)(CallerData);
typedef uint32_t (*DebugNativeFillColor)(CallerData);
typedef uint32_t (*DebugNativeInColor)(CallerData);
typedef void* (*DebugNativePath)(CallerData);
typedef bool (*DebugGetDraw)(CallerData);
typedef void (*DebugSetDraw)(CallerData, bool);
typedef bool (*DebugIsOpp)(CallerData);
#endif

struct ContourCallBacks {
	WindingAdd windingAddFuncPtr;
	WindingKeep windingKeepFuncPtr;
	WindingSubtract windingSubtractFuncPtr;
	WindingVisible windingVisibleFuncPtr;
	WindingZero windingZeroFuncPtr;
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

typedef void (*EmptyNativePath)(PathOutput );
typedef CurveType (*SetLineType)(Curve );

// returns data size and type as appropriate for line connecting input curve points
typedef Curve (*MakeLine)(Curve );


struct ContextCallBacks {
	EmptyNativePath emptyNativePathFuncPtr;
	MakeLine makeLineFuncPtr;
	SetLineType setLineTypeFuncPtr;
};

#if OP_DEBUG_DUMP
inline std::string noDumpFunc(CallerData caller, DebugLevel , DebugBase ) {
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

inline void* noNativePathFunc(CallerData ) {
	return nullptr;
}

inline bool noDebugGetDrawFunc(CallerData ) {
	return false;
}

inline void noDebugSetDrawFunc(CallerData , bool ) {
}

inline bool noIsOppFunc(CallerData ) {
	return false;
}
#endif

} // namespace PathOpsV0Lib

#endif