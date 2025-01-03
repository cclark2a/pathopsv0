// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOpsTypes_DEFINED
#define PathOpsTypes_DEFINED

#include "OpMath.h"

namespace PathOpsV0Lib {

/* think about:
the data could be int or double (or long double etc.) instead of float
allow this?
*/

// an instance of the pathops engine
struct Context;

enum class ContextError {
    none,  // no error found
	end,  // segment end does not connect to another segment
	finite,  // curve points are not finite
	intersection, // curve intersection error (should not occur)
	missing,  // results do not form closed loops
	toVertical, // rotating / skewing curve (to intersect) exceeds floating point range
	tree	// decision tree to join found edges is too complex
};

// collection of curves that share the same fill rules
struct Contour;

// caller defined curve type (e.g., line, arc, cubic, ...); value zero is reserved
enum class CurveType : int;

// caller-defined optional data for each curve
struct CurveUserData;

/* Curve describes a set of continuous points from start to end.
   Curve's points must be contained by a rectangle bounded by start and end.
   Curve's points must monotonically vary from start to end in x and y.
 */
struct CurveData {
	OpPoint start;
	OpPoint end;
//	CurveUserData optional;	// user data follows endpoints
};

// offset of user data (if any); immediately follows required curve points
inline size_t CurveUserDataOffset() {
	return sizeof(CurveData);
}

// location of user data
inline void* CurveUserData(CurveData* curve) {
	return (void*) ((char*) curve + CurveUserDataOffset());
}

struct Curve {
	CurveData* data;
	size_t size;  // total size pointed to by data (in bytes), including additional data, if any
	CurveType type;
};

// convenience for adding curves (e.g., Beziers) to contour
struct AddCurve {
	OpPoint* points;
	size_t size;	// size of data pointed to by points (in bytes)
	CurveType type;
};

// caller defined data representing how curves in a contour cover area
typedef void* WindingData;

struct Winding {
	WindingData data;
	size_t size;
};

// returns if an edge starts a fill, ends a fill, or does neither and should be discarded
enum class WindKeep {
    Discard,	// must be equal to zero
    End,		// edge ends a filled area
    Start,		// edge begins a filled area
};

// output path provided by caller
typedef void* PathOutput;

// curve callbacks

// intersects the curve and axis at the axis intercept
typedef OpRoots (*AxisT)(Curve , Axis , float axisIntercept, MatchEnds);

// returns number of points in curve user data that contribute to hull
typedef int (*HullPtCount)();

// typedef bool (*ControlNearlyEnd)(Curve );

typedef bool (*CurveIsFinite)(Curve );

// returns true if curve is line
typedef bool (*CurveIsLine)(Curve ); 

// returns true if curve degenerates to line
// typedef bool (*CurveIsLinear)(Curve ); 

// returns OpPoint at parameter t, where: t=0 is start, t=1 is end
typedef OpPoint (*PtAtT)(Curve , float t);

// returns true if additional data is equal (e.g., lines are always equal) 
typedef bool (*CurvesEqual)(Curve , Curve );

// returns point constructs curve's hull; the curve is tightly contained by the hull's polygon
typedef OpPoint (*CurveHull)(Curve, int index);

// adds curve to output
typedef void (*CurveOutput)(Curve, bool firstPt, bool lastPt, PathOutput );

// map the control points (if any) to lie inside the bounds of the curve (e.g., start and end)
typedef void (*CurvePinCtrl)(Curve);

// reverses order of control points, if there is more than one
typedef void (*CurveReverse)(Curve);

// rotates curve user data about origin (not normalized)
typedef void (*Rotate)(Curve , OpPoint origin, OpVector scale, Curve result);

// computes part of Curve from parameter t1 to t2, both from zero to one
typedef void (*SubDivide)(Curve , float t1, float t2, Curve result);

// returns tangent vector at parameter t, where: t=0 is start, t=1 is end
typedef OpVector (*CurveTangent)(Curve, float t);

// adds curve user data to bounds
typedef void (*SetBounds)(Curve , OpRect& );

// returns either x or y pair at parameter t, where: t=0 is start, t=1 is end
typedef OpPair (*XYAtT)(Curve , OpPair t, XyChoice );

// overrides engine maximum for a curve type
typedef float (*CurveConst)(Curve );

struct CurveCallBacks {
	CurveOutput curveOutputFuncPtr;
	AxisT axisTFuncPtr = nullptr;
	CurveHull curveHullFuncPtr = nullptr;
	CurveIsFinite curveIsFiniteFuncPtr = nullptr;
	CurveIsLine curveIsLineFuncPtr = nullptr;
	SetBounds setBoundsFuncPtr = nullptr;
	CurvePinCtrl curvePinCtrlFuncPtr = nullptr;
	CurveTangent curveTangentFuncPtr = nullptr;
	CurvesEqual curvesEqualFuncPtr = nullptr;
	PtAtT  ptAtTFuncPtr = nullptr;
	HullPtCount ptCountFuncPtr = nullptr;
	Rotate rotateFuncPtr = nullptr;
	SubDivide subDivideFuncPtr = nullptr;
	XYAtT xyAtTFuncPtr = nullptr;
	CurveReverse curveReverseFuncPtr = nullptr;
	CurveConst cutFuncPtr = nullptr;
	CurveConst normalLimitFuncPtr = nullptr;
	CurveConst interceptFuncPtr = nullptr;
};

// contour callbacks

// adds winding
typedef void (*WindingAdd)(Winding winding, Winding toAdd);

// subtracts winding
typedef void (*WindingSubtract)(Winding winding, Winding toSubtract);

// returns true if winding affects operation
typedef bool (*WindingVisible)(Winding winding);

// marks winding as having no effect
typedef void (*WindingZero)(Winding toZero);

// returns if curve transitions to a filled area and is kept; or if curve is discarded
typedef WindKeep (*WindingKeep)(Winding winding, Winding sum);

struct WindingCallBacks {
	WindingAdd windingAddFuncPtr;
	WindingKeep windingKeepFuncPtr;
	WindingVisible windingVisibleFuncPtr;
	WindingZero windingZeroFuncPtr;
	WindingSubtract windingSubtractFuncPtr = nullptr;
};

// context callbacks

// initializes caller's path as empty
typedef void (*EmptyCallerPath)(PathOutput );

// 
typedef CurveType (*SetLineType)(Curve );

typedef float (*MaxSignSwap)(Curve , Curve );
typedef int (*MaxCurveCurve)(Curve , Curve );
typedef int (*MaxLimbs)(Context* );

struct ContextCallBacks {
	SetLineType setLineTypeFuncPtr;
	EmptyCallerPath emptyCallerPathFuncPtr = nullptr;
	MaxSignSwap maxSignSwapFuncPtr = nullptr;
	MaxCurveCurve maxDepthFuncPtr = nullptr;
	MaxCurveCurve maxSplitsFuncPtr = nullptr;
	MaxLimbs maxLimbsFuncPtr = nullptr;
};

// return true if resolve should be aborted
typedef bool (*ErrorDispatch)(ContextError , Context* , Curve* );

struct ErrorHandler {
	ErrorDispatch errorDispatchFuncPtr;
};

} // namespace PathOpsV0Lib

#endif
