// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOpsTypes_DEFINED
#define PathOpsTypes_DEFINED

#include "OpMath.h"

enum class OpType;
struct OpPair;
enum class WindKeep;
struct OpWinding;

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

struct Context;

/* A collection of curves in an operand that share the same fill rules.
 */
struct Contour;

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

/* convenience for adding curve to contour
 */
struct AddCurve {
	Context* context;
	OpPoint* points;
	size_t size;	// size of points in bytes
	OpType type;
};

/* caller defined data representing how curves in a contour cover area
 */
struct WindingData;

struct Winding {
	WindingData* data;
	size_t size;
};

/* convenience for adding winding data to contour
 */
struct AddWinding {
	Contour* contour;
	int* windings;
	size_t size;
};

typedef int ContourID;

/* the contour ID and number of curves in the contour
 */
struct ContourData {
	int contourID;
	int curves;
};

/* the number of contours and overall number of curves 
 */
struct PathData {
	int contours;
	int curves;
};

// path provided by caller
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

typedef void (*SetBounds)(Curve , OpPointBounds& );

// returns either x or y pair at parameter t, where: t=0 is start, t=1 is end
typedef OpPair (*XYAtT)(Curve , OpPair t, XyChoice );

#if OP_DEBUG_DUMP
typedef void (*DumpSet)(Curve , const char*& str);
#endif

#if OP_DEBUG_IMAGE
// !!! documentation comment missing
typedef void (*DebugAddToPath)(Curve , class SkPath& );
#endif

struct CallBacks {
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
	DumpSet dumpSetFuncPtr;
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
#endif
#if OP_DEBUG_IMAGE
typedef std::string (*WindingImageOut)(Winding , int index);
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
#endif
#if OP_DEBUG_IMAGE
	WindingImageOut windingImageOutFuncPtr;
#endif
};


} // namespace PathOpsV0Lib

#endif