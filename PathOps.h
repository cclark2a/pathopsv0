// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOps_DEFINED
#define PathOps_DEFINED

#include "OpDebug.h"
#include "OpDebugDump.h"
#include "OpOperators.h"

#if OP_DEBUG
#include <vector>
#endif

struct OpInPath {
	OpInPath(const void* ext) 
		: externalReference(ext)  {
	}
	bool isInverted() const;
#if OP_DEBUG_DUMP
	~OpInPath();
	static const void* MakeExternalReference();
	static void ReleaseExternalReference(const void* );
	DUMP_DECLARATIONS
#endif

	const void* externalReference;
};

struct OpOutPath {
	OpOutPath(void* ext) 
		: externalReference(ext) { 
		OP_DEBUG_CODE(debugID = 0);
	}
	void setEmpty();
	void setInverted(bool wasInverted);
#if OP_DEBUG
	bool debugIsEmpty() const;
	void debugNextID(struct OpEdge* );
#endif
#if OP_DEBUG_DUMP
	~OpOutPath();
	static void* MakeExternalReference();
	static void ReleaseExternalReference(void* );
	DUMP_DECLARATIONS
#endif

	void* externalReference;
#if OP_DEBUG
	int debugID;
#endif
};

bool PathOps(OpInPath& left, OpInPath& right, OpOperator , OpOutPath& result);
bool PathSimplify(OpInPath& path, OpOutPath& result);

#if OP_DEBUG
// entry point if operation success is already known
bool DebugPathOps(OpInPath& left, OpInPath& right, OpOperator , OpOutPath& result,
        OpDebugExpect expected, std::string testname, std::vector<OpDebugWarning>& );
bool DebugPathSimplify(OpInPath& path, OpOutPath& result, 
		OpDebugExpect expected, std::string testname, std::vector<OpDebugWarning>& );
#endif

// the interfaces above will be replaced with the context model below (maybe?)

extern "C" {

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

/* usage:


*/

struct Context;

struct Point {
	float x;
	float y;
 };

struct Vector {
	float dx;
	float dy;
 };

struct PtT {
	Point pt;
	float t;
};

struct PairXY {
	float smaller;
	float larger;
};

constexpr int MAX_ROOTS = 5;

struct Roots {
	float roots[MAX_ROOTS];
	size_t count;
};

struct RootPtTs {
	Roots roots;
	PtT ptT[MAX_ROOTS];
};

enum class Axis {
	horizontal,
	vertical
};

enum class XyChoice {
	x,
	y
};

/* Curve describes a set of continuous points from start to end.
   Curve's points must be contained by rectangle bounded by start and end.
   Curve's points must montonically vary from start to end in x and y.
 */
struct CurveData {
	Point start;
	Point end;
	char optionalAdditionalData[];
};

/*  The type of curve, contour, or operation. The types available are defined by the caller,
    and interpreted through installed function callbacks.
 */
typedef int TypeID;

/* SetCurveLength() (see below) describes the length of each curve.
 */
struct Curve {
	CurveData* data;
	TypeID type;
};

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

// functions
/* Adds one curve. Uses SetCurveLength() to retrieve the length to copy.
 */
void Add(Context* , Curve );

/* Makes a PathOps context. 
 */
Context* Create();

/* Deletes a PathOps context, and frees any memory associated with that context.
 */
void Delete(Context* );

/* returns error code of previous call
 */
int Error(Context* );

/* subsequent calls to Add() associates data and type arrays with this contour ID
 */
void Contour(Context* , TypeID contour);

/* Operate on curves provided by Add() with user defined operation.
   Calls user supplied functions to return computed path one curve at a time.
 */
void Resolve(Context* , TypeID operation);


// callbacks

/* provided function intersects the curve and axis at the axis intercept
 */
typedef Roots (*AxisRay)(Curve , Axis , float axisIntercept, float start, float end);
void SetAxisRay(Context* , AxisRay);

/* provided function returns length of data used by this type of curve
   return zero if length could not be determined; this will cause Resolve() to fail
 */
typedef size_t (*CurveLength)(Curve );
void SetCurveLength(Context* , CurveLength);

/* provided function returns true if curve can be represented by two points
 */
typedef bool (*CurveIsLinear)(Curve ); 
void SetCurveIsLinear(Context* , CurveIsLinear);

/* provided function returns OpPoint at parameter t, where: t=0 is start, t=1 is end
 */
typedef OpPoint (*CurvePtAtT)(Curve , float t);
void SetCurvePtAtT(Context* , CurvePtAtT);

/* provided function returns array of parameter t that make curve monotonic
 */
typedef Roots (*CurveExtrema)(Curve , XyChoice );
void SetExtrema(Context* , CurveExtrema);

/* provided function returns array of parameter t values and points where line intersects curve
 */
typedef RootPtTs (*LineIntersect)(Curve curve, Curve line); 
void SetLineIntersect(Context* , LineIntersect);

/* provided function receives overall stats for next contour generated by Resolve()
 */
typedef ContourData (*NextContour)();
void SetNextContour(Context* , NextContour);

/* provided function points to storage for next curve generated by Resolve().
 */
typedef TypeID (*NextCurve)(CurveData* );
void SetNextCurve(Context* , NextCurve);

/* provided function receives overall stats for next path generated by Resolve()
 */
typedef PathData (*NextPath)();
void SetNextPath(Context* , NextPath);

/* provided function returns normal vector at parameter t, where: t=0 is start, t=1 is end
 */
typedef OpVector (*CurveNormal)(Curve, float t);
void SetNormal(Context* , CurveNormal);

/* provided function computes part of Curve from parameter t1 to t2, both from zero to one. 
   Computed curve is stored in partial using length. Returns storage required. If returned size 
   differs from length, function is called again. Returning zero causes Resolve() to fail.
 */
typedef size_t (*SubDivide)(Curve , float t1, float t2, CurveData* partial, size_t length);
void SetSubDivide(Context* , SubDivide);

/* provided function returns tangent vector at parameter t, where: t=0 is start, t=1 is end
 */
typedef Vector (*CurveTangent)(Curve, float t);
void SetTangent(Context* , CurveTangent);

/* Provided function rotates Curve using the matrix that maps (align.start, align.end) to
   ((0, 0), (0, align.end.y - align.start.y)). The original Curve scale is not preserved.
   Computed curve is stored in vertical using length. Returns storage required. If returned size 
   differs from length, function is called again. Returning zero causes Resolve() to fail.
 */
typedef size_t (*Vertical)(Curve toRotate, Curve align, CurveData* vertical, size_t length);
void SetVertical(Context* , Vertical);

/* provided function returns either x or y pair at parameter t, where: t=0 is start, t=1 is end
 */
typedef PairXY (*CurveXYAtT)(Curve , float t, XyChoice );
void SetCurveXYAtT(Context* , CurveXYAtT);


// utilities
size_t AddQuads(Context* context, Curve curve);
Point QuadControlPt(Point start, Point control, Point end, PtT ptT1, PtT ptT2);
Point QuadPointAtT(Point start, Point control, Point end, float t);

} // extern "C"

} // namespace PathOpsV0Lib

#endif
