// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOps_DEFINED
#define PathOps_DEFINED

#include "OpDebug.h"
#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

/* usage:


*/

// functions
/* Adds one curve to winding's contour.
 */
void Add(AddCurve , AddWinding );

/* Makes a PathOps context: an instance of the PathOps engine. Optional caller data may be added.
 */
Context* CreateContext(AddContext );

/* Deletes a PathOps context, and frees any memory associated with that context.
 */
void DeleteContext(Context* );

/* Makes a PathOps contour: a collection of curves. Optional caller data may be added.
 */
Contour* CreateContour(AddContour );

/* Returns error code of previous call.
 */
ContextError Error(Context* );

/* Removes curves added to contour. Remaining state remains.
 */
void ResetContour(Contour* );

/* Operate on curves provided by Add(). Calls curve output callback with path output.
 */
void Resolve(Context* , PathOutput );

void SetContextCallBacks(Context* ,  EmptyNativePath, MakeLine, SetLineType);

CurveType SetCurveCallBacks(Context* , AxisRawHit,
		CurveHull, CurveIsFinite, CurveIsLine,
		SetBounds, CurveNormal, CurveOutput, CurvePinCtrl,
		CurveReverse, CurveTangent, CurvesEqual, PtAtT, DoublePtAtT,
		PtCount, Rotate, SubDivide, XYAtT
		OP_DEBUG_DUMP_PARAMS(DebugDumpCurveName, DebugDumpCurveExtra)
		OP_DEBUG_IMAGE_PARAMS(DebugAddToPath)
	);

void SetWindingCallBacks(Contour* , WindingAdd, WindingKeep ,
		WindingSubtract , WindingVisible, WindingZero
		OP_DEBUG_DUMP_PARAMS(DebugDumpContourIn, DebugDumpContourOut, DebugDumpContourExtra)
		OP_DEBUG_IMAGE_PARAMS(DebugImageOut, DebugNativePath, DebugGetDraw, DebugSetDraw, 
				DebugIsOpp)
);

#if OP_DEBUG
void Debug(Context* , OpDebugData& );
#endif

} // namespace PathOpsV0Lib

#endif
