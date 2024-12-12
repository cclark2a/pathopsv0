// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOps_DEFINED
#define PathOps_DEFINED

#include "OpDebug.h"
#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

/* usage:


*/

// functions
// Adds one curve to winding's contour.
void Add(AddCurve , AddWinding );

// Makes a PathOps context: an instance of the PathOps engine. Optional caller data may be added.
Context* CreateContext(AddContext );

// Deletes a PathOps context, and frees any memory associated with that context.
void DeleteContext(Context* );

// Makes a PathOps contour: a collection of curves. Optional caller data may be added.
Contour* CreateContour(AddContour );

// Returns error code of previous call.
ContextError Error(Context* );

// Adjusts curves to place all numerical data in the same range 
void Normalize(Context* );

// Removes curves added to contour. Remaining state remains.
void ResetContour(Contour* );

/* Operate on curves provided by Add(). Calls curve output callback with path output.
 */
void Resolve(Context* , PathOutput );

void SetContextCallBacks(Context* ,  EmptyNativePath, MakeLine , SetLineType , MaxSignSwap ,
		MaxCurveCurve , MaxCurveCurve , MaxLimbs);

void SetError(Context* , ContextError );
void SetErrorHandler(Context* , ErrorDispatch );

// !!! reorganize this so that required parameters are first, quad nullptr params second,
//     line nullptr params last; then add nullptr as default parameter values
// !!! make all debug parameters optional throughout code
CurveType SetCurveCallBacks(Context* , AxisT,
		CurveHull, CurveIsFinite, CurveIsLine,
		SetBounds, CurveOutput, CurvePinCtrl,
		CurveReverse, CurveTangent, CurvesEqual, PtAtT,
		HullPtCount, Rotate, SubDivide, XYAtT, 
		CurveConst cut, CurveConst normalLimit, CurveConst interceptLimit);

void SetWindingCallBacks(Contour* , WindingAdd, WindingKeep ,
		WindingSubtract , WindingVisible, WindingZero  OP_DEBUG_PARAMS(DebugBitOper)
		OP_DEBUG_DUMP_PARAMS(DebugDumpContourIn, DebugDumpContourOut, DebugDumpContourExtra)
		OP_DEBUG_IMAGE_PARAMS(DebugImageOut, DebugNativePath, DebugGetDraw, DebugSetDraw, 
				DebugIsOpp)
);

#if OP_DEBUG
void Debug(Context* , OpDebugData& );
void SetDebugCurveCallBacks(Context* , CurveType , DebugScale 
		OP_DEBUG_DUMP_PARAMS(DebugDumpCurveName, DebugDumpCurveExtra)
		OP_DEBUG_IMAGE_PARAMS(DebugAddToPath) );
#endif

} // namespace PathOpsV0Lib

#endif
