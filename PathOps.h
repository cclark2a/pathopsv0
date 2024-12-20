// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOps_DEFINED
#define PathOps_DEFINED

#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

// adds one curve to winding's contour
void Add(AddCurve , AddWinding );

// makes a PathOps context: an instance of the PathOps engine
Context* CreateContext();

// deletes a PathOps context, and frees any memory associated with that context
void DeleteContext(Context* );

// makes a PathOps contour: a collection of curves
Contour* CreateContour(Context* );

// returns error code of previous call
ContextError Error(Context* );

// adjusts curves to place all numerical data in the same range
void Normalize(Context* );

// removes curves added to contour; callbacks are unaffected
void ResetContour(Contour* );

// operate on added curves; calls curve output callback with path output
void Resolve(Context* , PathOutput );

// global callbacks
void SetContextCallBacks(Context* ,  EmptyNativePath , MakeLine , SetLineType , MaxSignSwap ,
		MaxCurveCurve , MaxCurveCurve , MaxLimbs);

// sets the context into an error state
void SetError(Context* , ContextError );

// error callback; allows overriding error behavior
void SetErrorHandler(Context* , ErrorDispatch );

// curve callbacks; describes geometry between endpoints
CurveType SetCurveCallBacks(Context* , CurveOutput, AxisT = nullptr,
		CurveHull = nullptr, CurveIsFinite = nullptr, CurveIsLine = nullptr,
		SetBounds = nullptr, CurvePinCtrl = nullptr, CurveTangent = nullptr, 
		CurvesEqual = nullptr, PtAtT = nullptr, HullPtCount = nullptr, Rotate = nullptr, 
		SubDivide = nullptr, XYAtT = nullptr, CurveReverse = nullptr, CurveConst cut = nullptr, 
		CurveConst normalLimit = nullptr, CurveConst interceptLimit = nullptr);

// winding callbacks; specifies which curves are kept and discarded
void SetWindingCallBacks(Contour* , WindingAdd , WindingKeep ,
		WindingVisible , WindingZero , WindingSubtract = nullptr); 

}

#endif
