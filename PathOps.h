// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef PathOps_DEFINED
#define PathOps_DEFINED

#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

// makes a context: an instance of the PathOps engine
Context* CreateContext(ContextUserData* contextUserData = nullptr);

// deletes a context, and frees any memory associated with that context
void DeleteContext(Context* );

// optional user data associated with this instance
ContextUserData* UserData(Context* );

// makes an empty contour: a collection of curves that share a winding
Contour* CreateContour(Context* , WindingData , size_t );

// returns new empty contour, or argument if already empty
Contour* Clone(Contour* );

// adds a curve to the contour
void Add(Contour* , AddCurve);
void Add(Contour* , Curve);

// returns error code of previous call
ContextError Error(Context* );

// removes curves added to contour; callbacks are unaffected
void ResetContour(Contour* );

// operate on added curves; calls curve output callback with path output (if not null)
// returns user-defined non-zero value if winding condition was met
WindingCondition Resolve(Context* );

// global callbacks
void SetContextCallbacks(Context* , ContextCallbacks );

// sets the context into an error state
void SetError(Context* , ContextError );

// error callback; allows overriding error behavior
void SetErrorHandler(Context* , ErrorDispatch );

// curve callbacks; describes geometry between endpoints
void SetCurveCallbacks(Context* , int nativeType, CurveCallbacks );

// winding callbacks; specifies which curves are kept and discarded
void SetWindingCallbacks(Context* , WindingCallbacks ); 

}

#endif
