// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOps_DEFINED
#define DebugOps_DEFINED

#include "DebugOpsTypes.h"

namespace PathOpsV0Lib {

#if OP_DEBUG

void Debug(Context* , OpDebugData& );
void SetDebugCurveCallbacks(Context* , int nativeCurveType , DebugCurveCallbacks );
void SetDebugContourCallbacks(Contour* , DebugContourCallbacks );
void SetDebugContextCallbacks(Context* , DebugContextCallbacks );
void SetDebugContourData(Contour* , DebugContourData , DebugContourType );
// void SetDebugContextData(Context* , DebugContextData , DebugContextType );

#if OP_DEBUG_IMAGE
void SetDebugContourImage(Contour* , Curve );
#endif

#endif

}

#endif
