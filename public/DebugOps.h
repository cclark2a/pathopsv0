// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOps_DEFINED
#define DebugOps_DEFINED

#include "DebugOpsTypes.h"

namespace PathOpsV0Lib {

#if OP_DEBUG

void SetDebugData(Context* , OpDebugData& );
void SetDebugCurveCallbacks(Context* , int nativeCurveType , DebugCurveCallbacks );
// void SetDebugContourCallbacks(Contour* , DebugContourCallbacks );
void SetDebugContextCallbacks(Context* , DebugContextCallbacks );
void SetDebugCurveData(Contour* , DebugCurveData );
#endif

}

#endif
