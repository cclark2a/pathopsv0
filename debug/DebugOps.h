// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOps_DEFINED
#define DebugOps_DEFINED

#include "DebugOpsTypes.h"

namespace PathOpsV0Lib {

#if OP_DEBUG

void Debug(Context* , OpDebugData& );
void SetDebugCurveCallbacks(Context* , CurveType , DebugCurveCallbacks );
void SetDebugContourCallbacks(Contour* , DebugContourCallbacks );
void SetDebugContextCallbacks(Context* , DebugContextCallbacks );
void SetDebugContourData(Contour* , DebugContourData );
void SetDebugContextData(Context* , DebugContextData );

#if WINDER_CONTOUR_EXPERIMENT
DebugContourData GetDebugContourData(Contour* );
#endif

#endif

}

#endif
