// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOps_DEFINED
#define DebugOps_DEFINED

#include "DebugOpsTypes.h"

namespace PathOpsV0Lib {

#if OP_DEBUG

void Debug(Context* , OpDebugData& );
void SetDebugCurveCallBacks(Context* , CurveType , DebugCurveCallBacks );
void SetDebugContourCallBacks(Contour* , DebugContourCallBacks );
void SetDebugContextCallBacks(Context* , DebugContextCallBacks );
void SetDebugContourData(Contour* , DebugContourData );
void SetDebugContextData(Context* , DebugContextData );

#if WINDER_CONTOUR_EXPERIMENT
DebugContourData GetDebugContourData(Contour* );
#endif

#endif

}

#endif
