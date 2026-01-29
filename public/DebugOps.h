// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOps_DEFINED
#define DebugOps_DEFINED

#include "DebugOpsTypes.h"

namespace PathOpsV0Lib {

#if OP_DEBUG

OpDebugData& GetDebugData(Context* );
void SetDebugData(Context* , const OpDebugData& );
void SetDebugCurveCallbacks(Context* , int nativeCurveType , const DebugCurveCallbacks& );
void SetDebugContextCallbacks(Context* , const DebugContextCallbacks& );
void SetDebugCurveData(Contour* , const DebugCurveData& );

#endif

}

#endif
