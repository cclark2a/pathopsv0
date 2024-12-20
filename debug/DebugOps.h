// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef DebugOps_DEFINED
#define DebugOps_DEFINED

#include "DebugOpsTypes.h"

namespace PathOpsV0Lib {

#if OP_DEBUG

void Debug(Context* , OpDebugData& );
void SetDebugCurveCallBacks(Context* , CurveType , DebugScale 
		OP_DEBUG_DUMP_PARAMS(DebugDumpCurveName, DebugDumpCurveExtra)
		OP_DEBUG_IMAGE_PARAMS(DebugAddToPath) );

void SetDebugWindingCallBacks(Contour* , DebugCallerData , PathOpsV0Lib::DebugBitOper
		OP_DEBUG_DUMP_PARAMS(DebugDumpContourIn, DebugDumpContourOut, DebugDumpContourExtra)
		OP_DEBUG_IMAGE_PARAMS(DebugImageOut, DebugNativePath, DebugGetDraw, DebugSetDraw, 
		DebugIsOpp) );

#endif

}

#endif
