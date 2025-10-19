// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef Line_DEFINED
#define Line_DEFINED

#include "PathOps.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

namespace PathOpsV0Lib {

inline size_t AddLine(Contour* contour, AddCurve curve) {
#if OP_DEBUG_IMAGE
    Curve line { curve.context, (CurveData*) curve.points, curve.size, curve.type };
    SetDebugContourImage(contour, line);
#endif
    Add(contour, curve);
    return 1;
}

#if OP_DEBUG_DUMP
inline std::string lineDebugDumpName() { 
    return "line"; 
}

#define LINE_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(lineDebugDumpName), \

#endif

inline void lineCallbacks(Context* context, CurveType nativeCurveType) {
    SetCurveCallbacks(context, nativeCurveType, { } );
    OP_DEBUG_CODE(SetDebugCurveCallbacks(context, nativeCurveType, { debugLineScale
        OP_DEBUG_DUMP_PARAMS(lineDebugDumpName, nullptr)
//        OP_DEBUG_IMAGE_PARAMS_OLD(debugLineToSkPath) 
        }));
}

}

#endif
