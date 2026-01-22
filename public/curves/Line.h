// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef Line_DEFINED
#define Line_DEFINED

#include "PathOps.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

namespace PathOpsV0Lib {

#if OP_TEST
struct DebugLine {
    CurveType curveType;
    size_t curveSize;
    OpPoint curveData[2];
};
#endif

inline size_t AddLine(Contour* contour, AddCurve curve) {
#if OP_TEST
    OP_ASSERT(sizeof(OpPoint) * 2 == sizeof(DebugLine::curveData));
    DebugLine debugLine { curve.type, sizeof(DebugLine::curveData) };
    memcpy(debugLine.curveData, curve.points, curve.size);
    SetDebugCurveData(contour, { (DebugCurve*) &debugLine, sizeof(debugLine) });
#endif
    if (curve.points[0] != curve.points[1])
        Add(contour, curve);
    return 1;
}

#if OP_DEBUG_SERIALIZE
inline std::string lineDebugDumpName() { 
    return "line"; 
}

#define LINE_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(lineDebugDumpName), \

#endif

inline void lineCallbacks(Context* context, CurveType nativeCurveType) {
    SetCurveCallbacks(context, nativeCurveType, { } );
    OP_DEBUG_CODE(SetDebugCurveCallbacks(context, nativeCurveType, { debugLineScale
            OP_DEBUG_DUMP_PARAMS(lineDebugDumpName, nullptr) }));
}

}

#endif
