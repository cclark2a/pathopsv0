// (c) 2025, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG_DUMP

#define OP_TAGGED_FUNCTION(f) { reinterpret_cast<DebugFunction>(PathOpsV0Lib::f), #f }

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/ConicBezier.h"
#include "curves/CubicBezier.h"
#include "curves/UnaryWinding.h"
#include "curves/BinaryWinding.h"
#include "curves/FrameWinding.h"
#include "curves/CutWinding.h"

#include "debug/OpDebugDump.h"

struct DebugTags {
    DebugFunction function;
    std::string tag;
};

std::vector<DebugTags> debugTags {
    LINE_TAGGED_FUNCTIONS
    QUAD_TAGGED_FUNCTIONS
    CONIC_TAGGED_FUNCTIONS
    CUBIC_TAGGED_FUNCTIONS
    UNARY_WINDING_TAGGED_FUNCTIONS
    BINARY_WINDING_TAGGED_FUNCTIONS
    FRAME_WINDING_TAGGED_FUNCTIONS
    CUT_WINDING_TAGGED_FUNCTIONS
#if OP_DEBUG
    DEBUG_SCALE_TAGGED_FUNCTIONS
#endif
#if OP_DEBUG_IMAGE
    UNARY_IMAGE_TAGGED_FUNCTIONS
    BINARY_IMAGE_TAGGED_FUNCTIONS
    FRAME_IMAGE_TAGGED_FUNCTIONS
    DEBUG_TO_SKPATH_TAGGED_FUNCTIONS
#endif
};

#undef OP_TAGGED_FUNCTION

// !!! if this becomes a bottleneck in the debugger, add binary sort
std::string debugFindTag(DebugFunction function) {
    if (nullptr == function)
        return "0 ";
    for (DebugTags& debugTag : debugTags) {
        if (function == debugTag.function)
            return debugTag.tag + " ";
    }
    OP_ASSERT(0);
    return "??? ";
}

DebugFunction debugFindFunction(const char*& str) {
    std::string tag = OpDebugLabel(str);
    if ("0" == tag)
        return nullptr;
    for (DebugTags& debugTag : debugTags) {
        if (tag == debugTag.tag)
            return debugTag.function;
    }
    OP_ASSERT(0);
    return nullptr;
}

#endif
