// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef CutWinding_DEFINED
#define CutWinding_DEFINED

#include "curves/FrameWinding.h"

namespace PathOpsV0Lib {

// cut discards frame with end that does not connect, as well as all singly-linked connected frames
// cut keeps frame with non-zero 'left', and connects it to kept fill 
// keep left connects down/right frame to cw loop formed by fill
// keep right connects down/right frame to ccw loop formed by fill

// both winding and sumWinding come from the same edge
inline WindKeep cutKeepFunc(Context* , Winding winding, Winding sumWinding) {
	FrameData wind(winding);
	FrameData sum(sumWinding);
	if (FrameFill::fill == wind.isFrame) {
        if (!wind.left || (sum.left && sum.left != wind.left))
             return WindKeep::Discard;
        return sum.left ? WindKeep::Start : WindKeep::End;
    }
	return sum.left ? WindKeep::End : WindKeep::Discard;
}

inline void cutCallbacks(Context* context) {
    SetWindingCallbacks(context, { frameAddFunc, cutKeepFunc, frameVisibleFunc, 
			frameZeroFunc, frameSubtractFunc, frameIntersectFunc });
}

// CutData (context user data) contains desired output loop direction, and tracks output pass.
enum class CutDirection {
    clockwise,
    counterclockwise
};

enum class CutPass {
    checkDirection,
    discardFill,
    discardFrame,
    outputLoop
};

struct CutData {
    CutData(CutDirection dir)
        : direction(dir) {
    }

    std::array<OpPoint, 3> corner;
    OpPoint priorPt;
    OpPoint firstPt;
    CutDirection direction; 
    CutPass pass = CutPass::checkDirection;
    bool sawFrame = false;
};

inline int cutMaxLoopsFunc(Context* ) {
    return 3;
}

inline Context* cutContext(ContextUserData* userData, CurveOutput output = nullptr) {
    Context* context = CreateContext(userData);
    ContextCallbacks contextCallbacks { output };
    contextCallbacks.maxLoopsFuncPtr = cutMaxLoopsFunc;
    SetContextCallbacks(context, contextCallbacks);
#if OP_DEBUG
    OpDebugData debugData(false);
    Debug(context, debugData);
	SetDebugContextCallbacks(context, { 
        frameDebugIsFill
        OP_DEBUG_DUMP_PARAMS(nullptr, frameDumpOutFunc, frameDumpSetFunc)
        OP_DEBUG_IMAGE_PARAMS(frameImageOutXFunc, frameImageOutFunc, frameColorFuncPtr)
    });
#endif
    return context;
}



#if OP_DEBUG_DUMP
#define CUT_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(cutKeepFunc), \
    OP_TAGGED_FUNCTION(cutMaxLoopsFunc), \

#endif
}

#endif
