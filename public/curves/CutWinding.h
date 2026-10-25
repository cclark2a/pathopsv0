// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef CutWinding_DEFINED
#define CutWinding_DEFINED

#include "curves/FrameWinding.h"
#include "PathOps.h"

namespace PathOpsV0Lib {

// cut discards frame with end that does not connect, as well as all singly-linked connected frames
// cut keeps frame with non-zero 'value', and connects it to kept fill 
// keep left connects down/right frame to cw loop formed by fill
// keep right connects down/right frame to ccw loop formed by fill

// both winding and sumWinding come from the same edge
inline WindKeep cutKeepFunc(Winding winding, Winding sumWinding) {
	FrameData wind(winding);
	FrameData sum(sumWinding);
	if (FrameFill::fill == wind.isFrame) {
        if (!wind.value || (sum.value && sum.value != wind.value))
             return WindKeep::Discard;
        return sum.value ? WindKeep::Start : WindKeep::End;
    }
	return sum.value ? WindKeep::End : WindKeep::Discard;
}

inline void cutCallbacks(Context* context) {
    SetWindingCallbacks(context, { frameAddFunc, cutKeepFunc, frameSubtractFunc, frameWoundFunc,
            frameVisibleFunc, frameLinkableFunc, frameZeroFunc, frameIntersectFunc });
}

// CutData (context user data) contains desired output loop direction, and tracks output pass.
enum class CutDirection {
    clockwise,
    counterclockwise
};

inline OpPtT cutTopLeft(const Curve& curve) {
    OpPoint start = curve.data->start;
    OpPoint end = curve.data->end;
    return start.y < end.y || (start.y == end.y && start.x < end.x) 
            ? OpPtT(start, 0) : OpPtT(end, 1);
}

struct CutTop {
    CutTop() {
    }

    CutTop(const Curve& curve, bool rev) {
        top = curve;
        tl = cutTopLeft(curve);
        reversed = rev;
    }

    OpVector tangent(Context* context) {
        const CurveCallbacks* calls = GetCurveCallbacks(context, top.type);
        if (!calls)
            return OpVector();
        OpVector tangent = calls->curveTangentFuncPtr 
                ? (*calls->curveTangentFuncPtr)(top, tl.t)
                : top.data->end - top.data->start;
        return reversed ? -tangent : tangent;
    }

    PathOpsV0Lib::Curve top;
    OpPtT tl;
    bool reversed;
};

struct CutData {
    CutData(CutDirection dir)
        : direction(dir) {
    }

    CutTop top;
    CutTop mate;
    CutDirection direction;
};

inline WindKeep cutKeepLoopFunc(Output output) {
    const Curve& curve = output.curve;
    LoopAttribute loopAttr = output.attribute;
    Context* context = curve.context;
    PathOpsV0Lib::ContextUserData data = UserData(context, UserDataType::outData);
    CutData* cutData = (CutData*) data.data;
    auto loopAttributeSet = [loopAttr](LoopAttribute attr) {
        return !!((int) loopAttr & (int) attr);
    };
    bool firstPt = loopAttributeSet(LoopAttribute::first);
    bool lastPt = loopAttributeSet(LoopAttribute::last);
    bool reversed = loopAttributeSet(LoopAttribute::reversed);
    if (firstPt)
        cutData->top = CutTop(curve, reversed);
    else {
        OpPtT testTL = cutTopLeft(curve);
        OpPtT cutTL = cutData->top.tl;
        if (cutTL.pt.y > testTL.pt.y || (cutTL.pt.y == testTL.pt.y && cutTL.pt.x > testTL.pt.x))
            cutData->top = CutTop(curve, reversed);
        else if (cutTL.pt == testTL.pt) 
            cutData->mate = CutTop(curve, reversed);
    }
    if (lastPt) {
        OpVector cutTangent = cutData->top.tangent(context);
        OpVector testTangent = cutData->mate.tangent(context);
        cutTangent.normalize();
        testTangent.normalize();
        if ((cutTangent.cross(testTangent) < 0)
                == (CutDirection::clockwise == cutData->direction))
            return WindKeep::Discard;
    }
    return WindKeep::Start;
}

#define CUT_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(cutKeepFunc), \
    OP_TAGGED_FUNCTION(cutKeepLoopFunc), \

inline Context* cutContext(ContextUserData userData, CurveOutput output = nullptr) {
    Context* context = CreateContext();
#if OP_DEBUG && OP_DEBUG_ONE_TEST
    SetDebugData(context, { "cutTest" } );
#endif
    AddUserData(context, userData);
    ContextCallbacks contextCallbacks { output, nullptr, cutKeepLoopFunc };
    SetContextCallbacks(context, contextCallbacks);
#if OP_DEBUG
	SetDebugContextCallbacks(context, { 
            frameDebugIsFill, nullptr, nullptr
            OP_DEBUG_SERIALIZE_PARAMS(frameDumpOutFunc, frameDumpSetFunc, nullptr,
            frameDumpOutFunc, frameImageNamesFunc, nullptr, nullptr)
#if OP_DEBUGGER
            , frameColorFunc
#else
            , "frameColorFunc"
#endif
    });
#endif
    return context;
}

}

#endif
