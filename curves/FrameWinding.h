// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef FrameWinding_DEFINED
#define FrameWinding_DEFINED

#include "PathOps.h"
#include "DebugOps.h"

namespace PathOpsV0Lib {


enum class FrameFill {
    fill,
	frame
};

struct FrameData {
    FrameData(FrameFill frameFill) 
		: isFrame(frameFill) {
        if (FrameFill::frame == frameFill)
            left = 0;
	}

    FrameData(FrameFill frameFill, int windValue) 
		: left(windValue)
		, isFrame(frameFill) {
	}

    FrameData(Winding w) {
        OP_ASSERT(w.size == sizeof(FrameData));
        std::memcpy(this, w.data, sizeof(FrameData));
	}

	void copyTo(Winding& w) const {
		OP_ASSERT(w.size == sizeof(FrameData));
		std::memcpy(w.data, this, sizeof(FrameData));
	}

    int left = 1;
	FrameFill isFrame = FrameFill::fill;
};

struct FrameWinding {
    FrameWinding(Context* context, FrameFill frameFill);

    Winding winding;
    FrameData data;
};

inline void frameAddFunc(Context* , Winding winding, Winding toAdd) {
	FrameData sum(winding);
	FrameData addend(toAdd);
	sum.left += addend.left;
	sum.copyTo(winding);
}

inline WindKeep frameDiscardFunc(Context* , Winding winding, Winding sumWinding) {
	FrameData wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameData sum(sumWinding);
	return !sum.left ? WindKeep::Start : WindKeep::Discard;
}

inline bool frameWoundFunc(Context* , Winding winding) {
	FrameData wind(winding);
    return !!wind.left;
}

inline bool frameIntersectFunc(Context* , Winding l, Winding r) {
    FrameData left(l);
    FrameData right(r);
    return FrameFill::fill == left.isFrame || FrameFill::fill == right.isFrame;
}

// both winding and sumWinding come from the same edge
inline WindKeep frameKeepFunc(Context* , Winding winding, Winding sumWinding) {
	FrameData wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameData sum(sumWinding);
	return sum.left ? WindKeep::Start : WindKeep::Discard;
}

inline void frameZeroFunc(Context* , Winding toZero) {
    FrameData zero(FrameFill::fill, 0);
    zero.copyTo(toZero);
}

inline void frameSubtractFunc(Context* , Winding winding, Winding toSubtract) {
	FrameData difference(winding);
	FrameData subtrahend(toSubtract);
	difference.left -= subtrahend.left;
	difference.copyTo(winding);
}

inline bool frameVisibleFunc(Context* , Winding winding) {
    FrameData test(winding);
    return FrameFill::frame == test.isFrame || !!test.left;
}

inline void frameCallbacks(Context* context) {
    SetWindingCallbacks(context, { frameAddFunc, frameKeepFunc, frameSubtractFunc, frameWoundFunc,
            frameVisibleFunc,  frameZeroFunc, frameIntersectFunc });
}

#if OP_DEBUG
inline bool frameDebugIsFill(Winding winding) {
    FrameData frameData(winding);
    return FrameFill::fill == frameData.isFrame;
}
#endif

#if OP_DEBUG_DUMP
inline std::string frameDumpOutFunc(Winding winding) {
    FrameData data(winding);
    return "{" + STR(data.left) + (FrameFill::frame == data.isFrame ? ":f}" : "}");
}

inline void frameDumpSetFunc(const char*& str, Winding& winding) {
    int left = OpDebugReadSizeT(str);
    FrameFill frameFill = OpDebugOptional(str, "fr") ? FrameFill::frame : FrameFill::fill;
    FrameData frameData(frameFill, left);
    frameData.copyTo(winding);
}

#define FRAME_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(frameAddFunc), \
    OP_TAGGED_FUNCTION(frameDiscardFunc), \
    OP_TAGGED_FUNCTION(frameIntersectFunc), \
    OP_TAGGED_FUNCTION(frameKeepFunc), \
    OP_TAGGED_FUNCTION(frameWoundFunc), \
    OP_TAGGED_FUNCTION(frameZeroFunc), \
    OP_TAGGED_FUNCTION(frameSubtractFunc), \
    OP_TAGGED_FUNCTION(frameVisibleFunc), \
    OP_TAGGED_FUNCTION(frameDebugIsFill), \
    OP_TAGGED_FUNCTION(frameDumpOutFunc), \
    OP_TAGGED_FUNCTION(frameDumpSetFunc), \

#endif

#if OP_DEBUG_IMAGE
inline std::string frameImageOutXFunc(Winding winding) {
    FrameData data(winding);
    return STR(data.left) + (FrameFill::frame == data.isFrame ? "fr" : "");
}

// !!! deprecated
inline std::string frameImageOutFunc(Winding winding, int index) {  // deprecated
    if (index > 0)
        return "-";
    FrameData data(winding);
    std::string s = STR(data.left);
    return s;
}

inline uint32_t frameColorFuncPtr(Winding winding, DebugEdgeType edgeType) {
    FrameData data(winding);
	if (edgeType.disabled)
		return FrameFill::fill == data.isFrame ? red : darkRed;
	else if (edgeType.inOutput)
		return FrameFill::fill == data.isFrame ? orange : darkOrange;
	else if (edgeType.unsortable)
		return FrameFill::fill == data.isFrame ? purple : darkViolet;
	else if (edgeType.curveCurve) {
		if (edgeType.ccOverlaps)
			return FrameFill::fill == data.isFrame ? orange : darkGreen;
		else
			return FrameFill::fill == data.isFrame ? purple : darkViolet;
	}
    return FrameFill::fill == data.isFrame ? debugBlack : darkGreen;
}

#define FRAME_IMAGE_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(frameImageOutXFunc), \
    OP_TAGGED_FUNCTION(frameImageOutFunc), \
    OP_TAGGED_FUNCTION(frameColorFuncPtr), \

#endif

inline Context* frameContext(CurveOutput output = nullptr) {
    Context* context = CreateContext();
    SetContextCallbacks(context, { output });
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

inline FrameWinding::FrameWinding(Context* context, FrameFill frameFill)
    : data(frameFill) {
    winding.contour = CreateContour(context, &data, sizeof(data));
#if OP_DEBUG
	SetDebugContourData(winding.contour, { &data, sizeof(data) }, DebugContourType::windingUserData);
#endif
}

}

#endif
