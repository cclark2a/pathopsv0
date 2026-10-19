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
		: value(FrameFill::fill == frameFill)  // frame:0 fill:1
        , isFrame(frameFill) {
	}

    FrameData(FrameFill frameFill, int initial) 
		: value(initial)
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

    int value;
	FrameFill isFrame;
};

struct FrameWinding {
    FrameWinding(Context* context, FrameFill frameFill);

    Winding winding;
    FrameData data;
};

inline void frameAddFunc(Winding winding, Winding toAdd) {
	FrameData sum(winding);
	FrameData addend(toAdd);
	sum.value += addend.value;
	sum.copyTo(winding);
}

inline WindKeep frameDiscardFunc(Winding winding, Winding sumWinding) {
	FrameData wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameData sum(sumWinding);
	return !sum.value ? WindKeep::Start : WindKeep::Discard;
}

inline bool frameWoundFunc(Winding winding) {
	FrameData wind(winding);
    return !!wind.value;
}

inline bool frameIntersectFunc(Winding l, Winding r) {
    FrameData value(l);
    FrameData right(r);
    return FrameFill::fill == value.isFrame || FrameFill::fill == right.isFrame;
}

// both winding and sumWinding come from the same edge
inline WindKeep frameKeepFunc(Winding winding, Winding sumWinding) {
	FrameData wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameData sum(sumWinding);
	return sum.value ? WindKeep::Start : WindKeep::Discard;
}

inline void frameZeroFunc(Winding toZero) {
    FrameData zero(FrameFill::fill, 0);
    zero.copyTo(toZero);
}

inline void frameSubtractFunc(Winding winding, Winding toSubtract) {
	FrameData difference(winding);
	FrameData subtrahend(toSubtract);
	difference.value -= subtrahend.value;
	difference.copyTo(winding);
}

inline bool frameVisibleFunc(Winding winding) {
    FrameData test(winding);
    return FrameFill::frame == test.isFrame || !!test.value;
}

inline void frameCallbacks(Context* context) {
    SetWindingCallbacks(context, { frameAddFunc, frameKeepFunc, frameSubtractFunc, 
            frameWoundFunc, frameVisibleFunc,  frameZeroFunc, frameIntersectFunc });
}

#if OP_DEBUG || OP_DEBUGGER
inline bool frameDebugIsFill(Winding winding) {
    FrameData frameData(winding);
    return FrameFill::fill == frameData.isFrame;
}
#endif

#if OP_DEBUG_SERIALIZE

inline std::string frameDumpOutFunc(Winding winding) {
    FrameData data(winding);
    return STR(data.value) + (FrameFill::frame == data.isFrame ? "fr" : "");
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

inline void frameDumpSetFunc(const char*& str, Winding& winding) {
    int value = OpDebugReadSizeT(str);
    FrameFill frameFill = OpDebugOptional(str, "fr") ? FrameFill::frame : FrameFill::fill;
    FrameData frameData(frameFill, value);
    frameData.copyTo(winding);
}

// !!! deprecated
inline std::vector<std::string> frameImageNamesFunc() {
    return { "fill", "frame" };
}

#define DUMP_FRAME_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(frameDumpSetFunc), \
    OP_TAGGED_FUNCTION(frameImageNamesFunc), \

#endif

#if OP_DEBUGGER

inline uint32_t frameColorFunc(Winding winding, DebugEdgeType edgeType) {
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

#define FRAME_COLOR_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(frameColorFunc), \

#else

#define FRAME_COLOR_TAGGED_FUNCTIONS \
    OP_TAGGED_EMPTY_FUNCTION(frameColorFunc), \

#endif

inline Context* frameContext(CurveOutput output = nullptr) {
    Context* context = CreateContext();
#if OP_DEBUG && OP_DEBUG_ONE_TEST
    SetDebugData(context, { "frameTest" } );
#endif
    SetContextCallbacks(context, { output });
    frameCallbacks(context);
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

inline FrameWinding::FrameWinding(Context* context, FrameFill frameFill)
    : data(frameFill) {
    winding.contour = CreateContour(context, &data, sizeof(data));
}

}

#endif
