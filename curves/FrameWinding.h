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
	FrameFill isFrame;
};

struct FrameWinding {
    FrameWinding(Context* context, FrameFill frameFill);

    Winding winding;
    FrameData data;
    Contour* contour;
};

// winding is always frame; toAdd comes from another edge, and may be frame or fill
inline void frameAddFunc(Context* , Winding winding, Winding toAdd) {
	FrameData sum(winding);
	if (FrameFill::frame == sum.isFrame)
		return;
	FrameData addend(toAdd);
	if (FrameFill::frame == addend.isFrame)
		return;
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

// winding is always frame; toAdd comes from another edge, and may be frame or fill
inline void frameSubtractFunc(Context* , Winding winding, Winding toSubtract) {
	FrameData difference(winding);
	if (FrameFill::frame == difference.isFrame)
		return;
	FrameData subtrahend(toSubtract);
	if (FrameFill::frame == subtrahend.isFrame)
		return;
	difference.left -= subtrahend.left;
	difference.copyTo(winding);
}

inline bool frameVisibleFunc(Context* , Winding winding) {
    FrameData test(winding);
    return !!test.left;
}

inline void frameCallbacks(Context* context) {
    SetWindingCallbacks(context, { frameAddFunc, frameKeepFunc, frameVisibleFunc, 
			frameZeroFunc, frameSubtractFunc, frameIntersectFunc });
}

#if OP_DEBUG_DUMP
inline std::string frameDumpOutFunc(Winding winding) {
    FrameData frameData(winding);
    std::string s = "{" + STR(frameData.left) + ", " + STR((int) frameData.isFrame) + "}";
    return s;
}

inline void frameDumpSetFunc(const char*& str, Winding& winding) {
    int left = OpDebugReadSizeT(str);
    FrameFill frameFill = (FrameFill) OpDebugReadSizeT(str);
    FrameData frameData(frameFill, left);
    frameData.copyTo(winding);
}

#define FRAME_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(frameAddFunc), \
    OP_TAGGED_FUNCTION(frameDiscardFunc), \
    OP_TAGGED_FUNCTION(frameIntersectFunc), \
    OP_TAGGED_FUNCTION(frameKeepFunc), \
    OP_TAGGED_FUNCTION(frameZeroFunc), \
    OP_TAGGED_FUNCTION(frameSubtractFunc), \
    OP_TAGGED_FUNCTION(frameVisibleFunc), \
    OP_TAGGED_FUNCTION(frameDumpOutFunc), \
    OP_TAGGED_FUNCTION(frameDumpSetFunc), \

#endif

#if OP_DEBUG_IMAGE
inline std::string frameImageOutXFunc(Winding winding) {
    FrameData data(winding);
    return STR(data.left) + ((bool) data.isFrame ? "f" : "");
}

inline std::string frameImageOutFunc(Winding winding, int index) {  // deprecated
    if (index > 0)
        return "-";
    FrameData data(winding);
    std::string s = STR(data.left);
    return s;
}

#define FRAME_IMAGE_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(frameImageOutXFunc), \
    OP_TAGGED_FUNCTION(frameImageOutFunc), \

#endif

inline Context* frameContext(CurveOutput output = nullptr) {
    Context* context = CreateContext();
    SetContextCallbacks(context, { output });
#if OP_DEBUG
    OpDebugData debugData(false);
    Debug(context, debugData);
	SetDebugContextCallbacks(context, { 
        OP_DEBUG_DUMP_CODE(nullptr, frameDumpOutFunc, frameDumpSetFunc)
        OP_DEBUG_IMAGE_PARAMS(frameImageOutXFunc, frameImageOutFunc)
    });
#endif
    return context;
}

inline FrameWinding::FrameWinding(Context* context, FrameFill frameFill)
    : data(frameFill) {
    winding.data = &data;
    winding.size = sizeof(data);
    contour = CreateContour(context, winding);
#if OP_DEBUG
	SetDebugContourData(contour, { &data, sizeof(data) }, DebugContourType::windingUserData );
#endif
}

}

#endif
