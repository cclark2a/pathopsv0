// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef FrameWinding_DEFINED
#define FrameWinding_DEFINED

#include "PathOps.h"

namespace PathOpsV0Lib {


enum class FrameFill {
    fill,
	frame
};

struct FrameWinding {
    FrameWinding(FrameFill frameFill) 
		: isFrame(frameFill) {
	}

    FrameWinding(FrameFill frameFill, int windValue) 
		: left(windValue)
		, isFrame(frameFill) {
	}

    FrameWinding(Winding w) {
        OP_ASSERT(w.size == sizeof(FrameWinding));
        std::memcpy(this, w.data, sizeof(FrameWinding));
	}

	void copyTo(Winding& w) const {
		OP_ASSERT(w.size == sizeof(FrameWinding));
		std::memcpy(w.data, this, sizeof(FrameWinding));
	}

    int left = 1;
	FrameFill isFrame;
};

// winding is always frame; toAdd comes from another edge, and may be frame or fill
inline void frameAddFunc(Context* , Winding winding, Winding toAdd) {
	FrameWinding sum(winding);
	if (FrameFill::frame == sum.isFrame)
		return;
	FrameWinding addend(toAdd);
	if (FrameFill::frame == addend.isFrame)
		return;
	sum.left += addend.left;
	sum.copyTo(winding);
}

inline WindKeep frameDiscardFunc(Context* , Winding winding, Winding sumWinding) {
	FrameWinding wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameWinding sum(sumWinding);
	return !sum.left ? WindKeep::Start : WindKeep::Discard;
}

inline bool frameIntersectFunc(Context* , Winding l, Winding r) {
    FrameWinding left(l);
    FrameWinding right(r);
    return FrameFill::fill == left.isFrame || FrameFill::fill == right.isFrame;
}

// both winding and sumWinding come from the same edge
inline WindKeep frameKeepFunc(Context* , Winding winding, Winding sumWinding) {
	FrameWinding wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameWinding sum(sumWinding);
	return sum.left ? WindKeep::Start : WindKeep::Discard;
}

inline void frameZeroFunc(Context* , Winding toZero) {
    FrameWinding zero(FrameFill::fill, 0);
    zero.copyTo(toZero);
}

// winding is always frame; toAdd comes from another edge, and may be frame or fill
inline void frameSubtractFunc(Context* , Winding winding, Winding toSubtract) {
	FrameWinding difference(winding);
	if (FrameFill::frame == difference.isFrame)
		return;
	FrameWinding subtrahend(toSubtract);
	if (FrameFill::frame == subtrahend.isFrame)
		return;
	difference.left -= subtrahend.left;
	difference.copyTo(winding);
}

inline bool frameVisibleFunc(Context* , Winding winding) {
    FrameWinding test(winding);
    return !!test.left;
}

inline void frameCallbacks(Context* context) {
    SetWindingCallbacks(context, { frameAddFunc, frameKeepFunc, frameVisibleFunc, 
			frameZeroFunc, frameSubtractFunc, frameIntersectFunc });
}

#if OP_DEBUG_DUMP
inline std::string frameDumpOutFunc(Winding winding) {
    FrameWinding frameWinding(winding);
    std::string s = "{" + STR(frameWinding.left) + ", " + STR((int) frameWinding.isFrame) + "}";
    return s;
}
#endif

#if OP_DEBUG_IMAGE
inline std::string frameImageOutFunc(Winding winding, int index) {
    if (index > 0)
        return "-";
    FrameWinding unaryWinding(winding);
    std::string s = STR(unaryWinding.left);
    return s;
}
#endif

}

#endif
