// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef CutWinding_DEFINED
#define CutWinding_DEFINED

#include "curves/FrameWinding.h"

namespace PathOpsV0Lib {

// both winding and sumWinding come from the same edge
inline WindKeep cutKeepLeftFunc(Context* , Winding winding, Winding sumWinding) {
	FrameData wind(winding);
	FrameData sum(sumWinding);
	if (FrameFill::fill == wind.isFrame)
		return !wind.left && sum.left ? WindKeep::Start : WindKeep::Discard;
	return sum.left ? WindKeep::End : WindKeep::Discard;
}

inline WindKeep cutKeepRightFunc(Context* , Winding winding, Winding sumWinding) {
	FrameData wind(winding);
	FrameData sum(sumWinding);
	if (FrameFill::fill == wind.isFrame)
		return wind.left && !sum.left ? WindKeep::End : WindKeep::Discard;
	return sum.left ? WindKeep::Start : WindKeep::Discard;
}

inline void cutLeftCallbacks(Context* context) {
    SetWindingCallbacks(context, { frameAddFunc, cutKeepLeftFunc, frameVisibleFunc, 
			frameZeroFunc, frameSubtractFunc, frameIntersectFunc });
}

inline void cutRightCallbacks(Context* context) {
    SetWindingCallbacks(context, { frameAddFunc, cutKeepRightFunc, frameVisibleFunc, 
			frameZeroFunc, frameSubtractFunc, frameIntersectFunc });
}

#if OP_DEBUG_DUMP
#define CUT_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(cutKeepLeftFunc), \
    OP_TAGGED_FUNCTION(cutKeepRightFunc), \

#endif
}

#endif
