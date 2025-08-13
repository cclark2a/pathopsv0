// (c) 2023, Cary Clark cclark2@gmail.com

#include "curves/QuadBezier.h"

using namespace PathOpsV0Lib;

// curve types
static CurveType lineType = 1;
static CurveType quadType = 2;
constexpr size_t lineSize = sizeof(OpPoint) * 2;
constexpr size_t quadSize = sizeof(OpPoint) * 3;

enum class FrameFill {
	frame,
	fill
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
static void frameAddFunc(Winding winding, Winding toAdd) {
	FrameWinding sum(winding);
	if (FrameFill::frame == sum.isFrame)
		return;
	FrameWinding addend(toAdd);
	if (FrameFill::frame == addend.isFrame)
		return;
	sum.left += addend.left;
	sum.copyTo(winding);
}

// both winding and sumWinding come from the same edge
static WindKeep frameKeepFunc(Winding winding, Winding sumWinding) {
	FrameWinding wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameWinding sum(sumWinding);
	return sum.left ? WindKeep::Start : WindKeep::Discard;
}

// winding is always frame; toAdd comes from another edge, and may be frame or fill
static void frameSubtractFunc(Winding winding, Winding toSubtract) {
	FrameWinding difference(winding);
	if (FrameFill::frame == difference.isFrame)
		return;
	FrameWinding subtrahend(toSubtract);
	if (FrameFill::frame == subtrahend.isFrame)
		return;
	difference.left -= subtrahend.left;
	difference.copyTo(winding);
}

static bool frameVisibleFunc(Winding winding) {
    FrameWinding test(winding);
    return !!test.left;
}

static void frameZeroFunc(Winding toZero) {
    FrameWinding zero(FrameFill::fill, 0);
    zero.copyTo(toZero);
}

static bool allowDisjointLines(ContextError err, Context* , Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

void ContainsTest() {
    Context* context = CreateContext();
    SetContextCallbacks(context, { } );

    SetCurveCallbacks(context, lineType, { } );
    quadCallbacks(context, quadType);

    // example: given points describing a pair of closed loops with quadratic Beziers, find
    //          their intersection
    FrameWinding windingData(FrameFill::frame, 1);
    Winding frameWinding { &windingData, sizeof(windingData) };
    Contour* frameContour = CreateContour(context, frameWinding);

    // note that the data below omits start points for curves that match the previous end point
                      //  start      end      control
    OpPoint contour1[] { { 2, 0 }, { 0, 2 }, { 1, 2 },  // quad: start, control, end
                                             { 2, 3 },  // line:                 end
                                             { 2, 0 },  // line:                 end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads(frameContour, { &contour1[0], quadSize, quadType } );
    Add(     frameContour, { &contour1[2], lineSize, lineType } );
    Add(     frameContour, { &contour1[3], lineSize, lineType } );

    FrameWinding fillData(FrameFill::fill, 1);
    Winding fillWinding { &fillData, sizeof(fillData) };
    Contour* fillContour = CreateContour(context, fillWinding);
    OpPoint contour2[] { { 0, 0 },           { 1, 1 },  // line: start,          end
                                   { 1, 3 }, { 0, 3 },  // quad:        control, end
                                             { 0, 0 },  // line:                 end
    };
    Add(     fillContour, { &contour2[0], lineSize, lineType } );
    AddQuads(fillContour, { &contour2[1], quadSize, quadType } );
    Add(     fillContour, { &contour2[3], lineSize, lineType } );

	SetErrorHandler(context, allowDisjointLines);
	Normalize(context);
    auto handleError = [context](bool result, bool expected) {
        if (ContextError::none != Error(context)) {
            OP_ASSERT(result == expected);
            exit(1);
        }
    };
    auto testOne = [context, handleError](bool expected) {
        SetWindingCallbacks(context, { frameAddFunc, frameKeepFunc, frameVisibleFunc, 
			    frameZeroFunc, frameSubtractFunc });
        bool isOutside = Resolve(context, nullptr);
        handleError(isOutside, expected);
    };
    testOne(true);
    DeleteContext(context);
}
