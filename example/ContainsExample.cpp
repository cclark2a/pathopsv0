// (c) 2023, Cary Clark cclark2@gmail.com

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/FrameWinding.h"

using namespace PathOpsV0Lib;

// curve types
constexpr CurveType lineType = 1;
constexpr CurveType quadType = 2;
constexpr size_t lineSize = sizeof(OpPoint) * 2;
constexpr size_t quadSize = sizeof(OpPoint) * 3;
constexpr WindingCondition framePartiallyContained = 1;
constexpr WindingCondition frameFullyContained = 2;

static bool allowDisjointLines(ContextError err, Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

static WindingCondition framePartiallyContainsFunc(Context* , WindKeep keep) {
    return WindKeep::Start == keep ? framePartiallyContained : 0;
}

static WindingCondition frameFullyContainsFunc(Context* , WindKeep keep) {
    return WindKeep::Start == keep ? frameFullyContained : 0;
}

void ContainsExample() {
    Context* context = frameContext();
    lineCallbacks(context, lineType);
    quadCallbacks(context, quadType);

    // example: given points describing a pair of closed loops with quadratic Beziers, find
    //          their intersection
    FrameWinding frameWinding(context, FrameFill::frame);

    // note that the data below omits start points for curves that match the previous end point
                      //  start    control     end
    OpPoint contour1[] { { 2, 0 }, { 0, 2 }, { 1, 2 },  // quad: start, control, end
                                             { 2, 3 },  // line:                 end
                                             { 2, 0 },  // line:                 end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads(frameWinding.winding.contour, { context, &contour1[0], quadSize, quadType } );
    AddLine( frameWinding.winding.contour, { context, &contour1[2], lineSize, lineType } );
    AddLine( frameWinding.winding.contour, { context, &contour1[3], lineSize, lineType } );

    FrameWinding fillWinding(context, FrameFill::fill);
    OpPoint contour2[] { { 0, 0 },           { 1, 1 },  // line: start,          end
                                   { 1, 3 }, { 0, 3 },  // quad:        control, end
                                             { 0, 0 },  // line:                 end
    };
    AddLine( fillWinding.winding.contour, { context, &contour2[0], lineSize, lineType } );
    AddQuads(fillWinding.winding.contour, { context, &contour2[1], quadSize, quadType } );
    AddLine( fillWinding.winding.contour, { context, &contour2[3], lineSize, lineType } );

	SetErrorHandler(context, allowDisjointLines);
    auto handleError = [context](WindingCondition result, WindingCondition expected) {
        OpDebugOut("contains example " + STR(result == expected ? "worked as expected" : "failed")
                + "\n");
        if (ContextError::none != Error(context))
            exit(1);
    };
    SetWindingCallbacks(context, { frameAddFunc, frameKeepFunc, frameSubtractFunc, frameWoundFunc,
            frameVisibleFunc, frameZeroFunc, frameIntersectFunc, framePartiallyContainsFunc });
    WindingCondition isOutside = Resolve(context);
    handleError(isOutside, framePartiallyContained);
    SetWindingCallbacks(context, { frameAddFunc, frameKeepFunc, frameSubtractFunc, frameWoundFunc,
            frameVisibleFunc, frameZeroFunc, frameIntersectFunc, nullptr, frameFullyContainsFunc });
    isOutside = Resolve(context);
    handleError(isOutside, 0);  // not fully contained
    DeleteContext(context);
}

OP_TINY_MAIN(ContainsExample)
