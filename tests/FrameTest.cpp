// (c) 2024, Cary Clark cclark2@gmail.com

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/NoCurve.h"

using namespace PathOpsV0Lib;

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
Winding frameAddFunc(Winding winding, Winding toAdd) {
	FrameWinding sum(winding);
	if (FrameFill::fill == sum.isFrame) {
		FrameWinding addend(toAdd);
		if (FrameFill::fill == addend.isFrame) {
			sum.left += addend.left;
			sum.copyTo(winding);
		}
	}
    return winding;
}

// both winding and sumWinding come from the same edge
WindKeep frameKeepFunc(Winding winding, Winding sumWinding) {
	FrameWinding wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameWinding sum(sumWinding);
	return sum.left ? WindKeep::Start : WindKeep::Discard;
}

// winding is always frame; toAdd comes from another edge, and may be frame or fill
Winding frameSubtractFunc(Winding winding, Winding toSubtract) {
	FrameWinding difference(winding);
	if (FrameFill::fill == difference.isFrame) {	
		FrameWinding subtrahend(toSubtract);
		if (FrameFill::fill == subtrahend.isFrame) {	
			difference.left -= subtrahend.left;
			difference.copyTo(winding);
		}
	}
    return winding;
}

// curve types
CurveType frameLine = (CurveType) 0;  // unset
constexpr size_t frameLineSize = sizeof(OpPoint) * 2;
CurveType frameQuad = (CurveType) 0;  // unset
constexpr size_t frameQuadSize = sizeof(OpPoint) * 3;

void frameOutput(Curve c, bool firstPt, bool lastPt, PathOutput output) {
    std::string outStr = frameLine == c.type ? "line: " : "quad: ";
    auto addPtStr = [&outStr](const OpPoint& pt, std::string delimiter) {
        outStr += "{ " + std::to_string(pt.x) + ", " + std::to_string(pt.y) + " }" + delimiter;
    };
    addPtStr(c.data->start, ", ");
	if (frameQuad == c.type)
        addPtStr(quadControlPt(c), ", ");
    addPtStr(c.data->end, "\n");
    OpDebugOut(outStr);
}

bool frameVisibleFunc(Winding winding) {
    FrameWinding test(winding);
    return !!test.left;
}

void frameZeroFunc(Winding toZero) {
    FrameWinding zero(FrameFill::fill, 0);
    zero.copyTo(toZero);
}

#if OP_DEBUG_DUMP
inline void frameDumpInFunc(const char*& str, Winding winding) {
    FrameWinding frameWinding(winding);
    OpDebugRequired(str, "{");
    frameWinding.left = OpDebugReadSizeT(str);
	frameWinding.isFrame = (FrameFill) OpDebugReadSizeT(str);
    OpDebugRequired(str, "}");
    frameWinding.copyTo(winding);
}

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

Curve frameMakeLine(Curve c) {
    c.type = frameLine;
    c.size = frameLineSize;
    return c;
}

CurveType frameSetLineType(Curve ) {
    return frameLine;
}

bool allowDisjointLines(ContextError err, Context* , Contour* , Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

void testFrame() {
    using namespace PathOpsV0Lib;

    Context* context = CreateContext({nullptr, 0});
    SetContextCallBacks(context, noEmptyPath, frameMakeLine, frameSetLineType, maxSignSwap,
			maxDepth, maxSplits, maxLimbs);

#if OP_DEBUG
    OpDebugData debugData(false);
    Debug(context, debugData);
#endif

    frameLine = SetCurveCallBacks(context, lineAxisT, 
			nullptr, nullptr, nullptr, 
			nullptr, frameOutput, nullptr, 
			nullptr, lineTangent, nullptr, linePtAtT, 
            nullptr, nullptr, nullptr, lineXYAtT,
			lineCut, lineNormalLimit, lineInterceptLimit
    );
    frameQuad = SetCurveCallBacks(context, quadAxisT, quadHull, 
			quadIsFinite, quadIsLine, quadSetBounds, frameOutput, quadPinCtrl, 
            nullptr, quadTangent, quadsEqual, quadPtAtT, quadHullPtCount, quadRotate, 
			quadSubDivide, quadXYAtT, lineCut, lineNormalLimit, lineInterceptLimit
    );

	FrameFill frameContourData = FrameFill::frame;
    Contour* frameContour = CreateContour({context, &frameContourData, sizeof(frameContourData)});
    SetWindingCallBacks(frameContour, frameAddFunc, frameKeepFunc, 
            frameSubtractFunc, frameVisibleFunc, frameZeroFunc 
    		OP_DEBUG_PARAMS(noDebugBitOper)
            OP_DEBUG_DUMP_PARAMS(frameDumpInFunc, frameDumpOutFunc, noDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(noWindingImageOutFunc, noNativePathFunc,
                    noDebugGetDrawFunc, noDebugSetDrawFunc, noIsOppFunc)
	);
    FrameWinding frameData(FrameFill::frame, 1);
    AddWinding frameAddWinding { frameContour, &frameData, sizeof(frameData) };

	FrameFill fillContourData = FrameFill::fill;
    Contour* fillContour = CreateContour({context, &fillContourData, sizeof(fillContourData)});
    SetWindingCallBacks(fillContour, frameAddFunc, frameKeepFunc, 
            frameSubtractFunc, frameVisibleFunc, frameZeroFunc 
    		OP_DEBUG_PARAMS(noDebugBitOper)
	        OP_DEBUG_DUMP_PARAMS(frameDumpInFunc, frameDumpOutFunc, noDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(noWindingImageOutFunc, noNativePathFunc,
                    noDebugGetDrawFunc, noDebugSetDrawFunc, noIsOppFunc)
	);
    FrameWinding fillData(FrameFill::fill, 1);
    AddWinding fillAddWinding { fillContour, &fillData, sizeof(fillData) };

	// example: return line parts in hourglass fill
#if 0
    OpPoint line[] { { 2, 0 }, { 0, 2 } };
    Add({ line, frameLineSize, frameLine }, frameAddWinding );
    OpPoint hourglass[] { { 0.5, 1 }, { 2.5, 1 }, { 1.5, 0 }, { 1.5, 2 }, { 0.5, 1 } };
	for (int index = 0; index < 4; ++index)
		Add({ &hourglass[index], frameLineSize, frameLine }, fillAddWinding );
#else
	OpPoint line[] { { 10, 10 }, { 20, 20 } };
	OpPoint quad[] { { 30, 30 }, { 50, 50 }, { 40, 30 } };
    Add({ line, frameLineSize, frameLine }, frameAddWinding );
    Add({ quad, frameQuadSize, frameQuad }, frameAddWinding );
    OpPoint rect[] { { 15, 15 }, { 45, 15 }, { 45, 45 }, { 15, 45 }, { 15, 15 } };
	for (int index = 0; index < 4; ++index)
		Add({ &rect[index], frameLineSize, frameLine }, fillAddWinding );
#endif

	SetErrorHandler(context, allowDisjointLines);
	Normalize(context);
    Resolve(context, nullptr);
	ContextError error = Error(context);
    DeleteContext(context);
    if (ContextError::none != error)
        exit(1);
}

#if OP_DEBUG && OP_TINY_TEST
bool OpDebugSkipBreak() {
	return true;
}
#endif

