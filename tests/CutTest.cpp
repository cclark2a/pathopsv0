// (c) 2024, Cary Clark cclark2@gmail.com

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/CutWinding.h"

using namespace PathOpsV0Lib;

// curve types
constexpr CurveType line = 1;
constexpr size_t lineSize = sizeof(OpPoint) * 2;
constexpr CurveType quad = 2;
constexpr size_t quadSize = sizeof(OpPoint) * 3;

// A cut path operation computes the cut frame inside the fill, and half the fill; either the
// clockwise or counterclockwise fill that connnects to the frame inside the fill.
// Cut instructs the engine to provide curve loops in multiple passes so that it can select
// the loop that meets the desired criteria.
// On the first pass, Cut output returns OutputKeep::keep and accumulates until lastPt is true.
// Cut output tracks if the accumulated loop is in the desired direction.
// On the second pass, Cut out returns OutputKeep::discard for filled curves, as well as framed
// curves if the desired direction was seen in the first pass.
// If the desired direction was not seen, the third pass will generate a loop with the same
// kept frame edges but different connecting filled edges. Cut output returns OutputKeep::discard
// for all edges to remove them from the active curve pool.

struct TestData {
    TestData(CutDirection dir)
        : cutData(dir) {
    }

    CutData cutData;
    std::string outStr;
};

WindKeep cutOutput(Curve c, Winding w, bool firstPt, bool lastPt) {
    TestData& test = *(TestData*) UserData(c.context);
    CutData& cut = test.cutData;
	FrameData wind(w);
    switch (cut.pass) {
        case CutPass::outputLoop: {
            test.outStr = line == c.type ? "line: " : "quad: ";
            auto addPtStr = [&test](OpPoint pt, std::string delimiter) {
                test.outStr += pt.toString() + delimiter;
            };
            addPtStr(c.data->start, ", ");
	        if (quad == c.type)
                addPtStr(quadControlPt(c), ", ");
            addPtStr(c.data->end, "\n");
            } return WindKeep::Discard;
        case CutPass::checkDirection: {
            if (firstPt) {
                cut.corner[1] = OpPoint(SetToNaN::dummy);
                cut.firstPt = c.data->start;
            } else {
                OpPoint least = cut.corner[1];
                if (!least.isFinite() || c.data->start.x < least.x || 
                        (c.data->start.x == least.x && c.data->start.y < least.y)) {
                    cut.corner = { cut.priorPt, c.data->start, c.data->end };
                }
            }
            cut.priorPt = c.data->start;
            cut.sawFrame |= FrameFill::frame == wind.isFrame;
            WindKeep keep = FrameFill::frame == wind.isFrame ? WindKeep::Start : WindKeep::End;
            if (!lastPt)
                return keep;
            if (!cut.sawFrame) {
                cut.pass = CutPass::discardFill;
                return keep;
            }
            if (cut.firstPt != c.data->end)
                cut.pass = CutPass::discardFrame;
            OpVector v0 = cut.corner[1] - cut.corner[0];
            OpVector v1 = cut.corner[1] - cut.corner[2];
            v0.normalize();
            v1.normalize();
            float cross = v0.cross(v1);
            if ((cross > 0) == (CutDirection::clockwise == cut.direction))
                cut.pass = CutPass::outputLoop;
            else
                cut.pass = CutPass::discardFill;
            return keep;
            } 
        case CutPass::discardFill:
            return FrameFill::fill == wind.isFrame ? WindKeep::Discard : WindKeep::Start;
        case CutPass::discardFrame:
            return FrameFill::frame == wind.isFrame ? WindKeep::Discard : WindKeep::End;
        default:
            OP_ASSERT(0);
    }
    return WindKeep::Discard;
}

static bool allowDisjointLines(ContextError err, Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

void TestCut() {
    TestData testData(CutDirection::clockwise);
    Context* context = cutContext((ContextUserData*) &testData, cutOutput);
    lineCallbacks(context, line);
    quadCallbacks(context, quad);

    // filled rectangle
    OpPoint rect[] { { 15, 15 }, { 45, 15 }, { 45, 45 }, { 15, 45 }, { 15, 15 } };
    FrameWinding fillWinding(context, FrameFill::fill);
	for (int index = 0; index < 4; ++index)
		AddLine(fillWinding.winding.contour, { context, &rect[index], lineSize, line } );

    FrameWinding cutWinding(context, FrameFill::frame);
    // cut curves do not completely cross rectangle
	OpPoint linePts[] { { 10, 10 }, { 20, 20 } };
	OpPoint quadPts[] { { 30, 30 }, { 40, 30 }, { 50, 50 } };
    AddLine(cutWinding.winding.contour, { context, linePts, lineSize, line } );
    AddQuads(cutWinding.winding.contour, { context, quadPts, quadSize, quad } );

    // no output is generated, error is set
	SetErrorHandler(context, allowDisjointLines);
    cutCallbacks(context);
    Resolve(context);
    OP_ASSERT(ContextError::missing == Error(context));
    OP_ASSERT(testData.outStr.empty());
    testData.cutData = CutData(CutDirection::counterclockwise);
    Resolve(context);
    OP_ASSERT(ContextError::missing == Error(context));
    OP_ASSERT(testData.outStr.empty());

    // added line completes cut; resolve should outputs left half, right half; no error
    OpPoint connectingPts[] { linePts[1], quadPts[0] };
    AddLine(cutWinding.winding.contour, { context, connectingPts, lineSize, line } );
    cutCallbacks(context);
    testData.cutData = CutData(CutDirection::clockwise);
    Resolve(context);
    OP_ASSERT(ContextError::none == Error(context));
    OP_ASSERT(std::string::npos != testData.outStr.find("{ 15, 45 }")); 
    OP_ASSERT(std::string::npos == testData.outStr.find("{ 45, 15 }")); 
    testData.cutData = CutData(CutDirection::counterclockwise);
    OP_ASSERT(ContextError::none == Error(context));
    OP_ASSERT(std::string::npos == testData.outStr.find("{ 15, 45 }")); 
    OP_ASSERT(std::string::npos != testData.outStr.find("{ 45, 15 }")); 

    DeleteContext(context);
}

OP_TINY_MAIN(TestCut)  // main() for cmake
