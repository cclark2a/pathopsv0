// (c) 2025, Cary Clark cclark2@gmail.com

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

WindKeep cutOutput(Output o) {
    const Curve& c = o.curve;
    ContextUserData data = UserData(c.context, UserDataType::outData);
    TestData& test = *(TestData*) data.data;
    test.outStr += line == c.type ? "line: " : "quad: ";
    auto addPtStr = [&test](OpPoint pt, std::string delimiter) {
        test.outStr += pt.toString() + delimiter;
    };
    addPtStr(c.data->start, ", ");
    if (quad == c.type)
        addPtStr(quadControlPt(c), ", ");
    addPtStr(c.data->end, "\n");
    return WindKeep::Discard;
}

static bool allowTreeFail(ContextError err, Curve* ) {
	return ContextError::disjoint == err || ContextError::tree == err;
}

void TestCut() {
    TestData testData(CutDirection::clockwise);
    ContextUserData userData { &testData, sizeof(testData), UserDataType::outData };
    Context* context = cutContext(userData, cutOutput);
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

	SetErrorHandler(context, allowTreeFail);
    cutCallbacks(context);
    Resolve(context);
    // No output is generated, no error is set. To detect frame pieces, use a different output func.
    OP_ASSERT(ContextError::none == Error(context));
    OP_ASSERT(testData.outStr.empty());
    testData.cutData = CutData(CutDirection::counterclockwise);
    Resolve(context);
    OP_ASSERT(ContextError::none == Error(context));
    OP_ASSERT(testData.outStr.empty());

    // added line completes cut; resolve should output left half, right half; no error
    OpPoint connectingPts[] { linePts[1], quadPts[0] };
    AddLine(cutWinding.winding.contour, { context, connectingPts, lineSize, line } );
 //   cutCallbacks(context);
    testData.cutData = CutData(CutDirection::clockwise);
    Resolve(context);
    OP_ASSERT(ContextError::none == Error(context));
    OP_ASSERT(std::string::npos != testData.outStr.find("{ 15.000000, 45.000000 }")); 
    OP_ASSERT(std::string::npos == testData.outStr.find("{ 45.000000, 15.000000 }")); 
    testData.cutData = CutData(CutDirection::counterclockwise);
    testData.outStr.clear();
    Resolve(context);
    OP_ASSERT(ContextError::none == Error(context));
    OP_ASSERT(std::string::npos == testData.outStr.find("{ 15.000000, 45.000000 }")); 
    OP_ASSERT(std::string::npos != testData.outStr.find("{ 45.000000, 15.000000 }")); 

    DeleteContext(context);
}

OP_TINY_MAIN(TestCut)  // main() for cmake
