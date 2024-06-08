// (c) 2023, Cary Clark cclark2@gmail.com
#include "PathOps.h"
#include "OpCurve.h"

constexpr int lineType = 1;
constexpr int quadType = 2;
constexpr int simplifyOperation = 1;

size_t curveLength(PathOpsV0Lib::Curve c) {
    switch (c.type) {
    case lineType:
        return 4;
    case quadType:
        return 6;
    default:
        exit(2);
    }
    return 0;
}

void testNewInterface() {
    using namespace PathOpsV0Lib;

    Context* context = Create();

    auto setCurveCallbacks = [context](int type) {
        SetCurveLength(context, curveLength);
    };

    setCurveCallbacks(lineType);
    setCurveCallbacks(quadType);

    auto setOperationCallbacks = [/*context*/](int type) {

    };

    setOperationCallbacks(simplifyOperation);

    // example: given points describing a pair of closed loops with quadratic Beziers, find
    //          their intersection

    // note that the data below omits start points for curves that match the previous end point
                      // start     end      control
    Point contour1[] { { 2, 0 }, { 1, 2 }, { 0, 2 },  // quad: start, end, control
                                 { 1, 2 }, { 2, 3 },  // line: start, end
                                           { 2, 0 },  // line: end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads(context, { (CurveData*) &contour1[0], quadType } );
    Add(context, { (CurveData*) &contour1[3], lineType } );
    Add(context, { (CurveData*) &contour1[4], lineType } );

    Point contour2[] { { 0, 0 }, { 1, 1 },            // line: start, end
                                 { 1, 3 }, { 0, 3 },  // quad: end, control
                                 { 1, 3 }, { 0, 0 },  // line: start, end
    };
    Add(context, { (CurveData*) &contour2[0], lineType } );
    AddQuads(context, { (CurveData*) &contour2[1], quadType } );
    Add(context, { (CurveData*) &contour2[4], lineType } );

    Resolve(context, simplifyOperation);
    Delete(context);

    if (Error(context)) {
        exit(1);
    }
}

#include "OpContour.h"

namespace PathOpsV0Lib {

Context* Create() {
    OpContours* contours = dumpInit();
    contours->newInterface = true;
    return (Context*) contours;
}

void Add(Context* context, Curve curve) {
    OpContours* contours = (OpContours*) context;
    if (!contours->contours.size())
        (void) contours->makeContour(OpOperand::left);
    // !!! use current contour instead
    OpContour& contour = contours->contours.back();
    // !!! create variant of curve which has pointer to data
    contour.segments.emplace_back(context, curve, &contour);
}

void Contour(Context* context, TypeID contourID) {
    // reuse existing contour
    OpContours* contours = (OpContours*) context;
    for (OpContour& c : contours->contours)
        if (c.operand == (OpOperand) contourID)
            return;  // !!! set current contour
    contours->makeContour((OpOperand) contourID);
}

void Delete(Context* context) {
    delete (OpContours*) context;
}

int Error(Context* context) {
    // !!! incomplete
    return 0;
}

void Resolve(Context* context, TypeID operation) {
    OpOutPath result(nullptr);  // missing external reference for now is new interface flag
    OpContours* contours = (OpContours*) context;
    contours->opIn = (OpOperator) operation;
    // !!! change this to record error instead of success
    /* bool success = */ contours->pathOps(result);
}

void SetCurveLength(Context* context, CurveLength curveLengthFuncPtr) {
    OpContours* contours = (OpContours*) context;
    contours->curveLengthFuncPtr = curveLengthFuncPtr;
}


} // namespace PathOpsV0Lib


OpPoint::OpPoint(struct PathOpsV0Lib::Point pt) {
    x = pt.x;
    y = pt.y;
}

OpCurve::OpCurve(struct PathOpsV0Lib::Context* context, struct PathOpsV0Lib::Curve curve) {
    OpContours* contours = (OpContours*) context;
    size_t length = (*contours->curveLengthFuncPtr)(curve);
    curveData = contours->allocateCurveData(length);
    memcpy(curveData, curve.data, length);
    type = (OpType) curve.type;
    newInterface = true;
}

OpSegment::OpSegment(struct PathOpsV0Lib::Context* context, struct PathOpsV0Lib::Curve curve,
        OpContour* contourPtr)    
    : c(context, curve)
    , winding(WindingUninitialized::dummy)
    , disabled(false)  {
    complete(contourPtr);  // only for id, which could be debug code if id is not needed
    OP_DEBUG_CODE(contour = nullptr);   // can't set up here because it may still move
    OP_DEBUG_CODE(debugStart = SectReason::test);   // temp for new interface
    OP_DEBUG_CODE(debugEnd = SectReason::test);     //  "
    OP_DEBUG_CODE(debugZero = ZeroReason::uninitialized);
}

struct PathOpsV0Lib::Point OpPoint::v0Pt() const {
    return *((struct PathOpsV0Lib::Point*) this);
}
