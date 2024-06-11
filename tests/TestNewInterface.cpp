// (c) 2023, Cary Clark cclark2@gmail.com
#include "PathOps.h"
#include "OpCurve.h"

OpType lineType = OpType::no;  // unset
OpType quadType = OpType::no;

int simplifyOperation = -1;  // unset  !!! replace int with type

// exit() is called as a debugging aid. Once complete, exit() should never be called

size_t lineLength() {
    return sizeof(OpPoint) * 2;
}

size_t quadLength() {
    return sizeof(OpPoint) * 3;
}

size_t linePtCount() {
    return 2;
}

size_t quadPtCount() {
    return 3;
}

bool lineIsLine(const PathOpsV0Lib::CurveData* ) {
    return true;
}

bool quadIsLine(const PathOpsV0Lib::CurveData* ) {
    return false;
}

OpRoots lineAxisRawHit(PathOpsV0Lib::CurveData* c, Axis axis, float axisIntercept, MatchEnds ends) {
    const float* ptr = c->start.asPtr(axis);
    if (!OpMath::Between(ptr[0], axisIntercept, ptr[2]))
        return OpRoots(axisIntercept < ptr[0] ? RootFail::outsideFirstPt : RootFail::outsideLastPt);
    // strict equality fails for denomalized numbers
    // if (ptr[2] == ptr[0]) {
    if (fabsf(ptr[2] - ptr[0]) <= OpEpsilon)   // coincident line values are computed later
        return OpRoots(OpNaN, OpNaN);
    return OpRoots((axisIntercept - ptr[0]) / (ptr[2] - ptr[0]));
}

OpRoots quadAxisRawHit(PathOpsV0Lib::CurveData* c, Axis axis, float axisIntercept, MatchEnds ends) {
    OpPoint controlPt = *(OpPoint*) c->optionalAdditionalData;
    return PathOpsV0Lib::QuadAxisRawHit(c->start, controlPt, c->end, axis, axisIntercept);
}

OpPoint linePtAtT(PathOpsV0Lib::CurveData* c, float t) {
    if (0 == t)
        return c->start;
    if (1 == t)
        return c->end;
    return (1 - t) * c->start + t * c->end;
    return OpPoint();
}

OpPoint quadPtAtT(PathOpsV0Lib::CurveData* c, float t) {
    OpPoint controlPt = *(OpPoint*) c->optionalAdditionalData;
    return PathOpsV0Lib::QuadPointAtT(c->start, controlPt, c->end, t);
}

OpPair lineXYAtT(PathOpsV0Lib::CurveData* c, OpPair t, XyChoice xyChoice) {
    return (1 - t) * c->start.choice(xyChoice) + t * c->end.choice(xyChoice);
}

OpPair quadXYAtT(PathOpsV0Lib::CurveData* c, OpPair t, XyChoice xyChoice) {
    OpPoint controlPt = *(OpPoint*) c->optionalAdditionalData;
    return PathOpsV0Lib::QuadXYAtT(c->start, controlPt, c->end, t, xyChoice);
}

bool linesEqual(PathOpsV0Lib::CurveData* one, PathOpsV0Lib::Curve two) {
    if (lineType != two.type)
        exit(4);
    return true;
}

bool quadsEqual(PathOpsV0Lib::CurveData* one, PathOpsV0Lib::Curve two) {
    if (quadType != two.type)
        exit(5);
    OpPoint* ctrlPt1 = (OpPoint*) one->optionalAdditionalData;
    OpPoint* ctrlPt2 = (OpPoint*) two.data->optionalAdditionalData;
    return ctrlPt1->x == ctrlPt2->x && ctrlPt1->y == ctrlPt2->y;
}

OpVector lineTangent(const PathOpsV0Lib::CurveData* c, float ) {
    return c->end - c->start;
}

OpVector quadTangent(const PathOpsV0Lib::CurveData* c, float t) {
    OpPoint controlPt = *(OpPoint*) c->optionalAdditionalData;
    return PathOpsV0Lib::QuadTangent(c->start, controlPt, c->end, t);
}

OpVector lineNormal(const PathOpsV0Lib::CurveData* c, float ) {
    return { c->start.y - c->end.y, c->end.x - c->start.x };
}

OpVector quadNormal(const PathOpsV0Lib::CurveData* c, float t) {
    OpVector tan = quadTangent(c, t);
    return { -tan.dy, tan.dx };
}

OpCurve lineSubDivide(PathOpsV0Lib::Context* context, const PathOpsV0Lib::Curve curve, 
        OpPtT ptT1, OpPtT ptT2) {
    OpCurve result(context, curve);
    result.curveData->start = ptT1.pt;
    result.curveData->end = ptT2.pt;
    return result;
}

OpCurve quadSubDivide(PathOpsV0Lib::Context* context, const PathOpsV0Lib::Curve curve,
        OpPtT ptT1, OpPtT ptT2) {
    OpCurve result(context, curve);
    result.curveData->start = ptT1.pt;
    result.curveData->end = ptT2.pt;
    OpPoint controlPt = *(OpPoint*) curve.data->optionalAdditionalData;
    OpPoint subControl = PathOpsV0Lib::QuadControlPt(curve.data->start, controlPt, curve.data->end,
            ptT1, ptT2);
    *(OpPoint*) result.curveData->optionalAdditionalData = subControl;
    return result;
}

OpPoint noHull(const PathOpsV0Lib::CurveData* c, int index) {
    exit(5); // should never be called
    return OpPoint();
}

OpPoint quadHull(const PathOpsV0Lib::CurveData* c, int index) {
    if (1 == index)
        return *(OpPoint*) c->optionalAdditionalData;
    exit(6); // should never be called
    return OpPoint();
}

bool noLinear(const PathOpsV0Lib::CurveData* c) {
    exit(7); // should never be called
    return false;
}

bool quadIsLinear(const PathOpsV0Lib::CurveData* c) {
    OpPoint controlPt = *(OpPoint*) c->optionalAdditionalData;
    OpVector diffs[] { controlPt - c->start, c->end - c->start };
    float cross = diffs[0].cross(diffs[1]);
    return fabsf(cross) <= OpEpsilon;
}

void noReverse(const PathOpsV0Lib::CurveData* ) {
}

std::vector<OpPoint*> noControls(const PathOpsV0Lib::CurveData* ) {
    return std::vector<OpPoint*>();
}

std::vector<OpPoint*> quadControl(const PathOpsV0Lib::CurveData* c) {
    return std::vector<OpPoint*>(1, (OpPoint*) c->optionalAdditionalData); 
}

void noSetControls(const PathOpsV0Lib::CurveData* , std::vector<OpPoint>& ) {
}

void quadSetControl(const PathOpsV0Lib::CurveData* c, std::vector<OpPoint>& pts) {
    *(OpPoint*) c->optionalAdditionalData = pts[0];
}

#if OP_DEBUG_IMAGE
#include "include/core/SkPath.h"

void debugLineAddToPath(const PathOpsV0Lib::CurveData* c, SkPath& path) {
	path.lineTo(c->end.x, c->end.y);
}

void debugQuadAddToPath(const PathOpsV0Lib::CurveData* c, SkPath& path) {
    OpPoint controlPt = *(OpPoint*) c->optionalAdditionalData;
	path.quadTo(controlPt.x, controlPt.y, c->end.x, c->end.y);
}
#endif

void testNewInterface() {
    using namespace PathOpsV0Lib;

    Context* context = Create();
    lineType = SetCurveCallBacks(context, lineAxisRawHit, noHull, lineIsLine, 
            noLinear, noControls, noSetControls, lineLength, lineNormal, noReverse,
            lineTangent, linesEqual, linePtAtT, /* double not required */ linePtAtT, 
            linePtCount, lineSubDivide, lineXYAtT
#if OP_DEBUG_IMAGE
		    , debugLineAddToPath
#endif
    );
    quadType = SetCurveCallBacks(context, quadAxisRawHit, quadHull, quadIsLine, 
            quadIsLinear, quadControl, quadSetControl, quadLength,  quadNormal, noReverse,
            quadTangent, quadsEqual, quadPtAtT, /* double not required */ quadPtAtT, 
            quadPtCount, quadSubDivide, quadXYAtT
#if OP_DEBUG_IMAGE
		    , debugQuadAddToPath
#endif
    );
    simplifyOperation = SetOperationCallBacks(context);   // !!! incomplete

    // example: given points describing a pair of closed loops with quadratic Beziers, find
    //          their intersection

    // note that the data below omits start points for curves that match the previous end point
                      // start     end      control
    OpPoint contour1[] { { 2, 0 }, { 1, 2 }, { 0, 2 },  // quad: start, end, control
                                   { 1, 2 }, { 2, 3 },  // line: start, end
                                             { 2, 0 },  // line: end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads(context, { (CurveData*) &contour1[0], (OpType) quadType } );
    Add(context, { (CurveData*) &contour1[3], (OpType) lineType } );
    Add(context, { (CurveData*) &contour1[4], (OpType) lineType } );

    OpPoint contour2[] { { 0, 0 }, { 1, 1 },            // line: start, end
                                   { 1, 3 }, { 0, 3 },  // quad: end, control
                                   { 1, 3 }, { 0, 0 },  // line: start, end
    };
    Add(context, { (CurveData*) &contour2[0], (OpType) lineType } );
    AddQuads(context, { (CurveData*) &contour2[1], (OpType) quadType } );
    Add(context, { (CurveData*) &contour2[4], (OpType) lineType } );

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
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    return (Context*) contours;
}

void Add(Context* context, Curve curve) {
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    if (!contours->contours.size())
        (void) contours->makeContour(OpOperand::left);
    // !!! use current contour instead
    OpContour& contour = contours->contours.back();
    // !!! create variant of curve which has pointer to data
    contour.segments.emplace_back(context, curve, &contour);
}

void Contour(Context* context, OpType contourID) {
    // reuse existing contour
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    for (OpContour& c : contours->contours)
        if (c.operand == (OpOperand) contourID)
            return;  // !!! set current contour
    contours->makeContour((OpOperand) contourID);
}

void Delete(Context* context) {
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    delete contours;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = nullptr;
#endif
}

int Error(Context* context) {
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    // !!! incomplete
    return 0;
}

void Resolve(Context* context, int operation) {
    OpOutPath result(nullptr);  // missing external reference for now is new interface flag
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    contours->opIn = (OpOperator) operation;
    // !!! change this to record error instead of success
    /* bool success = */ contours->pathOps(result);
}

OpType SetCurveCallBacks(Context* context, AxisRawHit axisFunc, CurveHull hullFunc,
        CurveIsLine isLineFunc, CurveIsLinear isLinearFunc, CurveControls controlsFunc,
        SetControls setControlsFunc,
        CurveLength lengthFunc, CurveNormal normalFunc, CurveReverse reverseFunc, 
        CurveTangent tangentFunc,
        CurvesEqual equalFunc, PtAtT ptAtTFunc, PtAtT doublePtAtTFunc, 
        PtCount ptCountFunc, SubDivide subDivideFunc, XYAtT xyAtTFunc
#if OP_DEBUG_IMAGE
		, DebugAddToPath debugAddToPathFunc
#endif
) {
    OpContours* contours = (OpContours*) context;
    contours->callBacks.push_back( { axisFunc, hullFunc, isLineFunc, isLinearFunc,
        controlsFunc, setControlsFunc, lengthFunc, normalFunc, reverseFunc, tangentFunc,
        equalFunc, ptAtTFunc, doublePtAtTFunc, ptCountFunc, subDivideFunc, xyAtTFunc 
#if OP_DEBUG_IMAGE
		, debugAddToPathFunc
#endif
        } );
    return (OpType) contours->callBacks.size();
}

int SetOperationCallBacks(Context* ) {
    // !!! incomplete
    return 1;
}

} // namespace PathOpsV0Lib


OpCurve::OpCurve(struct PathOpsV0Lib::Context* context, struct PathOpsV0Lib::Curve curve) {
    contours = (OpContours*) context;
    size_t length = (*contours->callBack(curve).curveLengthFuncPtr)();
    curveData = contours->allocateCurveData(length);
    memcpy(curveData, curve.data, length);
    type = curve.type;
    newInterface = true;
}

OpRoots OpCurve::axisRawHit(Axis offset, float intercept, MatchEnds matchEnds) const {
    OP_ASSERT(newInterface);
    return (*contours->callBack(type).axisRawHitFuncPtr)(curveData, offset, intercept, matchEnds);
}

bool OpCurve::isLine() const {
    return !newInterface ? OpType::line == type 
            : (*contours->callBack(type).curveIsLineFuncPtr)(curveData);
}

OpPoint OpCurve::firstPt() const {
    return newInterface ? curveData->start : pts[0]; 
} 

OpPoint OpCurve::lastPt() const {
    return newInterface ? curveData->end : pts[pointCount() - 1]; 
} 

void OpCurve::setFirstPt(OpPoint pt) {
    (newInterface ? curveData->start : pts[0]) = pt;
}

void OpCurve::setLastPt(OpPoint pt) {
    (newInterface ? curveData->end : pts[pointCount() - 1]) = pt;
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

void OpPointBounds::set(const OpCurve& c) {
    if (c.newInterface)
        return set(c.curveData->start, c.curveData->end);
    set(c.pts, c.pointCount());
}

