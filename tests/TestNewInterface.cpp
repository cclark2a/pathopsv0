// (c) 2023, Cary Clark cclark2@gmail.com
#include "PathOps.h"
#include "OpCurve.h"

// curve types
OpType lineType = OpType::no;  // unset
OpType quadType = OpType::no;

// exit() is called as a debugging aid. Once complete, exit() should never be called

#if 0
constexpr size_t lineLength() {
    return sizeof(OpPoint) * 2;
}

constexpr size_t quadLength() {
    return sizeof(OpPoint) * 3;
}
#endif

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

OpCurve lineSubDivide(PathOpsV0Lib::Context* context, PathOpsV0Lib::Curve curve, 
        OpPtT ptT1, OpPtT ptT2) {
    OpContours* contours = (OpContours*) context;
    OpCurve result(contours, curve);
    result.curveData->start = ptT1.pt;
    result.curveData->end = ptT2.pt;
    return result;
}

OpCurve quadSubDivide(PathOpsV0Lib::Context* context, PathOpsV0Lib::Curve curve,
        OpPtT ptT1, OpPtT ptT2) {
    OpContours* contours = (OpContours*) context;
    OpCurve result(contours, curve);
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

// contour types    // !!! probably needs its own type
OpType windingFill = OpType::no;  // unset
OpType evenOddFill = OpType::no;

// operation types    // !!! probably needs its own type
OpType simplifyOperation = OpType::no;  // unset

#include "include/pathops/SkPathOps.h"

static const SkPathOp inverseOpMapping[5][2][2] {
 {{kDifference_SkPathOp, kIntersect_SkPathOp}, {kIntersect_SkPathOp, kReverseDifference_SkPathOp }},
 {{kIntersect_SkPathOp, kDifference_SkPathOp}, {kReverseDifference_SkPathOp, kIntersect_SkPathOp }},
 {{kIntersect_SkPathOp, kReverseDifference_SkPathOp}, {kDifference_SkPathOp, kIntersect_SkPathOp }},
 {{kXOR_SkPathOp, kXOR_SkPathOp }, {kXOR_SkPathOp, kXOR_SkPathOp}},
 {{kReverseDifference_SkPathOp, kIntersect_SkPathOp}, {kIntersect_SkPathOp, kDifference_SkPathOp }},
};

/* Given a PathOps operator, and if the operand fills are inverted, return the equivalent operator
   treating the operands as non-inverted.
 */
SkPathOp MapInvertedSkPathOp(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted) {
    return inverseOpMapping[(int) op][leftOperandIsInverted][rightOperandIsInverted];
}

static const bool outInverse[5][2][2] {
    { { false, false }, { true, false } },  // diff
    { { false, false }, { false, true } },  // sect
    { { false, true }, { true, true } },    // union
    { { false, true }, { true, false } },   // xor
    { { false, true }, { false, false } },  // rev diff
};

/* Given a PathOps operator, and if the operand fills are inverted, return true if the output is
   inverted.
 */
bool SkPathOpInvertOutput(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted) {
    return outInverse[(int) op][leftOperandIsInverted][rightOperandIsInverted];
}

// user extensions to interpret the meaning of winding and operators

/* table of winding states that the op types use to keep an edge
	left op (first path)	right op (second path)		keep if:
			0					0					---
			0					flipOff				union, rdiff, xor
			0					flipOn				union, rdiff, xor
			0					1					---
		    flipOff				0					union, diff, xor
		    flipOff				flipOff				intersect, union
		    flipOff				flipOn				diff, rdiff
		    flipOff				1					intersect, rdiff, xor
		    flipOn				0					union, diff, xor
		    flipOn				flipOff				diff, rdiff
		    flipOn				flipOn				intersect, union
		    flipOn				1					intersect, rdiff, xor
			1					0					---
			1					flipOff				intersect, diff, xor
			1					flipOn				intersect, diff, xor
			1					1					---
*/

enum class BinaryOperators {
    Subtract,
    Intersect,
    Union,
    ExclusiveOr,
    ReverseSubtract
};

struct BinaryWinding {
	int left;
	int right;
};

#if 0 // not ready for prime time
WindKeep ApplyWinding() {
	OpWinding su = sum;
	OpWinding wi = winding;
	OpContours* contours = this->contours();
	WindState left = contours->windState(wi.left(), su.left(), OpOperand::left);
	WindState right = contours->windState(wi.right(), su.right(), OpOperand::right);
	if (left != WindState::flipOff && left != WindState::flipOn
			&& right != WindState::flipOff && right != WindState::flipOn) {
		return WindKeep::Discard;
	}
	bool bothFlipped = (left == WindState::flipOff || left == WindState::flipOn)
			&& (right == WindState::flipOff || right == WindState::flipOn);
	WindKeep keep = WindKeep::Discard;
	switch (contours->opOperator) {
	case BinaryOperators::Subtract:
		if (bothFlipped ? left != right : WindState::one == left || WindState::zero == right)
			keep = su.right() || !su.left() ? WindKeep::End : WindKeep::Start;
		break;
	case BinaryOperators::Intersect:
		if (bothFlipped ? left == right : WindState::zero != left && WindState::zero != right)
			keep = !su.left() || !su.right() ? WindKeep::End : WindKeep::Start;
		break;
	case BinaryOperators::Union:
		if (bothFlipped ? left == right : WindState::one != left && WindState::one != right)
			keep = !su.left() && !su.right() ? WindKeep::End : WindKeep::Start;
		break;
	case BinaryOperators::ExclusiveOr:
		if (!bothFlipped)
			keep = !su.left() == !su.right() ? WindKeep::End : WindKeep::Start;
		break;
	case BinaryOperators::ReverseSubtract:
		if (bothFlipped ? left != right : WindState::zero == left || WindState::one == right)
			keep = su.left() || !su.right() ? WindKeep::End : WindKeep::Start;
		break;
	default:
		OP_ASSERT(0);
	}
	return keep;
}
#endif

void testNewInterface() {
    using namespace PathOpsV0Lib;

    Context* context = CreateContext();

#if OP_DEBUG
    OpDebugData debugData(false);
    debugData.debugCurveCurve1 = 2;
    debugData.debugCurveCurve2 = 7;
    debugData.debugCurveCurveDepth = 6;
    Debug(context, debugData);
#endif

    lineType = SetCurveCallBacks(context, lineAxisRawHit, noHull, lineIsLine, 
            noLinear, noControls, noSetControls, /*lineLength,*/ lineNormal, noReverse,
            lineTangent, linesEqual, linePtAtT, /* double not required */ linePtAtT, 
            linePtCount, lineSubDivide, lineXYAtT
#if OP_DEBUG_IMAGE
		    , debugLineAddToPath
#endif
    );
    quadType = SetCurveCallBacks(context, quadAxisRawHit, quadHull, quadIsLine, 
            quadIsLinear, quadControl, quadSetControl, /*quadLength,*/  quadNormal, noReverse,
            quadTangent, quadsEqual, quadPtAtT, /* double not required */ quadPtAtT, 
            quadPtCount, quadSubDivide, quadXYAtT
#if OP_DEBUG_IMAGE
		    , debugQuadAddToPath
#endif
    );
    simplifyOperation = SetOperationCallBacks(context);   // !!! incomplete

    // example: given points describing a pair of closed loops with quadratic Beziers, find
    //          their intersection
    Contour* contour = CreateContour(context);
    int windingData[] = { 1 };
    AddWinding addWinding { contour, windingData, sizeof(windingData) };
    constexpr size_t lineSize = sizeof(OpPoint) * 2;
    constexpr size_t quadSize = sizeof(OpPoint) * 3;

    // note that the data below omits start points for curves that match the previous end point
                      // start     end      control
    OpPoint contour1[] { { 2, 0 }, { 1, 2 }, { 0, 2 },  // quad: start, end, control
                                   { 1, 2 }, { 2, 3 },  // line: start, end
                                             { 2, 0 },  // line: end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads({ context, &contour1[0], quadSize, quadType }, addWinding );
    Add(     { context, &contour1[3], lineSize, lineType }, addWinding );
    Add(     { context, &contour1[4], lineSize, lineType }, addWinding );

    OpPoint contour2[] { { 0, 0 }, { 1, 1 },            // line: start, end
                                   { 1, 3 }, { 0, 3 },  // quad: end, control
                                   { 1, 3 }, { 0, 0 },  // line: start, end
    };
    Add(     { context, &contour2[0], lineSize, lineType }, addWinding );
    AddQuads({ context, &contour2[1], quadSize, quadType }, addWinding );
    Add(     { context, &contour2[4], lineSize, lineType }, addWinding );

    Resolve(context, simplifyOperation);
    DeleteContext(context);

    if (Error(context)) {
        exit(1);
    }
}

#include "OpContour.h"

namespace PathOpsV0Lib {

Context* CreateContext() {
    OpContours* contours = dumpInit();
    contours->newInterface = true;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
#if OP_DEBUG_IMAGE
    OpDebugImage::init();
    oo();
#endif
    return (Context*) contours;
}

void Add(AddCurve curve, AddWinding windings) {
    OpContours* contours = (OpContours*) curve.context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    OpContour* contour = (OpContour*) windings.contour;
    // Use winding's contour. If none, use most recent. If none, make one.
    if (!contour) {
        OpContourStorage* store = contours->contourStorage;
        if (!store)
            contour = contours->allocateContour();
        else
            contour = &store->storage[store->used - 1];
    }
    contour->segments.emplace_back(curve, windings);
}

ContourID AddContour(Context* context, Winding* winding) {
    // reuse existing contour
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    OpContour* contour = contours->makeContour((OpOperand) 0);
    return contour->id;
}

void DeleteContext(Context* context) {
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
        /*CurveLength lengthFunc, */ CurveNormal normalFunc, CurveReverse reverseFunc, 
        CurveTangent tangentFunc,
        CurvesEqual equalFunc, PtAtT ptAtTFunc, PtAtT doublePtAtTFunc, 
        PtCount ptCountFunc, SubDivide subDivideFunc, XYAtT xyAtTFunc
#if OP_DEBUG_IMAGE
		, DebugAddToPath debugAddToPathFunc
#endif
) {
    OpContours* contours = (OpContours*) context;
    contours->callBacks.push_back( { axisFunc, hullFunc, isLineFunc, isLinearFunc,
        controlsFunc, setControlsFunc, /*lengthFunc,*/ normalFunc, reverseFunc, tangentFunc,
        equalFunc, ptAtTFunc, doublePtAtTFunc, ptCountFunc, subDivideFunc, xyAtTFunc 
#if OP_DEBUG_IMAGE
		, debugAddToPathFunc
#endif
        } );
    return (OpType) contours->callBacks.size();
}

OpType SetOperationCallBacks(Context* ) {
    // !!! incomplete
    return (OpType) 1;
}

#if OP_DEBUG
void Debug(Context* context, OpDebugData& debugData) {
    OpContours* contours = (OpContours*) context;
    contours->dumpCurve1 = debugData.debugCurveCurve1;
    contours->dumpCurve2 = debugData.debugCurveCurve2;
    contours->debugBreakDepth = debugData.debugCurveCurveDepth;
}
#endif


} // namespace PathOpsV0Lib


OpCurve::OpCurve(OpContours* c, PathOpsV0Lib::Curve curve) {
    contours = c;
    size = curve.size;
    curveData = contours->allocateCurveData(size);
    memcpy(curveData, curve.data, size);
    type = curve.type;
    newInterface = true;
}

OpRoots OpCurve::axisRawHit(Axis offset, float intercept, MatchEnds matchEnds) const {
    OP_ASSERT(newInterface);
    return contours->callBack(type).axisRawHitFuncPtr(curveData, offset, intercept, matchEnds);
}

bool OpCurve::isLine() const {
    return !newInterface ? OpType::line == type 
            : contours->callBack(type).curveIsLineFuncPtr(curveData);
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

OpSegment::OpSegment(PathOpsV0Lib::AddCurve addCurve, PathOpsV0Lib::AddWinding addWinding)    
    : c((OpContours*) addCurve.context, 
            { (PathOpsV0Lib::CurveData*) addCurve.points, addCurve.size, addCurve.type } )
    , winding((OpContour*) addWinding.contour, 
            { (PathOpsV0Lib::WindingData*) addWinding.windings, addWinding.size } )
    , disabled(false)  {
    complete((OpContour*) addWinding.contour);  // only for id; could be debug code if not needed
    OP_DEBUG_CODE(contour = nullptr);   // can't set up here because it may still move
    OP_DEBUG_CODE(debugStart = SectReason::test);   // temp for new interface
    OP_DEBUG_CODE(debugEnd = SectReason::test);     //  "
    OP_DEBUG_CODE(debugZero = ZeroReason::uninitialized);
}

void OpPointBounds::set(const OpCurve& c) {
    if (c.newInterface) {
        set(c.curveData->start, c.curveData->end);
        std::vector<OpPoint*> ctrlPts = 
                c.contours->callBack(c.type).curveControlsFuncPtr(c.curveData);
        for (OpPoint* ctrlPt : ctrlPts)
            add(*ctrlPt);
        return;
    }
    set(c.pts, c.pointCount());
}

OpWinding::OpWinding(OpContour* c, PathOpsV0Lib::Winding copy)
: contour(c)
OP_DEBUG_PARAMS(debugType(WindingType::copy)) {
    w.data = contour->contours->allocateWinding(copy.size);
	memcpy(w.data, copy.data, copy.size);
    w.size = copy.size;
}

// returns true if not equal
bool OpWinding::compare(PathOpsV0Lib::Winding comp) const {
    return w.size != comp.size || memcmp(w.data, comp.data, w.size);
}

PathOpsV0Lib::Winding OpWinding::copyData() {
    OpContours* contours = contour->contours;
    PathOpsV0Lib::Winding copy { contours->allocateWinding(w.size), w.size };
    memcpy(copy.data, w.data, w.size);
    return copy;
}

void OpWinding::setWind(OpContour* contour, const OpWinding& fromSegment) {
	contour = fromSegment.contour;
	size_t len = fromSegment.w.size;
	w.data = contour->contours->allocateWinding(len);
	memcpy(w.data, fromSegment.w.data, len);
	w.size = len;

}