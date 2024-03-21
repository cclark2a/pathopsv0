// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpEdge.h"
#include "PathOps.h"

#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"
#endif

static SkPath* skPath(const void* extRef) {
    return (SkPath*) extRef;
}

bool PathSimplify(OpInPath path, OpOutPath result) {
    SkPath empty;
	OpInPath emptyPath(&empty);
    return PathOps(path, emptyPath, OpOperator::Union, result);
}

#if OP_DEBUG
bool DebugPathSimplify(OpInPath path, OpOutPath result, 
		OpDebugExpect expected, std::string testname, std::vector<OpDebugWarning>& warnings) {
    SkPath empty;
	OpInPath emptyPath(&empty);
    return DebugPathOps(path, emptyPath, OpOperator::Union, result, expected, testname, warnings);
}
#endif

bool OpInPath::isInverted() const {
	return skPath(externalReference)->isInverseFillType();
}

void OpOutPath::setEmpty() {
	skPath(externalReference)->reset();
}

void OpOutPath::setInverted(bool wasInverted) {
    SkPathFillType fillType = wasInverted ? SkPathFillType::kInverseEvenOdd : SkPathFillType::kEvenOdd;
    skPath(externalReference)->setFillType(fillType);
}

#if OP_DEBUG_DUMP

bool OpOutPath::debugIsEmpty() const {
    return skPath(externalReference)->isEmpty();
}

std::string OpInPath::debugDump(DebugLevel , DebugBase b) const {
    SkPath* skPath = (SkPath*) externalReference;
    if (DebugBase::dec == b)
        skPath->dump();
    else
        skPath->dumpHex();
    return "";
}

std::string OpOutPath::debugDump(DebugLevel , DebugBase b) const {
    SkPath* skPath = (SkPath*) externalReference;
    if (DebugBase::dec == b)
        skPath->dump();
    else
        skPath->dumpHex();
    return "";
}

#endif

static SkPoint toSkPoint(OpPoint pt) {
    return SkPoint::Make(pt.x, pt.y);
}

// return false to abort output
bool OpCurve::output(OpOutPath& path, bool firstPt, bool lastPt) {
    SkPath* skpath = skPath(path.externalReference);
    if (firstPt)
        skpath->moveTo(toSkPoint(pts[0]));
    switch (type) {
    case OpType::line: 
        skpath->lineTo(toSkPoint(pts[1])); 
        break;
    case OpType::quad: 
        skpath->quadTo(toSkPoint(pts[1]), toSkPoint(pts[2])); 
        break;
    case OpType::conic: 
        skpath->conicTo(toSkPoint(pts[1]), toSkPoint(pts[2]), weight); 
        break;
    case OpType::cubic: 
        skpath->cubicTo(toSkPoint(pts[1]), toSkPoint(pts[2]), toSkPoint(pts[3])); 
        break;
    default:
        OP_ASSERT(0);
        return false;
    }
    if (lastPt)
        skpath->close();
    return true;
}

bool OpContours::build(OpInPath path, OpOperand operand) {
    const SkPath& skpath = *skPath(path.externalReference);
    setFillType(operand, SkPathFillType::kEvenOdd == skpath.getFillType()
            || SkPathFillType::kInverseEvenOdd == skpath.getFillType() 
            ? OpFillType::evenOdd : OpFillType::winding);
    OpContour* head = makeContour(operand);
    SkPath::RawIter iter(skpath);
    SkPath::Verb verb;
    do {
        OpPoint pts[4];
        verb = iter.next((SkPoint*) pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            head = addMove(head, operand, pts);
            break;
        case SkPath::kLine_Verb:
            head->addLine(pts);
            break;
        case SkPath::kQuad_Verb:
            if (!head->addQuad(pts))
                return false;
            break;
        case SkPath::kConic_Verb:
            if (!head->addConic(pts, iter.conicWeight()))
                return false;
            break;
        case SkPath::kCubic_Verb:
            if (!head->addCubic(pts))
                return false;
            break;
        case SkPath::kClose_Verb:
            break;
        case SkPath::kDone_Verb:
            break;
        }
    } while (verb != SkPath::kDone_Verb);
    head->addClose();
    return true;
}
