#include "OpContour.h"
#include "OpEdge.h"
#include "PathOps.h"

#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA

#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"

bool PathSimplify(OpInPath path, OpOutPath result) {
    SkPath empty;
	OpInPath emptyPath(&empty);
    return PathOps(path, emptyPath, OpOperator::Union, result);
}

bool OpInPath::isInverted() const {
	return skPath->isInverseFillType();
}

void OpOutPath::setEmpty() {
	skPath->reset();
}

void OpOutPath::setInverted(bool wasInverted) {
    SkPathFillType fillType = wasInverted ? SkPathFillType::kInverseEvenOdd : SkPathFillType::kEvenOdd;
    skPath->setFillType(fillType);
}

#if OP_DEBUG_DUMP

bool OpOutPath::debugIsEmpty() const {
    return skPath->isEmpty();
}

void dmp(const OpOutPath& outPath)  {
    outPath.skPath->dump();
}

#endif

SkPoint OpPoint::toSkPoint() const {
    return SkPoint::Make(x, y);
}

void OpEdge::output(OpOutPath path) {
    SkPath* skPath = path.skPath;
    skPath->moveTo(whichPtT().pt.toSkPoint());
    const OpEdge* firstEdge = this;
    OpEdge* edge = this;
    do {
        SkPoint skEndPt = edge->whichPtT(EdgeMatch::end).pt.toSkPoint();
        OpType type = edge->type();
        if (OpType::line == type) 
            skPath->lineTo(skEndPt); 
        else {
            SkPoint skCtrlPt0 = edge->ctrlPts[0].toSkPoint();
            if (OpType::quad == type) 
                skPath->quadTo(skCtrlPt0, skEndPt); 
            else if (OpType::conic == type) 
                skPath->conicTo(skCtrlPt0, skEndPt, edge->weight);
            else {
                OP_ASSERT(OpType::cubic == type);
                SkPoint skCtrlPt1 = edge->ctrlPts[1].toSkPoint();
                if (EdgeMatch::end == edge->whichEnd)
                    std::swap(skCtrlPt0, skCtrlPt1);
                skPath->cubicTo(skCtrlPt0, skCtrlPt1, skEndPt);
            }
        }
        edge = edge->nextOut();
    } while (firstEdge != edge);
    skPath->close();
}

bool OpContours::build(OpInPath path, OpOperand operand) {
    const SkPath& skPath = *path.skPath;
    if (!skPath.isFinite())
        return debugFail();
    setFillType(operand, SkPathFillType::kEvenOdd == skPath.getFillType()
            || SkPathFillType::kInverseEvenOdd == skPath.getFillType() 
            ? OpFillType::evenOdd : OpFillType::winding);
    OpContour* head = makeContour(operand);
    SkPath::RawIter iter(skPath);
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
            head->addQuad(pts);
            break;
        case SkPath::kConic_Verb:
            head->addConic(pts, iter.conicWeight());
            break;
        case SkPath::kCubic_Verb:
            head->addCubic(pts);
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

#endif
