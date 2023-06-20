#include "OpEdgeBuilder.h"
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

bool OpOutPath::setEmpty() {
	skPath->reset();
	return true;
}

bool OpOutPath::setInverted(bool wasInverted) {
    SkPathFillType fillType = wasInverted ? SkPathFillType::kInverseEvenOdd : SkPathFillType::kEvenOdd;
    skPath->setFillType(fillType);
	return true;
}

#if OP_DEBUG_DUMP

void OpOutPath::dump() const {
    skPath->dump();
}

#endif

void OpPoint::toSkPoint(SkPoint* skPt) const {
    skPt->fX = x;
    skPt->fY = y;
}

void OpEdge::output(OpOutPath path) {
    SkPoint skPt[4];
    whichPtT().pt.toSkPoint(&skPt[0]);
    SkPath* skPath = path.skPath;
    skPath->moveTo(skPt[0]);
    const OpEdge* firstEdge = this;
    OpEdge* edge = this;
    do {
        OpType type = edge->setLinear() ? lineType : edge->segment->c.type;
        start.pt.toSkPoint(&skPt[0]);
        ctrlPts[0].toSkPoint(&skPt[1]);
        ctrlPts[1].toSkPoint(&skPt[2]);
        end.pt.toSkPoint(&skPt[3]);
        if (EdgeMatch::end == edge->whichEnd) {
            std::swap(skPt[0], skPt[3]);
            if (cubicType == type)
                std::swap(skPt[1], skPt[2]);
        }
        switch (type) {
        case pointType: 
            assert(0); 
            break;  // !!!unimplemented
        case lineType: 
            skPath->lineTo(skPt[3]); 
            break;
        case quadType: 
            skPath->quadTo(skPt[1], skPt[3]); 
            break;
        case conicType: 
            skPath->conicTo(skPt[1], skPt[3], edge->weight); 
            break;
        case cubicType: 
            edge->ctrlPts[1].toSkPoint(&skPt[2]);
            skPath->cubicTo(skPt[1], skPt[2], skPt[3]);  break;
            break;
        default: 
            assert(0);
        }
        edge->clearActive();
        edge = edge->nextEdge;
    } while (firstEdge != edge);
    skPath->close();
}

bool OpContours::build(OpInPath path, OpOperand operand) {
    const SkPath& skPath = *path.skPath;
    if (!skPath.isFinite())
        return false;
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
            head = head->addMove(pts);
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
    head->finish();
    return true;
}

#endif
