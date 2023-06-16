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

void OpEdgeBuilder::Output(OpEdge* edge, OpOutPath path) {
    SkPoint skPt[4];
    edge->whichPtT().pt.toSkPoint(&skPt[0]);
    SkPath* skPath = path.skPath;
    skPath->moveTo(skPt[0]);
    const OpEdge* firstEdge = edge;
    do {
        OpType type = edge->setLinear() ? lineType : edge->segment->c.type;
        edge->start.pt.toSkPoint(&skPt[0]);
        edge->ctrlPts[0].toSkPoint(&skPt[1]);
        edge->ctrlPts[1].toSkPoint(&skPt[2]);
        edge->end.pt.toSkPoint(&skPt[3]);
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

bool OpSegmentBuilder::Build(OpInPath path, OpContours& c, OpOperand operand) {
    const SkPath& skPath = *path.skPath;
    if (!skPath.isFinite())
        return false;
    OpFillType exor = SkPathFillType::kEvenOdd == skPath.getFillType()
            || SkPathFillType::kInverseEvenOdd == skPath.getFillType() 
            ? OpFillType::evenOdd : OpFillType::winding;
    (OpOperand::left == operand ? c.left : c.right) = exor;
    OpContour* head = nullptr;
    SkPath::RawIter iter(skPath);
    OpPoint curveStart;
    SkPath::Verb verb;
    OpPoint lastPoint;
    bool hasLastPoint = false;
    do {
        OpPoint pts[4];
        verb = iter.next((SkPoint*) pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            // !!! if frame paths are supported, don't add close unless fill is set
            if (hasLastPoint && lastPoint != curveStart) {
                head->addClose(lastPoint, curveStart);
                hasLastPoint = false;
            }
            c.contours.emplace_back(&c, operand);
            head = &c.contours.back();
            curveStart = pts[0];
            head->addMove(&pts[0]);
            continue;
        case SkPath::kLine_Verb:
            head->addLine(pts);
            lastPoint = pts[1];
            hasLastPoint = true;
            break;
        case SkPath::kQuad_Verb:
            head->addQuad(pts);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kConic_Verb:
            head->addConic(pts, iter.conicWeight());
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kCubic_Verb:
            head->addCubic(pts);
            lastPoint = pts[3];
            hasLastPoint = true;
            break;
        case SkPath::kClose_Verb:
            break;
        case SkPath::kDone_Verb:
            break;
        }
    } while (verb != SkPath::kDone_Verb);
    if (hasLastPoint && lastPoint != curveStart)
        head->addClose(lastPoint, curveStart);
    return true;
}

#endif
