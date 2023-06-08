#include "OpEdgeBuilder.h"
#include "OpContour.h"
#include "OpEdges.h"
#include "PathOps.h"

// build list of linked edges
// if they are closed, done
// if not, match up remainder
// make sure normals point same way
// prefer smaller total bounds

bool OpEdgeBuilder::Assemble(OpContours& c, OpOutPath path) {
    OpEdges edges(c, EdgesToSort::byBox);  // collect active edges and sort them
    if (!edges.inX.size())
        return true;
    for (auto edge : edges.inX) {
        edge->setActive();
    }
#if 01 && OP_DEBUG
    clear();
    hideSegmentEdges();
    showIntersections();

    edges.draw();
//    addEdges();
    OpDebugOut("");
#endif
    // match up edges that have only a single possible prior or next link, and add them to new list
    std::vector<OpEdge*> linkups;
    for (auto& leftMost : edges.inX) {
        if (!leftMost->winding.visible())
            continue;   // likely marked as part of a loop below
        if (!leftMost->isActive())  // check if already saved in linkups
            continue;
        assert(EdgeLink::unlinked == leftMost->priorLink);
        assert(EdgeLink::unlinked == leftMost->nextLink);
        leftMost->whichEnd = EdgeMatch::start;
        OpEdge* linkup = leftMost->linkUp(EdgeMatch::start, leftMost);
        if (!linkup->containsLink(leftMost)) {
            Output(linkup, path);
            OpEdge* pFirst = leftMost->prepareForLinkup();
            linkups.emplace_back(pFirst);
            continue;
        }
        if (linkup != leftMost)
            linkup->lastEdge = leftMost;
        if (leftMost->isClosed(linkup)) {
            Output(leftMost, path);  // emit the contour
            continue;
        }
        OpEdge* first = linkup ? linkup : leftMost;
        OpEdge* newLast = leftMost->linkUp(EdgeMatch::end, first);
        if (!leftMost->containsLink(newLast)) {
            Output(newLast, path);
            first = leftMost->prepareForLinkup();
            linkups.emplace_back(first);
            continue;
        }
        if (newLast != leftMost) {
            if (linkup)
                linkup->lastEdge = newLast;
            // if a closed loop is formed, just output that
            // if it is nearly a loop and can be closed with a unsortable edge, do that
            // !!! TODO : find direction of loop at add 'reverse' param to output if needed
            //     direction should consider whether edge normal points to inside or outside
        }
            if (newLast->isClosed(first) || c.closeGap(newLast, first)) {
                Output(first, path);  // emit the contour
                continue;
            }
        first = first->prepareForLinkup();
        linkups.emplace_back(first);
    }
    for (auto linkup : linkups) {
        assert(!linkup->isLoop(WhichLoop::prior, EdgeLoop::link, LeadingLoop::will));
        assert(!linkup->isLoop(WhichLoop::next, EdgeLoop::link, LeadingLoop::will));
        assert(linkup->lastEdge);
        assert(!linkup->priorEdge);
        do {
            linkup->setActive();
        } while ((linkup = linkup->nextEdge));
    }
#if 01 && OP_DEBUG
    OpDebugOut("");
#endif
    // !!! to do : find edges to fill gaps in remaining pieces, starting with the largest
    for (auto linkup : linkups) {
        if (!linkup->isActive())
            continue;
        if (!linkup->lastEdge->isClosed(linkup) && !c.closeGap(linkup->lastEdge, linkup)
                && !linkup->matchLink(linkups))
            return false;   // if edges form loop with tail, fail
        // determine direction of linked edges, if any (a straight line won't have a direction)
        Output(linkup, path);  // emit the contour
    }
    return true;
}

#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA

#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"

bool OpSegmentBuilder::Build(OpInPath path, OpContours& c, OpOperand operand) {
    const SkPath& skPath = *path.skPath;
    if (!skPath.isFinite())
        return false;
    OpFillType exor = SkPathFillType::kEvenOdd == skPath.getFillType()
        || SkPathFillType::kInverseEvenOdd == skPath.getFillType() 
        ? OpFillType::evenOdd : OpFillType::winding;
    c.contours.emplace_back(&c, operand);
    (OpOperand::left == operand ? c.left : c.right) = exor;
    OpContour* head = &c.contours.back();
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

#endif
