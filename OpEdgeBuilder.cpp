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
//        OpDebugBreak(leftMost, 267, true);
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
        OpEdge* newLast = leftMost->linkUp(EdgeMatch::end, leftMost);
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
            // !!! NOT : if it is nearly a loop and can be closed with a unsortable edge, do that
            // !!! TODO : find direction of loop at add 'reverse' param to output if needed
            //     direction should consider whether edge normal points to inside or outside
            if (newLast->isClosed(first) /* || c.closeGap(newLast, first) */ ) {
                Output(first, path);  // emit the contour
                continue;
            }
        }
        first = first->prepareForLinkup();
        linkups.emplace_back(first);
    }

    // !!! this code:
    //   || c.closeGap(leftMost, linkup)
    // was removed after 'is closed' check above
    // removed because it can add edges with a zeroed winding too soon
    // either: add a new loop to see if linkup can be closed, or
    // add check for close gap below
    for (auto linkup : linkups) {
        assert(!linkup->isLoop(WhichLoop::prior, EdgeLoop::link, LeadingLoop::will));
        assert(!linkup->isLoop(WhichLoop::next, EdgeLoop::link, LeadingLoop::will));
        assert(linkup->lastEdge);
        assert(!linkup->priorEdge);
        do {
            linkup->setActive();
        } while ((linkup = linkup->nextEdge));
    }
    // for each remainder, find closest
    // see if gap can be closed by an 'is sum loop' edge
    // !!! this should be generalized to allow more more than one edge
    // create array of qualifying endpoints
    // create array of qualifying 'is sum loop' edges
    if (c.sumLoops.size()) {
        std::vector<int> loopyCounts;
        loopyCounts.resize(c.sumLoops.size(), 0);
        auto findSumLoop = [&](const std::vector<OpEdge*>& sumLoops, OpPoint pt, OpEdge** linkPtr) { // lambda
            assert(!*linkPtr);
            for (size_t sumIndex = 0; sumIndex < sumLoops.size(); ++sumIndex) {
                const OpEdge* loopy = sumLoops[sumIndex];
                assert(loopy->isLoop(WhichLoop::prior, EdgeLoop::sum, LeadingLoop::in));
                const OpEdge* test = loopy;
                do {
                    assert(test->isSumLoop);
                    if (test->start.pt == pt || test->end.pt == pt) {
                        assert(!*linkPtr || *linkPtr == loopy); // !!! multiple matching sum loop needs code
                        *linkPtr = const_cast<OpEdge*>(loopy);
                        loopyCounts[sumIndex]++;
                        assert(loopyCounts[sumIndex] <= 2);  // larger, likely will not work
                        break;
                    }
                    test = test->priorSum;
                } while (test != loopy);
            }
        };
        // if linkup free end matches sum loop, and sum loop winding agrees, add it
        // !!! wait on winding check until we have a test case
        for (size_t index = 0; index < linkups.size(); ++index) {
            auto edge = linkups[index];
            if (EdgeLink::unlinked == edge->priorLink)
                findSumLoop(c.sumLoops, edge->start.pt, &edge->priorEdge);
            if (EdgeLink::unlinked == edge->lastEdge->nextLink)
                findSumLoop(c.sumLoops, edge->lastEdge->end.pt, &edge->lastEdge->nextEdge);
        }
        // if sum loop bridges a pair of free ends, connect it up
        for (size_t index = 0; index < linkups.size(); ++index) {
            auto edge = linkups[index];
            WhichLoop loopEnd = WhichLoop::undetermined;
            WhichLoop farEnd = WhichLoop::undetermined;
            if (EdgeLink::unlinked == edge->priorLink) {
                OpEdge* loopy = edge->priorEdge;
                if (loopy) {
                    assert(loopy->isSumLoop);
                    const OpEdge* test = loopy;
                    do {
                        if (edge->start.pt == test->start.pt)
                            loopEnd = WhichLoop::prior;
                        else if (edge->start.pt == test->end.pt)
                            loopEnd = WhichLoop::next;
                        else
                            goto next;
                        for (auto farSide : linkups) {
                            if (edge == farSide)
                                continue;
                            if (EdgeLink::unlinked == farSide->priorLink && loopy == farSide->priorEdge)
                                farEnd = WhichLoop::prior;
                            else if (EdgeLink::unlinked == farSide->lastEdge->nextLink 
                                    && loopy == farSide->lastEdge->nextEdge)
                                farEnd = WhichLoop::next;
                        }
                    next:
                        test = test->priorSum;
                    } while (test != loopy);
                }
            }
            if (EdgeLink::unlinked == edge->lastEdge->nextLink) {
                OpEdge* loopy = edge->lastEdge->nextEdge;
                if (loopy) {

                }
            }
        }
    }
#if 0
    // Find bounds of remaining linked lists. Prioritize the largest bounds
    for (auto linkup : linkups) {
        assert(linkup->lastEdge);
        assert(!linkup->lastEdge->nextEdge);
        // !!! incomplete
    }
#endif
#if 01 && OP_DEBUG
    OpDebugOut("");
#endif
    // !!! to do : find edges to fill gaps in remaining pieces, starting with the largest
    for (auto linkup : linkups) {
        if (!linkup->isActive())
            continue;
        if (!linkup->matchLink(linkups))
            return false;   // if edges form loop with tail, fail
        // determine direction of linked edges, if any (a straight line won't have a direction)
        Output(linkup, path);  // emit the contour
#if 0 && OP_DEBUG
        dump(linkups);
        OpDebugOut("");
#endif
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
