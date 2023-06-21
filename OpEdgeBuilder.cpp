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
//    focus(445);
    redraw();
    OpDebugOut("");
#endif
    // match up edges that have only a single possible prior or next link, and add them to new list
    std::vector<OpEdge*> linkups;
    for (auto& leftMost : edges.inX) {
        if (!leftMost->winding.visible())
            continue;   // likely marked as part of a loop below
        if (!leftMost->isActive())  // check if already saved in linkups
            continue;
        OP_ASSERT(EdgeLink::unlinked == leftMost->priorLink);
        OP_ASSERT(EdgeLink::unlinked == leftMost->nextLink);
        leftMost->whichEnd = EdgeMatch::start;
        OpEdge* linkup = leftMost->linkUp(EdgeMatch::start, leftMost);
        if (!linkup->containsLink(leftMost)) {
            linkup->output(path);
            OpEdge* pFirst = leftMost->prepareForLinkup();
            linkups.emplace_back(pFirst);
            continue;
        }
        if (linkup != leftMost)
            linkup->lastEdge = leftMost;
        if (leftMost->isClosed(linkup)) {
            leftMost->output(path);  // emit the contour
            continue;
        }
        OpEdge* first = linkup ? linkup : leftMost;
        OpEdge* newLast = leftMost->linkUp(EdgeMatch::end, first);
        if (!leftMost->containsLink(newLast)) {
            newLast->output(path);
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
                first->output(path);  // emit the contour
                continue;
            }
        first = first->prepareForLinkup();
        linkups.emplace_back(first);
    }
    for (auto linkup : linkups) {
        OP_ASSERT(!linkup->isLoop(WhichLoop::prior, EdgeLoop::link, LeadingLoop::will));
        OP_ASSERT(!linkup->isLoop(WhichLoop::next, EdgeLoop::link, LeadingLoop::will));
        OP_ASSERT(linkup->lastEdge);
        OP_ASSERT(!linkup->priorEdge);
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
        linkup->output(path);  // emit the contour
    }
    return true;
}
