#include "OpContour.h"
#include "OpEdge.h"

OpIntersection* OpContour::addIntersection(const OpPtT& t, OpSegment* seg, SelfIntersect self_, int cID
        OP_DEBUG_PARAMS(IntersectMaker maker)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg, self_, cID  OP_DEBUG_PARAMS(maker));
    return next;
}

#if OP_DEBUG
void OpContour::debugComplete() {
    id = contours->id++;
}
#endif

OpIntersection* OpContours::allocateIntersection() {
    if (sectStorage->used == ARRAY_COUNT(sectStorage->storage)) {
        OpSectStorage* next = new OpSectStorage;
        next->next = sectStorage;
        sectStorage = next;
    }
    return &sectStorage->storage[sectStorage->used++];
}

bool OpContours::closeGap(OpEdge* last, OpEdge* first) {
    OpPoint start = first->whichPtT(EdgeMatch::start).pt;
    OpPoint end = last->whichPtT(EdgeMatch::end).pt;
    for (auto& contour : contours) {
        for (auto& segment : contour.segments) {
#if 0 && OP_DEBUG
            if (4 == segment.id)
                OpDebugOut("");
#endif
            for (auto& edge : segment.edges) {
                if (start != edge.start.pt && end != edge.start.pt)
                    continue;
                if (start != edge.end.pt && end != edge.end.pt)
                    continue;
                if (!edge.unsortable && edge.windZero != WindZero::noFlip)
                    continue;
                if (edge.isPoint)
                    continue;
                last->setNextEdge(&edge);
                first->setPriorEdge(&edge);
                edge.setNextEdge(first);
                edge.setPriorEdge(last);
                edge.whichEnd = start == edge.end.pt ? EdgeMatch::start : EdgeMatch::end;
                return true;
            }
        }
    }
    return false;
}
