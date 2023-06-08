#include "OpContour.h"
#include "OpEdge.h"

OpIntersection* OpContour::addIntersection(const OpPtT& t, OpSegment* seg, SelfIntersect self_, int cID
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
        const OpIntersection* master, const OpEdge* edge, const OpEdge* oEdge)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg, self_, cID  OP_DEBUG_PARAMS(maker, line, file, reason, master, edge, oEdge, 
            nullptr, nullptr));
    return next;
}

OpIntersection* OpContour::addIntersection(const OpPtT& t, OpSegment* seg, SelfIntersect self_, int cID
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason,
        const OpSegment* dSeg, const OpSegment* oSeg)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg, self_, cID  OP_DEBUG_PARAMS(maker, line, file, reason, nullptr, nullptr, 
            nullptr, dSeg, oSeg));
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
    auto connectBetween = [=](OpEdge* edge) {  // lambda
        if (start != edge->start.pt && end != edge->start.pt)
            return false;
        if (start != edge->end.pt && end != edge->end.pt)
            return false;
        edge->linkNextPrior(first, last);
        return true;
    };
    for (auto edge : unsortables) {
        if (connectBetween(edge)) {
            edge->unsortable = false;   // only use edge once
            return true;
        }
    }
    return false;
}
