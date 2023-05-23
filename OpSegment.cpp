#include "OpContour.h"
#include "OpSegment.h"

OpSegment::OpSegment(const OpPoint pts[], OpType type, OpContour* parent)
    : contour(parent)
    , c(pts, type)
    , winding(parent->operand) {
    complete();
}

OpSegment::OpSegment(const OpPoint pts[3], float w, OpContour* parent)
    : contour(parent)
    , c(pts, w)
    , winding(parent->operand) {
    complete();
}

OpSegment::OpSegment(OpPoint pt1, OpPoint pt2, OpContour* parent)
    : contour(parent)
    , c(pt1, pt2)
    , winding(parent->operand) {
    complete();
}

// !!! optimization:  if called from opedge linkup, could abort if >1 active found
// if operand is diff or reverse diff, the subtracted path has an interior of zero
// if the operand is union or intersection, both paths have an interior of one
void OpSegment::activeAtT(const OpEdge* edge, EdgeMatch match, std::vector<FoundEdge>& oppEdges,
        AllowReversal canReverse) const {
    assert(edge->winding.visible());
    // each prospective match normal must agree with edge, indicating direction of area outside fill
    OpPtT ptT = edge->whichPtT(match);
    for (auto sectPtr : intersections) {
        OpIntersection& sect = *sectPtr;
        if (sect.ptT.t < ptT.t)
            continue;  // !!! could binary search for 1st if intersection list is extremely long
        if (sect.ptT.t > ptT.t)
            break;
        OpPtT oppPtT = sect.opp->ptT;
        if (ptT.pt != oppPtT.pt)
            continue;
        const OpSegment* sectSeg = sect.opp->segment;
        OpEdge* start = sectSeg->findActive(oppPtT, EdgeMatch::start);  // !!! optimization: walk edges in order
        // flip zero if operator is diff/revdiff and edges are from both operands
        if (start && start != edge) {
            WindZero startZeroSide = start->windZero;
            if (EdgeMatch::start == match)
                flip(&startZeroSide);
            if (edge->windZero == startZeroSide) {
                if (!start->hasLinkTo(match))
                    oppEdges.emplace_back(start, EdgeMatch::start, AllowReversal::no);
                else if (AllowReversal::yes == canReverse && !start->hasLinkTo(Opposite(match)))
                    oppEdges.emplace_back(start, EdgeMatch::end, canReverse);
            }
        }
        OpEdge* end = sectSeg->findActive(oppPtT, EdgeMatch::end);
        if (end && end != edge) {
            WindZero endZeroSide = end->windZero;
            if (EdgeMatch::end == match)
                flip(&endZeroSide);
            if (edge->windZero == endZeroSide) {
                if (!end->hasLinkTo(match))
                    oppEdges.emplace_back(end, EdgeMatch::end, AllowReversal::no);
                else if (AllowReversal::yes == canReverse && !end->hasLinkTo(Opposite(match)))
                    oppEdges.emplace_back(end, EdgeMatch::start, canReverse);
            }
        }
    }
    if ((EdgeMatch::start == match && edge->start.t > 0)
            || (EdgeMatch::end == match && edge->end.t < 1)) {
        EdgeMatch neighbor = EdgeMatch::start == match ? Opposite(edge->whichEnd) : edge->whichEnd;
        OpEdge* nextDoor = findActive(ptT, neighbor);
        for (auto& alreadyFound : oppEdges)
            if (alreadyFound.edge == nextDoor)
                return;
        if (nextDoor) {
            if (edge->windZero == nextDoor->windZero) {
                if (!nextDoor->hasLinkTo(match))
                    oppEdges.emplace_back(nextDoor, neighbor, AllowReversal::no);
            }
        }
    }
}

OpIntersection* OpSegment::addIntersection(const OpPtT& ptT  OP_DEBUG_PARAMS(IntersectMaker maker)) {
    auto sect = contour->addIntersection(ptT, this, SelfIntersect::none, 0  OP_DEBUG_PARAMS(maker));
    intersections.push_back(sect);
    resortIntersections = true;
    return sect;
}

OpIntersection* OpSegment::addIntersection(const OpPtT& ptT, int coinID  
        OP_DEBUG_PARAMS(IntersectMaker maker)) {
    auto sect = contour->addIntersection(ptT, this, SelfIntersect::none, coinID  OP_DEBUG_PARAMS(maker));
    intersections.push_back(sect);
    resortIntersections = true;
    return sect;
}

OpIntersection* OpSegment::addIntersection(const OpPtT& ptT, SelfIntersect self  
        OP_DEBUG_PARAMS(IntersectMaker maker)) {
    auto sect = contour->addIntersection(ptT, this, self, 0  OP_DEBUG_PARAMS(maker));
    intersections.push_back(sect);
    resortIntersections = true;
    return sect;
}

void OpSegment::apply() {
    for (auto& edge : edges)
        edge.apply();
}

// if bounds is dirty, recalc bounds from intersections
// if segment collapses to point, remove entries from intersections?
void OpSegment::calcBounds() {
    if (!recomputeBounds)
        return;
    recomputeBounds = false;
    OpPointBounds bounds;
    for (auto sect : intersections) {
        bounds.add(sect->ptT.pt);
    }
    tightBounds.set(bounds);
    if (!tightBounds.isEmpty())
        return;
    winding.zero(ZeroReason::collapsed);
}

int OpSegment::coinID(bool flipped) {
    int coinID = ++contour->contours->coincidenceID;
    if (flipped)
        coinID = -coinID;
    return coinID;
}

void OpSegment::complete() {
    ptBounds.set(c);
    tightBounds.set(c);
    recomputeBounds = false;
    resortIntersections = true;
// #if OP_DEBUG     // used only by sort; probably unnecessary there
    id = contour->contours->id++;
// #endif
}

bool OpSegment::containsSect(float t, const OpSegment* opp) const {
    for (const OpIntersection* sect : intersections) {
        if (sect->ptT.t == t && sect->opp->segment == opp)
            return true;
    }
    return false;
}

// !!! would it be any better (faster) to split this into findStart / findEnd instead?
OpEdge* OpSegment::findActive(OpPtT ptT, EdgeMatch match) const {
    for (auto& edge : edges) {
        if (edge.isPoint)
            continue;
        if (ptT == (EdgeMatch::start == match ? edge.start : edge.end))
            return !edge.isActive() || !edge.winding.visible() ? nullptr : const_cast<OpEdge*>(&edge);
    }
    return nullptr;
}

void OpSegment::findExtrema() {
    if (pointType == c.type)
        return;
    if (!winding.left && !winding.right)
        return;
    assert(0 == edges.size());  // otherwise, call repair afterwards
    std::vector<OpPtT> selfPtTs;
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.xExtrema); ++index) {
        if (OpMath::IsNaN(tightBounds.xExtrema[index].t))
            break;
        selfPtTs.push_back(tightBounds.xExtrema[index]);
    }
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.yExtrema); ++index) {
        if (OpMath::IsNaN(tightBounds.yExtrema[index].t))
            break;
        selfPtTs.push_back(tightBounds.yExtrema[index]);
    }
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.inflections); ++index) {
        if (OpMath::IsNaN(tightBounds.inflections[index].t))
            break;
        selfPtTs.push_back(tightBounds.inflections[index]);
    }
    std::sort(selfPtTs.begin(), selfPtTs.end(), [](const OpPtT& s1, const OpPtT& s2) {
        return s1.t < s2.t; });
    if (selfPtTs.size()) {
        OpPtT last(OpPoint(), 0);
        for (auto& ptT : selfPtTs) {
            if (ptT.t == last.t)
                continue;
            // don't recompute end pts:  tightbounds and segment have t=0,xtrema,1,..
            if (1 == ptT.t)
                break;
            // add self intersections (e.g., the inflection point) since the opposite is this segment
            // can't point opp at self yet, since intersections will grow later
            addIntersection(ptT, SelfIntersect::self  OP_DEBUG_PARAMS(IntersectMaker::makeEdges));
            last = ptT;
        }
    }
    sortIntersections();
    assert(0 == intersections.front()->ptT.t);
    assert(1 == intersections.back()->ptT.t);
#if 0
    const OpIntersection* last = intersections.front();
    for (auto sect : intersections) {
        if (sect->ptT.t == last->ptT.t) // this is true for first intersection always
            sect->unsortable = last->unsortable; // note that this copies unsortable only
        last = sect;
    }
#endif
}

// start/end range is necessary since cubics can have more than one t at a point
float OpSegment::findPtT(float start, float end, OpPoint opp) const {
    if (OpType::lineType == c.type) {
        // this won't work for curves with linear control points since t is not necessarily linear
        OpVector lineSize = c.pts[1] - c.pts[0];
        float result = fabsf(lineSize.dy) > fabsf(lineSize.dx) ?
                (opp.y - c.pts[0].y) / lineSize.dy : (opp.x - c.pts[0].x) / lineSize.dx;
        return start <= result && result <= end ? result : OpNaN;
    }
    rootCellar hRoots, vRoots;
    int hRootCount = c.axisRayHit(Axis::horizontal, opp.y, hRoots, start, end);
    assert(1 >= hRootCount);
    int vRootCount = c.axisRayHit(Axis::vertical, opp.x, vRoots, start, end);
    assert(1 >= vRootCount);
    if (vRootCount)
        hRoots[hRootCount++] = vRoots[0];
    if (!hRootCount)
        return OpNaN;
    if (1 == hRootCount)
        return hRoots[0];
    return (hRoots[0] + hRoots[1]) / 2;
}

FoundPtT OpSegment::findPtT(float start, float end, OpPoint opp, float* result) const {
    if (OpType::lineType != c.type) {
        rootCellar hRoots, vRoots;
        int hRootCount = c.axisRayHit(Axis::horizontal, opp.y, hRoots, start, end);
        int vRootCount = c.axisRayHit(Axis::vertical, opp.x, vRoots, start, end);
        if (1 < hRootCount || 1 < vRootCount)
            return FoundPtT::multiple;
        // !!! here, since we've already called axis ray hit, don't call it again in find pt t below
        //     call common code that follows axis ray hit above
    }
    *result = findPtT(start, end, opp);
    return FoundPtT::single;
}

void OpSegment::fixEdges(const OpPtT& alias, OpPoint master  OP_DEBUG_PARAMS(int masterSectID)) {
    for (auto& edge : edges) {
        if (edge.start.pt == alias.pt) {
            OP_DEBUG_CODE(edge.debugOriginalStart = edge.start.pt);
            OP_DEBUG_CODE(edge.debugAliasStartID = masterSectID);
            edge.start.pt = master;
            edge.startAliased = true;
        }
        if (edge.end.pt == alias.pt) {
            OP_DEBUG_CODE(edge.debugOriginalEnd = edge.end.pt);
            OP_DEBUG_CODE(edge.debugAliasEndID = masterSectID);
            edge.end.pt = master;
            edge.endAliased = true;
        }
        if (edge.start.pt == edge.end.pt) {
            edge.isPoint = true;
            edge.winding.zero(ZeroReason::isPoint);
        }
    }
}

void OpSegment::fixIntersections(OpPoint alias, OpPoint master  OP_DEBUG_PARAMS(int masterSectID)) {
    for (auto sectPtr : intersections) {  // iterate again since cubic could cross over itself
        OpIntersection& inner = *sectPtr;
        if (inner.ptT.pt != alias)
            continue;
        if (edges.size())
            fixEdges(inner.ptT, master  OP_DEBUG_PARAMS(masterSectID));
        OP_DEBUG_CODE(inner.debugOriginal = inner.ptT.pt);
        OP_DEBUG_CODE(inner.debugAliasID = masterSectID);
        inner.ptT.pt = master;
        recomputeBounds = true;
        if (this == inner.opp->segment)
            continue;
        for (auto oPtr : inner.opp->segment->intersections) {
            OpIntersection& o = *oPtr;
            if (o.ptT.pt != alias)
                continue;
            OP_DEBUG_CODE(o.debugOriginal = o.ptT.pt);
            OP_DEBUG_CODE(o.debugAliasID = masterSectID);
            o.ptT.pt = master;
            o.segment->recomputeBounds = true;
            if (o.segment->edges.size())
                o.segment->fixEdges(o.ptT, master  OP_DEBUG_PARAMS(masterSectID));
            o.segment->fixIntersections(alias, master  OP_DEBUG_PARAMS(masterSectID));
        }
    }
}

void OpSegment::flip(WindZero* windZero) {
    if (WindZero::noFlip == *windZero)
        return;
    *windZero = WindZero::normal == *windZero ? WindZero::opp : WindZero::normal;
}

// when called, edges contain monotonic curves; intersections contains points
// split edges until all points are accounted for
void OpSegment::intersectEdge() {
    std::vector<OpEdge> merge;
    std::vector<OpIntersection*>::const_iterator sectIter = intersections.begin();
    for (std::vector<OpEdge>::const_iterator edgeIter = edges.begin();
            edgeIter != edges.end(); edgeIter++) {
        OpPtT start = edgeIter->start;
        OpPtT end = edgeIter->end;
        for ( ; sectIter != intersections.end(); sectIter++) {
            const OpIntersection& sect = **sectIter;
            if (sect.ptT.t == start.t) {   // ok if points don't match 
                continue;                       // (avoids short non-zero line with equal ts on end)
            }
            if (sect.ptT.t >= end.t) {
                if (edgeIter->start == start)
                    merge.push_back(*edgeIter);
                else
                    merge.emplace_back(this, start, end  OP_DEBUG_PARAMS(EdgeMaker::intersectEdge1));
                break;
            }
            if (start.pt != sect.ptT.pt) {
                merge.emplace_back(this, start, sect.ptT  OP_DEBUG_PARAMS(EdgeMaker::intersectEdge2));
                start = sect.ptT;
            }
            // else OpDebugOut("zero length span");
        }
    }
    if (merge.size() > edges.size())
        edges.swap(merge);
}

// count and sort extrema; create an edge for each extrema + 1
void OpSegment::makeEdges() {
    if (pointType == c.type)
        return;
    if (!winding.left && !winding.right)
        return;
    edges.reserve(intersections.size() - 1);
    const OpIntersection* last = intersections.front();
    for (auto sectPtr : intersections) {
        const OpIntersection& sect = *sectPtr;
        if (sect.ptT.t == last->ptT.t)
            continue;
        edges.emplace_back(this, last->ptT, sect.ptT  OP_DEBUG_PARAMS(EdgeMaker::makeEdges));
 //       if (last->unsortable && sect.unsortable)
 //           edges.back().unsortable = true;
        last = &sect;
    }
}

MatchEnds OpSegment::matchEnds(const OpSegment* opp, bool* reversed) const {
    MatchEnds result = MatchEnds::none;
    *reversed = false;
    if (c.pts[0] == opp->c.pts[0])
        result = MatchEnds::start;
    else if (c.pts[0] == opp->c.lastPt()) {
        result = MatchEnds::start;
        *reversed = true;
    }
    bool foundEnd = false;
    if (c.lastPt() == opp->c.pts[0])
        *reversed = foundEnd = true;
    else if (c.lastPt() == opp->c.lastPt())
        foundEnd = true;
    if (foundEnd) {
        result = (MatchEnds)((int) result | (int) MatchEnds::end);
        assert(MatchEnds::end == result || MatchEnds::both == result);
    }
    return result;
}

// look for coincident start or end with the range of, but missing from target's coincidence
void OpSegment::missingCoincidence(const OpPtT& start, const OpPtT& end,
        OpSegment* targetSegment,
        std::vector<int>& foundIDs, std::vector<MissingIntersection>& missing) const {
    for (auto sectPtr : intersections) {
        const OpIntersection& sect = *sectPtr;
        if (!sect.coincidenceID)
            continue;
        if (foundIDs.end() != std::find(foundIDs.begin(), foundIDs.end(), sect.coincidenceID))
            continue;
        if (!OpPoint::Between(start.pt, sect.ptT.pt, end.pt)) {
            continue;
        }
        bool alreadyFound = false;
        for (const auto target : targetSegment->intersections) {
            if (target->ptT.t == sect.ptT.t) {
                alreadyFound = true;
                break;
            }
        }
        if (alreadyFound)
            continue;
        missing.emplace_back(start, end, targetSegment, sect);
        foundIDs.push_back(sect.coincidenceID);
        sect.segment->missingCoincidence(start, end, targetSegment, foundIDs, missing);
    }
}

// Note the set of coincident edges bounded by identical ids may be of different lengths.
// A third coincidence may begin in those bounds; the third may have an intersection
// in the primary or opposite segment.
// The primary may need additional edges for coincidences found by opposites.
void OpSegment::missingCoincidence() {
    if (!intersections.size()) {
        assert(!winding.visible());
        return;
    }
    std::vector<MissingIntersection> missing;
     OpIntersection** sectPtr = &intersections.front();
    std::vector<int> coinStart;
    do {
        int coinID = (*sectPtr)->coincidenceID;
        if (!coinID)
            continue;
        if (coinStart.end() != std::find(coinStart.begin(), coinStart.end(), coinID))
            continue;
        coinStart.push_back(coinID);
         OpIntersection** lastPtr = sectPtr;
        std::vector<int> coinsFound { coinID };  // look for other coincidence inside this one
        do {
            const OpIntersection* last = *++lastPtr;
            if (coinID == last->coincidenceID)  // this segment coin goes from sect to last
                break;
            assert(*lastPtr != intersections.back());
            if (last->coincidenceID && coinsFound.end() !=
                    std::find(coinsFound.begin(), coinsFound.end(), last->coincidenceID)) {
                coinsFound.push_back(last->coincidenceID);
            }
        } while (true);
        if (coinsFound.size() > 1) {  // see if this edge list is missing coincident ends indirectly
            for (OpIntersection** testPtr = sectPtr + 1; testPtr != lastPtr; ++testPtr) {
                const OpIntersection* test = *testPtr;
                int testID = test->coincidenceID;
                if (!testID)
                    continue;
                assert(coinID != testID);
                // recursively visit intersection lists to see if they have points not on edges
                test->segment->missingCoincidence((*sectPtr)->ptT, (*lastPtr)->ptT, this, coinsFound, missing);
            }
        }
    } while (++sectPtr != &intersections.back());
    for (const MissingIntersection& miss : missing) {
        float foundT = miss.segment->findPtT(miss.start.t, miss.end.t, miss.intersection.ptT.pt);
        if (OpMath::IsNaN(foundT))
            continue;
        OpPtT foundPtT {miss.intersection.ptT.pt, foundT};
        bool foundOne = false;
        for (OpIntersection* inner : miss.segment->intersections) {
            if (miss.intersection.ptT.t == inner->ptT.t) {
                // !!! what to do if points don't match? wait for real-life test case
                assert(miss.intersection.ptT.pt == inner->ptT.pt);
                foundOne = true;
                break;
            }
            if (miss.intersection.ptT.pt == inner->ptT.pt) {
                // points match but t values do not. Figure out what do do
                assert(0);
                foundOne = true;
                break;
            }
        }
        if (!foundOne) {
            OpIntersection* sect = miss.segment->addIntersection(foundPtT,
                    SelfIntersect::missing  OP_DEBUG_PARAMS(IntersectMaker::missingCoincidence1));
            OpIntersection* opp = miss.intersection.segment->addIntersection(foundPtT,
                    SelfIntersect::missing  OP_DEBUG_PARAMS(IntersectMaker::missingCoincidence2));
            sect->pair(opp);
            miss.segment->resortIntersections = true;
        }
    }
}

// A coincident span may contain multiple edges.
// The individual edge and opp winding counts may vary. So the initial winding difference has to
// be applied to all subsequent edges, but only the initial opp winding can be zeroed.
bool OpSegment::resolveCoincidence() {
    if (!intersections.size()) {
        assert(!winding.visible());
        return true;
    }
    OpIntersection** sectPtr = &intersections.front();
    do {
        OpIntersection* sect = *sectPtr;
        int coinID = sect->coincidenceID;
        if (!coinID)
            continue;
        OpIntersection** lastPtr = sectPtr;
        do {
            assert(*lastPtr != intersections.back());
            ++lastPtr;
        } while (coinID != (*lastPtr)->coincidenceID);  // this segment coin goes from sect to last
        bool backwards = coinID < 0;   // sect and opp travel in different directions
        OpSegment* oSegment = sect->opp->segment;
        OpIntersection** oppSectPtr = &oSegment->intersections.front();  // find opp coinid first, last
        while (coinID != (*oppSectPtr)->coincidenceID) {
            assert(*oppSectPtr != oSegment->intersections.back());
            ++oppSectPtr;
        }
        OpIntersection** oppLastPtr = oppSectPtr;
        do {
            assert(*oppLastPtr != oSegment->intersections.back());
            ++oppLastPtr;
        } while (coinID != (*oppLastPtr)->coincidenceID);    // opp coin goes from oppSect to oppLast
        if (backwards)
            std::swap(oppSectPtr, oppLastPtr);    // oppSect matches sect; oppLast matches last
        int direction = backwards ? -1 : 1;
        EdgeMatch match = backwards ? EdgeMatch::end : EdgeMatch::start;
        EdgeMatch oppositeMatch = Opposite(match);
        bool swapEdgeAndOpp = false;
        OpEdge* edge = &edges.front();
        while (edge->start != sect->ptT) {  // sets edge to first in segment coin
            assert(edge != &edges.back());
            ++edge;
        }
        // sect and opp edges must have matching points for the spans with identical windings        
        OpEdge* oEdge = backwards ? &oSegment->edges.back() : &oSegment->edges.front();
        OP_DEBUG_CODE(OpEdge* oLast = backwards ? &oSegment->edges.front() : &oSegment->edges.back());
        while (oEdge->ptT(match).t != sect->opp->ptT.t) {   // points may not match if sect list has multiple t matches
            assert(oEdge != oLast);  // !!! add preflight when edge is created if this ever asserts
            oEdge += direction;
        }
        // first time, create spans of unique windings from edges and opposite edges
        OpEdge* firstEdge = edge;
        OpEdge* firstOpp = oEdge;
        assert(firstOpp->ptT(match).pt == firstEdge->start.pt);
        OpWinding eWinding = edge->winding;
        OpWinding oWinding = oEdge->winding;
        std::vector<const OpEdge*> eWindingChanges;
        float lastT = (*lastPtr)->ptT.t;
        while (edge->end.t != lastT) {  // ...so loop may be skipped
            OpEdge* next = edge + 1;
            if (eWinding.left != next->winding.left || eWinding.right != next->winding.right) {
                // The proportion of where the winding change happens on one edge can't be used to
                // compute the t value on the opposite edge, because the two segments' control 
                // points aren't aligned: the t values may accelerate differently.
                eWindingChanges.push_back(next);
                eWinding = next->winding;
            }
            edge = next;
        }
        std::vector<const OpEdge*> oWindingChanges;
        float oppLastT = (*oppLastPtr)->ptT.t;
        while (oEdge->ptT(oppositeMatch).t != oppLastT) {
            OpEdge* oNext = oEdge + direction;
            if (oWinding.left != oNext->winding.left || oWinding.right != oNext->winding.right) {
                oWindingChanges.push_back(oNext);
                oWinding = oNext->winding;
            }
            oEdge = oNext;
        }
        if (eWindingChanges.size() && oWindingChanges.size()) {
            bool changed = oSegment->splitAtWinding(eWindingChanges, firstOpp, direction
                    OP_DEBUG_PARAMS(*oppLastPtr, oppositeMatch));
            changed |= splitAtWinding(oWindingChanges, firstEdge, 1
                    OP_DEBUG_PARAMS(*lastPtr, EdgeMatch::end));
            if (changed)
                return false;  // call again if either edge list has been rewritten
        }
        swapEdgeAndOpp = !eWindingChanges.size() && oWindingChanges.size();
        if (swapEdgeAndOpp) {
            if (backwards) {
                std::swap(oppSectPtr, oppLastPtr);
                std::swap(oEdge, firstOpp);
                std::swap(sectPtr, lastPtr);
                std::swap(edge, firstEdge);
            }
            std::swap(firstEdge, firstOpp);
            std::swap(lastPtr, oppLastPtr);
        }
        edge = firstEdge;
        lastT = (*lastPtr)->ptT.t;
        oEdge = firstOpp;
        // !!! tricky: can it be written to be more clear?
        //     at this point edge and oEdge are at start of list of coincidences
        //     because of point alignment, one or the other may include edges with the same 
        //     start and end points (a non-zero t range). These point edges should be zeroed
        //     but advance the edge lists unevenly. Each time through the loop, zero all
        //     point edges. Move nonzero pairs from opp to edge. Advance point edges, but
        //     only advance both if both are not points. Stop with both pairs are on final t.
        do {
            bool edgeIsPoint = edge->isPoint;
            bool oppIsPoint = oEdge->isPoint;
            if (edgeIsPoint)
                edge->winding.zero(ZeroReason::isCoinPoint);
            else if (!oppIsPoint)
                edge->winding.move(oEdge->winding, contour->contours, backwards);
            if (!edgeIsPoint || oppIsPoint)
                oEdge->winding.zero(ZeroReason::resolveCoin);
            bool edgeDone = edge->end.t == lastT;
            bool oppDone = oEdge->ptT(oppositeMatch).t == oppLastT;
            if (edgeDone && oppDone)
                break;
            if (!edgeDone && (edgeIsPoint || !oppIsPoint))
                ++edge;
            if (!oppDone && (oppIsPoint || !edgeIsPoint))
                oEdge += direction;
        } while (true);
        (*sectPtr)->zeroCoincidenceID();
        (*lastPtr)->zeroCoincidenceID();
    } while (++sectPtr != &intersections.back());
    return true;
}

// Different intersections and self intersections may compute the same t but different point values.
// After edges are processed, but before they are assembled, resolve this by moving duplicates
// towards the edge end.
// !!! bug? what if both points are at t == 0, t == 1? do both need to be preserved? or is this OK?
// !!! a possible optimization: record the last fixed number of intersections, and only scan if
//     the current number is greater. 
void OpSegment::resolvePoints() {
    auto oppPtT = [=](const OpIntersection& sect) {    // lambda 
        if (sect.opp)
            return sect.opp->ptT.t;
        if (SelfIntersect::none != sect.self)
            return sect.ptT.t;
        for (auto oPtr : sect.segment->intersections) {
            const OpIntersection& o = *oPtr;
            if (o.ptT.pt != sect.ptT.pt)
                continue;
            if (o.segment != this)  // ref uses '='
                continue;
            if (o.coincidenceID != sect.coincidenceID)
                continue;
            return o.ptT.t;
        }
        assert(0);
        return OpNaN;
    };
    const OpIntersection* last = intersections.front();
    for (auto sectPtr : intersections) {
        const OpIntersection& sect = *sectPtr;
        if (sect.ptT.t == last->ptT.t && sect.ptT.pt != last->ptT.pt) {
            // get opposite intersection to see if it is close to edge end
            float sectOppT = oppPtT(sect);
            float lastOppT = oppPtT(*last);
            bool masterIsLast = fabsf(sectOppT - .5f) < fabsf(lastOppT - .5f);
            OpPoint masterPt = masterIsLast ? last->ptT.pt : sect.ptT.pt; // bias towards end
            OpPoint aliasPt = masterIsLast ? sect.ptT.pt : last->ptT.pt;
            // if intersection or edge equals alias pt, change to master pt
            fixIntersections(aliasPt, masterPt  OP_DEBUG_PARAMS(sect.id));
        }
        last = &sect;
    }
}

void OpSegment::sortIntersections() {
    if (!resortIntersections) {
#if OP_DEBUG
        const OpIntersection* last = intersections.size() ? intersections.front() : nullptr;
        for (const auto sectPtr : intersections) {
            assert(last->ptT.t <= sectPtr->ptT.t);
            last = sectPtr;
        }
#endif
        return;
    }
    std::sort(intersections.begin(), intersections.end(),
        [](const OpIntersection* s1, const OpIntersection* s2) {
            return s1->ptT.t < s2->ptT.t 
                    /* || (s1->ptT.t == s2->ptT.t && !s1->unsortable < !s2->unsortable ) */ ; 
        });
    resortIntersections = false;
}

// if there are winding changes, find the split t for the opposite edge
bool OpSegment::splitAtWinding(const std::vector<const OpEdge*>& windingChanges,
        const OpEdge* first, int direction
        OP_DEBUG_PARAMS(const OpIntersection* last, EdgeMatch oppositeMatch)) {
    std::vector<OpPtT> splitTs;
    const OpEdge* const * eChange = &windingChanges.front();
    const OpEdge* split = first;
    do {    // loop once per edge
        do {    // loop once per winding change
            OpPoint changePt = (*eChange)->start.pt;
            if (!OpPoint::Between(split->start.pt, changePt, split->end.pt))
                break;
            // !!! skip exact matches; consider moving this into findPtT for other callers
            if (split->start.pt != changePt && split->end.pt != changePt) {
                float splitT = findPtT(split->start.t, split->end.t, changePt);
                if (split->start.t < splitT && splitT < split->end.t) {   // t inside
                    OpPtT changePtT{ changePt, splitT };
                    splitTs.push_back(changePtT);
                    addIntersection(changePtT, SelfIntersect::split
                            OP_DEBUG_PARAMS(IntersectMaker::splitAtWinding));
                }
            }
            if (++eChange > &windingChanges.back())
                goto done;  // only exit to outside loop
        } while (true);
        assert(split->ptT(oppositeMatch).t != last->ptT.t);
        split += direction;
    } while (true);
done:
    assert(eChange > &windingChanges.back());
    if (!splitTs.size())
        return false;
    if (direction < 0)
        std::reverse(splitTs.begin(), splitTs.end());
    splitTs.push_back(edges.back().end);
    std::vector<OpEdge> splits;
    const OpPtT* splitPtT = &splitTs.front();
    const OpEdge* edge = &edges.front();
    OpPtT lastPtT = edge->start;
    do {
        if (edge->end.t <= splitPtT->t) {
            if (lastPtT == edge->start)
                splits.push_back(*edge);
            else
                splits.emplace_back(edge, lastPtT, edge->end  ODP(EdgeMaker::resolveCoin1));
            ++edge;
            lastPtT = edge->start;
        } else {
            assert(splitPtT <= &splitTs.back());
            splits.emplace_back(edge, lastPtT, *splitPtT  ODP(EdgeMaker::resolveCoin2));
            lastPtT = *splitPtT++;
        }
    } while (edge <= &edges.back());
    std::swap(edges, splits);
    sortIntersections();
    return true;
}

bool OpSegment::validEdge(float startT, float endT) const {
    auto valid = [&](float t) {
        return !(startT < t && t < endT);
    };
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.xExtrema); ++index)
        if (!valid(tightBounds.xExtrema[index].t))
            return false;
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.yExtrema); ++index)
        if (!valid(tightBounds.yExtrema[index].t))
            return false;
    for (size_t index = 0; index < ARRAY_COUNT(tightBounds.inflections); ++index)
        if (!valid(tightBounds.inflections[index].t))
            return false;
    return true;
}

// active at t seems too complicated: this is the same code/idea simplified for edge set windings
OpEdge* OpSegment::visibleAdjacent(OpEdge* edge, const OpPtT& ptT) {
    OpEdge* result = nullptr;
    if (0 != ptT.t && 1 != ptT.t) {
        for (OpEdge& test : edges) {
            if (test.end.t < ptT.t)
                continue;
            if (test.start.t > ptT.t)
                break;
            if (&test == edge)
                continue;
            if (!test.winding.visible())
                continue;
            assert(test.start.t == ptT.t || test.end.t == ptT.t);
            result = &test;
        }
    }
    for (auto sectPtr : intersections) {
        OpIntersection& sect = *sectPtr;
        if (sect.ptT.t < ptT.t)
            continue;  // !!! could binary search for 1st if intersection list is extremely long
        if (sect.ptT.t > ptT.t)
            break;
        if (SelfIntersect::self == sect.self)
            continue;
        assert(this != sect.opp->segment);
        assert(ptT.pt == sect.ptT.pt);
        assert(ptT.pt == sect.opp->ptT.pt);
        for (OpEdge& search : sect.opp->segment->edges) {
            if (search.start.t > sect.opp->ptT.t)
                break;
            if (search.end.t < sect.opp->ptT.t)
                continue;
            if (!search.winding.visible())
                continue;
            assert(&search != edge);
            if (result) // if there are two results, give up
                return nullptr;
            assert(search.start.t == sect.opp->ptT.t || search.end.t == sect.opp->ptT.t);
            result = &search;
        }
    }
    return result;
}

static bool compareXBox(const OpSegment* s1, const OpSegment* s2) {
    const OpRect& r1 = s1->ptBounds;
    const OpRect& r2 = s2->ptBounds;
    if (r1.left < r2.left)
        return true;
    if (r1.left > r2.left)
        return false;
    if (r1.left == r2.left && r1.right < r2.right)
        return true;
    if (r1.left == r2.left && r1.right > r2.right)
        return false;
    return s1->id < s2->id;
}

OpSegments::OpSegments(OpContours& contours) {
    inX.clear();
    for (auto& contour : contours.contours) {
        for (auto& segment : contour.segments) {
            if (pointType == segment.c.type)
                continue;
            inX.push_back(&segment);
        }
    }
    std::sort(inX.begin(), inX.end(), compareXBox);
}

void OpSegments::AddIntersection(OpSegment* opp, OpSegment* seg) {
    OpRoots septs;
    assert(lineType == seg->c.type);
    std::array<OpPoint, 2> edgePts { seg->c.pts[0], seg->c.pts[1] };
#if 0 && OP_DEBUG
    if (4 == opp->id && 2 == seg->id) {
        showSegments();
        showIDs();
        OpDebugOut("");
    }
#endif
    septs.count = opp->c.rayIntersect(edgePts, septs.roots);
    bool reversed;
    MatchEnds common = seg->matchEnds(opp, &reversed);
    if (!septs.count && MatchEnds::none == common)
        return; // IntersectResult::no;
    if (lineType == opp->c.type && MatchEnds::both == common) {
        seg->winding.move(opp->winding, seg->contour->contours, seg->c.pts[0] != opp->c.pts[0]);
        opp->winding.zero(ZeroReason::addIntersection);
        return; // IntersectResult::yes;
    }
    if (2 == septs.count && lineType == opp->c.type) {
        return (void) OpEdges::CoincidentCheck({ seg->c.pts[0], 0 }, { seg->c.pts[1], 1 },
                { opp->c.pts[0], 0}, { opp->c.pts[1], 1 }, seg, opp );
    }
    if ((int) MatchEnds::start & (int) common)
        septs.addEnd(reversed ? 1 : 0);
    if ((int) MatchEnds::end & (int) common)
        septs.addEnd(reversed ? 0 : 1);
    if (septs.count > 1)
        std::sort(&septs.roots[0], &septs.roots[septs.count]);
//    int foundCount = 0;
    for (unsigned index = 0; index < septs.count; ++index) {
        OpPtT oppPtT { opp->c.ptAtT(septs.get(index)), septs.get(index) };
        float edgeT = seg->findPtT(0, 1, oppPtT.pt);
        if (OpMath::IsNaN(edgeT))
            continue;
        // pin point to both bounds, but only if it is on edge
        opp->tightBounds.pin(&oppPtT.pt);
        seg->tightBounds.pin(&oppPtT.pt);
        OpPtT edgePtT { oppPtT.pt, edgeT };
        OpIntersection* sect = seg->addIntersection(edgePtT  OP_DEBUG_PARAMS(IntersectMaker::addIntersection3));
        OpIntersection* oSect = opp->addIntersection(oppPtT  OP_DEBUG_PARAMS(IntersectMaker::addIntersection4));
        sect->pair(oSect);
//        ++foundCount;
    }
#if 0
    // if foundCount > 1, check each future edge pair to see if they are sortable
    for (int index = 0; index < foundCount - 1; ++index) {
        // create a pair of edges and see if one is in front of the other in x or y
        OpIntersection** oSectPtrB = &opp->intersections.back() - index;
        OpIntersection** oSectPtrA = oSectPtrB - 1;
        OpPtT start = (*oSectPtrA)->ptT;
        OpPtT end = (*oSectPtrB)->ptT;
        if (start.t > end.t)
            std::swap(start, end);
        if (!opp->validEdge(start.t, end.t))
            continue;
        OpEdge oEdge(opp, start, end  OP_DEBUG_PARAMS(EdgeMaker::addTest));
        OpIntersection** sSectPtrB = &seg->intersections.back() - index;
        OpIntersection** sSectPtrA = sSectPtrB - 1;
        OpEdge sEdge(seg, (*sSectPtrA)->ptT, (*sSectPtrB)->ptT  OP_DEBUG_PARAMS(EdgeMaker::addTest));
        OpEdges edges(&sEdge, &oEdge);
        FoundWindings foundWindings = edges.setWindings();
        if (FoundWindings::fail == foundWindings)
            return IntersectResult::fail;
        // if the intersection did not catastrophically fail, the edges are sortable, or coincident
        // this was: (edge fail names have changed)
//        if (sEdge.sum.visible() || EdgeFail::distance == sEdge.fail 
//                || oEdge.sum.visible() || EdgeFail::distance == oEdge.fail)
        // if this is right, add back with a comment why
        if (sEdge.sum.visible() || oEdge.sum.visible())
            continue;
        // if neither edge is in front of the other, they aren't sortable
        (*oSectPtrA)->unsortable = true;
        (*oSectPtrB)->unsortable = true;
        (*sSectPtrA)->unsortable = true;
        (*sSectPtrB)->unsortable = true;
    }
    return foundCount ? IntersectResult::yes : IntersectResult::no;
#endif
}

void OpSegments::findCoincidences() {
    // take care of totally coincident segments
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (!seg->winding.visible())
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (!opp->winding.visible())
                continue;
            if (seg->ptBounds != opp->ptBounds)
                continue;
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed);
            if (MatchEnds::both == match && seg->c.type == opp->c.type) {
                // if control points and weight match, treat as coincident: transfer winding
                bool coincident = false;
                switch (seg->c.type) {
                    case noType:
                        assert(0);
                        break;
                    case pointType:
                        break;
                    case lineType:
                        coincident = true;
                        break;
                    case quadType:
                        coincident = seg->c.pts[1] == opp->c.pts[1];
                        break;
                    case conicType:
                        coincident = seg->c.pts[1] == opp->c.pts[1]
                            && seg->c.weight == opp->c.weight;
                        break;
                    case cubicType:
                        coincident = seg->c.pts[1] == opp->c.pts[1 + (int) reversed]
                            && seg->c.pts[2] == opp->c.pts[2 - (int) reversed];
                        break;
                }
                if (coincident) {
                    seg->winding.move(opp->winding, seg->contour->contours, reversed);
                    opp->winding.zero(ZeroReason::findCoincidences);
                    if (!seg->winding.visible() || !opp->winding.visible())
                        continue;
                }
            }
        }
    }
}

FoundIntersections OpSegments::findIntersections() {
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (!seg->winding.visible())
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (!opp->winding.visible())
                continue;
            if (seg->ptBounds.right < opp->ptBounds.left)
                break;
            if (!seg->ptBounds.intersects(opp->ptBounds))
                continue;
            // for line-curve intersection we can directly intersect
            if (lineType == seg->c.type) {
                AddIntersection(opp, seg);
                continue;
            } else if (lineType == opp->c.type) {
                AddIntersection(seg, opp);
                continue;
            }
            // check if segments share endpoints
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed);
            OpIntersection* oppSect;
            if ((int) MatchEnds::start & (int) match) {
                auto sect = seg->addIntersection(OpPtT{ seg->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections3));
                if (reversed)
                    oppSect = opp->addIntersection(OpPtT{ opp->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections4));
                else
                    oppSect = opp->addIntersection(OpPtT{ opp->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections5));
                sect->pair(oppSect);
            }
            if ((int) MatchEnds::end & (int) match) {
                auto sect = seg->addIntersection(OpPtT{ seg->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections6));
                if (reversed)
                    oppSect = opp->addIntersection(OpPtT{ opp->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections7));
                else
                    oppSect = opp->addIntersection(OpPtT{ opp->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(IntersectMaker::findIntersections8));
                sect->pair(oppSect);
            }
        }
    }
    return FoundIntersections::yes; // !!! if something can fail, return 'fail' (don't return 'no')
}

