#include "OpContour.h"
#include "OpSegment.h"

OpSegment::OpSegment(const CurvePts& pts, OpType type
        OP_DEBUG_PARAMS(SectReason startReason, SectReason endReason, OpContour* debugContour))
    : c(&pts.pts.front(), type)
    , winding(WindingUninitialized::dummy) {
    complete(OP_DEBUG_CODE(debugContour));
    OP_DEBUG_CODE(contour = nullptr);
    OP_DEBUG_CODE(debugStart = startReason);
    OP_DEBUG_CODE(debugEnd = endReason);
}

// !!! optimization:  if called from opedge linkup, could abort if >1 active found
// if operand is diff or reverse diff, the subtracted path has an interior of zero
// if the operand is union or intersection, both paths have an interior of one
void OpSegment::activeAtT(const OpEdge* edge, EdgeMatch match, std::vector<FoundEdge>& oppEdges,
        AllowReversal canReverse) const {
    OP_ASSERT(edge->winding.visible());
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

OpIntersection* OpSegment::addEdgeSect(const OpPtT& ptT  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason,
        const OpEdge* debugEdge, const OpEdge* debugOpp)) {
    OP_ASSERT(!debugContains(ptT, debugOpp->segment));
    auto sect = contour->addEdgeSect(ptT, this  
            OP_DEBUG_PARAMS(maker, line, file, reason, debugEdge, debugOpp));
    intersections.push_back(sect);
    resortIntersections = true;
    return sect;
}

OpIntersection* OpSegment::addSegBase(const OpPtT& ptT  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
        const OpSegment* oSeg)) {
    OP_ASSERT(!debugContains(ptT, oSeg));
    auto sect = contour->addSegSect(ptT, this, SectFlavor::none, 0  
            OP_DEBUG_PARAMS(maker, line, file, reason, oSeg));
    intersections.push_back(sect);
    return sect;
}

OpIntersection* OpSegment::addSegSect(const OpPtT& ptT  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
        const OpSegment* oSeg)) {
    resortIntersections = true;
    return addSegBase(ptT  OP_DEBUG_PARAMS(maker, line, file, reason, oSeg));
}

OpIntersection* OpSegment::addCoin(const OpPtT& ptT, int coinID  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
        const OpSegment* oSeg)) {
    OP_ASSERT(!debugContains(ptT, oSeg));
    auto sect = contour->addSegSect(ptT, this, SectFlavor::none, coinID  
            OP_DEBUG_PARAMS(maker, line, file, reason, oSeg));
    intersections.push_back(sect);
    resortIntersections = true;
    return sect;
}

#if 0
OpIntersection* OpSegment::addIntersection(const OpPtT& ptT, SectFlavor flavor  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
        const OpIntersection* master, const OpEdge* debugEdge, const OpEdge* debugOpp)) {
    auto sect = contour->addIntersection(ptT, this, flavor, 0  
            OP_DEBUG_PARAMS(maker, line, file, reason, master, debugEdge, debugOpp));
    intersections.push_back(sect);
    resortIntersections = true;
    return sect;
}
#endif

OpIntersection* OpSegment::addUnsectable(const OpPtT& ptT, SectFlavor flavor  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, const OpSegment* oSeg)) {
    OP_ASSERT(!debugContains(ptT, oSeg));
    auto sect = contour->addSegSect(ptT, this, flavor, 0  
            OP_DEBUG_PARAMS(maker, line, file, SectReason::curveCurveUnsectable, oSeg));
    intersections.push_back(sect);
    resortIntersections = true;
    return sect;
}

void OpSegment::apply() {
    for (auto& edge : edges)
        edge.apply();
}

#if 0
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
#endif

int OpSegment::coinID(bool flipped) {
    int coinID = ++contour->contours->coincidenceID;
    if (flipped)
        coinID = -coinID;
    return coinID;
}

void OpSegment::complete(OP_DEBUG_CODE(OpContour* debugContour)) {
    ptBounds.set(c);
    recomputeBounds = false;
    resortIntersections = false;
// #if OP_DEBUG     // used only by sort; probably unnecessary there
    id = debugContour->contours->id++;  // contour pointer is not set up
// #endif
}

bool OpSegment::containsIntersection(OpPtT ptT, const OpSegment* opp) const {
    for (auto sect : intersections) {
        if (sect->opp->segment != opp)
            continue;
        if (sect->ptT.t == ptT.t || sect->ptT.pt == ptT.pt)
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

// start/end range is necessary since cubics can have more than one t at a point
float OpSegment::findPtT(float start, float end, OpPoint opp) const {
    float result;
    OP_DEBUG_CODE(FoundPtT found =) findPtT(start, end, opp, &result);
    OP_ASSERT(FoundPtT::single == found);
    return result;
}

FoundPtT OpSegment::findPtT(Axis axis, float start, float end, float opp, float* result) const {
    if (OpType::line != c.type) {
        OpRoots roots = c.axisRayHit(axis, opp, start, end);
         if (1 < roots.count)
            return FoundPtT::multiple;
         *result = 0 == roots.count ? OpNaN : roots.roots[0];
    } else {
        *result = (opp - c.pts[0].choice(axis)) / (c.pts[1].choice(axis) - c.pts[0].choice(axis));
        if (start > *result || *result > end)
            *result = OpNaN;
    }
    return FoundPtT::single;
}

FoundPtT OpSegment::findPtT(float start, float end, OpPoint opp, float* result) const {
    if (OpType::line != c.type) {
        OpRoots hRoots = c.axisRayHit(Axis::horizontal, opp.y, start, end);
        OpRoots vRoots = c.axisRayHit(Axis::vertical, opp.x, start, end);
        if (1 < hRoots.count || 1 < vRoots.count)
            return FoundPtT::multiple;
        if (0 == hRoots.count && 0 == vRoots.count)
            *result = OpNaN;
        else if (0 == hRoots.count)
            *result = vRoots.roots[0];
        else if (0 == vRoots.count)
            *result = hRoots.roots[0];
        else
            *result = (hRoots.roots[0] + vRoots.roots[0]) / 2;
    } else {
        // this won't work for curves with linear control points since t is not necessarily linear
        OpVector lineSize = c.pts[1] - c.pts[0];
        *result = fabsf(lineSize.dy) > fabsf(lineSize.dx) ?
            (opp.y - c.pts[0].y) / lineSize.dy : (opp.x - c.pts[0].x) / lineSize.dx;
        if (start > *result || *result > end)
            *result = OpNaN;
    }
    return FoundPtT::single;
}

FoundPtT OpSegment::findPtT(const OpPtT& start, const OpPtT& end, OpPoint opp, float* result) const {
    if (start.pt == opp) {
        *result = start.t;
        return FoundPtT::single;
    }
    if (end.pt == opp) {
        *result = end.t;
        return FoundPtT::single;
    }
    return findPtT(start.t, end.t, opp, result);
}

void OpSegment::fixEdges(OpPoint alias, OpPoint master  OP_DEBUG_PARAMS(int masterSectID)) {
    for (auto& edge : edges) {
        if (edge.start.pt == alias) {
            OP_DEBUG_CODE(edge.debugOriginalStart = edge.start.pt);
            OP_DEBUG_CODE(edge.debugAliasStartID = masterSectID);
            edge.start.pt = master;
            edge.startAliased = true;
        }
        if (edge.end.pt == alias) {
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
            fixEdges(alias, master  OP_DEBUG_PARAMS(masterSectID));
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
                o.segment->fixEdges(alias, master  OP_DEBUG_PARAMS(masterSectID));
            o.segment->fixIntersections(alias, master  OP_DEBUG_PARAMS(masterSectID));
        }
        // if adjusting the points causes a coincidence to collapse, remove it
        int coinID = inner.coincidenceID;
        if (coinID) {
            OP_DEBUG_CODE(bool foundCoin = false);
            for (auto coinPtr : intersections) {
                if (coinPtr->coincidenceID != coinID)
                    continue;
                if (coinPtr == &inner)
                    continue;
                OP_DEBUG_CODE(foundCoin = true);
                if (inner.ptT.pt == coinPtr->ptT.pt) {
                    inner.zeroCoincidenceID();
                    coinPtr->zeroCoincidenceID();
                    for (auto oppPtr : inner.opp->segment->intersections) {
                        if (oppPtr->coincidenceID != coinID)
                            continue;
                        oppPtr->zeroCoincidenceID();
                    }
                }
                break;
            }
            OP_ASSERT(foundCoin);
        }
    }
}

void OpSegment::flip(WindZero* windZero) {
    if (WindZero::noFlip == *windZero)
        return;
    *windZero = WindZero::normal == *windZero ? WindZero::opp : WindZero::normal;
}

#if 0
// when called, edges contain monotonic curves; intersections contains points
// split edges until all points are accounted for
void OpSegment::intersectEdge() {
    std::vector<OpEdge> merge;
    std::vector<OpIntersection*>::const_iterator sectIter = intersections.begin();
    for (std::vector<OpEdge>::const_iterator edgeIter = edges.begin();
            edgeIter != edges.end(); edgeIter++) {
        OpPtT start = edgeIter->start;
        OpPtT end = edgeIter->end;
        OP_DEBUG_CODE(const OpIntersection* debugStartSect = nullptr);
        for ( ; sectIter != intersections.end(); sectIter++) {
            const OpIntersection& sect = **sectIter;
            if (sect.ptT.t == start.t) {   // ok if points don't match 
                continue;                       // (avoids short non-zero line with equal ts on end)
            }
            if (sect.ptT.t >= end.t) {
                if (edgeIter->start == start)
                    merge.push_back(*edgeIter);
                else
                    merge.emplace_back(this, start, end  
                            OP_DEBUG_PARAMS(EDGE_MAKER(intersectEdge1), debugStartSect, nullptr));
                break;
            }
            if (start.pt != sect.ptT.pt) {
                merge.emplace_back(this, start, sect.ptT  
                        OP_DEBUG_PARAMS(EDGE_MAKER(intersectEdge2), debugStartSect, &sect));
                OpDebugBreak(&merge.back(), 402);
                OpDebugBreak(&merge.back(), 396);
                start = sect.ptT;
                OP_DEBUG_CODE(debugStartSect = &sect);
            }
        }
    }
    if (merge.size() > edges.size())
        edges.swap(merge);
}
#endif

void OpSegment::intersectRange(const OpSegment* opp, std::vector<OpIntersection*>& range) {
    OP_DEBUG_CODE(float last = -1);
    for (auto sect : intersections) {
        if (sect->opp->segment == opp) {
            OP_ASSERT(last < sect->ptT.t);
            OP_DEBUG_CODE(last = sect->ptT.t);
            range.push_back(sect);
        }
    }
}

void OpSegment::makeEdge(OP_DEBUG_CODE(EdgeMaker maker, int line, std::string file)) {
    if (!edges.size()) 
        edges.emplace_back(this, OpPtT(c.pts[0], 0), OpPtT(c.lastPt(), 1), false
                OP_DEBUG_PARAMS(maker, line, file, nullptr, nullptr));
}

// count and sort extrema; create an edge for each extrema + 1
void OpSegment::makeEdges() {
    if (!winding.left() && !winding.right()) {
        edges.clear();
        return;
    }
    if (1 == edges.size() && 2 == intersections.size())
        return;
    edges.clear();
    edges.reserve(intersections.size() - 1);
    const OpIntersection* last = intersections.front();
    int unsectableCount = 0;
    OP_DEBUG_CODE(std::vector<bool> unOpp);
    for (auto sectPtr : intersections) {
        const OpIntersection& sect = *sectPtr;
        if (sect.ptT.t != last->ptT.t) {
            if (last->flavor == SectFlavor::unsectableStart) {
                unsectableCount++;
                OP_DEBUG_CODE(unOpp.push_back(IntersectMaker::unsectableOppStart == last->debugMaker));
            }
            edges.emplace_back(this, last->ptT, sect.ptT, !!unsectableCount  // (is in unsectable)
                    OP_DEBUG_PARAMS(EDGE_MAKER(makeEdges), last, sectPtr));
//            OpDebugBreak(&edges.back(), 445);
            OP_DEBUG_CODE(edges.back().debugUnOpp = unsectableCount && unOpp.back());
        }
        if (sect.flavor == SectFlavor::unsectableEnd) {
            unsectableCount--;
            OP_DEBUG_CODE(unOpp.pop_back());
        }
        if (sect.ptT.t != last->ptT.t)
            last = &sect;
    }
}

MatchEnds OpSegment::matchEnds(const OpSegment* opp, bool* reversed, MatchSect matchSect) const {
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
        OP_ASSERT(MatchEnds::end == result || MatchEnds::both == result);
    }
    if (MatchEnds::none == result || MatchEnds::both == result)
        return result;
    if (MatchSect::existing != matchSect || !*reversed)
        return result;
    if (result != matchExisting(opp))
        return result;
    return MatchEnds::none;
}

MatchEnds OpSegment::matchExisting(const OpSegment* opp) const {
    if (contour != opp->contour || 2 == contour->segments.size())
        return MatchEnds::none;
    if (this + 1 == opp
            || (&contour->segments.front() == opp && &contour->segments.back() == this))
        return MatchEnds::end;
    if (this - 1 == opp
            || (&contour->segments.front() == this && &contour->segments.back() == opp))
        return MatchEnds::start;
    return MatchEnds::none;
}

#if 0
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
#endif

#if 0
// Note the set of coincident edges bounded by identical ids may be of different lengths.
// A third coincidence may begin in those bounds; the third may have an intersection
// in the primary or opposite segment.
// The primary may need additional edges for coincidences found by opposites.
void OpSegment::missingCoincidence() {
    if (!intersections.size()) {
        OP_ASSERT(!winding.visible());
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
            OP_ASSERT(*lastPtr != intersections.back());
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
                OP_ASSERT(coinID != testID);
                // recursively visit intersection lists to see if they have points not on edges
                test->segment->missingCoincidence((*sectPtr)->ptT, (*lastPtr)->ptT, this, 
                        coinsFound, missing);
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
                OP_ASSERT(miss.intersection.ptT.pt == inner->ptT.pt);
                foundOne = true;
                break;
            }
            if (miss.intersection.ptT.pt == inner->ptT.pt) {
                // points match but t values do not. Figure out what do do
                OP_ASSERT(0);
                foundOne = true;
                break;
            }
        }
        if (!foundOne) {
            OpIntersection* sect = miss.segment->addIntersection(foundPtT,
                    SectFlavor::missing  OP_DEBUG_PARAMS(SECT_MAKER(missingCoincidence),
                    SectReason::missingCoincidence, &miss.intersection, nullptr, nullptr));
            // !!! the intersection is already in this segment; why add it again?
            OP_ASSERT(0); // !!! add explanation of why this is correct
            OpIntersection* opp = miss.intersection.segment->addIntersection(foundPtT,
                    SectFlavor::missing  OP_DEBUG_PARAMS(SECT_MAKER(missingCoincidenceOpp),
                    SectReason::missingCoincidence, &miss.intersection, nullptr, nullptr));
            sect->pair(opp);
            miss.segment->resortIntersections = true;
        }
    }
}
#endif

#if 0
// A coincident span may contain multiple edges.
// The individual edge and opp winding counts may vary. So the initial winding difference has to
// be applied to all subsequent edges, but only the initial opp winding can be zeroed.
bool OpSegment::resolveCoincidence() {
    if (!intersections.size()) {
        OP_ASSERT(!winding.visible());
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
            OP_ASSERT(*lastPtr != intersections.back());
            ++lastPtr;
        } while (coinID != (*lastPtr)->coincidenceID);  // this segment coin goes from sect to last
        bool backwards = coinID < 0;   // sect and opp travel in different directions
        OpSegment* oSegment = sect->opp->segment;
        OpIntersection** oppSectPtr = &oSegment->intersections.front();  // find opp coinid first, last
        while (coinID != (*oppSectPtr)->coincidenceID) {
            OP_ASSERT(*oppSectPtr != oSegment->intersections.back());
            ++oppSectPtr;
        }
        OpIntersection** oppLastPtr = oppSectPtr;
        do {
            OP_ASSERT(*oppLastPtr != oSegment->intersections.back());
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
            OP_ASSERT(edge != &edges.back());
            ++edge;
        }
        // sect and opp edges must have matching points for the spans with identical windings        
        OpEdge* oEdge = backwards ? &oSegment->edges.back() : &oSegment->edges.front();
        OP_DEBUG_CODE(OpEdge* oLast = backwards ? &oSegment->edges.front() : &oSegment->edges.back());
        while (oEdge->ptT(match).t != sect->opp->ptT.t) {   // points may not match if sect list has multiple t matches
            OP_ASSERT(oEdge != oLast);  // !!! add preflight when edge is created if this ever asserts
            oEdge += direction;
        }
        // first time, create spans of unique windings from edges and opposite edges
        OpEdge* firstEdge = edge;
        OpEdge* firstOpp = oEdge;
        OP_ASSERT(firstOpp->ptT(match).pt == firstEdge->start.pt);
        OpWinding eWinding = edge->winding;
        OpWinding oWinding = oEdge->winding;
        std::vector<const OpEdge*> eWindingChanges;
        float lastT = (*lastPtr)->ptT.t;
        while (edge->end.t != lastT) {  // ...so loop may be skipped
            OpEdge* next = edge + 1;
            if (eWinding.left() != next->winding.left() || eWinding.right() != next->winding.right()) {
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
            if (oWinding.left() != oNext->winding.left() || oWinding.right() != oNext->winding.right()) {
                oWindingChanges.push_back(oNext);
                oWinding = oNext->winding;
            }
            oEdge = oNext;
        }
        if (eWindingChanges.size() && oWindingChanges.size()) {
            bool changed = oSegment->splitAtWinding(eWindingChanges, firstOpp, direction
                    OP_DEBUG_PARAMS(*oppLastPtr, oppositeMatch, firstEdge,
                    SectReason::resolveCoin_windingChange));
            changed |= splitAtWinding(oWindingChanges, firstEdge, 1
                    OP_DEBUG_PARAMS(*lastPtr, EdgeMatch::end, firstOpp,
                    SectReason::resolveCoin_oWindingChange));
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
            std::swap(lastT, oppLastT);
        }
        edge = firstEdge;
        oEdge = firstOpp;
        // !!! tricky: can it be written to be more clear?
        //     at this point edge and oEdge are at start of list of coincidences
        //     because of point alignment, one or the other may include edges with the same 
        //     start and end points (a non-zero t range). These point edges should be zeroed
        //     but advance the edge lists unevenly. Each time through the loop, zero all
        //     point edges. Move nonzero pairs from opp to edge. Advance point edges, but
        //     only advance both if both are not points. Stop with both pairs are on final t.
#if OP_DEBUG
        const OpSegment* debugSeg = this;
        const OpSegment* debugOpp = oSegment;
        if (swapEdgeAndOpp)
            std::swap(debugSeg, debugOpp);
#endif
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
            if (!edgeDone && (edgeIsPoint || !oppIsPoint)) {
                OP_ASSERT(edge < &debugSeg->edges.back());
                ++edge;
            }
            if (!oppDone && (oppIsPoint || !edgeIsPoint)) {
                OP_ASSERT(&debugOpp->edges.front() <= oEdge + direction &&
                    oEdge + direction <= &debugOpp->edges.back());
                oEdge += direction;
            }
        } while (true);
        (*sectPtr)->zeroCoincidenceID();
        (*lastPtr)->zeroCoincidenceID();
    } while (++sectPtr != &intersections.back());
    return true;
}
#endif

#if 0   // moving points is evil. Don't do it
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
        if (SectFlavor::none != sect.flavor)
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
        OP_ASSERT(0);
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
#endif

void OpSegment::sortIntersections() {
    if (!resortIntersections) {
#if OP_DEBUG
        const OpIntersection* last = intersections.size() ? intersections.front() : nullptr;
        for (const auto sectPtr : intersections) {
            OP_ASSERT(last->ptT.t <= sectPtr->ptT.t);
            last = sectPtr;
        }
#endif
        return;
    }
    std::sort(intersections.begin(), intersections.end(),
        [](const OpIntersection* s1, const OpIntersection* s2) {
            return s1->ptT.t < s2->ptT.t 
        // use SectFlavor -1/0/1 value order to put unsectableStart at front; unsectableEnd at back
                    || (s1->ptT.t == s2->ptT.t && (int) s1->flavor < (int) s2->flavor); 
        });
    resortIntersections = false;
}

#if 0
// if there are winding changes, find the split t for the opposite edge
bool OpSegment::splitAtWinding(const std::vector<const OpEdge*>& windingChanges,
        const OpEdge* first, int direction
        OP_DEBUG_PARAMS(const OpIntersection* last, EdgeMatch oppositeMatch,
        const OpEdge* firstEdge, SectReason reason)) {
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
                    addIntersection(changePtT, SectFlavor::split
                            OP_DEBUG_PARAMS(SECT_MAKER(splitAtWinding), reason, last,
                            firstEdge, nullptr));
                }
            }
            if (++eChange > &windingChanges.back())
                goto done;  // only exit to outside loop
        } while (true);
        OP_ASSERT(split->ptT(oppositeMatch).t != last->ptT.t);
        split += direction;
    } while (true);
done:
    OP_ASSERT(eChange > &windingChanges.back());
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
                splits.emplace_back(edge, lastPtT, edge->end  
                        OP_DEBUG_PARAMS(EDGE_MAKER(resolveCoin1), nullptr, nullptr));
            ++edge;
            lastPtT = edge->start;
        } else {
            OP_ASSERT(splitPtT <= &splitTs.back());
            splits.emplace_back(edge, lastPtT, *splitPtT  
                    OP_DEBUG_PARAMS(EDGE_MAKER(resolveCoin2), nullptr, nullptr));
            lastPtT = *splitPtT++;
        }
    } while (edge <= &edges.back());
    std::swap(edges, splits);
    sortIntersections();
    return true;
}
#endif

#if 0
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
            OP_ASSERT(test.start.t == ptT.t || test.end.t == ptT.t);
            result = &test;
        }
    }
    for (auto sectPtr : intersections) {
        OpIntersection& sect = *sectPtr;
        if (sect.ptT.t < ptT.t)
            continue;  // !!! could binary search for 1st if intersection list is extremely long
        if (sect.ptT.t > ptT.t)
            break;
        OP_ASSERT(this != sect.opp->segment);
        OP_ASSERT(ptT.pt == sect.ptT.pt);
        OP_ASSERT(ptT.pt == sect.opp->ptT.pt);
        for (OpEdge& search : sect.opp->segment->edges) {
            if (search.start.t > sect.opp->ptT.t)
                break;
            if (search.end.t < sect.opp->ptT.t)
                continue;
            if (!search.winding.visible())
                continue;
            OP_ASSERT(&search != edge);
            if (result) // if there are two results, give up
                return nullptr;
            OP_ASSERT(search.start.t == sect.opp->ptT.t || search.end.t == sect.opp->ptT.t);
            result = &search;
        }
    }
    return result;
}
#endif

bool OpSegment::debugFail() const {
#if OP_DEBUG
    return contour->contours->debugFail();
#endif
    return false;
}

bool OpSegment::debugSuccess() const {
#if OP_DEBUG
    return contour->contours->debugSuccess();
#endif
    return true;
}
