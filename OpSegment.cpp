#include "OpContour.h"
#include "OpSegment.h"

OpSegment::OpSegment(const OpCurve& pts, OpType type, OpContour* contourPtr
        OP_DEBUG_PARAMS(SectReason startReason, SectReason endReason))
    : c(pts.pts, pts.weight, type)
    , winding(WindingUninitialized::dummy)
    , disabled(false)
    , recomputeBounds(false) 
    , resortIntersections(false) {
    complete(contourPtr);  // only for id, which could be debug code if id is not needed
    OP_DEBUG_CODE(contour = nullptr);   // can't set up here because it may still move
    OP_DEBUG_CODE(debugStart = startReason);
    OP_DEBUG_CODE(debugEnd = endReason);
    OP_DEBUG_CODE(debugZero = ZeroReason::uninitialized);
}

OpSegment::OpSegment(const LinePts& pts, OpContour* contourPtr
        OP_DEBUG_PARAMS(SectReason startReason, SectReason endReason))
    : c(&pts.pts.front(), OpType::line)
    , winding(WindingUninitialized::dummy)
    , disabled(false)
    , recomputeBounds(false) 
    , resortIntersections(false) {
    complete(contourPtr);  // only for id, which could be debug code if id is not needed
    OP_DEBUG_CODE(contour = nullptr);   // can't set up here because it may still move
    OP_DEBUG_CODE(debugStart = startReason);
    OP_DEBUG_CODE(debugEnd = endReason);
    OP_DEBUG_CODE(debugZero = ZeroReason::uninitialized);
}

// !!! optimization:  if called from opedge linkup, could abort if >1 active found
// if operand is diff or reverse diff, the subtracted path has an interior of zero
// if the operand is union or intersection, both paths have an interior of one
// Unsectable edges may or may not be able to have their wind zero side computed;
// for now, treat any unsectable multiple as having a zero side whether it does or not.
// returns true if emplaced edge has pals

// instead of returning more than one, it should return:
// 1 edge to match if there is only one
// there are more than one possible (even if only one is active) so none are returned
// rediscover why activeNeighbor is called separately
// document why (or why not) operator is immaterial
bool OpSegment::activeAtT(const OpEdge* edge, EdgeMatch match, std::vector<FoundEdge>& oppEdges) const {
    unsigned edgesSize = oppEdges.size();
    OP_ASSERT(!edge->disabled);
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
        auto skipCheck = [](const OpEdge* edge) {
            return edge->many.isSet() || edge->unsortable;
        };
        // flip zero if operator is diff/revdiff and edges are from both operands
        // if operator is xor, all zeroes match each other
        auto checkZero = [match](const OpEdge* edge, EdgeMatch eMatch) {
            WindZero zeroSide = edge->windZero;
            if (eMatch == match)
                WindZeroFlip(&zeroSide);
            return zeroSide;
        };
        const OpSegment* sectSeg = sect.opp->segment;
        OpEdge* start = sectSeg->findEnabled(oppPtT, EdgeMatch::start);  // !!! optimization: walk edges in order
        if (start && start != edge && (skipCheck(edge)
                || edge->windZero == checkZero(start, EdgeMatch::start) || skipCheck(start))) {
            if (!start->hasLinkTo(match))
                oppEdges.emplace_back(start, EdgeMatch::none);
            // disable assuming that these are reversed and are therefore disqualified
     //       else if (!start->hasLinkTo(Opposite(match)))
     //           oppEdges.emplace_back(start, EdgeMatch::end);
        }
        OpEdge* end = sectSeg->findEnabled(oppPtT, EdgeMatch::end);
        if (end && end != edge && (skipCheck(edge) 
                || edge->windZero == checkZero(end, EdgeMatch::end) || skipCheck(end))) {
            if (!end->hasLinkTo(match))
                oppEdges.emplace_back(end, EdgeMatch::none);
            // disable assuming that these are reversed and are therefore disqualified
   //         else if (!end->hasLinkTo(Opposite(match)))
   //             oppEdges.emplace_back(end, EdgeMatch::start);
        }
    }
    for (unsigned index = edgesSize; index < oppEdges.size(); ++index) {
        if (oppEdges[index].edge->pals.size())
            return true;
    }
    return false;
}

// returns true if emplaced edge has pals
bool OpSegment::activeNeighbor(const OpEdge* edge, EdgeMatch match, 
        std::vector<FoundEdge>& oppEdges) const {
    if ((EdgeMatch::start == match && edge->start.t == 0)
            || (EdgeMatch::end == match && edge->end.t == 1))
        return false;
    EdgeMatch neighbor = EdgeMatch::start == match ? Opposite(edge->whichEnd) : edge->whichEnd;
    OpPtT ptT = edge->whichPtT(match);
    OpEdge* nextDoor = findEnabled(ptT, neighbor);
    if (!nextDoor) 
       return false;
    for (auto& alreadyFound : oppEdges)
        if (alreadyFound.edge == nextDoor)
            return false;
    if (nextDoor->hasLinkTo(match))
        return false;
    auto skipCheck = [](const OpEdge* edge) { 
        return edge->many.isSet() || edge->unsortable;
    };
    if (skipCheck(edge) || edge->windZero == nextDoor->windZero || skipCheck(nextDoor)) {
        oppEdges.emplace_back(nextDoor, EdgeMatch::none);
        return !!nextDoor->pals.size();
    }
    return false;
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
    auto sect = contour->addSegSect(ptT, this, 0, 0  
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
    auto sect = contour->addSegSect(ptT, this, coinID, 0  
            OP_DEBUG_PARAMS(maker, line, file, reason, oSeg));
    intersections.push_back(sect);
    resortIntersections = true;
    return sect;
}

OpIntersection* OpSegment::alreadyContains(const OpPtT& ptT, const OpSegment* oppSegment) const {
	for (auto sectPtr : intersections) {
		const OpIntersection& sect = *sectPtr;
        if (!sect.opp)
            continue;
		if (oppSegment == sect.opp->segment && (ptT.pt == sect.ptT.pt || ptT.t == sect.ptT.t))
			return sectPtr;
	}
	return nullptr;
}

OpIntersection* OpSegment::addUnsectable(const OpPtT& ptT, int unsectableID, const OpSegment* oSeg 
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file)) {
    OpIntersection* sect = alreadyContains(ptT, oSeg);
    if (sect) {
        OP_ASSERT(!sect->unsectableID);
        sect->unsectableID = unsectableID;
    } else {
        sect = contour->addSegSect(ptT, this, 0, unsectableID  
                OP_DEBUG_PARAMS(maker, line, file, SectReason::curveCurveUnsectable, oSeg));
        intersections.push_back(sect);
        resortIntersections = true;
    }
    return sect;
}

void OpSegment::apply() {
    for (auto& edge : edges)
        edge.apply();
}

int OpSegment::coinID(bool flipped) const {
    int coinID = ++contour->contours->coincidenceID;
    return flipped ? -coinID : coinID;
}

void OpSegment::complete(OpContour* contourPtr) {
    ptBounds.set(c);
// #if OP_DEBUG     // used only by sort; probably unnecessary there
    id = contourPtr->contours->id++;  // contour pointer is not set up
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
OpEdge* OpSegment::findEnabled(OpPtT ptT, EdgeMatch match) const {
    for (auto& edge : edges) {
        if (ptT == (EdgeMatch::start == match ? edge.start : edge.end))
            return edge.disabled ? nullptr : const_cast<OpEdge*>(&edge);
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

std::vector<OpIntersection*> OpSegment::intersectRange(const OpSegment* opp) {
	if (resortIntersections)
		sortIntersections();
    OP_DEBUG_CODE(float last = -1);
    std::vector<OpIntersection*> result;
    for (auto sect : intersections) {
            if (sect->opp->segment == opp) {
                OP_ASSERT(last < sect->ptT.t);
                OP_DEBUG_CODE(last = sect->ptT.t);
                result.push_back(sect);
            }
        }
    return result;
}

void OpSegment::makeEdge(OP_DEBUG_CODE(EdgeMaker maker, int line, std::string file)) {
    if (!edges.size()) 
        edges.emplace_back(this, OpPtT(c.pts[0], 0), OpPtT(c.lastPt(), 1)
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
    std::vector<int> unsectables;
    const OpIntersection* last = intersections.front();
    OP_ASSERT(!resortIntersections);
    for (auto sectPtr : intersections) {
        const OpIntersection& sect = *sectPtr;
        if (sect.ptT.t != last->ptT.t) {
            edges.emplace_back(this, last->ptT, sect.ptT
                    OP_DEBUG_PARAMS(EDGE_MAKER(makeEdges), last, sectPtr));
            if (unsectables.size())
                edges.back().unsectableID = unsectables.back();
        }
        if (sect.unsectableID) {
            auto found = std::find(unsectables.begin(), unsectables.end(), sect.unsectableID);
            if (unsectables.end() == found)
                unsectables.push_back(sect.unsectableID);
            else
                unsectables.erase(found);
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
            return s1->ptT.t < s2->ptT.t; 
    // !!! don't think this is scalable if multiple edges are unsectable
        // use SectFlavor -1/0/1 value order to put unsectableStart at front; unsectableEnd at back
        //            || (s1->ptT.t == s2->ptT.t && (int) s1->flavor < (int) s2->flavor); 
        });
    resortIntersections = false;
}

int OpSegment::unsectableID() const {
    return ++contour->contours->unsectableID;
}

struct CoinPair {
    CoinPair(OpEdge* o, int id, EdgeMatch m  OP_DEBUG_PARAMS(std::vector<OpEdge>* oE))
        : opp(o)
        , coinID(id) 
        , match(m)
        OP_DEBUG_PARAMS(oppEdges(oE)) {
    }
    OpEdge* opp;
    int coinID;
    EdgeMatch match;
    OP_DEBUG_CODE(std::vector<OpEdge>* oppEdges);
};

// at present, only applies to horizontal and vertical lines
void OpSegment::windCoincidences() {
    if (OpType::line != c.type)
        return;
    if (disabled)
        return;
    OpVector tangent = c.asLine().tangent();
    if (tangent.dx && tangent.dy)
        return;
    OP_ASSERT(tangent.dx || tangent.dy);
    OP_ASSERT(!resortIntersections);
    std::vector<CoinPair> coinPairs;
    OpEdge* edge = &edges.front();
    for (auto sectPtr : intersections) {
        if (1 == sectPtr->ptT.t)
            break;
        if (edge->start.t != sectPtr->ptT.t) {
            OP_ASSERT(edge->start.t < sectPtr->ptT.t);
            OP_ASSERT(edge < &edges.back());
            ++edge;
            OP_ASSERT(edge->start.t == sectPtr->ptT.t);
        }
        int coinID = sectPtr->coincidenceID;
        auto pairID = [coinID](CoinPair& pair) { 
            return coinID == pair.coinID; 
        };
        if (coinID) {
            auto coinIter = std::find_if(coinPairs.begin(), coinPairs.end(), pairID);
            if (coinPairs.end() != coinIter) {
                coinPairs.erase(coinIter);
                coinID = 0;
            }
        }
        if (!coinID && coinPairs.empty())
            continue;
        if (edge->disabled) {
            if (coinID)
                coinPairs.emplace_back(nullptr, coinID, EdgeMatch::none
                        OP_DEBUG_PARAMS(nullptr));
            continue;
        }
        if (coinID) {
            OpSegment* oppSegment = sectPtr->opp->segment;
            OP_ASSERT(tangent.dot(oppSegment->c.asLine().tangent())); 
            auto& oppEdges = sectPtr->opp->segment->edges;
            OpEdge* oppEdge = &oppEdges.front();
            EdgeMatch match = coinID > 0 ? EdgeMatch::start : EdgeMatch::end;
            while (oppEdge->segment != oppSegment || oppEdge->ptT(match) != sectPtr->opp->ptT) {
                OP_ASSERT(EdgeMatch::end == match || oppEdge->ptT(match).t <= sectPtr->ptT.t);
                OP_ASSERT(EdgeMatch::start == match || oppEdge->ptT(match).t >= sectPtr->ptT.t);
                OP_ASSERT(oppEdge < &oppEdges.back());
                ++oppEdge;
            }
            coinPairs.emplace_back(oppEdge, coinID, match  OP_DEBUG_PARAMS(&oppEdges));
        }
        for (auto& coinPair : coinPairs) {
            if (!coinPair.opp)
                continue;
            if (coinPair.opp->ptT(coinPair.match).pt != sectPtr->ptT.pt) {
#if OP_DEBUG
                if (EdgeMatch::start == coinPair.match) {
                    OP_ASSERT(coinPair.opp > &coinPair.oppEdges->front());
                    OP_ASSERT((coinPair.opp - 1)->ptT(coinPair.match).pt == sectPtr->ptT.pt);
                } else {
                    OP_ASSERT(coinPair.opp < &coinPair.oppEdges->back());
                    OP_ASSERT((coinPair.opp + 1)->ptT(coinPair.match).pt == sectPtr->ptT.pt);
                }
#endif
                continue;
            }
            OP_ASSERT(edge->start.pt == coinPair.opp->ptT(coinPair.match).pt);
            OP_ASSERT(edge->end.pt == coinPair.opp->ptT(Opposite(coinPair.match)).pt);
            edge->winding.move(coinPair.opp->winding, contour->contours, coinPair.coinID < 0);
            coinPair.opp->setDisabled(OP_DEBUG_CODE(ZeroReason::hvCoincidence));
            if (coinPair.coinID > 0) {
                if (coinPair.opp < &coinPair.oppEdges->back())
                    ++coinPair.opp;
                else
                    coinPair.opp = nullptr;
            } else {
                if (coinPair.opp > &coinPair.oppEdges->front())
                    --coinPair.opp;
                else
                    coinPair.opp = nullptr;
            }
        }
    }
}

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
