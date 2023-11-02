// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpSegment.h"

void FoundEdge::check(std::vector<FoundEdge>* edges, OpEdge* test, EdgeMatch em, OpPoint match) {
    if (edges && edges->size())
        return;
    float gapSq = (test->whichPtT(em).pt - match).lengthSquared();
	if (distSq > gapSq) {
		distSq = gapSq;
		edge = test;
        whichEnd = em;
	}
}

void FoundEdge::reset() {
    edge = nullptr;
    perimeter = OpInfinity;
    closeSq = OpInfinity;
    distSq = OpInfinity;
    index = -1;
    whichEnd = EdgeMatch::none;
    chop = ChopUnsortable::none;
    connects = false;
    loops = false;
}

OpSegment::OpSegment(const OpCurve& pts, OpType type, OpContour* contourPtr
        OP_DEBUG_PARAMS(SectReason startReason, SectReason endReason))
    : c(pts.pts, pts.weight, type)
    , winding(WindingUninitialized::dummy)
    , disabled(false)
    , recomputeBounds(false) {
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
    , recomputeBounds(false) {
    complete(contourPtr);  // only for id, which could be debug code if id is not needed
    OP_DEBUG_CODE(contour = nullptr);   // can't set up here because it may still move
    OP_DEBUG_CODE(debugStart = startReason);
    OP_DEBUG_CODE(debugEnd = endReason);
    OP_DEBUG_CODE(debugZero = ZeroReason::uninitialized);
}

// !!! optimization:  if called from opedge linkup, could abort if >1 active found?

// Unsectable edges may or may not be able to have their wind zero side computed;
// for now, treat any unsectable multiple as having a zero side whether it does or not.
// returns true if emplaced edge has pals

// activeNeighbor is called separately because this iterates through opposite intersections only
bool OpSegment::activeAtT(const OpEdge* edge, EdgeMatch match, std::vector<FoundEdge>& oppEdges,
        bool* hadLinkTo) const {
    unsigned edgesSize = oppEdges.size();
    OP_ASSERT(!edge->disabled);
    // each prospective match normal must agree with edge, indicating direction of area outside fill
    // if number of matching sects doesn't agree with opposite, collect next indirection as well
    OpPtT ptT = edge->whichPtT(match);
    *hadLinkTo = false;
    for (auto sectPtr : sects.i) {
        OpIntersection& sect = *sectPtr;
        if (sect.ptT.t < ptT.t)
            continue;  // !!! could binary search for 1st if intersection list is extremely long
        if (sect.ptT.t > ptT.t)
            break;
        OpIntersection* oSect = sect.opp;
        if (ptT.pt != oSect->ptT.pt)
            continue;
        // op operator is not needed since zero side was computed by apply
        auto checkZero = [match](const OpEdge* test, EdgeMatch eWhich, EdgeMatch testEnd) {
            WindZero zeroSide = test->windZero;
            EdgeMatch which = eWhich == testEnd ? EdgeMatch::start : EdgeMatch::end;
            if (which == match)
                WindZeroFlip(&zeroSide);
            return zeroSide;
        };
        auto saveMatch = [edge, match, &oppEdges, &oSect, checkZero](EdgeMatch testEnd) {
            OpSegment* oSeg = oSect->segment;
            OpEdge* test = oSeg->findEnabled(oSect->ptT, testEnd);  // !!! optimization: walk edges in order
            bool result = false;
            if (test && test != edge && (edge->unsortable || test->unsortable
                    || edge->windZero == checkZero(test, edge->whichEnd, testEnd))) {
                result = test->hasLinkTo(match);
                if (!result && !test->isPal(edge))
                    oppEdges.emplace_back(test, EdgeMatch::none);
            }
            return result;
        };
        *hadLinkTo |= saveMatch(EdgeMatch::start);
        *hadLinkTo |= saveMatch(EdgeMatch::end);
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
    if (edge->unsortable || edge->windZero == nextDoor->windZero || nextDoor->unsortable) {
        oppEdges.emplace_back(nextDoor, EdgeMatch::none);
        return !!nextDoor->pals.size();
    }
    return false;
}

OpIntersection* OpSegment::addEdgeSect(const OpPtT& ptT  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason,
        const OpEdge* debugEdge, const OpEdge* debugOpp)) {
    OP_ASSERT(!sects.debugContains(ptT, debugOpp->segment));
    return sects.add(contour->addEdgeSect(ptT, this  
            OP_DEBUG_PARAMS(maker, line, file, reason, debugEdge, debugOpp)));
}

OpIntersection* OpSegment::addSegBase(const OpPtT& ptT  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
        const OpSegment* oSeg)) {
    OP_ASSERT(!sects.debugContains(ptT, oSeg));
    return sects.add(contour->addSegSect(ptT, this 
            OP_DEBUG_PARAMS(maker, line, file, reason, oSeg)));
}

OpIntersection* OpSegment::addSegSect(const OpPtT& ptT, const OpSegment* oSeg  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason)) {
    if (sects.contains(ptT, oSeg))
        return nullptr;
    return addSegBase(ptT  OP_DEBUG_PARAMS(maker, line, file, reason, oSeg));
}

OpIntersection* OpSegment::addCoin(const OpPtT& ptT, int coinID, MatchEnds coinEnd, 
        const OpSegment* oSeg  
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason)) {
    if (sects.contains(ptT, oSeg))  // triggered by fuzz763_13
        return nullptr;
    return sects.add(contour->addCoinSect(ptT, this, coinID, coinEnd  
            OP_DEBUG_PARAMS(maker, line, file, reason, oSeg)));
}

OpIntersection* OpSegment::addUnsectable(const OpPtT& ptT, int usectID, MatchEnds end,
        const OpSegment* oSeg  OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file)) {
    OpIntersection* sect = sects.contains(ptT, oSeg);
    if (sect) {
        OP_ASSERT(!sect->unsectID);
        sect->unsectID = usectID;
        sect->unsectEnd = end;
        return sect;
    }
    return sects.add(contour->addUnsect(ptT, this, usectID, end  
            OP_DEBUG_PARAMS(maker, line, file, SectReason::curveCurveUnsectable, oSeg)));
}

void OpSegment::apply() {
    for (auto& edge : edges)
        edge.apply();
}

int OpSegment::coinID(bool flipped) const {
    int coinID = nextID();
    return flipped ? -coinID : coinID;
}

void OpSegment::complete(OpContour* contourPtr) {
    ptBounds.set(c);
// #if OP_DEBUG     // used only by sort; probably unnecessary there
    id = nextID(contourPtr);  // segment's contour pointer is not set up
// #endif
}

// !!! would it be any better (faster) to split this into findStart / findEnd instead?
OpEdge* OpSegment::findEnabled(const OpPtT& ptT, EdgeMatch match) const {
    for (auto& edge : edges) {
        if (ptT == edge.ptT(match))
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

void OpSegment::makeEdge(OP_DEBUG_CODE(EdgeMaker maker, int line, std::string file)) {
    if (!edges.size()) 
        edges.emplace_back(this, OpPtT(c.pts[0], 0), OpPtT(c.lastPt(), 1)
                OP_DEBUG_PARAMS(maker, line, file, nullptr, nullptr));
}

void OpSegment::makeEdges() {
    if (!winding.left() && !winding.right()) {
        edges.clear();
        return;
    }
    if (1 == edges.size() && 2 == sects.i.size())
        return;
    edges.clear();
    edges.reserve(sects.i.size() - 1);
    sects.makeEdges(this);
}

MatchEnds OpSegment::matchEnds(const OpSegment* opp, bool* reversed, MatchEnds* existing,
        MatchSect matchSect) const {
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
        result = (MatchEnds) ((int) result | (int) MatchEnds::end);
        OP_ASSERT(MatchEnds::end == result || MatchEnds::both == result);
    }
    if (existing)
        *existing = result;
    if (MatchEnds::none == result || MatchEnds::both == result)
        return result;
    if (MatchSect::existing != matchSect || !*reversed)
        return result;
    if (result != matchExisting(opp))
        return result;
    return MatchEnds::none;
}

MatchEnds OpSegment::matchExisting(const OpSegment* opp) const {
    MatchEnds result = MatchEnds::none;
    if (contour != opp->contour)  //  || 2 == contour->segments.size() : curve/line e.g. cubicOp97x
        return result;
    if (this - 1 == opp || (&contour->segments.front() == this && &contour->segments.back() == opp))
        result = MatchEnds::start;
    if (this + 1 == opp || (&contour->segments.front() == opp && &contour->segments.back() == this))
        result = (MatchEnds) ((int) result | (int) MatchEnds::end);
    return result;
}

int OpSegment::nextID(OpContour* contourPtr) const {
    return contourPtr->nextID();
}

// should be inlined. Out of line for ease of setting debugging breakpoints
void OpSegment::setDisabled(OP_DEBUG_CODE(ZeroReason reason)) {
	disabled = true; 
    OP_DEBUG_CODE(debugZero = reason); 
}

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
    sects.windCoincidences(edges  OP_DEBUG_PARAMS(tangent));
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


