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
    , disabled(false)  {
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
    , disabled(false)  {
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
                zeroSide = !zeroSide;
            return zeroSide;
        };
        auto saveMatch = [edge, &oppEdges, &oSect, checkZero](EdgeMatch testEnd) {
            OpSegment* oSeg = oSect->segment;
            OpEdge* test = oSeg->findEnabled(oSect->ptT, testEnd);  // !!! optimization: walk edges in order
            bool result = false;
            if (test && test != edge && (edge->unsortable || test->unsortable
                    || edge->windZero == checkZero(test, edge->which(), testEnd))) {
                result = test->hasLinkTo(testEnd);
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
    EdgeMatch neighbor = EdgeMatch::start == match ? !edge->which() : edge->which();
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
        OP_LINE_FILE_DEF(SectReason reason,  const OpEdge* debugEdge, const OpEdge* debugOpp)) {
    OP_ASSERT(!sects.debugContains(ptT, debugOpp->segment));
    return sects.add(contour->addEdgeSect(ptT, this  
            OP_LINE_FILE_CALLER(reason, debugEdge, debugOpp)));
}

OpIntersection* OpSegment::addSegBase(const OpPtT& ptT  
        OP_LINE_FILE_DEF(SectReason reason, const OpSegment* oSeg)) {
    OP_ASSERT(!sects.debugContains(ptT, oSeg));
    return sects.add(contour->addSegSect(ptT, this  OP_LINE_FILE_CALLER(reason, oSeg)));
}

OpIntersection* OpSegment::addSegSect(const OpPtT& ptT, const OpSegment* oSeg    
        OP_LINE_FILE_DEF(SectReason reason)) {
    if (sects.contains(ptT, oSeg))
        return nullptr;
    return addSegBase(ptT  OP_LINE_FILE_CALLER(reason, oSeg));
}

OpIntersection* OpSegment::addCoin(const OpPtT& ptT, int coinID, MatchEnds coinEnd, 
        const OpSegment* oSeg  OP_LINE_FILE_DEF(SectReason reason)) {
    if (sects.contains(ptT, oSeg))  // triggered by fuzz763_13
        return nullptr;
    return sects.add(contour->addCoinSect(ptT, this, coinID, coinEnd  
            OP_LINE_FILE_CALLER(reason, oSeg)));
}

OpIntersection* OpSegment::addUnsectable(const OpPtT& ptT, int usectID, MatchEnds end,
        const OpSegment* oSeg    OP_LINE_FILE_DEF(SectReason reason)) {
    OpIntersection* sect = sects.contains(ptT, oSeg);
    if (sect) {
        OP_ASSERT(!sect->unsectID);
        sect->unsectID = usectID;
        sect->unsectEnd = end;
        sects.resort = true;
        return sect;
    }
    return sects.add(contour->addUnsect(ptT, this, usectID, end  
            OP_LINE_FILE_CALLER(reason, oSeg)));
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
    setBounds();
// #if OP_DEBUG     // !!! used only by sort; probably unnecessary?
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

// used to find unsectable range; assumes range all has about the same slope
// !!! this may be a bad idea if two near coincident edges turn near 90 degrees
float OpSegment::findAxisT(Axis axis, float start, float end, float opp) const {
    if (OpType::line != c.type) {
        OpRoots roots = c.axisRayHit(axis, opp, start, end);
        if (1 == roots.count)
            return roots.roots[0];
    } else {
        float pt0xy = c.pts[0].choice(axis);
        float result = (opp - pt0xy) / (c.pts[1].choice(axis) - pt0xy);
        if (start <= result && result <= end)
            return result;
    }
    return OpNaN;
}

#if 0  // unused
// returns t closest to start, end
// !!! curious: why doesn't this call start.pt.nearly(... ?
float OpSegment::findNearbyT(const OpPtT& start, const OpPtT& end, OpPoint opp) const {
    if (start.pt == opp)
        return start.t;
    if (end.pt == opp)
        return end.t;
    return findValidT(start.t, end.t, opp);
}
#endif

// returns t iff opp point is between start and end
// start/end range is necessary since cubics can have more than one t at a point
float OpSegment::findValidT(float start, float end, OpPoint opp) const {
    if (OpType::line != c.type) {
        OpRoots hRoots = c.axisRayHit(Axis::horizontal, opp.y, start, end);
        OpRoots vRoots = c.axisRayHit(Axis::vertical, opp.x, start, end);
        if (1 != hRoots.count && 1 != vRoots.count) {
            if (0 == start && opp.isNearly(c.pts[0]))
                return 0;
            if (1 == end && opp.isNearly(c.lastPt()))
                return 1;
            return OpNaN;
        }
        if (1 != hRoots.count) {
            OP_ASSERT(1 == vRoots.count);  // !!! triggered by thread_loops46134
            return vRoots.roots[0];
        }
        if (1 != vRoots.count)
            return hRoots.roots[0];
        OpPoint hPt = c.ptAtT(hRoots.roots[0]);
        OpPoint vPt = c.ptAtT(vRoots.roots[0]);
        return (hPt - opp).lengthSquared() < (vPt - opp).lengthSquared() 
                ? hRoots.roots[0] : vRoots.roots[0];
    }
    // this won't work for curves with linear control points since t is not necessarily linear
    OpVector lineSize = c.pts[1] - c.pts[0];
    float result = fabsf(lineSize.dy) > fabsf(lineSize.dx) ?
        (opp.y - c.pts[0].y) / lineSize.dy : (opp.x - c.pts[0].x) / lineSize.dx;
    if (start <= result && result <= end)
        return result;
    return OpNaN;
}

void OpSegment::makeEdge(const OpPtT& s, const OpPtT& e  
        OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file)) {
    if (!edges.size()) 
        edges.emplace_back(this, s, e  OP_DEBUG_PARAMS(maker, line, file, nullptr, nullptr));
}


void OpSegment::makeEdge(OP_DEBUG_CODE(EdgeMaker maker, int line, std::string file)) {
    makeEdge(OpPtT(c.pts[0], 0), OpPtT(c.lastPt(), 1)  OP_DEBUG_PARAMS(maker, line, file));
}

void OpSegment::makeEdges() {
    if (!winding.left() && !winding.right()) {
       edges.clear();
       return;
    }
    if (2 == sects.i.size() && 1 == edges.size() && !edges[0].start.t && 1 == edges[0].end.t)
        return;
    edges.clear();
    edges.reserve(sects.i.size() - 1);
    sects.makeEdges(this);
}

MatchReverse OpSegment::matchEnds(const OpSegment* opp) const {
    MatchReverse result { MatchEnds::none, false };
    OP_ASSERT(c.pts[0] != c.lastPt());
    OP_ASSERT(opp->c.pts[0] != opp->c.lastPt());
    if (c.pts[0] == opp->c.pts[0])
        result = { MatchEnds::start, false };
    else if (c.pts[0] == opp->c.lastPt())
        result = { MatchEnds::start, true };
    if (c.lastPt() == opp->c.lastPt())
        result = { result.match | MatchEnds::end, false };
    else if (c.lastPt() == opp->c.pts[0])
        result = { result.match | MatchEnds::end, true };
    return result;
}

/* Since all segments have to be checked against all other segments, it adds complexity
   to special case consecutive segments (or start/end of contour segments) that share
   a point. Instead, pick up common end point when the consecutive segments are compared.
MatchEnds OpSegment::matchExisting(const OpSegment* opp) const {
    MatchEnds result = MatchEnds::none;
    if (contour != opp->contour)  //  || 2 == contour->segments.size() : curve/line e.g. cubicOp97x
        return result;
    if (this - 1 == opp || (&contour->segments.front() == this && &contour->segments.back() == opp))
        result = MatchEnds::start;
    if (this + 1 == opp || (&contour->segments.front() == opp && &contour->segments.back() == this))
        result |= MatchEnds::end;
    return result;
}
*/

// keep control point inside curve bounds
// further, if old control point is axis aligned with end point, keep relationship after moving
// !!! detect if segment collapses to point?
// !!! don't change opposite end point
void OpSegment::moveTo(float matchT, OpPoint equalPt) {
    OP_ASSERT(0 == matchT || 1 == matchT);
    OpPoint& endPt = 0 == matchT ? c.pts[0] : c.pts[c.pointCount() - 1];
    OP_ASSERT(endPt.isNearly(equalPt));
    if (c.pts[0] == c.lastPt())
        disabled = true;
    endPt = equalPt;
    c.pinCtrl();
    setBounds();
    for (OpIntersection* sect : sects.i) {
        if (sect->ptT.t == matchT) {
            sect->ptT.pt = equalPt;
            if (sect->opp->ptT.pt != equalPt)
                sect->opp->segment->moveTo(sect->opp->ptT.t, equalPt);
        }
    }
    edges.clear();
}

int OpSegment::nextID(OpContour* contourPtr) const {
    return contourPtr->nextID();
}

void OpSegment::setBounds() {
    ptBounds.set(c);
    closeBounds = ptBounds;
    closeBounds.outsetClose();
}

// should be inlined. Out of line for ease of setting debugging breakpoints
void OpSegment::setDisabled(OP_DEBUG_CODE(ZeroReason reason)) {
	disabled = true; 
    OP_DEBUG_CODE(debugZero = reason); 
}

// at present, only applies to horizontal and vertical lines
// !!! experiment: add support for linear diagonals
void OpSegment::windCoincidences() {
    if (OpType::line != c.type)
        return;
    if (disabled)
        return;
    OpVector tangent = c.asLine().tangent();
    if (tangent.dx && tangent.dy) {
        // iterate through edges; if edge is linear and matches opposite, mark both coincident
        bool foundCoincidence  = false;
        for (OpEdge& edge : edges) {
            if (!edge.isLinear())
                continue;
            // iterate through sects that match edge start and end, looking for parallel edges
            OpIntersection** firstBegin = nullptr;
            OpIntersection** firstEnd = nullptr;  // one after last start
            OpIntersection** lastBegin = nullptr;
            OpIntersection** lastEnd = nullptr;  // one after last end
            for (OpIntersection** sPtr = &sects.i.front(); sPtr <= &sects.i.back(); ++sPtr) {
                OpIntersection* sect = *sPtr;
                if (!firstBegin) {
                    if (sect->ptT == edge.start)
                        firstBegin = sPtr;
                    continue;
                }
                if (!firstEnd) {
                    if (sect->ptT != edge.start)
                        firstEnd = sPtr;
                }
                if (!lastBegin) {
                    if (sect->ptT == edge.end)
                        lastBegin = sPtr;
                    continue;
                }
                if (!lastEnd) {
                    if (sect->ptT != edge.end)
                        lastEnd = sPtr;
                    break;
                }
            }
            // look for an end with a matching opposite intersection
            for (OpIntersection** firstTest = firstBegin; firstTest < firstEnd; ++firstTest) {
                OpIntersection* firstOSect = (*firstTest)->opp;
                OpSegment* oSegment = firstOSect->segment;
                for (OpIntersection** lastTest = lastBegin; lastTest < lastEnd; ++lastTest) {
                    OpIntersection* lastOSect = (*lastTest)->opp;
                    if (lastOSect->segment != oSegment)
                        continue;
                    // see if the opp corresponds to a linear edge
                    OpIntersection* oStart = firstOSect;
                    OpIntersection* oEnd = lastOSect;
                    bool reversed = oStart->ptT.t > oEnd->ptT.t;
                    if (reversed)
                        std::swap(oStart, oEnd);
                    OpEdge* oppEdge = oSegment->findEnabled(oStart->ptT, EdgeMatch::start);
                    if (!oppEdge)
                        break;
                    if (!oppEdge->isLinear())
                        break;
                    // verify that all four intersections are not used to mark coincidence
                    if (oStart->coincidenceID)
                        return;
                    if (oEnd->coincidenceID)
                        return;
                    OpIntersection* const* eStart = oSegment->sects.entry(oppEdge->start, this);
                    if (!eStart || (*eStart)->opp->coincidenceID)
                        return;
                    OpIntersection* const* eEnd = oSegment->sects.entry(oppEdge->end, this);
                    if (!eEnd || (*eEnd)->opp->coincidenceID)
                        return;
                    // mark the intersections as coincident
                    int coinID = oSegment->coinID(reversed);
                    oStart->coincidenceID = coinID;
                    OP_ASSERT(MatchEnds::none == oStart->coinEnd);
                    oStart->coinEnd = MatchEnds::start;
                    oEnd->coincidenceID = coinID;
                    OP_ASSERT(MatchEnds::none == oEnd->coinEnd);
                    oEnd->coinEnd = MatchEnds::end;
                    (*eStart)->opp->coincidenceID = coinID;
                    OP_ASSERT(MatchEnds::none == (*eStart)->opp->coinEnd);
                    (*eStart)->opp->coinEnd = reversed ? MatchEnds::end : MatchEnds::start;
                    (*eEnd)->opp->coincidenceID = coinID;
                    OP_ASSERT(MatchEnds::none == (*eEnd)->opp->coinEnd);
                    (*eEnd)->opp->coinEnd = reversed ? MatchEnds::start : MatchEnds::end;
                    sects.resort = true;
                    foundCoincidence = true;
                }
            }
        }
        if (foundCoincidence)
            sects.windCoincidences(edges  OP_DEBUG_PARAMS(tangent));
        return;
    }
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


