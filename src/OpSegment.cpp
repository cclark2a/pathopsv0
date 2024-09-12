// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurve.h"
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

OpSegment::OpSegment(PathOpsV0Lib::AddCurve addCurve, PathOpsV0Lib::AddWinding addWinding)    
    : contour((OpContour*) addWinding.contour)
    , c(contour->contours,  
            { (PathOpsV0Lib::CurveData*) addCurve.points, addCurve.size, addCurve.type } )
    , winding(contour, { (PathOpsV0Lib::WindingData*) addWinding.windings, addWinding.size } )
    , id(contour->nextID())
    , disabled(false)
    , hasCoin(false)
    , hasUnsectable(false) {
    setBounds();
    OP_DEBUG_IMAGE_CODE(debugColor = black);
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
            if (test && test != edge && (edge->isUnsortable || test->isUnsortable
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
    if ((EdgeMatch::start == match && edge->startT == 0)
            || (EdgeMatch::end == match && edge->endT == 1))
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
    if (edge->isUnsortable || edge->windZero == nextDoor->windZero || nextDoor->isUnsortable) {
        oppEdges.emplace_back(nextDoor, EdgeMatch::none);
        return !!nextDoor->pals.size();
    }
    return false;
}

OpIntersection* OpSegment::addEdgeSect(const OpPtT& ptT  
        OP_LINE_FILE_DEF(const OpEdge* debugEdge, const OpEdge* debugOpp)) {
    OP_ASSERT(!sects.debugContains(ptT, debugOpp->segment));
    return sects.add(contour->addEdgeSect(ptT, this  
            OP_LINE_FILE_CALLER(debugEdge, debugOpp)));
}

OpIntersection* OpSegment::addSegBase(const OpPtT& ptT  
        OP_LINE_FILE_DEF(const OpSegment* oSeg)) {
    OP_ASSERT(!sects.debugContains(ptT, oSeg));
    return sects.add(contour->addSegSect(ptT, this  OP_LINE_FILE_CALLER(oSeg)));
}

OpIntersection* OpSegment::addSegSect(const OpPtT& ptT, const OpSegment* oSeg    
        OP_LINE_FILE_DEF()) {
    if (sects.contains(ptT, oSeg))
        return nullptr;
    return addSegBase(ptT  OP_LINE_FILE_CALLER(oSeg));
}

OpIntersection* OpSegment::addCoin(const OpPtT& ptT, int coinID, MatchEnds coinEnd, 
        const OpSegment* oSeg  OP_LINE_FILE_DEF()) {
    if (sects.contains(ptT, oSeg))  // triggered by fuzz763_13
        return nullptr;
    return sects.add(contour->addCoinSect(ptT, this, coinID, coinEnd  
            OP_LINE_FILE_CALLER(oSeg)));
}

OpIntersection* OpSegment::addUnsectable(const OpPtT& ptT, int usectID, MatchEnds end,
        const OpSegment* oSeg    OP_LINE_FILE_DEF()) {
    // !!! replace with assert to disallow contains here
    // rework caller to do contains in pairs prior to calling add
    OpIntersection* sect = sects.contains(ptT, oSeg);
    if (sect) {
        OP_ASSERT(!sect->unsectID);
        sect->setUnsect(usectID, end);
        sects.resort = true;
        return sect;
    }
    return sects.add(contour->addUnsect(ptT, this, usectID, end  OP_LINE_FILE_CALLER(oSeg)));
}

void OpSegment::apply() {
    for (auto& edge : edges)
        edge.apply();
}

        // walk sects in coincidences;
        // look at opp sects (reversed if necessary)
        // if opp sects is missing this point, add it
void OpSegment::betweenIntersections() {
    if (!hasCoin)
        return;
    // if this segment has coincident runs with three or more edges, make the sects consistent
    std::vector<OpIntersection*> coincidences;
    std::vector<OpIntersection*> betweens;  // !!! this needs to be in sects (i.e., in opp segment)
    // A and B are coincident; C is also coincident with B, but C-A coin may have been missed
    auto checkOpp = [coincidences, betweens](OpIntersection* sectC) {
        for (OpIntersection* sectB : coincidences) {
            OpSegment* segC = sectC->segment;
            OP_ASSERT(segC != sectB->segment);
            OpIntersection* sectA = sectB->opp;
            OpSegment* segA = sectA->segment;
            OP_ASSERT(segC != segA);
            // check A for C coincidence
            OpIntersections& aSects = sectA->segment->sects;
            if (aSects.i.end() != std::find_if(aSects.i.begin(), aSects.i.end(), 
                    [segC](const OpIntersection* test) { return test->opp->segment == segC; }))
                continue;
            segA->sects.resort = true;
            segC->sects.resort = true;
            // Construct new coincidence between A and C. Add the shorter of a end and c end.
            OpIntersection* cEndSect = sectC->coinOtherEnd();
            OpIntersection* aEndSect = sectA->coinOtherEnd();
            // both aEnd and cEnd should link to B, but not to each other
            OP_ASSERT(aEndSect->opp->segment == sectB->segment);
            OP_ASSERT(cEndSect->opp->segment == sectB->segment);
            int acID = segC->nextID();
            MatchEnds cMatch = sectC->coincidenceID < 0 ? MatchEnds::end : MatchEnds::start;
            OpIntersection* cS = segC->addCoin(sectC->ptT, acID, cMatch, segA  OP_LINE_FILE_PARAMS());
            OpPtT aStart { sectC->ptT.pt, segA->c.match(0, 1, sectC->ptT.pt) };
            MatchEnds aMatch = sectA->coincidenceID < 0 ? MatchEnds::end : MatchEnds::start;
            OpIntersection* aS = segA->addCoin(aStart, acID, aMatch, segC  OP_LINE_FILE_PARAMS());
            cS->pair(aS);
            bool shortA = aEndSect->opp->ptT.t < cEndSect->opp->ptT.t; 
            OpPtT cEndPtT = shortA ? aEndSect->ptT : cEndSect->ptT;
            OpPtT aEndPtT = cEndPtT;
            if (shortA)
                cEndPtT.t = segC->c.match(0, 1, cEndPtT.pt);
            else
                aEndPtT.t = segA->c.match(0, 1, aEndPtT.pt);
            OpIntersection* cE = segC->addCoin(cEndPtT, acID, !cMatch, segA  OP_LINE_FILE_PARAMS());
            OpIntersection* aE = segA->addCoin(aEndPtT, acID, !aMatch, segC  OP_LINE_FILE_PARAMS());
            cE->pair(aE);
        }
        // check to see if added point collapses through identical t? or adjacent has idential pt?
        // !!! probably should put that check in sorting
    };
    if (sects.resort)
        sects.sort();
    for (OpIntersection* sect : sects.i) {
        OP_ASSERT(sect->segment == this);
        if (!sect->coincidenceID)
                continue;
        if (MatchEnds::start == sect->coinEnd) {
            if (coincidences.size())
                checkOpp(sect);
            coincidences.push_back(sect);
            continue;
        }
        OP_ASSERT(MatchEnds::end == sect->coinEnd);
        auto found = std::find_if(coincidences.begin(), coincidences.end(), [sect]
                (const OpIntersection* cT) { return cT->coincidenceID == sect->coincidenceID; });
        OP_ASSERT(coincidences.end() != found);
        coincidences.erase(found);
    }
}

int OpSegment::coinID(bool flipped) {
    int coinID = nextID();
    hasCoin = true;
    return flipped ? -coinID : coinID;
}

#if 0
void OpSegment::complete() {
    setBounds();
    id = nextID();
}
#endif

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
float OpSegment::findAxisT(Axis axis, float start, float end, float opp) {
    if (!c.isLine()) {
        OpRoots roots = c.axisRayHit(axis, opp, start, end);
        if (1 == roots.count)
            return roots.roots[0];
    } else {
        float pt0xy = c.firstPt().choice(axis);
        float result = (opp - pt0xy) / (c.lastPt().choice(axis) - pt0xy);
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
float OpSegment::findValidT(float start, float end, OpPoint opp) {
    if (!c.isLine()) {
        OpRoots hRoots = c.axisRayHit(Axis::horizontal, opp.y, start, end);
        OpRoots vRoots = c.axisRayHit(Axis::vertical, opp.x, start, end);
        if (1 != hRoots.count && 1 != vRoots.count) {
            if (0 == start && opp.isNearly(c.firstPt()))
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
    OpVector lineSize = c.lastPt() - c.firstPt();
    float result = fabsf(lineSize.dy) > fabsf(lineSize.dx) ?
        (opp.y - c.firstPt().y) / lineSize.dy : (opp.x - c.firstPt().x) / lineSize.dx;
    if (start <= result && result <= end)
        return result;
    return OpNaN;
}

void OpSegment::makeEdge(OP_LINE_FILE_NP_DEF()) {
    if (!edges.size()) 
        edges.emplace_back(this  OP_LINE_FILE_PARAMS());
}

void OpSegment::makeEdges() {
    if (disabled) {
       edges.clear();
       return;
    }
#if 0  // !!! this fails if intersection is unsectable; checking is probably not worth it ?
       //     need to measure how often edge already is exists to justify more extensive check
       //     does this need to check for other things? zero winding, coincidence, disabled, ?
    if (2 == sects.i.size() && 1 == edges.size() && !edges[0].start.t && 1 == edges[0].end.t)
        return;
#endif
    edges.clear();
    edges.reserve(sects.i.size() - 1);
    sects.makeEdges(this);
}

MatchReverse OpSegment::matchEnds(const LinePts& linePts) const {
    if (disabled)
        return { MatchEnds::none, false };
    return c.matchEnds(linePts);
}

MatchReverse OpSegment::matchEnds(const OpSegment* opp) const {
    if (opp->disabled)
        return { MatchEnds::none, false };
    LinePts oppLine { opp->c.firstPt(), opp->c.lastPt() };
    return matchEnds(oppLine);
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
    OpPoint endPt = 0 == matchT ? c.firstPt() : c.lastPt();
    OP_ASSERT(endPt.soClose(equalPt));
    contour->contours->addAlias(endPt, equalPt);
    0 == matchT ? c.setFirstPt(equalPt) : c.setLastPt(equalPt);
    c.pinCtrl();
    if (c.firstPt() == c.lastPt())
        setDisabled(OP_LINE_FILE_NPARAMS());
    setBounds();
    for (OpIntersection* sect : sects.i) {
        if (sect->ptT.t == matchT) {
            sect->ptT.pt = equalPt;
            if (sect->opp->ptT.pt != equalPt) {
                if (sect->opp->ptT.onEnd())
                    sect->opp->segment->moveTo(sect->opp->ptT.t, equalPt);
                else
                    sect->opp->ptT.pt = equalPt;
            }
        }
    }
    edges.clear();
}

// two segments are coincident so move opp's winding to this and disabled opp
void OpSegment::moveWinding(OpSegment* opp, bool backwards) {
        winding.move(opp->winding, backwards);
        if (!winding.visible())
            setDisabled(OP_LINE_FILE_NPARAMS());
        opp->winding.zero();
        opp->setDisabled(OP_LINE_FILE_NPARAMS());
}

#if 0
bool OpSegment::nearby(float t, const OpSegment* opp, const OpPtT& base, float step) const {
	OpPtT testSegPtT { c.ptTAtT(t) };
	OpPtT testOppPtT { opp->c.ptTAtT(opp->findValidT(0, 1, testSegPtT.pt)) };
	bool near = testOppPtT.pt.isNearly(testSegPtT.pt);
    return near;
}
#endif

int OpSegment::nextID() const {
    return contour->nextID();
}

void OpSegment::setBounds() {
    ptBounds = c.ptBounds();
    closeBounds = ptBounds.outsetClose();
}

void OpSegment::setDisabled(OP_LINE_FILE_NP_DEF()) {
	disabled = true; 
    // coincident/unsectable intersections may confuse; remove any
    size_t index = sects.i.size();
    while (index) {
        OpIntersection* i = sects.i[--index];
// !!! experiment: remove all intersections
//        if (!i->coincidenceID && !i->unsectID)
//            continue;
        OpSegment* opp = i->opp->segment;
        size_t oIndex = opp->sects.i.size();
        while (oIndex) {
            OpIntersection* o = opp->sects.i[--oIndex];
            if (o->opp->segment == this)
                opp->sects.i.erase(opp->sects.i.begin() + oIndex);
        }
        sects.i.erase(sects.i.begin() + index);
    }
    OP_LINE_FILE_SET(debugSetDisabled); 
}

// !!! this finds coincidence on lines, but it needs to find unsectables as well ?
#if 0
void OpSegment::windCoincidences() {
//    if (!c.isLine())
 //       return;
    if (disabled)
        return;
    // iterate through edges; if edge is linear and matches opposite, mark both coincident
    for (OpEdge& edge : edges) {
        if (!edge.isLine())
            continue;
        // !!! consider removing unsectable if line is also coincident
        if (edge.isUnsectable())
            continue;
        // iterate through sects that match edge start and end, looking for parallel edges
        OpIntersection** firstBegin = nullptr;
        OpIntersection** firstEnd = nullptr;  // one after last start
        OpIntersection** lastBegin = nullptr;
        OpIntersection** lastEnd = nullptr;  // one after last end
        for (OpIntersection** sPtr = &sects.i.front(); sPtr <= &sects.i.back(); ++sPtr) {
            OpIntersection* sect = *sPtr;
            if (!firstBegin) {
                if (sect->ptT == edge.start())
                    firstBegin = sPtr;
                continue;
            }
            if (!firstEnd) {
                if (sect->ptT != edge.start())
                    firstEnd = sPtr;
            }
            if (!lastBegin) {
                if (sect->ptT == edge.end())
                    lastBegin = sPtr;
                continue;
            }
            if (!lastEnd) {
                if (sect->ptT != edge.end())
                    lastEnd = sPtr;
                break;
            }
        }
        // look for an end with a matching opposite intersection
        for (OpIntersection** firstTest = firstBegin; firstTest < firstEnd; ++firstTest) {
            OpIntersection* firstOSect = (*firstTest)->opp;
            if ((*firstTest)->ptT.pt != firstOSect->ptT.pt) {
                OP_ASSERT(firstOSect->unsectID);
                continue;
            }
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
                if (!oppEdge->isLine())
                    break;
                if (oppEdge->isUnsectable())
                    break;
                // verify that all four intersections are not used to mark coincidence
                if (oStart->coincidenceID)
                    break;  // !!! was return;
                if (oEnd->coincidenceID)
                    break;  // !!! was return;
                OpIntersection* const* eStart = oSegment->sects.entry(oppEdge->start(), this);
                if (!eStart || (*eStart)->opp->coincidenceID)
                    break;  // !!! was return;
                OpIntersection* const* eEnd = oSegment->sects.entry(oppEdge->end(), this);
                if (!eEnd || (*eEnd)->opp->coincidenceID)
                    break;  // !!! was return;
                // mark the intersections as coincident
                int coinID = oSegment->coinID(reversed);
                OP_ASSERT(MatchEnds::none == oStart->coinEnd);
                oStart->setCoin(coinID, MatchEnds::start);
                OP_ASSERT(MatchEnds::none == oEnd->coinEnd);
                oEnd->setCoin(coinID, MatchEnds::end);
                OP_ASSERT(MatchEnds::none == (*eStart)->opp->coinEnd);
                (*eStart)->opp->setCoin(coinID, reversed ? MatchEnds::end : MatchEnds::start);
                OP_ASSERT(MatchEnds::none == (*eEnd)->opp->coinEnd);
                (*eEnd)->opp->setCoin(coinID, reversed ? MatchEnds::start : MatchEnds::end);
                sects.resort = true;
            }
        }
    }
    sects.windCoincidences(edges);
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

// replace calls that generate coin ids with newWindCoincidences()
// Currently, a pair of edges intersected by a third edge may produce a coincidence that goes
// undetected. The code that generates coincidence in general is overly complicated. If  
// newWindCoincidences() is run after all intersections and edges are generated, it should be
// able to find all coincidences and resolve all windings. To be successful, it may need to 
// generate additional intersections and edges if runs of coincident edges don't line up.

// Walk the edges and intersections at the same time. If the edge is not a line, continue.
// Use the intersection opposite to find possible coincident edges. If the opposite is not a line,
// ignore. If the opposite end point is not in the edge bounds, ignore. If the opposite end point
// is not on the edge's line, ignore.
#if 0
void OpSegment::newWindCoincidences() {
    if (disabled)
        return;
    size_t edgeIndex = 0;  // use index instead of pointer so new edges may be inserted
    size_t sectIndex = 0;
    OP_ASSERT(sects.i.size());
    OpIntersection* next = sects.i[0];
    OpIntersection* sect;
    auto advanceSect = [&next, &sect, &sectIndex, this]() {
        sect = next;
        next = sectIndex < sects.i.size() ? sects.i[sectIndex++] : nullptr;
    };
    while (edgeIndex < edges.size()) {
        OpEdge* edge = &edges[edgeIndex++];
        if (!edge->isLine())
            continue;
        if (edge->disabled)
            continue;
        for (;;) {  // while sect is for a skipped edge ...
            advanceSect();
            OP_ASSERT(edge->startT >= sect->ptT.t);
            if (edge->startT == sect->ptT.t) 
                break;
            OP_ASSERT(sectIndex < sects.i.size());
        }
        auto checkCoin = [&edge /*, &sect */](OpEdge* oEdge, EdgeMatch oMatch) {
            if (!oEdge->isLine())
                return;
            if (oEdge->disabled)
                return;
            if (oEdge->isUnsectable())
                return;
            OpPoint oEnd = EdgeMatch::start == oMatch ? oEdge->endPt() : oEdge->startPt();
#if 0  // eventually, handle partial coincident. For now, just detect full coincidence
            if (!edge->ptBounds.contains(oEnd))
                return;
            LinePts linePts { edge->start.pt, edge->end.pt };
            if (!linePts.ptOnLine(oEnd))
                return;
#else
            if (edge->endPt() != oEnd)
                return;
#endif
#if 0 && OP_DEBUG
            std::string s = "coin? edge:" + STR(edge->id) + " oEdge:" + STR(oEdge->id);
            if (EdgeMatch::end == oMatch)
                s += " reversed";
            if (sect->coincidenceID)
                s += " extend";
#endif
            if (!edge->winding.visible())
                return;
            OP_ASSERT(oEdge->winding.visible());
            edge->winding.move(oEdge->winding, EdgeMatch::end == oMatch);
            oEdge->setDisabledZero(OP_LINE_FILE_NPARAMS());
            if (!edge->winding.visible()) {
                edge->setDisabled(OP_LINE_FILE_NPARAMS());
            }
//            OP_DEBUG_CODE(OpDebugOut(s + "\n"));
        };
        next = sect;
        --sectIndex;
        do {   // for each sect with the same t
            advanceSect();
            if (MatchEnds::start == sect->coinEnd) {
                OP_ASSERT(sect->coincidenceID);
                continue;
            }
            if (MatchEnds::start == sect->unsectEnd) {
                OP_ASSERT(sect->unsectID);
                continue;
            }
            OpIntersection* oppSect = sect->opp;
            if (sect->ptT.pt != oppSect->ptT.pt) {
                OP_ASSERT(oppSect->unsectID);
                OP_ASSERT(MatchEnds::none != sect->unsectEnd);
                continue;
            }
            if (MatchEnds::start == sect->coinEnd)
                continue;
            // if coin is at end, check should report coin extension
            OP_ASSERT(sect->coincidenceID == oppSect->coincidenceID);
    //        OP_ASSERT(!oppSect->unsectID);  // this is ok; check opp edge for unsectability
    //        OP_ASSERT(MatchEnds::none == oppSect->unsectEnd);  // ditto
            OpSegment* oppSeg = oppSect->segment;
            OP_ASSERT(!oppSeg->disabled);
            size_t oppEdgeIndex = 0;
            OpEdge* oppEdge = nullptr;
            do {
                if (oppEdgeIndex == oppSeg->edges.size()) {  // no opp edge if t value range is zero 
                    oppEdge = nullptr;
                    break;
                }
                oppEdge = &oppSeg->edges[oppEdgeIndex++];
            } while (oppEdge->startPt() != edge->startPt() && oppEdge->endPt() != edge->startPt());
            if (!oppEdge)
                continue;
            EdgeMatch match = oppEdge->startPt() == edge->startPt() ? EdgeMatch::start : EdgeMatch::end;
            OP_ASSERT(EdgeMatch::start == match || oppEdge->endPt() == edge->startPt());
            checkCoin(oppEdge, match);
            if (EdgeMatch::end == match && oppEdgeIndex < oppSeg->edges.size()) {
                oppEdge = &oppSeg->edges[oppEdgeIndex];
                checkCoin(oppEdge, EdgeMatch::start);
            }
        } while (next && next->ptT.t == sect->ptT.t);
    }
}
#endif

void OpSegment::makeCoins() {
    if (!hasCoin)
        return;
    if (disabled)
        return;
    for (OpEdge& edge : edges) {
        if (edge.disabled)
            continue;
        // if edge is coincident, transfer windings and unsectable sects to boss
        for (OpIntersection* cSect : edge.cSects) {
            OpSegment* oSeg = cSect->opp->segment;
            OP_ASSERT(oSeg != this);
            if (oSeg->disabled)
                continue;
            int cID = abs(cSect->coincidenceID);
            EdgeMatch match = cSect->coincidenceID < 0 ? EdgeMatch::end : EdgeMatch::start;
            OP_ASSERT(cID);
            for (OpEdge& oEdge : oSeg->edges) {
                if (oEdge.disabled)
                    continue;
                if (edge.startPt() != oEdge.ptT(match).pt)
                    continue;
                OP_ASSERT(edge.endPt() == oEdge.ptT(!match).pt);
                for (OpIntersection* oSect : oEdge.cSects) {
                    if (abs(oSect->coincidenceID) != cID)
                        continue;
                    OP_ASSERT(edge.winding.visible());
                    OP_ASSERT(oEdge.winding.visible());
                    edge.winding.move(oEdge.winding, cSect->coincidenceID < 0);
                    oEdge.setDisabledZero(OP_LINE_FILE_NPARAMS());
                    if (!edge.winding.visible()) {
                        edge.setDisabled(OP_LINE_FILE_NPARAMS());
                        goto nextEdge;
                    }
                }
            }
        }
nextEdge: ;
    }
}

// create list of unsectable edges that match previous found unsectable intersections.
void OpSegment::makePals() {
    if (!hasUnsectable)
        return;
    if (disabled)
        return;
    for (OpEdge& edge : edges) {
        if (edge.disabled)
            continue;
        for (OpIntersection* uSect : edge.uSects) {
            int uID = abs(uSect->unsectID);
            OP_ASSERT(uID);
            OpSegment* oSeg = uSect->opp->segment;
            OP_ASSERT(oSeg != this);
            if (oSeg->disabled)
                continue;
            for (OpEdge& oEdge : oSeg->edges) {
                if (oEdge.disabled)
                    continue;
                for (OpIntersection* oSect : oEdge.uSects) {
                    if (abs(oSect->unsectID) != uID)
                        continue;
                    auto found = std::find_if(edge.uPals.begin(), edge.uPals.end(), 
                            [&edge](const EdgePal& test) { return &edge != test.edge; });
                    if (edge.uPals.end() == found) {
                        EdgePal pal { &oEdge, uSect->unsectID != oSect->unsectID 
                                OP_DEBUG_PARAMS(uID) };
                        edge.uPals.push_back(pal);
                    }
                }
            }
        }
    }
}
