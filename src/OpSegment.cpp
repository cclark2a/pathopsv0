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
    , willDisable(false)
    , hasCoin(false)
    , hasUnsectable(false)
    , startMoved(false)
    , endMoved(false) {
    setBounds();
    OP_DEBUG_IMAGE_CODE(debugColor = black);
}

// !!! optimization:  if called from opedge linkup, could abort if >1 active found?

// Unsectable edges may or may not be able to have their wind zero side computed;
// for now, treat any unsectable multiple as having a zero side whether it does or not.
// returns true if emplaced edge has pals

// activeNeighbor is called separately because this iterates through opposite intersections only
// returns true if any found edge is a pal
bool OpSegment::activeAtT(const OpEdge* edge, EdgeMatch match, std::vector<FoundEdge>& oppEdges
        ) const {
    unsigned edgesSize = oppEdges.size();
    OP_ASSERT(!edge->disabled);
    // each prospective match normal must agree with edge, indicating direction of area outside fill
    // if number of matching sects doesn't agree with opposite, collect next indirection as well
    OpPtT ptT = edge->whichPtT(match);
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
        auto isSortable = [](const OpEdge* e, const OpEdge* o) {
            if (Unsortable::none != e->isUnsortable)
                return false;
            if (!e->isUnsectable())
                return true;
            if (e->isPal(o))
                return true;
            if (!o->ray.find(e))
                return true;
            return o->ray.sectsAllPals(e);
        };
        auto saveMatch = [edge, &oppEdges, &oSect, checkZero, isSortable](EdgeMatch testEnd) {
            OpSegment* oSeg = oSect->segment;
            OpEdge* test = oSeg->findEnabled(oSect->ptT, testEnd);  // !!! optimization: walk edges in order
            if (!test || test == edge)
                return;
            if (isSortable(edge, test) && isSortable(test, edge)
                    && edge->windZero != checkZero(test, edge->which(), testEnd))
                return;
            if (!test->hasLinkTo(testEnd))
                oppEdges.emplace_back(test, EdgeMatch::none);
        };
        saveMatch(EdgeMatch::start);
        saveMatch(EdgeMatch::end);
    }
    for (unsigned index = edgesSize; index < oppEdges.size(); ++index) {
        if (oppEdges[index].edge->isUnsectable())
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
    if (Unsortable::none != edge->isUnsortable || edge->windZero == nextDoor->windZero 
            || Unsortable::none != nextDoor->isUnsortable) {
        oppEdges.emplace_back(nextDoor, EdgeMatch::none);
        return nextDoor->isUnsectable();
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

OpPoint OpSegment::aliasOriginal(MatchEnds end) const {
    OP_ASSERT((MatchEnds::start == end && startMoved) || (MatchEnds::end == end && endMoved));
    OpPoint aliased = MatchEnds::start == end ? c.firstPt() : c.lastPt();
    OpPoint original = contour->contours->findAlias(aliased);
    OP_ASSERT(original.isFinite());
    return original;
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
    edges.clear();
    if (disabled)
       return;
    OP_ASSERT(sects.i.size());
    edges.reserve(sects.i.size() - 1);
    sects.makeEdges(this);
}

MatchReverse OpSegment::matchEnds(const LinePts& linePts) const {
    if (disabled || willDisable)
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
OpPoint OpSegment::moveTo(float matchT, OpPoint oppEqualPt) {
    OP_ASSERT(0 == matchT || 1 == matchT);
    OpPoint endPt = 0 == matchT ? c.firstPt() : c.lastPt();
    if (0 == matchT ? startMoved : endMoved)
        return endPt;
//    OP_ASSERT(endPt.soClose(equalPt));  // fuzz_k1 triggers assert
    OpPoint equalPt = contour->contours->existingAlias(oppEqualPt);
    if (endPt == equalPt)
        return equalPt;  // caller must move opp instead
    bool useSectOpp = false;
    for (OpIntersection* sect : sects.i) {
        if (sect->ptT.t != matchT)
            continue;
        OpIntersection* sectOpp = sect->opp;
        if (sectOpp->ptT.pt == equalPt)
            continue;
        OpSegment* segOpp = sectOpp->segment;
        float sectOppT = sectOpp->ptT.t;
        if ((sectOppT != 0 || !segOpp->startMoved || segOpp->c.firstPt() != sectOpp->ptT.pt) 
                && (sectOppT != 1 || !segOpp->endMoved || segOpp->c.lastPt() != sectOpp->ptT.pt))
            continue;
        equalPt = sectOpp->ptT.pt;
        if (endPt == equalPt)
            return equalPt;  // caller must move opp instead
        OP_ASSERT(!useSectOpp);
        useSectOpp = true;
    }
    if (!contour->contours->addAlias(endPt, equalPt))
        return OpPoint(SetToNaN::dummy);
    if (0 == matchT) {
         c.setFirstPt(equalPt);
         startMoved = true;
    } else {
         c.setLastPt(equalPt);
         endMoved = true;
    }
    c.pinCtrl();
// defer disabling until all moves are complete; disable small segments will clean up
   if (c.firstPt() == c.lastPt())
        willDisable = true;
    setBounds();
    for (OpIntersection* sect : sects.i) {
        OpDebugBreak(sect, 581);
        OpDebugBreak(sect, 582);
        if (sect->ptT.t == matchT) {
            sect->ptT.pt = equalPt;
            if (sect->opp->ptT.pt != equalPt) {
                if (sect->opp->ptT.onEnd()) {
                    OpPoint altPt = sect->opp->segment->moveTo(sect->opp->ptT.t, equalPt);
                    OP_ASSERT(!altPt.isFinite());
                } else
                    sect->opp->ptT.pt = equalPt;
            }
        }
    }
    edges.clear();
    return useSectOpp ? equalPt : OpPoint(SetToNaN::dummy);
}

// two segments are coincident so move opp's winding to this and disabled opp
void OpSegment::moveWinding(OpSegment* opp, bool backwards) {
        winding.move(opp->winding, backwards);
        if (!winding.visible())
            setDisabled(OP_LINE_FILE_NPARAMS());
        opp->winding.zero();
        opp->setDisabled(OP_LINE_FILE_NPARAMS());
}

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

bool OpSegment::simpleEnd(const OpEdge* edge) const {
    if (!sects.simpleEnd())
        return false;
    return edge == &edges.back();

}

bool OpSegment::simpleStart(const OpEdge* edge) const {
    if (!sects.simpleStart())
        return false;
    return edge == &edges.front();
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

// Note that this must handle a many-to-many relationship between seg and opp.
// Coincident runs of edges may be interrupted by other intersections but their winding is
// unaffected (only other coins may break inner coincident windings).
void OpSegment::makeCoins() {
    if (!hasCoin)
        return;
    if (disabled)
        return;
    for (size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
        OpEdge& edge = edges[edgeIndex];
        if (!edge.coinPals.size())
            continue;
        if (edge.disabled)
            continue;
        // if edge is coincident, transfer windings and unsectable sects to boss
        bool transferred = false;
        for (const CoinPal& cPal : edge.coinPals) {
            OpSegment* oSeg = cPal.opp;
            OP_ASSERT(oSeg != this);
            if (oSeg->disabled)
                continue;
            int cID = cPal.coinID;
            OP_ASSERT(cID);
            auto transferWinding = [&edge, cID, &transferred](OpEdge* oEdge, EdgeMatch match) {
                if (oEdge->disabled)
                    return false;
                if (edge.startPt() != oEdge->ptT(match).pt)
                    return false;
                std::vector<CoinPal>& ocPals = oEdge->coinPals;
                if (ocPals.end() == std::find_if(ocPals.begin(), ocPals.end(), [cID]
                        (const CoinPal& ocPal){ return ocPal.coinID == cID; }))
                    return false;
                OP_ASSERT(oEdge->winding.visible());
                edge.winding.move(oEdge->winding, cID < 0);
                oEdge->setDisabledZero(OP_LINE_FILE_NPARAMS());
                transferred = true;
                return true;
            };
            // repeat for any opp edges that make up the rest of this coin span
            auto transferCopy = [](OpEdge* oTest, OpEdge* oEdge) {
                if (oTest->coinPals != oEdge->coinPals)
                    return false;
                oTest->setDisabledZero(OP_LINE_FILE_NPARAMS());
                return true;
            };
            size_t oSize = oSeg->edges.size();
            if (cID < 0) {                
                for (size_t oppIndex = oSize; oppIndex-- != 0; ) {
                    OpEdge* oEdge = &oSeg->edges[oppIndex];
                    if (transferWinding(oEdge, EdgeMatch::end)) {
                        while (oppIndex-- != 0 && transferCopy(&oSeg->edges[oppIndex], oEdge))
                             ;
                        break;
                    }
                }
            } else {
                for (size_t oppIndex = 0; oppIndex < oSize; ++oppIndex) {
                    OpEdge* oEdge = &oSeg->edges[oppIndex];
                    if (transferWinding(oEdge, EdgeMatch::start)) {
                         while (++oppIndex < oSize && transferCopy(&oSeg->edges[oppIndex], oEdge))
                             ;
                         break;
                    }
                }
            }
        }
        // repeat for any edges that make up the rest of this coin span
        if (!transferred)
            continue;
        bool setDisabled = !edge.winding.visible();
        if (setDisabled)
            edge.setDisabled(OP_LINE_FILE_NPARAMS());
        while (++edgeIndex < edges.size()) {
            OpEdge& test = edges[edgeIndex];
            if (test.coinPals != edge.coinPals)
                break;
            test.winding = edge.winding;
            if (setDisabled)
                test.setDisabled(OP_LINE_FILE_NPARAMS());
        }
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
        for (OpIntersection* uSect : edge.unSects) {
            int uID = abs(uSect->unsectID);
            OP_ASSERT(uID);
            OpSegment* oSeg = uSect->opp->segment;
            OP_ASSERT(oSeg != this);
            if (oSeg->disabled)
                continue;
            for (OpEdge& oEdge : oSeg->edges) {
                if (oEdge.disabled)
                    continue;
                for (OpIntersection* oSect : oEdge.unSects) {
                    if (abs(oSect->unsectID) != uID)
                        continue;
                    auto found = std::find_if(edge.pals.begin(), edge.pals.end(), 
                            [&edge](const EdgePal& test) { return &edge != test.edge; });
                    if (edge.pals.end() == found) {
                        EdgePal pal { &oEdge, uSect->unsectID != oSect->unsectID 
                                OP_DEBUG_PARAMS(uID) };
                        edge.pals.push_back(pal);
                    }
                }
            }
        }
    }
}
