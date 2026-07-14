// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContext.h"
#include "OpCurve.h"
#include "OpSegment.h"
#include "OpWinder.h"

#if 0
void FoundEdge::check(std::vector<FoundEdge>* edges, OpEdge* test, EdgeMatch em, OpPoint match) {
	if (edges && edges->size())
		return;
	float gapSq = (test->whichSect(em).pt - match).lengthSquared();
	if (distSq > gapSq) {
		distSq = gapSq;
		edge = test;
		whichEnd = em;
	}
}
#endif

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

OpSegment::OpSegment(PathOpsV0Lib::Contour* libContour, PathOpsV0Lib::AddCurve addCurve)    
	: contour((OpContour*) libContour)
	, c({ addCurve.context, (PathOpsV0Lib::CurveData*) addCurve.points, 
			addCurve.size, addCurve.type }, Rotated::init )
    , winding(contour->winding())
    , id(contour->nextID())
{
    init();
}

// !!! optimization:  if called from opedge linkup, could abort if >1 active found?

// Unsectable edges may or may not be able to have their wind zero side computed;
// for now, treat any unsectable multiple as having a zero side whether it does or not.
// returns true if emplaced edge has pals

// active neighbor is called separately because this iterates through opposite intersections only
// returns true if any found edge is a pal
bool OpSegment::activeAtT(OpEdge* edge, EdgeMatch match, MatchZero matchZero,
		std::vector<FoundEdge>& oppEdges) const {
	unsigned edgesSize = (unsigned) oppEdges.size();
	OP_ASSERT(!edge->disabled);
	// each prospective match normal must agree with edge, indicating direction of area outside fill
	// if number of matching sects doesn't agree with opposite, collect next indirection as well
	OpPtT ptT = edge->whichSect(match);
	for (auto sectPtr : sects.i) {
		OpIntersection& sect = *sectPtr;
		if (sect.ptT.pt != ptT.pt) {
			if (sect.ptT.t < ptT.t)
				continue;  // !!! could binary search for 1st if intersection list is extremely long
			if (sect.ptT.t > ptT.t)
				break;
		}
		OpIntersection* oSect = sect.opp;
//		if (ptT.pt != oSect->ptT.pt)
//			continue;
		// op operator is not needed since zero side was computed by apply
		auto checkZero = [match](const OpEdge* test, EdgeMatch eWhich, EdgeMatch testEnd) {
			WindZero zeroSide = test->windZero;
			EdgeMatch which = eWhich == testEnd ? EdgeMatch::start : EdgeMatch::end;
			if (which == match)
				zeroSide = !zeroSide;
			return zeroSide;
		};
		auto isSortable = [](const OpEdge* e, OpEdge* o) {
		#if 0
			if (!e->isSortable())
				return false;
			if (e->isSummable())
				return true;
			if (e->isPal(o))
				return true;
			if (!o->ray.find(e))
				return true;
			return o->ray.sectsAllPals(e);
		#else
			return e->isSortable() && e->isSummable();
		#endif
		};
		auto saveMatch = [edge, &oppEdges, &oSect, checkZero, matchZero, isSortable](EdgeMatch testEnd) {
			OpSegment* oSeg = oSect->segment;
			OpEdge* test = oSeg->findEnabled(oSect->ptT, testEnd);  // !!! optimization: walk edges in order
			if (!test || test == edge)
				return;
			if (test->smallTRange)
				return; 
			if (MatchZero::yes == matchZero && isSortable(edge, test) && isSortable(test, edge)
					&& edge->windZero != checkZero(test, edge->which(), testEnd))
				return;
			if (!test->hasLinkTo(EdgeMatch::start == test->which() ? testEnd : !testEnd))
				oppEdges.emplace_back(test, EdgeMatch::none);
		};
		saveMatch(EdgeMatch::start);
		saveMatch(EdgeMatch::end);
	}
	for (unsigned index = edgesSize; index < oppEdges.size(); ++index) {
		if (oppEdges[index].edge->hasPals())
			return true;
	}
	return false;
}

// returns true if emplaced edge has pals
// allow linked if true requires candidates to be in linked list
bool OpSegment::activeNeighbor(const OpEdge* edge, EdgeMatch match, AllowLinked allowLinked,
		std::vector<FoundEdge>& oppEdges) const {
	if ((EdgeMatch::start == match && edge->startT == 0)
			|| (EdgeMatch::end == match && edge->endT == 1))
		return false;
	EdgeMatch neighbor = EdgeMatch::start == match ? !edge->which() : edge->which();
	OpPtT ptT = edge->whichSect(match);
	OpEdge* nextDoor = findEnabled(ptT, neighbor);
	if (!nextDoor) 
	   return false;
	for (auto& alreadyFound : oppEdges)
		if (alreadyFound.edge == nextDoor)
			return false;
	if (AllowLinked::no == allowLinked ? nextDoor->hasLinkTo(neighbor) : !nextDoor->priorEdge)
		return false;
	if (!edge->isSortable() || edge->windZero == nextDoor->windZero || !nextDoor->isSortable()) {
		oppEdges.emplace_back(nextDoor, EdgeMatch::none);
		oppEdges.back().neighborEnd = neighbor;
		return nextDoor->hasPals();
	}
	return false;
}

void OpSegment::addDisjointIntersections() {
	sects.sort();
	if (!sects.i.size() || 0 != sects.i.front()->ptT.t) {
		OpIntersection* sect = addSegBase({ c.firstPt(), 0 }  OP_LINE_FILE_PARAMS(this));
		sect->pair(sect);
	    sects.sort();
	}
	if (!sects.i.size() || 1 != sects.i.back()->ptT.t) {
		OpIntersection* sect = addSegBase({ c.lastPt(), 1 }  OP_LINE_FILE_PARAMS(this));
		sect->pair(sect);
	}
}

#if 0
OpIntersection* OpSegment::addEdgeSect(const OpPtT& ptT  
		OP_LINE_FILE_DEF(const OpEdge* debugEdge, const OpEdge* debugOpp)) {
	OP_ASSERT(!sects.debugContains(ptT, debugOpp->segment));
	return sects.add(contour->addEdgeSect(ptT, this  
			OP_LINE_FILE_CALLER(debugEdge, debugOpp)));
}
#endif

OpIntersection* OpSegment::addSegBase(const OpPtT& ptT  OP_LINE_FILE_DEF(const OpSegment* oSeg)) {
	OP_ASSERT(!sects.debugContains(ptT, oSeg));
	return sects.add(contour->addSegSect(ptT, this  OP_LINE_FILE_CALLER(oSeg)));
}

OpIntersection* OpSegment::addSegEnd(const OpPtT& ptT, const OpSegment* oSeg    
		OP_LINE_FILE_ARGS()) {
	OpIntersection* result = addSegSect(ptT, oSeg  OP_LINE_FILE_CARGS());
	result->callerPt = c.ptAtT(ptT.t);
	return result;
}

OpIntersection* OpSegment::addSegSect(const OpPtT& ptT, const OpSegment* oSeg    
		OP_LINE_FILE_ARGS()) {
	if (sects.contains(ptT, oSeg))
		return nullptr;
	OpIntersection* result = addSegBase(ptT  OP_LINE_FILE_CALLER(oSeg));
	return result;
}

OpIntersection* OpSegment::addCoin(const OpPtT& ptT, int coinID, MatchEnds coinEnd, CoinOpp coinOpp,
		const OpSegment* oSeg  OP_LINE_FILE_ARGS()) {
		// !!! commented out still may be necessary, but contains may need to be coinContains?
//	if (sects.contains(ptT, oSeg))  // triggered by fuzz763_13
//		return nullptr;
	OpIntersection* next = contour->context->allocateIntersection();
	next->set(ptT, this  OP_LINE_FILE_CALLER(id, oSeg->id));
	next->setCoin(coinID, coinEnd, coinOpp);  // 0 if no coincidence; neg if coin pairs are reversed
	return sects.add(next);
}

OpIntersection* OpSegment::addUnsectable(const OpPtT& ptT, int usectID, MatchEnds end,
		const OpSegment* oSeg    OP_LINE_FILE_ARGS()) {
	// !!! replace with assert to disallow contains here
	// rework caller to do contains in pairs prior to calling add
	OpIntersection* sect = sects.contains(ptT, oSeg);
	if (sect) {
		OP_ASSERT(!sect->unsectID);
		sect->setUnsect(usectID, end);
		sects.unsorted = true;
		return sect;
	}
	return sects.add(contour->addUnsect(ptT, this, usectID, end  OP_LINE_FILE_CALLER(oSeg)));
}

WindingCondition OpSegment::apply() {
	for (auto& edge : edges) {
        if (WindingCondition windingCondition = edge.apply())
            return windingCondition;
    }
    return 0;
}

struct Misses {
	Misses(const OpPtT& aPtT, const OpPtT& cPtT, OpSegment* a, OpSegment* c, 
			OpIntersection* aInC, OpIntersection* cInA, int aCid, int cCid)
		: aStart(aPtT)
		, cStart(cPtT)
		, segA(a)
		, segC(c)
		, aSSect(aInC)
		, cSSect(cInA)
		, aCoinID(aCid)
		, cCoinID(cCid)
		, used(false) {
	}

	OpPtT aStart;
	OpPtT cStart;
	OpSegment* segA;
	OpSegment* segC;
	OpIntersection* aSSect;
	OpIntersection* cSSect;
	int aCoinID;
	int cCoinID;
	bool used;
};

// if a and b are coincident, and a and c are coincident, add b/c coin if missing
void OpSegment::manyCoincidences() {
	if (!hasCoin)
		return;
	// if this segment has coincident runs with three or more edges, make the sects consistent
	std::vector<Misses> misses;
	// A and B are coincident; C is also coincident with B, but C-A coin may have been missed
	// find or create the missing intersections
	auto checkStart = [&misses](OpIntersection* sectB, OpIntersection* sectC) {
		OP_DEBUG_CODE(OpSegment* segB = sectB->segment);
		OpSegment* segC = sectC->segment;
		OP_ASSERT(segB != segC);
		OpIntersection* sectA = sectB->opp;
		OpSegment* segA = sectA->segment;
		OP_ASSERT(segA != segB);
		OP_ASSERT(segA != segC);
		// check A for C
		OpPtT ptTc = sectC->ptT;
		OpPtT ptTa(SetToNaN::dummy);
		OpIntersection* cInA = segA->sects.coinContains(ptTc.pt, segC, &ptTa);
		if (cInA && cInA->coincidenceID)
			return;
		if (OpMath::IsNaN(ptTa.t))
			ptTa = OpPtT(ptTc.pt, segA->c.matchVector(ptTc.pt, segC->c.normal(ptTc.t)));
		if (OpMath::IsNaN(ptTa.t))
			return;
		OpPoint ptA = ptTa.pt;
		OpIntersection* aInC = segC->sects.coinContains(ptA, segA, &ptTc);
		if (aInC && aInC->coincidenceID)
			return;
		if (OpMath::IsNaN(ptTc.t))
			ptTc = OpPtT(ptA, segC->c.matchVector(ptA, segA->c.normal(ptTa.t)));
		if (OpMath::IsNaN(ptTc.t))
			return;
		misses.emplace_back(ptTa, ptTc, segA, segC, cInA, aInC, sectA->coincidenceID
				, sectC->coincidenceID);
	};
	auto checkEnd = [](Misses& miss, OpIntersection* bEnd, OpVector thresh) {
		OpSegment* segA = miss.segA;
		OpSegment* segC = miss.segC;
		OP_DEBUG_CODE(OpIntersection* oppSect = bEnd->opp);
		OP_ASSERT(oppSect->segment == segA || oppSect->segment == segC);
		OpPtT oppPtT = bEnd->ptT;
		OpPtT ptTc(SetToNaN::dummy);
		OpIntersection* aInC = segC->sects.coinContains(oppPtT.pt, segA, &ptTc);
		if (aInC && aInC->coincidenceID)
			return;
		if (OpMath::IsNaN(ptTc.t))
			ptTc = OpPtT(oppPtT.pt, segC->c.matchVector(oppPtT.pt, 
					bEnd->segment->c.normal(oppPtT.t)));
		if (OpMath::IsNaN(ptTc.t))
			return;
		OpPtT ptTa(SetToNaN::dummy);
		OpIntersection* cInA = segA->sects.coinContains(ptTc.pt, segC, &ptTa);
		if (cInA && cInA->coincidenceID)
			return;
		if (OpMath::IsNaN(ptTa.t))
			ptTa = OpPtT(ptTc.pt, segA->c.matchVector(ptTc.pt, segC->c.normal(ptTc.t)));
		if (OpMath::IsNaN(ptTa.t))
			return;
		// once both start and end of the missing intersections are found, set their coin and ends
		if (miss.aStart.isNearly(ptTa, thresh))
			return;
		if (miss.cStart.isNearly(ptTc, thresh))
			return;
//		if (miss.aStart.t > ptTa.t)
//			std::swap(miss.aStart, ptTa);
		bool aFlipped = miss.aStart.t > ptTa.t;
		bool cFlipped = miss.cStart.t > ptTc.t;
		int coinID = miss.segC->coinID(cFlipped != aFlipped);
		// remove existing sects between the to-be-added coin sects
		auto removeSects = [](OpSegment* seg, float startT, float endT, OpSegment* opp) {
			if (startT > endT)
				std::swap(startT, endT);
			size_t index = seg->sects.i.size();
			std::vector<int> uIDs;
			while (index--) {
				OpIntersection* test = seg->sects.i[index];
				if (test->ptT.t <= startT)
					break;
				if (test->ptT.t >= endT)
					continue;
				if (test->opp->segment != opp)
					continue;
				if (test->coincidenceID)
					continue;
				if (test->unsectID)  // if erasing this one, remove the companion unsectable ID
					uIDs.push_back(test->unsectID);
				seg->sects.i.erase(seg->sects.i.begin() + index);
			}
			if (uIDs.empty())
				return;
			bool segHasUnsectable = false;  // reset seg and sects cache bits
			bool segHasPairs = false;
			for (OpIntersection* check : seg->sects.i) {
				if (check->coincidenceID)
					segHasPairs = true;
				if (!check->unsectID)
					continue;
				if (uIDs.end() == std::find(uIDs.begin(), uIDs.end(), check->unsectID)) {
					segHasUnsectable = true;
					segHasPairs = true;
					continue;
				}
				check->zeroUnsectPair();
				seg->sects.unsorted = true;
			}
			seg->hasUnsectable = segHasUnsectable;
			seg->sects.hasPairs = segHasPairs;
		};
		removeSects(segA, miss.aStart.t, ptTa.t, segC);
		removeSects(segC, miss.cStart.t, ptTc.t, segA);
		auto setSect = [coinID](OpIntersection*& cInA, const OpPtT& aPtT,
				OpSegment* segA, MatchEnds match, CoinOpp coinOpp, OpSegment* segC  
				OP_LINE_FILE_ARGS()) {
			if (cInA)
				cInA->setCoin(coinID, match, cInA->segment == segA ? coinOpp : !coinOpp);
			else
				cInA = segA->addCoin(aPtT, coinID, match, coinOpp, segC  OP_LINE_FILE_PARGS());
		};
		setSect(miss.aSSect, miss.aStart, segA, MatchEnds::start, CoinOpp::no, segC  OP_LINE_FILE_PARGS());
		setSect(miss.cSSect, miss.cStart, segC, MatchEnds::start, CoinOpp::yes, segA  OP_LINE_FILE_PARGS());
		setSect(aInC, ptTa, segA, MatchEnds::end, CoinOpp::no, segC  OP_LINE_FILE_PARGS());
		setSect(cInA, ptTc, segC, MatchEnds::end, CoinOpp::yes, segA  OP_LINE_FILE_PARGS());
		if (miss.aSSect->segment != aInC->segment)
			std::swap(aInC, cInA);
		if (aFlipped)
			std::swap(miss.aSSect->coinEnd, aInC->coinEnd);
		if (cFlipped)
			std::swap(miss.cSSect->coinEnd, cInA->coinEnd);
		miss.aSSect->pair(miss.cSSect);
		cInA->pair(aInC);
		OP_DEBUG_VALIDATE_CODE(miss.aSSect->debugCoinValidate());
		OP_DEBUG_VALIDATE_CODE(miss.cSSect->debugCoinValidate());
		OP_DEBUG_VALIDATE_CODE(cInA->debugCoinValidate());
		OP_DEBUG_VALIDATE_CODE(aInC->debugCoinValidate());
		miss.used = true;
	};
	if (sects.unsorted)
		sects.sort();
	OpVector thresh = threshold();
	std::vector<OpIntersection*> coinSects;
	for (OpIntersection* sect : sects.i) {
		OP_ASSERT(sect->segment == this);
		if (!sect->coincidenceID)
				continue;
		if (MatchEnds::start == sect->coinEnd) {
			for (OpIntersection* coinSect : coinSects) {
				checkStart(coinSect, sect->opp);  // candidates' t is <= sect opp t
			}
			coinSects.push_back(sect);
			continue;
		}
		OP_ASSERT(MatchEnds::end == sect->coinEnd);
		auto csIter = std::find_if(coinSects.begin(), coinSects.end(), [sect](OpIntersection* test) {
			return test->coincidenceID == sect->coincidenceID;
		});
		OP_ASSERT(coinSects.end() != csIter);
		coinSects.erase(csIter);
		for (Misses& miss : misses) {
			if (miss.used)
				continue;
			if (miss.aCoinID != sect->coincidenceID && miss.cCoinID != sect->coincidenceID)
				continue;
			checkEnd(miss, sect, thresh);  // use lesser of candidates' t, since it is <= sect opp T
		}
	}
}

struct MissingSect {
    OpIntersection* sect;
    OpIntersection* coinStart;
};

// walk sects with coincidences; look at opp sects
// if opp sects is missing this point, add intersection
// if intersection is added, rebuild edges (should be rare)
    // a pair of segments are partially coincident: testQuads25683917
    // a third segment intersects one of the pair but not the other (curve grazes line)
    // without between coincidence, winding sees two edges in one coin but only one in the other
void OpSegment::betweenCoincidence() {
	if (disabled)
		return;
	if (!hasCoin)
		return;
	std::vector<OpIntersection*> coinSects;
	std::vector<MissingSect> missing;
	OpVector thresh = threshold();
	for (OpIntersection* sect : sects.i) {
		if (!sect->coincidenceID && coinSects.empty())
				continue;
		if (MatchEnds::start == sect->coinEnd) {
		    coinSects.push_back(sect);
        } else if (MatchEnds::end == sect->coinEnd) {
		    auto csIter = std::find_if(coinSects.begin(), coinSects.end(), 
                    [sect](OpIntersection* test) {
			    return test->coincidenceID == sect->coincidenceID;
		    });
		    OP_ASSERT(coinSects.end() != csIter);
		    coinSects.erase(csIter);
        }
        // check this point against all coincident ranges containing it; is it in opp. sect list?
        OpSegment* thirdParty = sect->opp->segment;
        for (auto coinSect : coinSects) {
            OpSegment* coinOpp = coinSect->opp->segment;
            if (thirdParty == coinOpp)
                continue;
            auto sectInOpp = std::find_if(coinOpp->sects.i.begin(), coinOpp->sects.i.end(),
                    [sect, thresh](OpIntersection* oppSect) {
                return sect->ptT.pt.isNearly(oppSect->ptT.pt, thresh);
            } );
            if (sectInOpp == coinOpp->sects.i.end())
                missing.push_back({ sect, coinSect });
        }
	}
    // add missing points to coincident ranges
    for (const MissingSect& miss : missing) {
        OP_ASSERT(MatchEnds::start == miss.coinStart->coinEnd);
#if OP_DEBUG
        auto coinEnd = std::find_if(sects.i.begin(), sects.i.end(), [miss]
                (const OpIntersection* sect) {
            return MatchEnds::end == sect->coinEnd 
                    && miss.coinStart->coincidenceID == sect->coincidenceID; 
        } );
#endif
        OP_ASSERT(sects.i.end() != coinEnd);
        OpSegment* sectOpp = miss.sect->opp->segment;
        OpSegment* coinOpp = miss.coinStart->opp->segment;
	#if 0
        float oppCoinStartT = miss.coinStart->opp->ptT.t;
        float oppCoinRange = (*coinEnd)->opp->ptT.t - oppCoinStartT;
        float coinRange = (*coinEnd)->ptT.t - miss.coinStart->ptT.t;
		// this fails; t does not necessarily change linearly, so cannot always be interpolated (testLoops6686)
        OpPtT oppPtT { miss.sect->ptT.pt,  oppCoinStartT 
                + (miss.sect->ptT.t - miss.coinStart->ptT.t) * oppCoinRange / coinRange };
	#else
		OpPtT oppPtT { miss.sect->ptT.pt, coinOpp->c.matchVector(miss.sect->ptT.pt,
				miss.sect->segment->c.tangent(miss.sect->ptT.t)) };
		if (!OpMath::IsFinite(oppPtT.t))
			continue;
	#endif
        OpIntersection* oSect = coinOpp->addSegSect(oppPtT, sectOpp  OP_LINE_FILE_PARGS());
        if (!oSect)
            continue;
        OpIntersection* iSect = sectOpp->addSegBase(miss.sect->opp->ptT
                OP_LINE_FILE_PARAMS(coinOpp));
        iSect->pair(oSect);
        sectOpp->sects.sort();
        coinOpp->sects.sort();
    }
}

int OpSegment::coinID(bool flipped) {
	int coinID = nextID();
	hasCoin = true;
	return flipped ? -coinID : coinID;
}

void OpSegment::disableSmall() {
	if (disabled)
		return;
	if (!sects.i.size() || sects.i.front()->ptT.pt == sects.i.back()->ptT.pt)
		setDisabled(OP_LINE_FILE_NPARGS());
}

// !!! this was lineIntersect which could miss if normal line points away from seg
//     but it was changed without fixing the root bug, so may make things less stable ...
OpPtT OpSegment::distance(const OpPtT& segPtT, OpSegment* opp) {
	for (OpIntersection* oppSect : opp->sects.i) {
		if (oppSect->opp->ptT.pt == segPtT.pt)
			return oppSect->ptT;
	}
	OpVector normal = c.normal(segPtT.t);
	if (!normal.isFinite())
		return OpPtT(SetToNaN::dummy);
	LinePts normLine { (OpPoint) segPtT.pt - normal, segPtT.pt + normal };
	MatchEnds oppEnds = opp->c.firstPt() == segPtT.pt ? MatchEnds::start 
			: opp->c.lastPt() == segPtT.pt ? MatchEnds::end : MatchEnds::none;
	OpRoots roots = opp->c.rayIntersect(normLine, oppEnds);
	float bestSq = OpInfinity;
	OpPtT bestPtT(SetToNaN::dummy);
	for (float root : roots.roots) {
		OpPtT oppPtT = opp->c.ptTAtT(root);
		float distSq = (segPtT.pt - oppPtT.pt).lengthSquared();
		if (bestSq > distSq) {
			bestSq = distSq;
			bestPtT = oppPtT;
		}
	}
	return bestPtT;
}

// !!! would it be any better (faster) to split this into findStart / findEnd instead?
OpEdge* OpSegment::findEnabled(const OpPtT& ptT, EdgeMatch match) const {
	for (auto& edge : edges) {
		// !!! this required both pt and t to match; try matching only point
		if (ptT.pt == edge.ptT(match).pt) {
			if (edge.smallTRange)
				continue;
			return edge.disabled ? nullptr : const_cast<OpEdge*>(&edge);
		}
	}
	return nullptr;
}

// rarely, moving points prevents finding matching ends. If there is no end, do an exhaustive search
#if 0
void OpSegment::findMissingEnds() {
	if (disabled)
		return;
	OP_ASSERT(!sects.unsorted);
	OpContext* context = contour->context;
	if (context->allowError(PathOpsV0Lib::ContextError::end, &c.c)) {
		bool missingStart = !sects.i.size() || 0 != sects.i.front()->ptT.t;
		if (missingStart) {
			OpIntersection* sect = contour->addSegSect({c.firstPt(), 0}, this  
					OP_LINE_FILE_PARAMS(this));
			sect->pair(sect);
			sects.i.insert(sects.i.begin(), sect);
		}
		bool missingEnd = !sects.i.size() || 1 != sects.i.back()->ptT.t;
		if (missingEnd) {
			OpIntersection* sect = addSegBase({c.lastPt(), 1}  OP_LINE_FILE_PARAMS(this));
			sect->pair(sect);
		}
	}
}
#endif

#if 0
// returns t iff opp point is between start and end
// start/end range is necessary since cubics can have more than one t at a point
float OpSegment::findValidT(float start, float end, OpPoint opp) {
	if (!c.isLine()) {
		OpRoots hRoots = c.axisRayHit(Axis::horizontal, opp.y, start, end);
		OpRoots vRoots = c.axisRayHit(Axis::vertical, opp.x, start, end);
	#if 01 // code coverage says this is unused, but it is required for loop48977
		if (1 != hRoots.count() && 1 != vRoots.count()) {
			if (0 == start && opp.isNearly(c.firstPt(), threshold()))
				return 0;
			if (1 == end && opp.isNearly(c.lastPt(), threshold()))
				return 1;
			return OpNaN;
		}
		if (1 != hRoots.count()) {
			OP_ASSERT(1 == vRoots.count());  // !!! triggered by thread_loops46134
			return vRoots.roots[0];
		}
	#endif
		if (1 != vRoots.count())
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
#endif

float OpSegment::findLineT(OpPoint opp) {
	OP_ASSERT(c.isLine());
	OpVector lineSize = c.lastPt() - c.firstPt();
	float result = fabsf(lineSize.dy) > fabsf(lineSize.dx) ?
		(opp.y - c.firstPt().y) / lineSize.dy : (opp.x - c.firstPt().x) / lineSize.dx;
	if (0 <= result && result <= 1)
		return result;
	return OpNaN;
}

// if intersections collapse, return true to restart; we're in the middle of walking sect array
bool OpSegment::fixCCSects() {
	if (!sects.hasCCSects)
		return false;
	sects.sort();
//	OP_ASSERT(2 < sects.i.size());
	OP_ASSERT(0 == sects.i.front()->ptT.t);
	OP_ASSERT(1 == sects.i.back()->ptT.t);
	if (startMoved() || endMoved()) {
		for (OpIntersection* test : sects.i) {
			if (c.aliasBounds().contains(test->ptT.pt))
				continue;
			if (PrefFound::disabled != moveSects(test->ptT, test->ptT.t < .5 
                    ? c.firstPt() : c.lastPt()))
				return false;
		}
	}
	OpVector segTan = c.lastPt() - c.firstPt();
	OpIntersection* prior = sects.i[0];
	// rework to advance while points are equal
	// still, if any are cc sect, check for move?
	size_t index = 0;
	OpIntersection* mid;
	bool midIsCcSect = false;
	do {  // skip sects until sect point changes
		mid = sects.i[++index];
		midIsCcSect |= mid->ccSect;
	} while (prior->ptT.t == mid->ptT.t || prior->ptT.pt == mid->ptT.pt);
	// allow consecutive sects to have the same pt; then find the 
	// prior different pt and the next different pt. If the mid range of points is 
	// not monotonic, then move range to the closer of prior and next
	while (++index < sects.i.size()) {
		OpIntersection* next = sects.i[index];
		if (mid->ptT.t == next->ptT.t || mid->ptT.pt == next->ptT.pt) {
			midIsCcSect |= next->ccSect;
			continue;
		}
		OP_ASSERT(prior->ptT.t < mid->ptT.t && prior->ptT.pt != mid->ptT.pt);
		OP_ASSERT(mid->ptT.t < next->ptT.t && mid->ptT.pt != next->ptT.pt);
		if (midIsCcSect) {
			OpVector priorV = mid->ptT.pt - prior->ptT.pt;
			bool priorOK = segTan.dx * priorV.dx >= 0 && segTan.dy * priorV.dy >= 0;
			OpVector nextV = next->ptT.pt - mid->ptT.pt;
			bool nextOK = segTan.dx * nextV.dx >= 0 && segTan.dy * nextV.dy >= 0;
			if (!priorOK || !nextOK) {
				float priorDistSq = priorV.lengthSquared();
				float nextDistSq = nextV.lengthSquared();
				// If collapsed, return false; otherwise, return true to restart.
				return PrefFound::disabled == moveSects(mid->ptT, priorDistSq < nextDistSq 
                        ? prior->ptT.pt : next->ptT.pt);
			}
		}  
		prior = mid;
		mid = next;
		midIsCcSect = mid->ccSect;
	}
	return false;
}

void OpSegment::init() {
	disabled = false;
	hasCoin = false;
	hasUnsectable = false;
	OpContext* context = contour->context;
	if (!c.isFinite()) {
		context->setError(PathOpsV0Lib::ContextError::finite  OP_DEBUG_PARAMS(id));
		setDisabled(OP_LINE_FILE_NPARGS());
	} else
		context->maxBounds.add(c.callerBounds());  // for threshold
}

void OpSegment::makeCoins() {
	if (disabled)
		return;
	sects.sort();  // make coins may add sects; so another segment may have added to this one
	auto nextSect = [this](unsigned* index, float t, MatchEnds match, bool alwaysAdvance) {
		do {
			if (alwaysAdvance) {
				alwaysAdvance = false;
				continue;
			}
			OpIntersection* result = sects.i[*index];
			if (result->ptT.t < t)
				continue;
			if (result->ptT.t > t)
				break; 
			if (result->coinEnd == match)
				continue;
			return result;
		} while (++*index < sects.i.size());
		return (OpIntersection*) nullptr;
	};
	unsigned startIndex = 0;
	for (OpEdge& edge : edges) {
		if (edge.disabled || !edge.isLine())
			continue;
		OpIntersection* startSect = nextSect(&startIndex, edge.startT, MatchEnds::start, false);
		if (!startSect)
			continue;
		OP_ASSERT(startSect->ptT.t == edge.startT);
		unsigned firstEnd = startIndex;
		OpIntersection* firstEndSect = nextSect(&firstEnd, edge.endT, MatchEnds::end, false);
		if (!firstEndSect)
			continue;
		OP_ASSERT(firstEndSect->ptT.t == edge.endT);
		// the segment may have multiple coincident lines with different opposites, all
		// starting at the same t
		do {
			OpIntersection* oppStart = startSect->opp;
			OpSegment* oppSeg = oppStart->segment;
			int oppStartID = oppSeg->id;
			// ideally, the sects should be sorted by id so that the start and end could
			// be walked lock-step. In practice, the sorting required for coins and unsectables
			// make this difficult. For now, just walk all sects with matching ts
			unsigned endIndex = firstEnd;
			OpIntersection* endSect = firstEndSect;
			while (oppStartID != endSect->opp->segment->id) {
				endSect = nextSect(&endIndex, edge.endT, MatchEnds::end, true);
				if (!endSect)
					goto nextSect;
			}
		{ // for goto
			OpIntersection* oppEnd = endSect->opp;
			OP_ASSERT(oppSeg == oppEnd->segment);
			std::vector<CoinPal>& pals = edge.coinPals;
			if (pals.end() != std::find_if(pals.begin(), pals.end(), [oppSeg](CoinPal& pal) {
					return pal.opp == oppSeg; }))
				continue;
			bool reversed = oppStart->ptT.t > oppEnd->ptT.t;
			OpPoint first = edge.curve.firstPt();
			OpPoint last = edge.curve.lastPt();
			for (OpEdge& oppEdge : oppSeg->edges) {
				if (!oppEdge.isLine())
					continue;
				if (reversed ? first == oppEdge.curve.lastPt() && last == oppEdge.curve.firstPt()
						: first == oppEdge.curve.firstPt() && last == oppEdge.curve.lastPt()) {
					int coinID = sects.coinRange(edge, oppEdge.segment, reversed);
					if (coinID) {
						if (reversed != (coinID < 0))
							coinID = -coinID;
						edge.coinPals.push_back({ oppEdge.segment, coinID });
						oppEdge.coinPals.push_back({ this, coinID });
					}
					break;
				}
			}
		} // for goto
	nextSect:
			;
		} while ((startSect = nextSect(&startIndex, edge.startT, MatchEnds::start, true)));
	}
}

void OpSegment::makeEdge(OP_LINE_FILE_NP_ARGS()) {
	if (!edges.size()) 
		edges.emplace_back(this  OP_LINE_FILE_PARGS());
}

void OpSegment::makeEdges() {
	edges.clear();
	if (disabled)
	   return;
	OP_ASSERT(sects.i.size());
	edges.reserve(sects.i.size());
	sects.makeEdges(this);
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
#if 1  // !!! causes nearly axis-aligned unsectables to fail (loop134368)
	   //     but, not doing so causes joiner trees to explode
				if (!edge.bounds().intersects(oEdge.bounds()))
					continue;
#endif
				for (OpIntersection* oSect : oEdge.unSects) {
					if (abs(oSect->unsectID) != uID)
						continue;
					OP_ASSERT(!edge.isPal(&oEdge));
					edge.pals.emplace_back(&oEdge, uID, uSect->unsectID != oSect->unsectID);
					edge.unsummable = true;
					hasPals = true;
					contour->hasPals = true;
					continue;
				}
			}
		}
	}
}

OpPtT OpSegment::matchEnd(OpPoint opp) {
#if 0
	OpPtT alignedEnd = alignToEnd(opp);
	if (OpMath::IsNaN(alignedEnd.t))
		alignedEnd = { opp, c.match(0, 1, opp) };
	if (!OpMath::IsNaN(alignedEnd.t))
		alignedEnd.t = OpMath::PinNear(alignedEnd.t);
	return alignedEnd;
#else
	if (c.start == c.end) {
		if (c.c.data->start == opp)
			return { c.start, 0 };
		if (c.c.data->end == opp)
			return { c.start, 1 };
	}
	if (c.start == opp)
		return { opp, 0 };
	if (c.end == opp)
		return { opp, 1 };
	return OpPtT(SetToNaN::dummy);
#endif
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

bool OpSegment::mergeEndPoints() {
	OP_ASSERT(disabled || !sects.i.empty());
	if (disabled || sects.i.front()->ptT.t == sects.i.back()->ptT.t
			|| sects.i.front()->ptT.t != 0 || sects.i.back()->ptT.t != 1) {
		OP_DEBUG_CODE(if (!disabled))
			setDisabled(OP_LINE_FILE_NPARGS());
		endsMerged = true;
		return false;
	}
	OP_ASSERT(sects.i.front()->ptT.t == 0);
	OP_ASSERT(sects.i.back()->ptT.t == 1);
	auto merge = [this](float endT, int initial, int delta) {
		bool rerun = false;
		int mergeId = 0;
		OpPoint endPt = endT ? c.c.data->end : c.c.data->start;
        OpPoint masterPt = endT ? c.end : c.start;
		// scan to see if merge is required
		int final = initial ? -1 : (int) sects.i.size();
		for (int index = initial; index != final; index += delta) {
			OP_ASSERT(0 <= index && index < (int) sects.i.size());
			OpIntersection* sect = sects.i[index];
			if (!c.isSmall && endT != sect->ptT.t)
				break;
			if (endPt == sect->opp->ptT.pt && endPt == sect->opp->callerPt)
				continue;
			goto needsMerge;
		}
		return false;
needsMerge:
		for (int index = initial; index != final; index += delta) {
			OP_ASSERT(0 <= index && index < (int) sects.i.size());
			OpIntersection* sect = sects.i[index];
			if (!c.isSmall && endT != sect->ptT.t)
				break;
            if (sect->mergeID) {
                masterPt = sect->ptT.pt;
                mergeId = sect->mergeID;
            } else if (sect->opp->mergeID) {
                masterPt = sect->opp->ptT.pt;
                mergeId = sect->opp->mergeID;
            } else if (!mergeId && sect->opp->segment->c.isSmall)
				masterPt = sect->opp->segment->c.start;
        }
		if (!mergeId)
			mergeId = contour->nextID();
		// mark all matching t with merge id
		for (int index = initial; index != final; index += delta) {
			OP_ASSERT(0 <= index && index < (int) sects.i.size());
			OpIntersection* sect = sects.i[index];
			if (!c.isSmall && endT != sect->ptT.t)
				break;
			if (mergeId == sect->mergeID)
				continue;
			sect->setMerge(mergeId, masterPt, MergeType::endPoint);
			sect->opp->segment->setEndsUnmerged();
			rerun = true;
		}
		return rerun;
	};
	bool runAgain = merge(0.f, 0, 1);
	runAgain |= merge(1.f, (int) sects.i.size() - 1, -1);
	endsMerged = true;  // don't merge this segment again
	return runAgain;  // but if an opposite segment changed, do rerun overlapping contours' segments
}

// after all intersections are found, scan list for close by results
// the threshold is the union of the context threshold and the distance from seg to opp
// if a pair of sects are within the threshold, they share a point, so give them matching merge ids
// It may be possible for a run of nearly identical points to have multiple merge ids. However, wait
// for a test case before coding for this.
bool OpSegment::mergeIntersections() {
	if (disabled) {
		merged = true;
		return false;
	}
	// !!! maybe (like cc) this should also use gap dist between seg and opp as threshold vals
	OpVector thresh = threshold();
    PathOpsV0Lib::CurveConst smallFuncPtr = contour->context->callback(c.c.type).smallTFuncPtr;
    float smallT = (smallFuncPtr ? (*smallFuncPtr)(c.c) : 32.f) * OpEpsilon;
	size_t index = 0;
	bool runAgain = false;
	do {
		// find range of nearly identical points
		OpIntersection* first = sects.i[index];
		OpRect mergeBounds = first->setMergeBounds(thresh);
		OpPtT mergePtT = first->ptT;
		int mergeId = first->mergeID;
		size_t endIndex = index;
		size_t startIndex = index;
		bool needsMerging = mergePtT.pt != first->opp->ptT.pt && !first->opp->unsectID;
		// !!! restructure to gather 1 or more sects close to each other
		//     if the sect/opp distance exceeds the threshold, expand the gather on both ends
		//     if the opp has already been merged, reuse the master merge/id
		//     if there is no master merge/id make one, assign to these sects and opp sects
		while (++endIndex < sects.i.size()) {
			OpIntersection* test = sects.i[endIndex];
			if (mergePtT.pt != test->ptT.pt) {
				OpRect testBounds = test->setMergeBounds(thresh);
				if (!mergeBounds.intersects(testBounds) && mergePtT.t + smallT < test->ptT.t)
					break;
				OpVector testWH = testBounds.widthHeight();
				OpVector oldThresh = thresh;
				thresh.dx = std::max(testWH.dx, thresh.dx);
				thresh.dy = std::max(testWH.dy, thresh.dy);
				if (!(oldThresh == thresh) && index > 1) {  // go backwards if prior sects are now mergable
					index -= 1;
					goto doBackup;
				}
				mergeBounds = testBounds;
				needsMerging = true;
			} else
				needsMerging |= test->ptT.pt != test->opp->ptT.pt && !test->opp->unsectID;
			if (test->mergeID) {
				if (mergeId && test->mergeID != mergeId)
					mergeMultiple(mergePtT.pt, mergeId, test->ptT.pt, test->mergeID);
				else {
					mergePtT = test->ptT;
					mergeId = test->mergeID;
				}
			}
		}
		// if pt != opp pt, choose side that has existing merge id
		for (; index < endIndex; ++index) {
			OpIntersection* opp = sects.i[index]->opp;
			if (!opp->mergeID)
				continue;
			if (opp->unsectID)
				continue;
			if (!opp->segment->merged)
				continue;
			if (mergeId && opp->mergeID != mergeId)
				mergeMultiple(mergePtT.pt, mergeId, opp->ptT.pt, opp->mergeID);
			else {
				mergePtT.pt = opp->ptT.pt;
				mergeId = opp->mergeID;
			}
		}
		if (needsMerging) {
			index = startIndex;
			if (!mergeId && index + 1 < endIndex)
				mergeId = contour->nextID();
			// if any in range are already merged, use that for all points
			for (; index < endIndex; ++index) {
				runAgain |= sects.i[index]->setMerge(mergeId, mergePtT.pt, MergeType::midPoint);
			}
		}
		index = endIndex;
doBackup:
		;
	} while (index + 1 < sects.i.size());
	if (sects.i.front()->ptT.pt == sects.i.back()->ptT.pt) {
		OP_ASSERT(0 == sects.i.front()->ptT.t);
		OP_ASSERT(1 == sects.i.back()->ptT.t);
		setDisabled(OP_LINE_FILE_NPARGS());
	}
	merged = true;  // don't merge this segment again
	return runAgain;  // but if an opposite segment changed, do rerun overlapping contours' segments
}

#if 0
// !!! confused on how to write this
// not sure what merge id on intersection means...
bool OpSegment::mergeOpposites() {
	if (disabled) {
		oppMerged = true;
		return false;
	}
	size_t index = 0;
	bool runAgain = false;
	do {
		OpIntersection* first = sects.i[index];
		OpPoint mergePt = first->ptT.pt;
		size_t endIndex = index;
		while (++endIndex < sects.i.size()) {
			OpIntersection* test = sects.i[endIndex];
			if (mergePt == test->ptT.pt && !test->mergeID)
				continue;
			if (first->ptT.pt == first->opp->ptT.pt) {
				++index;
				continue;
			}
		}
	} while (index + 1 < sects.i.size());
	oppMerged = true;  // don't merge this segment's sects' opposites again
	return runAgain;  // but if an opposite segment changed, do rerun overlapping contours' segments
}
#endif

// chase intersections and map ones with merge id to master id
// if points are both ends, mark it small
void OpSegment::mergeMultiple(OpPoint masterPt, int masterID, OpPoint mergePt, int mergeID) {
	// iterate through all owned contours
//	OP_ASSERT(masterPt != mergePt);
	OP_ASSERT(masterID != mergeID);
	for (OpContour* neighbor : contour->overlapOwner->overlaps) {
	// iterate through segments' sects, looking for matching merge pt/id
		for (OpSegment& seg : neighbor->segments) {
			if (seg.disabled)
				continue;
			OP_ASSERT(!seg.sects.unsorted);
			for (OpIntersection* sect : seg.sects.i) {
				if (sect->mergeID == mergeID) {
					OP_ASSERT(sect->ptT.pt == mergePt);
				// change them to master pt, master id
					sect->mergeID = masterID;
					sect->ptT.pt = masterPt;
				}
			}
			if (seg.sects.i.front()->ptT.pt == seg.sects.i.back()->ptT.pt)
				seg.setDisabled(OP_LINE_FILE_NPARGS());
		}
	}
}

PrefFound OpSegment::moveSects(OpPtT match, OpPoint destination) {
	OP_ASSERT(!sects.i.empty());
	sects.sort();
	SectCleanup cleanup = sects.moveSects(match, destination,
			destination == sects.i.front()->ptT.pt ? MatchEnds::start : 
			destination == sects.i.back()->ptT.pt ? MatchEnds::end : MatchEnds::none);
	switch (cleanup) {
		case SectCleanup::none:
			return PrefFound::ok;
		case SectCleanup::sectsRemoved:
			hasCoin = false;
			hasUnsectable = false;
			sects.hasCCSects = false;
			sects.hasPairs = false;
			for (OpIntersection* sect : sects.i) {
				if (sect->collapsed)
					continue;
				hasCoin |= !!sect->coincidenceID;
				hasUnsectable |= !!sect->unsectID;
				sects.hasCCSects |= sect->ccSect;
				sects.hasPairs |= hasCoin | hasUnsectable;
			}
			return PrefFound::ok;
		case SectCleanup::segmentCollapsed:
			setDisabled(OP_LINE_FILE_NPARGS());
			return PrefFound::disabled;
		default:
			OP_ASSERT(0);
			return PrefFound::ok;
	}
}

// two segments are coincident so move opp's winding to this and disabled opp
bool OpSegment::moveWinding(OpSegment* opp, bool backwards) {
	winding.move(opp->winding, backwards);
	opp->winding.zero();
	opp->setDisabled(OP_LINE_FILE_NPARGS());
	OpContour* oContour = opp->contour;
	contour->addMerge(oContour);
	if (winding.visible()) {
#if 0
		if (oContour != contour && coinContours.end() == std::find(coinContours.begin(),
				coinContours.end(), oContour))
			coinContours.push_back(oContour);
#endif
		return true;
	}
	setDisabled(OP_LINE_FILE_NPARGS());
	return false;
}

int OpSegment::nextID() const {
	return contour->nextID();
}

void OpSegment::normalize() {
    if (c.isSmall) {
		setDisabled(OP_LINE_FILE_NPARGS());
        return;
	}
	// !!! where does curve control point pin happen?
	// since we are leaving original curve alone, can it be postponed or not done at all?
	if (c.start == c.end)
		setDisabled(OP_LINE_FILE_NPARGS());
}

#if 0
// !!! seems arbitrary; it doesn't prioritize matching pt or t; just first found
OpPtT OpSegment::ptAtT(const OpPtT& match) const {
	OP_ASSERT(!disabled);
	if (match.pt == c.firstPt() || match.t == 0)
		return OpPtT(c.firstPt(), 0);
	if (match.pt == c.lastPt() || match.t == 1)
		return OpPtT(c.lastPt(), 1);
	for (const OpIntersection* sect : sects.i) {
		if (sect->ptT.pt == match.pt || sect->ptT.t == match.t)
			return sect->ptT;
	}
	return OpPtT(SetToNaN::dummy);
}
#endif

#if 0
void OpSegment::remap(OpPoint oldAlias, OpPoint newAlias) {
	if (oldAlias == c.firstPt()) {
		movePt({ oldAlias, 0 }, newAlias);
		return;
	}
	if (oldAlias == c.lastPt()) {
		movePt({ oldAlias, 1 }, newAlias);
		return;
	}
	for (OpIntersection* sect : sects.i) {
		if (sect->ptT.pt == oldAlias)
			sect->ptT.pt = newAlias;
	}
}
#endif

// this used to remove opposite intersections; leave those in so adjacent segments on either
// side of disabled segment can find each other
void OpSegment::setDisabled(OP_LINE_FILE_NP_ARGS()) {
	disabled = true;  // set only using this helper so debug data is uniform
	OP_LINE_FILE_SET(debugSetDisabled); 
}

void OpSegment::setEndsUnmerged() {
	endsMerged = contour->segEndsMerged = false;
}

void OpSegment::setUnmerged() {
	merged = contour->segMerged = false;
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

OpVector OpSegment::threshold() const {
	return contour->context->threshold; 
}

float OpSegment::thresholdLength() const {
	return contour->context->thresholdLength; 
}

// Note that this must handle a many-to-many relationship between seg and opp.
// Coincident runs of edges may be interrupted by other intersections but their winding is
// unaffected (only other coins may break inner coincident windings).

// If edges are not identical, add filler to connect disabled next to kept coin (testQuads2558209)
void OpSegment::transferCoins() {
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
		// if edge is coincident, transfer windings and unsectable sects
		for (CoinPal& cPal : edge.coinPals) {
			OpSegment* oSeg = cPal.opp;
			OP_ASSERT(oSeg != this);
			if (oSeg->disabled)
				continue;
			int cID = cPal.coinID;
			OP_ASSERT(cID);
			EdgeMatch match = cID > 0 ? EdgeMatch::start : EdgeMatch::end;
			for (OpEdge& oEdge : oSeg->edges) {
				if (edge.startPt() != oEdge.ptT(match).pt)
					continue;
				if (oEdge.disabled)
					continue;
#if OP_DEBUG
				std::vector<CoinPal>& ocPals = oEdge.coinPals;
				auto ocPal = std::find_if(ocPals.begin(), ocPals.end(), [cID]
						(const CoinPal& ocPal){ return ocPal.coinID == cID; });
				OP_ASSERT(ocPals.end() != ocPal);
#endif
//				OP_ASSERT(oEdge.winding.visible());
				OP_ASSERT(!edge.disabled);
				edge.winding.move(oEdge.winding, cID < 0);
				oEdge.winding.zero();
				oEdge.setDisabled(OP_LINE_FILE_NPARGS());
				contour->addMerge(oSeg->contour);
				if (edge.winding.visible())
					break;
				edge.setDisabled(OP_LINE_FILE_NPARGS());
				goto giveUp;
			}
		}
giveUp:
		;
	}
}

#if 0
void OpSegment::tripleSect() {
	if (disabled)
		return;
	OP_DEBUG_CODE(int safetyHatch = 10);
	while (TripleSected::tryAgain == sects.tripleSect()) {
		contour->context->sortIntersections();
		OP_ASSERT(--safetyHatch);
	}
}
#endif

bool OpSegment::zeroSmall(bool zeroStart) {
	return c.zeroSmall(*contour, zeroStart); 
}