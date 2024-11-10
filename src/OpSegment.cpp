// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurve.h"
#include "OpSegment.h"
#include "OpWinder.h"

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
	if (!c.isFinite()) {
		contour->contours->setError(PathOpsV0Lib::ContextError::finite  OP_DEBUG_PARAMS(id));
		disabled = true;
	} else {
		contour->contours->addToBounds(c);
		ptBounds = c.ptBounds();
		closeBounds = ptBounds;  // no threshold until all segment bounds are set
	}
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
	if (nextDoor->hasLinkTo(neighbor))
		return false;
	if (Unsortable::none != edge->isUnsortable || edge->windZero == nextDoor->windZero 
			|| Unsortable::none != nextDoor->isUnsortable) {
		oppEdges.emplace_back(nextDoor, EdgeMatch::none);
		return nextDoor->isUnsectable();
	}
	return false;
}

void OpSegment::addAlias(OpPoint original, OpPoint alias) {
	contour->contours->addAlias(original, alias);
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
		// !!! commented out still may be necessary, but contains may need to be coinContains?
//	if (sects.contains(ptT, oSeg))  // triggered by fuzz763_13
//		return nullptr;
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
		sects.unsorted = true;
		return sect;
	}
	return sects.add(contour->addUnsect(ptT, this, usectID, end  OP_LINE_FILE_CALLER(oSeg)));
}

OpPtT OpSegment::alignToEnd(OpPoint oppPt) const {
	OpPtT segPtT(SetToNaN::dummy);
	OpPtAliases& aliases = contour->contours->aliases;
	if (c.firstPt().isNearly(oppPt, threshold()) 
			|| (startMoved && aliases.isSmall(c.firstPt(), oppPt)))
		segPtT = { c.firstPt(), 0 };
	else if (c.lastPt().isNearly(oppPt, threshold()) 
			|| (endMoved  && aliases.isSmall(c.lastPt(), oppPt)))
		segPtT = { c.lastPt(), 1 };
	return segPtT;
}

void OpSegment::apply() {
	for (auto& edge : edges)
		edge.apply();
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

		// walk sects in coincidences;
		// look at opp sects (reversed if necessary)
		// if opp sects is missing this point, add it
void OpSegment::betweenIntersections() {
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
		OpPoint ptC = sectC->ptT.pt;
		OpPtT ptTa(SetToNaN::dummy);
		OpIntersection* cInA = segA->sects.coinContains(ptC, segC, &ptTa);
		if (cInA && cInA->coincidenceID)
			return;
		if (OpMath::IsNaN(ptTa.t))
			ptTa = OpPtT(ptC, segA->c.match(0, 1, ptC));
		if (OpMath::IsNaN(ptTa.t))
			return;
		OpPoint ptA = ptTa.pt;
		OpPtT ptTc;
		OpIntersection* aInC = segC->sects.coinContains(ptA, segA, &ptTc);
		if (aInC && aInC->coincidenceID)
			return;
		if (OpMath::IsNaN(ptTc.t))
			ptTc = OpPtT(ptA, segC->c.match(0, 1, ptA));
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
		OpPoint oppPt = bEnd->ptT.pt;
		OpPtT ptTc;
		OpIntersection* aInC = segC->sects.coinContains(oppPt, segA, &ptTc);
		if (aInC && aInC->coincidenceID)
			return;
		if (OpMath::IsNaN(ptTc.t))
			ptTc = OpPtT(oppPt, segC->c.match(0, 1, oppPt));
		if (OpMath::IsNaN(ptTc.t))
			return;
		OpPoint ptC = ptTc.pt;
		OpPtT ptTa(SetToNaN::dummy);
		OpIntersection* cInA = segA->sects.coinContains(ptC, segC, &ptTa);
		if (cInA && cInA->coincidenceID)
			return;
		if (OpMath::IsNaN(ptTa.t))
			ptTa = OpPtT(ptC, segA->c.match(0, 1, ptC));
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
		auto setSect = [coinID](OpIntersection*& cInA, const OpPtT& aPtT,
				OpSegment* segA, MatchEnds match, OpSegment* segC  OP_LINE_FILE_DEF()) {
			if (cInA)
				cInA->setCoin(coinID, match);
			else
				cInA = segA->addCoin(aPtT, coinID, match, segC  OP_LINE_FILE_PARAMS());
		};
		setSect(miss.aSSect, miss.aStart, segA, MatchEnds::start, segC  OP_LINE_FILE_PARAMS());
		setSect(miss.cSSect, miss.cStart, segC, MatchEnds::start, segA  OP_LINE_FILE_PARAMS());
		setSect(aInC, ptTa, segA, MatchEnds::end, segC  OP_LINE_FILE_PARAMS());
		setSect(cInA, ptTc, segC, MatchEnds::end, segA  OP_LINE_FILE_PARAMS());
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

// returns point that matches input; returned point may be nearby; may already be aliased
// segment is unchanged, but equal and nearby point are added to aliases
SegPt OpSegment::checkAliases(OpPtT match) {
	OP_ASSERT(0 == match.t || 1 == match.t);
	OpPoint endPt = 0 == match.t ? c.firstPt() : c.lastPt();
	SegPt result;
	if (0 == match.t ? startMoved : endMoved) {
		if (match.pt == endPt)
			result = { endPt, PtType::isAlias };
		else {
			addAlias(match.pt, endPt);
			result = { endPt, PtType::original };
		}
	} else {
		result = contour->contours->aliases.addIfClose(match.pt);
		if (endPt != result.pt) {
			addAlias(endPt, result.pt);
			result.ptType = PtType::mapSegment;
		}
	}
	// collect all intersections that match; add all matches to aliases
	for (OpIntersection* sect : sects.i) {
		if (sect->ptT.t != match.t)
			continue;
		if (sect->ptT.pt != result.pt) {
			addAlias(sect->ptT.pt, result.pt);
			result.ptType = PtType::mapSegment;
		}
	}
	return result;
}

int OpSegment::coinID(bool flipped) {
	int coinID = nextID();
	hasCoin = true;
	return flipped ? -coinID : coinID;
}

void OpSegment::disableSmall() {
	if (disabled)
		return;
	if (willDisable || isSmall() || !sects.i.size())
		setDisabled(OP_LINE_FILE_NPARAMS());
	OpPoint pt = sects.i[0]->ptT.pt;
	for (size_t index = 1; index < sects.i.size(); ++index) {
		if (pt != sects.i[index]->ptT.pt)
			return;
	}
	setDisabled(OP_LINE_FILE_NPARAMS());
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

// rarely, moving points prevents finding matching ends. If there is no end, do an exhaustive search
void OpSegment::findMissingEnds() {
	if (willDisable && !disabled)
		setDisabled(OP_LINE_FILE_NPARAMS());
	if (disabled)
		return;
	OP_ASSERT(!sects.unsorted);
	OpContours* contours = contour->contours;
	if (!sects.i.size() || 0 != sects.i.front()->ptT.t) {
		OpPoint alias = contours->aliases.existing(c.firstPt());
		OP_ASSERT(alias != c.firstPt());
		for (OpContour* cont : contours->contours) {
			for (OpSegment& opp : cont->segments) {
				if (opp.disabled)
					continue;
				if (opp.c.firstPt() == alias) {
					OpIntersection* sect = addSegSect(OpPtT(alias, 0), &opp  OP_LINE_FILE_PARAMS());
					OpIntersection* oSect = opp.addSegSect(OpPtT(alias, 0), this  OP_LINE_FILE_PARAMS());
					sect->pair(oSect);
					sects.sort();
					opp.sects.sort();

				} 
				if (opp.c.lastPt() == alias) {
					OpIntersection* sect = addSegSect(OpPtT(alias, 0), &opp  OP_LINE_FILE_PARAMS());
					OpIntersection* oSect = opp.addSegSect(OpPtT(alias, 1), this  OP_LINE_FILE_PARAMS());
					sect->pair(oSect);
					sects.sort();
					opp.sects.sort();
				}
			}
		}
	}
	if (!sects.i.size() || 1 != sects.i.back()->ptT.t) {
		OpPoint alias = contours->aliases.existing(c.lastPt());
		OP_ASSERT(alias != c.lastPt());
		for (OpContour* cont : contours->contours) {
			for (OpSegment& opp : cont->segments) {
				if (opp.disabled)
					continue;
				if (opp.c.firstPt() == alias) {
					OpIntersection* sect = addSegSect(OpPtT(alias, 1), &opp  OP_LINE_FILE_PARAMS());
					OpIntersection* oSect = opp.addSegSect(OpPtT(alias, 0), this  OP_LINE_FILE_PARAMS());
					sect->pair(oSect);
					sects.sort();
					opp.sects.sort();

				} 
				if (opp.c.lastPt() == alias) {
					OpIntersection* sect = addSegSect(OpPtT(alias, 1), &opp  OP_LINE_FILE_PARAMS());
					OpIntersection* oSect = opp.addSegSect(OpPtT(alias, 1), this  OP_LINE_FILE_PARAMS());
					sect->pair(oSect);
					sects.sort();
					opp.sects.sort();
				}
			}
		}
	}
	if (startMoved || endMoved)
		resetBounds();
}

// returns t iff opp point is between start and end
// start/end range is necessary since cubics can have more than one t at a point
float OpSegment::findValidT(float start, float end, OpPoint opp) {
	if (!c.isLine()) {
		OpRoots hRoots = c.axisRayHit(Axis::horizontal, opp.y, start, end);
		OpRoots vRoots = c.axisRayHit(Axis::vertical, opp.x, start, end);
		if (1 != hRoots.count && 1 != vRoots.count) {
			if (0 == start && opp.isNearly(c.firstPt(), threshold()))
				return 0;
			if (1 == end && opp.isNearly(c.lastPt(), threshold()))
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

bool OpSegment::isSmall() {
	return contour->contours->aliases.isSmall(c.c.data->start, c.c.data->end);
}

void OpSegment::makeCoins() {
	if (disabled)
		return;
	sects.sort();  // make coins may add sects; so another segment may have added to this one
	auto nextSect = [this](unsigned* index, float t, bool alwaysAdvance) {
		do {
			if (alwaysAdvance) {
				alwaysAdvance = false;
				continue;
			}
			OpIntersection* result = sects.i[*index];
			if (result->ptT.t < t)
				continue;
			if (MatchEnds::none == result->coinEnd)
				return result;
		} while (++*index < sects.i.size());
		return (OpIntersection*) nullptr;
	};
	unsigned startIndex = 0;
	for (OpEdge& edge : edges) {
		if (edge.disabled /* || edge.isUnsectable() */ || !edge.isLine())
			continue;
		OpIntersection* startSect = nextSect(&startIndex, edge.startT, false);
		if (!startSect)
			break;
		if (startSect->ptT.t > edge.startT)
			continue;
		unsigned firstEnd = startIndex;
		OpIntersection* endSect = nextSect(&firstEnd, edge.endT, false);
		if (!endSect)
			break;
		if (endSect->ptT.t > edge.endT)
			continue;
		OP_ASSERT(startSect->ptT.t == edge.startT);
		OP_ASSERT(endSect->ptT.t == edge.endT);
//		if (MatchEnds::none != startSect->unsectEnd && MatchEnds::none != endSect->unsectEnd)
//			continue;
		unsigned endIndex = firstEnd;
		do {
			int oppSegID = endSect->opp->segment->id;
			while (startSect->opp->segment->id < oppSegID) {
				startSect = nextSect(&startIndex, edge.startT, true);
				if (!startSect)
					return;
				if (startSect->ptT.t > edge.startT)
					goto nextEdge;
			}
			oppSegID = startSect->opp->segment->id;
			while (endSect->opp->segment->id < oppSegID) {
				endSect = nextSect(&endIndex, edge.endT, true);
				if (!endSect)
					return;
				if (endSect->ptT.t > edge.endT)
					goto nextEdge;
			}
			OpIntersection* oppStart = startSect->opp;
			OpIntersection* oppEnd = endSect->opp;
			OpSegment* oppSegment = oppStart->segment;
			if (oppSegment != oppEnd->segment)
				continue;
			bool reversed = oppStart->ptT.t > oppEnd->ptT.t;
			for (OpEdge& oppEdge : oppSegment->edges) {
				if (!oppEdge.isLine())
					continue;
				if (reversed ? edge.start().pt == oppEdge.end().pt 
						: edge.start().pt == oppEdge.start().pt) {

					OpWinder::CoincidentCheck(edge, oppEdge);
					break;
				}
			}
			endSect = nextSect(&endIndex, edge.endT, true);
		} while (endSect && edge.endT == endSect->ptT.t);
nextEdge:
		startIndex = firstEnd;
	}
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

OpPtT OpSegment::matchEnd(OpPoint opp) const {
	OpPtT alignedEnd = alignToEnd(opp);
	if (OpMath::IsNaN(alignedEnd.t))
		alignedEnd = { opp, c.match(0, 1, opp) };
	if (!OpMath::IsNaN(alignedEnd.t))
		alignedEnd.t = OpMath::PinNear(alignedEnd.t);
	return alignedEnd;
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

OpPoint OpSegment::mergePoints(OpPtT segPtT, OpSegment* opp, OpPtT oppPtT) {
	SegPt segPt = checkAliases(segPtT);
	SegPt oppPt = opp->checkAliases(oppPtT);
	if (segPt.pt != oppPt.pt && PtType::noMatch != segPt.ptType && PtType::noMatch != oppPt.ptType)
		return remapPts(oppPt.pt, segPt.pt);
	if (PtType::noMatch == segPt.ptType && PtType::noMatch == oppPt.ptType) {
		if (oppPt.pt == segPt.pt)
			return segPt.pt;
		addAlias(oppPt.pt, segPt.pt);
		segPt.ptType = PtType::isAlias;
		oppPt.ptType = PtType::original;
	}
	OpPoint destPt = PtType::noMatch == segPt.ptType ? oppPt.pt : segPt.pt;
	if (segPtT.pt != segPt.pt || PtType::mapSegment == segPt.ptType)
		movePt(segPtT, destPt);
	if (oppPtT.pt != segPt.pt || PtType::mapSegment == oppPt.ptType)
		opp->movePt(oppPtT, destPt);
	return segPt.pt;
}

// keep control point inside curve bounds
// further, if old control point is axis aligned with end point, keep relationship after moving
// detect if segment collapses to point?
OpPoint OpSegment::movePt(OpPtT match, OpPoint destination) {
	OP_ASSERT(0 == match.t || 1 == match.t);
	// if end point and equal point are both aliases (rare), do a global remap of all points so 
	// that the two are combined into a single alias
	if (0 == match.t) {
		 c.setFirstPt(destination);
		 startMoved = true;
	} else {
		 c.setLastPt(destination);
		 endMoved = true;
	}
	c.pinCtrl();
// defer disabling until all moves are complete; disable small segments will clean up
   if (c.firstPt() == c.lastPt())
		willDisable = true;
//    setBounds();  // defer fixing in middle of finding intersections, which uses sorted bounds
	if (match.pt != destination)
		sects.moveSects(match, destination);
	edges.clear();
	return destination;
}

// two segments are coincident so move opp's winding to this and disabled opp
bool OpSegment::moveWinding(OpSegment* opp, bool backwards) {
	winding.move(opp->winding, backwards);
	opp->winding.zero();
	opp->setDisabled(OP_LINE_FILE_NPARAMS());
	if (winding.visible())
		return true;
	setDisabled(OP_LINE_FILE_NPARAMS());
	return false;
}

int OpSegment::nextID() const {
	return contour->nextID();
}

void OpSegment::normalize() {
	c.normalize(); 
	resetBounds();
}

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

OpPoint OpSegment::remapPts(OpPoint oldAlias, OpPoint newAlias) {
	return contour->contours->remapPts(oldAlias, newAlias);
}

void OpSegment::resetBounds() {
	ptBounds = c.ptBounds();
	if (ptBounds.isEmpty()) {
		disabled = true;
		closeBounds = ptBounds;
	} else
		closeBounds = ptBounds.outset(threshold());
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

OpVector OpSegment::threshold() const {
	return contour->contours->threshold(); 
}

// Note that this must handle a many-to-many relationship between seg and opp.
// Coincident runs of edges may be interrupted by other intersections but their winding is
// unaffected (only other coins may break inner coincident windings).
void OpSegment::transferCoins() {
	OP_DEBUG_CONTEXT();
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
				edge.winding.move(oEdge.winding, cID < 0);
				oEdge.setDisabledZero(OP_LINE_FILE_NPARAMS());
				break;
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
