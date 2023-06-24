#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpEdges.h"
#include "OpSegment.h"

OpEdges::OpEdges(OpContours& contours, EdgesToSort edgesToSort) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				addEdge(&edge, edgesToSort);
			}
		}
	}
	sort(edgesToSort);
}

OpEdges::OpEdges(OpEdge* sEdge, OpEdge* oEdge) {
	addEdge(sEdge, EdgesToSort::byCenter);
	addEdge(oEdge, EdgesToSort::byCenter);
#if OP_DEBUG_IMAGE
	OpDebugImage::add(sEdge);
	OpDebugImage::add(oEdge);
#endif
}

void OpEdges::addEdge(OpEdge* edge, EdgesToSort edgesToSort) {
	if (!edge->winding.visible())	// skip edges no longer visible
		return;
	if (EdgesToSort::byBox == edgesToSort || edge->ptBounds.height())
		inX.push_back(edge);
	if (EdgesToSort::byCenter == edgesToSort && edge->ptBounds.width())
		inY.push_back(edge);
}

IntersectResult OpEdges::CoincidentCheck(OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
		OpSegment* segment, OpSegment* oppSegment) {
	OpVector abDiff = aPtT.pt - bPtT.pt;
	XyChoice xyChoice = fabsf(abDiff.dx) < fabsf(abDiff.dy) ? XyChoice::inY : XyChoice::inX;
	float A = aPtT.pt.choice(xyChoice);
	float B = bPtT.pt.choice(xyChoice);
	OP_ASSERT(A != B);
	float C = cPtT.pt.choice(xyChoice);
	float D = dPtT.pt.choice(xyChoice);
	OP_ASSERT(C != D);
	bool flipped = A < B != C < D;
	bool AinCD = OpMath::Between(C, A, D);
	bool BinCD = OpMath::Between(C, B, D);
	if (AinCD && BinCD)
		return AddPair(xyChoice, aPtT, bPtT, cPtT, dPtT, flipped, segment, oppSegment);
	bool CinAB = OpMath::Between(A, C, B);
	bool DinAB = OpMath::Between(A, D, B);
	if (CinAB && DinAB)
		return AddPair(xyChoice, cPtT, dPtT, aPtT, bPtT, flipped, oppSegment, segment);
	if (!AinCD && !BinCD)
		return IntersectResult::no;
	OP_ASSERT(CinAB || DinAB);
	float AorB = AinCD ? A : B;
	OpPtT ptTAorB = AinCD ? aPtT : bPtT;
	float CorD = CinAB ? C : D;
	OpPtT ptTCorD = CinAB ? cPtT : dPtT;
	if (AorB == CorD) {
		if (segment->containsIntersection(ptTAorB, oppSegment))
			return IntersectResult::yes;
		OpIntersection* sect = segment->addSegSect(ptTAorB  OP_DEBUG_PARAMS(
				SECT_MAKER(addCoincidentCheck), SectReason::coinPtsMatch, oppSegment));
		OpIntersection* oSect = oppSegment->addSegSect(ptTCorD  OP_DEBUG_PARAMS(
				SECT_MAKER(addCoincidentCheckOpp), SectReason::coinPtsMatch, segment));
		sect->pair(oSect);
		return IntersectResult::yes;
	}
	// pass a mix of seg and opp; construct one t for each
	int coinID = segment->coinID(flipped);
	AddMix(xyChoice, ptTAorB, flipped, cPtT, dPtT, segment, oppSegment, coinID);
	AddMix(xyChoice, ptTCorD, flipped, aPtT, bPtT, oppSegment, segment, coinID);
	return IntersectResult::yes;
}

// this edge has points A, B; opp edge has points C, D
// adds 0: no intersection; 1: end point only; 2: partial or full coincidence
IntersectResult OpEdges::CoincidentCheck(const OpEdge& edge, const OpEdge& opp) {
	return OpEdges::CoincidentCheck(edge.start, edge.end, opp.start, opp.end,
			const_cast<OpSegment*>(edge.segment), const_cast<OpSegment*>(opp.segment));
}

void OpEdges::AddMix(XyChoice xyChoice, OpPtT ptTAorB, bool flipped,
		OpPtT cPtT, OpPtT dPtT, OpSegment* segment, OpSegment* oppSegment, int coinID) {
	float eStart = ptTAorB.pt.choice(xyChoice);
	if (flipped)
		std::swap(cPtT, dPtT);
	float oStart = cPtT.pt.choice(xyChoice);
	float oEnd = dPtT.pt.choice(xyChoice);
	float oTRange = dPtT.t - cPtT.t;
	OpPtT oCoinStart { ptTAorB.pt, cPtT.t + (eStart - oStart) / (oEnd - oStart) * oTRange };
	OP_ASSERT(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
	OpIntersection* sect = segment->addCoin(ptTAorB, coinID  
			OP_DEBUG_PARAMS(SECT_MAKER(addMix), SectReason::coinPtsMatch, oppSegment));
	OpIntersection* oSect = oppSegment->addCoin(oCoinStart, coinID  
			OP_DEBUG_PARAMS(SECT_MAKER(addMixOpp), SectReason::coinPtsMatch, segment));
	sect->pair(oSect);
}

// If we got here because a pair of edges are coincident, that coincidence may have already been
// recorded when the pair of segments were checked, or the intersections may have been computed.
IntersectResult OpEdges::AddPair(XyChoice xyChoice, OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
	bool flipped, OpSegment* segment, OpSegment* oppSegment) {
	std::vector<OpIntersection*> range;
	// set range to contain intersections that match this segment and opposite segment
	segment->intersectRange(oppSegment, range);
	// return existing intersection that matches segment coincident ends
	auto findSect = [&](OpPtT ptT, OpSegment* oppSegment) {	// lambda
		for (auto entry : range) {
			if (entry->ptT.t == ptT.t || entry->ptT.pt == ptT.pt)
				return entry;
		}
		return (OpIntersection*) nullptr;
	};
	// returns index into existing coincidence pairs. Even is outside pair, odd is inside pair.
	// !!! note the return allows us to know if pair of calls to in coin range encompasses one or
	//     more pairs, but we don't take advantage of that yet
	auto inCoinRange = [&](float t, int* coinID) {
		OpIntersection* coinStart = nullptr;
		int index = 0;
		for (auto entry : range) {
			if (!entry->coincidenceID)
				continue;
			if (!coinStart) {
				coinStart = entry;
				++index;
				continue;
			}
			OP_ASSERT(entry->coincidenceID == coinStart->coincidenceID);
			if (coinStart->ptT.t <= t && t <= entry->ptT.t) {
				*coinID = coinStart->coincidenceID;
				return index;
			}
			coinStart = nullptr;
			++index;
		}
		OP_ASSERT(!coinStart);
		*coinID = 0;
		return index;
	};
	int coinID = segment->coinID(flipped);	// generate an id, although it may go unused
	// find the intersection, or make one, if it is not within an existing coincident run
	OpIntersection* sect1 = findSect(aPtT, oppSegment);
	OpIntersection* sect2 = findSect(bPtT, oppSegment);
	OpIntersection* addedSect1 = nullptr;
	OpIntersection* addedSect2 = nullptr;
	// remember if this t value is in a coincident range, and if so,which one
	int aCoinID;
	int aIndex = inCoinRange(aPtT.t, &aCoinID);
	bool aInCoincidence = aIndex & 1;
	int bCoinID;
	int bIndex = inCoinRange(bPtT.t, &bCoinID);
	bool bInCoincidence = bIndex & 1;
	// assign a new or existing coin id if sect doesn't already have one
	if (sect1) {
		if (!aInCoincidence) {
			OP_ASSERT(!sect1->coincidenceID);
			sect1->coincidenceID = bInCoincidence ? bCoinID : coinID;
		}
	} else if (!aInCoincidence)	// or if it doesn't exist and isn't in a coin range, make one
		sect1 = addedSect1 = segment->addCoin(aPtT, coinID
			OP_DEBUG_PARAMS(SECT_MAKER(addPair_aPtT), SectReason::coinPtsMatch, oppSegment));
	if (sect2) {
		if (!bInCoincidence) {
			OP_ASSERT(!sect2->coincidenceID);
			sect2->coincidenceID = aInCoincidence ? aCoinID : coinID;
		}
	} else if (!bInCoincidence)
		sect2 = addedSect2 = segment->addCoin(bPtT, coinID
			OP_DEBUG_PARAMS(SECT_MAKER(addPair_bPtT), SectReason::coinPtsMatch, oppSegment));
	if (!addedSect1 || !addedSect2) {
		range.clear();
		oppSegment->intersectRange(segment, range);
	}
	// add the opposite that goes with the created segment sect
	float oStart  OP_DEBUG_INITIALIZE_TO_SILENCE_WARNING;
	float oTRange  OP_DEBUG_INITIALIZE_TO_SILENCE_WARNING;
	float oXYRange  OP_DEBUG_INITIALIZE_TO_SILENCE_WARNING;
	int oSect1Coin = 0;
	OpIntersection* oSect1;
	if (addedSect1 || addedSect2) {
		if (flipped)
			std::swap(cPtT, dPtT);
		oStart = cPtT.pt.choice(xyChoice);
		float oEnd = dPtT.pt.choice(xyChoice);
		oTRange = dPtT.t - cPtT.t;
		oXYRange = oEnd - oStart;
	}
	if (addedSect1) {
		float eStart = aPtT.pt.choice(xyChoice);
		OpPtT oCoinStart{ aPtT.pt, cPtT.t + (eStart - oStart) / oXYRange * oTRange };
		OP_ASSERT(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
		oSect1 = oppSegment->addCoin(oCoinStart, coinID
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_oppStart), SectReason::coinPtsMatch, segment));
		sect1->pair(oSect1);
	} else {
		oSect1 = findSect(cPtT, segment);
		int cCoinID;
		int cIndex = inCoinRange(cPtT.t, &cCoinID);
		bool cInCoincidence = cIndex & 1;
		if (!cInCoincidence)
			oSect1Coin = cInCoincidence ? cCoinID : coinID;	// delay for sect 2 call to find sect
	}
	if (addedSect2) {
		float eEnd = bPtT.pt.choice(xyChoice);
		OpPtT oCoinEnd{ bPtT.pt, cPtT.t + (eEnd - oStart) / oXYRange * oTRange };
		OP_ASSERT(OpMath::Between(cPtT.t, oCoinEnd.t, dPtT.t));
		OpIntersection* oSect2 = oppSegment->addCoin(oCoinEnd, coinID
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_oppEnd), SectReason::coinPtsMatch, segment));
		sect2->pair(oSect2);
	} else {
		OpIntersection* oSect2 = findSect(dPtT, segment);
		int dCoinID;
		int dIndex = inCoinRange(dPtT.t, &dCoinID);
		bool dInCoincidence = dIndex & 1;
		if (!dInCoincidence)
			oSect2->coincidenceID = dInCoincidence ? dCoinID : coinID;
	}
	if (oSect1Coin)
		oSect1->coincidenceID = oSect1Coin;
	return IntersectResult::yes;
}

// !!! I'm bothered that segment / segment calls a different form of this
void OpEdges::AddLineCurveIntersection(OpEdge& opp, const OpEdge& edge) {
	auto alreadyContains = [](const std::vector<OpIntersection*>& sects, const OpPtT& edgePtT,
			const OpSegment* segment) {
		for (auto sectPtr : sects) {
			const OpIntersection& sect = *sectPtr;
			if (segment == sect.opp->segment && (edgePtT.pt == sect.ptT.pt || edgePtT.t == sect.ptT.t))
				return true;
		}
		return false;
	};
	OP_ASSERT(opp.segment != edge.segment);
	OpRoots septs;
	OP_ASSERT(edge.isLine_impl);
	LinePts edgePts { edge.start.pt, edge.end.pt };
	const OpCurve& oppCurve = opp.setCurve();
	septs = oppCurve.rayIntersect(edgePts);
//	OP_ASSERT(septs.count < 3);	// while it should never be 3, it's OK to accept those intersections
	if (!septs.count)
		return;
	// Note that coincident check does not receive intercepts as a parameter; in fact, the intercepts
	// were not calculated (the roots are uninitialized). This is because coincident check will 
	// compute the actual coincident start and end without the roots introducing error.
	if (2 == septs.count && opp.setLinear())
		return (void) CoincidentCheck(edge, opp);
	for (unsigned index = 0; index < septs.count; ++index) {
		float oppT = OpMath::Interp(opp.start.t, opp.end.t, septs.get(index));
		OpPtT oppPtT { oppCurve.ptAtT(septs.get(index)), oppT };
		// !!! if match allows correct point not to be contained by edge bounds, document why + keep example
		//     bug5240 test fails if contains test is missing
		if (!edge.ptBounds.contains(oppPtT.pt))
			continue;
		float edgeT;
		FoundPtT foundPtT = edge.segment->findPtT(edge.start.t, edge.end.t, oppPtT.pt, &edgeT);
		if (FoundPtT::multiple == foundPtT)
			return;
		if (OpMath::Between(0, edgeT, 1)) {
            // pin point to both bounds, but only if it is on edge
			OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
			OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
            oSegment->ptBounds.pin(&oppPtT.pt);
            eSegment->ptBounds.pin(&oppPtT.pt);	// !!! doubtful this is ever needed with contains test above
			OpPtT edgePtT { oppPtT.pt, OpMath::Interp(edge.start.t, edge.end.t, edgeT) };
			if (alreadyContains(eSegment->intersections, edgePtT, oSegment))
				OP_ASSERT(alreadyContains(oSegment->intersections, oppPtT, eSegment));
			else {
				OpIntersection* sect = eSegment->addEdgeSect(edgePtT  
						OP_DEBUG_PARAMS(SECT_MAKER(edgeLineCurve), SectReason::lineCurve, 
						&edge, &opp));
				OpIntersection* oSect = oSegment->addEdgeSect(oppPtT  
						OP_DEBUG_PARAMS(SECT_MAKER(edgeLineCurveOpp), SectReason::lineCurve, 
						&edge, &opp));
				sect->pair(oSect);
			}
		}
	}
}

// note: sorts from high to low
struct CompareDistance {
	CompareDistance(Axis a)
		: axis(a) {}

	bool operator()(const EdgeDistance& s1, const EdgeDistance& s2) {
		if (s1.edge->priorSum() == s2.edge && axis == s2.edge->sumAxis
				&& axis == s1.edge->priorSum()->sumAxis)
			return false;
		if (s2.edge->priorSum() == s1.edge && axis == s1.edge->sumAxis
				&& axis == s2.edge->priorSum()->sumAxis)
			return true;
		return s1.distance > s2.distance;
	}
	
	Axis axis;
};

// !!! may duplicate unsectable, which has no loops and may be easier to manage
FoundWindings OpEdges::checkForLoops(Axis axis) {
	std::vector<OpEdge*>& edges = Axis::horizontal == axis ? inX : inY;
	for (OpEdge* edge : edges) {
		edge->active_impl = edge->winding.visible() && edge->fail != EdgeFail::horizontal;
	}
	for (OpEdge* edge : edges) {
		if (!edge->isActive())
			continue;
		OpEdge* loopEnd = edge->hasLoop(WhichLoop::prior, EdgeLoop::sum, LeadingLoop::in);
		if (!loopEnd)
			continue;
		size_t loopyIndex = 0;
		for (OpEdge* test : edges) {
			if (loopEnd == test)
				break;
			++loopyIndex;
		}
		OP_ASSERT(loopyIndex < edges.size());
		std::vector<EdgeDistance> loopyDistances;
		Axis perpendicular = !axis;
		float center = loopEnd->center.pt.choice(perpendicular);
		float normal = loopEnd->center.pt.choice(axis);
		float edgeCenterT = loopEnd->center.t;
		loopEnd->isSumLoop = true;
		FoundIntercept loopyResult = findRayIntercept(loopyIndex, axis, loopEnd, center,
				normal, edgeCenterT, &loopyDistances);
		if (FoundIntercept::fail == loopyResult)
			continue;
		if (FoundIntercept::overflow == loopyResult)
			return FoundWindings::fail;
		std::sort(loopyDistances.begin(), loopyDistances.end(), CompareDistance(axis));	// sorts from high to low
		OpEdge* loopStart = loopEnd->priorSum();
		if (loopyDistances.size() > 1) {
			EdgeDistance& distance = loopyDistances[loopyDistances.size() - 2];
			if (loopEnd != distance.edge)
				loopEnd->setPriorSum(distance.edge);	// will swap with actual loop start later
			else
				loopEnd->setPriorSum(nullptr);
		} else
			loopEnd->setPriorSum(nullptr);
		OpEdge* loopMember = loopEnd;
		OpEdge* loopTravel = loopStart;
		do {
			loopMember->clearActive();
			loopMember->isSumLoop = true;
			if (loopStart != loopMember)
				loopMember->loopStart = loopStart;
			loopMember = loopTravel;
			loopTravel = loopTravel->priorSum();
		} while (loopEnd != loopMember);
	}
	return FoundWindings::yes;
}

#if 0
// maybe (some of) edge properties must be set (sumNormal appears to be only used here)
// if edge windings are mostly computed on first pass, perhaps not
// suspect this is mostly unneeded
void OpEdges::checkForLoopy(EdgeDistance* last, Axis axis, std::vector<EdgeDistance>& distance) {
	/* if this edge found an edge to the right, and the right edge has already been summed
	   and, that right edge's center overlaps this edge, and the right edge did not
	   see this edge, then mark both as loopy */
	OpEdge* edge = last->edge;
	while (++last <= &distance.back()) {
		OpEdge* right = last->edge;
		OP_ASSERT(right != edge);
		OP_ASSERT(last->distance <= 0 || right->priorSum() == edge);
		if (axis != right->sumAxis)
			continue;
		float oneEnd = edge->start.pt.choice(axis);
		float otherEnd = edge->end.pt.choice(axis);
		float sumNormal = right->sumNormal;
		if ((oneEnd >= sumNormal && otherEnd >= sumNormal)
				|| (oneEnd <= sumNormal && otherEnd <= sumNormal))
			continue;
		if (right->containsSum(edge))
			continue;
		if (edge->priorSum() != right->priorSum())
			continue;
		OP_ASSERT(0);	// with unsectable edges, is this ever encountered?
		right->setPriorSum(edge);
		right->sumNormal = edge->sumNormal;
	}
}
#endif

#if 0
FoundIntersections OpEdges::findIntersections() {
#if OP_DEBUG_COMPARE
	OpDebugCompare debugCompare("innerFindSect");
#endif
	for (auto edgeIter = inX.begin(); edgeIter != inX.end(); ++edgeIter) {
		OpEdge* edge = const_cast<OpEdge*>(*edgeIter);
		for (auto oppIter = edgeIter + 1; oppIter != inX.end(); ++oppIter) {
			OpEdge* opp = const_cast<OpEdge*>(*oppIter);
#if OP_DEBUG_COMPARE
			debugCompare.edges(edge, opp);
#endif
	OpDebugBreakIf(edge, 396, 402 == opp->id);
	OpDebugBreakIf(opp, 396, 402 == edge->id);
			if (edge->ptBounds.right < opp->ptBounds.left)
				break;
			if (!edge->ptBounds.intersects(opp->ptBounds))
				continue;
			if (edge->segment == opp->segment && OpType::cubic != edge->segment->c.type)	
				continue;  // non-cubic handled in opsegment makeedges
			// for line-curve intersection we can directly intersect
			if (OpType::line != edge->segment->c.type && edge->setLinear()) {
				AddLineCurveIntersection(*opp, *edge);
				continue;
			} else if (OpType::line != opp->segment->c.type && opp->setLinear()) {
				AddLineCurveIntersection(*edge, *opp);
				continue;
			}
			// if the curves' bounds share only a single intersection edge, check endpoints only
			if (edge->ptBounds.touches(opp->ptBounds)) {
				OpSegment* eSegment = const_cast<OpSegment*>(edge->segment);
				OpSegment* oSegment = const_cast<OpSegment*>(opp->segment);
				for (auto& edgePtT : { edge->start, edge->end }) {
					for (auto& oppPtT : { opp->start, opp->end }) {
						// end matching end was handled earlier in op segments find intersection
						if (edgePtT.pt != oppPtT.pt)
							continue;
						if (eSegment == oSegment && edgePtT.t == oppPtT.t)
							continue;
						if (OpMath::IsInt(edgePtT.t) && OpMath::IsInt(oppPtT.t))
							continue;
						if (eSegment->containsIntersection(edgePtT, oSegment)
								|| oSegment->containsIntersection(oppPtT, eSegment))
							continue;
						OpIntersection* sect = eSegment->addIntersection(edgePtT
								OP_DEBUG_PARAMS(SECT_MAKER(edgeIntersections), 
								SectReason::sharedEndPoint, nullptr, edge, opp));
						OpIntersection* oSect = oSegment->addIntersection(oppPtT
								OP_DEBUG_PARAMS(SECT_MAKER(edgeIntersectionsOpp),
								SectReason::sharedEndPoint, nullptr, opp, edge));
						sect->pair(oSect);
					}
				}
				continue;
			}
			if (OpType::line == edge->segment->c.type || OpType::line == opp->segment->c.type)
				continue;
			OpCurveCurve OpCurveCurve(edge, opp);
			SectFound result = OpCurveCurve.divideAndConquer();
			if (SectFound::fail == result || SectFound::overflow == result)
				return FoundIntersections::fail;
		}
	}
#if OP_DEBUG_COMPARE
	debugCompare.close();
#endif
	return FoundIntersections::yes;
}
#endif

// at some point, do some math or rigorous testing to figure out how extreme this can be
// for now, keep making it smaller until it breaks
#define WINDING_NORMAL_LIMIT  0.001 // !!! no idea what this should be

FoundIntercept OpEdges::findRayIntercept(size_t inIndex, Axis axis, OpEdge* edge, float center,
		float normal, float edgeCenterT, std::vector<EdgeDistance>* distance) {
	bool isLoopy = edge->isSumLoop;	// if loopy, ignore other loopy members
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	Axis perpendicular = !axis;
	OpVector backRay = -ray;
	float mid = .5;
	float midEnd = .5;
	std::vector<OpEdge*>& inArray = Axis::horizontal == axis ? inX : inY;
	do {
		distance->emplace_back(edge, 0, normal, edgeCenterT);
		int index = inIndex;
		// start at edge with left equal to or left of center
		while (index != 0) {
			OpEdge* test = inArray[--index];
			if (test == edge)
				continue;
			if (test->ptBounds.ltChoice(axis) > normal)
				continue;
			if (test->ptBounds.rbChoice(axis) < normal)
				continue;
			if (test->unsortable)
				goto tryADifferentCenter;
			if (!test->winding.visible())
				continue;
			if (isLoopy && edge->inSumLoop(test))
				continue;
			{
				const OpCurve& testCurve = test->setCurve();
				OpRoots cepts = testCurve.axisRayHit(axis, normal);
				// get the normal at the intersect point and see if it is usable
				if (1 != cepts.count) {
					// !!! if intercepts is 2 or 3, figure out why (and what to do)
					// !!! likely need to try a different ray
					OP_ASSERT(0 == cepts.count);
					continue;
				}
				float cept = cepts.get(0);
				if (OpMath::IsNaN(cept) || 0 == cept || 1 == cept)
					goto tryADifferentCenter;
				bool overflow;
				float tNxR = testCurve.tangent(cept).normalize(&overflow).cross(backRay);
				if (overflow)
					OP_DEBUG_FAIL(*test, FoundIntercept::overflow);
				if (fabs(tNxR) < WINDING_NORMAL_LIMIT
					&& (!test->setLinear() || test->start.t >= cept || cept >= test->end.t)) {
					goto tryADifferentCenter;
				}
				OpPoint pt = testCurve.ptAtT(cept);
				distance->emplace_back(test, center - pt.choice(perpendicular), normal, cept);
			}
		}
		return FoundIntercept::yes;
	tryADifferentCenter:
		mid /= 2;
		midEnd = midEnd < .5 ? 1 - mid : mid;
		float middle = OpMath::Interp(edge->ptBounds.ltChoice(axis), edge->ptBounds.rbChoice(axis), 
				midEnd);
		const OpCurve& edgeCurve = edge->setCurve();  // ok to be in loop (lazy)
		edgeCenterT = edgeCurve.center(axis, middle);
		if (OpMath::IsNaN(edgeCenterT) || mid <= 1.f / 256.f) {	// give it at most eight tries
			markUnsortable(edge, axis, ZeroReason::recalcCenter);
			return FoundIntercept::fail;	// nonfatal error (!!! give it a different name!)
		}
		// if find ray intercept can't find, restart with new center, normal, distance, etc.
		center = edgeCurve.ptAtT(edgeCenterT).choice(perpendicular);
		OP_ASSERT(!OpMath::IsNaN(center));
		normal = edgeCurve.ptAtT(edgeCenterT).choice(axis);
		OP_ASSERT(!OpMath::IsNaN(normal));
		distance->clear();
	} while (true);
}

void OpEdges::markUnsortable(OpEdge* edge, Axis axis, ZeroReason reason) {
	if (Axis::vertical == axis || inY.end() == std::find(inY.begin(), inY.end(), edge)) {
		edge->winding.zero(reason);
		edge->unsortable = true;
		edge->segment->contour->contours->unsortables.push_back(edge);
	}
	edge->fail = Axis::vertical == axis ? EdgeFail::vertical : EdgeFail::horizontal;
}

// if horizontal axis, look at rect top/bottom
ChainFail OpEdges::setSumChain(size_t inIndex, Axis axis) {
	// see if normal at center point is in direction of ray
	std::vector<OpEdge*>& inArray = Axis::horizontal == axis ? inX : inY;
	OpEdge* edge = inArray[inIndex];
	OP_ASSERT(edge->winding.visible());
	const OpSegment* edgeSeg = edge->segment;
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	bool overflow;
	float NxR = edgeSeg->c.tangent(edge->center.t).normalize(&overflow).cross(ray);
	if (overflow)
		OP_DEBUG_FAIL(*edge, ChainFail::normalizeOverflow);
	if (fabs(NxR) < WINDING_NORMAL_LIMIT) {
		markUnsortable(edge, axis, ZeroReason::tangentXRay);
		return ChainFail::normalizeUnderflow;  // nonfatal error
	}
	// intersect normal with every edge in the direction of ray until we run out 
	Axis perpendicular = !axis;
	float center = edge->center.pt.choice(perpendicular);
	float normal = edge->center.pt.choice(axis);
	if (normal == edge->start.pt.choice(axis)
			|| normal == edge->end.pt.choice(axis)) {
		markUnsortable(edge, axis, ZeroReason::noNormal);
		return ChainFail::noNormal;  // nonfatal error
	}
	float edgeCenterT = edge->center.t;
	// advance to furthest that could influence the sum winding of this edge
	inIndex += 1;
	for (; inIndex < inArray.size(); ++inIndex) {
		OpEdge* advance = inArray[inIndex];
		if (advance->ptBounds.ltChoice(perpendicular) > center)
			break;
	}
	std::vector<EdgeDistance> distance;
//	OpDebugPlayback(edge, 41, true);
	FoundIntercept foundIntercept = findRayIntercept(inIndex, axis, edge, center, normal,
			edgeCenterT, &distance);
	OP_ASSERT(distance.size());
	if (FoundIntercept::fail == foundIntercept)
		return ChainFail::failIntercept;
	if (FoundIntercept::overflow == foundIntercept)
		return ChainFail::normalizeOverflow;
	OP_ASSERT(Axis::vertical == axis || FoundIntercept::set != foundIntercept);
	std::sort(distance.begin(), distance.end(), CompareDistance(axis));	// sorts from high to low
	// if edge is simple (not unsectable)
	// walk from low to high (backwards) until simple edge with sum winding is found
	// if edge found is unsectable, don't set its sum winding, but accumulate its winding for edge
	if (!edge->unsectable && ResolveWinding::fail == setWindingByDistance(axis, distance))
		return ChainFail::failIntercept;	// !!! replace with unique fail state
	EdgeDistance* last = &distance.front();
	if (last->edge == edge) {
		edge->sumAxis = axis;
		// sumT is not set because it applies to the prior edge, and there is none
		return ChainFail::none;
	}
	// leftmost edge may have no prior
	while (last->edge != edge) {
		EdgeDistance* next = last + 1;
		OpEdge* nextEdge = next->edge;
		if (Axis::neither == nextEdge->sumAxis) {
			nextEdge->setPriorSum(last->edge);
			// note that these are associated with next, not last, because multiple different 
			//	  'next's may have rays that intersect last
			nextEdge->sumAxis = axis;
//			nextEdge->sumNormal = last->normal;  // for horizontal axis, y value of intersecting ray
			nextEdge->sumT = last->t;
		}
		last = next;
	}

//	checkForLoopy(last, axis, distance);	// don't think it's right; rewrite if needed
	return ChainFail::none;
}

// distance array may include edges to right. Remove code that collects that?
ResolveWinding OpEdges::setWindingByDistance(Axis axis, std::vector<EdgeDistance>& distance) {
	if (1 == distance.size())
		return CalcFail::fail == distance[0].edge->calcWinding(axis, distance[0].t) ?
				ResolveWinding::fail : ResolveWinding::resolved;
	// find edge; then walk backwards to first known sum 
	int firstIndex = -1;	// result if no sum is found
	int edgeIndex = -1;
	for (auto distIter = distance.rbegin(); distIter != distance.rend(); ++distIter) {
		if (distIter->distance < 0)
			continue;
		if (edgeIndex < 0) {
			if (distIter->edge->unsectable)
				return ResolveWinding::loop;	// !!! not really a loop, add a different label
			OP_ASSERT(0 == distIter->distance);
			edgeIndex = &*distIter - &distance.front();
			continue;
		}
		if (0 == distIter->distance)	   // If more than one candidate has a distance of zero,
			return ResolveWinding::loop;   //  winding cannot be resolved.
		OP_ASSERT(distIter->distance > 0);
		OpEdge* prior = distIter->edge;
		if (prior->unsectable) {		// do not start with unsectable winding
			OP_ASSERT(!prior->sum.isSet());
			continue;
		}
		if (!prior->sum.isSet())
			continue;
		firstIndex = &*distIter - &distance.front();
		break;
	}
	// starting with found or zero if none, accumulate sum up to winding
	OpWinding sumWinding(WindingEdge::dummy);	// sets left/right to zero
	OP_DEBUG_CODE(sumWinding.debugType = WindingType::sum);
	if (firstIndex >= 0) {
		OpEdge* prior = distance[firstIndex].edge;
		sumWinding = prior->sum;
		// if pointing down/left, subtract winding
		if (CalcFail::fail == prior->subIfDL(axis, distance[firstIndex].t, &sumWinding))  
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
	}
	++firstIndex;
	for (auto distIter = distance.begin() + firstIndex; distIter != distance.begin() + edgeIndex;
			++distIter) {
		OpEdge* prior = distIter->edge;
		if (CalcFail::fail == prior->addSub(axis, distIter->t, &sumWinding)) // if d/l sub; if u/r add
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
		if (distIter->edge->unsectable)
			continue;
		prior->setSum(sumWinding);
		if (CalcFail::fail == prior->subIfDL(axis, distIter->t, &prior->sum))
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
	}
	OpEdge* edge = distance[edgeIndex].edge;
	edge->setSum(sumWinding);
	if (CalcFail::fail == edge->addIfUR(axis, distance[edgeIndex].t))
		OP_DEBUG_FAIL(*edge, ResolveWinding::fail);
	return ResolveWinding::resolved;
}

FoundWindings OpEdges::setWindings(OpContours* contours) {
	// test sum chain for correctness; recompute if prior or next are inconsistent
	for (Axis axis : { Axis::horizontal, Axis::vertical }) {
		std::vector<OpEdge*>& edges = Axis::horizontal == axis ? inX : inY;
		for (size_t index = 0; index < edges.size(); ++index) {
			OpEdge* edge = edges[index];
			OpDebugPlayback(edge, 463);
			OpDebugPlayback(edge, 440);
			if (Axis::neither != edge->sumAxis)
				continue;
			if (!edge->winding.visible())	// may not be visible in vertical pass
				continue;
			if (EdgeFail::horizontal == edge->fail && Axis::vertical == axis)
				edge->fail = EdgeFail::none;
			ChainFail chainFail = setSumChain(index, axis);
			if (ChainFail::normalizeOverflow == chainFail)
				OP_DEBUG_FAIL(*edge, FoundWindings::fail);
		}
#if OP_DEBUG
		static bool debugNoLoopCheckNeeded = true;
		if (debugNoLoopCheckNeeded)
			for (OpEdge* edge : edges)
				OP_ASSERT(!edge->hasLoop(WhichLoop::prior, EdgeLoop::sum, LeadingLoop::in));
		else
#endif
			checkForLoops(axis);	// if unneeded, remove isSumLoop
		for (size_t index = 0; index < edges.size(); ++index) {
			OpEdge* edge = edges[index];
			if (!edge->winding.visible())
				continue;
			if (edge->fail == EdgeFail::horizontal)
				continue;
			if (edge->sum.isSet())
				continue;
			if (edge->unsectable)
				continue;
			if (edge->unsortable)
				continue;
			if (!edge->isSumLoop) {
				OpEdge* prior = edge->priorSum();
				if (prior && prior->isSumLoop && prior->loopStart && !prior->inSumLoop(edge))
					edge->setPriorSum(prior->loopStart);
			}
			OP_DEBUG_CODE(int debugWindingLimiter = 0);
			// this is wrong: but the correct t value was lost when the array of distances went away
			// while we can store that in the edge for this use, wait to see if this code is still
			// needed. If not, unwind dependencies (e.g., remove sumT)
			ResolveWinding resolve = edge->findWinding(axis, edge->center.t  
					OP_DEBUG_PARAMS(&debugWindingLimiter));
			if (ResolveWinding::fail == resolve)
				OP_DEBUG_FAIL(*edge, FoundWindings::fail);
		}
		for (auto& edge : edges) {
			if (!edge->winding.visible())
				continue;
			if (edge->sum.isSet())
				continue;
			if (edge->unsectable)
				continue;
			if (edge->unsortable)
				continue;
			if (edge->fail == EdgeFail::horizontal)
				continue;
			OP_DEBUG_FAIL(*edge, FoundWindings::fail);
		}
	}
	return FoundWindings::yes;
}

static bool compareXBox(const OpEdge* s1, const OpEdge* s2) {
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

// starting at left (-x), increasing
static bool compareXCenter(const OpEdge* s1, const OpEdge* s2) {
	return s1->ptBounds.left < s2->ptBounds.left;
}

// starting at top (-y), increasing
static bool compareYCenter(const OpEdge* s1, const OpEdge* s2) {
	return s1->ptBounds.top < s2->ptBounds.top;
}

void OpEdges::sort(EdgesToSort sortBy) {
	if (EdgesToSort::byBox == sortBy) {
		std::sort(inX.begin(), inX.end(), compareXBox);
		return;
	}
	OP_ASSERT(EdgesToSort::byCenter == sortBy);
	std::sort(inX.begin(), inX.end(), compareXCenter);
	std::sort(inY.begin(), inY.end(), compareYCenter);
}
