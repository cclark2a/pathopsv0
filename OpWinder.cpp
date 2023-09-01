#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include "PathOps.h"

OpWinder::OpWinder(OpContours& contours, EdgesToSort edgesToSort) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				addEdge(&edge, edgesToSort);
			}
		}
	}
	sort(edgesToSort);
	workingAxis = Axis::neither;
}

OpWinder::OpWinder(OpEdge* sEdge, OpEdge* oEdge) {
	addEdge(sEdge, EdgesToSort::byCenter);
	addEdge(oEdge, EdgesToSort::byCenter);
#if OP_DEBUG_IMAGE
	OpDebugImage::add(sEdge);
	OpDebugImage::add(oEdge);
#endif
	workingAxis = Axis::neither;
}

void OpWinder::addEdge(OpEdge* edge, EdgesToSort edgesToSort) {
	if (edge->disabled)
		return;
	if (EdgesToSort::byBox == edgesToSort || edge->ptBounds.height())
		inX.push_back(edge);
	if (EdgesToSort::byCenter == edgesToSort && edge->ptBounds.width())
		inY.push_back(edge);
}

IntersectResult OpWinder::CoincidentCheck(OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
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
		if (segment->sects.contains(ptTAorB, oppSegment))
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
IntersectResult OpWinder::CoincidentCheck(const OpEdge& edge, const OpEdge& opp) {
	return OpWinder::CoincidentCheck(edge.start, edge.end, opp.start, opp.end,
			const_cast<OpSegment*>(edge.segment), const_cast<OpSegment*>(opp.segment));
}

void OpWinder::AddMix(XyChoice xyChoice, OpPtT ptTAorB, bool flipped,
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
IntersectResult OpWinder::AddPair(XyChoice xyChoice, OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
	bool flipped, OpSegment* segment, OpSegment* oppSegment) {
	// set range to contain intersections that match this segment and opposite segment
	std::vector<OpIntersection*> range = segment->sects.range(oppSegment);
	// return existing intersection that matches segment coincident ends
	auto findSect = [](const std::vector<OpIntersection*>& range, OpPtT ptT) {	// lambda
		for (auto entry : range) {
			if (entry->ptT.t == ptT.t || entry->ptT.pt == ptT.pt)
				return entry;
		}
		return (OpIntersection*) nullptr;
	};
	// returns index into existing coincidence pairs. Even is outside pair, odd is inside pair.
	// !!! note the return allows us to know if pair of calls to in coin range encompasses one or
	//     more pairs, but we don't take advantage of that yet
	auto inCoinRange = [](const std::vector<OpIntersection*>& range, float t, int* coinID) {
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
				if (coinID) {
					OP_ASSERT(!*coinID || *coinID == coinStart->coincidenceID);	 // !!! assert means two coin runs; is that possible?
					*coinID = coinStart->coincidenceID;
				}
				break;
			}
			coinStart = nullptr;
			++index;
		}
		return (bool) (index & 1);
	};
	// find the intersection, or make one, if it is not within an existing coincident run
	OpIntersection* sect1 = findSect(range, aPtT);
	OpIntersection* sect2 = findSect(range, bPtT);
	// remember if this t value is in a coincident range, and if so, which one
	int coinID = 0;
	bool aInCoincidence = inCoinRange(range, aPtT.t, &coinID);
	bool bInCoincidence = inCoinRange(range, bPtT.t, &coinID);
	if (!coinID)
		coinID = segment->coinID(flipped);
	else
		OP_ASSERT(flipped ? coinID < 0 : coinID > 0);	// should never assert
	// assign a new or existing coin id if sect doesn't already have one
	if (!aInCoincidence) {
		if (sect1) {
			OP_ASSERT(!sect1->coincidenceID);
			sect1->coincidenceID = coinID;
		} else	// or if it doesn't exist and isn't in a coin range, make one
			sect1 = segment->addCoin(aPtT, coinID
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_aPtT), SectReason::coinPtsMatch, oppSegment));
	}
	if (!bInCoincidence) {
		if (sect2) {
			OP_ASSERT(!sect2->coincidenceID);
			sect2->coincidenceID = coinID;
		} else
			sect2 = segment->addCoin(bPtT, coinID
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_bPtT), SectReason::coinPtsMatch, oppSegment));
	}
	std::vector<OpIntersection*> oRange = oppSegment->sects.range(segment);
	OpIntersection* oSect1 = findSect(oRange, { aPtT.pt, -1 });
	OpIntersection* oSect2 = findSect(oRange, { bPtT.pt, -1 });
	// add the opposite that goes with the created segment sect
	float oStart = 0; // !!! initialize to silence warning
	float oTRange = 0; // !!! initialize to silence warning
	float oXYRange = 0; // !!! initialize to silence warning
	if (!oSect1 || !oSect2) {
		if (flipped)
			std::swap(cPtT, dPtT);
		oStart = cPtT.pt.choice(xyChoice);
		float oEnd = dPtT.pt.choice(xyChoice);
		oTRange = dPtT.t - cPtT.t;
		oXYRange = oEnd - oStart;
	}
	if (!oSect1) {
		float eStart = aPtT.pt.choice(xyChoice);
		OpPtT oCoinStart{ aPtT.pt, cPtT.t + (eStart - oStart) / oXYRange * oTRange };
		OP_ASSERT(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
		oSect1 = oppSegment->addCoin(oCoinStart, coinID
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_oppStart), SectReason::coinPtsMatch, segment));
		sect1->pair(oSect1);
	} else if (!(inCoinRange(oRange, oSect1->ptT.t, nullptr) & 1))
		oSect1->coincidenceID = coinID;
	if (!oSect2) {
		float eEnd = bPtT.pt.choice(xyChoice);
		OpPtT oCoinEnd{ bPtT.pt, cPtT.t + (eEnd - oStart) / oXYRange * oTRange };
		OP_ASSERT(OpMath::Between(cPtT.t, oCoinEnd.t, dPtT.t));
		oSect2 = oppSegment->addCoin(oCoinEnd, coinID
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_oppEnd), SectReason::coinPtsMatch, segment));
		sect2->pair(oSect2);
	} else if (!(inCoinRange(oRange, oSect2->ptT.t, nullptr) & 1))
		oSect2->coincidenceID = coinID;
	return IntersectResult::yes;
}


// !!! conditionally, upscale t so we can call segment line curve intersection to see what 
//     bugs that introduces
// !!! I'm bothered that segment / segment calls a different form of this
void OpWinder::AddLineCurveIntersection(OpEdge& opp, const OpEdge& edge) {
	OP_ASSERT(opp.segment != edge.segment);
	OP_ASSERT(edge.isLine_impl);
	LinePts edgePts { edge.start.pt, edge.end.pt };
#define USE_SEGMENT_SECT 1
#if USE_SEGMENT_SECT
    OpRoots septs = opp.segment->c.rayIntersect(edgePts);
#else
	const OpCurve& oppCurve = opp.setCurve();
	OpRoots septs = oppCurve.rayIntersect(edgePts);
#endif
	if (!septs.count)
		return;
	// Note that coincident check does not receive intercepts as a parameter; in fact, the intercepts
	// were not calculated (the roots are uninitialized). This is because coincident check will 
	// compute the actual coincident start and end without the roots introducing error.
	if (2 == septs.count && opp.setLinear())
		return (void) CoincidentCheck(edge, opp);
	for (unsigned index = 0; index < septs.count; ++index) {
#if USE_SEGMENT_SECT
		if (opp.start.t > septs.get(index) || septs.get(index) > opp.end.t)
			continue;
		OpPtT oppPtT { opp.segment->c.ptAtT(septs.get(index)), septs.get(index) };
#else
		float oppT = OpMath::Interp(opp.start.t, opp.end.t, septs.get(index));
		OpPtT oppPtT { oppCurve.ptAtT(septs.get(index)), oppT };
#endif
		float edgeT;
#if USE_SEGMENT_SECT
		FoundPtT foundPtT = edge.segment->findPtT(0, 1, oppPtT.pt, &edgeT);
#else
		FoundPtT foundPtT = edge.segment->findPtT(edge.start.t, edge.end.t, oppPtT.pt, &edgeT);
#endif
		if (FoundPtT::multiple == foundPtT)
			return;
		if (!OpMath::Between(0, edgeT, 1))
			continue;
        // pin point to both bounds, but only if it is on edge
		OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
		OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
		OP_DEBUG_CODE(OpPoint debugPt = oppPtT.pt);
        oSegment->ptBounds.pin(&oppPtT.pt);
//      eSegment->ptBounds.pin(&oppPtT.pt);	// !!! doubtful this is needed with contains test above
		OP_ASSERT(debugPt == oppPtT.pt);	// detect if pin is still needed
#if USE_SEGMENT_SECT
		OpPtT edgePtT { oppPtT.pt, edgeT };
#else
		OpPtT edgePtT { oppPtT.pt, OpMath::Interp(edge.start.t, edge.end.t, edgeT) };
#endif
		if (eSegment->sects.alreadyContains(edgePtT, oSegment))
			; // OP_ASSERT(oSegment->debugAlreadyContains(oppPtT.pt, eSegment));	// !!! debug loops51i
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

// !!! restructure to mark all zero distance as unsectableID (if there is more than one) ?
// wait for actual test case where 3 or more edges are unsectable before rewriting
void SectRay::markPals(OpEdge* edge) {
	EdgeDistance* pal = nullptr;
	EdgeDistance* home = nullptr;
	int unsectID = abs(edge->unsectableID);
	for (auto& dist : edge->ray.distances) {
		if (dist.edge == edge)
			home = &dist;
		else if (abs(dist.edge->unsectableID) == unsectID) {
			pal = &dist;
			if (edge->isPal(pal->edge))
				return;
		} else
			continue;
		if (pal && home)
			break;
	}
	OP_ASSERT(home);
	if (!pal) {
		edge->unsectableID = 0;
		edge->unsortable = true;
	} else
		edge->pals.push_back(*pal);
}

// this used to mark all adjacent zero distant edges as pals
// separately, it marks edges with matching unsectableID values as pals
// keep track of adjacent edges in zero distance array, so that they can be paired later
// start here;
// if first edge is smaller than other in the pair, defer marking the other as unsectable, since
// the non-overlapping part may be fine
// just test the non-overlapping part of orderability?
void SectRay::markUnsectableGroups(OpEdge* home) {
	auto homePos = std::find_if(distances.begin(), distances.end(), 
			[&home](const EdgeDistance& dist) {
		return home == dist.edge; });
	auto lesser = std::lower_bound(distances.begin(), distances.end(), *homePos,
			[](const EdgeDistance& a, const EdgeDistance& b) {
			return a.cept < b.cept;
	});
	if (lesser > homePos)
		return;
	auto greater = std::upper_bound(distances.begin(), distances.end(), *homePos,
			[](const EdgeDistance& a, const EdgeDistance& b) {
			return a.cept < b.cept;
	});
	for (auto test = lesser; test < greater; ++test) {
		if (test == homePos)
			continue;
		if (home->unsectableID && abs(home->unsectableID) == abs(test->edge->unsectableID))
			continue;
		auto& testDist = test->edge->ray.distances;
		auto testDistI = std::find_if(testDist.begin(), testDist.end(), 
				[&test](const EdgeDistance& ed) {
			return test->edge == ed.edge;
		});
		auto testHomeI = std::find_if(testDist.begin(), testDist.end(), 
				[&home](const EdgeDistance& ed) {
			return home == ed.edge;
		});
		if (testDist.end() != testHomeI && testDistI->cept == testHomeI->cept)
			home->markUnsectable(test->edge, axis, homePos->t, test->t);
	}
}

// this catches unsectables by keeping track of edges that are found to be adjacent
// each time a ray is cast. If the edge is seen to the left during one ray cast, and to the right
// on another, it is marked as an unsectable pair.
void SectRay::markUnsectables(OpEdge* home) {
	OpDebugBreak(home, 564);
	OpDebugBreak(home, 548);
	EdgeDistance* last = &distances[0];
	for (EdgeDistance* test = &distances[1]; test <= &distances.back(); ++test) {
		auto& lastMore = last->edge->moreRay;
		if (lastMore.end() == std::find(lastMore.begin(), lastMore.end(), test->edge))
			lastMore.push_back(test->edge);
		auto& lastLess = last->edge->lessRay;
		if (lastLess.end() != std::find(lastLess.begin(), lastLess.end(), test->edge))
			goto markIfUnmarked;
		{	
			auto& testLess = test->edge->lessRay;
			if (testLess.end() == std::find(testLess.begin(), testLess.end(), last->edge))
				testLess.push_back(last->edge);
			auto& testMore = test->edge->moreRay;
			if (testMore.end() == std::find(testMore.begin(), testMore.end(), last->edge))
				continue;
		}
	markIfUnmarked:
		if (!test->edge->unsectableID) {
			OP_ASSERT(!last->edge->unsectableID);
			test->edge->markUnsectable(last->edge, axis, test->t, last->t);
		}
	}
#if 0
	// use sorted distances to mark previous and next edges by winding order
	auto homePos = std::find_if(distances.begin(), distances.end(), 
			[&home](const EdgeDistance& dist) {
		return home == dist.edge; });
	OP_ASSERT(distances.end() != homePos);
	// returns iterator to distance with cept just less than home cept, if any
	auto lesser = std::lower_bound(distances.begin(), distances.end(), *homePos,
			[](const EdgeDistance& a, const EdgeDistance& b) {
			return a.cept < nextafterf(b.cept, -OpInfinity);
	});
	if (lesser < homePos) {
		const auto& lesserDist = lesser->edge->ray.distances;
		auto lesserPos = std::find_if(lesserDist.begin(), lesserDist.end(), 
				[&lesser](const EdgeDistance& dist) {
				return lesser->edge == dist.edge; });
		OP_ASSERT(lesserDist.end() != lesserPos);
		auto homeInLesser = std::find_if(lesserDist.begin(), lesserDist.end(), 
				[&home](const EdgeDistance& dist) {
				return home == dist.edge; });
		if (homeInLesser < lesserPos)
			home->markUnsectable(lesserPos->edge, axis, homePos->t, lesserPos->t);
	}
	// returns iterator to distance with cept just more than home cept, if any
	auto greater = std::upper_bound(distances.begin(), distances.end(), *homePos,
			[](const EdgeDistance& a, const EdgeDistance& b) {
			return a.cept < nextafterf(b.cept, OpInfinity);
	});
	if (homePos < greater && greater != distances.end()) {
		const auto& greaterDist = greater->edge->ray.distances;
		auto greaterPos = std::find_if(greaterDist.begin(), greaterDist.end(), 
				[&greater](const EdgeDistance& dist) {
				return greater->edge == dist.edge; });
		OP_ASSERT(greaterDist.end() != greaterPos);
		auto homeInGreater = std::find_if(greaterDist.begin(), greaterDist.end(), 
				[&home](const EdgeDistance& dist) {
				return home == dist.edge; });
		if (homeInGreater > greaterPos)
			home->markUnsectable(greaterPos->edge, axis, homePos->t, greaterPos->t);
	}
#endif
}

bool SectRay::betweenUnsectables(OpEdge* home) {
	// !!! this logic requires uIDs exist in pairs
	// we need to a) use a different convention to represent single edges whose ray has zero distances
	//     and/or b) defer this check until after all ray zero distance pairs have been found
	std::vector<int> uIDs;
	bool edgeFound = false;
	for (const auto& dist : distances) {
		if (home == dist.edge) {
			OP_ASSERT(dist.cept == homeCept);
			edgeFound = true;
			continue;
		}
		if (dist.cept == homeCept)
			return false;	// if edge and one other has zero distance, it is unsectable
		if (!dist.edge->unsectableID)
			continue;
		auto uIter = std::find(uIDs.begin(), uIDs.end(), dist.edge->unsectableID);
		if (uIDs.end() == uIter) {
			if (!edgeFound)
				uIDs.push_back(dist.edge->unsectableID);
		} else {
			if (edgeFound)
				return true;	// if one of pair of usect IDs is seen, when scanning for home 
			uIDs.erase(uIter);
		}
	}
	return false;
}

// at some point, do some math or rigorous testing to figure out how extreme this can be
// for now, keep making it smaller until it breaks
#define WINDING_NORMAL_LIMIT  0.001 // !!! no idea what this should be

bool SectRay::findIntercept(OpEdge* test) {
	if (test->ptBounds.ltChoice(axis) > normal)
		return true;
	if (test->ptBounds.rbChoice(axis) < normal)
		return true;
	if (test->unsortable)
		return false;
	if (test->disabled)
		return true;
	// start here;
	// failed to switch over to segment everywhere, may explain why experiment failed
	// !!! EXPERIMENT
	// try using segment's curve instead of edge curve
	// edge curve's control points, especially small ones, may magnify error
#define RAY_USE_SEGMENT 0
#if RAY_USE_SEGMENT
	const OpCurve& testCurve = test->segment->c;
#else
	const OpCurve& testCurve = test->setCurve();
#endif
	OpRoots roots = testCurve.axisRayHit(axis, normal);
	// get the normal at the intersect point and see if it is usable
	if (1 != roots.count) {
		// !!! if intercepts is 2 or 3, figure out why (and what to do)
		// !!! likely need to try a different ray
		OP_ASSERT(0 == roots.count);
		return true;
	}
	float root = roots.get(0);
#if RAY_USE_SEGMENT
	if (OpMath::IsNaN(root) || test->start.t >= root || root >= test->end.t)
		return false;
#else
	if (OpMath::IsNaN(root) || 0 == root || root == 1)
		return false;
#endif
	bool overflow;
	OpVector tangent = testCurve.tangent(root).normalize(&overflow);
	if (overflow)
		return false;
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	OpVector backRay = -ray;
	float tNxR = tangent.cross(backRay);
	if (fabs(tNxR) < WINDING_NORMAL_LIMIT)
		return false;
	OpPoint pt = testCurve.ptAtT(root);
	Axis perpendicular = !axis;
	float testXY = pt.choice(perpendicular);
	distances.emplace_back(test, testXY, root);
	return true;
}

FoundIntercept OpWinder::findRayIntercept(size_t inIndex, float normal, float homeCept) {
#if RAY_POINTER
	if (!home->ray)
		home->ray = new SectRay(normal, homeCept, axis);
	SectRay& ray = *home->ray;
#else
	SectRay& ray =  home->ray;
	ray.normal = normal;
	ray.homeCept = homeCept;
	ray.axis = workingAxis;
#endif
	Axis perpendicular = !workingAxis;
	float mid = .5;
	float midEnd = .5;
	std::vector<OpEdge*>& inArray = Axis::horizontal == workingAxis ? inX : inY;
	float homeMidT = home->center.t;
	do {
		ray.distances.clear();
		ray.distances.emplace_back(home, homeCept, homeMidT);
		int index = inIndex;
		// start at edge with left equal to or left of center
		while (index != 0) {
			OpEdge* test = inArray[--index];
			if (test == home)
				continue;
			if (!ray.findIntercept(test))
				goto tryADifferentCenter;
		}
		return FoundIntercept::yes;
	tryADifferentCenter:
		mid /= 2;
		midEnd = midEnd < .5 ? 1 - mid : mid;
		float middle = OpMath::Interp(home->ptBounds.ltChoice(workingAxis), 
				home->ptBounds.rbChoice(workingAxis), midEnd);
#if RAY_USE_SEGMENT
		const OpCurve& homeCurve = home->segment->c;
#else
		const OpCurve& homeCurve = home->setCurve();  // ok to be in loop (lazy)
#endif
		homeMidT = homeCurve.center(workingAxis, middle);
		if (OpMath::IsNaN(homeMidT) || mid <= 1.f / 256.f) {	// give it at most eight tries
			markUnsortable();
			return FoundIntercept::fail;	// nonfatal error (!!! give it a different name!)
		}
		// if find ray intercept can't find, restart with new center, normal, distance, etc.
		ray.homeCept = homeCept = homeCurve.ptAtT(homeMidT).choice(perpendicular);
		OP_ASSERT(!OpMath::IsNaN(homeCept));
		ray.normal = normal = homeCurve.ptAtT(homeMidT).choice(workingAxis);
		OP_ASSERT(!OpMath::IsNaN(normal));
	} while (true);
}

void OpWinder::markUnsortable() {
	if (Axis::vertical == workingAxis || inY.end() == std::find(inY.begin(), inY.end(), home)) 
		home->setUnsortable();
	home->fail = Axis::vertical == workingAxis ? EdgeFail::vertical : EdgeFail::horizontal;
}

// if horizontal axis, look at rect top/bottom
ChainFail OpWinder::setSumChain(size_t inIndex) {
	// see if normal at center point is in direction of ray
	std::vector<OpEdge*>& inArray = Axis::horizontal == workingAxis ? inX : inY;
	home = inArray[inIndex];
	OP_ASSERT(!home->disabled);
	const OpSegment* edgeSeg = home->segment;
	OpVector rayLine = Axis::horizontal == workingAxis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	bool overflow;
	float NxR = edgeSeg->c.tangent(home->center.t).normalize(&overflow).cross(rayLine);
	if (overflow)
		OP_DEBUG_FAIL(*home, ChainFail::normalizeOverflow);
	if (fabs(NxR) < WINDING_NORMAL_LIMIT) {
		markUnsortable();
		return ChainFail::normalizeUnderflow;  // nonfatal error
	}
	// intersect normal with every edge in the direction of ray until we run out 
	Axis perpendicular = !workingAxis;
	float normal = home->center.pt.choice(workingAxis);
	if (normal == home->start.pt.choice(workingAxis)
			|| normal == home->end.pt.choice(workingAxis)) {
		markUnsortable();
		return ChainFail::noNormal;  // nonfatal error
	}
	float homeCept = home->center.pt.choice(perpendicular);
	// advance to furthest that could influence the sum winding of this edge
	inIndex += 1;
	for (; inIndex < inArray.size(); ++inIndex) {
		OpEdge* advance = inArray[inIndex];
		if (advance->ptBounds.ltChoice(perpendicular) > homeCept)
			break;
	}
	FoundIntercept foundIntercept = findRayIntercept(inIndex, normal, homeCept);
#if RAY_POINTER
	OP_ASSERT(home->ray && home->ray->distances.size());
	SectRay& ray = *(home->ray);
#else
	OP_ASSERT(home->ray.distances.size());
	SectRay& ray = home->ray;
#endif
	if (FoundIntercept::fail == foundIntercept)
		return ChainFail::failIntercept;
	if (FoundIntercept::overflow == foundIntercept)
		return ChainFail::normalizeOverflow;
	std::sort(ray.distances.begin(), ray.distances.end(), 
			[](const EdgeDistance& s1, const EdgeDistance& s2) {
		return s1.cept < s2.cept;
	});
	return ChainFail::none;
}

ResolveWinding OpWinder::setWindingByDistance() {
	// find edge; then walk backwards to first known sum 
#if RAY_POINTER
	SectRay& ray = *(home->ray);
#else
	SectRay& ray = home->ray;
#endif
	OP_ASSERT(ray.distances.size());
	if (1 == ray.distances.size()) {
		OP_ASSERT(home == ray.distances[0].edge);
		OP_ASSERT(!home->pals.size());
		OP_ASSERT(!home->unsectableID);
		OpWinding prev(WindingTemp::dummy);
		// look at direction of edge relative to ray and figure winding/oppWinding contribution
		if (CalcFail::fail == home->addIfUR(ray.axis, ray.distances[0].t, &prev))
			OP_DEBUG_FAIL(*home, ResolveWinding::fail);
		OP_EDGE_SET_SUM(home, prev);
		return ResolveWinding::resolved;
	}
	if (!home->unsectableID && ray.betweenUnsectables(home)) {
		home->setBetween();
		return ResolveWinding::resolved;	// !!! nonfatal error (but, do not retry vertically)
	}
	// starting with found or zero if none, accumulate sum up to winding
	OpWinding sumWinding(WindingTemp::dummy);
	int sumIndex = ray.distances.size();
	while (ray.distances[--sumIndex].edge != home) 
		OP_ASSERT(sumIndex > 0);
	float homeT = ray.distances[sumIndex].t;  // used by unsectable, later
	while (--sumIndex >= 0 && (ray.distances[sumIndex].edge->unsectableID 
			|| !ray.distances[sumIndex].edge->sum.isSet()))
		;
	if (sumIndex >= 0) {
		EdgeDistance& sumDistance = ray.distances[sumIndex];
		OpEdge* sumEdge = sumDistance.edge;
		OP_ASSERT(!sumEdge->pals.size());
		sumWinding = sumEdge->sum;
		OP_DEBUG_CODE(sumWinding.debugType = WindingType::temp);
		// if pointing down/left, subtract winding
		if (CalcFail::fail == sumEdge->subIfDL(ray.axis, sumDistance.t, &sumWinding))  
			OP_DEBUG_FAIL(*sumEdge, ResolveWinding::fail);
	}
	// walk from the known sum to (and including) the edge
	OpEdge* prior;
	do {
		OP_ASSERT(sumIndex + 1 < (int) ray.distances.size());
		EdgeDistance& dist = ray.distances[++sumIndex];
		prior = dist.edge;
		if (home->unsectableID && abs(prior->unsectableID) == abs(home->unsectableID))
			break;
		NormalDirection normDir = prior->normalDirection(ray.axis, dist.t);
		if (NormalDirection::underflow == normDir || NormalDirection::overflow == normDir) {
			prior->setUnsortable();
			continue;
		}
		if (NormalDirection::downLeft == normDir && !prior->pals.size())
			OP_EDGE_SET_SUM(prior, sumWinding);
		if (CalcFail::fail == prior->addSub(ray.axis, dist.t, &sumWinding)) // if d/l sub; if u/r add
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
		if (NormalDirection::upRight == normDir && !prior->pals.size())
			OP_EDGE_SET_SUM(prior, sumWinding);
	} while (home != prior);
	if (!home->unsectableID)
		return ResolveWinding::resolved;
	// if home is unsectable, set its sum winding as if all of its pals' windings were a single edge
	OP_ASSERT(!home->many.isSet());
	// winding must be replaced by all unsectable windings -- however, other unsectables will want 
	//   to see the original winding. This is why 'many' is used. After all sums are computed
	//   replace winding with many.
	home->many = home->winding;	// back up winding
	OpContours* contours = home->segment->contour->contours;
	NormalDirection normDir = home->normalDirection(ray.axis, homeT);
	OP_ASSERT(NormalDirection::underflow != normDir && NormalDirection::overflow != normDir);
	for (const auto& pal : home->pals) {
		NormalDirection palDir = pal.edge->normalDirection(ray.axis, homeT);
		OP_ASSERT(NormalDirection::underflow != palDir && NormalDirection::overflow != palDir);
		home->winding.move(pal.edge->winding, contours, normDir != palDir);
	}
	if (CalcFail::fail == home->addIfUR(ray.axis, homeT, &sumWinding))
		OP_DEBUG_FAIL(*home, ResolveWinding::fail);
	OP_EDGE_SET_SUM(home, sumWinding);
	std::swap(home->many, home->winding);  // restore winding, put total of pals in many
	return ResolveWinding::resolved;	   // (will copy many to winding after all many are found)
}

FoundWindings OpWinder::setWindings(OpContours* contours) {
	// test sum chain for correctness; recompute if prior or next are inconsistent
	for (Axis a : { Axis::horizontal, Axis::vertical }) {
		workingAxis = a;
		std::vector<OpEdge*>& edges = Axis::horizontal == workingAxis ? inX : inY;
		for (size_t index = 0; index < edges.size(); ++index) {
			home = edges[index];
			if (home->ray.distances.size())
				continue;
			if (home->disabled)	// may not be visible in vertical pass
				continue;
//			if (home->unsectableID)	// compute the ray intersections for unsectables to find pals
//				continue;			// but not to use the result in computing winding
			if (home->between)
				continue;
			if (EdgeFail::horizontal == home->fail && Axis::vertical == workingAxis)
				home->fail = EdgeFail::none;
			else if (home->unsortable)  // may be too small
				continue;
			ChainFail chainFail = setSumChain(index);
			if (ChainFail::normalizeOverflow == chainFail)
				OP_DEBUG_FAIL(*home, FoundWindings::fail);
		}
	}
	for (auto& contour : contours->contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
	#if RAY_POINTER
				SectRay& ray = *(edge->ray);
	#else
				SectRay& ray = edge.ray;
	#endif
				if (edge.disabled)
					continue;
				if (edge.unsortable)
					continue;
				if (!edge.unsectableID)
					ray.markUnsectables(&edge);
				// walk edge from low to high (backwards) until simple edge with sum winding is found
				// if edge is unsectable and next to another unsectable, don't set its sum winding, 
				// but accumulate its winding for other edges and store accumulated total in each unsectable
				// mark consecutive pairs or more of unsectable as multiples
				ray.markUnsectableGroups(&edge);
			}
		}
	}
	for (auto& contour : contours->contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
	#if RAY_POINTER
				SectRay& ray = *(edge->ray);
	#else
				SectRay& ray = edge.ray;
	#endif

				if (edge.unsectableID)
					ray.markPals(&edge);
			}
		}
	}
	for (auto& contour : contours->contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				if (edge.sum.isSet())
					continue;
				if (edge.disabled)
					continue;
				if (edge.unsortable)
					continue;
	#if RAY_POINTER
				SectRay& ray = *(edge->ray);
	#else
				SectRay& ray = edge.ray;
	#endif
				if (!ray.distances.size())
					continue;
				home = &edge;
				ResolveWinding resolveWinding = setWindingByDistance();
				if (ResolveWinding::fail == resolveWinding)
					OP_DEBUG_FAIL(edge, FoundWindings::fail);
			}
		}
	}
	for (auto& contour : contours->contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				if (edge.disabled)
					continue;
				if (edge.unsectableID) {
					OP_ASSERT(edge.pals.size());
					std::swap(edge.winding, edge.many);
				}
				if (edge.sum.isSet())
					continue;
				if (edge.unsortable)
					continue;
				if (edge.fail == EdgeFail::horizontal)
					continue;
				OP_DEBUG_FAIL(edge, FoundWindings::fail);
			}
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

void OpWinder::sort(EdgesToSort sortBy) {
	if (EdgesToSort::byBox == sortBy) {
		std::sort(inX.begin(), inX.end(), compareXBox);
		return;
	}
	OP_ASSERT(EdgesToSort::byCenter == sortBy);
	std::sort(inX.begin(), inX.end(), compareXCenter);
	std::sort(inY.begin(), inY.end(), compareYCenter);
}
