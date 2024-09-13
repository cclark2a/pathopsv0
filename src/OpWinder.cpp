// (c) 2023, Cary Clark cclark2@gmail.com
#include <cmath>
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include "PathOps.h"

// this catches unsectables by keeping track of edges that are found to be adjacent
// each time a ray is cast. If the edge is seen to the left during one ray cast, and to the right
// on another, it is marked as an unsectable pair.
// !!! There may be unsectable pairs with other edges between. Wait for that before coding.
// !!! This is now detected when rays are cast. Change this temporarily to assert that it is not
//     needed
void SectRay::addPals(OpEdge* home) {
	OP_ASSERT(this == &home->ray);
	if (!distances.size())
		return;
	auto matchCept = [home](EdgeDistance* test) {
//		OP_ASSERT(axis == test->edge->ray.axis);  // !!! I don't think this matters ?
		home->addPal(*test);
		if (EdgeDistance* homeDist = test->edge->ray.find(home)) {
			test->edge->addPal(*homeDist);
//			OP_DEBUG_CODE(EdgeDistance* testDist = test->edge->ray.find(test->edge));
// !!! this asserts if there are three or more pals
// consider writing more complex test to detect if edge between pals is not a pal
//			OP_ASSERT(abs(homeDist - testDist) == 1);
		}
	};
	EdgeDistance* homeDist = find(home);
	EdgeDistance* test = homeDist;
	float lowLimit = OpMath::NextSmaller(homeCept);
    bool priorIsPal = false;
	while (test > &distances.front() && (--test)->cept >= lowLimit) {
		OP_ASSERT((test + 1)->cept >= test->cept);
		matchCept(test);
        priorIsPal = true;
	}
	test = homeDist;
	float highLimit = OpMath::NextLarger(homeCept);
    bool nextIsPal = false;
	while (test < &distances.back() && (++test)->cept <= highLimit) {
		OP_ASSERT((test - 1)->cept <= test->cept);
		matchCept(test);
        nextIsPal = true;
	}
	// if axes are different, and if y-axis edge is oriented nw/se (not ne/sw), reverse
	auto axesReversed = [home](OpEdge* test) {
		if (test->ray.axis == home->ray.axis)
			return false;
		OpEdge* vertical = Axis::vertical == test->ray.axis ? test : home;
		OpVector dxy = vertical->endPt() - vertical->startPt();
		if (!dxy.dy)
			return false;
		return dxy.dy > 0 ? dxy.dx > 0 : dxy.dx < 0;
	};
    // check next ray intersected edge if it hasn't been checked already
    // !!! stops at 1; don't know if we may need more than one
    // !!! thread_circles54530 failed only on laptop 
	auto addIfFlipped = [axesReversed, homeDist, home, this](DistEnd offset) {
		OP_ASSERT(DistEnd::back == offset || DistEnd::front == offset);
		if (homeDist == end(offset))
			return;
		OpEdge* edge = next(homeDist, offset)->edge;
		SectRay& ray = edge->ray;
		EdgeDistance* dist = ray.find(edge);
		if (!dist)
			return;
		if (axesReversed(edge))
			offset = !offset;		
		if (dist == ray.end(offset))
			return;
		if (ray.next(dist, offset)->edge != home)
			return;
		home->addPal(*dist);
		edge->addPal(*homeDist);
		return;
	};
	if (!priorIsPal)
		addIfFlipped(DistEnd::front);
	if (!nextIsPal)
		addIfFlipped(DistEnd::back);
}

bool SectRay::checkOrder(const OpEdge* home) const {
	for (const EdgeDistance* dist = &distances.front(); (dist + 1)->edge != home; ++dist) {
		OpEdge* prior = dist->edge;
		OpEdge* last = (dist + 1)->edge;
		// pal should be set in time for this : testQuads26519435
		if (prior->isUnsectable() || last->isUnsectable() || last->isPal(prior))
			continue;
		if (last->ray.distances.size() > 1 && last->ray.axis == axis) {
			EdgeDistance* lastDist = last->ray.find(last);
			if (lastDist < &last->ray.distances.back() && (lastDist + 1)->edge == prior) {
				return false;
			}
		}
		if (prior->ray.distances.size() > 1 && prior->ray.axis == axis) {
			EdgeDistance* priorDist = prior->ray.find(prior);
			if (priorDist > &prior->ray.distances.front() && (priorDist - 1)->edge == last) {
				return false;
			}
		}
		if (dist->cept == (dist + 1)->cept) {
			OP_DEBUG_CODE(prior->contours()->debugFailOnEqualCepts = true);
			return false;
		}
	}
	return true;
}

EdgeDistance* SectRay::find(OpEdge* edge) {
    if (!distances.size())
		return nullptr;
	for (auto test = &distances.back(); test >= &distances.front(); --test) {
		if (test->edge == edge)
			return test;
	}
	return nullptr;
}

// at some point, do some math or rigorous testing to figure out how extreme this can be
// for now, keep making it smaller until it breaks
// !!! fails (I think) on pentrek13 edge 1045 NxR:00221
#define WINDING_NORMAL_LIMIT  0.008 // 0.004  fails on testQuads19022897 edge 151 NxR:-0.00746

FindCept SectRay::findIntercept(OpEdge* home, OpEdge* test) {
	if (test->ptBounds.ltChoice(axis) > normal)
		return FindCept::ok;
	if (test->ptBounds.rbChoice(axis) < normal)
		return FindCept::ok;
	if (test == home)
		return FindCept::ok;
	if (test->isUnsortable)
		return FindCept::unsortable;
	if (test->disabled)
		return FindCept::ok;
	bool uSectPair = test->isUnsectable() && home->isUnsectable() && test->isUnsectablePair(home);
	if (uSectPair && !firstTry)
		return FindCept::unsectable;
	// !!! EXPERIMENT  try using segment's curve instead of edge curve
	//                 edge curve's control points, especially small ones, may magnify error
	//                 failed to switch over to segment everywhere, may explain why experiment failed
	OpRoots roots = test->curve.axisRayHit(axis, normal);  // get the normal at the intersect point	
	float root = OpNaN;
	float testXY = OpNaN;
	auto pushUsectDist = [this, test, &testXY, &root, uSectPair]() {
		if (uSectPair) {
			distances.emplace_back(test, testXY, root, false);
			return FindCept::addPal;
		}
		return FindCept::retry;
	};
	if (1 != roots.count)
		return pushUsectDist();  // preferable for thread_cubics157381
	root = roots.get(0);
	if (OpMath::IsNaN(root) || 0 == root || root == 1)
		return pushUsectDist();
	OpVector tangent = test->curve.tangent(root).normalize();
	if (!tangent.isFinite() || tangent == OpVector{ 0, 0 } )
		return pushUsectDist();
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	OpVector backRay = -ray;
	float tNxR = tangent.cross(backRay);
	if (fabs(tNxR) < WINDING_NORMAL_LIMIT)
		return pushUsectDist();
	OpPoint pt = test->curve.ptAtT(root);
	Axis perpendicular = !axis;
	testXY = pt.choice(perpendicular);
	bool reversed = tangent.dot(homeTangent) < 0;
	distances.emplace_back(test, testXY, root, reversed);
	if (!uSectPair && OpMath::Equalish(testXY, homeCept))
		return FindCept::retry;  // e.g., testQuads1877923 has two small quads which just miss 
	return uSectPair ? FindCept::addPal : FindCept::ok;
}

void SectRay::sort() {
	std::sort(distances.begin(), distances.end(), 
			[](const EdgeDistance& s1, const EdgeDistance& s2) {
		return s1.cept < s2.cept || (s1.cept == s2.cept && s1.edge->id < s2.edge->id);
	});
}

OpWinder::OpWinder(OpContours& contours, EdgesToSort edgesToSort) {
	for (auto contour : contours.contours) {
		for (auto& segment : contour->segments) {
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
		OpIntersection* sect = segment->addSegSect(ptTAorB, oppSegment  OP_LINE_FILE_PARAMS());
		OpIntersection* oSect = oppSegment->addSegSect(ptTCorD, segment  OP_LINE_FILE_PARAMS());
		sect->pair(oSect);
		return IntersectResult::yes;
	}
	// check if an existing coincidence range is extended
	int coinID;
	if (OpIntersection* segSect = segment->sects.contains(ptTAorB, oppSegment);
			segSect && segSect->coincidenceID)
		coinID = segSect->coincidenceID;
	else if (OpIntersection* oppSect = oppSegment->sects.contains(ptTCorD, segment); 
			oppSect && oppSect->coincidenceID)
		coinID = oppSect->coincidenceID;
	// pass a mix of seg and opp; construct one t for each
	else
		coinID = segment->coinID(flipped);
	AddMix(xyChoice, ptTAorB, flipped, cPtT, dPtT, segment, oppSegment, coinID, 
			ptTAorB == aPtT ? MatchEnds::start : MatchEnds::end);
	AddMix(xyChoice, ptTCorD, flipped, aPtT, bPtT, oppSegment, segment, coinID, 
			(ptTAorB == aPtT) == flipped ? MatchEnds::start : MatchEnds::end);
	return IntersectResult::yes;
}

// this edge has points A, B; opp edge has points C, D
// adds 0: no intersection; 1: end point only; 2: partial or full coincidence
IntersectResult OpWinder::CoincidentCheck(const OpEdge& edge, const OpEdge& opp) {
	return OpWinder::CoincidentCheck(edge.start(), edge.end(), opp.start(), opp.end(),
			const_cast<OpSegment*>(edge.segment), const_cast<OpSegment*>(opp.segment));
}

void OpWinder::AddMix(XyChoice xyChoice, OpPtT ptTAorB, bool flipped, OpPtT cPtT, OpPtT dPtT,
		OpSegment* segment, OpSegment* oppSegment, int coinID, MatchEnds match) {
	float eStart = ptTAorB.pt.choice(xyChoice);
	if (flipped)
		std::swap(cPtT, dPtT);
	float oStart = cPtT.pt.choice(xyChoice);
	float oEnd = dPtT.pt.choice(xyChoice);
	float oTRange = dPtT.t - cPtT.t;
	OpPtT oCoinStart { ptTAorB.pt, cPtT.t + (eStart - oStart) / (oEnd - oStart) * oTRange };
	OP_ASSERT(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
	// !!! if sects already exist and are not marked coincident, just mark them?
	OpIntersection* segSect = segment->sects.contains(ptTAorB, oppSegment);
	OpIntersection* oppSect = oppSegment->sects.contains(oCoinStart, segment);
	if (segSect && oppSect && !segSect->coincidenceID && !oppSect->coincidenceID) {
		OP_ASSERT(MatchEnds::both != match);
		segSect->setCoin(coinID, match);
		oppSect->setCoin(coinID, flipped ? !match : match);
	}
	if (segSect || oppSect)  // required by fuzz763_3, fuzz763_5
        return;
    OpIntersection* sect = segment->contour->addCoinSect(ptTAorB, segment, coinID, match  
			OP_LINE_FILE_PARAMS(oppSegment));
	segment->sects.add(sect);
	OpIntersection* oSect = oppSegment->contour->addCoinSect(oCoinStart, oppSegment, coinID, 
			flipped ? MatchEnds::start : MatchEnds::end 
			OP_LINE_FILE_PARAMS(segment));
	oppSegment->sects.add(oSect);
	OP_ASSERT(sect && oSect);
	sect->pair(oSect);
}

// If we got here because a pair of edges are coincident, that coincidence may have already been
// recorded when the pair of segments were checked, or the intersections may have been computed.
// !!! if this code is attempting to grow existing coin runs (maybe it is?) then it needs to 
//     remove the former ends so the result has a single start and end for each sect list run
IntersectResult OpWinder::AddPair(XyChoice xyChoice, OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
	bool flipped, OpSegment* segment, OpSegment* oppSegment) {
	// set range to contain intersections that match this segment and opposite segment
	std::vector<OpIntersection*> range;
	segment->sects.range(oppSegment, range);
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
	bool addToExistingRange = false;
	bool addedSect1 = false;
	bool addedSect2 = false;
	bool addedOSect1 = false;
	bool addedOSect2 = false;
	if (!coinID)
		coinID = segment->coinID(flipped);
	else {
		OP_ASSERT(flipped ? coinID < 0 : coinID > 0);	// should never assert
		addToExistingRange = true;
	}
	// assign a new or existing coin id if sect doesn't already have one
	// this is called in cases as simple as two coincident line segments
	// preflight both calls to addCoin; if either will fail, give up early
	// (failure means that points are too close together to form a meaningful coincidence)
	bool iStartContains = false;
	bool iEndContains = false;
	if (!aInCoincidence && !sect1)  // segment already has intersection (segment start); e.g., line doubles back
		// and if it doesn't exist and isn't in a coin range, make one
		iStartContains = segment->sects.contains(aPtT, oppSegment);
	if (!bInCoincidence && !sect2)  // segment already has intersection (segment end); e.g., line doubles back
		iEndContains = segment->sects.contains(bPtT, oppSegment);
	if (iStartContains || iEndContains)
		return IntersectResult::no;
	std::vector<OpIntersection*> oRange;
	oppSegment->sects.range(segment, oRange);
	OpIntersection* oSect1 = findSect(oRange, { aPtT.pt, -1 });
	OpIntersection* oSect2 = findSect(oRange, { bPtT.pt, -1 });
	// add the opposite that goes with the created segment sect
	float oStart = 0;
	float oXYRange = 0;
	if (!oSect1 || !oSect2) {
		if (flipped)
			std::swap(cPtT, dPtT);
		oStart = cPtT.pt.choice(xyChoice);
		float oEnd = dPtT.pt.choice(xyChoice);
		oXYRange = oEnd - oStart;
	}
	bool setOSect1CoinID = false;
	bool setOSect2CoinID = false;
	OpPtT oCoinStart;
	OpPtT oCoinEnd;
	bool oStartContains = false;
	bool oEndContains = false;
	if (!oSect1) {
		float eStart = aPtT.pt.choice(xyChoice);
		oCoinStart = { aPtT.pt, OpMath::Interp(cPtT.t, dPtT.t, (eStart - oStart) / oXYRange) };
		OP_ASSERT(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
		oStartContains = oppSegment->sects.contains(oCoinStart, segment);
	} else   // segment already has intersection (start or end); e.g., line doubles back
			if (!(inCoinRange(oRange, oSect1->ptT.t, nullptr) & 1))
		setOSect1CoinID = true;  // defer so that check of o sect 2 isn't affected
	if (!oSect2) {
		float eEnd = bPtT.pt.choice(xyChoice);
		oCoinEnd = { bPtT.pt, OpMath::Interp(cPtT.t, dPtT.t, (eEnd - oStart) / oXYRange ) };
		OP_ASSERT(OpMath::Between(cPtT.t, oCoinEnd.t, dPtT.t));
		oEndContains = oppSegment->sects.contains(oCoinEnd, segment);
	} else   // segment already has intersection (start or end); e.g., line doubles back
			if (!(inCoinRange(oRange, oSect2->ptT.t, nullptr) & 1))
		setOSect2CoinID = true;
	// call addCoin only if both sides succeeded
	if (oStartContains || oEndContains)
		return IntersectResult::no;
	if (!aInCoincidence) {
		if (sect1) {
			OP_ASSERT(!sect1->coincidenceID);
			OP_ASSERT(MatchEnds::none == sect1->coinEnd);
			sect1->setCoin(coinID, MatchEnds::start);
			segment->sects.resort = true;
		} else {
			sect1 = segment->addCoin(aPtT, coinID, MatchEnds::start, oppSegment
						OP_LINE_FILE_PARAMS());
			OP_ASSERT(sect1);
			addedSect1 = true;
		}
	}
	if (!bInCoincidence) {
		if (sect2) {
			OP_ASSERT(!sect2->coincidenceID);
			OP_ASSERT(MatchEnds::none == sect2->coinEnd);
			sect2->setCoin(coinID, MatchEnds::end);
			segment->sects.resort = true;
		} else {
			sect2 = segment->addCoin(bPtT, coinID, MatchEnds::end, oppSegment
						OP_LINE_FILE_PARAMS());
			OP_ASSERT(sect2);
			addedSect2 = true;
		}
	}
	if (!oSect1 && sect1) {
		oSect1 = oppSegment->addCoin(oCoinStart, coinID, flipped ? MatchEnds::end 
				: MatchEnds::start, segment  OP_LINE_FILE_PARAMS());
		OP_ASSERT(oSect1);
		sect1->pair(oSect1);
		addedOSect1 = true;
	}
	if (!oSect2 && sect2) {
		oSect2 = oppSegment->addCoin(oCoinEnd, coinID, flipped ? MatchEnds::start 
				: MatchEnds::end, segment  OP_LINE_FILE_PARAMS());
		OP_ASSERT(oSect2);
		sect2->pair(oSect2);
		addedOSect2 = true;
	}
	if (setOSect1CoinID) {
		OP_ASSERT(!oSect1->coincidenceID);
		OP_ASSERT(MatchEnds::none == oSect1->coinEnd);
		oSect1->setCoin(coinID, flipped ? MatchEnds::end : MatchEnds::start); // !!! untested
		oppSegment->sects.resort = true;
	}
	if (setOSect2CoinID) {
		OP_ASSERT(!oSect2->coincidenceID);
		OP_ASSERT(MatchEnds::none == oSect2->coinEnd);
		oSect2->setCoin(coinID, flipped ? MatchEnds::start : MatchEnds::end);
		oppSegment->sects.resort = true;
	}
	if (addToExistingRange) {
		auto coinOutside = [](std::vector<OpIntersection*>& range, 
				bool addedSect1, OpIntersection* sect1, bool addedSect2, OpIntersection* sect2) {
			if (addedSect1 || addedSect2) {
				if (addedSect1)
					range.push_back(sect1);
				if (addedSect2)
					range.push_back(sect2);
				std::sort(range.begin(), range.end(), [](
						const OpIntersection* s1, const OpIntersection* s2) {
						return s1->ptT.t < s2->ptT.t; });
				for (auto sectIter = range.begin() + 1; sectIter < range.end() - 1; ++sectIter)
					if ((*sectIter)->coincidenceID)
						(*sectIter)->zeroCoincidenceID();
			}
		};
		coinOutside(range, addedSect1, sect1, addedSect2, sect2);
		segment->sects.resort |= addedSect1 | addedSect2;
		coinOutside(oRange, addedOSect1, oSect1, addedOSect2, oSect2);
		oppSegment->sects.resort |= addedOSect1 | addedOSect2;
	}
	if ((setOSect1CoinID || (sect1 && oSect1)) && (setOSect2CoinID || (sect2 && oSect2)))
		return IntersectResult::yes;
	else
		return IntersectResult::fail;
}


// upscale t to call segment line curve intersection
// !!! I'm bothered that segment / segment calls a different form of this
// Return if an intersection was added so that op curve curve can record this
// then, change op curve curve checks for undetected coincidence between pair of curves if this
// intersection pair forms such (issue3517)
IntersectResult OpWinder::AddLineCurveIntersection(OpEdge& opp, OpEdge& edge, bool secondAttempt) {
	OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
	OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
	OP_ASSERT(oSegment != eSegment);
	OP_ASSERT(edge.curve.debugIsLine());
	LinePts edgePts { edge.startPt(), edge.endPt() };
    OpRoots septs = oSegment->c.rayIntersect(edgePts, MatchEnds::none); 
	IntersectResult sectAdded = IntersectResult::no;
	// check the ends of each edge to see if they intersect the opposite edge (if missed earlier)
	auto addPair = [eSegment, oSegment  OP_DEBUG_PARAMS(opp, edge)](OpPtT oppPtT, OpPtT edgePtT,
			IntersectResult& added  OP_LINE_FILE_DEF()) {
		if (!eSegment->sects.contains(edgePtT, oSegment)
				&& !oSegment->sects.contains(oppPtT, eSegment)) {
			OP_ASSERT(!OpMath::IsNaN(edgePtT.t));
			OpIntersection* sect = eSegment->addEdgeSect(edgePtT  
					OP_LINE_FILE_CALLER(&edge, &opp));
			OP_ASSERT(!OpMath::IsNaN(oppPtT.t));
			OpIntersection* oSect = oSegment->addEdgeSect(oppPtT  
					OP_LINE_FILE_CALLER(&edge, &opp));
			sect->pair(oSect);
			added = IntersectResult::yes;
		}
	};
	if (!septs.count) {
		auto checkEnd = [opp](const OpPtT& start) {
			float t = opp.segment->c.match(opp.startT, opp.endT, start.pt);
			if (OpMath::IsNaN(t))
				return OpPtT();
			return OpPtT { start.pt, t };
		};
		OpPtT oppStart = checkEnd(edge.start());
		if (!OpMath::IsNaN(oppStart.t))
			addPair(oppStart, edge.start(), sectAdded  OP_LINE_FILE_PARAMS());
		OpPtT oppEnd = checkEnd(edge.end());
		if (!OpMath::IsNaN(oppEnd.t))
			addPair(oppEnd, edge.end(), sectAdded  OP_LINE_FILE_PARAMS());
		if (IntersectResult::yes == sectAdded)
			return sectAdded;
	}
	MatchReverse match = opp.segment->matchEnds(edgePts);
	if (septs.fail == RootFail::rawIntersectFailed) {
		// binary search on opp t-range to find where vert crosses zero
		OpCurve rotated = opp.segment->c.toVertical(edgePts, match.match);
		septs.roots[0] = rotated.tZeroX(opp.startT, opp.endT);
		septs.count = 1;
	}
	// Note that coincident check does not receive intercepts as a parameter; in fact, the intercepts
	// were not calculated (the roots are uninitialized). This is because coincident check will 
	// compute the actual coincident start and end without the roots introducing error.
	if (2 == septs.count && opp.isLine())
		return CoincidentCheck(edge, opp);
	bool tInRange = false;
	for (unsigned index = 0; index < septs.count; ++index) {
		if (opp.startT > septs.get(index)) {
			if (opp.startPt().isNearly(edge.startPt()))
				addPair(opp.start(), OpPtT(opp.startPt(), edge.startT), sectAdded  OP_LINE_FILE_PARAMS());
			else if (opp.startPt().isNearly(edge.endPt()))
				addPair(opp.start(), OpPtT(opp.startPt(), edge.endT), sectAdded  OP_LINE_FILE_PARAMS());
			else {
		        OpCurve rotated = opp.segment->c.toVertical(edgePts, match.match);
		        septs.roots[0] = rotated.tZeroX(opp.startT, opp.endT);
				if (opp.startT > septs.get(index))
                    continue;
            }
		}
		if (septs.get(index) > opp.endT) {
			if (opp.endPt().isNearly(edge.startPt()))
				addPair(opp.end(), OpPtT(opp.endPt(), edge.startT), sectAdded  OP_LINE_FILE_PARAMS());
			else if (opp.endPt().isNearly(edge.endPt()))
				addPair(opp.end(), OpPtT(opp.endPt(), edge.endT), sectAdded  OP_LINE_FILE_PARAMS());
			else {
		        OpCurve rotated = opp.segment->c.toVertical(edgePts, match.match);
		        septs.roots[0] = rotated.tZeroX(opp.startT, opp.endT);
                if (septs.get(index) > opp.endT)
				    continue;
            }
		}
		tInRange = true;
		OpPtT oppPtT { opp.segment->c.ptAtT(septs.get(index)), septs.get(index) };
		if (OpMath::IsNaN(oppPtT.t))
			continue;
		float edgeT = edge.segment->findValidT(0, 1, oppPtT.pt);
		if (!OpMath::Between(0, edgeT, 1))  // !!! shouldn't this just be a nan test?
			continue;
#if OP_DEBUG_RECORD
		OpDebugRecordSuccess(index);
#endif
		if (0 == edgeT)
			oppPtT.pt = eSegment->c.firstPt();
		else if (1 == edgeT)
			oppPtT.pt = eSegment->c.lastPt();
        // pin point to both bounds, but only if it is on edge
//		OP_DEBUG_CODE(OpPoint debugPt = oppPtT.pt);
        oSegment->ptBounds.pin(&oppPtT.pt);
//      eSegment->ptBounds.pin(&oppPtT.pt);	// !!! doubtful this is needed with contains test above
//		OP_ASSERT(debugPt == oppPtT.pt);	// rarely needed, but still triggered (e.g., joel_15x)
		OpPtT edgePtT { oppPtT.pt, edgeT };
		addPair(oppPtT, edgePtT, sectAdded  OP_LINE_FILE_PARAMS());
	}
	if (!tInRange && opp.isLine() && !secondAttempt) {
		OpDebugRecordStart(edge, opp);
		return AddLineCurveIntersection(edge, opp, true);
	}
	return sectAdded;
}

FoundIntercept OpWinder::findRayIntercept(size_t homeIndex, OpVector homeTan, float normal, 
		float homeCept) {
	SectRay& ray = home->ray;
	ray.homeTangent = homeTan;
	ray.normal = normal;
	ray.homeCept = homeCept;
	ray.axis = workingAxis;
	Axis perpendicular = !workingAxis;
	float mid = .5;
	float midEnd = .5;
	std::vector<OpEdge*>& inArray = Axis::horizontal == workingAxis ? inX : inY;
	ray.homeT = OpMath::Ratio(home->startT, home->endT, home->center.t);
	// if find intercept fails, retry some number of times
	// if all retries fail, distinguish between failure cases
	//   if it failed because closest edge was too close, mark pair as unsectable
	ray.firstTry = true;
	do {
		ray.distances.clear();
		ray.distances.emplace_back(home, homeCept, ray.homeT, false);
		size_t inIndex = setInIndex(homeIndex, homeCept, inArray);
		// start at edge with left equal to or left of center
		while (inIndex != 0) {
			OpEdge* test = inArray[--inIndex];
			FindCept findCept = ray.findIntercept(home, test);
			if (FindCept::ok == findCept)
				continue;
			if (FindCept::retry == findCept)
				goto tryADifferentCenter;
			if (FindCept::addPal == findCept) {
				EdgeDistance& tDist = ray.distances.back();
				home->addPal(tDist);
				continue;
			}
			if (FindCept::unsortable == findCept)
				goto giveUp;
		}
		if (ray.distances.size() <= 1) 
			return FoundIntercept::yes;
		ray.sort();
		if (ray.distances.front().edge == home)
			return FoundIntercept::yes;
		if (ray.checkOrder(home))
			return FoundIntercept::yes;
	tryADifferentCenter:
		mid /= 2;
		midEnd = midEnd < .5 ? 1 - mid : mid;
		float middle = OpMath::Interp(home->ptBounds.ltChoice(workingAxis), 
				home->ptBounds.rbChoice(workingAxis), midEnd);
		float homeMidT = home->curve.center(workingAxis, middle);  // note: 0 to 1 on edge curve
		if (OpMath::IsNaN(homeMidT) || mid <= 1.f / 256.f) {  // give it at most eight tries
			if (!home->isUnsectable())
				markUnsortable();
			break;	// give up
		}
		// if find ray intercept can't find, restart with new center, normal, distance, etc.
		ray.homeCept = homeCept = home->curve.ptAtT(homeMidT).choice(perpendicular);
		OP_ASSERT(!OpMath::IsNaN(homeCept));
		ray.normal = normal = home->curve.ptAtT(homeMidT).choice(workingAxis);
		ray.homeT = homeMidT;
		OP_ASSERT(!OpMath::IsNaN(normal));
	} while (true);
giveUp:
	// give up case: sort and go home
	ray.sort();
	return FoundIntercept::fail;	// nonfatal error (!!! give it a different name!)
}

void OpWinder::markUnsortable() {
	if (Axis::vertical == workingAxis || inY.end() == std::find(inY.begin(), inY.end(), home)) 
		home->setUnsortable();
	home->rayFail = Axis::vertical == workingAxis ? EdgeFail::vertical : EdgeFail::horizontal;
}

size_t OpWinder::setInIndex(size_t homeIndex, float homeCept, std::vector<OpEdge*>& inArray) {
	Axis perpendicular = !workingAxis;
	// advance to furthest that could influence the sum winding of this edge
	size_t inIndex = homeIndex + 1;
	for (; inIndex < inArray.size(); ++inIndex) {
		OpEdge* advance = inArray[inIndex];
		if (advance->ptBounds.ltChoice(perpendicular) > homeCept)
			break;
	}
	return inIndex;
}

// if horizontal axis, look at rect top/bottom
ChainFail OpWinder::setSumChain(size_t homeIndex) {
	// see if normal at center point is in direction of ray
	std::vector<OpEdge*>& inArray = Axis::horizontal == workingAxis ? inX : inY;
	home = inArray[homeIndex];
	OP_ASSERT(!home->disabled);
	const OpSegment* edgeSeg = home->segment;
	OpVector rayLine = Axis::horizontal == workingAxis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	OpVector homeTangent = edgeSeg->c.tangent(home->center.t);
	float NxR = homeTangent.normalize().cross(rayLine);
	if (!OpMath::IsFinite(NxR))
		OP_DEBUG_FAIL(*home, ChainFail::normalizeOverflow);
	if (fabs(NxR) < WINDING_NORMAL_LIMIT) {
		markUnsortable();
		return ChainFail::normalizeUnderflow;  // nonfatal error
	}
	// intersect normal with every edge in the direction of ray until we run out 
	float normal = home->center.pt.choice(workingAxis);
	if (normal == home->startPt().choice(workingAxis)
			|| normal == home->endPt().choice(workingAxis)) {
		markUnsortable();
		return ChainFail::noNormal;  // nonfatal error
	}
	Axis perpendicular = !workingAxis;
	float homeCept = home->center.pt.choice(perpendicular);
	FoundIntercept foundIntercept = findRayIntercept(homeIndex, homeTangent, normal, homeCept);
	if (FoundIntercept::fail == foundIntercept)
		return ChainFail::failIntercept;
	if (FoundIntercept::overflow == foundIntercept)
		return ChainFail::normalizeOverflow;
	return ChainFail::none;
}

ResolveWinding OpWinder::setWindingByDistance(OpContours* contours) {
	// find edge; then walk backwards to first known sum 
	SectRay& ray = home->ray;
	OP_ASSERT(ray.distances.size());
	if (1 == ray.distances.size()) {
		OP_ASSERT(home == ray.distances[0].edge);
		if (home->pals.size() || home->isUnsectable())  // !!! move this to where unsectable is set?
			home->setUnsortable();
		else {
			OpWinding prev(home, WindingSum::dummy);
			// look at direction of edge relative to ray and figure winding/oppWinding contribution
			if (CalcFail::fail == home->addIfUR(ray.axis, ray.distances[0].edgeInsideT, &prev))
				home->setUnsortable();
			else
				OP_EDGE_SET_SUM(home, prev.w);
		}
		return ResolveWinding::resolved;
	}
	// don't set the sum winding if this has pals or if any prior edge has this as a pal
	// back up only while the previous has pals
	// and: don't set the sum winding if the prior or next edge pt is very close to this pt
	// !!! any prior pal is called several times with the same edge, below. Optimization:
	// !!!    cache the answer in distance edge ?
	auto anyPriorPal = [ray](OpEdge* edge, int sumIndex) {
		if (edge->pals.size())
			return true;
		for (;;) {
			int next = sumIndex + 1;
			if (next >= (int) ray.distances.size())
				break;
			if (!ray.distances[next].edge->pals.size())
				break;
			sumIndex = next;
		}
		size_t last = (size_t) (sumIndex + 1);
		float lastCept = last < ray.distances.size() ? ray.distances[last].cept : OpNaN;
		bool lastIsEdge = false;
		do {
			const EdgeDistance& dist = ray.distances[sumIndex];
			OpEdge* previous = dist.edge;
			if (previous->isPal(edge))
				return true;
			if (lastIsEdge && OpMath::Equalish(lastCept, dist.cept))
				return true;
			lastIsEdge = previous == edge;
			if (lastIsEdge) {
				if (OpMath::Equalish(lastCept, dist.cept))
					return true;
			} else if (!previous->pals.size())
				break;
			lastCept = dist.cept;
		} while (--sumIndex >= 0);
		return false;
	};
	// starting with found or zero if none, accumulate sum up to winding
	OpWinding sumWinding(home, WindingSum::dummy);
	int sumIndex = ray.distances.size();
	while (ray.distances[--sumIndex].edge != home) 
		OP_ASSERT(sumIndex > 0);
	float homeT = ray.distances[sumIndex].edgeInsideT;  // used by unsectable, later
	while (--sumIndex >= 0 && (anyPriorPal(ray.distances[sumIndex].edge, sumIndex) 
			|| !ray.distances[sumIndex].edge->sum.isSet()))
		;
	if (sumIndex > 0 && !home->pals.size() && EdgeFail::none == home->rayFail && !ray.checkOrder(home))
		return ResolveWinding::retry;
	if (sumIndex >= 0) {
		EdgeDistance& sumDistance = ray.distances[sumIndex];
		OpEdge* sumEdge = sumDistance.edge;
		OP_ASSERT(!sumEdge->pals.size());
		sumWinding.w = sumEdge->sum.copyData();
		OP_DEBUG_CODE(sumWinding.debugType = WindingType::temp);
		// if pointing down/left, subtract winding
		if (CalcFail::fail == sumEdge->subIfDL(ray.axis, sumDistance.edgeInsideT, &sumWinding))  
			OP_DEBUG_FAIL(*sumEdge, ResolveWinding::fail);
	}
	OpEdge* prior;
	do {
		OP_ASSERT(sumIndex + 1 < (int) ray.distances.size());
		EdgeDistance& dist = ray.distances[++sumIndex];
		prior = dist.edge;
		if (home->pals.size() && (home == prior || home->isPal(prior)))
			break;
		NormalDirection normDir = prior->normalDirection(ray.axis, dist.edgeInsideT);
		if (NormalDirection::underflow == normDir) {
			prior->setUnsortable();
			continue;
		}
		if (NormalDirection::downLeft == normDir && !anyPriorPal(prior, sumIndex))
			OP_EDGE_SET_SUM(prior, sumWinding.w);
		if (CalcFail::fail == prior->addSub(ray.axis, dist.edgeInsideT, &sumWinding)) // if d/l sub; if u/r add
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
		if (NormalDirection::upRight == normDir && !anyPriorPal(prior, sumIndex))
			OP_EDGE_SET_SUM(prior, sumWinding.w);
	} while (home != prior);
	if (!home->pals.size()) {
		if (!home->sum.isSet())
			OP_EDGE_SET_SUM(home, sumWinding.w);
		return ResolveWinding::resolved;
	}
	// if home is unsectable, set its sum winding as if all of its pals' windings were a single edge
	OP_ASSERT(!home->many.isSet());
	// winding must be replaced by all unsectable windings -- however, other unsectables will want 
	//   to see the original winding. This is why 'many' is used. After all sums are computed
	//   replace winding with many.
	home->many = home->winding;	// back up winding
	for (const auto& pal : home->pals) {
		home->winding.move(pal.edge->winding, pal.reversed);
	}
	if (!home->winding.visible()) {
		home->setDisabled(OP_LINE_FILE_NPARAMS());
		home->windPal = true;
	}
	if (CalcFail::fail == home->addIfUR(ray.axis, homeT, &sumWinding))
		home->setUnsortable();
	else
		OP_EDGE_SET_SUM(home, sumWinding.w);
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
			if (home->ray.distances.size() && EdgeFail::none == home->rayFail)
				continue;
			if (home->disabled)	// may not be visible in vertical pass
				continue;
			if (EdgeFail::center == home->rayFail)
				continue;
	//		if (home->between)	// !!! set sum chain even if ray cannot be used for this edge...?
	//			continue;
			if (EdgeFail::horizontal == home->rayFail && Axis::vertical == workingAxis)
				home->rayFail = EdgeFail::none;
			else if (home->isUnsortable)  // may be too small
				continue;
			ChainFail chainFail = setSumChain(index);
			if (ChainFail::normalizeOverflow == chainFail)
				OP_DEBUG_FAIL(*home, FoundWindings::fail);
		}
	}
	for (auto contour : contours->contours) {
		for (auto& segment : contour->segments) {
			for (auto& edge : segment.edges) {
				SectRay& ray = edge.ray;
				if (edge.disabled)
					continue;
	//			if (edge.isUnsortable())	// !!! add pals even if ray cannot be used for this edge...?
	//				continue;
				if (edge.isUnsectable())
					edge.markPals();
				else
					ray.addPals(&edge);
			}
		}
	}

	for (auto contour : contours->contours) {
		for (auto& segment : contour->segments) {
			for (auto& edge : segment.edges) {
					// copy pals if reciprocal, and points to other pals (thread_cubics2247347)
				std::vector<EdgeDistance>& pals = edge.pals;
				std::vector<EdgeDistance*> reciprocals;
				for (EdgeDistance& pal : pals) {
					bool foundReciprocal = false;
					std::vector<EdgeDistance*> locals;
					for (EdgeDistance& oPal : pal.edge->pals) {
						if (oPal.edge == &edge) {
							foundReciprocal = true;
							continue;
						}
						if (pals.end() == std::find_if(pals.begin(), pals.end(), [&oPal]
								(const EdgeDistance& test) {
								return test.edge == oPal.edge;
						})) {
							if (reciprocals.end() == std::find_if(reciprocals.begin(),
									reciprocals.end(), [&oPal](const EdgeDistance* test) {
									return test->edge == oPal.edge; }))
								locals.push_back(&oPal);
						}
					}
					if (foundReciprocal)
						reciprocals.insert(reciprocals.end(), locals.begin(), locals.end());
				}
				for (EdgeDistance* reciprocal : reciprocals) {
					pals.push_back(*reciprocal);
				}
			}
		}
	}

	// sort edges so that largest edges' winding sums are computed first
	std::vector<OpEdge*> bySize;
	for (auto contour : contours->contours) {
		for (auto& segment : contour->segments) {
			for (auto& edge : segment.edges) {
				if (edge.disabled)
					continue;
				if (edge.isUnsortable)
					continue;
				SectRay& ray = edge.ray;
				if (!ray.distances.size())
					continue;
				bySize.push_back(&edge);
			}
		}
	}
	// This used to sort by ray order, so that edges at the beginning of ray distances are resolved
	// first. The intent was to make it easier to detect bad sum chains and perhaps recompute them.
	// However, it had the downside of resolving short ambiguous edges early, and propogating bad
	// results to larger more easily resolved edges. example: thread_loops169
	std::sort(bySize.begin(), bySize.end(), [](const auto& s1, const auto& s2) {
		return s1->ptBounds.perimeter() > s2->ptBounds.perimeter(); 
	} );
	for (auto edge : bySize) {
		if (edge->sum.isSet())
			continue;
		home = edge;
		ResolveWinding resolveWinding = setWindingByDistance(contours);
		if (ResolveWinding::retry == resolveWinding) {
			workingAxis = home->ray.axis;
			std::vector<OpEdge*>& edges = Axis::horizontal == workingAxis ? inX : inY;
			auto found = std::find(edges.begin(), edges.end(), home);
			OP_ASSERT(edges.end() != found);
			size_t index = found - edges.begin();
			setSumChain(index);
			resolveWinding = setWindingByDistance(contours);
			OP_ASSERT(ResolveWinding::retry != resolveWinding);
		}
		if (ResolveWinding::fail == resolveWinding)
			OP_DEBUG_FAIL(*home, FoundWindings::fail);
	}
	for (auto contour : contours->contours) {
		for (auto& segment : contour->segments) {
			for (auto& edge : segment.edges) {
				if (edge.disabled)
					continue;
				if (EdgeFail::center == edge.rayFail)
					continue;
				if (edge.pals.size() && edge.many.isSet())
					std::swap(edge.winding, edge.many);
				if (edge.sum.isSet())
					continue;
				if (edge.isUnsortable)
					continue;
				if (edge.rayFail == EdgeFail::horizontal)
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
