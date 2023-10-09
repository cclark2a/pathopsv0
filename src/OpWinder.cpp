// (c) 2023, Cary Clark cclark2@gmail.com
#include <cmath>
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
		OpIntersection* sect = segment->addSegSect(ptTAorB, oppSegment  
				OP_DEBUG_PARAMS(SECT_MAKER(addCoincidentCheck), SectReason::coinPtsMatch));
		OpIntersection* oSect = oppSegment->addSegSect(ptTCorD, segment  
				OP_DEBUG_PARAMS(SECT_MAKER(addCoincidentCheckOpp), SectReason::coinPtsMatch));
		sect->pair(oSect);
		return IntersectResult::yes;
	}
	// pass a mix of seg and opp; construct one t for each
	int coinID = segment->coinID(flipped);
	AddMix(xyChoice, ptTAorB, flipped, cPtT, dPtT, segment, oppSegment, coinID, MatchEnds::start);
	AddMix(xyChoice, ptTCorD, flipped, aPtT, bPtT, oppSegment, segment, coinID, MatchEnds::end);
	return IntersectResult::yes;
}

// this edge has points A, B; opp edge has points C, D
// adds 0: no intersection; 1: end point only; 2: partial or full coincidence
IntersectResult OpWinder::CoincidentCheck(const OpEdge& edge, const OpEdge& opp) {
	return OpWinder::CoincidentCheck(edge.start, edge.end, opp.start, opp.end,
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
	OpIntersection* sect = segment->addCoin(ptTAorB, coinID, match, oppSegment
			OP_DEBUG_PARAMS(SECT_MAKER(addMix), SectReason::coinPtsMatch));
	OpIntersection* oSect = oppSegment->addCoin(oCoinStart, coinID, 
			flipped ? MatchEnds::start : MatchEnds::end, segment 
			OP_DEBUG_PARAMS(SECT_MAKER(addMixOpp), SectReason::coinPtsMatch));
	if (sect && oSect) // required by fuzz763_3, fuzz763_5
		sect->pair(oSect);
}

// If we got here because a pair of edges are coincident, that coincidence may have already been
// recorded when the pair of segments were checked, or the intersections may have been computed.
// !!! if this code is attempting to grow existing coin runs (maybe it is?) then it needs to 
//     remove the former ends so the result has a single start and end for each sect list run
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
	// this is called in cases as simple as two coincident line segments
	if (!aInCoincidence) {
		if (sect1) {  // segment already has intersection (segment start); e.g., line doubles back
			OP_ASSERT(!sect1->coincidenceID);
			sect1->coincidenceID = coinID;
			OP_ASSERT(MatchEnds::none == sect1->sectEnd);
			sect1->sectEnd = MatchEnds::start;
		} else	// or if it doesn't exist and isn't in a coin range, make one
			sect1 = segment->addCoin(aPtT, coinID, MatchEnds::start, oppSegment
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_aPtT), SectReason::coinPtsMatch));
	}
	if (!bInCoincidence) {
		if (sect2) {  // segment already has intersection (segment end); e.g., line doubles back
			OP_ASSERT(!sect2->coincidenceID);
			sect2->coincidenceID = coinID;
			OP_ASSERT(MatchEnds::none == sect2->sectEnd);
			sect2->sectEnd = MatchEnds::end;
		} else
			sect2 = segment->addCoin(bPtT, coinID, MatchEnds::end, oppSegment
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_bPtT), SectReason::coinPtsMatch));
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
		oSect1 = oppSegment->addCoin(oCoinStart, coinID, flipped ? MatchEnds::end : MatchEnds::start, segment
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_oppStart), SectReason::coinPtsMatch));
		if (!oSect1)  // triggered by fuzz763_3 
			return IntersectResult::fail; // triggered by pentrek7 (maybe)
		if (!sect1)
			return IntersectResult::fail; // triggered by pentrek7 (maybe)
		sect1->pair(oSect1);
	} else {  // segment already has intersection (start or end); e.g., line doubles back
		if (!(inCoinRange(oRange, oSect1->ptT.t, nullptr) & 1)) {
			OP_ASSERT(!oSect1->coincidenceID);
			oSect1->coincidenceID = coinID;
			OP_ASSERT(MatchEnds::none == oSect1->sectEnd);
			oSect1->sectEnd = flipped ? MatchEnds::end : MatchEnds::start;	// !!! added without testing
		}
	}
	if (!oSect2) {
		float eEnd = bPtT.pt.choice(xyChoice);
		OpPtT oCoinEnd{ bPtT.pt, cPtT.t + (eEnd - oStart) / oXYRange * oTRange };
		OP_ASSERT(OpMath::Between(cPtT.t, oCoinEnd.t, dPtT.t));
		oSect2 = oppSegment->addCoin(oCoinEnd, coinID, flipped ? MatchEnds::start : MatchEnds::end, segment
				OP_DEBUG_PARAMS(SECT_MAKER(addPair_oppEnd), SectReason::coinPtsMatch));
		if (!oSect2)
			return IntersectResult::fail; // triggered by fuzz763_13
		if (!sect2)
			return IntersectResult::fail;
		sect2->pair(oSect2);
	} else {  // segment already has intersection (start or end); e.g., line doubles back
		if (!(inCoinRange(oRange, oSect2->ptT.t, nullptr) & 1)) {
			OP_ASSERT(!oSect2->coincidenceID);
			oSect2->coincidenceID = coinID;
			OP_ASSERT(MatchEnds::none == oSect2->sectEnd);
			oSect2->sectEnd = flipped ? MatchEnds::start : MatchEnds::end;
		}
	}
	return IntersectResult::yes;
}


// !!! conditionally, upscale t so we can call segment line curve intersection to see what 
//     bugs that introduces
// !!! I'm bothered that segment / segment calls a different form of this
void OpWinder::AddLineCurveIntersection(OpEdge& opp, OpEdge& edge, bool secondAttempt) {
#if 0 && OP_DEBUG
	OP_ASSERT(edge.segment->c.ptAtT(edge.start.t) == edge.start.pt);
	OP_ASSERT(edge.segment->c.ptAtT(edge.end.t) == edge.end.pt);
	OP_ASSERT(opp.segment->c.ptAtT(opp.start.t) == opp.start.pt);
	OP_ASSERT(opp.segment->c.ptAtT(opp.end.t) == opp.end.pt);
#endif
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
	if (2 == septs.count && opp.isLinear())
		return (void) CoincidentCheck(edge, opp);
	bool tInRange = false;
	for (unsigned index = 0; index < septs.count; ++index) {
#if USE_SEGMENT_SECT
		if (opp.start.t > septs.get(index) || septs.get(index) > opp.end.t)
			continue;
		tInRange = true;
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
		OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
		if (0 == edgeT)
			oppPtT.pt = eSegment->c.pts[0];
		else if (1 == edgeT)
			oppPtT.pt = eSegment->c.lastPt();
        // pin point to both bounds, but only if it is on edge
		OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
//		OP_DEBUG_CODE(OpPoint debugPt = oppPtT.pt);
        oSegment->ptBounds.pin(&oppPtT.pt);
//      eSegment->ptBounds.pin(&oppPtT.pt);	// !!! doubtful this is needed with contains test above
//		OP_ASSERT(debugPt == oppPtT.pt);	// rarely needed, but still triggered (e.g., joel_15x)
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
	if (!tInRange && opp.isLine_impl && !secondAttempt)
		AddLineCurveIntersection(edge, opp, true);
}

EdgeDistance* SectRay::find(OpEdge* edge) {
	for (auto test = &distances.back(); test >= &distances.front(); --test) {
		if (test->edge == edge)
			return test;
	}
	return nullptr;
}

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
		if (!test->edge->ray.distances.size())
			return;
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
	float lowLimit = std::nextafterf(homeCept, -OpInfinity);
	while (test > &distances.front() && (--test)->cept >= lowLimit) {
		OP_ASSERT((test + 1)->cept >= test->cept);
		matchCept(test);
	}
	test = homeDist;
	float highLimit = std::nextafterf(homeCept, +OpInfinity);
	while (test < &distances.back() && (++test)->cept <= highLimit) {
		OP_ASSERT((test - 1)->cept <= test->cept);
		matchCept(test);
	}
}

// at some point, do some math or rigorous testing to figure out how extreme this can be
// for now, keep making it smaller until it breaks
#define WINDING_NORMAL_LIMIT  0.004 // !!! fails (I think) on pentreck13 edge 1045 NxR:00221

FindCept SectRay::findIntercept(OpEdge* test) {
	if (test->ptBounds.ltChoice(axis) > normal)
		return FindCept::ok;
	if (test->ptBounds.rbChoice(axis) < normal)
		return FindCept::ok;
	if (test->unsortable)
		return FindCept::unsortable;
	if (test->disabled)
		return FindCept::ok;
	// start here (eventually)
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
	if (1 < roots.count) {
		test->setUnsortable();  // triggered by joel_14x (very small cubic)
		return FindCept::unsortable;
	}
	if (1 != roots.count) {
		OP_ASSERT(0 == roots.count);
		return FindCept::retry;
	}
	float root = roots.get(0);
#if RAY_USE_SEGMENT
	if (OpMath::IsNaN(root) || test->start.t >= root || root >= test->end.t)
		return FindCept::retry;
#else
	if (OpMath::IsNaN(root) || 0 == root || root == 1)
		return FindCept::retry;
#endif
	bool overflow;
	OpVector tangent = testCurve.tangent(root).normalize(&overflow);
	if (overflow)
		return FindCept::retry;
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	OpVector backRay = -ray;
	float tNxR = tangent.cross(backRay);
	if (fabs(tNxR) < WINDING_NORMAL_LIMIT)
		return FindCept::retry;
	OpPoint pt = testCurve.ptAtT(root);
	Axis perpendicular = !axis;
	float testXY = pt.choice(perpendicular);
#if 0 && OP_DEBUG
	OpDebugOut("testXY:" + OpDebugDumpHex(testXY) + " homeCept:" + OpDebugDumpHex(homeCept)
			+ " nextafter:" + OpDebugDumpHex(std::nextafterf(testXY, homeCept))
			+ " nextafter(testXY, homeCept) == homeCept:"
			+ (std::nextafterf(testXY, homeCept) == homeCept ? "true\n" : "false\n"));
#endif
	bool reversed = tangent.dot(homeTangent) < 0;
	distances.emplace_back(test, testXY, root, reversed);
	return std::nextafterf(testXY, homeCept) == homeCept ? FindCept::unsectable : FindCept::ok;
}

FoundIntercept OpWinder::findRayIntercept(size_t inIndex, OpVector homeTan, float normal, 
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
	ray.homeT = home->center.t;
	// if find intercept fails, retry some number of times
	// if all retries fail, distinguish between failure cases
	//   if it failed because closest edge was too close, mark pair as unsectable
	std::vector<EdgeDistance> touching;
	do {
		// !!! restructure this slightly to break out to try a different center when touching is pushed back,
		// unless it's the last go round; then, find all ray intersections before marking it unsectable/unsortable
		size_t touches = touching.size();
		ray.distances.clear();
		ray.distances.emplace_back(home, homeCept, ray.homeT, false);
		int index = inIndex;
		// start at edge with left equal to or left of center
		FindCept findCept = FindCept::ok;
		while (index != 0) {
			OpEdge* test = inArray[--index];
			if (test == home)
				continue;
			findCept = ray.findIntercept(test);
			if (FindCept::unsectable == findCept) {
				EdgeDistance& tDist = ray.distances.back();
				if (home->isLinear() && test->isLinear())
					// !!! if home is long and test is short, may need to see if trying again
					// with normal outside test gives sectable result. Wait for that test case
					// to code
					// !!! not sure how to assert for this
					home->addPal(tDist);
				else if (touches == touching.size())
					touching.push_back(ray.distances.back());
			} else if (FindCept::retry == findCept)
				goto tryADifferentCenter;
		}
		if (touches == touching.size())
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
		float homeMidT = homeCurve.center(workingAxis, middle);
		if (OpMath::IsNaN(homeMidT) || mid <= 1.f / 256.f) {  // give it at most eight tries
			// look for the same edge touching multiple times; the pair are unsectable
			if (FindCept::unsectable == findCept) {
				markUnsortable();
				return FoundIntercept::fail;	// nonfatal error (!!! give it a different name!)
			}
			while (touching.size()) {
				EdgeDistance& touch = touching[0];
				OpEdge* test = touch.edge;
				int count = std::count_if(touching.begin(), touching.end(), 
						[&test](auto dist){ return dist.edge == test; });
				if (count > 1)
					home->addPal(touch);
				touching.erase(std::remove_if(touching.begin(), touching.end(), 
						[&test](auto dist){ return dist.edge == test; }), touching.end());
			}
			if (!home->unsectableID && !home->pals.size())
				markUnsortable();
			return FoundIntercept::fail;	// nonfatal error (!!! give it a different name!)
		}
		// if find ray intercept can't find, restart with new center, normal, distance, etc.
		ray.homeCept = homeCept = homeCurve.ptAtT(homeMidT).choice(perpendicular);
		OP_ASSERT(!OpMath::IsNaN(homeCept));
		ray.normal = normal = homeCurve.ptAtT(homeMidT).choice(workingAxis);
		ray.homeT = homeMidT;
		OP_ASSERT(!OpMath::IsNaN(normal));
	} while (true);
}

void OpWinder::markUnsortable() {
	if (Axis::vertical == workingAxis || inY.end() == std::find(inY.begin(), inY.end(), home)) 
		home->setUnsortable();
	home->rayFail = Axis::vertical == workingAxis ? EdgeFail::vertical : EdgeFail::horizontal;
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
	OpVector homeTangent = edgeSeg->c.tangent(home->center.t);
	float NxR = homeTangent.normalize(&overflow).cross(rayLine);
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
	FoundIntercept foundIntercept = findRayIntercept(inIndex, homeTangent, normal, homeCept);
	OP_ASSERT(home->ray.distances.size());
	SectRay& ray = home->ray;
	std::sort(ray.distances.begin(), ray.distances.end(), 
			[](const EdgeDistance& s1, const EdgeDistance& s2) {
		return s1.cept < s2.cept;
	});
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
		if (home->pals.size() || home->unsectableID)
			home->setUnsortable();
		else {
			OpWinding prev(WindingTemp::dummy);
			// look at direction of edge relative to ray and figure winding/oppWinding contribution
			if (CalcFail::fail == home->addIfUR(ray.axis, ray.distances[0].t, &prev))
				home->setUnsortable();
			else
				OP_EDGE_SET_SUM(home, prev);
		}
		return ResolveWinding::resolved;
	}
	// walk from the known sum to (and including) the edge (e.g., edge 669 in issue1417)
	//   if an edge is in another edge's pal list, even if it is not reciprocal: 
	//   only count the edge with the pal list
	// might require 2 passes; one to find edge with pals, one to mark those edges not to be counted
	int rayIndex = -1;  // pal detection needs to start before first valid sum
	OpEdge* palEdge;
	std::vector<OpEdge*> pals;
	while ((palEdge = ray.distances[++rayIndex].edge) != home) {
		if (!palEdge->pals.size())
			continue;
		if (pals.end() != std::find_if(pals.begin(), pals.end(), [&palEdge](OpEdge* palOwner) { 
				return palEdge->isPal(palOwner) && palOwner->isPal(palEdge); }))
			continue;
		pals.push_back(palEdge);
	}
	if (pals.size()) {
		rayIndex = -1;
		OpEdge* test;
		while ((test = ray.distances[++rayIndex].edge) != home) {
			if (pals.end() != std::find(pals.begin(), pals.end(), test))
				continue;
			auto testInEdge = std::find_if(pals.begin(), pals.end(), [&test](OpEdge* palOwner) { 
					return !test->isPal(palOwner) && palOwner->isPal(test); });
			if (pals.end() != testInEdge) {
				ray.distances[rayIndex].skipPal = (*testInEdge)->sum.isSet();
				ray.distances[rayIndex].skipSum = true;
			}
		}
	}
	// starting with found or zero if none, accumulate sum up to winding
	OpWinding sumWinding(WindingTemp::dummy);
	int sumIndex = ray.distances.size();
	while (ray.distances[--sumIndex].edge != home) 
		OP_ASSERT(sumIndex > 0);
	float homeT = ray.distances[sumIndex].t;  // used by unsectable, later
	while (--sumIndex >= 0 && (ray.distances[sumIndex].edge->pals.size() 
			|| !ray.distances[sumIndex].edge->sum.isSet() || ray.distances[sumIndex].skipSum))
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
	OpEdge* prior;
	do {
		OP_ASSERT(sumIndex + 1 < (int) ray.distances.size());
		EdgeDistance& dist = ray.distances[++sumIndex];
		prior = dist.edge;
		if (home->pals.size() && (home == prior || home->isPal(prior)))
			break;
		NormalDirection normDir = prior->normalDirection(ray.axis, dist.t);
		if (NormalDirection::underflow == normDir || NormalDirection::overflow == normDir) {
			prior->setUnsortable();
			continue;
		}
		if (dist.skipPal)  // true if another distance in ray has a pal list with prior in it
			continue;
		if (NormalDirection::downLeft == normDir && !prior->pals.size() && !dist.skipSum)
			OP_EDGE_SET_SUM(prior, sumWinding);
		if (CalcFail::fail == prior->addSub(ray.axis, dist.t, &sumWinding)) // if d/l sub; if u/r add
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
		if (NormalDirection::upRight == normDir && !prior->pals.size() && !dist.skipSum)
			OP_EDGE_SET_SUM(prior, sumWinding);
	} while (home != prior);
	if (!home->pals.size())
		return ResolveWinding::resolved;
	// if home is unsectable, set its sum winding as if all of its pals' windings were a single edge
	OP_ASSERT(!home->many.isSet());
	// winding must be replaced by all unsectable windings -- however, other unsectables will want 
	//   to see the original winding. This is why 'many' is used. After all sums are computed
	//   replace winding with many.
	home->many = home->winding;	// back up winding
	for (const auto& pal : home->pals) {
		home->winding.move(pal.edge->winding, contours, pal.reversed);
		if (!home->winding.visible())
			home->setDisabled(OP_DEBUG_CODE(ZeroReason::palWinding));
	}
	if (CalcFail::fail == home->addIfUR(ray.axis, homeT, &sumWinding))
		home->setUnsortable();
	else
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
			if (home->ray.distances.size() && EdgeFail::none == home->rayFail)
				continue;
			if (home->disabled)	// may not be visible in vertical pass
				continue;
			if (home->between)
				continue;
			if (EdgeFail::horizontal == home->rayFail && Axis::vertical == workingAxis)
				home->rayFail = EdgeFail::none;
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
				SectRay& ray = edge.ray;
				if (edge.disabled)
					continue;
				if (edge.unsortable)
					continue;
				if (edge.unsectableID)
					edge.markPals();
				else
					ray.addPals(&edge);
			}
		}
	}
	// sort edges so that largest edges' winding sums are computed first
	std::vector<OpEdge*> bySize;
	for (auto& contour : contours->contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				if (edge.disabled)
					continue;
				if (edge.unsortable)
					continue;
				SectRay& ray = edge.ray;
				if (!ray.distances.size())
					continue;
				bySize.push_back(&edge);
			}
		}
	}
	std::sort(bySize.begin(), bySize.end(), [](const auto& s1, const auto& s2) {
		return s1->ptBounds.perimeter() > s2->ptBounds.perimeter(); 
	} );
	for (auto edge : bySize) {
		if (edge->sum.isSet())
			continue;
		home = edge;
		ResolveWinding resolveWinding = setWindingByDistance(contours);
		if (ResolveWinding::fail == resolveWinding)
			OP_DEBUG_FAIL(*home, FoundWindings::fail);
	}
	for (auto& contour : contours->contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				if (edge.disabled)
					continue;
				if (edge.pals.size())
					std::swap(edge.winding, edge.many);
				if (edge.sum.isSet())
					continue;
				if (edge.unsortable)
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
