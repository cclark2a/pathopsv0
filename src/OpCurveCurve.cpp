// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include <utility>

OpCurveCurve::OpCurveCurve(OpEdge* edge, OpEdge* opp)
	: originalEdge(edge)
	, originalOpp(opp)
	, sectResult(false) {
	edge->contours()->reuse(edge->contours()->ccStorage);
	edgeCurves.emplace_back(edge);
	oppCurves.emplace_back(opp);
#if OP_DEBUG_DUMP
	edge->contours()->debugCurveCurve = this;
//		debugSaveID();
#endif
}


// trim front and back of ranges
SectFound OpCurveCurve::addUnsectable() {
	findEdgesTRanges(CurveRef::edge);
	findEdgesTRanges(CurveRef::opp);
	Axis larger = originalEdge->ptBounds.width() > originalEdge->ptBounds.height() ? 
			Axis::vertical : Axis::horizontal;
	for (auto edgePtr : edgeRuns) {
		auto& edge = *edgePtr;
		for (auto oppPtr : oppRuns) {
			auto& opp = *oppPtr;
			if (opp.ptBounds.ltChoice(larger) >= edge.ptBounds.rbChoice(larger)
					|| opp.ptBounds.rbChoice(larger) <= edge.ptBounds.ltChoice(larger))
				continue;
			auto findMatch = [larger](float xy, OpEdge& edge, OpPtT& found) {
				if (xy == edge.start.pt.choice(larger))
					found = edge.start;
				else if (xy == edge.end.pt.choice(larger))
					found = edge.end;
				else {
					found.pt.x = OpNaN;
					found.t = edge.findT(larger, xy);
				}
			};
			OpPtT edgeStart(SetToNaN::dummy), edgeEnd(SetToNaN::dummy),
					oppStart(SetToNaN::dummy), oppEnd(SetToNaN::dummy);
			float minXY = std::max(edge.ptBounds.ltChoice(larger), 
					opp.ptBounds.ltChoice(larger));
			float maxXY = std::min(edge.ptBounds.rbChoice(larger), 
					opp.ptBounds.rbChoice(larger));
			OP_ASSERT(minXY < maxXY);
			if (edge.start.pt.choice(larger) > edge.end.pt.choice(larger))
				std::swap(minXY, maxXY); // make max/min agree with edge start/end
			findMatch(minXY, edge, edgeStart);
			findMatch(maxXY, edge, edgeEnd);
			findMatch(minXY, opp, oppStart);
			findMatch(maxXY, opp, oppEnd);
			OP_ASSERT(!OpMath::IsNaN(edgeStart.t));  // !!! troubling: tripped by fuzz763_9 once...
			OP_ASSERT(!OpMath::IsNaN(edgeEnd.t));
			OP_ASSERT(!OpMath::IsNaN(oppStart.t));
			OP_ASSERT(!OpMath::IsNaN(oppEnd.t));
			if (!edgeStart.pt.isFinite())
				edgeStart.pt = oppStart.pt;
			if (!edgeEnd.pt.isFinite())
				edgeEnd.pt = oppEnd.pt;
			if (!oppStart.pt.isFinite())
				oppStart.pt = edgeStart.pt;
			if (!oppEnd.pt.isFinite())
				oppEnd.pt = edgeEnd.pt;
			bool flipped = oppStart.t > oppEnd.t;
			if (flipped)
				std::swap(oppStart, oppEnd); // flip to fix opp t (now opp pts are flipped vs. edge)
			OpSegment* segment = const_cast<OpSegment*>(edge.segment);
			int usectID = segment->nextID();
			OpIntersection* segSect1 = segment->addUnsectable(edgeStart, usectID, 
					MatchEnds::start, opp.segment  OP_DEBUG_PARAMS(SECT_MAKER(unsectableStart)));
			OpIntersection* segSect2 = segment->addUnsectable(edgeEnd, usectID,
					MatchEnds::end, opp.segment  OP_DEBUG_PARAMS(SECT_MAKER(unsectableEnd)));
			OpSegment* oppSegment = const_cast<OpSegment*>(opp.segment);
			OpIntersection* oppSect1 = oppSegment->addUnsectable(oppStart, 
					flipped ? -usectID : usectID, flipped ? MatchEnds::end : MatchEnds::start, segment  
					OP_DEBUG_PARAMS(
				    flipped ? IntersectMaker::unsectableOppEnd : IntersectMaker::unsectableOppStart,
					__LINE__, __FILE__));
			OpIntersection* oppSect2 = oppSegment->addUnsectable(oppEnd, 
					flipped ? -usectID : usectID, flipped ? MatchEnds::start : MatchEnds::end, segment  
					OP_DEBUG_PARAMS(
				    flipped ? IntersectMaker::unsectableOppStart : IntersectMaker::unsectableOppEnd,
					__LINE__, __FILE__));
			segSect1->pair(flipped ? oppSect2 : oppSect1);
			segSect2->pair(flipped ? oppSect1 : oppSect2);
		}
	}

	return SectFound::intersects;
}

#if CC_EXPERIMENT
// Scan through opposite curves and see if check point is inside deleted bounds. If so, use a
// different (but close by if possible) point to split the curve.
void OpCurveCurve::checkSplit(float loT, float hiT, CurveRef which, OpPtT& checkPtT) const {
	const OpEdge* opp = CurveRef::edge == which ? originalOpp : originalEdge;
	OpPtT loOpp = opp->start;
	const std::vector<OpEdge*>& oCurves = CurveRef::edge == which ? oppCurves : edgeCurves;
	auto checkO = [&checkPtT, &loOpp](const OpPtT& hiOpp, const OpPtT& endPt) {
		OP_ASSERT(loOpp.t <= hiOpp.t);
		if (loOpp.t == hiOpp.t) {
			loOpp = endPt;
			return true;
		}
		OpPointBounds bounds;
		bounds.set(loOpp.pt, hiOpp.pt);
		if (!bounds.isEmpty() && bounds.contains(checkPtT.pt))
			return false;
		loOpp = endPt;
		return true;

	};
	float startingT = checkPtT.t;
	float deltaT = OpEpsilon;
	do {
		for (OpEdge* oCurve : oCurves) {
			if (!oCurve->ccOverlaps)
				continue;
			if (!checkO(oCurve->start, oCurve->end))
				goto tryAgain;
		}
		if (checkO(opp->end, opp->end))
			return;
	tryAgain:
		checkPtT.t = startingT + deltaT;
		if (opp->start.t > checkPtT.t || checkPtT.t > opp->end.t) {
			checkPtT.t = startingT - deltaT;
			OP_ASSERT(opp->start.t < checkPtT.t && checkPtT.t < opp->end.t);
		}
		const OpEdge* edge = CurveRef::edge == which ? originalEdge : originalOpp;
		checkPtT.pt = edge->segment->c.ptAtT(checkPtT.t);
		deltaT *= -2;
	} while (true);
}
#endif

// this marks opp as splittable if edge is a curve
// this finds opp t if edge is a line
// runs twice: edge/opp, then opp/edge
// result is for all edges: no (intersections at all); fail; (something) split; line sect (no splits)
SectFound OpCurveCurve::curvesIntersect(CurveRef curveRef) {
	auto& edgeParts = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	auto& oppParts = CurveRef::edge == curveRef ? oppCurves : edgeCurves;
	SectFound result = SectFound::no;	// assumes no pair intersects; we're done
	for (auto edgePtr : edgeParts) {
		auto& edge = *edgePtr;
		if (edge.start.t >= edge.center.t || edge.center.t >= edge.end.t) {
			OP_ASSERT(OpDebugExpect::unknown == edge.contours()->debugExpect);
			return SectFound::fail;  // triggered by fuzz763_47
		}
		if (edge.isLine_impl)
			return SectFound::fail;  // triggered by fuzz753_91
		// rotate each to see if tight rotated bounds (with extrema) touch
		LinePts edgeLine { edge.start.pt, edge.end.pt };
		OpCurve& edgeRotated = const_cast<OpCurve&>(edge.setVertical());
		if (!edgeRotated.isFinite())
			return SectFound::fail;  // triggered by fuzzhang_1
		OpTightBounds edgeRotatedBounds(edgeRotated);
		for (auto oppPtr : oppParts) {
			auto& opp = *oppPtr;
			if (opp.isLine_impl)
				return SectFound::fail;  // triggered by fuzz763_4
			if (!edge.ptBounds.intersects(opp.ptBounds))
				continue;
			const OpCurve& oppCurve = opp.setCurve();
			OpCurve oppRotated = oppCurve.toVertical(edgeLine);
			if (!oppRotated.isFinite())
				OP_DEBUG_FAIL(*originalOpp, SectFound::fail);
			OpTightBounds oppRotatedBounds(oppRotated);
			if (!edgeRotatedBounds.intersects(oppRotatedBounds))
				continue;
			result = SectFound::split;	// an edge is to be split
			opp.doSplit = EdgeSplit::yes;
		}
	}
	return result;
}

/* note: does not (currently) reject via points bounds; rejects via rotated tight bounds 
  after an edge is compared against all opposites, if tight bounds didn't intersect, it isn't copied
 */
/* general idea:
	- on each pass, break curves into smaller ones
	- don't break if smaller piece is linear
	- discard curves and lines whose tight rotated bounds does not intersect any opposite curve
	- if linear, intersect as line with other curve (or line)
	- only linear pieces can generate intersections?
		- what about curve pieces whose ends intersect opposite curves?
	- linear/curve intersection has several outcomes:
		- no intersection
		- touch (both ends of curve are on same side of line, plus curve intersects line)
		- cross (ends of curves are on opposite sides, plus curve intersects line)
		- coincident (both ends of curve touch line, plus curve intersects line)
		- double cross (both ends touch line, but curve does not intersect in the middle)
		- near coincident (one end of curve touches line, plus middle interesects)
	- keep all results before generating intersections (or before sects are added to segment list)
	- discard linear edges after processing, so they don't generate results on the next pass

*/
SectFound OpCurveCurve::divideAndConquer() {
#if OP_DEBUG_IMAGE
	bool breakAtDraw = 27 == originalEdge->id && 131 == originalOpp->id;
	if (breakAtDraw) {
		hideOperands();
		hideSegmentEdges();
		showTemporaryEdges();
		showPoints();
		showValues();
		showGrid();
	}
#endif
	OP_ASSERT(1 == edgeCurves.size());
	OP_ASSERT(0 == edgeLines.size());
	OP_ASSERT(1 == oppCurves.size());
	OP_ASSERT(0 == oppLines.size());
//	edgeCurves[0].addMatchingEnds(oppCurves[0]);
	for (int depth = 1; depth < maxDepth; ++depth) {
#if OP_DEBUG_IMAGE
		if (breakAtDraw && 8 <= depth)
			OpDebugOut("");  // allows setting a breakpoint to debug curve/curve
#endif
		if (edgeLines.size() && oppLines.size())
			linearIntersect(edgeLines, oppLines);
		if (edgeLines.size())
			linearIntersect(edgeLines, oppCurves);
		if (oppLines.size())
			linearIntersect(oppLines, edgeCurves);
		SectFound oppResult = curvesIntersect(CurveRef::edge);
		if (SectFound::fail == oppResult)
			return oppResult;  // triggered by fuzzhang_1
		SectFound edgeResult = curvesIntersect(CurveRef::opp);
		if (SectFound::fail == edgeResult)
			OP_DEBUG_FAIL(*originalEdge, edgeResult);
		if (SectFound::no == oppResult || SectFound::no == edgeResult)
			return SectFound::no;
		if (edgeCurves.size() >= maxSplits || oppCurves.size() >= maxSplits)
			return addUnsectable();
		// old linear edges are removed from list by split below
		// iterate through all edges and mark ones that are linear
		// if one side has split more than double the other, do it again
		DoSplit splitType = DoSplit::marked;
		bool keepGoing;
		edgeLines.clear();
		oppLines.clear();
		do {
			keepGoing = false;
			if (splitType == DoSplit::marked || tooFew(CurveRef::edge)) {
				if (!split(CurveRef::edge, splitType))
					return SectFound::no;
				keepGoing = true;
			}
			if (splitType == DoSplit::marked || tooFew(CurveRef::opp)) {
				if (!split(CurveRef::opp, splitType))
					return SectFound::no;
				keepGoing = true;
			}
			splitType = DoSplit::all;
		} while (keepGoing);
		// draw rays through the center of all edges, tracking if each is left/top or right/bottom
#if OP_DEBUG_IMAGE
		if (breakAtDraw) {
				draw();
			if (depth >= 5) {
				OpDebugOut("");  // allows setting a breakpoint to debug curve/curve
			}
		}
#endif
	}
// OP_ASSERT(0);  // triggered by release_13, a fuzzer test expected to fail
	return SectFound::fail;
}

#if CC_EXPERIMENT
// Divide only by geometric midpoint. Midpoint is determined by endpoint intersection, or
// curve bounds if no intersection is found.
// try several approaches:
// 1) call existing code for baseline (bounds split on geometric middle, find curve root with line)
// 2) rotate curve to opp start/end line and use newton method to find intersection
//    if we're going to rotate, take advantage and do another bounds check
// 3) if line/line intersects, divide at that point
// 4) if line/line intersects, divide at corresponding ts
// 5) split geometric until t doesn't change (expect to be slow)
// 6) binary search on t (expect to be slower)
SectFound OpCurveCurve::divideExperiment() {
#if OP_DEBUG_IMAGE
	bool breakAtDraw = 27 == originalEdge->id && (97 == originalOpp->id || 33 == originalOpp->id);
	if (breakAtDraw) {
		playback();
		OpDebugOut("");
	}
#endif
	OP_ASSERT(1 == edgeCurves.size());
	OP_ASSERT(0 == edgeLines.size());
	OP_ASSERT(1 == oppCurves.size());
	OP_ASSERT(0 == oppLines.size());
	// 3)
	auto rotatedIntersect = [](OpEdge& edge, OpEdge& opp, bool* sectedPtr) {
		LinePts edgePts { edge.start.pt, edge.end.pt };
		const OpCurve& edgeCurve = edge.setCurve();
		OpCurve edgeRotated = edgeCurve.toVertical(edgePts);
		if (!edgeRotated.isFinite())
			return false;
		OpPointBounds eRotBounds;
		eRotBounds.set(edgeRotated);
		const OpCurve& oppCurve = opp.setCurve();
		OpCurve oppRotated = oppCurve.toVertical(edgePts);
		if (!oppRotated.isFinite())
			return false;
		OpPointBounds oRotBounds;
		oRotBounds.set(oppRotated);
		*sectedPtr = eRotBounds.intersects(oRotBounds);
		return true;
	};
	auto addSect = [this]() {
		if (!ccSects.size())
			return SectFound::no;
		for (CcSects& s : ccSects) {
			OpSegment* eSegment = const_cast<OpSegment*>(s.e.edge.segment);
			OpSegment* oSegment = const_cast<OpSegment*>(s.o.edge.segment);
			OpIntersection* sect = eSegment->addEdgeSect(s.e.ptT  
					OP_DEBUG_PARAMS(s.e.maker, __LINE__, std::string(__FILE__), SectReason::lineCurve, 
					&s.e.edge, &s.o.edge));
			OpIntersection* oSect = oSegment->addEdgeSect(s.o.ptT  
					OP_DEBUG_PARAMS(s.o.maker, __LINE__, std::string(__FILE__), SectReason::lineCurve, 
					&s.e.edge, &s.o.edge));
			sect->pair(oSect);
		}
		return SectFound::intersects;
	};
	auto recordSect = [this](OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT
				OP_DEBUG_PARAMS(IntersectMaker eMaker, IntersectMaker oMaker)) {
		snipEdge = edgePtT;
		snipOpp = oppPtT;
		OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
		OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
		// end points matching has already been recorded, so don't do it again
		if ((0 == edgePtT.t || 1 == edgePtT.t) && (0 == oppPtT.t || 1 == oppPtT.t))
			return;
		if (const OpIntersection* eClose = eSegment->sects.nearly(edgePtT, nullptr); eClose) {
			if (eClose->segment == oSegment)
				return;
			edgePtT = eClose->ptT;
		}
		if (const OpIntersection* oClose = oSegment->sects.nearly(oppPtT, nullptr); oClose) {
			if (oClose->segment == eSegment)
				return;
			oppPtT = oClose->ptT;
		}
		OpPtT::MeetInTheMiddle(edgePtT, oppPtT);
		if (eSegment->sects.nearly(edgePtT, oSegment))
			// even though intersection is already recorded, report here so edges can be trimmed
			return;
		if (oSegment->sects.nearly(oppPtT, eSegment))
			return;
		// note that that sects are not added until the end
		ccSects.emplace_back(edge, edgePtT, opp, oppPtT  OP_DEBUG_PARAMS(eMaker, oMaker));
	};
	auto ifExactly = [recordSect](OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT) {
		if (edgePtT.pt != oppPtT.pt)
			return false;
		recordSect(edge, edgePtT, opp, oppPtT 
				OP_DEBUG_PARAMS(IntersectMaker::edgeCCExact, IntersectMaker::oppCCExact));
		return true;
	};
	auto ifNearly = [recordSect](OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT) {
		if (!edgePtT.pt.isNearly(oppPtT.pt))
			return false;
		recordSect(edge, edgePtT, opp, oppPtT  
				OP_DEBUG_PARAMS(IntersectMaker::edgeCCNearly,  IntersectMaker::oppCCNearly));
		return true;
	};
	auto addIntersection = [recordSect](OpEdge& edge, OpEdge& opp) {
		OpPtT edgePtT;
		OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
		if (!edge.start.t || edge.start.pt == eSegment->c.pts[0])
			edgePtT = { eSegment->c.pts[0], 0 };
		else if (1 == edge.end.t || edge.end.pt == eSegment->c.lastPt())
			edgePtT = { eSegment->c.lastPt(), 1 };
		else
			edgePtT = edge.start;
		OpPtT oppPtT = { edgePtT.pt, 0 };
		if (edgePtT.pt != opp.segment->c.pts[0]) {
			if (edgePtT.pt == opp.segment->c.lastPt())
				oppPtT.t = 1;
			else {
				opp.segment->findPtT(0, 1, edgePtT.pt, &oppPtT.t);
				oppPtT.t = std::max(oppPtT.t, opp.start.t);
				oppPtT.t = std::min(oppPtT.t, opp.end.t);
				if (OpMath::IsNaN(oppPtT.t))
					oppPtT.t = opp.start.t;
			}
		}
		recordSect(edge, edgePtT, opp, oppPtT  
				OP_DEBUG_PARAMS(IntersectMaker::edgeCurveCurve, IntersectMaker::oppCurveCurve));
	};
	auto hullSect = [](OpEdge& edge, OpEdge& opp) {
		const OpCurve& eCurve = edge.setCurve();
		int ptCount = eCurve.pointCount();
		LinePts edgePts;
		edgePts.pts[1] = eCurve.pts[0];
		for (int index = 1; index <= ptCount; ++index) {
			edgePts.pts[0] = edgePts.pts[1];
			int endHull = index < ptCount ? index : 0;
			edgePts.pts[1] = eCurve.pts[endHull];
			OpRootPts septs = opp.segment->c.lineIntersect(edgePts, opp.start.t, opp.end.t);
			if (septs.count)
				OpDebugOut("");
			for (size_t inner = 0; inner < septs.count; ++inner) {
				// !!! experiment: try pinning result to hull edge bounds
				septs.ptTs[inner].pt.pin(edgePts.pts[0], edgePts.pts[1]);
				opp.hulls.emplace_back(&edge, septs.ptTs[inner], endHull);
			}
		}
	};
	auto addHullSect = [hullSect, recordSect](OpEdge& edge, OpEdge& opp, CurveRef curveRef) {
		hullSect(opp, edge);
		if (2 > edge.hulls.size())
			return false;
		const HullSect* endToEnd = &edge.hulls.back();
		if (endToEnd->end)
			return false;
		while (endToEnd > &edge.hulls.front() && !(endToEnd - 1)->end)
			--endToEnd;
		const HullSect* ctrlEdge = &edge.hulls.front();
		while (ctrlEdge->end) {
			const OpPtT& ctrlSect = ctrlEdge->sect;
			const HullSect* eToETest = endToEnd;
			const OpPtT& eToESect = eToETest->sect;
			do {
				if (ctrlSect.pt.isNearly(eToESect.pt) || (ctrlSect.t + OpEpsilon >= eToESect.t 
						&& ctrlSect.t <= eToESect.t + OpEpsilon)) {
					OpVector eToE = opp.end.pt - opp.start.pt;
					Axis axis = fabsf(eToE.dx) > fabsf(eToE.dy) ? Axis::vertical : Axis::horizontal;
					OpPtT oppPtT;
					if (opp.ptBounds.contains(ctrlSect.pt)) {
						oppPtT = opp.setCurve().findIntersect(axis, ctrlSect);
						oppPtT.t = OpMath::Interp(opp.start.t, opp.end.t, oppPtT.t);
					} else {
						float startDistSq = (ctrlSect.pt - opp.start.pt).lengthSquared();
						float endDistSq = (ctrlSect.pt - opp.end.pt).lengthSquared();
						oppPtT = startDistSq < endDistSq ? opp.start : opp.end;
					}
					if (CurveRef::edge == curveRef)
						recordSect(edge, ctrlSect, opp, oppPtT  OP_DEBUG_PARAMS(
								IntersectMaker::edgeCCHull, IntersectMaker::oppCCHull));
					else
						recordSect(opp, oppPtT, edge, ctrlSect  OP_DEBUG_PARAMS(
								IntersectMaker::oppCCHull, IntersectMaker::edgeCCHull));
					return true;
				}
			} while (++eToETest <= &edge.hulls.back());
			++ctrlEdge;
		}
		return false;
	};
	auto rangeRatio = [](const std::vector<OpEdge*>& curves, const OpEdge* original) {
		float originalRange = original->end.t - original->start.t;
		float piecewiseRange = curves.back()->end.t - curves.front()->start.t;
		return piecewiseRange / originalRange;
	};
#if 0
	auto hackOfAddUnsectable = [this]() {
		for (OpEdge* e : edgeCurves)
			e->doSplit = e->hulls.size() ? EdgeSplit::yes : EdgeSplit::no;
		for (OpEdge* o : oppCurves)
			o->doSplit = o->hulls.size() ? EdgeSplit::yes : EdgeSplit::no;
		return addUnsectable();
	};
#endif
	for (int depth = 1; depth < maxDepth; ++depth) {
#if OP_DEBUG_IMAGE
		if (breakAtDraw && 8 <= depth) {
			redraw();  // allows setting a breakpoint to debug curve/curve
			OpDebugOut("");
		}
#endif
		for (auto opp : oppCurves) {
			opp->hulls.clear();
			opp->ccOverlaps = false;
		}
		for (auto edgePtr : edgeCurves) {
			auto& edge = *edgePtr;
			edge.ccOverlaps = false;
			edge.hulls.clear();
			bool edgeDone = (edge.start.t + OpEpsilon >= edge.end.t) ||
					edge.start.pt.isNearly(edge.end.pt);
			for (auto oppPtr : oppCurves) {
				auto& opp = *oppPtr;
				if (depth > 1 && !edge.ptBounds.intersects(opp.ptBounds))
					continue;
				bool sected;
				if (!rotatedIntersect(edge, opp, &sected))
					return SectFound::fail;
				if (!sected)
					continue;
				if (!rotatedIntersect(opp, edge, &sected))
					return SectFound::fail;
				if (!sected)
					continue;
				opp.ccOverlaps = true;
				edge.ccOverlaps = true;
				// If any intersection is found, already found or not, remove piece around 
				// both edge and opp so that remaining edges can be checked for intersection; also,
				// this approach allows handling a single intersection at a time, regardless of
				// how many times the curves actually intersect.
				// Thus, the coincident/unsectable condition is how many times edge pieces
				// were removed. The removals need to alternate ends, so that the limits of the 
				// coincident/unsectable run can be found.

				// check end condition
				if (edgeDone && (opp.start.t + OpEpsilon >= opp.end.t 
						|| opp.start.pt.isNearly(opp.end.pt))) {
					addIntersection(opp, edge);
					goto splitEm;
				} 
				if (ifExactly(edge, edge.start, opp, opp.start)
						|| ifExactly(edge, edge.end, opp, opp.start)
						|| ifExactly(edge, edge.start, opp, opp.end)
						|| ifExactly(edge, edge.end, opp, opp.end)) {
					goto splitEm;
				}
				if (ifNearly(edge, edge.start, opp, opp.start)
						|| ifNearly(edge, edge.end, opp, opp.start)
						|| ifNearly(edge, edge.start, opp, opp.end)
						|| ifNearly(edge, edge.end, opp, opp.end)) {
					goto splitEm;
				}
				if (addHullSect(edge, opp, CurveRef::edge))
					goto splitEm;
				if (addHullSect(opp, edge, CurveRef::opp))
					goto splitEm;
			}
		}
		if (edgeCurves.size() >= maxSplits || oppCurves.size() >= maxSplits) {
            OpCurveCurve cc(originalEdge, originalOpp);
            return cc.divideAndConquer();
		}
		{
			constexpr float wildAssGuess = .8f;  // if curve range hasn't reduced to less than 80%
			if (3 <= depth && rangeRatio(edgeCurves, originalEdge) > wildAssGuess &&
					rangeRatio(oppCurves, originalOpp) > wildAssGuess) {
				OpCurveCurve cc(originalEdge, originalOpp);
				return cc.divideAndConquer();
			}
		}
#if OP_DEBUG_VERBOSE
		// save state prior to split and delete
		if ((int) dvDepthIndex.size() < depth)
			dvDepthIndex.push_back((int) dvAll.size());
		for (auto edge : edgeCurves)
			dvAll.push_back(edge);
		for (auto opp : oppCurves)
			dvAll.push_back(opp);
#endif
		{
			auto overlaps = [](const std::vector<OpEdge*>& curves) {
				for (auto curve : curves)
					if (curve->ccOverlaps)
						return true;
				return false;
			};
			if (!overlaps(edgeCurves) || !overlaps(oppCurves))
				return addSect();
		}
		splitHulls(CurveRef::edge, depth);
		splitHulls(CurveRef::opp, depth);
		goto emptyCheck;
	splitEm:
		snipAndGo(edgeCurves, originalEdge->segment, snipEdge);
		snipAndGo(oppCurves, originalOpp->segment, snipOpp);
	emptyCheck:
		if (!edgeCurves.size() || !oppCurves.size())
			return addSect();
	}
	OpCurveCurve cc(originalEdge, originalOpp);
	return cc.divideAndConquer();
//	return SectFound::fail;
}
#endif

void OpCurveCurve::findEdgesTRanges(CurveRef curveRef) {
	auto& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	std::vector <OpEdge*> ordered;
	for (auto edgePtr : curves) {
		if (EdgeSplit::yes != edgePtr->doSplit)	 // edge bounds didn't overlap opposite
			continue;
		ordered.push_back(edgePtr);
	}
	std::sort(ordered.begin(), ordered.end(), [](OpEdge*& lhs, OpEdge*& rhs) {
		return lhs->start.t < rhs->start.t; 
	});
	// if curve pieces are continuous, join them up
	auto& runs = CurveRef::edge == curveRef ? edgeRuns : oppRuns;
	for (auto edge : ordered) {
		if (!runs.size() || runs.back()->end.t != edge->start.t)
			runs.push_back(edge);
		else
			runs.back()->end = edge->end;
	}
	for (auto edge : runs) {
		OP_ASSERT(WindingType::winding == edge->winding.debugType);
		edge->subDivide();
	}
}

// Edge parts are all linear. See if each intersects with any of the opposite edges
void OpCurveCurve::linearIntersect(std::vector<OpEdge*>& edgeParts,
		std::vector<OpEdge*>& oppParts) {
	for (auto edgePtr : edgeParts) {
		auto& edge = *edgePtr;
		OP_ASSERT(edge.isLine_impl);
		// !!! optimization: make edge point bounds array of two points...
		edge.setPointBounds();
		for (auto oppPtr : oppParts) {
			auto& opp = *oppPtr;
			opp.setPointBounds();
			if (!edge.ptBounds.intersects(opp.ptBounds))
				continue;
#if OP_DEBUG_RECORD
			OpDebugRecordStart(opp, edge);
#endif
			if (IntersectResult::yes == OpWinder::AddLineCurveIntersection(opp, edge)) 
				sectResult = true;		
#if OP_DEBUG_RECORD
			OpDebugRecordEnd();
#endif
		}
	}
}

void OpCurveCurve::release() {
	OpContours* contours = originalEdge->contours();
	contours->release(contours->ccStorage);
}

#if CC_EXPERIMENT
// snip out the curve 16 units to the side of the intersection point, to prevent another closeby
// intersection from also getting recorded. The units maybe t values, or may be x/y distances.
void OpCurveCurve::snipAndGo(std::vector<OpEdge*>& curves, const OpSegment* segment, 
		const OpPtT& ptT) {
	// snip distance must be large enough to differ in x/y and in t
	constexpr float tStep = 16;  // !!! just a guess 
	float cutDt = std::max(OpEpsilon, std::nextafterf(ptT.t, OpInfinity) - ptT.t) * tStep;
	OpVector cutDxy = { std::max(OpEpsilon, std::nextafterf(ptT.pt.x, OpInfinity) - ptT.pt.x),
			std::max(OpEpsilon, std::nextafterf(ptT.pt.y, OpInfinity) - ptT.pt.y) };
	float minDistanceSq = cutDxy.lengthSquared() * tStep;
	std::array<OpPtT, 2> tRange;
	for (float direction : { -1, 1 }) {
		OpPtT cut;
		do {
			cut.t = std::max(0.f, std::min(1.f, ptT.t + direction * cutDt));
			cut.pt = segment->c.ptAtT(cut.t);
		} while ((cut.pt - ptT.pt).lengthSquared() < minDistanceSq && cut.t && 1 != cut.t
				&& (direction *= tStep));
		tRange[direction < 0 ? 0 : 1] = cut;
	}
	OP_ASSERT(tRange[0].t < tRange[1].t);
	// remove part or all of edges that overlap tRange
	std::vector<OpEdge*> snips;
	OpContours* contours = segment->contour->contours;
	auto addSnip = [contours](const OpEdge* edge, const OpPtT& start, const OpPtT& end) {
		void* block = contours->allocateEdge(contours->ccStorage);
		return new(block) OpEdge(edge, start, end  OP_DEBUG_PARAMS(EDGE_MAKER(split2)));
	};
	for (OpEdge* edge : curves) {
		if (edge->start.t >= tRange[1].t || edge->end.t <= tRange[0].t) {
			snips.push_back(edge);
			continue;
		}
		if (edge->start.t < tRange[0].t)
			snips.push_back(addSnip(edge, edge->start, tRange[0]));
		if (edge->end.t > tRange[1].t)
			snips.push_back(addSnip(edge, tRange[1], edge->end));
	}
	std::swap(snips, curves);
}
#endif

bool OpCurveCurve::split(CurveRef curveRef, DoSplit doSplit) {
	auto& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	auto& lines = CurveRef::edge == curveRef ? edgeLines : oppLines;
	std::vector<OpEdge*> splits;	// curves only
	for (auto edgePtr : curves) {
		auto& edge = *edgePtr;
		OP_ASSERT(!edge.isLine_impl);
		if (EdgeSplit::no == edge.doSplit && DoSplit::marked == doSplit)
			continue;
		// constructor does all the heavy lifting (initialization)
		// !!! think about what (if anything) would be useful to record in debug breadcrumbs
		// assume most curves split into more curves
		OpContours* contours = originalEdge->contours();
		for (NewEdge newEdge : { NewEdge::isLeft, NewEdge::isRight } ) {
			void* block = contours->allocateEdge(contours->ccStorage);
			OpEdge* split = new(block) OpEdge(&edge, edge.center, newEdge  
					OP_DEBUG_PARAMS(EDGE_MAKER(split1)));
			if (split->isLinear())
				lines.push_back(split);
			else
				splits.push_back(split);
		}
	}
	curves.swap(splits);
	return curves.size() || lines.size();
}

#if CC_EXPERIMENT
// start here;
// If edge has hull points that define split but opp does not, opp may be entirely inside edge hull.
// Modify below to keep opp corresponding to kept edge split to handle this case.
// where should the opp be stored?

// if end hull index == -1, discard both sides of sect (cutout via exact or nearby sect)
// if end hull >= 0, look for sect through curve
void OpCurveCurve::splitHulls(CurveRef which, int depth) {
	std::vector<OpEdge*>& curves = CurveRef::edge == which ? edgeCurves : oppCurves;
	std::vector<OpEdge*> splits;
	OpContours* contours = originalEdge->contours();
	for (auto edgePtr : curves) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		float edgeMidT = (edgePtr->start.t + edgePtr->end.t) / 2;
		if (!edge.hulls.size()) {
			OpPtT edgeMid = { edgePtr->segment->c.ptAtT(edgeMidT), edgeMidT };
			// while edgeMid is in a deleted bounds, bump edgeMidT
			checkSplit(edgePtr->start.t, edgePtr->end.t, which, edgeMid);
			void* blockL = contours->allocateEdge(contours->ccStorage);
			OpEdge* splitLeft = new(blockL) OpEdge(&edge, edgeMid, NewEdge::isLeft  
					OP_DEBUG_PARAMS(EDGE_MAKER(splitLeft)));
			splitLeft->ccOverlaps = true;
			splits.push_back(splitLeft);
			void* blockR = contours->allocateEdge(contours->ccStorage);
			OpEdge* splitRight = new(blockR) OpEdge(&edge, edgeMid, NewEdge::isRight  
					OP_DEBUG_PARAMS(EDGE_MAKER(splitRight)));
			splitRight->ccOverlaps = true;
			splits.push_back(splitRight);
			continue;
		}
		std::vector<HullSect>& hulls = edge.hulls;
		bool hasMid = false;
		for (const HullSect hull : hulls) {
			if ((hasMid = edge.start.t < hull.sect.t && hull.sect.t < edge.end.t))
				break;
		}
		if (!hasMid && OpMath::IsFinite(edge.center.t))
			hulls.emplace_back(nullptr, edge.center, 0);
		hulls.emplace_back(nullptr, edge.start, 0);
		hulls.emplace_back(nullptr, edge.end, 0);
		std::sort(hulls.begin(), hulls.end(), [] (const HullSect& s1, const HullSect& s2) {
			return s1.sect.t < s2.sect.t;
		});
		int writer = 0;
		for (size_t xing = 1; xing < hulls.size(); ++xing) {
			if (hulls[writer].sect.t >= hulls[xing].sect.t)
				continue;
			hulls[++writer] = hulls[xing];
		}
		hulls[writer] = hulls.back();
		int hullsResized = writer + 1;
		for (int index = 0; index + 1 < hullsResized; ++index) {
			// while hull sect is in a deleted bounds, bump its t and recompute
			checkSplit(edgePtr->start.t, hulls[index + 1].sect.t, which, hulls[index].sect);
			checkSplit(hulls[index].sect.t, edgePtr->end.t, which, hulls[index + 1].sect);
			OP_ASSERT(hulls[index].sect.t < hulls[index + 1].sect.t);
			float splitMidT = (hulls[index].sect.t + hulls[index + 1].sect.t) / 2;
			void* block = contours->allocateEdge(contours->ccStorage);
			OpEdge* split = new(block) OpEdge(&edge, hulls[index].sect, hulls[index + 1].sect  
					OP_DEBUG_PARAMS(EDGE_MAKER(split2)));
			split->bias = splitMidT < edgeMidT ? SplitBias::low : SplitBias::high;
			split->ccOverlaps = true;
			splits.push_back(split);
		}
	}
	curves.swap(splits);
}
#endif

bool OpCurveCurve::tooFew(CurveRef curveRef) {
	size_t curveCount, lineCount, oppCurveCount;
	if (CurveRef::edge == curveRef) {
		curveCount = edgeCurves.size();
		lineCount = edgeLines.size();
		oppCurveCount = oppCurves.size();
	} else {
		curveCount = oppCurves.size();
		lineCount = oppLines.size();
		oppCurveCount = edgeCurves.size();
	}
	return curveCount && curveCount * 2 + lineCount <= oppCurveCount;
}
