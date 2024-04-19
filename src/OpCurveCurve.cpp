// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include <utility>

#if OP_DEBUG_DUMP
int OpCurveCurve::debugExperiment;
#endif

OpCurveCurve::OpCurveCurve(OpEdge* edge, OpEdge* opp)
	: originalEdge(edge)
	, originalOpp(opp)
	, sectResult(false)
{
	edge->contours()->reuse(edge->contours()->ccStorage);
	edgeCurves.emplace_back(edge);
	oppCurves.emplace_back(opp);
	MatchEnds match = edge->segment->matchEnds(opp->segment).match;
	smallTFound = MatchEnds::start & match;
	largeTFound = MatchEnds::end & match;
#if OP_DEBUG_DUMP
	++debugExperiment;
	debugLocal = debugExperiment;  // copied so value is visible in debugger
	edge->contours()->debugCurveCurve = this;
//		debugSaveID();
#endif
}

// trim front and back of ranges
SectFound OpCurveCurve::addUnsectable() {
	findEdgesTRanges(CurveRef::edge);
	findEdgesTRanges(CurveRef::opp);
	Axis larger = originalEdge->ptBounds.largerAxis();
	for (auto edgePtr : edgeRuns) {
		auto& edge = *edgePtr;
		for (auto oppPtr : oppRuns) {
			auto& opp = *oppPtr;
			if (opp.ptBounds.ltChoice(larger) >= edge.ptBounds.rbChoice(larger)
					|| opp.ptBounds.rbChoice(larger) <= edge.ptBounds.ltChoice(larger))
				continue;
			float minXY = std::max(edge.ptBounds.ltChoice(larger), 
					opp.ptBounds.ltChoice(larger));
			float maxXY = std::min(edge.ptBounds.rbChoice(larger), 
					opp.ptBounds.rbChoice(larger));
			OP_ASSERT(minXY < maxXY);
			if (edge.start.pt.choice(larger) > edge.end.pt.choice(larger))
				std::swap(minXY, maxXY); // make max/min agree with edge start/end
			OpPtT edgeStart = edge.findT(larger, minXY);
			OpPtT edgeEnd = edge.findT(larger, maxXY);
			OpPtT oppStart = opp.findT(larger, minXY);
			OpPtT oppEnd = opp.findT(larger, maxXY);
			OP_ASSERT(!OpMath::IsNaN(edgeStart.t));  // !!! troubling: tripped by fuzz763_9 once...
			OP_ASSERT(!OpMath::IsNaN(edgeEnd.t));
			OP_ASSERT(!OpMath::IsNaN(oppStart.t));
			OP_ASSERT(!OpMath::IsNaN(oppEnd.t));
			if (edgeStart.pt != oppStart.pt) {
				if (!edgeStart.pt.isFinite())
					edgeStart.pt = oppStart.pt;
				else if (!oppStart.pt.isFinite())
					oppStart.pt = edgeStart.pt;
				else
					OpPtT::MeetInTheMiddle(edgeStart, oppStart);
			}
			if (edgeEnd.pt != oppEnd.pt) {
				if (!edgeEnd.pt.isFinite())
					edgeEnd.pt = oppEnd.pt;
				else if (!oppEnd.pt.isFinite())
					oppEnd.pt = edgeEnd.pt;
				else
					OpPtT::MeetInTheMiddle(edgeEnd, oppEnd);
			}
			bool flipped = oppStart.t > oppEnd.t;
			if (flipped)
				std::swap(oppStart, oppEnd); // flip to fix opp t (now opp pts are flipped vs. edge)
			OpSegment* segment = const_cast<OpSegment*>(edge.segment);
			int usectID = segment->nextID();
			OpIntersection* segSect1 = segment->addUnsectable(edgeStart, usectID, MatchEnds::start, 
					opp.segment  OP_LINE_FILE_PARAMS(SectReason::unsectableStart));
			OpIntersection* segSect2 = segment->addUnsectable(edgeEnd, usectID, MatchEnds::end, 
					opp.segment  OP_LINE_FILE_PARAMS(SectReason::unsectableEnd));
			OpSegment* oppSegment = const_cast<OpSegment*>(opp.segment);
			OpIntersection* oppSect1 = oppSegment->addUnsectable(oppStart, 
					flipped ? -usectID : usectID, 
					flipped ? MatchEnds::end : MatchEnds::start, segment  
					OP_LINE_FILE_PARAMS(
				    flipped ? SectReason::unsectableOppEnd : SectReason::unsectableOppStart));
			OpIntersection* oppSect2 = oppSegment->addUnsectable(oppEnd, 
					flipped ? -usectID : usectID, 
					flipped ? MatchEnds::start : MatchEnds::end, segment  
					OP_LINE_FILE_PARAMS(
				    flipped ? SectReason::unsectableOppStart : SectReason::unsectableOppEnd));
			segSect1->pair(flipped ? oppSect2 : oppSect1);
			segSect2->pair(flipped ? oppSect1 : oppSect2);
			OpDebugOut("");
		}
	}

	return SectFound::intersects;
}

struct TGap {
	TGap(const OpPtT& l, const OpPtT& h)
		: lo(l)
		, hi(h) {
	}

	OpPtT lo;
	OpPtT hi;
};

// if after breaking runs spacially on both edge and opp into two runs
// and one run is connected to already found intersections, remove that run
void OpCurveCurve::checkForGaps() {
	auto findGaps = [](std::vector<OpEdge*> curves) {
		std::vector<TGap> gaps;
		OpPtT last = curves[0]->start;
		for (auto edgePtr : curves) {
			OP_ASSERT(edgePtr->start.t >= last.t);  // if not sorted, fix
			if (edgePtr->start.t != last.t)
				gaps.emplace_back(last, edgePtr->start);
			last = edgePtr->end;
		}
		return gaps;
	};
	std::vector<TGap> edgeGaps = findGaps(edgeCurves);
	if (!edgeGaps.size())
		return;
	std::vector<TGap> oppGaps = findGaps(oppCurves);
	if (!oppGaps.size())
		return;
	MatchReverse m = originalEdge->segment->matchEnds(originalOpp->segment);
	auto markToDelete = [](std::vector<OpEdge*> curves, float loT, float hiT) {
		for (auto edgePtr : curves) {
			if (edgePtr->start.t >= loT && hiT >= edgePtr->end.t)
				edgePtr->ccOverlaps = false;
		}
	};
	if (smallTFound) {
		OpPointBounds eGapBounds { edgeGaps[0].lo.pt, edgeGaps[0].hi.pt };
		size_t oIndex = m.reversed ? oppGaps.size() - 1 : 0;
		OpPointBounds oGapBounds { oppGaps[oIndex].lo.pt, oppGaps[oIndex].hi.pt };
		if (eGapBounds.overlaps(oGapBounds))
			markToDelete(edgeCurves, 0, edgeGaps[0].lo.t);
		if (m.reversed)
			markToDelete(oppCurves, oppGaps[oIndex].hi.t, 1);
		else
			markToDelete(oppCurves, 0, oppGaps[0].lo.t);
	}
	if (largeTFound) {
		OpPointBounds eGapBounds { edgeGaps.back().lo.pt, edgeGaps.back().hi.pt };
		size_t oIndex = m.reversed ? 0 : oppGaps.size() - 1;
		OpPointBounds oGapBounds { oppGaps[oIndex].lo.pt, oppGaps[oIndex].hi.pt };
		if (eGapBounds.overlaps(oGapBounds))
			markToDelete(edgeCurves, edgeGaps.back().hi.t, 1);
		if (m.reversed)
			markToDelete(oppCurves, 0, oppGaps[0].lo.t);
		else
			markToDelete(oppCurves, oppGaps[oIndex].hi.t, 1);
	}
}

#if CC_EXPERIMENT
// Scan through opposite curves and see if check point is inside deleted bounds. If so, use a
// different (but close by if possible) point to split the curve.
bool OpCurveCurve::checkSplit(float loT, float hiT, CurveRef which, OpPtT& checkPtT) const {
	OP_ASSERT(loT <= checkPtT.t && checkPtT.t <= hiT);
	const std::vector<OpEdge*>& oCurves = CurveRef::edge == which ? oppCurves : edgeCurves;
	const OpEdge* edge = CurveRef::edge == which ? originalEdge : originalOpp;
	const OpCurve& eCurve = edge->segment->c;
	float startingT = checkPtT.t;
	float deltaT = OpEpsilon;
	int attempts = 0;
	OpPtT original = checkPtT;
	const int maxAttempts = 16;  // !!! arbitrary guess
	do {
		// check edge cases (e.g., checkPtT.pt == oCurve end) first
		for (OpEdge* oCurve : oCurves) {
			if (oCurve->ccOverlaps && oCurve->ptBounds.contains(checkPtT.pt))
				return original != checkPtT;
		}
		// check for gap between original and edge list, and between edges in edge list
		const OpEdge* opp = CurveRef::edge == which ? originalOpp : originalEdge;
		OpPtT delLo = opp->start;
		auto checkBounds = [checkPtT](const OpPtT& lo, const OpPtT& hi) {
			OpPointBounds delBounds(lo.pt, hi.pt);
			return delBounds.contains(checkPtT.pt);
		};
		for (OpEdge* oCurve : oCurves) {
			OpPtT delHi = oCurve->start;
			OP_ASSERT(delLo.t <= delHi.t);
			if (delLo.t < delHi.t && checkBounds(delLo, delHi))  // there's a gap between edges
				goto tryAgain;
			delLo = oCurve->end;
			if (!oCurve->ccOverlaps && checkBounds(delHi, delLo))  // check edge without overlap
				goto tryAgain;
		}
		if (delLo.t == opp->end.t || !checkBounds(delLo, opp->end))  // check last list / original
			return original != checkPtT;
	tryAgain:
		checkPtT.t = startingT + deltaT;
		if (loT > checkPtT.t || checkPtT.t > hiT) {
			checkPtT.t = startingT - deltaT;
			if (loT > checkPtT.t || checkPtT.t > hiT) {
				deltaT /= -2;
				continue;
			}
		}
		checkPtT.pt = eCurve.ptAtT(checkPtT.t);
		deltaT *= -2;
	} while (++attempts < maxAttempts);
//	OP_ASSERT(0);  // decide what to do when assert fires
	checkPtT = original;
	return false;
}
#endif

// this marks opp as splittable if edge is a curve
// this finds opp t if edge is a line
// runs twice: edge/opp, then opp/edge
// result for all edges: no (intersections at all); fail; (something) split; line sect (no splits)
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
			OpCurve oppRotated = opp.curve.toVertical(edgeLine);
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
	bool breakAtDraw = 24 == originalEdge->id && 34 == originalOpp->id;
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
	for (depth = 1; depth < maxDepth; ++depth) {
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
			return sectResult? SectFound::intersects : SectFound::no;
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
			if (depth >= 8) {
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
	int oneOr = 2;
	int theOther = 7;
	bool breakAtDraw = (oneOr == originalEdge->segment->id && theOther == originalOpp->segment->id)
			|| (oneOr == originalOpp->segment->id && theOther == originalEdge->segment->id);
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
		OpCurve edgeRotated = edge.curve.toVertical(edgePts);
		if (!edgeRotated.isFinite())
			return false;
		OpCurve oppRotated = opp.curve.toVertical(edgePts);
		if (!oppRotated.isFinite())
			return false;
		OpPointBounds eRotBounds;
		eRotBounds.set(edgeRotated);
		OpPointBounds oRotBounds;
		oRotBounds.set(oppRotated);
		*sectedPtr = eRotBounds.outsetClose().intersects(oRotBounds.outsetClose());
		return true;
	};
	auto addSect = [this]() {
		if (!ccSects.size())
			return SectFound::no;
		for (CcSects& s : ccSects) {
			OpSegment* eSegment = const_cast<OpSegment*>(s.e.edge->segment);
			OpSegment* oSegment = const_cast<OpSegment*>(s.o.edge->segment);
			OpIntersection* sect = eSegment->addEdgeSect(s.e.ptT  
					OP_LINE_FILE_PARAMS(SectReason::lineCurve, s.e.edge, s.o.edge));
			OpIntersection* oSect = oSegment->addEdgeSect(s.o.ptT  
					OP_LINE_FILE_PARAMS(SectReason::lineCurve, s.e.edge, s.o.edge));
			sect->pair(oSect);
		}
		return SectFound::intersects;
	};
	auto ifExactly = [this](OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT) {
		if (edgePtT.pt != oppPtT.pt)
			return false;
		recordSect(edge, edgePtT, opp, oppPtT 
				OP_LINE_FILE_PARAMS(SectReason::edgeCCExact, SectReason::oppCCExact));
		return true;
	};
	auto ifNearly = [this](OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT) {
		if (!edgePtT.pt.isNearly(oppPtT.pt))
			return false;
		recordSect(edge, edgePtT, opp, oppPtT  
				OP_LINE_FILE_PARAMS(SectReason::edgeCCNearly,  SectReason::oppCCNearly));
		return true;
	};
	auto addIntersection = [this](OpEdge& edge, OpEdge& opp) {
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
				oppPtT.t = opp.segment->findValidT(0, 1, edgePtT.pt);
				oppPtT.t = std::max(oppPtT.t, opp.start.t);
				oppPtT.t = std::min(oppPtT.t, opp.end.t);
				if (OpMath::IsNaN(oppPtT.t))
					oppPtT.t = opp.start.t;
			}
		}
		recordSect(edge, edgePtT, opp, oppPtT  
				OP_LINE_FILE_PARAMS(SectReason::edgeCurveCurve, SectReason::oppCurveCurve));
	};
#if 0
	auto addHullSect = [hullSect, recordSect](OpEdge& edge, OpEdge& opp, CurveRef curveRef) {
		hullSect(opp, edge);
		bool found = false;
		for (const HullSect& hull : edge.hulls) {
			if (SectType::endHull != hull.type)
				continue;
			const OpPtT& edgePtT = hull.sect;
			const OpPtT& oppPtT = hull.oSect;
			if (CurveRef::edge == curveRef)
				recordSect(edge, edgePtT, opp, oppPtT  OP_LINE_FILE_PARAMS(
						SectReason::edgeCCHull, SectReason::oppCCHull));
			else
				recordSect(opp, oppPtT, edge, edgePtT  OP_LINE_FILE_PARAMS(
						SectReason::oppCCHull, SectReason::edgeCCHull));
			found = true;
		}
		return found;
	};
	auto overlapRange = [](const std::vector<OpEdge*>& curves) {
		OpEdge* const * first = &curves.front();
		OpEdge* const * last = &curves.back();
		while (first < last && !(*first)->ccOverlaps)
			++first;
		while (first < last && !(*last)->ccOverlaps)
			--last;
		return (*last)->end.t - (*first)->start.t;
	};
	auto hackOfAddUnsectable = [this]() {
		for (OpEdge* e : edgeCurves)
			e->doSplit = e->hulls.size() ? EdgeSplit::yes : EdgeSplit::no;
		for (OpEdge* o : oppCurves)
			o->doSplit = o->hulls.size() ? EdgeSplit::yes : EdgeSplit::no;
		return addUnsectable();
	};
#endif
	// returns true if edge's line intersects opposite edge's curve
	// don't check against opposite segment's curve, since it may be offset
	// If edge is linear, but not a true line (e.g., the control points are nearly colinear with
	//  the end points) check the hulls rather than just the line. 
	// !!! share with hull code? how to check if one line is inside other line's hulls?
	auto lineIntersect = [](OpEdge& edge, OpEdge& opp) {
		LinePts edgePts;
		edgePts.pts = { edge.start.pt, edge.end.pt };
		OpRootPts septs = opp.curve.lineIntersect(edgePts);
		return septs.count;
	};
//	int splitCount = 0;
	// !!! start here;
	// looks like recursion is getting stuck trimming tiny bits from end instead of dividing in the
	// middle and/or detecting that it ought to look to see if the run is largely unsectable. It
	// should be trying in more than one place to find intersections somehow
	for (depth = 1; depth < maxDepth; ++depth) {
#if OP_DEBUG_IMAGE
		if (breakAtDraw && 4 <= depth) {
			redraw();  // allows setting a breakpoint to debug curve/curve
			OpDebugOut("debugLocal:" + STR(debugLocal) + "\n");
			dmpDepth();
			OpDebugOut("");
		}
#endif
		for (auto opp : oppCurves) {
			opp->hulls.clear();
			opp->ccOverlaps = false;
		}
		for (auto edgePtr : edgeCurves) {
			auto& edge = *edgePtr;
			edge.hulls.clear();
			edge.ccOverlaps = false;
			bool edgeDone = edge.start.isNearly(edge.end);
			for (auto oppPtr : oppCurves) {
				auto& opp = *oppPtr;
				if (depth > 1 && !edge.closeBounds().intersects(opp.closeBounds()))
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
				if (edge.isLinear() && edge.exactLine && !lineIntersect(edge, opp))
					goto tryClose;
				if (opp.isLinear() && opp.exactLine && !lineIntersect(opp, edge))
					goto tryClose;
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
				if (edgeDone && (opp.start.isNearly(opp.end))) {
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
				SetHullSects(edge, opp);
				SetHullSects(opp, edge);
			tryClose:
				if (edge.isClose() && opp.isClose()) {
					// !!! save man in the middle?
					closeEdge.save(edge, edge.start, opp, opp.start);
				}
				for (const OpPtT& ePtT : { edge.start, edge.end } )
					for (const OpPtT& oPtT : { opp.start, opp.end } )
						ifCloseSave(edge, ePtT, opp, oPtT);
			}
		}
		// if there is more than one crossover, look for unsectable
		if (ccSects.size() > (smallTFound || largeTFound ? 0 : 1)) { 
			findUnsectable();
			goto emptyCheck;
		}
		if ((smallTFound || largeTFound) && edgeCurves.size() > 2 && oppCurves.size() > 2)
			checkForGaps();
#if 0
		if (edgeCurves.size() >= maxSplits || oppCurves.size() >= maxSplits) {
            OpCurveCurve cc((OpEdge*) originalEdge, (OpEdge*) originalOpp);
            return cc.divideAndConquer();
		}
		{
			constexpr float wildAssGuess = .8f;  // if curve range hasn't reduced to less than 80%
			if (3 <= depth - splitCount - ccSects.size() && overlapRange(edgeCurves) > wildAssGuess
					&& overlapRange(oppCurves) > wildAssGuess) {
				// !!! instead of returning sects already found
				// look for another crossing on the other ends
				if (ccSects.size())
					return addSect();
				OpCurveCurve cc((OpEdge*) originalEdge, (OpEdge*) originalOpp);
				return cc.divideAndConquer();
			}
		}
#endif
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
		splitHulls(CurveRef::edge) || splitHulls(CurveRef::opp);  // only split one if effective
		goto emptyCheck;
	splitEm:
		{
//			++splitCount;  // since it does little, don't penalize overlap range
			OpContours* contours = originalEdge->contours();
			snipAndGo(edgeCurves, originalEdge->segment, contours, snipEdge);
			snipAndGo(oppCurves, originalOpp->segment, contours, snipOpp);
		}
	emptyCheck:
		if (!edgeCurves.size() || !oppCurves.size())
			return addSect();
	}
	OP_ASSERT(0);  // !!! see when this occurs so we can remove divide and conquer
	OpCurveCurve cc((OpEdge*) originalEdge, (OpEdge*) originalOpp);
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

void OpCurveCurve::SetHullSects(OpEdge& edge, OpEdge& opp) {
	int ptCount = edge.curve.pointCount();
	LinePts edgePts;
	edgePts.pts[1] = edge.curve.pts[0];
	for (int index = 1; index <= ptCount; ++index) {
		edgePts.pts[0] = edgePts.pts[1];
		int endHull = index < ptCount ? index : 0;
		edgePts.pts[1] = edge.curve.pts[endHull];
		if (edgePts.pts[0].isNearly(edgePts.pts[1]))
			continue;
		// since curve/curve intersection works by keeping overlapping edge bounds, it should
		// use edge, not segment, to find hull intersections
		OpRootPts septs = opp.curve.lineIntersect(edgePts);
		for (size_t inner = 0; inner < septs.count; ++inner) {
			OpPtT sectPtT = septs.ptTs[inner];
			// set to secttype endhull iff computed point is equal to or nearly an end point
			SectType sectType;
			if ((1 == index || 0 == endHull) && sectPtT.pt.isNearly(edgePts.pts[0])) {
				sectType = SectType::endHull;
				sectPtT.pt = edgePts.pts[0];
			} else if ((ptCount - 1 == index || 0 == endHull) 
					&& sectPtT.pt.isNearly(edgePts.pts[1])) {
				sectType = SectType::endHull;
				sectPtT.pt = edgePts.pts[1];
			} else
				sectType = endHull ? SectType::controlHull : SectType::midHull;
			sectPtT.t = OpMath::Interp(opp.start.t, opp.end.t, sectPtT.t);
			OP_ASSERT(opp.start.t <= sectPtT.t && sectPtT.t <= opp.end.t);
			opp.hulls.add(sectPtT, sectType, &edge);
		}
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
OpPtT OpCurveCurve::Cut(const OpPtT& ptT, const OpSegment* segment, float direction) {
	constexpr float tStep = 16;  // !!! just a guess 
	constexpr float cutDt = OpEpsilon * tStep;
	OpVector cutDxy = { OpMath::NextLarger(ptT.pt.x) - ptT.pt.x,
			OpMath::NextLarger(ptT.pt.y) - ptT.pt.y };
	float minDistanceSq = cutDxy.lengthSquared() * tStep;
	OpPtT cut;
	do {
		cut.t = OpMath::PinSorted(0, ptT.t + direction * cutDt, 1);
		cut.pt = segment->c.ptAtT(cut.t);
	} while ((cut.pt - ptT.pt).lengthSquared() < minDistanceSq 
			&& 0 < cut.t && cut.t < 1 && (direction *= tStep));
	return cut;
}

CutRangeT OpCurveCurve::CutRange(const OpPtT& ptT, const OpSegment* segment,
		float loEnd, float hiEnd) {
	constexpr float tStep = 16;  // !!! just a guess 
	constexpr float cutDt = OpEpsilon * tStep;
	OpVector cutDxy = { OpMath::NextLarger(ptT.pt.x) - ptT.pt.x,
			OpMath::NextLarger(ptT.pt.y) - ptT.pt.y };
	float minDistanceSq = cutDxy.lengthSquared() * tStep;
	CutRangeT tRange;
	for (float direction : { -1, 1 }) {
		OpPtT cut;
		float dir = direction;
		do {
			cut.t = std::max(loEnd, std::min(hiEnd, ptT.t + dir * cutDt));
			cut.pt = segment->c.ptAtT(cut.t);
		} while ((cut.pt - ptT.pt).lengthSquared() < minDistanceSq 
				 && loEnd < cut.t && cut.t < hiEnd && (dir *= tStep));
		(-1 == direction ? tRange.lo : tRange.hi) = cut;
	}
	return tRange;
}

// !!! just fooling around; fix debug params if serious
// !!! not sure how to adapt this to store two (or more?) different close points
//     maybe save the least and greatest t values?
//     then, could test mid value to see if it is also on the curve or diverges further away...
// !!! wait until test case shows up
void CcClose::save(OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT) {
	if (!lo.e.edge || lo.e.edge->start.t > edge.start.t)
		lo = CcSects(&edge, edgePtT, &opp, oppPtT  
			OP_LINE_FILE_PARAMS(SectReason::edgeCCHullPair, SectReason::oppCCHullPair));
	if (!hi.e.edge || hi.e.edge->start.t < edge.end.t)
		hi = CcSects(&edge, edgePtT, &opp, oppPtT  
			OP_LINE_FILE_PARAMS(SectReason::edgeCCHullPair, SectReason::oppCCHullPair));
	float dist = (edgePtT.pt - oppPtT.pt).lengthSquared();
	if (bestDist > dist) {
		bestDist = dist;
		best = CcSects(&edge, edgePtT, &opp, oppPtT  
				OP_LINE_FILE_PARAMS(SectReason::edgeCCHullPair, SectReason::oppCCHullPair));
	}
}

void OpCurveCurve::ifCloseSave(OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT) {
	if (!edgePtT.pt.soClose(oppPtT.pt))
		return;
	if (edgePtT.onEnd())
		return;
	if (oppPtT.onEnd())
		return;
	closeEdge.save(edge, edgePtT, opp, oppPtT);
}

void OpCurveCurve::recordSect(OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT
			OP_LINE_FILE_DEF(SectReason eReason, SectReason oReason)) {
#if 0
	// I doubt this the right appproach. Once we have a test case, flesh this out
	if (!smallTFound && edgePtT.t < originalEdge->center.t)
		smallTFound = true;
#endif
	snipEdge = edgePtT;
	snipOpp = oppPtT;
	OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
	OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
	// end points matching has already been recorded, so don't do it again
	if (edgePtT.onEnd() && oppPtT.onEnd())
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
	if (ccSects.end() != std::find_if(ccSects.begin(), ccSects.end(), 
			[edgePtT, oppPtT](auto ccSect) {
			return ccSect.e.ptT.isNearly(edgePtT) || ccSect.o.ptT.isNearly(oppPtT);
			} ))
		return;
	ccSects.emplace_back(&edge, edgePtT, &opp, oppPtT  OP_LINE_FILE_PARAMS(eReason, oReason));
}

void OpCurveCurve::snipOne(std::vector<OpEdge*>& curves, const OpSegment* segment, 
		OpContours* contours, const OpPtT& loPtT, const OpPtT& hiPtT) {
	OpPtT loCutPt = Cut(loPtT, segment, -1);
	OpPtT hiCutPt = Cut(hiPtT, segment, 1);
	std::vector<OpEdge*> snips;
	auto addSnip = [contours](const OpEdge* edge, const OpPtT& start, const OpPtT& end) {
		void* block = contours->allocateEdge(contours->ccStorage);
		OpEdge* newE = new(block) OpEdge(edge, start, end  OP_DEBUG_PARAMS(EDGE_MAKER(snip)));
		newE->ccOverlaps = true;
		return newE;
	};
	for (OpEdge* edge : curves) {
		if (edge->start.t < loCutPt.t && edge->end.t > loCutPt.t)
			snips.push_back(addSnip(edge, edge->start, loCutPt));
		if (edge->end.t > hiCutPt.t && edge->start.t < hiCutPt.t)
			snips.push_back(addSnip(edge, hiCutPt, edge->end));
		if (edge->start.t >= hiCutPt.t || edge->end.t <= loCutPt.t)
			snips.push_back(edge);
	}
	std::swap(snips, curves);
}

// snip out the curve 16 units to the side of the intersection point, to prevent another closeby
// intersection from also getting recorded. The units maybe t values, or may be x/y distances.
void OpCurveCurve::snipAndGo(std::vector<OpEdge*>& curves, const OpSegment* segment, 
		OpContours* contours, const OpPtT& ptT) {
	// snip distance must be large enough to differ in x/y and in t
	CutRangeT tRange = CutRange(ptT, segment, 0, 1);
	OP_ASSERT(tRange.lo.t < tRange.hi.t);
	// remove part or all of edges that overlap tRange
	std::vector<OpEdge*> snips;
	auto addSnip = [contours](const OpEdge* edge, const OpPtT& start, const OpPtT& end) {
		void* block = contours->allocateEdge(contours->ccStorage);
		OpEdge* newE = new(block) OpEdge(edge, start, end  OP_DEBUG_PARAMS(EDGE_MAKER(snip)));
		newE->ccOverlaps = true;
		return newE;
	};
	for (OpEdge* edge : curves) {
		if (edge->start.t >= tRange.hi.t || edge->end.t <= tRange.lo.t) {
			snips.push_back(edge);
			continue;
		}
		if (edge->start.t < tRange.lo.t)
			snips.push_back(addSnip(edge, edge->start, tRange.lo));
		if (edge->end.t > tRange.hi.t)
			snips.push_back(addSnip(edge, tRange.hi, edge->end));
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
			OpPtT center = { edge.segment->c.ptAtT(edge.center.t), edge.center.t };
			OpEdge* split = new(block) OpEdge(&edge, center, newEdge  
					OP_DEBUG_PARAMS(EDGE_MAKER(split)));
			if (split->isLinear())
				lines.push_back(split);
			else
				splits.push_back(split);
		}
	}
	curves.swap(splits);
	return curves.size() || lines.size();
}

void OpCurveCurve::splitDownTheMiddle(OpContours* contours, OpEdge& edge, const OpPtT& edgeMid, 
		std::vector<OpEdge*>* splits) {
	// !!!? while edgeMid is in a deleted bounds, bump edgeMidT
	//      (this isn't necessarily in the intersection hull)
	//      wait until this is necessary to make it work
	// checkSplit(edge.start.t, edge.end.t, which, edgeMid);
	void* blockL = contours->allocateEdge(contours->ccStorage);
	OpEdge* splitLeft = new(blockL) OpEdge(&edge, edgeMid, NewEdge::isLeft  
			OP_DEBUG_PARAMS(EDGE_MAKER(splitLeft)));
	splitLeft->ccOverlaps = true;
	splits->push_back(splitLeft);
	void* blockR = contours->allocateEdge(contours->ccStorage);
	OpEdge* splitRight = new(blockR) OpEdge(&edge, edgeMid, NewEdge::isRight  
			OP_DEBUG_PARAMS(EDGE_MAKER(splitRight)));
	splitRight->ccOverlaps = true;
	splits->push_back(splitRight);
}

#if CC_EXPERIMENT
// If edge has hull points that define split but opp does not, opp may be entirely inside edge hull.
// Modify below to keep opp corresponding to kept edge split to handle this case.
// where should the opp be stored?

// if end hull index == -1, discard both sides of sect (cutout via exact or nearby sect)
// if end hull >= 0, look for sect through curve
bool OpCurveCurve::splitHulls(CurveRef which) {
	std::vector<OpEdge*>& curves = CurveRef::edge == which ? edgeCurves : oppCurves;
	std::vector<OpEdge*>& oCurves = CurveRef::edge == which ? oppCurves : edgeCurves;
	std::vector<OpEdge*> splits;
	OpContours* contours = originalEdge->contours();
	for (auto edgePtr : curves) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		float edgeMidT = OpMath::Average(edge.start.t, edge.end.t);
		OpHulls& hulls = edge.hulls;
		auto closest = [oCurves](OpPoint pt) -> OpPtT {
			OpPoint oBest;
			float oBestDistSq = OpInfinity;
			auto bestPt = [&oBest, &oBestDistSq, pt](OpPoint testPt) {
				float distSq = (testPt - pt).lengthSquared();
				if (oBestDistSq > distSq) {
					oBestDistSq = distSq;
					oBest = testPt;
				}
			};
			// scan opposite curves for closest point
			OpPoint oLast;
			for (OpEdge* opp : oCurves) {
				OpPoint startPt = opp->start.pt;
				if (oLast != startPt)
					bestPt(startPt);
				OpPoint endPt = opp->end.pt;
				bestPt(endPt);
				oLast = endPt;
			}
			return { oBest, oBestDistSq };
		};
		auto ptTCloseTo = [edge](OpPtT oPtPair, const OpPtT& ptT, 
				OpPtT* result) {
			OpVector unitTan = edge.segment->c.tangent(ptT.t);
			OpVector tan = unitTan.setLength(sqrtf(oPtPair.t));
			if (1 == ptT.t)
				tan = -tan;
			OpPoint testPt = ptT.pt + tan;
			if (!edge.ptBounds.contains(testPt))
				return;
			// use unit tan to pass correct axis to find pt t
			Axis axis = fabsf(unitTan.dx) > fabsf(unitTan.dy) ? Axis::vertical : Axis::horizontal; 
			float resultT = edge.segment->findAxisT(axis, edge.start.t, edge.end.t, 
					testPt.choice(axis));
			if (!OpMath::IsNaN(resultT))
				*result = OpPtT(edge.segment->c.ptAtT(resultT), resultT);
		};
		// use the smaller distance to an end of oStart, oEnd, edgeMid
		auto splitPt = [closest, ptTCloseTo, edge](float oMidDist, OpPtT* result) {
			OpPtT oStart = closest(edge.start.pt);
			OpPtT oEnd = closest(edge.end.pt);
			// choose split point near edge end that is closest to opp end
			if (oMidDist <= std::min(oStart.t, oEnd.t))
				return;
			if (oStart.t > oEnd.t)
				ptTCloseTo(oEnd, edge.end, result);
			else
				ptTCloseTo(oStart, edge.start, result);
		};
		if (!hulls.h.size()) {
			// if the distance between the end points is small, choose a split point nearby
			OpPtT edgeMid = { edge.segment->c.ptAtT(edgeMidT), edgeMidT };
			OpPtT oMid = closest(edgeMid.pt);
			OpPtT closestPtT = edgeMid;
			splitPt(oMid.t, &closestPtT);
			if (closestPtT == edge.start || closestPtT == edge.end)
				closestPtT = edgeMid;
			splitDownTheMiddle(contours, edge, edgeMid, &splits);
			continue;
		}
		if (2 <= hulls.h.size()) {  // see if hulls are close enough to define an intersection
			hulls.sort(smallTFound);
			for (size_t index = 1; index < hulls.h.size(); ++index) {
				if (!hulls.sectCandidates(index, edge))
					continue;
				const OpEdge* oEdge = hulls.h[index - 1].opp;
				OP_ASSERT(oEdge);
				OpPtT oPtT;
				OpPtT hull1Sect = hulls.h[index - 1].sect;
				oPtT.t = oEdge->segment->findValidT(0, 1, hull1Sect.pt);
				if (OpMath::IsNaN(oPtT.t))
					return false;
				oPtT.pt = oEdge->segment->c.ptAtT(oPtT.t);
				if (!hulls.closeEnough(index - 1, edge, oEdge, &oPtT, &hull1Sect))
					continue;
				recordSect(edge, hull1Sect, const_cast<OpEdge&>(*oEdge), oPtT  
						OP_LINE_FILE_PARAMS(SectReason::edgeCCHullPair, 
						SectReason::oppCCHullPair));
				snipAndGo(curves, edge.segment, contours, hull1Sect);
				snipAndGo(oCurves, oEdge->segment, contours, oPtT);
				return true;
			}
		}
		int midCount = 0;
		for (const HullSect hull : hulls.h) {
			if (edge.start.t < hull.sect.t && hull.sect.t < edge.end.t)
				++midCount;
		}
		if (!midCount) {
			OpPtT centerPtT = edge.center;
			OpPtT oCtr = closest(centerPtT.pt);
			splitPt(oCtr.t, &centerPtT);
			if (OpMath::IsFinite(centerPtT.t))
				hulls.add(centerPtT, SectType::center);
		}
		hulls.add(edge.start, SectType::endHull);
		hulls.add(edge.end, SectType::endHull);
		hulls.nudgeDeleted(edge, *this, which);
		size_t splitCount = splits.size();
		for (size_t index = 0; index + 1 < hulls.h.size(); ++index) {
#if 0  // verify that this is the right approach with a test case
			if (smallTFound && 0 == index) {
				float tSpan = hulls.h[index + 1].sect.t - hulls.h[index].sect.t;
				if (tSpan < OpEpsilon * 64)  // !!! experiment in hackery -- number is wild guess
					continue;
			}
#endif
			void* block = contours->allocateEdge(contours->ccStorage);
			OpEdge* split = new(block) OpEdge(&edge, hulls.h[index].sect.t, 
					hulls.h[index + 1].sect.t  OP_DEBUG_PARAMS(EDGE_MAKER(hull)));
#if OP_DEBUG
			split->debugSplitStart = hulls.h[index].type;
			split->debugSplitEnd = hulls.h[index + 1].type;
#endif
			float splitMidT = OpMath::Average(hulls.h[index].sect.t, hulls.h[index + 1].sect.t);
			split->bias = splitMidT < edgeMidT ? SplitBias::low : SplitBias::high;
			split->ccOverlaps = true;
			splits.push_back(split);
		}
		if (splits.size() - splitCount < 2) {
			splits.clear(); // this may leave an edge was never used ... not the end of the world
			splitDownTheMiddle(contours, edge, edge.center, &splits);  // divide edge in two			
		}
	}
	curves.swap(splits);
	return false;
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

// !!! to do : look for 2nd best point if curves come close to intersecting twice
void OpCurveCurve::closest() {
    if (!OpMath::IsFinite(closeEdge.bestDist))
		return;
    // require either pt or t to be within some metric of closeness
    // since this specifies intersection points, some sloppiness is OK
    // as long as the point is (nearly) on each segment
    OpPtT sPtT = closeEdge.best.e.ptT;
	OpSegment* s = const_cast<OpSegment*>(closeEdge.best.e.edge->segment);
	OpSegment* o = const_cast<OpSegment*>(closeEdge.best.o.edge->segment);
	OpIntersection* sect = s->addSegSect(sPtT, o  OP_LINE_FILE_PARAMS(SectReason::soClose));
    OpPtT oPtT = OpPtT(sPtT.pt, closeEdge.best.o.ptT.t);
	OpIntersection* oSect = o->addSegSect(oPtT, s  OP_LINE_FILE_PARAMS(SectReason::soClose));
	if (sect && oSect) 
        sect->pair(oSect);
}

// !!! newer rewrite, remove original and '2' on this one
void OpCurveCurve::addUnsectable2(OpSegment* seg, const OpPtT& edgeStart, const OpPtT& edgeEnd,
		OpSegment* opp, OpPtT oppStart, OpPtT oppEnd) {  
	int usectID = seg->nextID();
	OpIntersection* segSect1 = seg->addUnsectable(edgeStart, usectID, MatchEnds::start, opp
			OP_LINE_FILE_PARAMS(SectReason::unsectable));
	OpIntersection* segSect2 = seg->addUnsectable(edgeEnd, usectID, MatchEnds::end, opp
			OP_LINE_FILE_PARAMS(SectReason::unsectable));
	bool flipped = oppStart.t > oppEnd.t;
	MatchEnds oppMatch = MatchEnds::start;
	if (flipped) {
		std::swap(oppStart, oppEnd); // flip to fix opp t (now opp pts are flipped vs. edge)
		usectID = -usectID;
		oppMatch = MatchEnds::end;
	}
	OpIntersection* oppSect1 = opp->addUnsectable(oppStart, usectID, oppMatch, seg
			OP_LINE_FILE_PARAMS(SectReason::unsectable));
	OpIntersection* oppSect2 = opp->addUnsectable(oppEnd, usectID, !oppMatch, seg
			OP_LINE_FILE_PARAMS(SectReason::unsectable));
	segSect1->pair(flipped ? oppSect2 : oppSect1);
	segSect2->pair(flipped ? oppSect1 : oppSect2);
}

void FoundPtTs::setEnd(const OpSegment* o, const OpCurve& curve, float t) {
	seg = { t ? curve.lastPt() : curve.pts[0], t };
	opp = { o->c.ptTAtT(o->findValidT(0, 1, seg.pt)) };
}

// (assuming its already been determined that the edge pair is likely partially unsectable
// binary search pair of curves
// look for range of unsectables and snip remaining more aggressively
// if one end t is already found, add it in to calculus of finding range of unsectable
// in any case, check the found range to see if the midpoint is also plausibly unsectable
// before treating the entire range as unsectable
// either: generate one unsectable range, or two points
// (unsectable range + one point or two unsectable ranges may be better, but wait for test case)
void OpCurveCurve::findUnsectable() {
	OpSegment* seg = const_cast<OpSegment*>(originalEdge->segment);
	OpSegment* opp = const_cast<OpSegment*>(originalOpp->segment);
	// start with existing found point
	OP_ASSERT(ccSects.size());
	constexpr float testFloorSq = OpEpsilon * OpEpsilon;
	auto binarySearch = [seg, opp, testFloorSq](const OpPtT& basePtT, float endSegT) {
		OpPtT baseOpp { opp->c.ptTAtT(opp->findValidT(0, 1, basePtT.pt)) };
		float distSq = (baseOpp.pt - basePtT.pt).lengthSquared();
		float baseDistSq = std::max(distSq, testFloorSq) * 64;  // !!! arbitrary guess
		bool searchDown = endSegT < basePtT.t;
		float loSegT = searchDown ? endSegT : basePtT.t;
		float hiSegT = searchDown ? basePtT.t : endSegT;
		float testSegT = OpMath::Average(loSegT, hiSegT);
		float step = OpMath::Average(testSegT, -hiSegT);
		OpPtT foundSegPtT;
		OpPtT foundOppPtT;
		for (;;) {
			OpPtT testSegPtT { seg->c.ptTAtT(testSegT) };
			OpPtT testOppPtT { opp->c.ptTAtT(opp->findValidT(0, 1, testSegPtT.pt)) };
			bool validOpp = !OpMath::IsNaN(testOppPtT.t);
			if (validOpp) {
				foundSegPtT = testSegPtT;
				foundOppPtT = testOppPtT;
			}
			if (fabsf(step) < OpEpsilon)
				return FoundPtTs { foundSegPtT, foundOppPtT };
			float testDistSq = (testOppPtT.pt - testSegPtT.pt).lengthSquared();
			// e.g., if step up, dist is smaller, searching down: reverse step
			if ((int) ((step > 0) ^ (validOpp & (testDistSq < baseDistSq)) ^ searchDown))
				step = -step;
			testSegT += step;
			step = step / 2;
		}
	};
	// find limit of where curve pair are nearly coincident
	std::sort(ccSects.begin(), ccSects.end(), [](const CcSects& a, const CcSects& b) {
			return a.e.ptT.t < b.e.ptT.t; });
	FoundLimits limits;
	if (0 == ccSects[0].e.ptT.t || smallTFound)
		limits.lo.setEnd(opp, originalEdge->segment->c, 0);
	else
		limits.lo = binarySearch(ccSects[0].e.ptT, 0);
	if (1 == ccSects.back().e.ptT.t || largeTFound)
		limits.hi.setEnd(opp, originalEdge->segment->c, 1);
	else
		limits.hi = binarySearch(ccSects.back().e.ptT, 1);
	// replace matching existing sects with new range, marking them as unsectable range
	auto sharePoint = [](OpPtT& seg, OpPtT& opp) {
		OpPtT::MeetInTheMiddle(seg, opp);
		seg.t = OpMath::PinNear(seg.t);
		opp.t = OpMath::PinNear(opp.t);
	};
	sharePoint(limits.lo.seg, limits.lo.opp);
	sharePoint(limits.hi.seg, limits.hi.opp);
	addUnsectable2(seg, limits.lo.seg, limits.hi.seg, opp, limits.lo.opp, limits.hi.opp);
	// remove cc sects within the unsectable range
	for (int index = (int) ccSects.size(); index > 0; ) {
		const CcSects& sect = ccSects[--index];
		if (limits.lo.seg.t > sect.e.ptT.t || sect.e.ptT.t > limits.hi.seg.t)
			continue;
		if (limits.lo.opp.t > sect.o.ptT.t || sect.o.ptT.t > limits.hi.opp.t)
			continue;
		ccSects.erase(ccSects.begin() + index);
	}
	// snip remaining parts of curve pairs as needed
	OpContours* contours = seg->contour->contours;
	snipOne(edgeCurves, seg, contours, limits.lo.seg, limits.hi.seg); 
	snipOne(oppCurves, opp, contours, limits.lo.opp, limits.hi.opp); 
}
