// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"

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
	// 3)
	auto splitSwap = [](OpEdge* edge, OpEdge* opp, bool swap) {
		if (swap)
			std::swap(edge, opp);
		edge->doSplit = EdgeSplit::yes;
		OP_ASSERT(EdgeSplit::defer != opp->doSplit);
		opp->doSplit |= EdgeSplit::keep;
	};
	auto setCenter = [](OpRoots& septs, OpEdge& edge) {
		float root = septs.roots[0];
		if (1 == septs.count && edge.start.t < root && root < edge.end.t) {
			// found root should not be too close to end, so split doesn't create miniscule edges
			constexpr float OP_SPLIT_MINIMUM = (1.f / 32);  // !!! tune this
			float spanMin = OP_SPLIT_MINIMUM * (edge.end.t - edge.start.t);
			root = std::min(std::max(root, edge.start.t + spanMin), edge.end.t - spanMin);
//			start here;
			// preserve former center for debugging
			edge.center = { edge.segment->c.ptAtT(root), root };
		}
	};
	auto lineCurvy = [splitSwap, setCenter](OpEdge& edge, OpEdge& opp) {
		LinePts edgePts { edge.start.pt, edge.end.pt };
		OpRoots septs = opp.segment->c.rayIntersect(edgePts);
		setCenter(septs, opp);
		splitSwap(&edge, &opp, true);
	};
	auto lineLine = [splitSwap, setCenter](OpEdge& edge, OpEdge& opp) {
		if (EdgeSplit::yes == opp.doSplit)
			return LineLine::alreadySplit;  // use the existing split center point in place of defining a new one
		LinePts edgePts { edge.start.pt, edge.end.pt };
		OpLine oppLine { opp.start.pt, opp.end.pt };
		OpRoots lineIntersect = oppLine.rawIntersect(edgePts);
		// keep track that intersect failed, and prioritize intersection with another edge
		if (1 != lineIntersect.count) {
			opp.doSplit = EdgeSplit::defer;
			return LineLine::noIntersection;  // if lines don't intersect, that's OK; return to try next test
		}
		OpPoint lineSectPt = oppLine.ptAtT(lineIntersect.roots[0]);
		OpVector oppLengths = opp.end.pt - opp.start.pt;
		Axis oppLongerAxis = fabsf(oppLengths.dx) > fabsf(oppLengths.dy) 
				? Axis::vertical : Axis::horizontal; 
		OpRoots oppSepts = opp.segment->c.axisRayHit(oppLongerAxis, 
				Axis::vertical == oppLongerAxis ? lineSectPt.x : lineSectPt.y); 
		setCenter(oppSepts, opp); 
		splitSwap(&edge, &opp, true);
		return LineLine::setCenter;
	};
	auto rotatedIntersect = [](OpEdge& edge, OpEdge& opp) {
		LinePts edgePts { edge.start.pt, edge.end.pt };
		const OpCurve& edgeCurve = edge.setCurve();
		OpCurve edgeRotated = edgeCurve.toVertical(edgePts);
		OpPointBounds eRotBounds;
		eRotBounds.set(edgeRotated);
		const OpCurve& oppCurve = opp.setCurve();
		OpCurve oppRotated = oppCurve.toVertical(edgePts);
		OpPointBounds oRotBounds;
		oRotBounds.set(oppRotated);
		return eRotBounds.intersects(oRotBounds);
	};
	auto addIntersection = [](OpEdge& edge, OpEdge& opp) {
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
		OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
		if (!edge.segment->sects.contains(edgePtT, opp.segment)
				&& !opp.segment->sects.contains(oppPtT, edge.segment)) {
			OpIntersection* sect = eSegment->addEdgeSect(edgePtT  
					OP_DEBUG_PARAMS(SECT_MAKER(edgeLineCurve), SectReason::lineCurve, 
					&edge, &opp));
			OpIntersection* oSect = oSegment->addEdgeSect(oppPtT  
					OP_DEBUG_PARAMS(SECT_MAKER(edgeLineCurveOpp), SectReason::lineCurve, 
					&edge, &opp));
			sect->pair(oSect);
			return true;
		}
		return false;
	};
	SectFound result = SectFound::no;
	for (int depth = 1; depth < maxDepth; ++depth) {
#if OP_DEBUG_IMAGE
		if (breakAtDraw && 8 <= depth)
			OpDebugOut("");  // allows setting a breakpoint to debug curve/curve
#endif
		for (auto opp : oppCurves)
			opp->doSplit = EdgeSplit::no;
		for (auto edgePtr : edgeCurves) {
			auto& edge = *edgePtr;
			edge.doSplit = EdgeSplit::no;
			bool edgeDone = (edge.start.t + OpEpsilon >= edge.end.t) ||
					edge.start.pt.isNearly(edge.end.pt);
			for (auto oppPtr : oppCurves) {
				auto& opp = *oppPtr;
				if (depth > 1 && !edge.ptBounds.intersects(opp.ptBounds))
					continue;
				if (!rotatedIntersect(edge, opp))
					continue;
				if (!rotatedIntersect(opp, edge))
					continue;
				// check end conditions
				if (edgeDone && (opp.start.t + OpEpsilon >= opp.end.t 
						|| opp.start.pt.isNearly(opp.end.pt))) {
					if (addIntersection(opp, edge))
						result = SectFound::intersects;
					continue;
				}
				float edgeCurvy = edge.curviness();
				float oppCurvy = opp.curviness();
				if (!(std::max(edgeCurvy, oppCurvy) <= OP_CURVACIOUS_LIMIT)) {  // ! includes nan
					// curviness guess: mid t pt distance to end pt line / line length
					if (!OpMath::IsFinite(edgeCurvy))
						return SectFound::fail;
					if (!OpMath::IsFinite(oppCurvy))
						return SectFound::fail;
					OP_ASSERT(oppCurvy);  // if zero, debug how we got here
					// if both are curvy, split the curvier one
					if (std::min(edgeCurvy, oppCurvy) > OP_CURVACIOUS_LIMIT) {
						splitSwap(&edge, &opp, edgeCurvy < oppCurvy);
						continue;
					}
					// if only one is curvy, use line/curve intersection to guess split
					if (edgeCurvy > OP_CURVACIOUS_LIMIT) {
						lineCurvy(opp, edge);
						centers.emplace_back(&edge, CurveRef::edge, depth, CenterSet::edgeCurvy);
						continue;
					}
					if (oppCurvy > OP_CURVACIOUS_LIMIT) {
						lineCurvy(edge, opp);
						centers.emplace_back(&opp, CurveRef::opp, depth, CenterSet::oppCurvy);
						continue;
					}
				}
				// if neither are curvy, use line/line to split the larger one (2nd param)
				if ((edge.end.pt - edge.start.pt).lengthSquared() <
						(opp.end.pt - opp.start.pt).lengthSquared()) {
					if (LineLine lResult = lineLine(edge, opp); LineLine::setCenter == lResult)
						centers.emplace_back(&opp, CurveRef::opp, depth, CenterSet::oppLineLine);
					else if (LineLine::noIntersection == lResult)
						dmp(centers);
					continue;
				} else if (LineLine::setCenter == lineLine(opp, edge)) {
					centers.emplace_back(&edge, CurveRef::edge, depth, CenterSet::edgeLineLine);
					continue;
				}
				// split curvier
				if (EdgeSplit::defer != opp.doSplit)
					splitSwap(&edge, &opp, edgeCurvy < oppCurvy);
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
		splitExperiment(edgeCurves, CurveRef::edge, depth);
		splitExperiment(oppCurves, CurveRef::opp, depth);
		if (!edgeCurves.size() || !oppCurves.size())
			return result;
	}
	return SectFound::fail;
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
void OpCurveCurve::splitExperiment(std::vector<OpEdge*>& curves, CurveRef which, int depth) {
	std::vector<OpEdge*> splits;
	for (auto edgePtr : curves) {
		auto& edge = *edgePtr;
		if (EdgeSplit::no == edge.doSplit) {
			centers.emplace_back(&edge, which, depth, CenterSet::splitNo);
			continue;
		}
		if (EdgeSplit::keep == edge.doSplit) {
			centers.emplace_back(&edge, which, depth, CenterSet::splitKeep);
			if (1 == curves.size())
				return;
			splits.push_back(edgePtr);
			continue;
		}
		if (EdgeSplit::defer == edge.doSplit) {
			// proportionally compute new center since t ranges may not be the same
			float edgeDt = edge.end.t - edge.start.t;
			OpEdge* link = edge.nextEdge ? edge.nextEdge : edge.priorEdge;
			OP_ASSERT(link);
			float linkDt = link->end.t - link->start.t;
			if (edge.nextEdge) {
				OP_ASSERT(link->start == edge.end);
				edge.center.t = edge.end.t - (link->center.t - link->start.t) * edgeDt / linkDt;
			} else {
				OP_ASSERT(link->end == edge.start);
				edge.center.t = edge.start.t + (link->end.t - link->center.t) * edgeDt / linkDt;
			}
			edge.center.pt = edge.segment->c.ptAtT(edge.center.t);
			centers.emplace_back(&edge, which, depth, CenterSet::defer);
		}
		OpContours* contours = originalEdge->contours();
		for (NewEdge newEdge : { NewEdge::isLeft, NewEdge::isRight } ) {
			void* block = contours->allocateEdge(contours->ccStorage);
			OpEdge* split = new(block) OpEdge(&edge, edge.center, newEdge  
					OP_DEBUG_PARAMS(EDGE_MAKER(split1)));
			splits.push_back(split);
			centers.emplace_back(split, which, depth, CenterSet::newEdge);
		}
		// try linking together to handle missing line intersection
		OpEdge* left = splits.back() - 1;
		OpEdge* right = splits.back();
		left->nextEdge = right;
		right->priorEdge = left;
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
