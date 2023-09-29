// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpSegment.h"
#include "OpWinder.h"

// trim front and back of ranges
SectFound OpCurveCurve::addUnsectable() {
	findEdgesTRanges(CurveRef::edge);
	findEdgesTRanges(CurveRef::opp);
	Axis larger = originalEdge->ptBounds.width() > originalEdge->ptBounds.height() ? 
			Axis::vertical : Axis::horizontal;
	for (auto& edge : edgeRuns) {
		for (auto& opp : oppRuns) {
			if (opp.ptBounds.ltChoice(larger) >= edge.ptBounds.rbChoice(larger)
					|| opp.ptBounds.rbChoice(larger) <= edge.ptBounds.ltChoice(larger))
				continue;
			auto findMatch = [larger](float xy, const OpEdge& edge, OpPtT& found) {
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
	std::vector<OpEdge>& edgeParts = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	std::vector<OpEdge>& oppParts = CurveRef::edge == curveRef ? oppCurves : edgeCurves;
	SectFound result = SectFound::no;	// assumes no pair intersects; we're done
	for (auto& edge : edgeParts) {
		if (edge.start.t >= edge.center.t || edge.center.t >= edge.end.t) {
			OP_ASSERT(OpDebugExpect::unknown == edge.segment->contour->contours->debugExpect);
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
		for (auto& opp : oppParts) {
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
	bool breakAtDraw = 104 == originalEdge->id && 106 == originalOpp->id;
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
			LinearIntersect(edgeLines, oppLines);
		if (edgeLines.size())
			LinearIntersect(edgeLines, oppCurves);
		if (oppLines.size())
			LinearIntersect(oppLines, edgeCurves);
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

void OpCurveCurve::findEdgesTRanges(CurveRef curveRef) {
	const std::vector<OpEdge>& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	std::vector <const OpEdge*> ordered;
	for (const auto& edge : curves) {
		if (EdgeSplit::yes != edge.doSplit)	 // edge bounds didn't overlap opposite
			continue;
		ordered.push_back(&edge);
	}
	std::sort(ordered.begin(), ordered.end(), [](const OpEdge*& lhs, const OpEdge*& rhs) {
		return lhs->start.t < rhs->start.t; 
	});
	// if curve pieces are continuous, join them up
	std::vector<OpEdge>& runs = CurveRef::edge == curveRef ? edgeRuns : oppRuns;
	for (auto edge : ordered) {
		if (!runs.size() || runs.back().end.t != edge->start.t)
			runs.push_back(*edge);
		else
			runs.back().end = edge->end;
	}
	for (auto& edge : runs) {
		OP_ASSERT(WindingType::winding == edge.winding.debugType);
		edge.subDivide();
	}
}

// Edge parts are all linear. See if each intersects with any of the opposite edges
void OpCurveCurve::LinearIntersect(std::vector<OpEdge>& edgeParts,
		std::vector<OpEdge>& oppParts) {
	for (OpEdge& edge : edgeParts) {
		OP_ASSERT(edge.isLine_impl);
		// !!! optimization: make edge point bounds array of two points...
		edge.setPointBounds();
		for (OpEdge& opp : oppParts) {
			opp.setPointBounds();
			if (!edge.ptBounds.intersects(opp.ptBounds))
				continue;
			OpWinder::AddLineCurveIntersection(opp, edge);
		}
	}
}

bool OpCurveCurve::split(CurveRef curveRef, DoSplit doSplit) {
	std::vector<OpEdge>& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	std::vector<OpEdge>& lines = CurveRef::edge == curveRef ? edgeLines : oppLines;
	std::vector<OpEdge> splits;	// curves only
	for (auto& edge : curves) {
		OP_ASSERT(!edge.isLine_impl);
		if (EdgeSplit::no == edge.doSplit && DoSplit::marked == doSplit)
			continue;
		// constructor invoked by emplace back does all the heavy lifting (initialization)
		// !!! think about what (if anything) would be useful to record in debug breadcrumbs
		// assume most curves split into more curves
		for (NewEdge newEdge : { NewEdge::isLeft, NewEdge::isRight } ) {
			splits.emplace_back(&edge, edge.center, newEdge  
					OP_DEBUG_PARAMS(EDGE_MAKER(split1), nullptr, nullptr));
//					CurveRef::edge == curveRef ? originalEdge : originalOpp,
//					CurveRef::edge == curveRef ? originalOpp : originalEdge));
			if (splits.back().isLinear()) {
				lines.push_back(splits.back());
				splits.pop_back();
			}
		}
	}
	curves.swap(splits);
	return curves.size() || lines.size();
}

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
