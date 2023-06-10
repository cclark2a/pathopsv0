#include "OpContour.h"
#include "OpEdgeIntersect.h"
#include "OpEdges.h"
#include "OpSegment.h"

// trim front and back of ranges
SectFound OpEdgeIntersect::addCurveCoincidence() {
	std::vector<OpEdge> edgeRuns = findEdgesTRanges(CurveRef::edge);
	std::vector<OpEdge> oppRuns = findEdgesTRanges(CurveRef::opp);
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
				else
					found.t = edge.findT(larger, xy);
			};
			OpPtT edgeStart, edgeEnd, oppStart, oppEnd;
			float minXY = std::max(edge.ptBounds.ltChoice(larger), 
					opp.ptBounds.ltChoice(larger));
			float maxXY = std::min(edge.ptBounds.rbChoice(larger), 
					opp.ptBounds.rbChoice(larger));
			assert(minXY < maxXY);
			if (edge.start.pt.choice(larger) > edge.end.pt.choice(larger))
				std::swap(minXY, maxXY);
			findMatch(minXY, edge, edgeStart);
			findMatch(maxXY, edge, edgeEnd);
			findMatch(minXY, opp, oppStart);
			findMatch(maxXY, opp, oppEnd);
			bool reversed = oppStart.t > oppEnd.t;
			if (reversed)
				std::swap(oppStart, oppEnd);
			if (!edgeStart.pt.isFinite())
				edgeStart.pt = oppStart.pt;
			if (!edgeEnd.pt.isFinite())
				edgeEnd.pt = oppEnd.pt;
			if (!oppStart.pt.isFinite())
				oppStart.pt = edgeStart.pt;
			if (!oppEnd.pt.isFinite())
				oppEnd.pt = edgeEnd.pt;
			OpSegment* segment = const_cast<OpSegment*>(edge.segment);
			int coinID = segment->coinID(reversed);
			OpIntersection* segSect1 = segment->addIntersection(edgeStart, coinID  
					OP_DEBUG_PARAMS(SECT_MAKER(addCurveCoinStart), SectReason::curveCurveCoincidence,
					nullptr, originalEdge, originalOpp));
			OpIntersection* segSect2 = segment->addIntersection(edgeEnd, coinID  
					OP_DEBUG_PARAMS(SECT_MAKER(addCurveCoinEnd), SectReason::curveCurveCoincidence,
					nullptr, originalEdge, originalOpp));
			OpSegment* oppSegment = const_cast<OpSegment*>(opp.segment);
			OpIntersection* oppSect1 = oppSegment->addIntersection(oppStart, coinID  
					OP_DEBUG_PARAMS(SECT_MAKER(addCurveCoinOppStart), SectReason::curveCurveCoincidence,
					nullptr, originalEdge, originalOpp));
			OpIntersection* oppSect2 = oppSegment->addIntersection(oppEnd, coinID  
					OP_DEBUG_PARAMS(SECT_MAKER(addCurveCoinOppEnd), SectReason::curveCurveCoincidence,
					nullptr, originalEdge, originalOpp));
			segSect1->pair(edgeStart.pt == oppStart.pt ? oppSect1 : oppSect2);
			segSect2->pair(edgeEnd.pt == oppStart.pt ? oppSect1 : oppSect2);
		}
	}

	return SectFound::intersects;
}

// this marks opp as splittable if edge is a curve
// this finds opp t if edge is a line
// runs twice: edge/opp, then opp/edge
// result is for all edges: no (intersections at all); fail; (something) split; line sect (no splits)
SectFound OpEdgeIntersect::curvesIntersect(CurveRef curveRef) {
	std::vector<OpEdge>& edgeParts = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	std::vector<OpEdge>& oppParts = CurveRef::edge == curveRef ? oppCurves : edgeCurves;
	SectFound result = SectFound::no;	// assumes no pair intersects; we're done
	for (auto& edge : edgeParts) {
		// !!! if assert fires, we missed detecting linear edge earlier
		assert(edge.start.t < edge.center.t && edge.center.t < edge.end.t);
		assert(!edge.isLine_impl);
		// rotate each to see if tight rotated bounds (with extrema) touch
		std::array<OpPoint, 2> edgeLine { edge.start.pt, edge.end.pt };
		OpCurve& edgeRotated = const_cast<OpCurve&>(edge.setVertical());
		if (!edgeRotated.isFinite())
			return SectFound::fail;
		OpTightBounds edgeRotatedBounds(edgeRotated);
		for (auto& opp : oppParts) {
			assert(!opp.isLine_impl);
			if (!edge.ptBounds.intersects(opp.ptBounds))
				continue;
			const OpCurve& oppCurve = opp.setCurve();
			OpCurve oppRotated;
			oppCurve.toVertical(edgeLine, oppRotated);
			if (!oppRotated.isFinite())
				return SectFound::fail;
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
SectFound OpEdgeIntersect::divideAndConquer() {
#if OP_DEBUG_IMAGE
	bool breakAtDraw = 53 == originalEdge->id && 45 == originalOpp->id;
	if (breakAtDraw) {
		hideOperands();
		hideSegmentEdges();
		showTemporaryEdges();
		showPoints();
		showValues();
		showGrid();
	}
#endif
	assert(1 == edgeCurves.size());
	assert(0 == edgeLines.size());
	assert(1 == oppCurves.size());
	assert(0 == oppLines.size());
	edgeCurves[0].addMatchingEnds(oppCurves[0]);
	for (int depth = 1; depth < maxDepth; ++depth) {
#if OP_DEBUG_IMAGE
		if (breakAtDraw && 8 <= depth)
			OpDebugOut("");
#endif
		if (edgeLines.size() && oppLines.size())
			LinearIntersect(edgeLines, oppLines);
		if (edgeLines.size())
			LinearIntersect(edgeLines, oppCurves);
		if (oppLines.size())
			LinearIntersect(oppLines, edgeCurves);
		SectFound oppResult = curvesIntersect(CurveRef::edge);
		if (SectFound::fail == oppResult)
			return oppResult;
		SectFound edgeResult = curvesIntersect(CurveRef::opp);
		if (SectFound::fail == edgeResult)
			return edgeResult;
		if (SectFound::no == oppResult || SectFound::no == edgeResult)
			return SectFound::no;
		if (edgeCurves.size() >= maxSplits || oppCurves.size() >= maxSplits)
			return addCurveCoincidence();
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
			if (depth >= 5)
				OpDebugOut("");
		}
#endif
	}
	assert(0);
	return SectFound::no;
}

static bool compareEdges(const OpEdge* lhs, const OpEdge* rhs) {
	return lhs->start.t < rhs->start.t; 
}

std::vector<OpEdge> OpEdgeIntersect::findEdgesTRanges(CurveRef curveRef) {
	const std::vector<OpEdge>& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	std::vector<OpEdge> runs;
	std::vector <const OpEdge*> ordered;
	for (const auto& edge : curves) {
		if (EdgeSplit::yes != edge.doSplit)	 // edge bounds didn't overlap opposite
			continue;
		ordered.push_back(&edge);
	}
	// !!! I'm a bad programmer. Lambda sort comparator doesn't work, but I don't know why
	std::sort(ordered.begin(), ordered.end(), compareEdges); // [](const OpEdge* lhs, const OpEdge* rhs) {
//		return lhs->start.t < rhs->start.t; 
//	});
	// if curve pieces are continuous, join them up
	for (auto edge : ordered) {
		if (!runs.size() || runs.back().end.t != edge->start.t)
			runs.push_back(*edge);
		else
			runs.back().end = edge->end;
	}
	for (auto& edge : runs)
		edge.complete();
	return runs;
}

// Edge parts are all linear. See if each intersects with any of the opposite edges
void OpEdgeIntersect::LinearIntersect(std::vector<OpEdge>& edgeParts,
		std::vector<OpEdge>& oppParts) {
	for (OpEdge& edge : edgeParts) {
		OP_ASSERT(edge.isLine_impl);
		// !!! optimization: make edge point bounds array of two points...
		edge.setPointBounds();
		for (OpEdge& opp : oppParts) {
			opp.setPointBounds();
			if (!edge.ptBounds.intersects(opp.ptBounds))
				continue;
			OpEdges::AddLineCurveIntersection(opp, edge);
		}
	}
}

bool OpEdgeIntersect::split(CurveRef curveRef, DoSplit doSplit) {
	std::vector<OpEdge>& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	std::vector<OpEdge>& lines = CurveRef::edge == curveRef ? edgeLines : oppLines;
	std::vector<OpEdge> splits;	// curves only
	for (auto& edge : curves) {
		assert(!edge.isLine_impl);
		if (EdgeSplit::no == edge.doSplit && DoSplit::marked == doSplit)
			continue;
		// constructor invoked by emplace back does all the heavy lifting (initialization)
		// !!! think about what (if anything) would be useful to record in debug breadcrumbs
		// assume most curves split into more curves
		for (NewEdge newEdge : { NewEdge::isLeft, NewEdge::isRight } ) {
			splits.emplace_back(&edge, edge.center, newEdge  
					OP_DEBUG_PARAMS(EDGE_MAKER(split1), nullptr, nullptr));
			if (splits.back().setLinear()) {
				lines.push_back(splits.back());
				splits.pop_back();
			}
		}
	}
	curves.swap(splits);
	return curves.size() || lines.size();
}

bool OpEdgeIntersect::tooFew(CurveRef curveRef) {
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
