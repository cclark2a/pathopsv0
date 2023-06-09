#include "OpContour.h"
#include "OpEdgeIntersect.h"
#include "OpEdges.h"
#include "OpSegment.h"

static std::vector<OpEdge> findEdgesTRanges(const std::vector<OpEdge>& curves) {
	std::vector<OpEdge> runs;
	std::vector <const OpEdge*> ordered;
	for (const auto& edge : curves) {
		if (EdgeSplit::yes != edge.doSplit)	 // edge bounds didn't overlap opposite
			continue;
		ordered.push_back(&edge);
	}
	std::sort(ordered.begin(), ordered.end(), [](const OpEdge* lhs, const OpEdge* rhs) {
		return lhs->start.t < rhs->start.t; 
	});
	OpPtT start( OpPoint(), OpInfinity );
	OpPtT end( OpPoint(), -OpInfinity );
	// if curve pieces are continuous, join them up
	for (auto edge : ordered) {
		if (!runs.size() || runs.back().end.t != edge->start.t)
			runs.push_back(*edge);
		else
			runs.back().end.t = edge->end.t;
	}
	for (auto& edge : runs)
		edge.complete();
	return runs;
}

static float oppositeT(const OpSegment* segment, const OpEdge& oppEdge, OpPoint test
		OP_DEBUG_PARAMS(int edgeID)) {
	float result;
	FoundPtT found = oppEdge.segment->findPtT(oppEdge.start, oppEdge.end, test, &result);
	if (FoundPtT::multiple == found) {
		assert(0); // debug further
		return OpNaN;
	}
	return result;
}

SectFound OpEdgeIntersect::addCoincidence() {
	OpSegment* segment = const_cast<OpSegment*>(edgeCurves.front().segment);
	OpSegment* oppSegment = const_cast<OpSegment*>(oppCurves.front().segment);
	edgeResults.push_back(edgeCurves.front());
	OpEdge& edgeResult = edgeResults.back();
	oppResults.push_back(oppCurves.front());
	OpEdge& oppResult = oppResults.back();
	// find the extreme pt values and adjust the opposite to match
	OpVector edgeDir = originalEdge->end.pt - originalEdge->start.pt;
	OpVector oppDir = originalOpp->end.pt - originalOpp->start.pt;
	bool overflow, overflow2;
	bool oppReversed = edgeDir.normalize(&overflow).dot(oppDir.normalize(&overflow2)) < 0;
	if (overflow || overflow2)
		return SectFound::overflow;
	if (oppReversed)
		std::swap(oppResult.start, oppResult.end);
	XyChoice edgeXY = fabsf(edgeDir.dx) > fabsf(edgeDir.dy) ? XyChoice::inX : XyChoice::inY;
	float edgeMin = OpNaN;
	float oppMin = OpNaN;
	float edgeMax = OpNaN;
	float oppMax = OpNaN;
	bool edgeReversed = edgeMin > edgeMax;
	bool useEdgeMin = OpMath::IsNaN(oppMin) || (edgeReversed ? edgeMin > oppMin : edgeMin < oppMin);
	bool useEdgeMax = OpMath::IsNaN(oppMax) || (edgeReversed ? edgeMax < oppMax : edgeMax > oppMax);
	OpRoots minRoots, maxRoots;
	if (useEdgeMin)
		minRoots = oppSegment->c.axisRayHit(toAxis(edgeXY), edgeMin, 
				originalOpp->start.t, originalOpp->end.t);
	else
		minRoots = segment->c.axisRayHit(toAxis(edgeXY), oppMin, 
				originalEdge->start.t, originalEdge->end.t);
	if (useEdgeMax)
		maxRoots = oppSegment->c.axisRayHit(toAxis(edgeXY), edgeMax, 
				originalOpp->start.t, originalOpp->end.t);
	else
		maxRoots = segment->c.axisRayHit(toAxis(edgeXY), oppMax, 
				originalEdge->start.t, originalEdge->end.t);
	assert(1 == minRoots.count);	// !!! expect these assert to fire
	assert(1 == maxRoots.count);	// !!! assert means more code is needed for no roots / multiple roots
	// if roots is zero, assume we walked off end; set to original start or end t
	// if roots is two or more, add debug check to see that they are close together, and pick one?
	if (useEdgeMin)
		oppResult.start = { edgeResult.start.pt, minRoots.roots[0] };
	else
		edgeResult.start = { oppResult.start.pt, minRoots.roots[0] };
	if (useEdgeMax)
		oppResult.end = { edgeResult.end.pt, maxRoots.roots[0] };
	else
		edgeResult.end = { oppResult.end.pt, maxRoots.roots[0] };
	if (oppReversed)
		std::swap(oppResult.start, oppResult.end);
	edgeResult.complete();
	oppResult.complete();
	addCurveCoin(edgeResult, oppResult);
	return SectFound::intersects;
}

// trim front and back of ranges
SectFound OpEdgeIntersect::addCurveCoincidence() {
	std::vector<OpEdge> edgeRuns = findEdgesTRanges(edgeCurves);
	std::vector<OpEdge> oppRuns = findEdgesTRanges(oppCurves);
	Axis larger = originalEdge->ptBounds.width() > originalEdge->ptBounds.height() ? 
			Axis::vertical : Axis::horizontal;
	for (auto& edge : edgeRuns) {
		for (auto& opp : oppRuns) {
			if (opp.ptBounds.ltChoice(larger) >= edge.ptBounds.rbChoice(larger)
					|| opp.ptBounds.rbChoice(larger) <= edge.ptBounds.ltChoice(larger))
				continue;
			std::pair<float, float> maxminXY = std::minmax(edge.ptBounds.ltChoice(larger), 
					opp.ptBounds.ltChoice(larger));
			OpPtT edgeStart = maxminXY.second == edge.start.pt.choice(larger) ?
					edge.start : edge.findPtT(larger, maxminXY.second);
			OpPtT edgeEnd = maxminXY.first == edge.end.pt.choice(larger) ?
					edge.end : edge.findPtT(larger, maxminXY.first);

			OpPtT oppStart = maxminXY.second == opp.start.pt.choice(larger) ?
					opp.start : maxminXY.second == opp.end.pt.choice(larger) ?
					opp.end : opp.findPtT(larger, maxminXY.second);
			OpPtT oppEnd = maxminXY.first == opp.start.pt.choice(larger) ?
					opp.start : maxminXY.first == opp.end.pt.choice(larger) ?
					opp.end : opp.findPtT(larger, maxminXY.first);
				start here;
		}
	}

	addCurveCoin(edgeResults.back(), oppResults.back());
	return SectFound::intersects;
}

void OpEdgeIntersect::addCurveCoin(OpEdge& edge, OpEdge& oppEdge) {
	edge.dump();
	OpSegment* segment = const_cast<OpSegment*>(edge.segment);
	OpSegment* oppSegment = const_cast<OpSegment*>(oppEdge.segment);
	float oppStartT = oppositeT(segment, oppEdge, edge.start.pt  OP_DEBUG_PARAMS(edge.id));
	float oppEndT = oppositeT(segment, oppEdge, edge.end.pt  OP_DEBUG_PARAMS(edge.id));
	float startT = oppositeT(oppSegment, edge, oppEdge.start.pt  OP_DEBUG_PARAMS(oppEdge.id));
	float endT = oppositeT(oppSegment, edge, oppEdge.end.pt  OP_DEBUG_PARAMS(oppEdge.id));
	bool reversed = segment->c.tangent(edge.center.t)
			.dot(oppSegment->c.tangent(oppEdge.center.t)) < 0;
	if (reversed) {
		std::swap(startT, endT);
		std::swap(oppEdge.start, oppEdge.end);
	}
	// There should be 2, 3, or 4 t values between the edge start and end.
	// If, for instance, opp start t is between, it could match start t or end t, if between
	OpPtT segStart = edge.start;
	OpPtT segEnd = edge.end;
	OpPtT oppStart = oppEdge.start;
	OpPtT oppEnd = oppEdge.end;
	if (OpMath::Between(edge.start.t, startT, edge.end.t))
		segStart = OpPtT(oppEdge.start.pt, startT);
	else
		oppStart = OpPtT(edge.start.pt, oppStartT);
	if (OpMath::Between(edge.start.t, endT, edge.end.t))
		segEnd = OpPtT(oppEdge.end.pt, endT);
	else
		oppEnd = OpPtT(edge.end.pt, oppEndT);
	if (segStart.t == segEnd.t || oppStart.t == oppEnd.t)
		return;
	// A reversed opp start is opp end and vice versa, but since both will be added as
	// intersections, another reversal doesn't improve anything
	//	if (reversed)
	//		std::swap(oppStart, oppEnd);
	int coinID = segment->coinID(reversed);
	// assume for now that its correct to remove all intersections between the coincident ids
	for (auto iterPtr = segment->intersections.begin(); iterPtr != segment->intersections.end(); ) {
		const OpIntersection* iter = *iterPtr;
		if (iter->segment != oppSegment || !OpMath::Between(segStart.t, iter->ptT.t, segEnd.t)
				|| iter->coincidenceID) {
			++iterPtr;
			continue;
		}
		OP_DEBUG_CODE((*iterPtr)->debugErased = true);
		segment->intersections.erase(iterPtr);
	}
	for (auto iterPtr = oppSegment->intersections.begin(); 
			iterPtr != oppSegment->intersections.end(); ) {
		const OpIntersection* iter = *iterPtr;
		if (iter->segment != segment || !OpMath::Between(oppStart.t, iter->ptT.t, oppEnd.t)
				|| iter->coincidenceID) {
			++iterPtr;
			continue;
		}
		OP_DEBUG_CODE((*iterPtr)->debugErased = true);
		oppSegment->intersections.erase(iterPtr);
	}
	OpIntersection* segSect1 = segment->addIntersection(segStart, coinID  
			OP_DEBUG_PARAMS(SECT_MAKER(addCurveCoinStart), SectReason::curveCurveCoincidence,
			nullptr, &edge, &oppEdge));
	OpIntersection* segSect2 = segment->addIntersection(segEnd, coinID  
			OP_DEBUG_PARAMS(SECT_MAKER(addCurveCoinEnd), SectReason::curveCurveCoincidence,
			nullptr, &edge, &oppEdge));
	OpIntersection* oppSect1 = oppSegment->addIntersection(oppStart, coinID  
			OP_DEBUG_PARAMS(SECT_MAKER(addCurveCoinOppStart), SectReason::curveCurveCoincidence,
			nullptr, &oppEdge, &edge));
	OpIntersection* oppSect2 = oppSegment->addIntersection(oppEnd, coinID  
			OP_DEBUG_PARAMS(SECT_MAKER(addCurveCoinOppEnd), SectReason::curveCurveCoincidence,
			nullptr, &oppEdge, &edge));
	segSect1->pair(segStart.pt == oppStart.pt ? oppSect1 : oppSect2);
	segSect2->pair(segEnd.pt == oppStart.pt ? oppSect1 : oppSect2);
}

// this marks opp as splittable if edge is a curve
// this finds opp t if edge is a line
// runs twice: edge/opp, then opp/edge
// result is for all edges: no (intersections at all); fail; (something) split; line sect (no splits)
SectFound OpEdgeIntersect::CurvesIntersect(std::vector<OpEdge>& edgeParts,
		std::vector<OpEdge>& oppParts) {
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

IntersectResult OpEdgeIntersect::CurveCenter(const OpEdge& edge, OpEdge& opp) {
	// check to see if center t is equal to start or end (because delta t is so small)
	if (edge.start.t >= edge.center.t || edge.center.t >= edge.end.t) {
		OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
		OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
		OpIntersection* sect = eSegment->addIntersection(edge.center  
				OP_DEBUG_PARAMS(SECT_MAKER(curveCenter), SectReason::degenerateCenter, 
				nullptr, &edge, &opp));
		const OpCurve& oppCurve = opp.setCurve();
		float oppTx = oppCurve.findIntersect(Axis::vertical, edge.center).t;
		float oppTy = oppCurve.findIntersect(Axis::horizontal, edge.center).t;
		OpPtT oppPtT { edge.center.pt, (oppTx + oppTy) / 2 };
		OpIntersection* oSect = oSegment->addIntersection(oppPtT  
				OP_DEBUG_PARAMS(SECT_MAKER(curveCenterOpp), SectReason::degenerateCenter, 
				nullptr, &opp, &edge));
		sect->pair(oSect);
		return IntersectResult::yes;
	}
	return IntersectResult::maybe;
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
		SectFound oppResult = CurvesIntersect(edgeCurves, oppCurves);
		if (SectFound::fail == oppResult)
			return oppResult;
		SectFound edgeResult = CurvesIntersect(oppCurves, edgeCurves);
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
			if (splitType == DoSplit::marked || edgeCurves.size() * 2 <= oppCurves.size()) {
				if (!Split(edgeCurves, edgeLines, splitType))
					return SectFound::no;
				keepGoing = true;
			}
			if (splitType == DoSplit::marked || edgeCurves.size() >= oppCurves.size() * 2) {
				if (!Split(oppCurves, oppLines, splitType))
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

bool OpEdgeIntersect::Split(std::vector<OpEdge>& curves, std::vector<OpEdge>& lines, DoSplit doSplit) {
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

#if 0

// edge is a line
			OpRoots septs;	// don't continue to split edges; look for the intersection directly
			septs = oppRotated.axisRayHit(Axis::vertical, 0);
			float edgeMinY = std::min(edgeRotated.pts[0].y, std::min(edgeRotated.pts[1].y, 
					std::min(edgeRotated.pts[2].y, edgeRotated.pts[3].y)));
			float edgeMaxY = std::max(edgeRotated.pts[0].y, std::max(edgeRotated.pts[1].y, 
					std::max(edgeRotated.pts[2].y, edgeRotated.pts[3].y)));
			for (unsigned index = 0; index < septs.count; ++index) {
				OpPoint pt = oppRotated.ptAtT(septs.get(index));
				if (edgeMinY > pt.y || pt.y > edgeMaxY)
					continue;
				if (SectFound::split != result)
					result = SectFound::intersects;	// a pair intersected, but may not split
				oppTs.push_back(OpMath::Interp(opp.start.t, opp.end.t, septs.get(index)));
			}

#endif
