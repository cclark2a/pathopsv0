#include "OpContour.h"
#include "OpEdgeIntersect.h"
#include "OpEdges.h"
#include "OpSegment.h"


static OpEdge& findEdgesTRange(std::vector<OpEdge>& parts, const OpSegment* oppSegment) {
	OpEdge& result = parts.front();
	for (auto iter = std::next(parts.begin()); iter != parts.end(); ++iter) {
		if (result.start.t > iter->start.t)
			result.start = iter->start;
		if (result.end.t < iter->end.t)
			result.end = iter->end;
	}
	// extend edge, if possible, by considering found intersections
	// assume that if part of the pair is coincident, there can be only one isolated common point
	// !!! may need to check if single isolated point is near or is on edge which is a line
	const OpSegment* segment = result.segment;
	OpPtT min = { OpPoint(), OpInfinity };
	OpPtT max = { OpPoint(), -OpInfinity };
	for (auto& cept : segment->intersections) {
		if (cept.segment != oppSegment)
			continue;
		if (cept.coincidenceID)
			continue;
		if (OpMath::Between(result.start.t, cept.ptT.t, result.end.t))
			continue;
		if (min.t > cept.ptT.t)
			min = cept.ptT;
		if (max.t < cept.ptT.t)
			max = cept.ptT;
	}
	if (min.t < max.t) {
		if (result.start.t > min.t)
			result.start = min;
		if (result.end.t < max.t)
			result.end = max;
	}
	result.subDivide();
	return result;
}

static float coincidentT(const OpSegment* segment, const OpEdge& oppEdge, OpPtT test
		OP_DEBUG_PARAMS(int edgeID)) {
	const OpSegment* oppSegment = oppEdge.segment;
	const OpCurve& oppCurve = oppSegment->c;
	std::array<OpPoint, 2> line = { test.pt, test.pt + segment->c.normal(test.t) };
	rootCellar cepts;
	int count = oppCurve.rawIntersect(line, cepts);
	float bestT = OpInfinity;
	float bestDistanceSq = OpInfinity;
	for (int index = 0; index < count; ++index) {
		float t = cepts[index];
		OpPoint pt = oppCurve.ptAtT(t);
		float distanceSq = (pt - test.pt).lengthSquared();
		if (bestDistanceSq > distanceSq) {
			bestDistanceSq = distanceSq;
			bestT = t;
			// bestT = distanceSq < OpEpsilon ? test.t : t;  this doesn't work for loops61i
		}
	}
	OpDebugOut("bestDistance: " + std::to_string(sqrtf(bestDistanceSq)) + " edge: "
			+ STR(edgeID) + " opp: " + STR(oppEdge.id) + "\n");
	return bestT;
}

void OpEdgeIntersect::addCoincidence() {
	OpSegment* segment = const_cast<OpSegment*>(edgeParts.front().segment);
//	OpSegment* oppSegment = const_cast<OpSegment*>(oppParts.front().segment);
	OpEdge& edgeResult = edgeParts.front();
	OpEdge& oppResult = oppParts.front();
	edgeResult.start.t = *std::min_element(edgeTs.begin(), edgeTs.end());
	edgeResult.start.pt = segment->c.ptAtT(edgeResult.start.t);
	edgeResult.end.t = *std::max_element(edgeTs.begin(), edgeTs.end());
	edgeResult.end.pt = segment->c.ptAtT(edgeResult.end.t);
	oppResult.start.t = *std::min_element(oppTs.begin(), oppTs.end());
	oppResult.start.pt = segment->c.ptAtT(oppResult.start.t);
	oppResult.end.t = *std::max_element(oppTs.begin(), oppTs.end());
	oppResult.end.pt = segment->c.ptAtT(oppResult.end.t);
	addCurveCoin(edgeResult, oppResult);
}

// trim front and back of ranges
void OpEdgeIntersect::addCurveCoincidence() {
	OpSegment* segment = const_cast<OpSegment*>(edgeParts.front().segment);
	OpSegment* oppSegment = const_cast<OpSegment*>(oppParts.front().segment);
	OpEdge& edge = findEdgesTRange(edgeParts, oppSegment);
	OpEdge& oppEdge = findEdgesTRange(oppParts, segment);
	addCurveCoin(edge, oppEdge);
}

void OpEdgeIntersect::addCurveCoin(OpEdge& edge, OpEdge& oppEdge) {
	OpSegment* segment = const_cast<OpSegment*>(edge.segment);
	OpSegment* oppSegment = const_cast<OpSegment*>(oppEdge.segment);
#if OP_DEBUG
	if (100 == edge.id && 112 == oppEdge.id)
		OpDebugOut("");
#endif
	float oppStartT = coincidentT(segment, oppEdge, edge.start  OP_DEBUG_PARAMS(edge.id));
	float oppEndT = coincidentT(segment, oppEdge, edge.end  OP_DEBUG_PARAMS(edge.id));
	float startT = coincidentT(oppSegment, edge, oppEdge.start  OP_DEBUG_PARAMS(oppEdge.id));
	float endT = coincidentT(oppSegment, edge, oppEdge.end  OP_DEBUG_PARAMS(oppEdge.id));
	bool reversed = segment->c.tangent(edge.center.t)
			.dot(oppSegment->c.tangent(oppEdge.center.t)) < 0;
	if (reversed) {
		std::swap(startT, endT);
		std::swap(oppEdge.start, oppEdge.end);
	}
	// There should be 2, 3, or 4 t values between the edge start and end.
	// If, for instance, opp start t is between, it could match start t or end t, if between
	OpPtT segStart = OpMath::Between(edge.start.t, startT, edge.end.t)
			? OpPtT(oppEdge.start.pt, startT) : edge.start;
	OpPtT segEnd = OpMath::Between(edge.start.t, endT, edge.end.t)
			? OpPtT(oppEdge.end.pt, endT) : edge.end;
	OpPtT oppStart = OpMath::Between(oppEdge.start.t, oppStartT, oppEdge.end.t)
			? OpPtT(edge.start.pt, oppStartT) : oppEdge.start;
	OpPtT oppEnd = OpMath::Between(oppEdge.start.t, oppEndT, oppEdge.end.t)
			? OpPtT(edge.end.pt, oppEndT) : oppEdge.end;
	assert(segStart.pt == oppStart.pt);
	assert(segEnd.pt == oppEnd.pt);
	// A reversed opp start is opp end and vice versa, but since both will be added as
	// intersections, another reversal doesn't improve anything
	//	if (reversed)
	//		std::swap(oppStart, oppEnd);
	int coinID = segment->coinID(reversed);
	// assume for now that its correct to remove all intersections between the coincident ids
	for (auto iter = segment->intersections.begin(); iter != segment->intersections.end(); ) {
		if (iter->segment != oppSegment || !OpMath::Between(segStart.t, iter->ptT.t, segEnd.t)
				|| iter->coincidenceID) {
			++iter;
			continue;
		}
		segment->intersections.erase(iter);
	}
	for (auto iter = oppSegment->intersections.begin(); iter != oppSegment->intersections.end(); ) {
		if (iter->segment != segment || !OpMath::Between(oppStart.t, iter->ptT.t, oppEnd.t)
				|| iter->coincidenceID) {
			++iter;
			continue;
		}
		oppSegment->intersections.erase(iter);
	}
	segment->intersections.emplace_back(segStart, oppSegment, coinID
			OP_DEBUG_PARAMS(IntersectMaker::addCurveCoincidence1));
	segment->intersections.emplace_back(segEnd, oppSegment, coinID
			OP_DEBUG_PARAMS(IntersectMaker::addCurveCoincidence2));
	oppSegment->intersections.emplace_back(oppStart, segment, coinID
			OP_DEBUG_PARAMS(IntersectMaker::addCurveCoincidence3));
	oppSegment->intersections.emplace_back(oppEnd, segment, coinID
			OP_DEBUG_PARAMS(IntersectMaker::addCurveCoincidence4));
}



bool OpEdgeIntersect::atMaxSplits() {
	auto checkSplits = [](std::vector<OpEdge>& parts) {
		int splits = 0;
		for (auto& edge : parts)
			if (EdgeSplit::no != edge.doSplit && ++splits >= maxSplits)
				return true;
		return false;
	};
	if (!checkSplits(edgeParts))
		return false;
	if (!checkSplits(oppParts))
		return false;
	auto keepSplits = [](std::vector<OpEdge>& parts) {
		std::vector<OpEdge> splits;
		for (auto& edge : parts) {
			if (EdgeSplit::no != edge.doSplit)
				splits.emplace_back(edge);
		}
		parts.swap(splits);
	};
	keepSplits(edgeParts);
	keepSplits(oppParts);
	return true;
}

// this marks edge as unsplittable if t span is too small
// this marks opp as splittable if edge is a curve
// this finds opp t if edge is a line
// runs twice: edge/opp, then opp/edge
// result is for all edges: no (intersections at all); fail; (something) split; line sect (no splits)
SectFound OpEdgeIntersect::CurvesIntersect(std::vector<OpEdge>& edgeParts,
		std::vector<OpEdge>& oppParts, std::vector<float>& oppTs) {
	SectFound result = SectFound::no;	// assumes no pair intersects; we're done
	for (auto& edge : edgeParts) {
		if (edge.start.t >= edge.center.t || edge.center.t >= edge.end.t) {
			edge.doSplit = EdgeSplit::unsplittable;
			continue;
		}
		// rotate each to see if tight rotated bounds (with extrema) touch
		std::array<OpPoint, 2> edgeLine = { edge.start.pt, edge.end.pt };
		const OpCurve& edgeRotated = edge.setVertical();
		if (!edgeRotated.isFinite())
			return SectFound::fail;
		OpTightBounds edgeRotatedBounds(edgeRotated);
		for (auto& opp : oppParts) {
			if (!edge.pointBounds.intersects(opp.pointBounds))
				continue;
			const OpCurve& oppCurve = opp.setCurve();
			OpCurve oppRotated;
			oppCurve.toVertical(edgeLine, oppRotated);
			if (!oppRotated.isFinite())
				return SectFound::fail;
			OpTightBounds oppRotatedBounds(oppRotated);
			if (!edgeRotatedBounds.intersects(oppRotatedBounds))
				continue;
			// if not line: opp to be split, opp cannot be split
			// if line: line intersection found hit(s), found no hits
			if (!edge.setLinear()) {
				if (EdgeSplit::no == opp.doSplit && !opp.setLinear()) {
					result = SectFound::split;	// an edge is to be split
					opp.doSplit = EdgeSplit::yes;
				}
				continue;
			}
			// edge is a line
			OpRoots septs;	// don't continue to split edges; look for the intersection directly
			septs.count = oppRotated.axisRayHit(Axis::vertical, 0, septs.roots);
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
			showPoints();
			showValues();
			showTs();
			::draw();
			OpDebugOut("");
		}
	}
	return result;
}

IntersectResult OpEdgeIntersect::CurveCenter(const OpEdge& edge, OpEdge& opp) {
	// check to see if center t is equal to start or end (because delta t is so small)
	if (edge.start.t >= edge.center.t || edge.center.t >= edge.end.t) {
		OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
		OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
		eSegment->intersections.emplace_back(edge.center, oSegment, 0  
				OP_DEBUG_PARAMS(IntersectMaker::curveCenter1));
		const OpCurve& oppCurve = opp.setCurve();
		float oppTx = oppCurve.findIntersect(Axis::vertical, edge.center).t;
		float oppTy = oppCurve.findIntersect(Axis::horizontal, edge.center).t;
		OpPtT oppPtT = { edge.center.pt, (oppTx + oppTy) / 2 };
		oSegment->intersections.emplace_back(oppPtT, eSegment, 0  
				OP_DEBUG_PARAMS(IntersectMaker::curveCenter2));
		return IntersectResult::yes;
	}
	return IntersectResult::maybe;
}


/* note: does not (currently) reject via points bounds; rejects via rotated tight bounds 
  after an edge is compared against all opposites, if tight bounds didn't intersect, it isn't copied
 */
SectFound OpEdgeIntersect::divideAndConquer() {
#if OP_DEBUG_IMAGE
	bool breakAtDraw = true; // 43 == edgeParts[0].id && 56 == oppParts[0].id;
#endif
	assert(1 == edgeParts.size());
	assert(1 == oppParts.size());
	OpEdge& edge = edgeParts[0];
	OpEdge& opp = oppParts[0];
	edge.addMatchingEnds(opp);
	for (int depth = 1; depth < maxDepth; ++depth) {
#if OP_DEBUG_IMAGE
		if (breakAtDraw && 7 == depth)
			OpDebugOut("");
#endif
		SectFound oppResult = CurvesIntersect(edgeParts, oppParts, oppTs);
		if (SectFound::fail == oppResult || SectFound::no == oppResult)
			return oppResult;
		SectFound edgeResult = CurvesIntersect(oppParts, edgeParts, edgeTs);
		if (SectFound::fail == edgeResult || SectFound::no == edgeResult)
			return edgeResult;
		if (edgeTs.size() >= maxSplits || oppTs.size() >= maxSplits) {
			addCoincidence();
			return SectFound::intersects;
		}
		OpDebugOut("");
		if (atMaxSplits()) {
			addCurveCoincidence();
			return SectFound::intersects;
		}
		Split(edgeParts, DoSplit::marked);
		if (!edgeParts.size())
			return SectFound::no;
		Split(oppParts, DoSplit::marked);
		if (!oppParts.size())
			return SectFound::no;
		// if one side has split more than double the other, do it again
		while (edgeParts.size() * 2 <= oppParts.size())
			Split(edgeParts, DoSplit::all);
		while (edgeParts.size() >= oppParts.size() * 2)
			Split(oppParts, DoSplit::all);
#if OP_DEBUG_IMAGE
		if (breakAtDraw) {
			draw();
			OpDebugOut("");
		}
#endif
	}
	assert(0);
	return SectFound::no;
}

void OpEdgeIntersect::Split(std::vector<OpEdge>& parts, DoSplit doSplit) {
	std::vector<OpEdge> splits;
	for (auto& edge : parts) {
		if (EdgeSplit::unsplittable == edge.doSplit)
			continue;
		if (EdgeSplit::no == edge.doSplit && DoSplit::marked == doSplit)
			continue;
		// constructor invoked by emplace back does all the heavy lifting (initialization)
		splits.emplace_back(&edge, edge.center, NewEdge::isLeft  OP_DEBUG_PARAMS(EdgeMaker::split));
		splits.emplace_back(&edge, edge.center, NewEdge::isRight  OP_DEBUG_PARAMS(EdgeMaker::split));
	}
	parts.swap(splits);
}

