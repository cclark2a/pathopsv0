#include "OpContour.h"
#include "OpEdgeIntersect.h"
#include "OpEdges.h"
#include "OpSegment.h"

start here;
// add a custom iterator that looks at both curves and lines
struct OpSectEdgeIter {
	OpSectEdgeIter(const std::vector<OpEdge>& c, const std::vector<OpEdge>& l, bool start)
		: curves(c)
		, lines(l) {
		if (start) {
			localIndex = 0; // iterator start index
			return;
		}
		localIndex = curves.size() + lines.size(); // iterator end index
	}

	bool operator!=(OpSectEdgeIter rhs) { 
		return localIndex != rhs.localIndex; 
	}

    const OpEdge& operator*() {
		size_t entries = curves.size();
		if (localIndex < entries)
			return curves[localIndex];
		int index = localIndex - entries;
		if (index < lines.size())
			return lines[index];
		OpDebugOut("iterator out of bounds! localIndex: " + STR(localIndex) + "\n");
		assert(0);
		return *(const OpEdge*) nullptr; 
	}

    void operator++() { 
		++localIndex;
	}

	const std::vector<OpEdge>& curves;
	const std::vector<OpEdge>& lines;
	size_t localIndex;
};

struct OpSectEdgeIterator {
	OpSectEdgeIterator(const std::vector<OpEdge>& c, const std::vector<OpEdge>& l)
		: curves(c)
		, lines(l) {
	}
	OpSectEdgeIter begin() { return OpSectEdgeIter(curves, lines, true); }
	OpSectEdgeIter end() { return OpSectEdgeIter(curves, lines, false); }
	bool empty() { return !(begin() != end()); }

	const std::vector<OpEdge>& curves;
	const std::vector<OpEdge>& lines;
};


static OpEdge findEdgesTRange(std::vector<OpEdge>& parts, const OpSegment* oppSegment) {
	OpSectEdgeIterator sectEdgeIterator(curves, lines);
	OpPtT start( OpPoint(), OpInfinity );
	OpPtT end( OpPoint(), -OpInfinity );
	for (const auto& edge : parts) {
		edge.dumpDetail();
		if (EdgeSplit::yes != edge.doSplit)
			continue;
		if (start.t > edge.start.t)
			start = edge.start;
		if (end.t < edge.end.t)
			end = edge.end;
	}
	// extend edge, if possible, by considering found intersections
	// assume that if part of the pair is coincident, there can be only one isolated common point
	// !!! may need to check if single isolated point is near or is on edge which is a line
	OpEdge result = parts[0];
	result.start = start;
	result.end = end;
	const OpSegment* segment = result.segment;
	OpPtT min { OpPoint(), OpInfinity };
	OpPtT max { OpPoint(), -OpInfinity };
	for (auto ceptPtr : segment->intersections) {
		const OpIntersection& cept = *ceptPtr;
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
	result.complete();
	return result;
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
	// if it is a simple noncoincident crossing, there may be only one t value (zero on the opposite)
	// probably detect that in the caller so we don't get this far if there is only one point
	// if there are two or more on one side and zero on the other, set use edge min / max appropriately
	if (edgeTs.size()) {
		edgeResult.start.t = *std::min_element(edgeTs.begin(), edgeTs.end());
		edgeResult.start.pt = segment->c.ptAtT(edgeResult.start.t);
		edgeResult.end.t = *std::max_element(edgeTs.begin(), edgeTs.end());
		edgeResult.end.pt = segment->c.ptAtT(edgeResult.end.t);
	}
	if (oppTs.size()) {
		oppResult.start.t = *std::min_element(oppTs.begin(), oppTs.end());
		oppResult.start.pt = oppSegment->c.ptAtT(oppResult.start.t);
		oppResult.end.t = *std::max_element(oppTs.begin(), oppTs.end());
		oppResult.end.pt = oppSegment->c.ptAtT(oppResult.end.t);
	}
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
	float edgeMin = edgeTs.size() ? edgeResult.start.pt.choice(edgeXY) : OpNaN;
	float oppMin = oppTs.size() ? oppResult.start.pt.choice(edgeXY) : OpNaN;
	float edgeMax = edgeTs.size() ? edgeResult.end.pt.choice(edgeXY) : OpNaN;
	float oppMax = oppTs.size() ? oppResult.end.pt.choice(edgeXY) : OpNaN;
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
	OpSegment* segment = const_cast<OpSegment*>(edgeCurves.front().segment);
	OpSegment* oppSegment = const_cast<OpSegment*>(oppCurves.front().segment);
	edgeResults.push_back(findEdgesTRange(edgeCurves, oppSegment));
	oppResults.push_back(findEdgesTRange(oppCurves, segment));
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

SectFound OpEdgeIntersect::addIntersections(OP_DEBUG_CODE(SectReason reason)) {
	if (!edgeTs.size() && !oppTs.size())
		return SectFound::no;
	// check to see if center t is equal to start or end (because delta t is so small)
	OpSegment* eSegment = const_cast<OpSegment*>(originalEdge->segment);
	OpSegment* oSegment = const_cast<OpSegment*>(originalOpp->segment);
	OpDebugBreak(originalEdge, 411);
	// pick the edge t or opp t 
	for (float edgeT : edgeTs) {
		OpPtT edgePtT { eSegment->c.ptAtT(edgeT), edgeT };
		OpIntersection* sect = eSegment->addIntersection(edgePtT  
				OP_DEBUG_PARAMS(SECT_MAKER(edgeT), reason, nullptr, originalEdge, originalOpp));
		edgePtT.pt.pin(originalOpp->ptBounds);
		OpPtT oppPtT { edgePtT.pt, oppositeT(eSegment, *originalOpp, edgePtT.pt  
				OP_DEBUG_PARAMS(originalEdge->id)) };
		OpIntersection* oSect = oSegment->addIntersection(oppPtT  
				OP_DEBUG_PARAMS(SECT_MAKER(edgeTOpp), reason, nullptr, originalEdge, originalOpp));
		sect->pair(oSect);
	} 
	for (float oppT : oppTs) {
		OpPtT oppPtT{ oSegment->c.ptAtT(oppT), oppT };
		OpIntersection* oSect = oSegment->addIntersection(oppPtT  
				OP_DEBUG_PARAMS(SECT_MAKER(oppT), reason, nullptr, originalEdge, originalOpp));
		oppPtT.pt.pin(originalEdge->ptBounds);
		OpPtT edgePtT{ oppPtT.pt, oppositeT(oSegment, *originalEdge, oppPtT.pt  
				OP_DEBUG_PARAMS(originalOpp->id)) };
		OpIntersection* sect = eSegment->addIntersection(edgePtT  
				OP_DEBUG_PARAMS(SECT_MAKER(oppTOpp), reason, nullptr, originalEdge, originalOpp));
		sect->pair(oSect);
	}
	return SectFound::intersects;
}


// this marks opp as splittable if edge is a curve
// this finds opp t if edge is a line
// runs twice: edge/opp, then opp/edge
// result is for all edges: no (intersections at all); fail; (something) split; line sect (no splits)
SectFound OpEdgeIntersect::CurvesIntersect(std::vector<OpEdge>& edgeParts,
		std::vector<OpEdge>& oppParts, std::vector<float>& oppTs) {
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
		SectFound oppResult = CurvesIntersect(edgeCurves, oppCurves, oppTs);
		if (SectFound::fail == oppResult)
			return oppResult;
		SectFound edgeResult = CurvesIntersect(oppCurves, edgeCurves, edgeTs);
		if (SectFound::fail == edgeResult)
			return edgeResult;
		if (SectFound::no == oppResult || SectFound::no == edgeResult) {
			if (!edgeTs.size() && !oppTs.size())
				return SectFound::no;
			if (1 == edgeTs.size() + oppTs.size())
				return addIntersections(OP_DEBUG_CODE(SectReason::divideAndConquer_oneT));
			// add intersections at extremes of returned t values
			return addCoincidence();
		}
		if (edgeCurves.size() >= maxSplits || oppCurves.size() >= maxSplits)
			return addCurveCoincidence();
		if (edgeTs.size() >= maxSplits || oppTs.size() >= maxSplits)
			return addCoincidence();
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
					return addIntersections(OP_DEBUG_CODE(SectReason::divideAndConquer_noEdgeToSplit));
				keepGoing = true;
			}
			if (splitType == DoSplit::marked || edgeCurves.size() >= oppCurves.size() * 2) {
				if (!Split(oppCurves, oppLines, splitType))
					return addIntersections(OP_DEBUG_CODE(SectReason::divideAndConquer_noOppToSplit));
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
