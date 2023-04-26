#include "OpContour.h"
#include "OpEdgeIntersect.h"
#include "OpEdges.h"
#include "OpSegment.h"

OpEdges::OpEdges(OpContours& contours, EdgesToSort edgesToSort) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			if (pointType == segment.c.type)
				continue;
			for (auto& edge : segment.edges) {
				addEdge(&edge, edgesToSort);
			}
		}
	}
	sort(edgesToSort);
}

OpEdges::OpEdges(OpEdge* sEdge, OpEdge* oEdge) {
	addEdge(sEdge, EdgesToSort::byCenter);
	addEdge(oEdge, EdgesToSort::byCenter);
#if OP_DEBUG_IMAGE
	OpDebugImage::add(sEdge);
	OpDebugImage::add(oEdge);
#endif
}

void OpEdges::addEdge(OpEdge* edge, EdgesToSort edgesToSort) {
	if (!edge->winding.visible())	// skip edges no longer visible
		return;
	if (EdgesToSort::byBox == edgesToSort || edge->ptBounds.height())
		inX.push_back(edge);
	if (EdgesToSort::byCenter == edgesToSort && edge->ptBounds.width())
		inY.push_back(edge);
}

IntersectResult OpEdges::CoincidentCheck(OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
		OpSegment* segment, OpSegment* oppSegment) {
	OpVector abDiff = aPtT.pt - bPtT.pt;
	XyChoice xyChoice = fabsf(abDiff.dx) < fabsf(abDiff.dy) ? XyChoice::inY : XyChoice::inX;
	float A = aPtT.pt.choice(xyChoice);
	float B = bPtT.pt.choice(xyChoice);
	assert(A != B);
	float C = cPtT.pt.choice(xyChoice);
	float D = dPtT.pt.choice(xyChoice);
	assert(C != D);
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
	assert(CinAB || DinAB);
	float AorB = AinCD ? A : B;
	OpPtT ptTAorB = AinCD ? aPtT : bPtT;
	float CorD = CinAB ? C : D;
	OpPtT ptTCorD = CinAB ? cPtT : dPtT;
	if (AorB == CorD) {
		OpIntersection* sect = segment->addIntersection(ptTAorB  OP_DEBUG_PARAMS(IntersectMaker::addCoincidentCheck1));
		OpIntersection* oSect = oppSegment->addIntersection(ptTCorD  OP_DEBUG_PARAMS(IntersectMaker::addCoincidentCheck2));
		sect->pair(oSect);
		return IntersectResult::yes;
	}
	// pass a mix of seg and opp; construct one t for each
	int coinID = segment->coinID(flipped);
	AddMix(xyChoice, ptTAorB, flipped, cPtT, dPtT, segment, oppSegment, coinID);
	AddMix(xyChoice, ptTCorD, flipped, aPtT, bPtT, oppSegment, segment, coinID);
	return IntersectResult::yes;
}

// this edge has points A, B; opp edge has points C, D
// adds 0: no intersection; 1: end point only; 2: partial or full coincidence
IntersectResult OpEdges::CoincidentCheck(const OpEdge& edge, const OpEdge& opp) {
	return OpEdges::CoincidentCheck(edge.start, edge.end, opp.start, opp.end,
			const_cast<OpSegment*>(edge.segment), const_cast<OpSegment*>(opp.segment));
}

void OpEdges::AddMix(XyChoice xyChoice, OpPtT ptTAorB, bool flipped,
		OpPtT cPtT, OpPtT dPtT, OpSegment* segment, OpSegment* oppSegment, int coinID) {
	float eStart = ptTAorB.pt.choice(xyChoice);
	if (flipped)
		std::swap(cPtT, dPtT);
	float oStart = cPtT.pt.choice(xyChoice);
	float oEnd = dPtT.pt.choice(xyChoice);
	float oTRange = dPtT.t - cPtT.t;
	OpPtT oCoinStart { ptTAorB.pt, cPtT.t + (eStart - oStart) / (oEnd - oStart) * oTRange };
	assert(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
	OpIntersection* sect = segment->addIntersection(ptTAorB, coinID  OP_DEBUG_PARAMS(IntersectMaker::addMix1));
	OpIntersection* oSect = oppSegment->addIntersection(oCoinStart, coinID  OP_DEBUG_PARAMS(IntersectMaker::addMix2));
	sect->pair(oSect);
}

IntersectResult OpEdges::AddPair(XyChoice xyChoice, OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
		bool flipped, OpSegment* segment, OpSegment* oppSegment) {
	float eStart = aPtT.pt.choice(xyChoice);
	float eEnd = bPtT.pt.choice(xyChoice);
	if (flipped)
		std::swap(cPtT, dPtT);
	float oStart = cPtT.pt.choice(xyChoice);
	float oEnd = dPtT.pt.choice(xyChoice);
	float oTRange = dPtT.t - cPtT.t;
	OpPtT oCoinStart { aPtT.pt, cPtT.t + (eStart - oStart) / (oEnd - oStart) * oTRange };
	OpPtT oCoinEnd { bPtT.pt, cPtT.t + (eEnd - oStart) / (oEnd - oStart) * oTRange };
	assert(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
	assert(OpMath::Between(cPtT.t, oCoinEnd.t, dPtT.t));
	int coinID = segment->coinID(flipped);
	OpIntersection* sect1 = segment->addIntersection(aPtT, coinID  OP_DEBUG_PARAMS(IntersectMaker::addPair1));
	OpIntersection* sect2 = segment->addIntersection(bPtT, coinID  OP_DEBUG_PARAMS(IntersectMaker::addPair2));
	OpIntersection* oSect1 = oppSegment->addIntersection(oCoinStart, coinID  OP_DEBUG_PARAMS(IntersectMaker::addPair3));
	OpIntersection* oSect2 = oppSegment->addIntersection(oCoinEnd, coinID  OP_DEBUG_PARAMS(IntersectMaker::addPair4));
	sect1->pair(oSect1);
	sect2->pair(oSect2);
	return IntersectResult::yes;
}

IntersectResult OpEdges::AddIntersection(OpEdge& opp, const OpEdge& edge) {
	auto alreadyContains = [](const std::vector<OpIntersection*>& sects, const OpPtT& edgePtT,
			const OpSegment* segment) {
		for (auto sectPtr : sects) {
			const OpIntersection& sect = *sectPtr;
			if (segment == sect.segment && (edgePtT.pt == sect.ptT.pt || edgePtT.t == sect.ptT.t))
				return true;
		}
		return false;
	};
	if (opp.segment == edge.segment && edge.ptBounds.touches(opp.ptBounds)
			&& (opp.start == edge.end || opp.end == edge.start)) {
		return IntersectResult::no;
	}
//	OpDebugBreak(&edge, 1, 2 == opp.id);
	OpRoots septs;
	assert(edge.isLine_impl);
	std::array<OpPoint, 2> edgePts { edge.start.pt, edge.end.pt };
	const OpCurve& oppCurve = opp.setCurve();
	septs.count = oppCurve.rayIntersect(edgePts, septs.roots);
//	assert(septs.count < 3);	// while it should never be 3, it's OK to accept those intersections
	if (!septs.count)
		return IntersectResult::no;
	// Note that coincident check does not receive intercepts as a parameter; in fact, the intercepts
	// were not calculated (the roots are uninitialized). This is because coincident check will 
	// compute the actual coincident start and end without the roots introducing error.
	if (opp.setLinear() && 2 == septs.count)
		return CoincidentCheck(edge, opp);
	int foundCount = 0;
	for (unsigned index = 0; index < septs.count; ++index) {
		float oppT = OpMath::Interp(opp.start.t, opp.end.t, septs.get(index));
		OpPtT oppPtT { oppCurve.ptAtT(septs.get(index)), oppT };
		float edgeT = edge.findPtT(oppPtT.pt);
		if (OpMath::Between(0, edgeT, 1)) {
            // pin point to both bounds, but only if it is on edge
			OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
			OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
            oSegment->tightBounds.pin(&oppPtT.pt);
            eSegment->tightBounds.pin(&oppPtT.pt);
			OpPtT edgePtT { oppPtT.pt, OpMath::Interp(edge.start.t, edge.end.t, edgeT) };
			if (alreadyContains(eSegment->intersections, edgePtT, oSegment))
				assert(alreadyContains(oSegment->intersections, oppPtT, eSegment));
			else {
				OpIntersection* sect = eSegment->addIntersection(edgePtT  OP_DEBUG_PARAMS(IntersectMaker::addIntersection1));
				OpIntersection* oSect = oSegment->addIntersection(oppPtT  OP_DEBUG_PARAMS(IntersectMaker::addIntersection2));
				sect->pair(oSect);
			}
			++foundCount;
		}
	}
	//	assert(foundCount < 2);
	return foundCount ? IntersectResult::yes : IntersectResult::no;
}

FoundIntersections OpEdges::findIntersections() {
#if OP_DEBUG_COMPARE
	OpDebugCompare debugCompare("innerFindSect");
#endif
	for (auto edgeIter = inX.begin(); edgeIter != inX.end(); ++edgeIter) {
		OpEdge* edge = const_cast<OpEdge*>(*edgeIter);
		for (auto oppIter = edgeIter + 1; oppIter != inX.end(); ++oppIter) {
			OpEdge* opp = const_cast<OpEdge*>(*oppIter);
#if OP_DEBUG_COMPARE
			debugCompare.edges(edge, opp);
#endif
			if (edge->ptBounds.right < opp->ptBounds.left)
				break;
			if (!edge->ptBounds.intersects(opp->ptBounds))
				continue;
			if (edge->segment == opp->segment && cubicType != edge->segment->c.type)	
				continue;  // non-cubic handled in opsegment makeedges
			// for line-curve intersection we can directly intersect
			if (lineType != edge->segment->c.type && edge->setLinear()) {
				(void)AddIntersection(*opp, *edge);
				continue;
			} else if (lineType != opp->segment->c.type && opp->setLinear()) {
				(void)AddIntersection(*edge, *opp);
				continue;
			}
			// if the curves' bounds share only a single intersection edge, check endpoints only
			if (edge->ptBounds.touches(opp->ptBounds)) {
				OpSegment* eSegment = const_cast<OpSegment*>(edge->segment);
				OpSegment* oSegment = const_cast<OpSegment*>(opp->segment);
				for (auto& edgePtT : { edge->start, edge->end }) {
					for (auto& oppPtT : { opp->start, opp->end }) {
						// end matching end was handled earlier in op segments find intersection
						if (edgePtT.pt != oppPtT.pt)
							continue;
						if (eSegment == oSegment && edgePtT.t == oppPtT.t)
							continue;
						if (OpMath::IsInt(edgePtT.t) && OpMath::IsInt(oppPtT.t))
							continue;
						if (eSegment->containsSect(edgePtT.t, oSegment)
								&& oSegment->containsSect(oppPtT.t, eSegment))
							continue;
						OpIntersection* sect = eSegment->addIntersection(edgePtT
								OP_DEBUG_PARAMS(IntersectMaker::findIntersections1));
						OpIntersection* oSect = oSegment->addIntersection(oppPtT
								OP_DEBUG_PARAMS(IntersectMaker::findIntersections2));
						sect->pair(oSect);
					}
				}
				continue;
			}
			if (lineType == edge->segment->c.type || lineType == opp->segment->c.type)
				continue;
			OpEdgeIntersect opEdgeIntersect(edge, opp);
			SectFound result = opEdgeIntersect.divideAndConquer();
			if (SectFound::fail == result)
				return FoundIntersections::fail;
		}
	}
#if OP_DEBUG_COMPARE
	debugCompare.close();
#endif
	return FoundIntersections::yes;
}

struct EdgeDistance {
	EdgeDistance(OpEdge* e, float d, float _t)
		: edge(e)
		, distance(d)
		, t(_t) {
	}

	OpEdge* edge;
	float distance;
	float t;
};

// note: sorts from high to low
static bool compareDistance(const EdgeDistance& s1, const EdgeDistance& s2) {
	return s1.distance > s2.distance;
}

// at some point, do some math or rigorous testing to figure out how extreme this can be
// for now, keep making it smaller until it breaks
#define WINDING_NORMAL_LIMIT  0.001 // !!! no idea what this should be

// if horizontal axis, look at rect top/bottom
void OpEdges::setSumChain(std::vector <OpEdge*>& inArray, size_t inIndex, Axis axis) {
	// see if normal at center point is in direction of ray
	OpEdge* edge = inArray[inIndex];
	assert(edge->winding.visible());
	const OpSegment* edgeSeg = edge->segment;
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	float NxR = edgeSeg->c.tangent(edge->center.t).normalize().cross(ray);
	if (fabs(NxR) < WINDING_NORMAL_LIMIT) {
		if (ray.dy) {
			// This is the second pass. If the edge is tiny, it is reasonable to ignore it.
			edge->winding.zero(ZeroReason::failWinding);
			// !!! strict 'is tiny' check -- figure out what this should be
			if (edge->end.t - edge->start.t <= OpEpsilon)
				edge->unsortable = true;
			else 
				edge->fail = EdgeFail::winding;
		}
		return;
	}
	// intersect normal with every edge in the direction of ray until we run out 
	Axis perpendicular = !axis;
	OpVector backRay = -ray;
	float center = edge->center.pt.choice(perpendicular);
	float normal = edge->center.pt.choice(axis);
	// mark all possible edges
	inIndex += 1;
	for (; inIndex < inArray.size(); ++inIndex) {
		OpEdge* advance = inArray[inIndex];
		if (advance->ptBounds.ltChoice(perpendicular) > center)
			break;
	}
	std::vector<EdgeDistance> distance;
	distance.emplace_back(edge, 0, edge->center.t);
	// start at edge with left equal to or left of center
	while (inIndex != 0) {
		OpEdge* test = inArray[--inIndex];
		if (test == edge)
			continue;
		if (!test->winding.visible())
			continue;
		if (test->ptBounds.ltChoice(axis) > normal)
			continue;
		if (test->ptBounds.rbChoice(axis) < normal)
			continue;
		OpPtT ptT = edge->findRayIntercept(test, 0, 1, axis, backRay);
		if (!OpMath::IsNaN(ptT.t))
			distance.emplace_back(test, center - ptT.pt.choice(perpendicular), ptT.t);
	}
	std::sort(distance.begin(), distance.end(), compareDistance);	// sorts from high to low
	EdgeDistance* last = &distance.front();
	// leftmost edge may have no prior, no next; if the right edge has no effect
	while (last->edge != edge) {
		EdgeDistance* next = last + 1;
		OpEdge* lastEdge = last->edge;
		OpEdge* nextEdge = next->edge;
		float delta = last->distance - next->distance;
		if (!nextEdge->priorSum) {
			nextEdge->setPriorSum(lastEdge);
			nextEdge->priorAxis = axis;
			nextEdge->priorT = last->t;
			nextEdge->unsortable |= delta <= OpEpsilon;
		}
		last = next;
	}
}

FoundWindings OpEdges::setWindings() {
	OP_DEBUG_CODE(OpDebugIntersectSave save(OpDebugIntersect::edge));
	// test sum chain for correctness; recompute if prior or next are inconsistent
	for (Axis axis : { Axis::horizontal, Axis::vertical }) {
		OpVector backRay = Axis::horizontal == axis ? OpVector{ -1, 0 } : OpVector{ 0, -1 };
		std::vector<OpEdge*>& edges = Axis::horizontal == axis ? inX : inY;
		for (size_t index = 0; index < edges.size(); ++index) {
			setSumChain(edges, index, axis);
		}
		for (OpEdge* edge : edges) {
			edge->active_impl = edge->winding.visible();
		}
		std::vector<OpEdge*> loops;
		for (OpEdge* edge : edges) {
			if (!edge->isActive())
				continue;
			OpEdge* test = edge->hasLoop(WhichLoop::prior, EdgeLoop::sum, LeadingLoop::inLoop);
			if (test) {
				if (48 == test->id && 47 == edge->id)
					edge->dumpSum();
				loops.push_back(test);
			}
			do {
				edge->clearActive();
				edge = edge->priorSum;
			} while (edge && edge->isActive());
		}
		// edges with loops have sum links which are wrong
		//	  either the sum links should be coincident, or should diverge
		Axis perpendicular = !axis;
		for (OpEdge* loopStart : loops) {
			float upperLimit = -OpInfinity;
			float lowerLimit = OpInfinity;
			OpEdge* edge = loopStart;
			do {
				upperLimit = std::max(upperLimit, edge->start.pt.choice(perpendicular));
				lowerLimit = std::min(lowerLimit, edge->start.pt.choice(perpendicular));
				upperLimit = std::max(upperLimit, edge->end.pt.choice(perpendicular));
				lowerLimit = std::min(lowerLimit, edge->end.pt.choice(perpendicular));
				edge = edge->priorSum;
			} while (edge != loopStart);
			enum class Limit {
				match,
				shorter,
				diverged
			};
			struct LoopInfo {
				LoopInfo(OpEdge* edge)
					: start(Limit::match)  // assume the edge is as long as the limits
					, end(Limit::match) {
					connections.push_back(edge);
					reversed.push_back(false);
				}

				bool fullMatch() const {
					return Limit::match == start && Limit::match == end;
				}
				std::vector<OpEdge*> connections;
				std::vector<bool> reversed;
				Limit start;
				Limit end;
			};
			std::vector<LoopInfo> loopInfo;
			// (try to) lengthen edges that are too short
			// if the lengthened edge matches the longest edge, mark both as coincident
			// if not, see if they diverge enough to establish a sort order
			// if they don't match, and don't diverge enough, try lengthening again
			do {
				loopInfo.emplace_back(edge);
				LoopInfo& info = loopInfo.back();
				float testStartXY = edge->start.pt.choice(perpendicular);
				assert(OpMath::Between(lowerLimit, testStartXY, upperLimit));
				if (lowerLimit != testStartXY && testStartXY != upperLimit) {
					info.start = Limit::shorter;
					OpEdge* test = edge;
					EdgeMatch testMatch = EdgeMatch::start;
					do {
						OpPtT testMatchPtT = test->ptT(testMatch);
						OpSegment* testSegment = const_cast<OpSegment*>(test->segment);
						OpDebugBreak(test, 6877, true);
						OpEdge* connector = testSegment->visibleAdjacent(test, testMatchPtT);
						if (!connector) {
							assert(0);	// code incomplete
							break;
						}
						info.connections.push_back(connector);
						// if connector also loops, try the next connector
						OpPoint connectorEnd = connector->ptT(Opposite(testMatch)).pt;
						bool reversed = testMatchPtT.pt != connectorEnd;
						info.reversed.push_back(reversed);
						assert(testMatchPtT.pt == (reversed ? connector->ptT(testMatch).pt : connectorEnd));
						assert(test->winding == (reversed ? -connector->winding : connector->winding));
						OpPoint limitPt = reversed ? connectorEnd : connector->ptT(testMatch).pt;
						float connectorLimit = limitPt.choice(perpendicular);
						if (upperLimit == connectorLimit || lowerLimit == connectorLimit)
							break;	// if connector matches, edge + connector may make a coincidence part
						if (connector->isLoop(WhichLoop::prior, EdgeLoop::sum, LeadingLoop::inLoop)) {
							test = connector;
							if (reversed)
								testMatch = Opposite(testMatch);
							continue;
						}
						// project ray between short length and end of added dual length
						OpPtT ptT = connector->findRayIntercept(edge, testStartXY, 
								connectorLimit, axis, backRay);
						// detect if edges diverge
						assert(!OpMath::IsNaN(ptT.t));	// just here to suppress warning
						assert(0); // incomplete
					} while (true);
					assert(0);	// incomplete
				}
				float testEndXY = edge->end.pt.choice(perpendicular);
				assert(OpMath::Between(lowerLimit, testEndXY, upperLimit));
				if (lowerLimit != testEndXY && testEndXY != upperLimit) {
					info.end = Limit::shorter;
					OpEdge* test = edge;
					do {
						OpEdge* connector = test->visibleAdjacent(EdgeMatch::end);
						if (connector)
							assert(0);	// incomplete
						assert(0);	// incomplete
					} while (true);
					assert(0);	// incomplete
				}
				edge = edge->priorSum;
			} while (edge != loopStart);
			// each loop edge end is either match, shorter, or diverged
			// if all are match, mark pair as coincident
			bool allMatch = true;
			for (auto& info : loopInfo) {
				allMatch &= info.fullMatch();
			}
			if (allMatch) {
				for (auto& info : loopInfo) {
					assert(1 == info.connections.size());	// code incomplete
					OpEdge* test = info.connections[0];
					if (!test->winding.visible())
						continue;
					LoopInfo* cInfo = &info;
					while (cInfo != &loopInfo.back()) {
						++cInfo;
						// treat pair as coincident and apply all of one winding to the other
						assert(1 == cInfo->connections.size());
						OpEdge* opp = cInfo->connections[0];
						test->winding.move(opp->winding, test->segment->contour->contours,
								cInfo);
					}
					// recompute test prior; assert that it is does not make a loop
					for (size_t index = 0; index < edges.size(); ++index) {
						OpEdge* test2 = edges[index];
						if (test2->winding.visible() && test2->priorEdge
								&& !test2->priorEdge->winding.visible()) {
							setSumChain(edges, index, axis);
							assert(!test2->isLoop(WhichLoop::prior, EdgeLoop::sum, LeadingLoop::inLoop));
						}
					}
				}
#if 0	// Replace below with (possibly) remove looping from edges and recompute sum chains that
		// point to these edges.
				// break loop, and point to next actual edge or null if none
				for (size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
					if (edge != edges[edgeIndex])
						continue;
					(SumLoop::prior == loopType ? edge->priorSum : edge->nextSum) = nullptr;
					setSumChain(edges, edgeIndex, axis);
					break;
				}
#endif
			} else
				assert(0); // code incomplete
			continue;
		}
	}
	if ((0)) {
		bool loops;
		do {
			loops = false;
			// seen prior, seen next are set so edge is checked only once for looping
	//		start here;
			// this is unsustainable because the edges making up the loops are not the same lengths
			// so it isn't possible to combine windings and set one winding to zero
			// since this loop operates on an index to an edge pointer, it would be possible to
			// split edges as was done in the coincident case
			// an alternative is to mark the edge as unsortable and then everywhere else use that
			// info to not rely on sum winding. Instead, compute the winding contribution for as 
			// many edges as are unsortable until a sortable edge with sum winding is found.
			// because of the 'edges are different lengths' condition, the edge may or may not be
			// part of the unsortable loop -- which sure seems like another reason why the edges
			// triggering this must be split...

			// another tack
			// any pair of edges that can't be ordered by ray are either coincident or divergent
			// if they are different lengths, then subsequent connected edges can be considered
			// if the connected edges eventually share a point, the edges are likely coincident
			// if the connected edges diverge, then all edges in that list can receive the resolved
			//   winding
			// this is viable for a pair of edges in a loop, but what if there are three or more?
			// 
			assert(0); // incomplete
			OpEdge* edge = nullptr;
			int index = 0;
			if (edge->isLoop(WhichLoop::prior, EdgeLoop::sum, LeadingLoop::inLoop))
				edge->markFailPrior(inX, Axis::horizontal);
			if (loops && edge->winding.visible())
				setSumChain(inX, index, Axis::horizontal);
		} while (loops);
	}
	for (auto& edge : inX) {
		if (edge->winding.visible() && edge->sum.unset()) {
			OP_DEBUG_CODE(int debugWindingLimiter = 0);
			edge->findWinding(Axis::horizontal  OP_DEBUG_PARAMS(&debugWindingLimiter));
		}
	}
	for (auto& edge : inX)
		if (edge->winding.visible() && edge->sum.unset() && !edge->unsortable)
			return FoundWindings::fail;
	for (size_t index = 0; index < inY.size(); ++index) {
		OpEdge* edge = inY[index];
		if (edge->winding.visible() && edge->sum.unset())
			setSumChain(inY, index, Axis::vertical);
	}
	if ((0)) {	// see comment for analogous x loop above
		for (size_t index = 0; index < inY.size(); ++index) {
			OpEdge* edge = inY[index];
			if (!edge->winding.visible())
				continue;
			if (!edge->sum.unset())
				continue;
			bool loops;
			do {
				loops = false;
				if (edge->isLoop(WhichLoop::prior, EdgeLoop::sum, LeadingLoop::inLoop))
					edge->markFailPrior(inY, Axis::vertical);
				if (loops && edge->winding.visible())
					setSumChain(inY, index, Axis::vertical);
			} while (loops);
		}
	}
	for (auto& edge : inY) {
		if (edge->winding.visible() && edge->sum.unset()) {
			OP_DEBUG_CODE(int debugWindingLimiter = 0);
			edge->findWinding(Axis::vertical  OP_DEBUG_PARAMS(&debugWindingLimiter));
		}
	}
	for (auto& edge : inY)
		if (edge->winding.visible() && edge->sum.unset() && !edge->unsortable)
			return FoundWindings::fail;
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

void OpEdges::sort(EdgesToSort sortBy) {
	if (EdgesToSort::byBox == sortBy) {
		std::sort(inX.begin(), inX.end(), compareXBox);
		return;
	}
	assert(EdgesToSort::byCenter == sortBy);
	std::sort(inX.begin(), inX.end(), compareXCenter);
	std::sort(inY.begin(), inY.end(), compareYCenter);
}
