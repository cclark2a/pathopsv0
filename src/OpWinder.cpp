// (c) 2023, Cary Clark cclark2@gmail.com
#include <cmath>
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include "PathOps.h"

bool RayTargets::addContainer(Axis axis, OpContour* container, OpRect& bounds) {
	if (match(container))
		return false;
	t.push_back({ container, bounds });
	if (edges)
		return true;
	--index; 
	set(axis);
	return true;
}

// only include contours that intersect their parents and the chain bounds
void RayTargets::build(OpEdge* edge) {
	// construct rectangle from edge bounds and ray to edge's sects' bounds 
	chainBounds = edge->bounds;
	OpContour* contour = edge->segment->contour;
	const OpRect& overlapBounds = contour->overlapOwner->overlapBounds;
	if (Axis::horizontal == edge->ray.axis)
		chainBounds.left = overlapBounds.left;
	else
		chainBounds.top = overlapBounds.top;
	// build inX/Y from sects intersecting chain bounds
	std::vector<RayTarget> openTargets;
	t.clear();
	for (OpContour* member : contour->members()) {
		OpPointBounds sectBounds(member->bounds.intersect(chainBounds));
		if (sectBounds.isEmpty())
			continue;
		t.push_back({ member, sectBounds });
	}
	for (RayTarget& target : openTargets) {
		if (!match(target.contour))
			t.push_back(target);
	}
}

void RayTargets::reset(Axis axis) {
	index = SIZE_MAX;
	set(axis); 
}

void RayTargets::set(Axis axis) {
	edgeIndex = SIZE_MAX;
	for (;;) {
		edges = nullptr;
		if (++index >= t.size())
			return;
		RayTarget& target = t[index];
		OpContour* contour = target.contour;
		if (!contour)
			continue;
		edges = Axis::horizontal == axis ? &contour->inX : &contour->inY;	
		if (!edges->empty())
			return;
	}
}

// walk targets (backwards?) to find edges that contribute to the ray
// currently unsortable just gives up, but still keeps sorted rays found earlier
//   walking each contour individually will change that behavior
//   does that behavior need to be preserved?
//   it seems odd -- but could be reproduced if needed (with effort)
//   maybe put that off until it is needed...
// !!! caller assumes this looks at chainBounds, but it does not...
OpEdge* RayTargets::next(Axis axis, float homeCept) {
	Axis uppity = !axis;
	OpRect* bounds;
	while (edges) {
		// advance to furthest that could influence the sum winding of this edge
		if (edgeIndex >= edges->size()) {  
			edgeIndex = 0;
			do {
				bounds = &(*edges)[edgeIndex]->bounds;
			} while (bounds->ltChoice(uppity) <= homeCept && ++edgeIndex < edges->size());
			if (0 == edgeIndex--) {
				set(axis);
				continue;
			}
		}
		OpEdge* result = (*edges)[edgeIndex];
		bounds = &result->bounds;
		OP_ASSERT(bounds->ltChoice(uppity) <= homeCept);
		if (0 == edgeIndex--)
			set(axis);
		if (bounds->rbChoice(uppity) >= chainBounds.ltChoice(uppity))
			return result;
	}
	return nullptr;
}

bool RayTargets::match(OpContour* contour) const {
	return std::any_of(t.begin(), t.end(), [contour](const RayTarget& rayTarget) 
			{ return rayTarget.contour == contour; } );
}

// assumes home is first
// !!! consolidate only if edge and existing are not pals?
// !!! what if edge ray axis is not sect ray axis?
// over is set if xy is near or greater than home cept
void SectRay::addDistance(OpEdge* addEdge, float xy, float root, bool reversed
		OP_DEBUG_PARAMS(OpEdge* debugParent)) {
	distances.emplace_back(addEdge, xy, root);  // add edge to dist
	if (reversed)
		distances.back().reversed = true;
}

// recursively add contours containing edges that can affect accumulated winding
bool SectRay::addContainers(OpEdge* addEdge, OpEdge* home) {
	auto addOne = [home, addEdge](OpContour* container) {
		if (home->segment->contour == container)
			return false;
		OpRect& cBounds = container->bounds;
		// r-reference avoids struct copy
		OpRect&& bounds = Axis::horizontal == home->ray.axis  // area in contour between addEdge and home
			? OpRect(std::max(cBounds.left, addEdge->bounds.right), cBounds.top,
					std::min(cBounds.right, home->bounds.left), cBounds.bottom)
			: OpRect(cBounds.left, std::max(container->bounds.top, addEdge->bounds.bottom),
					cBounds.right, std::min(container->bounds.bottom, home->bounds.top));
		if (bounds.isEmpty())
			return false;
//		if (std::none_of(containers.begin(), containers.end(), [container](OpContour* test) {
//				return container == test; }))
//			containers.push_back(container);
		return home->ray.targets.addContainer(home->ray.axis, container, bounds);
	};
	auto addContour = [addOne](OpContour* contour) {
		bool added = addOne(contour);
		for (OpContour* merge : contour->merges)
			added |= addOne(merge);
		return added;
	};
	// if added edge depends on contour not in winder target, add it
	OpSegment* segment = addEdge->segment;
	bool added = addContour(segment->contour);
#if 0  // unnecessary?
	for (OpContour* contour : segment->coinContours) {
		added |= addContour(contour);
	}
#endif
	return added;
}

struct RayWalk {
	RayWalk(std::vector<Distance>* d)
		: dists(d) {
		index = (int) dists->size() - 2;  // skip home edge
	}

	std::vector<Distance>* dists;
	int index;
};

// testLine6587
// !!! 77 has 86 as dependent; 86 points to 72->83->69->75 ; add edge 84 points to 75
//     when we add 84, 86 is no longer dependent; but since 84->75, and 86..->75, 
//     84 should now be marked dependent
// walk dependence trees backwards for ray and add; return true if they have common root
bool SectRay::checkAdd(OpEdge* toAdd) {
	auto next = [](RayWalk& walk) {
		if (!walk.dists)
			return (Distance*) nullptr;
		Distance* result = &(*walk.dists)[walk.index];
		if (!walk.index) {
			if (!result->dependent)
				walk.dists = nullptr;
			else {
				OP_ASSERT(result->edge->ray.sorted);
				walk.dists = &result->edge->ray.distances;
				OP_ASSERT(!walk.dists->empty());
				walk.index = walk.dists->size() - 1; // skip last
			}
		}
		--walk.index;
		return result;
	};
	OP_ASSERT(!distances.empty());
	RayWalk sectDists(&distances);
	if (sectDists.index < 0)
		return false;
	RayWalk addDists(&toAdd->ray.distances);
	if (addDists.index < 0)
		return false;
	Distance* dist = next(sectDists);
	Distance* add = next(addDists);
	while (dist && add) {
		if (dist->edge == add->edge)
			return true;
		float distCept = dist->cept;
		if (distCept >= add->cept)
			dist = next(sectDists);
		if (distCept <= add->cept)
			add = next(addDists);
	}
	return false;
}

// this catches unsectables by keeping track of edges that are found to be adjacent
// each time a ray is cast. If the edge is seen to the left during one ray cast, and to the right
// on another, it is marked as an unsectable pair.
// !!! There may be unsectable pairs with other edges between. Wait for that before coding.
// !!! This is now detected when rays are cast. Change this temporarily to assert that it is not
//     needed
void SectRay::addPals(OpEdge* home) {
	OP_ASSERT(this == &home->ray);
	if (distances.empty())
		return;
	auto matchCept = [home](Distance* test) {
		home->addPal(test);
		if (const Distance* homeDist = test->edge->ray.find(home)) {
			test->edge->addPal(homeDist->edge, 0, homeDist->reversed);
//			OP_DEBUG_CODE(Distance* testDist = test->edge->ray.find(test->edge));
// !!! this asserts if there are three or more pals
// consider writing more complex test to detect if edge between pals is not a pal
//			OP_ASSERT(abs(homeDist - testDist) == 1);
		}
	};
	Distance* homeDist = find(home);
	Distance* test = homeDist;
	float margin = home->margin();
	float lowLimit = homeCept - home->margin();
	bool priorIsPal = false;
	while (test > &distances.front() && (--test)->cept >= lowLimit) {
		OP_ASSERT((test + 1)->cept >= test->cept);
		matchCept(test);
		priorIsPal = true;
	}
	test = homeDist;
	float highLimit = homeCept + margin;
	bool nextIsPal = false;
	while (test < &distances.back() && (++test)->cept <= highLimit) {
		OP_ASSERT((test - 1)->cept <= test->cept);
		matchCept(test);
		nextIsPal = true;
	}
	// if axes are different, and if y-axis edge is oriented nw/se (not ne/sw), reverse
	auto axesReversed = [home](OpEdge* test) {
		if (test->ray.axis == home->ray.axis)
			return false;
		OpEdge* vertical = Axis::vertical == test->ray.axis ? test : home;
		OpVector dxy = vertical->endPt() - vertical->startPt();
		if (!dxy.dy)
			return false;
		return dxy.dy > 0 ? dxy.dx > 0 : dxy.dx < 0;
	};
	// check next ray intersected edge if it hasn't been checked already
	// !!! stops at 1; don't know if we may need more than one
	// !!! thread_circles54530 failed only on laptop 
	auto addIfFlipped = [axesReversed, homeDist, home, this](DistEnd offset) {
		OP_ASSERT(DistEnd::back == offset || DistEnd::front == offset);
		if (homeDist == end(offset))
			return;
		OpEdge* edge = next(homeDist, offset)->edge;
		SectRay& ray = edge->ray;
		const Distance* dist = ray.find(edge);
		if (!dist)
			return;
		if (axesReversed(edge))
			offset = !offset;		
		if (dist == ray.end(offset))
			return;
		if (ray.next(dist, offset)->edge != home)
			return;
		// the copied edge pal does not have its reverse bit set since it is the home distance
		// compare the two to see if they are reversed with respect to each other
		Distance edgeRev = *dist;
		Distance homeRev = *homeDist;
		NormalDirection edgeNorm = edge->normalDirection(ray.axis, edgeRev.edgeInsideT);
		NormalDirection homeNorm = home->normalDirection(home->ray.axis, homeRev.edgeInsideT);
		if (edgeNorm != homeNorm) {
			edgeRev.reversed = true;
			homeRev.reversed = true;
		}
		home->addPal(&edgeRev);
		edge->addPal(&homeRev);
	};
	if (!priorIsPal)
		addIfFlipped(DistEnd::front);
	if (!nextIsPal)
		addIfFlipped(DistEnd::back);
}

// check ray edges for entry prior to edge
bool SectRay::canSetSum(const OpEdge* edge) const {
	return distances.size() && distances[0].edge != edge;
}

RayOrder SectRay::checkClose(const OpEdge* home) const {
	OP_ASSERT(sorted);
	// check to see if closest to home is too close
	const Distance* homeD = nullptr;
	const Distance* dist = &distances.back();
	do {
		if (home == dist->edge)
			homeD = dist;
	} while (--dist >= &distances.front() && !homeD);
	OP_ASSERT(homeD);
	OP_ASSERT(homeD->cept == homeCept);
	float margin = home->margin();
	if (dist >= &distances.front() && OpMath::Equal(dist->cept, homeCept, margin))
		return RayOrder::tooClose;
	dist = homeD + 1;
	if (dist <= &distances.back() && OpMath::Equal(dist->cept, homeCept, margin))
		return RayOrder::tooClose;
	return RayOrder::ok;
}

// check if pair of rays contributing to home are reversed from their own ray order (informative)
// check if distance adjacent to home is too close to safely determine the ray order (actionable)
RayOrder SectRay::checkOrder(const OpEdge* home) {
	RayOrder result = RayOrder::ok;
	for (Distance* dist = &distances.front(); (dist + 1)->edge != home; ++dist) {
		OpEdge* prior = dist->edge;
		OpEdge* last = (dist + 1)->edge;
		// pal should be set in time for this : testQuads26519435
		if (prior->isUnsectable() || last->isUnsectable() || last->isPal(prior))
			continue;
		if (last->ray.distances.size() > 1 && last->ray.axis == axis) {
			Distance* lastDist = last->ray.find(last);
			if (lastDist < &last->ray.distances.back() && (lastDist + 1)->edge == prior) {
				dist->rayOrder = RayOrder::unordered;
				(dist + 1)->rayOrder = RayOrder::unordered;
				OP_DEBUG_CODE(lastDist->edge->debugUnordered = true);
				OP_DEBUG_CODE((lastDist + 1)->edge->debugUnordered = true);
				result = RayOrder::unordered;
			}
		}
		if (prior->ray.distances.size() > 1 && prior->ray.axis == axis) {
			Distance* priorDist = prior->ray.find(prior);
			if (priorDist > &prior->ray.distances.front() && (priorDist - 1)->edge == last) {
				dist->rayOrder = RayOrder::unordered;
				(dist + 1)->rayOrder = RayOrder::unordered;
				OP_DEBUG_CODE(priorDist->edge->debugUnordered = true);
				OP_DEBUG_CODE((priorDist - 1)->edge->debugUnordered = true);
				result = RayOrder::unordered;
			}
		}
	}
	return result;
}

Distance* SectRay::find(const OpEdge* edge) {
	if (distances.empty())
		return nullptr;
	for (auto test = &distances.back(); test >= &distances.front(); --test) {
		if (test->edge == edge)
			return test;
	}
	return nullptr;
}

FindCept SectRay::findCept(OpEdge* edge, OpEdge* test) {
	if (test->bounds.ltChoice(axis) > normal)
		return FindCept::ok;
	if (test->bounds.rbChoice(axis) < normal)
		return FindCept::ok;
	if (test == edge)
		return FindCept::ok;
	if (Unsortable::none != test->isUnsortable && Unsortable::tooManyTries != test->isUnsortable)
		return FindCept::unsortable;
	if (test->disabled)
		return FindCept::ok;
	if (test->centerless)
		return FindCept::retry;
	bool uSectPair = test->isUnsectable() && edge->isUnsectable() && test->isPal(edge);
//	if (uSectPair)
//		return FindCept::unsectable;
	OpRoots roots = test->curve.axisRayHit(axis, normal);  // get the normal at the intersect point	
	float root = OpNaN;
	float testXY = OpNaN;
	auto pushUsectDist = [this, test, &testXY, &root, uSectPair  OP_DEBUG_PARAMS(edge)]() {
		if (uSectPair) {
			addDistance(test, testXY, root, false  OP_DEBUG_PARAMS(edge));
	//		addContainers(winder, test);	// do this in next pass
			return FindCept::addPal;
		}
		return FindCept::retry;
	};
	if (1 != roots.count())
		return pushUsectDist();  // preferable for thread_cubics157381
	root = roots.get(0);
	if (OpMath::IsNaN(root) || 0 == root || root == 1)
		return pushUsectDist();
	OpVector tangent = test->curve.tangent(root).normalize();
	if (!tangent.isFinite() || tangent == OpVector{ 0, 0 } )
		return pushUsectDist();
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	OpVector backRay = -ray;
	float tNxR = tangent.cross(backRay);
	if (fabs(tNxR) < edge->segment->c.normalLimit())
		return pushUsectDist();
	OpPoint pt = test->curve.ptAtT(root);
	Axis perpendicular = !axis;
	testXY = pt.choice(perpendicular);
	bool reversed = tangent.dot(homeTangent) < 0;
	addDistance(test, testXY, root, reversed  OP_DEBUG_PARAMS(edge));
	sorted = false;
	if (!uSectPair && OpMath::Equal(testXY, homeCept, edge->margin()))
		return FindCept::retry;  // e.g., testQuads1877923 has two small quads which just miss 
	return uSectPair ? FindCept::addPal : FindCept::ok;
}

bool SectRay::isOrdered(size_t index) {
	OP_ASSERT(distances.size() > 1);
	OP_ASSERT(index < distances.size());
	Distance& distance = distances[index];
	auto ordered = [distance](OpEdge* test, bool lessThan) {
		std::vector<Distance>& testDs = test->ray.distances;
		auto dIter = std::find_if(testDs.begin(), testDs.end(), [distance](Distance& testD) {
				return distance.edge == testD.edge; });
		if (dIter == testDs.end())
			return true;
		auto tIter = std::find_if(testDs.begin(), testDs.end(), [test](Distance& testD) {
				return test == testD.edge; });
		OP_ASSERT(tIter != testDs.end());
		OP_ASSERT(tIter != dIter);
		return lessThan == (dIter < tIter);
	};
	if (0 < index && !ordered(distances[index - 1].edge, false))
		return false;
	return index + 1 >= distances.size() || ordered(distances[index + 1].edge, true);
}

// search for dependent
// if found, remove edges before dependent
void SectRay::markDependents(OpEdge* edge) {
	float margin = edge->margin();
	float last = homeCept;
	size_t dependentIndex = 0;
	size_t index = distances.size();
	while (index-- && distances[index].edge != edge)
		OP_ASSERT(index);
	while (index--) {
		Distance& distance = distances[index];
		if (!OpMath::Equal(distance.cept, last, margin) && !distance.edge->isUnsectable()
				 && isOrdered(index)) {
			if (!dependentIndex) {
				distance.dependent = true;
				dependentIndex = index;
			} else {
				distances.erase(distances.begin(), distances.begin() + dependentIndex);
				return;
			}
		} else if (dependentIndex) {
			OP_ASSERT(distances[dependentIndex].dependent);
			distances[dependentIndex].dependent = false;
			dependentIndex = 0;
		}
		last = std::min(distance.cept, homeCept);
	}
}

// returns true iff all pals of edge are in ray's distances
bool SectRay::sectsAllPals(const OpEdge* edge) const {
	unsigned found = 0;
	OP_ASSERT(edge->pals.size());
	for (const auto& test : distances) {
		if (edge->isPal(test.edge)) {
			++found;
			break;
		}
	}
	OP_ASSERT(found <= edge->pals.size());
	return found == edge->pals.size();
}

void SectRay::sort() {
	if (sorted)
		return;
	sorted = true;
	std::sort(distances.begin(), distances.end(), [](const Distance& s1, const Distance& s2) {
			return s1.cept < s2.cept || (s1.cept == s2.cept && s1.edge->id < s2.edge->id); });
}

bool SectRay::tryADifferentCenter(OpEdge* edge) {
	mid /= 2;
	midEnd = midEnd < .5 ? 1 - mid : mid;
	float middle = OpMath::Interp(edge->bounds.ltChoice(axis), 
			edge->bounds.rbChoice(axis), midEnd);
	float homeMidT = edge->curve.center(axis, middle);  // note: 0 to 1 on edge curve
	bool tooMany = mid <= interceptLimit;
	if (OpMath::IsNaN(homeMidT) || tooMany) {  // give it at most eight tries
		if (!edge->isUnsectable())
			edge->markUnsortable(tooMany ? Unsortable::tooManyTries : Unsortable::noMidT);
		return false;	// give up
	}
	// if find ray intercept can't find, restart with new center, normal, distance, etc.
	// !!! result is not pinned to bounds
	homeCept = edge->curve.ptAtT(homeMidT).choice(!axis);
	OP_ASSERT(!OpMath::IsNaN(homeCept));
	normal = edge->curve.ptAtT(homeMidT).choice(axis);
	homeT = homeMidT;
	OP_ASSERT(!OpMath::IsNaN(normal));
	return true;
}

// add all edges in contour, and any other contours which the caller says overlap
OpWinder::OpWinder(OpContext& context) {
	for (OpContour* contour : context.contours) {
		OP_ASSERT(!contour->isEmpty());
		contour->addEdges();
	}
}

struct SectPtT {
	SectPtT(OpSegment* seg, const OpSegment* opp, OpPtT cePtT, XyChoice xyChoice)
		: ptT(cePtT)
		, sect(seg->sects.contains(cePtT, opp)) {
		if (sect)
			ptT = sect->ptT;
		OpPtAliases& aliases = seg->contour->context->aliases;
		original = ptT.pt;
		if (OpPoint possibleAlias = aliases.existing(ptT.pt); possibleAlias != ptT.pt)
			ptT.pt = possibleAlias;
	}

	OpPtT ptT;
	OpIntersection* sect;
	OpPoint original;
};

struct SectPair {
	SectPair(CoinEnd& ce, OpSegment* base, XyChoice xy) 
		: seg(ce.seg, ce.opp, ce.ptT, xy)
		, opp(ce.opp, ce.seg, { ce.ptT.pt, ce.oppT.choice(xy) }, xy) 
		, ceSeg(ce.seg)
		, ceOpp(ce.opp)
		, xyChoice(xy)
		, isBaseSegment(base == ce.seg) {
		if (seg.ptT.pt != opp.ptT.pt) {
			if (seg.ptT.pt != seg.original)
				if (opp.ptT.pt != opp.original)
					opp.ptT.pt = ce.seg->contour->context->remapPts(opp.ptT.pt, seg.ptT.pt);
				else
					opp.ptT.pt = seg.ptT.pt;
			else
				seg.ptT.pt = opp.ptT.pt;
		}
	}

	void addSect(MatchEnds match, int coinID, bool oppReversed  OP_LINE_FILE_ARGS()) {
		MatchReverse segMatch { match, oppReversed };
		MatchEnds oppMatch = segMatch.flipped();
		if (!isBaseSegment)
			std::swap(segMatch.match, oppMatch);
		if (seg.sect && opp.sect && !seg.sect->coincidenceID && !opp.sect->coincidenceID) {
			seg.sect->setCoin(coinID, segMatch.match);
			opp.sect->setCoin(coinID, oppMatch);
		} else {
			OP_ASSERT(!seg.sect && !opp.sect);
			seg.sect = ceSeg->addCoin(seg.ptT, coinID, segMatch.match, ceOpp  OP_LINE_FILE_CARGS());
			opp.sect = ceOpp->addCoin(opp.ptT, coinID, oppMatch, ceSeg  OP_LINE_FILE_CARGS());
		}
		seg.sect->pair(opp.sect);
	}

	SectPtT seg;
	SectPtT opp;
	OpSegment* ceSeg;
	OpSegment* ceOpp;
	XyChoice xyChoice;
	bool isBaseSegment;
};

struct CoinSects {
	CoinSects(CoinEnd& coinStart, CoinEnd& coinEnd, OpSegment* b, XyChoice xyChoice) 
		: start(coinStart, b, xyChoice)
		, end(coinEnd, b, xyChoice) {
		if (coinStart.seg != coinEnd.seg) {
			std::swap(end.seg, end.opp);
			std::swap(end.ceSeg, end.ceOpp);
			end.isBaseSegment = !end.isBaseSegment;
		}
		OpContext* context = coinStart.seg->contour->context;
		OpVector threshold = context->threshold();
		auto checkClose = [context, threshold](OpSegment* seg, SectPtT& s, SectPtT& e) {
			bool near = s.ptT.isNearly(e.ptT, threshold);
			OpPoint sPt = s.ptT.pt;
			OpPoint ePt = e.ptT.pt;
			if (near && sPt != ePt) {
				if (sPt != s.original) {
					if (ePt != e.original)
						context->remapPts(ePt, sPt);
					else
						seg->movePt(e.ptT, sPt);
				} else if (ePt != e.original)
					seg->movePt(s.ptT, ePt);
			}
			return near;
		};
		ptsAreClose = checkClose(coinStart.seg, start.seg, end.seg);
		ptsAreClose |= checkClose(coinStart.opp, start.opp, end.opp);
	}

	void addSect(int coinID, bool oppReversed  OP_LINE_FILE_ARGS()) {
		start.addSect(MatchEnds::start, coinID, oppReversed  OP_LINE_FILE_CARGS());
		end.addSect(MatchEnds::end, coinID, oppReversed  OP_LINE_FILE_CARGS());
		OP_DEBUG_VALIDATE_CODE(start.seg.sect->debugCoinValidate());
		OP_DEBUG_VALIDATE_CODE(start.opp.sect->debugCoinValidate());
		OP_DEBUG_VALIDATE_CODE(end.seg.sect->debugCoinValidate());
		OP_DEBUG_VALIDATE_CODE(end.opp.sect->debugCoinValidate());
	}

	bool areUnsectable() const {
		auto unsectMatch = [](const OpIntersection* s, const OpIntersection* e) {
			return s && e && s->unsectID && s->unsectID == e->unsectID;
		};
		return unsectMatch(start.seg.sect, end.seg.sect)
				|| unsectMatch(start.opp.sect, end.opp.sect);
	}

	SectPair start;
	SectPair end;
	bool ptsAreClose;
};

bool CoinEnd::onBothEnds(XyChoice xyChoice) const {
	return OpMath::NearlyEndT(ptT.t) && OpMath::NearlyEndT(oppT.choice(xyChoice));
}

IntersectResult OpWinder::CoincidentCheck(OpSegment* seg, OpSegment* opp) {
	std::array<CoinEnd, 4> ends {{{ seg, opp, { seg->c.firstPt(), 0 }, OpVector() }, 
			{ seg, opp, { seg->c.lastPt(), 1 }, OpVector() },
			{ opp, seg, { opp->c.firstPt(), 0 }, OpVector() },
			{ opp, seg, { opp->c.lastPt(), 1 }, OpVector() }}};
	bool oppReversed;
	XyChoice xyChoice;
	IntersectResult result = CoincidentCheck(ends, &oppReversed, &xyChoice);
	if (IntersectResult::coincident == result &&
			ends[1].onBothEnds(xyChoice) && ends[2].onBothEnds(xyChoice))
		seg->moveWinding(opp, oppReversed);
	return result;
}

IntersectResult OpWinder::CoincidentCheck(std::array<CoinEnd, 4>& ends, bool* oppReversedPtr,
		XyChoice* xyChoicePtr) {
	const auto [minX, maxX] = std::minmax_element(ends.begin(), ends.end(), 
			[](const CoinEnd& a, const CoinEnd& b) { return a.ptT.pt.x < b.ptT.pt.x; });
	const auto [minY, maxY] = std::minmax_element(ends.begin(), ends.end(), 
			[](const CoinEnd& a, const CoinEnd& b) { return a.ptT.pt.y < b.ptT.pt.y; });
	XyChoice xyChoice = maxX->ptT.pt.x - minX->ptT.pt.x < 
			maxY->ptT.pt.y - minY->ptT.pt.y ? XyChoice::inY : XyChoice::inX;
	std::sort(ends.begin(), ends.end(), [xyChoice](const CoinEnd& a, const CoinEnd& b) {
			return a.ptT.pt.choice(xyChoice) < b.ptT.pt.choice(xyChoice); });
	if (ends[1].ptT.pt == ends[2].ptT.pt) { // return if they share a point in the middle
		// single points should have already been added
		OP_ASSERT(ends[1].seg->sects.debugContains(ends[1].ptT, ends[1].opp));
		OP_ASSERT(ends[2].seg->sects.debugContains(ends[2].ptT, ends[2].opp));
		return IntersectResult::yes;
	}
	if (ends[0].seg == ends[1].seg)  // no overlap
		return IntersectResult::no;
	// if sorted order is reversed from input edge, reverse sort
	auto inputReversed = [&ends](const OpSegment* e){
		const CoinEnd* found = nullptr;
		for (const CoinEnd& end : ends) {
			if (end.seg != e)
				continue;
			if (found)  // when 2nd coin end with edge is seen, return if backwards
				return end.ptT.t < found->ptT.t;
			found = &end;
		}
		OP_ASSERT(0);
		return false;
	};
	OpSegment* baseSeg = ends[0].seg;
	OpSegment* baseOpp = ends[0].opp;
	if (inputReversed(baseSeg)) {
		std::swap(ends[0], ends[3]);
		std::swap(ends[1], ends[2]);
	}
	bool oppReversed = inputReversed(baseOpp);  // if opp order is reversed, make a note of that
	bool overlap = ends[0].seg == ends[3].seg;
	auto ratioInOpp = [&ends](int mid) {
		CoinEnd* oppStart = &ends[mid - 1];
		CoinEnd* oppEnd = &ends[mid + 1];
		if (oppStart->seg != oppEnd->seg)
			(1 == mid ? oppEnd : oppStart) = &ends[mid ^ 2];
		OP_ASSERT(oppEnd->seg == oppStart->seg);
		OP_ASSERT(oppEnd->seg != ends[mid].seg);
		float oppStartT = oppStart->ptT.t;
		float oppEndT = oppEnd->ptT.t;
		ends[mid].oppT = OpMath::Interp(oppStartT, oppEndT, ends[mid].oppT);
	};
	// check !xyChoice to see if unratio'd t values map to points with similar ratios
	// to confirm that the midpoint is on the opposite seg's line
	auto calcOppT = [&ends, xyChoice, ratioInOpp, baseOpp, oppReversed]
			(int start, int end, int midIndex) {
		CoinEnd& mid = ends[midIndex];
		if (mid.opp == baseOpp && oppReversed)
			std::swap(start, end);
		mid.oppT = OpMath::Ratio(ends[start].ptT.pt, ends[end].ptT.pt, mid.ptT.pt);
		float minorT = mid.oppT.choice(!xyChoice);
		if (OpMath::IsNaN(minorT))  // line is horizontal or vertical
			return true;
		if (0 > minorT || minorT > 1)
			return false;
		ratioInOpp(midIndex);
		float majorT = mid.oppT.choice(xyChoice);
		OpPoint oppPt = mid.opp->c.ptAtT(majorT);
		return oppPt.isNearly(mid.ptT.pt, mid.opp->threshold());
	};
	if (ends[0].ptT.pt == ends[1].ptT.pt) {  // common start
		OP_ASSERT(ends[0].seg != ends[1].seg);
		ends[1].oppT = { ends[0].ptT.t, ends[0].ptT.t };
	} else if (!calcOppT(0, overlap ? 3 : 2, 1))
		return IntersectResult::no;
	if (ends[2].ptT.pt == ends[3].ptT.pt) { // common end; start/end match
		OP_ASSERT(ends[2].seg != ends[3].seg);
		ends[2].oppT = { ends[3].ptT.t, ends[3].ptT.t };
	} else if (!calcOppT(overlap ? 0 : 1, 3, 2))
		return IntersectResult::no;
	CoinSects coinSects(ends[1], ends[2], baseSeg, xyChoice);
	if (coinSects.areUnsectable())
		return IntersectResult::no;
	if (coinSects.ptsAreClose)
		return IntersectResult::yes;
	int coinID = ends[0].seg->coinID(oppReversed);
	coinSects.addSect(coinID, oppReversed  OP_LINE_FILE_PARGS());
	if (oppReversedPtr)
		*oppReversedPtr = oppReversed;
	if (xyChoicePtr)
	   *xyChoicePtr = xyChoice;
	return IntersectResult::coincident;
}

FoundIntercept OpWinder::findCept(OpEdge* edge) {
	SectRay& ray = edge->ray;
	ray.homeT = OpMath::Ratio(edge->startT, edge->endT, edge->center.t);
	// if find intercept fails, retry some number of times
	// if all retries fail, distinguish between failure cases
	// if it failed because closest edge was too close, mark pair as unsectable
	do {
		ray.distances.clear();
		ray.addDistance(edge, ray.homeCept, ray.homeT, false  OP_DEBUG_PARAMS(edge));
		ray.targets.reset(ray.axis);
		// start at edge with left equal to or left of center
		while (OpEdge* test = ray.targets.next(ray.axis, ray.homeCept)) {
			FindCept findCept = ray.findCept(edge, test);  // adds to back
			if (FindCept::ok == findCept) {
				continue;
			}
			if (FindCept::retry == findCept) 
				goto tryADifferentCenter;
			if (FindCept::addPal == findCept) {
				Distance& tDist = ray.distances.back();
				edge->addPal(&tDist);
				continue;
			}
			if (FindCept::unsortable == findCept)
				return FoundIntercept::fail;
		}
		if (ray.distances.size() <= 1) 
			return FoundIntercept::yes;
		ray.sort();
		if (ray.distances.front().edge == edge)
			return FoundIntercept::yes;
		if (RayOrder::ok == ray.checkClose(edge)) {
			ray.markDependents(edge);
			return FoundIntercept::yes;
		}
tryADifferentCenter:
		;
	} while (ray.tryADifferentCenter(edge));
	return FoundIntercept::fail;	// nonfatal error (!!! give it a different name!)
}

void OpEdge::markUnsortable(Unsortable unsortable) {
	if (Axis::vertical == ray.axis)
		setUnsortable(unsortable);
	else {
		for (RayTarget& target : ray.targets.t) {
			std::vector<OpEdge*>& inY = target.contour->inY;
			if (inY.end() != std::find(inY.begin(), inY.end(), this)) 
				goto foundInY;
		}
		setUnsortable(unsortable);
foundInY: ;
	}
	rayFail = Axis::vertical == ray.axis ? EdgeFail::vertical : EdgeFail::horizontal;
}

FoundWindings OpWinder::setPriors(OpEdge* edge  OP_DEBUG_PARAMS(std::vector<OpEdge*>& debugVisited)) {
	OP_ASSERT(debugVisited.end() == std::find(debugVisited.begin(), debugVisited.end(), edge));
	OP_DEBUG_CODE(debugVisited.push_back(edge));
	OP_ASSERT(!edge->ray.distances.empty());
	Distance& distance = edge->ray.distances.front();
	if ((!distance.dependent || FoundWindings::fail != setPriors(distance.edge  OP_DEBUG_PARAMS(debugVisited))) 
			&& !edge->sum.isSet()) {
//		home = edge;
		ResolveWinding resolveWinding = setWindingByDistance(edge);
		if (ResolveWinding::retry == resolveWinding) {
//			workingAxis = home->ray.axis;  // if edges in ray are too close, 
			OP_DEBUG_CODE(ChainFail debugFail = ) setCept(edge);  // look for a better spot
			OP_ASSERT(ChainFail::none == debugFail);
			resolveWinding = setWindingByDistance(edge);
			OP_ASSERT(ResolveWinding::retry != resolveWinding);
		}
		if (ResolveWinding::fail == resolveWinding)
			OP_DEBUG_FAIL(*edge, FoundWindings::fail);
	}
	return FoundWindings::yes;
}

ChainFail OpWinder::setCept(OpEdge* edge) {
	// see if normal at center point is in direction of ray
	OP_ASSERT(!edge->disabled);
	const OpSegment* edgeSeg = edge->segment;
	OpVector rayLine = Axis::horizontal == edge->ray.axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	float midTest = edge->center.t;
	OP_ASSERT(edge->startT < midTest && midTest < edge->endT);
	float normalLimit = edge->segment->c.normalLimit();
	float midLo = midTest - edge->startT;
	float midHi = edge->endT - midTest;
//	OpVector homeTangent;
	// curve may be flat at center and curved at one end, so don't give up too soon (loop45964)
	auto testTangent = [edge, edgeSeg, rayLine, normalLimit](float testT) {
		edge->ray.homeTangent = edgeSeg->c.tangent(testT);
		float NxR = edge->ray.homeTangent.normalize().cross(rayLine);
		if (!OpMath::IsFinite(NxR))
			return ChainFail::normalizeOverflow;
		if (fabs(NxR) < normalLimit)
			return ChainFail::normalizeUnderflow;  // nonfatal error
		return ChainFail::none;
	};
	bool firstPass = true;  // don't check 'hi' and 'lo' first time: they are the same
	bool isLinear = edge->curve.isLine();  // if curve is line, just check once
	ChainFail chainFail;
	do {  // check in both directions to see if, away from center, curve unflattens
		chainFail = testTangent(midTest);
		if (ChainFail::normalizeUnderflow != chainFail || isLinear)
			break;
		if (!firstPass) {
			chainFail = testTangent(edge->endT - midHi);
			if (ChainFail::normalizeUnderflow != chainFail) {
				midTest = edge->endT - midHi;
				break;
			}
		}
		midLo /= 2;  // godbolt.org says 'divide by two' is identical to 'multiply by 0.5'
		midTest = edge->startT + midLo;
		midHi /= 2;
	} while (std::max(midLo, midHi) > edge->ray.interceptLimit);  // check both since center may not be mid t
	if (ChainFail::normalizeOverflow == chainFail)
		OP_DEBUG_FAIL(*edge, chainFail);  // fatal error : cross product returned infinite / nan
	if (ChainFail::normalizeUnderflow == chainFail) {  // nonfatal error -- try vertical instead
		if (Axis::vertical == edge->ray.axis)
			edge->markUnsortable(Unsortable::rayTooShallow);
		return chainFail;
	}
	OpPoint midPt = edge->curve.ptAtT((midTest - edge->startT) / (edge->endT - edge->startT));
	edge->ray.normal = midPt.choice(edge->ray.axis);
	// intersect normal with every edge in the direction of ray until we run out 
	Axis perpendicular = !edge->ray.axis;
	edge->ray.homeCept = midPt.choice(perpendicular);
	FoundIntercept foundIntercept = findCept(edge);
	edge->ray.sort();
	if (FoundIntercept::fail == foundIntercept)
		return ChainFail::failIntercept;
	if (FoundIntercept::overflow == foundIntercept)
		return ChainFail::normalizeOverflow;
	return ChainFail::none;
}

ChainFail OpWinder::addContainers(OpEdge* top, OpEdge* child, std::vector<OpEdge*>& addedSet) {
	OP_ASSERT(child->ray.sorted || child->ray.distances.empty());
	bool seenHome = false;
	bool added = false;
	for (size_t index = child->ray.distances.size(); index--; ) {
		Distance& distance = child->ray.distances[index];
		if (distance.edge == child) {
			seenHome = true;
			continue;
		}
		if (!seenHome)
			continue;
		if (addedSet.end() != std::find(addedSet.begin(), addedSet.end(), distance.edge))
			continue;
		addedSet.push_back(distance.edge);
		ChainFail result = addContainers(top, distance.edge, addedSet);
		if (ChainFail::none != result)
			return result;
		added |= top->ray.addContainers(distance.edge, top);
		if (distance.dependent)
			break;
	}

	return added ? ChainFail::containerAdded : ChainFail::none;
}

ResolveWinding OpWinder::setWindingByDistance(OpEdge* edge) {
	// find edge; then walk backwards to first known sum 
	// if previous edge is dependent, it cannot use this distance list -- other contours required
	//  recursively evaluate the sum of the dependent edge before proceeding
	OP_ASSERT(!edge->debugSumSet);
	OP_DEBUG_CODE(edge->debugSumSet = true);
	SectRay& ray = edge->ray;
	OP_ASSERT(ray.distances.size());
	if (edge == ray.distances[0].edge) {
		if (edge->isUnsectable())  // !!! move this to where unsectable is set?
			edge->setUnsortable(Unsortable::homeUnsectable);
		else {
			OpWinding prev(edge, WindingSum::dummy);
			// look at direction of edge relative to ray and figure winding/oppWinding contribution
			if (CalcFail::fail == edge->addIfUR(ray.axis, ray.distances[0].edgeInsideT, &prev))
				edge->setUnsortable(Unsortable::addCalcFail);
			else
				edge->setSum(prev  OP_LINE_FILE_PARGS());
		}
		return ResolveWinding::resolved;
	}
	if (!edge->isUnsectable() && EdgeFail::none == edge->rayFail
			&& RayOrder::tooClose == ray.checkClose(edge))
		return ResolveWinding::retry;
	// don't set the sum winding if this has pals or if any prior edge has this as a pal
	// back up only while the previous has pals
	// and: don't set the sum winding if the prior or next edge pt is very close to this pt
	// !!! any prior pal is called several times with the same edge, below. Optimization:
	// !!!    cache the answer in distance edge ?
	auto anyPriorPal = [ray](OpEdge* edge, int sumIndex) {
		if (edge->isUnsectable())
			return true;
		for (;;) {
			int next = sumIndex + 1;
			if (next >= (int) ray.distances.size())
				break;
			if (!ray.distances[next].edge->isUnsectable())
				break;
			sumIndex = next;
		}
		size_t last = (size_t) (sumIndex + 1);
		float lastCept = last < ray.distances.size() ? ray.distances[last].cept : OpNaN;
		bool lastIsEdge = false;
		float margin = edge->margin();
		do {
			const Distance& dist = ray.distances[sumIndex];
			OpEdge* previous = dist.edge;
			if (previous->isPal(edge))
				return true;
			if (lastIsEdge && OpMath::Equal(lastCept, dist.cept, margin))
				return true;
			lastIsEdge = previous == edge;
			if (lastIsEdge) {
				if (OpMath::Equal(lastCept, dist.cept, margin))
					return true;
			} else if (!previous->isUnsectable())
				break;
			lastCept = dist.cept;
		} while (--sumIndex >= 0);
		return false;
	};
	// starting with found or zero if none, accumulate sum up to winding
	// edges in ray may have winding contributions from contours not in home contour's sect
	// remove winding values if distance edge contour is not in home contour sect list
	OpContour* winderOwner = edge->segment->contour->overlapOwner;
	OpWinding sumWinding(edge, WindingSum::dummy);
	int sumIndex = (int) ray.distances.size();
	while (ray.distances[--sumIndex].edge != edge) 
		OP_ASSERT(sumIndex > 0);
	float homeT = ray.distances[sumIndex].edgeInsideT;  // used by unsectable, later
	/* RayOrder order = */ ray.checkOrder(edge);
	while (--sumIndex >= 0) {
		Distance& dist = ray.distances[sumIndex];
		if ((dist.dependent || !anyPriorPal(dist.edge, sumIndex)) && dist.edge->sum.isSet() 
				&& RayOrder::unordered != dist.rayOrder)
			break;
	}
	OpContext* context = edge->context();
	if (sumIndex >= 0) {
		Distance& sumDistance = ray.distances[sumIndex];
		OpEdge* sumEdge = sumDistance.edge;
//		OP_ASSERT(!sumEdge->isUnsectable());
		if (sumEdge->sum.isSet())
			sumWinding.w = sumEdge->sum.copyData(context);
		else
			sumWinding.zero(context);
		OP_DEBUG_CODE(sumWinding.debugType = DebugWindingType::temp);
	// if pointing down/left, subtract winding
	// if sumEdge coin pals' contour is not in home's contour sect, also subtract from winding
		if (CalcFail::fail == sumEdge->subIfDL(winderOwner, ray.axis, sumDistance.edgeInsideT, 
				&sumWinding))  
			OP_DEBUG_FAIL(*sumEdge, ResolveWinding::fail);
	}
	OpEdge* prior;
	do {
		OP_ASSERT(sumIndex + 1 < (int) ray.distances.size());
		Distance& dist = ray.distances[++sumIndex];
		prior = dist.edge;
		if (edge->isUnsectable() && (edge == prior || edge->isPal(prior)))
			break;
		NormalDirection normDir = prior->normalDirection(ray.axis, dist.edgeInsideT);
		if (NormalDirection::underflow == normDir) {
			prior->setUnsortable(Unsortable::underflow);
			continue;
		}
		if (NormalDirection::downLeft == normDir && (dist.dependent || !anyPriorPal(prior, sumIndex))
				&& RayOrder::unordered != ray.distances[sumIndex].rayOrder) {
			prior->setSum(sumWinding  OP_LINE_FILE_PARGS());
		}
		if (CalcFail::fail == prior->addSub(winderOwner, ray.axis, dist.edgeInsideT, &sumWinding)) // if d/l sub; if u/r add
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
		if (NormalDirection::upRight == normDir && (dist.dependent || !anyPriorPal(prior, sumIndex))
				&& RayOrder::unordered != ray.distances[sumIndex].rayOrder) {
			prior->setSum(sumWinding  OP_LINE_FILE_PARGS());
		}
	} while (edge != prior);
	if (!edge->isUnsectable()) {
		if (!edge->sum.isSet())
			edge->setSum(sumWinding  OP_LINE_FILE_PARGS());
		return ResolveWinding::resolved;
	}
	// if edge is unsectable, set its sum winding as if all of its pals' windings were a single edge
	OP_ASSERT(!edge->many.isSet());
	// winding must be replaced by all unsectable windings -- however, other unsectables will want 
	//   to see the original winding. This is why 'many' is used. After all sums are computed
	//   replace winding with many.
	edge->many.setWind(edge->winding);
	for (const auto& pal : edge->pals) {
		edge->winding.move(context, pal.edge->winding, pal.reversed);
	}
	if (!edge->winding.visible(context)) {
		edge->setDisabled(OP_LINE_FILE_NPARGS());
//		edge->segment->contour->isOpen = true;		// !!! may be required (wait for test case)
//		edge->windPal = true;	// !!! doesn't appear to be necessary
	}
	if (CalcFail::fail == edge->addIfUR(ray.axis, homeT, &sumWinding))
		edge->setUnsortable(Unsortable::addCalcFail2);
	else
		edge->setSum(sumWinding  OP_LINE_FILE_PARGS());
	std::swap(edge->many, edge->winding);  // restore winding, put total of pals in many
	return ResolveWinding::resolved;	   // (will copy many to winding after all many are found)
}

// think about how to write this so it sets the windings for each contour
// probably it should walk the edges in the contour, after setting the inX/inY to the 
// cache of sect edges held by the contour (or the 

FoundWindings OpWinder::setWindings(OpContext& context) {
	OP_DEBUG_CONTEXT();
	std::vector<OpEdge*> verticals;
	std::vector<OpEdge*> edges;
	for (OpContour* contour : context.contours) {
		edges.insert(edges.end(), contour->inX.begin(), contour->inX.end());
		verticals.insert(verticals.end(), contour->inY.begin(), contour->inY.end());
	}
	// test sum chain for correctness; recompute if prior or next are inconsistent
	for (Axis axis : { Axis::horizontal, Axis::vertical }) {
		bool firstTry = true;
		if (Axis::vertical == axis)
			std::swap(verticals, edges);
		do {
			std::sort(edges.begin(), edges.end(), [axis](const OpEdge* s1, const OpEdge* s2) {
				return Axis::horizontal == axis ? s1->bounds.left < s2->bounds.left
						: s1->bounds.top < s2->bounds.top;
			});
			for (OpEdge* edge : edges) {  // first pass: find ray cepts for sorting
				if (edge->disabled)	// may not be visible in vertical pass
					continue;
				if (edge->centerless)
					continue;
				if (!edge->ray.distances.empty() && EdgeFail::none == edge->rayFail)
					continue;
				edge->ray.interceptLimit = edge->segment->c.interceptLimit();
				edge->ray.axis = axis;
				if (firstTry)
					edge->ray.targets.build(edge);
				ChainFail chainFail = setCept(edge);
				if (ChainFail::normalizeOverflow == chainFail)
					OP_DEBUG_FAIL(*edge, FoundWindings::fail);
				if (EdgeFail::horizontal == edge->rayFail
						&& verticals.end() != std::find(verticals.begin(), verticals.end(), edge)) {
					OP_ASSERT(Axis::horizontal == axis);
					edge->ray.distances.clear();
				}
//				start here;
				// edge 332 cannot add 304 because it is too close when axis is horizontal so 
				//  it tries vertical next. While this works, it means some 332/304 pairs in
				//  ray distances are horizontal and unsectable, and some are vertical and sectable
			}
			firstTry = false;
			std::sort(edges.begin(), edges.end(), [](const OpEdge* s1, const OpEdge* s2) {
		//		OP_ASSERT(OpMath::IsFinite(s1->ray.homeCept));  // !!! if distances empty: no cept
				return s1->ray.homeCept < s2->ray.homeCept;
			});
			std::vector<OpEdge*> addedEdges;
			for (OpEdge* edge : edges) {  // second pass: find left/topmost edge, make its chain first
				if (axis != edge->ray.axis)
					continue;
				SectRay& ray = edge->ray;
				if (ray.distances.empty())
					continue;
				//  add containers bracketed by dependent and home edges
				std::vector<OpEdge*> addedSet = { edge };
				ChainFail added = addContainers(edge, edge, addedSet);
				if (ChainFail::none == added)
					continue;
				OP_ASSERT(ChainFail::containerAdded == added);
				addedEdges.push_back(edge);
			}
			edges.clear();
			edges.insert(edges.end(), addedEdges.begin(), addedEdges.end());
			for (OpEdge* addedEdge : addedEdges) {
				addedEdge->ray.distances.clear();
			}
			for (auto contour : context.contours) {
				for (auto& segment : contour->segments) {
					for (auto& edge : segment.edges) {
						SectRay& ray = edge.ray;
						if (edge.disabled)
							continue;
						if (edge.isUnsectable())
							edge.markPals();
						else
							ray.addPals(&edge);
						for (EdgePal& edgePal : edge.pals) {  // check if dependent is now a pal
							int uID = edgePal.unsectID;
							for (Distance& dist : ray.distances) {
								if (!dist.dependent)
									continue;
								std::vector<EdgePal>& d = dist.edge->pals;
								if (std::none_of(d.begin(), d.end(), [uID](EdgePal& td) {
										return uID == td.unsectID; } ))
									continue;
								dist.dependent = false;
								edges.push_back(&edge);
							}
							for (Distance& pDist : edgePal.edge->ray.distances) {
								if (!pDist.dependent)
									continue;
								if (pDist.edge == &edge) {
									pDist.dependent = false;
									if (edges.end() != std::find(edges.begin(), edges.end(), 
											&edge))
										edges.push_back(&edge);
								}
							}
						}
					}
				}
			}
		} while (!edges.empty());
	}
	// if a pair of pals share an edge, this puts each in the other's pal list
	for (auto contour : context.contours) {
		for (auto& segment : contour->segments) {
			for (auto& edge : segment.edges) {
					// copy pals if reciprocal, and points to other pals (thread_cubics2247347)
				std::vector<EdgePal>& pals = edge.pals;
				std::vector<EdgePal*> reciprocals;
				for (EdgePal& pal : pals) {
					bool foundReciprocal = false;
					std::vector<EdgePal*> locals;
					for (EdgePal& oPal : pal.edge->pals) {
						if (oPal.edge == &edge) {
							foundReciprocal = true;
							continue;
						}
							// if pals overlap bounds, they are not reciprocal (fuzz763_378)
						if (!edge.bounds.overlaps(oPal.edge->bounds))
							continue;
						if (pals.end() == std::find_if(pals.begin(), pals.end(), [&oPal]
								(const EdgePal& test) {
								return test.edge == oPal.edge;
						})) {
							if (reciprocals.end() == std::find_if(reciprocals.begin(),
									reciprocals.end(), [&oPal](const EdgePal* test) {
									return test->edge == oPal.edge; })) {
								locals.push_back(&oPal);
							}
						}
					}
					if (foundReciprocal)
						reciprocals.insert(reciprocals.end(), locals.begin(), locals.end());
				}
				for (EdgePal* reciprocal : reciprocals) {
					edge.addPal(reciprocal->edge, reciprocal->unsectID, reciprocal->reversed);
				}
			}
		}
	}
	// sort edges so that largest edges' winding sums are computed first
	// iterate all sect contours and mark them so they aren't inspected again
	// collect edges common to sect contours and compute their sums before moving to inner contours
	// then, iterate the edge size by perimeter
//	context.setSortedBounds();
	std::vector<OpEdge*> sectsByX;
	std::vector<OpEdge*> sectsByY;
	for (auto contour : context.contours) {
		// if edge (later) has horizontal ray, use left most; if vertical, use top most
		auto gather = [contour](std::vector<OpEdge*>& sectsByAxis, Axis axis) {
//			OP_ASSERT(!contour->isSorted(axis));
//			contour->setSorted(axis);
			for (auto& segment : contour->segments) {
				for (auto& edge : segment.edges) {
					if (edge.disabled)
						continue;
					if (axis != edge.ray.axis)
						continue;
					if (Unsortable::none != edge.isUnsortable)
						continue;
					if (edge.ray.distances.empty())
						continue;
					sectsByAxis.push_back(&edge);
				}
			}
		};
		gather(sectsByX, Axis::horizontal);
		gather(sectsByY, Axis::vertical);
	}
	for (auto sectsBy : { sectsByX, sectsByY }) {
		std::sort(sectsBy.begin(), sectsBy.end(), [](const auto& s1, const auto& s2) {
			return s1->bounds.perimeter() > s2->bounds.perimeter(); 
		} );
		for (auto edge : sectsBy) {
			if (edge->sum.isSet())
				continue;
//			if (edge->isUnsectable())
//				continue;
			OP_DEBUG_CODE(std::vector<OpEdge*> debugVisited);
			if (FoundWindings::fail == setPriors(edge  OP_DEBUG_PARAMS(debugVisited)))
				OP_DEBUG_FAIL(*edge, FoundWindings::fail);
		}
	}
	for (auto contour : context.contours) {
		for (auto& segment : contour->segments) {
			for (auto& edge : segment.edges) {
				if (edge.disabled)
					continue;
				if (edge.centerless)
					continue;
				if (edge.isUnsectable() && edge.many.isSet())
					std::swap(edge.winding, edge.many);
				if (edge.sum.isSet())
					continue;
				if (Unsortable::none != edge.isUnsortable)
					continue;
				if (edge.rayFail == EdgeFail::horizontal)
					continue;
				OP_DEBUG_FAIL(edge, FoundWindings::fail);
			}
		}
	}
	return FoundWindings::yes;
}
