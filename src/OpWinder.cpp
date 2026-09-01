// (c) 2023, Cary Clark cclark2@gmail.com
#include <cmath>
#include "OpCurveCurve.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include "PathOps.h"

bool RayTargets::addContainer(Axis axis, OpContour* container, OpRect& bounds) {
	if (match(container))
		return false;
	t.push_back({ container, bounds });
	if (inXY)
		return true;
	--tIndex;  // !!! weird that this backs up one only -- what if prior contour is empty?
	set(axis);
	return true;
}

// only include contours that intersect their parents and the chain bounds
void RayTargets::build(OpEdge* edge) {
	// construct rectangle from edge bounds and ray to edge's sects' bounds 
    context = edge->context();
	chainBounds = edge->bounds();
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
	tIndex = SIZE_MAX;
	set(axis); 
}

void RayTargets::set(Axis axis) {
	edgeIndex = SIZE_MAX;
	for (;;) {
		inXY = nullptr;
    #if OP_DEBUG_DUMP
		debugEdgesContour = nullptr;
		debugEdgesAxis = Axis::neither;
	#endif
		if (++tIndex >= t.size())
			return;
		RayTarget& target = t[tIndex];
		OpContour* contour = target.contour;
		if (!contour)
			continue;
		inXY = Axis::horizontal == axis ? &contour->inX : &contour->inY;	
    #if OP_DEBUG_DUMP
        debugEdgesContour = contour;
        debugEdgesAxis = axis;
    #endif
		if (!inXY->empty())
			return;
	}
}

// walk targets (backwards?) to find edges that contribute to the ray
// currently unsortable just gives up, but still keeps sorted rays found earlier
//   walking each contour individually will change that behavior
//   does that behavior need to be preserved?
//   it seems odd -- but could be reproduced if needed (with effort)
//   maybe put that off until it is needed...
OpEdge* RayTargets::next(Axis axis, float homeCept) {
	Axis uppity = !axis;
	OpRect bounds;
    OpVector threshold = context->threshold;
    float thresXY = threshold.choice(uppity);
	while (inXY) {
		// advance to furthest that could influence the sum winding of this edge
		if (edgeIndex >= inXY->size()) {  
			edgeIndex = 0;
			do {
				bounds = (*inXY)[edgeIndex]->curve.callerBounds();
			} while (bounds.ltChoice(uppity) - thresXY <= homeCept && ++edgeIndex < inXY->size());
			if (0 == edgeIndex--) {
				set(axis);
				continue;
			}
		}
		OpEdge* result = (*inXY)[edgeIndex];
		bounds = result->curve.callerBounds();
		OP_ASSERT(bounds.ltChoice(uppity) - thresXY <= homeCept);
		if (0 == edgeIndex--)
			set(axis);
		if (bounds.rbChoice(uppity) >= chainBounds.ltChoice(uppity))
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
bool SectRay::addContainers(OpEdge* addEdge) {
	auto addOne = [this, addEdge](OpContour* container) {
		if (home->segment->contour == container)
			return false;
		OpRect& cBounds = container->bounds;
		// r-reference avoids struct copy
		OpRect homeBounds = home->bounds();
		OpRect addBounds = addEdge->bounds();
		OpRect&& bounds = Axis::horizontal == axis  // area in contour between addEdge and home
			? OpRect(std::max(cBounds.left, addBounds.right), cBounds.top,
					std::min(cBounds.right, homeBounds.left), cBounds.bottom)
			: OpRect(cBounds.left, std::max(container->bounds.top, addBounds.bottom),
					cBounds.right, std::min(container->bounds.bottom, homeBounds.top));
		if (bounds.isEmpty())
			return false;
//		if (std::none_of(containers.begin(), containers.end(), [container](OpContour* test) {
//				return container == test; }))
//			containers.push_back(container);
		return targets.addContainer(axis, container, bounds);
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
				walk.index = (int) walk.dists->size() - 1; // skip last
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
void SectRay::addPals() {
	OP_ASSERT(this == &home->ray);
	if (distances.empty())
		return;
	auto matchCept = [this](const Distance* test) {
		home->addPal(test);
		if (const Distance* homeDist = test->edge->ray.find(home)) {
			test->edge->addPal(homeDist);
//			OP_DEBUG_CODE(Distance* testDist = test->edge->ray.find(test->edge));
// !!! this asserts if there are three or more pals
// consider writing more complex test to detect if edge between pals is not a pal
//			OP_ASSERT(abs(homeDist - testDist) == 1);
		}
		return true;
	};
	const Distance* homeDist = find(home);
	const Distance* test = homeDist;
	float margin = home->margin();
	float lowLimit = homeCept - home->margin();
	bool priorIsPal = false;
	while (test > &distances.front() && (--test)->cept >= lowLimit) {
		OP_ASSERT((test + 1)->cept >= test->cept);
		priorIsPal = matchCept(test);
	}
#if 0
	if (!priorIsPal && homeDist > &distances.front()) {  // check order
		OpEdge* priorEdge = (homeDist - 1)->edge;
		if (priorEdge->ray.distances.size() > 1 && priorEdge->ray.axis == axis) {
			Distance* priorDist = priorEdge->ray.find(priorEdge);
			OP_ASSERT(priorDist);
			if (priorDist > &priorEdge->ray.distances.front() && (priorDist - 1)->edge == home)
				priorIsPal = matchCept(test);
		}
	}
#endif
	test = homeDist;
	float highLimit = homeCept + margin;
	bool nextIsPal = false;
	while (test < &distances.back() && (++test)->cept <= highLimit) {
		OP_ASSERT((test - 1)->cept <= test->cept);
		nextIsPal = matchCept(test);
	}
#if 0
	if (!nextIsPal && homeDist < &distances.back()) {  // check order
		OpEdge* nextEdge = (homeDist + 1)->edge;
		if (nextEdge->ray.distances.size() > 1 && nextEdge->ray.axis == axis) {
			Distance* nextDist = nextEdge->ray.find(nextEdge);
			OP_ASSERT(nextDist);
			if (nextDist < &nextEdge->ray.distances.back() && (nextDist + 1)->edge == home)
				nextIsPal = matchCept(test);
		}
	}
#endif
	// if axes are different, and if y-axis edge is oriented nw/se (not ne/sw), reverse
	auto axesReversed = [this](OpEdge* test) {
		if (test->ray.axis == axis)
			return false;
		OpEdge* vertical = Axis::vertical == test->ray.axis ? test : home;
		OpVector dxy = vertical->curve.lastPt() - vertical->curve.firstPt();
		if (!dxy.dy)
			return false;
		return dxy.dy > 0 ? dxy.dx > 0 : dxy.dx < 0;
	};
	// check next ray intersected edge if it hasn't been checked already
	// !!! stops at 1; don't know if we may need more than one
	// !!! thread_circles54530 failed only on laptop 
	auto addIfFlipped = [axesReversed, homeDist, this](DistEnd offset) {
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
		NormalDirection homeNorm = home->normalDirection(axis, homeRev.edgeInsideT);
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

RayOrder SectRay::checkClose() const {
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
void SectRay::checkOrder() {
	for (Distance* dist = &distances.front(); (dist + 1)->edge != home; ++dist) {
		OpEdge* prior = dist->edge;
		OpEdge* last = (dist + 1)->edge;
		// pal should be set in time for this : testQuads26519435
		if (prior->hasPals() || last->hasPals())
			continue;
		if (last->ray.distances.size() > 1 && last->ray.axis == axis) {
			const Distance* lastDist = last->ray.find(last);
			OP_ASSERT(lastDist);
			if (lastDist < &last->ray.distances.back() && (lastDist + 1)->edge == prior) {
				dist->rayOrder = RayOrder::unordered;
				(dist + 1)->rayOrder = RayOrder::unordered;
				OP_DEBUG_CODE(lastDist->edge->debugUnordered = true);
				OP_DEBUG_CODE((lastDist + 1)->edge->debugUnordered = true);
			}
		}
		if (prior->ray.distances.size() > 1 && prior->ray.axis == axis) {
			const Distance* priorDist = prior->ray.find(prior);
			OP_ASSERT(priorDist);
			if (priorDist > &prior->ray.distances.front() && (priorDist - 1)->edge == last) {
				dist->rayOrder = RayOrder::unordered;
				(dist + 1)->rayOrder = RayOrder::unordered;
				OP_DEBUG_CODE(priorDist->edge->debugUnordered = true);
				OP_DEBUG_CODE((priorDist - 1)->edge->debugUnordered = true);
			}
		}
#if CHECK_SNIP
        // reference id in edges set by intersection snip to find unordered
        if (last->snipped && prior->snipped) {
            if (OpIntersections::SnippedBy(last->segment, prior->segment, normal, axis)) {
                dist->rayOrder = RayOrder::unordered;
                (dist + 1)->rayOrder = RayOrder::unordered;
            }
        }
#endif
	}
}

// ray must be inside bounds, not equal to bounds
bool SectRay::contains(OpPoint test) {
	OP_ASSERT(home);
	if (!insideBounds.isFinite())
		insideBounds = home->curve.aliasBounds().intersect(home->curve.callerBounds());
	float testXY = test.choice(axis);
	return insideBounds.ltChoice(axis) < testXY && testXY < insideBounds.rbChoice(axis);
}

const Distance* SectRay::find(const OpEdge* edge) const {
	auto dIter = std::find_if(distances.begin(), distances.end(), [edge](const Distance& test) {
			return test.edge == edge; });
	if (dIter == distances.end())
		return nullptr;
	const Distance* edgeDist = &*dIter;
	return edgeDist;
}

FindCept SectRay::findCept(OpEdge* test, AllowTooManyRetries allow) {
	if (test->curve.callerBounds().ltChoice(axis) > normal)
		return FindCept::ok;
	if (test->curve.callerBounds().rbChoice(axis) < normal)
		return FindCept::ok;
	if (test == home)
		return FindCept::ok;
	// !!! loop11661 fails if too many retries is allowed -- find test that requires this
	// !!! grshapearc fais if test enabled. 
	if (!test->isSortable() && (AllowTooManyRetries::no == allow 
			|| Unsortable::tooManyTries != test->unsortable))
		return FindCept::unsortable;
	if (test->centerless)
		return FindCept::retry;
	if (test->disabled && test->coinPals.empty())
		return FindCept::ok;
	if (test->curve.start.isFinite() || test->curve.end.isFinite()) {
		// check if axis at normal is between ends of nearly coincident edges (testQuad2558209)
        // !!! can coin pals have opposite edge instead of segment ?
		for (const CoinPal& pal : test->coinPals) {
			bool palsReversed = pal.coinID < 0;
			for (const OpEdge& palEdge : pal.opp->edgeList) {
				if (palEdge.disabled)
					continue;
#if 0  // !!! not sure; this seems like overreach; disallows truly coincident edges (op/testRect2)
				if (&palEdge == home)
					return FindCept::retry;
#endif
				if (std::none_of(palEdge.coinPals.begin(), palEdge.coinPals.end(),
						[pal](const CoinPal& oPal) { return oPal == pal; } ))
					continue;
				if (!test->bounds().overlaps(palEdge.bounds()))
					continue;
		#if 0 && OP_DEBUG
				if (test->startPt() != palEdge.startPt() && !palsReversed) {
					OpDebugOut("!!! " + home->context()->debugData.testname + "\n");
				}
		#endif
//				OP_ASSERT(test->startPt().isNearly(palEdge.startPt(), test->context()->threshold)
//						|| palsReversed);
				OpPoint testStart = test->curve.callerFirst();
                const OpCurve& palCurve = palEdge.curve;
				OpPoint oppStart = palsReversed ? palCurve.callerLast() : palCurve.callerFirst();
				if (testStart != oppStart 
						&& OpMath::Between(testStart.choice(axis), normal, oppStart.choice(axis)))
					return FindCept::retry;
				OpPoint testEnd = test->curve.callerLast();
				OpPoint oppEnd = palsReversed ? palCurve.callerFirst() : palCurve.callerLast();
				if (testEnd != oppEnd 
						&& OpMath::Between(testEnd.choice(axis), normal, oppEnd.choice(axis)))
					return FindCept::retry;
			}
		}
	}
	if (test->disabled)
		return FindCept::ok;
	bool uSectPair = test->isPal(home);
//	if (uSectPair)
//		return FindCept::unsectable;
	OpRoots roots = test->curve.axisRayHit(axis, normal);  // get the normal at the intersect point	
	float root = OpNaN;
	float testXY = OpNaN;
	auto pushUsectDist = [this, test, &testXY, &root, uSectPair]() {
		if (uSectPair) {
			addDistance(test, testXY, root, false  OP_DEBUG_PARAMS(home));
            sorted = false;
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
	if (fabs(tNxR) < home->segment->c.normalLimit())
		return pushUsectDist();
	OpPoint pt = test->curve.ptAtT(root);
	Axis perpendicular = !axis;
	testXY = pt.choice(perpendicular);
    // If intersection of ray with test edge is very close to edge end, it may produce a false 
    // result. The ray may slip between the actual end and miss another edge with an end very close
    // by.  There needs to be a multipler on threshold (ex. chalkboard 16634438230468487913)
    OpContext* context = test->context();
    PathOpsV0Lib::CurveConst rayEndFun = context->callbacks[test->curve.c.type].rayEndFuncPtr;
	OpVector threshold = context->threshold * (rayEndFun ? rayEndFun(test->curve.c) : 4.f);
	if (pt.isNearly(test->curve.c.data->start, threshold))
		return pushUsectDist();
	if (pt.isNearly(test->curve.c.data->end, threshold))
		return pushUsectDist();
	bool reversed = tangent.dot(homeTangent) < 0;
	addDistance(test, testXY, root, reversed  OP_DEBUG_PARAMS(home));
	sorted = false;
	if (!uSectPair && OpMath::Equal(testXY, homeCept, home->margin()))
		return FindCept::retry;  // e.g., testQuads1877923 has two small quads which just miss 
	return uSectPair ? FindCept::addPal : FindCept::ok;
}

bool SectRay::isOrdered(size_t index) const {
	OP_ASSERT(distances.size() > 1);
	OP_ASSERT(index < distances.size());
	const Distance& distance = distances[index];
	auto ordered = [distance](OpEdge* test, bool lessThan) {
		const Distance* testDist = test->ray.find(distance.edge);
		if (!testDist)
			return true;
		const Distance* testSelf = test->ray.find(test);
		OP_ASSERT(testSelf);
		OP_ASSERT(testDist != testSelf);
		return lessThan == (testDist < testSelf);
	};
	if (0 < index && !ordered(distances[index - 1].edge, false))
		return false;
	return index + 1 >= distances.size() || ordered(distances[index + 1].edge, true);
}

// search for dependent
// only allow dependent if it is not too close, it is sectable, and it has the correct axis
// also check if the proposed dependent has a different order in neighboring distance arrays
// if dependent is found, remove edges before dependent
void SectRay::markDependents() {
	float margin = home->margin();
	size_t index = distances.size();
	OP_ASSERT(index >= 2);
	size_t dependentIndex = index;  // candidate to mark dependent, if smaller than home
	while (index-- && distances[index].edge != home)  // find home index
		OP_ASSERT(index);
	if (!index)  // do nothing if home is first
		return;
	Distance* test = &distances[--index];  // compare test (first to left of, or above) with home
	OpEdge* testEdge = test->edge;
	SectRay* testRay = &testEdge->ray;
	const Distance* testSelf = testRay->find(testEdge);  // position in its own distance array
//	if (!testSelf)  // !!! test's ray wasn't built first, even though it is to the left / above home
//		return;
	float testCept = test->cept;
	OP_ASSERT(testCept <= homeCept);  // assert if needs to be std::min(test.cept, homeCept);
	if (!OpMath::Equal(testCept, homeCept, margin) && !testEdge->hasPals()  // not close
			&& testRay->axis == axis && testSelf) {
		const Distance* testHome = testRay->find(home);
		if (!testHome || testSelf < testHome)
			dependentIndex = index;
	}
	while (index--) {
		Distance& prior = distances[index];  // index is now one to the left of test
		OpEdge* priorEdge = prior.edge;
		SectRay* priorRay = &priorEdge->ray;
		const Distance* priorSelf = priorRay->find(priorEdge);
//		OP_ASSERT(priorSelf);
		if (!OpMath::Equal(prior.cept, testCept, margin) && !priorEdge->hasPals()
				&& priorRay->axis == testRay->axis && priorSelf) {
			bool outOfOrder = false;  // if not too close, sectable, same-axis: it's ordered
			const Distance* priorTest = priorRay->find(testEdge);
			if (priorTest)
				outOfOrder = priorTest < priorSelf;  // unless prior ray has pair out of order
			const Distance* testPrior = testRay->find(priorEdge);
			if (testPrior)
				outOfOrder = testSelf < testPrior;  // unless test ray has pair out of order
			if (!outOfOrder && distances.size() > dependentIndex)
				break;  // edge one past dependent is not too close, etc.
			dependentIndex = index;
		} else
			dependentIndex = distances.size();  // need to find dependent and find one to left
		test = &prior;
		testEdge = priorEdge;
		testRay = priorRay;
		testSelf = priorSelf;
		OP_ASSERT(prior.cept <= testCept && prior.cept <= homeCept);
		testCept = prior.cept;  // assert if needs to be std::min(prior.cept, homeCept or testCept);
	}
	if (distances.size() <= dependentIndex) 
		return;
	test->dependent = true;
	if (dependentIndex) {
		// keep distances to be erased in case, later, axis conflict requires them
		OP_DEBUG_CODE(debugErased.insert(debugErased.begin(), distances.begin(), 
				distances.begin() + dependentIndex));
		distances.erase(distances.begin(), distances.begin() + dependentIndex);
	}
}

// returns true iff all pals of edge are in ray's distances
#if 0
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
#endif

void SectRay::sort() {
	if (sorted)
		return;
	sorted = true;
	std::sort(distances.begin(), distances.end(), [](const Distance& s1, const Distance& s2) {
			return s1.cept < s2.cept || (s1.cept == s2.cept && s1.edge->id < s2.edge->id); });
}

bool SectRay::tryADifferentCenter() {
	for (;;) {
		mid /= 2;
		midEnd = midEnd < .5 ? 1 - mid : mid;
		float middle = OpMath::Interp(home->bounds().ltChoice(axis), 
				home->bounds().rbChoice(axis), midEnd);
		float homeMidT = home->curve.center(axis, middle);  // note: 0 to 1 on edge curve
		float interceptLimit = home->segment->c.interceptLimit();
		bool tooMany = mid <= interceptLimit;			
		if (OpMath::IsNaN(homeMidT) || tooMany) {  // give it at most eight tries
			if (!home->hasPals())
				home->markUnsortable(tooMany ? Unsortable::tooManyTries : Unsortable::noMidT);
			return false;	// give up
		}
		// if find ray intercept can't find, restart with new center, normal, distance, etc.
		// !!! result is not pinned to bounds
		OpPoint midPt = home->curve.ptAtT(homeMidT);
		if (!contains(midPt))
			continue;
		homeCept = midPt.choice(!axis);
		OP_ASSERT(!OpMath::IsNaN(homeCept));
		normal = midPt.choice(axis);
		homeT = homeMidT;
		OP_ASSERT(!OpMath::IsNaN(normal));
		return true;
	} 
}

struct SectPtT {
	SectPtT(OpSegment* seg, const OpSegment* opp, OpPtT cePtT, XyChoice xyChoice)
		: ptT(cePtT)
		, sect(seg->sects.contains(cePtT, opp)) {
		if (sect)
			ptT = sect->ptT;
        original = sect ? sect->callerPt : ptT.pt;
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
			if (seg.ptT.pt != seg.original) {
				if (opp.ptT.pt != opp.original)
					OP_ASSERT(0);  // !!! rewritten; trace through new code
				else
					opp.ptT.pt = seg.ptT.pt;
			} else
				seg.ptT.pt = opp.ptT.pt;
		}
	}

	void addSect(MatchEnds match, int coinID, bool oppReversed  OP_LINE_FILE_ARGS()) {
		MatchReverse segMatch { match, oppReversed };
		MatchEnds oppMatch = segMatch.flipped();
		if (!isBaseSegment)
			std::swap(segMatch.match, oppMatch);
		if (seg.sect && opp.sect && !seg.sect->coincidenceID && !opp.sect->coincidenceID) {
			seg.sect->setCoin(coinID, segMatch.match, CoinOpp::no);
			opp.sect->setCoin(coinID, oppMatch, CoinOpp::yes);
		} else {
			OP_ASSERT(!seg.sect && !opp.sect);
			seg.sect = ceSeg->addCoin(seg.ptT, coinID, segMatch.match, CoinOpp::no, ceOpp  
					OP_LINE_FILE_CARGS());
			opp.sect = ceOpp->addCoin(opp.ptT, coinID, oppMatch, CoinOpp::yes, ceSeg  
					OP_LINE_FILE_CARGS());
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
		OpContour* contour = coinStart.seg->contour;
		OpVector threshold = contour->context->threshold;
		auto checkClose = [threshold](OpSegment* seg, SectPtT& s, SectPtT& e) {
			bool near = s.ptT.isNearly(e.ptT, threshold);
		#if 1
//			OP_ASSERT(!near || s.ptT.pt != e.ptT.pt);
		#else
			// !!! if points are near but not equal, earlier calc. ought to have found alias
			if (near && s.ptT.pt != e.ptT.pt) {
				if (s.ptT.pt != s.original) {
					contour->remapPts(e.ptT.pt, s.ptT.pt);
					if (e.ptT.pt == e.original)
						segment->movePt(e.ptT.pt, s.ptT.pt);
				} else if (e.ptT.pt != e.original)
					contour->movePt(s.ptT.pt, e.ptT.pt);
			}
		#endif
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

	bool fullCoincidence() const {
		return OpMath::NearlyEndT(start.seg.ptT.t) && OpMath::NearlyEndT(start.opp.ptT.t)
				&& OpMath::NearlyEndT(end.seg.ptT.t) && OpMath::NearlyEndT(end.opp.ptT.t)
				&& !OpMath::Equal(start.seg.ptT.t, end.seg.ptT.t, OpEpsilon * 2)
				&& !OpMath::Equal(start.opp.ptT.t, end.opp.ptT.t, OpEpsilon * 2);
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
	if (coinSects.fullCoincidence()) {
		baseSeg->moveWinding(baseOpp, oppReversed);
		return IntersectResult::coincident;
	}
	int coinID = ends[0].seg->coinID(oppReversed);
	coinSects.addSect(coinID, oppReversed  OP_LINE_FILE_PARGS());
	if (oppReversedPtr)
		*oppReversedPtr = oppReversed;
	if (xyChoicePtr)
	   *xyChoicePtr = xyChoice;
	return IntersectResult::coincident;
}

FoundIntercept OpWinder::FindACept(OpEdge* edge, AllowTooManyRetries allow) {
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
			FindCept findCept = ray.findCept(test, allow);  // adds to back
			if (FindCept::ok == findCept || AllowTooManyRetries::yes == allow)
				continue;
			if (FindCept::retry == findCept) 
				goto tryADifferentCenter;
			if (FindCept::addPal == findCept) {
				Distance& tDist = ray.distances.back();
				edge->addPal(&tDist);
				continue;
			}
			if (FindCept::unsortable == findCept)
				goto tryADifferentCenter;  // !!! was return FoundIntercept::fail; which did not set edge to unsortable
		}
		if (ray.distances.size() <= 1) 
			return FoundIntercept::yes;
		ray.sort();
		if (ray.distances.front().edge == edge)
			return FoundIntercept::yes;
		if (RayOrder::ok == ray.checkClose()) {
			ray.markDependents();
			return FoundIntercept::yes;
		}
tryADifferentCenter:
		OpNop();
	} while (ray.tryADifferentCenter());
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

FoundWindings OpWinder::SetPriors(OpEdge* edge  OP_DEBUG_PARAMS(std::vector<OpEdge*>& debugVisited)) {
	OP_ASSERT(debugVisited.end() == std::find(debugVisited.begin(), debugVisited.end(), edge));
	OP_DEBUG_CODE(debugVisited.push_back(edge));
	OP_ASSERT(!edge->ray.distances.empty());
	Distance& distance = edge->ray.distances.front();
	if ((!distance.dependent || FoundWindings::fail != SetPriors(distance.edge  OP_DEBUG_PARAMS(debugVisited))) 
			&& !edge->sum.isSet()) {
//		home = edge;
		ResolveWinding resolveWinding = SetWindingByDistance(edge);
		if (ResolveWinding::retry == resolveWinding) {
//			workingAxis = home->ray.axis;  // if edges in ray are too close, 
			OP_DEBUG_CODE(ChainFail debugFail = ) SetCept(edge);  // look for a better spot
			OP_ASSERT(ChainFail::none == debugFail);
			resolveWinding = SetWindingByDistance(edge);
			OP_ASSERT(ResolveWinding::retry != resolveWinding);
		}
		if (ResolveWinding::fail == resolveWinding)
			OP_DEBUG_FAIL(*edge, FoundWindings::fail);
	}
	return FoundWindings::yes;
}

ChainFail OpWinder::SetCept(OpEdge* edge) {
	// see if normal at center point is in direction of ray
	OP_ASSERT(!edge->disabled);
	const OpSegment* edgeSeg = edge->segment;
	edge->rayFail = EdgeFail::none;
	OpVector rayLine = Axis::horizontal == edge->ray.axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	float midTest = edge->center.t;
	OP_ASSERT(edge->startT < midTest && midTest < edge->endT);
	float normalLimit = edge->segment->c.normalLimit();
	float midLo = midTest - edge->startT;
	float midHi = edge->endT - midTest;
	float limit = std::max(OpEpsilon, (edge->endT - edge->startT) * edge->curve.interceptLimit());
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
	} while (std::max(midLo, midHi) > limit);  // check both since center may not be mid t
	if (ChainFail::normalizeOverflow == chainFail)
		OP_DEBUG_FAIL(*edge, chainFail);  // fatal error : cross product returned infinite / nan
	if (ChainFail::normalizeUnderflow == chainFail) {  // nonfatal error -- try vertical instead
		if (Axis::vertical == edge->ray.axis || 0 == edge->curve.width())
			edge->markUnsortable(Unsortable::rayTooShallow);
		return chainFail;
	}
	edge->ray.mid = 1;
	if (!edge->ray.tryADifferentCenter())
		return ChainFail::outsideBounds;
	// intersect normal with every edge in the direction of ray until we run out 
	FoundIntercept foundIntercept = FindACept(edge, AllowTooManyRetries::no);
	edge->ray.sort();
	if (FoundIntercept::fail == foundIntercept)
		return ChainFail::failIntercept;
	if (FoundIntercept::overflow == foundIntercept)
		return ChainFail::normalizeOverflow;
	return ChainFail::none;
}

ChainFail OpWinder::AddContainers(OpEdge* top, OpEdge* child, std::vector<OpEdge*>& addedSet) {
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
		ChainFail result = AddContainers(top, distance.edge, addedSet);
		if (ChainFail::none != result)
			return result;
		added |= top->ray.addContainers(distance.edge);
		if (distance.dependent)
			break;
	}

	return added ? ChainFail::containerAdded : ChainFail::none;
}

ResolveWinding OpWinder::SetWindingByDistance(OpEdge* edge) {
	// find edge; then walk backwards to first known sum 
	// if previous edge is dependent, it cannot use this distance list -- other contours required
	//  recursively evaluate the sum of the dependent edge before proceeding    
	OP_ASSERT(!edge->debugSumSet || !edge->isSortable() || edge->unsummable);
	OP_DEBUG_CODE(edge->debugSumSet = true);
	SectRay& ray = edge->ray;
	OP_ASSERT(ray.distances.size());
	if (edge == ray.distances[0].edge) {
		if (!edge->isSummable() || edge->hasPals())  // !!! move this to where unsummable is set?
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
	if (edge->isSummable() && EdgeFail::none == edge->rayFail
			&& RayOrder::tooClose == ray.checkClose())
		return ResolveWinding::retry;
	// don't set the sum winding if this has pals or if any prior edge has this as a pal
	// back up only while the previous has pals
	// and: don't set the sum winding if the prior or next edge pt is very close to this pt
	// !!! any prior pal is called several times with the same edge, below. Optimization:
	// !!!    cache the answer in distance edge ?
	auto anyPriorPal = [ray](OpEdge* edge, int sumIndex) {
		if (!edge->isSummable())
			return true;
		for (;;) {
			int next = sumIndex + 1;
			if (next >= (int) ray.distances.size())
				break;
			if (ray.distances[next].edge->isSummable())
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
			} else if (previous->isSummable())
				break;
			lastCept = dist.cept;
		} while (--sumIndex >= 0);
		return false;
	};
	// starting with found or zero if none, accumulate sum up to winding
	// edges in ray may have winding contributions from contours not in home contour's sect
	// remove winding values if distance edge contour is not in home contour sect list
	OpWinding sumWinding(edge, WindingSum::dummy);
	int sumIndex = (int) ray.distances.size();
	while (ray.distances[--sumIndex].edge != edge) 
		OP_ASSERT(sumIndex > 0);
	float homeT = ray.distances[sumIndex].edgeInsideT;  // used by unsectable, later
	ray.checkOrder();  // !!! start here; trace through edges with pals and without sums to see if that works
	while (--sumIndex >= 0) {
		Distance& dist = ray.distances[sumIndex];
		if ((dist.dependent || !anyPriorPal(dist.edge, sumIndex)) && dist.edge->sum.isSet() 
				&& RayOrder::unordered != dist.rayOrder)
			break;
	}
	if (sumIndex >= 0) {
		Distance& sumDistance = ray.distances[sumIndex];
		OpEdge* sumEdge = sumDistance.edge;
//		OP_ASSERT(!sumEdge->isUnsectable());
		if (sumEdge->sum.isSet())
			sumWinding.w = sumEdge->sum.copyData();
		else
			sumWinding.zero();
		OP_DEBUG_CODE(sumWinding.debugType = DebugWindingType::temp);
	// if pointing down/left, subtract winding
	// if sumEdge coin pals' contour is not in home's contour sect, also subtract from winding
		if (CalcFail::fail == sumEdge->subIfDL(ray.axis, sumDistance.edgeInsideT, 
				&sumWinding))  
			OP_DEBUG_FAIL(*sumEdge, ResolveWinding::fail);
	}
	OpEdge* prior;
	bool edgeUnsummable = edge->unsummable;
	do {
		OP_ASSERT(sumIndex + 1 < (int) ray.distances.size());
		Distance& dist = ray.distances[++sumIndex];
		prior = dist.edge;
		if (edgeUnsummable && (edge == prior || edge->isPal(prior)))
			break;
		NormalDirection normDir = prior->normalDirection(ray.axis, dist.edgeInsideT);
		if (NormalDirection::underflow == normDir) {
			prior->setUnsortable(Unsortable::underflow);
			continue;
		}
		bool allowSetSum = prior->isSummable() && ((dist.dependent || !anyPriorPal(prior, sumIndex))
				&& RayOrder::unordered != ray.distances[sumIndex].rayOrder);
		if (!allowSetSum && prior == edge)
			edgeUnsummable = true;
		if (allowSetSum && NormalDirection::downLeft == normDir)
			prior->setSum(sumWinding  OP_LINE_FILE_PARGS());
		if (CalcFail::fail == prior->addSub(ray.axis, dist.edgeInsideT, &sumWinding)) // if d/l sub; if u/r add
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
		if (allowSetSum && NormalDirection::upRight == normDir)
			prior->setSum(sumWinding  OP_LINE_FILE_PARGS());
	} while (edge != prior);
	if (!edgeUnsummable) {
		if (!edge->sum.isSet())
			edge->setSum(sumWinding  OP_LINE_FILE_PARGS());
		return ResolveWinding::resolved;
	}
	if (!edge->winding.visible()) {
		edge->setDisabled(OP_LINE_FILE_NPARGS());
//		edge->segment->contour->isOpen = true;		// !!! may be required (wait for test case)
//		edge->windPal = true;	// !!! doesn't appear to be necessary
	}
	if (CalcFail::fail == edge->addIfUR(ray.axis, homeT, &sumWinding))
		edge->setUnsortable(Unsortable::addCalcFail2);
	else if (!edgeUnsummable)
		edge->setSum(sumWinding  OP_LINE_FILE_PARGS());
	return ResolveWinding::resolved;	   
}

// think about how to write this so it sets the windings for each contour
// probably it should walk the edges in the contour, after setting the inX/inY to the 
// cache of sect edges held by the contour (or the 

FoundWindings OpWinder::SetWindings(OpContext& context) {
	for (OpContour* contour : context.contours) {
		OP_ASSERT(!contour->isEmpty());
		contour->addEdges();
	}
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
				return Axis::horizontal == axis ? s1->curve.left() < s2->curve.left()
						: s1->curve.top() < s2->curve.top();
			});
			for (OpEdge* edge : edges) {  // first pass: find ray cepts for sorting
				if (edge->disabled)	// may not be visible in vertical pass
					continue;
				if (edge->centerless)
					continue;
				if (!edge->ray.distances.empty() && EdgeFail::none == edge->rayFail)
					continue;
				edge->ray.axis = axis;
				if (firstTry)
					edge->ray.targets.build(edge);
				ChainFail chainFail = SetCept(edge);
				if (ChainFail::normalizeOverflow == chainFail)
					OP_DEBUG_FAIL(*edge, FoundWindings::fail);
				if (EdgeFail::horizontal == edge->rayFail
						&& verticals.end() != std::find(verticals.begin(), verticals.end(), edge)) {
					OP_ASSERT(Axis::horizontal == axis);
					edge->ray.distances.clear();
				} else if (EdgeFail::none != edge->rayFail 
						|| ChainFail::failIntercept == chainFail) {
	// keep first horizontal (or recompute it) if ray finding gives up
	// or keep first vertical if, after giving up, edge is more horizontal than vertical
	//	? use edge bounds to choose w or h?
	// in any case, find the cumulative winding of the target edge and all too-close edges
	// if total winding changes sign, allow two edges in output
	// if total winding goes from zero to non-zero, allow one edge
	// otherwise, disable
					OpPoint midPt = edge->curve.ptAtT((edge->center.t - edge->startT) 
							/ (edge->endT - edge->startT));
					edge->ray.axis = edge->bounds().largerAxis();
					if (!edge->ray.contains(midPt)) {
						edge->ray.mid = 1;
						if (!edge->ray.tryADifferentCenter())
							continue;
					} else {
						edge->ray.normal = midPt.choice(edge->ray.axis);
						edge->ray.homeCept = midPt.choice(!edge->ray.axis);
					}
					edge->ray.targets.chainBounds = edge->bounds();
					OpContour* contour = edge->segment->contour;
					const OpRect& overlapBounds = contour->overlapOwner->overlapBounds;
					if (Axis::horizontal == edge->ray.axis)
						edge->ray.targets.chainBounds.left = overlapBounds.left;
					else
						edge->ray.targets.chainBounds.top = overlapBounds.top;
					FindACept(edge, AllowTooManyRetries::yes);
					edge->ray.sort();
				}
			#if 0  // some variant of this may be needed to debug more issues like this
				// edge 332 cannot add 304 because it is too close when axis is horizontal so 
				//  it tries vertical next. While this works, it means some 332/304 pairs in
				//  ray distances are horizontal and unsectable, and some are vertical and sectable
				if (350 == edge->id || 351 == edge->id || 390 == edge->id) {
					OpEdge* e350 = findEdge(350);
					OpEdge* e351 = findEdge(351);
					OpEdge* e390 = findEdge(390);
					auto axisStr = [](OpEdge* e) {
						Axis a = e->ray.axis;
						const char* aCh = Axis::vertical == a ? "v" : Axis::horizontal == a ? "h" 
								: "-";
						std::string s = STR(e->id) + " axis:" + STR(aCh) + " ";
						if (Unsortable::none != e->unsortable)
							s += "u ";
						return s;
					};
					OpDebugOut("edge:" + STR(edge->id) + " " + axisStr(e350) + axisStr(e351) 
							+ axisStr(e390) + "\n");
				}
			#endif
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
				ChainFail added = AddContainers(edge, edge, addedSet);
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
			// !!! start here;
			// this does too much work!
			// at very least, it should only consider contours edges intersect
			for (auto contour : context.contours) {
				for (auto& segment : contour->segments) {
					for (auto& edge : segment.edgeList) {
						if (edge.disabled)
							continue;
						SectRay& ray = edge.ray;
						if (edge.hasPals())
							edge.markPals();
						else
							ray.addPals();
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
			for (auto& edge : segment.edgeList) {
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
						if (!edge.bounds().overlaps(oPal.edge->bounds()))
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
					OpEdge* oEdge = reciprocal->edge;
					edge.addPal(oEdge, reciprocal->unsectID, reciprocal->reversed);
					std::vector<Distance>& oDist = oEdge->ray.distances;
					if (oDist.end() != std::find_if(oDist.begin(), oDist.end(), [edge]
							(const Distance& test) { return test.edge->id == edge.id; } ))
						oEdge->addPal(&edge, reciprocal->unsectID, reciprocal->reversed);
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
				for (auto& edge : segment.edgeList) {
					if (edge.disabled)
						continue;
					if (axis != edge.ray.axis)
						continue;
					if (!edge.isSortable())
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
			return s1->bounds().perimeter() > s2->bounds().perimeter(); 
		} );
		for (auto edge : sectsBy) {
			if (edge->sum.isSet())
				continue;
			if (!edge->isSummable())
				continue;
			OP_DEBUG_CODE(std::vector<OpEdge*> debugVisited);
			if (FoundWindings::fail == SetPriors(edge  OP_DEBUG_PARAMS(debugVisited)))
				OP_DEBUG_FAIL(*edge, FoundWindings::fail);
		}
	}
	for (auto contour : context.contours) {
		for (auto& segment : contour->segments) {
			for (auto& edge : segment.edgeList) {
				if (edge.disabled)
					continue;
				if (edge.centerless)
					continue;
				if (edge.sum.isSet())
					continue;
				if (!edge.isSummable())
					continue;
				if (!edge.isSortable())
					continue;
				if (edge.rayFail == EdgeFail::horizontal)
					continue;
				if (edge.hasPals())
					continue;
				OP_DEBUG_FAIL(edge, FoundWindings::fail);
			}
		}
	}
	return FoundWindings::yes;
}
