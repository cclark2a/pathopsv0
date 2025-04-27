// (c) 2023, Cary Clark cclark2@gmail.com
#include <cmath>
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include "PathOps.h"

// !!! start here;
#if 0
// if check members adds contour, restart build target if chain bounds got bigger
static bool checkMembers(Axis axis, OpContour* member, std::vector<RayTarget>& openTargets, 
		OpRect& chainBounds, std::vector<OpContour*>& visited) {
	const OpRect& setBounds = member->setOwner->setBounds;
	float leftTop = setBounds.ltChoice(axis);
	if (chainBounds.ltChoice(axis) > leftTop) {
		*(&chainBounds.left + +axis) = leftTop;
		return false;
	}
	for (OpContour* inner : member->members()) {
		if (visited.end() != std::find_if(visited.begin(), visited.end(),
				[inner](OpContour* test) { return inner == test; } ))
			continue;
		visited.push_back(inner);
		if (!inner->isOpen())
			continue;
		OpPointBounds innerBounds(inner->bounds.intersect(chainBounds));
		if (innerBounds.isEmpty())
			continue;
		if (openTargets.end() != std::find_if(openTargets.begin(), openTargets.end(),
				[inner](RayTarget& test) { return inner == test.contour; } ))
			continue;
		openTargets.push_back({ inner, innerBounds });
		if (!checkMembers(axis, inner, openTargets, chainBounds, visited))
			return false;
	}
	return true;
}
#endif

// if horizontal axis, look at rect top/bottom

// only include contours that intersect their parents and the chain bounds
void RayTargets::build(OpWinder* winder) {
	// construct rectangle from home bounds and ray to home's sects' bounds 
	chainBounds = winder->home->bounds;
	axis = winder->workingAxis;
	OpContour* contour = winder->home->segment->contour;
	const OpRect& overlapBounds = contour->overlapOwner->overlapBounds;
	if (Axis::horizontal == axis)
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
	// if member bounds is before contour bounds, skip it
	// but: if member contour is completely to the left / above home, can't skip it
	#if 0	
		Axis perpendicular = !axis;
		if (!member->isOpen()) {  // if coincidence or design makes this not a loop, can't skip
			float memberLimit = member->bounds.rbChoice(perpendicular) 
					+  contour->context->threshold().choice(perpendicular);
			if (memberLimit < contour->bounds.ltChoice(perpendicular))
				continue;
		} 
		else {  // if contour is open and its set contour is open, include it (testLine320627)
			for (auto inner : member->coincident) {
				if (openTargets.end() != std::find_if(openTargets.begin(), openTargets.end(),
						[inner](RayTarget& test) { return inner == test.contour; } ))
					continue;
				OpPointBounds innerBounds(inner->bounds.intersect(chainBounds));
				if (innerBounds.isEmpty())
					continue;
				openTargets.push_back({ inner, innerBounds });
			}
		}
	#endif
		t.push_back({ member, sectBounds });
	}
	for (RayTarget& target : openTargets) {
		if (!match(target.contour))
			t.push_back(target);
	}
}

void RayTargets::reset() {
	index = (size_t) -1; 
	set(); 
}

void RayTargets::set() {
	edgeIndex = INT_MAX;
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
OpEdge* RayTargets::next(float homeCept) {
	Axis perpendicular = !axis;
	while (edges) {
		// advance to furthest that could influence the sum winding of this edge
		if (edgeIndex >= edges->size()) {  
			edgeIndex = 0;
			while ((*edges)[edgeIndex]->bounds.ltChoice(perpendicular) <= homeCept
						&& ++edgeIndex < edges->size())
				;
			if (0 == edgeIndex--) {
				set();
				continue;
			}
		}
		OpEdge* result = (*edges)[edgeIndex];
		OP_ASSERT(result->bounds.ltChoice(perpendicular) <= homeCept);
		if (0 == edgeIndex--)
			set();
		return result;
	}
	return nullptr;
}

bool RayTargets::match(OpContour* contour) const {
	return t.end() != std::find_if(t.begin(), t.end(), [contour](const RayTarget& rayTarget) 
			{ return rayTarget.contour == contour; } );
}

#if 0
bool SectRay::addCoinContours(OpWinder* winder) {
	if (!checkCoins)
		return false;
	std::vector<RayTarget> add;
	auto addContour = [&add, winder](OpContour* contour) {
		if (!winder->targets.match(contour) && add.end() == std::find_if(add.begin(), add.end(),
				[contour](RayTarget& target) { return target.contour == contour; } )) {
			OpPointBounds bounds(contour->bounds.intersect(winder->targets.chainBounds));
			if (bounds.isEmpty())
				return;
			add.push_back({ contour, bounds });
		}
	};
	for (EdgePal& edgePal : distances) {
		for (CoinPal& coinPal : edgePal.edge->coinPals) {
			addContour(coinPal.opp->contour);
		}
		for (OpContour* oContour : edgePal.edge->segment->coinContours) {
			addContour(oContour);
		}
	}
	if (add.empty())
		return false;
	winder->targets.t.insert(winder->targets.t.end(), add.begin(), add.end());
	return true;
}
#endif

static void checkMembers(OpContour* contour, OpWinder* winder, std::vector<RayTarget>& add, 
		std::vector<OpContour*>& visited) {
	if (visited.end() != std::find(visited.begin(), visited.end(), contour))
		return;
	visited.push_back(contour);
	if (winder->targets.match(contour))
		return;
	if (add.end() != std::find_if(add.begin(), add.end(),
			[contour](RayTarget& target) { return target.contour == contour; } )) 
		return;
	OpPointBounds bounds(contour->bounds.intersect(winder->targets.chainBounds));
	if (!bounds.isEmpty())
		add.push_back({ contour, bounds });
	for (OpContour* inner : contour->merges) {
		checkMembers(inner, winder, add, visited);
	}
}

bool SectRay::addDependentContours(OpWinder* winder) {
	std::vector<OpContour*> visited;
	std::vector<RayTarget> add;
	for (EdgePal& edgePal : distances) {
		OpEdge* test = edgePal.edge;
		if (winder->home == test)
			continue;
		OP_ASSERT(!test->ray.distances.empty());
		for (EdgePal& testPal : test->ray.distances) {
			if (testPal.edge == test)
				continue;
			checkMembers(testPal.edge->segment->contour, winder, add, visited);
		}
	}
	if (add.empty())
		return false;
	winder->targets.t.insert(winder->targets.t.end(), add.begin(), add.end());
	return true;
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
	auto matchCept = [home](const EdgePal* test) {
		home->addPal(*test);
		if (const EdgePal* homeDist = test->edge->ray.find(home)) {
			test->edge->addPal(*homeDist);
//			OP_DEBUG_CODE(EdgePal* testDist = test->edge->ray.find(test->edge));
// !!! this asserts if there are three or more pals
// consider writing more complex test to detect if edge between pals is not a pal
//			OP_ASSERT(abs(homeDist - testDist) == 1);
		}
	};
	const EdgePal* homeDist = find(home);
	const EdgePal* test = homeDist;
	float threshold = home->context()->threshold().choice(axis);
	float lowLimit = homeCept - threshold;
	bool priorIsPal = false;
	while (test > &distances.front() && (--test)->cept >= lowLimit) {
		OP_ASSERT((test + 1)->cept >= test->cept);
		matchCept(test);
		priorIsPal = true;
	}
	test = homeDist;
	float highLimit = homeCept + threshold;
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
		const EdgePal* dist = ray.find(edge);
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
		EdgePal edgeRev = *dist;
		EdgePal homeRev = *homeDist;
		NormalDirection edgeNorm = edge->normalDirection(ray.axis, edgeRev.edgeInsideT);
		NormalDirection homeNorm = home->normalDirection(home->ray.axis, homeRev.edgeInsideT);
		if (edgeNorm != homeNorm) {
			edgeRev.reversed = true;
			homeRev.reversed = true;
		}
		home->addPal(edgeRev);
		edge->addPal(homeRev);
		return;
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

// check if pair of rays contributing to home are reversed from their own ray order (informative)
// check if distance adjacent to home is too close to safely determine the ray order (actionable)
RayOrder SectRay::checkOrder(const OpEdge* home) {
	RayOrder result = RayOrder::ok;
	for (EdgePal* dist = &distances.front(); (dist + 1)->edge != home; ++dist) {
		OpEdge* prior = dist->edge;
		OpEdge* last = (dist + 1)->edge;
		// pal should be set in time for this : testQuads26519435
		if (prior->isUnsectable() || last->isUnsectable() || last->isPal(prior))
			continue;
		if (last->ray.distances.size() > 1 && last->ray.axis == axis) {
			EdgePal* lastDist = last->ray.find(last);
			if (lastDist < &last->ray.distances.back() && (lastDist + 1)->edge == prior) {
				dist->rayOrder = RayOrder::unordered;
				(dist + 1)->rayOrder = RayOrder::unordered;
				OP_DEBUG_CODE(lastDist->edge->debugUnordered = true);
				OP_DEBUG_CODE((lastDist + 1)->edge->debugUnordered = true);
				result = RayOrder::unordered;  // may be overridden below
			}
		}
		if (prior->ray.distances.size() > 1 && prior->ray.axis == axis) {
			EdgePal* priorDist = prior->ray.find(prior);
			if (priorDist > &prior->ray.distances.front() && (priorDist - 1)->edge == last) {
				dist->rayOrder = RayOrder::unordered;
				(dist + 1)->rayOrder = RayOrder::unordered;
				OP_DEBUG_CODE(priorDist->edge->debugUnordered = true);
				OP_DEBUG_CODE((priorDist - 1)->edge->debugUnordered = true);
				result = RayOrder::unordered;  // may be overridden below
			}
		}
	}
	// check to see if closest to home is too close
	const EdgePal* homeD = nullptr;
	const EdgePal* dist = &distances.back();
	do {
		if (home == dist->edge)
			homeD = dist;
	} while (--dist >= &distances.front() && !homeD);
	OP_ASSERT(homeD);
	float hCept = homeD->cept;
	float threshold = home->context()->threshold().choice(axis);
	if (dist >= &distances.front() && OpMath::Equal(dist->cept, hCept, threshold))
		return RayOrder::tooClose;
	dist = homeD + 1;
	if (dist <= &distances.back() && OpMath::Equal(dist->cept, hCept, threshold))
		return RayOrder::tooClose;
	return result;
}

EdgePal* SectRay::find(const OpEdge* edge) {
	if (distances.empty())
		return nullptr;
	for (auto test = &distances.back(); test >= &distances.front(); --test) {
		if (test->edge == edge)
			return test;
	}
	return nullptr;
}

FindCept SectRay::findIntercept(OpWinder* winder, OpEdge* test) {
	if (test->bounds.ltChoice(axis) > normal)
		return FindCept::ok;
	if (test->bounds.rbChoice(axis) < normal)
		return FindCept::ok;
	OpEdge* home = winder->home;
	if (test == home)
		return FindCept::ok;
	if (Unsortable::none != test->isUnsortable && Unsortable::tooManyTries != test->isUnsortable)
		return FindCept::unsortable;
	if (test->disabled)
		return FindCept::ok;
	bool uSectPair = test->isUnsectable() && home->isUnsectable() && test->isPal(home);
	if (uSectPair && !firstTry)
		return FindCept::unsectable;
	// !!! EXPERIMENT  try using segment's curve instead of edge curve
	//                 edge curve's control points, especially small ones, may magnify error
	//                 failed to switch over to segment everywhere, may explain why experiment failed
	OpRoots roots = test->curve.axisRayHit(axis, normal);  // get the normal at the intersect point	
	float root = OpNaN;
	float testXY = OpNaN;
	auto pushUsectDist = [this, test, &testXY, &root, uSectPair]() {
		if (uSectPair) {
			distances.emplace_back(test, testXY, root, false);
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
	bool reversed = tangent.dot(homeTangent) < 0;
	distances.emplace_back(test, testXY, root, reversed);
	sorted = false;
#if 0
	if (!test->coinPals.empty() && missingContour(winder, test))
		checkCoins = true;
	else if (!test->segment->coinContours.empty() && missingContour(winder, test->segment))
		checkCoins = true;
#endif
	if (!uSectPair && OpMath::Equal(testXY, homeCept, 
			home->context()->threshold().choice(perpendicular)))
		return FindCept::retry;  // e.g., testQuads1877923 has two small quads which just miss 
	return uSectPair ? FindCept::addPal : FindCept::ok;
}

bool SectRay::incomplete() const {
	for (const EdgePal& edgePal : distances) {
		OpEdge* test = edgePal.edge;
		if (test->ray.distances.empty())
			return true;
	}
	return false;
}

bool SectRay::missingContour(OpWinder* winder, OpEdge* test) const {
	for (CoinPal& coinPal : test->coinPals) {
		OpContour* contour = coinPal.opp->contour;
		if (!winder->targets.match(contour))
			return true;
	}
	return false;
}

bool SectRay::missingContour(OpWinder* winder, OpSegment* test) const {
	for (OpContour* contour : test->coinContours) {
		if (!winder->targets.match(contour))
			return true;
	}
	return false;
}

// returns true iff all pals of edge are in ray's distances
bool SectRay::sectsAllPals(const OpEdge* edge) const {
	unsigned found = 0;
	const std::vector<EdgePal>& ePals = edge->pals;
	OP_ASSERT(ePals.size());
	for (const auto& test : distances) {
		const OpEdge* tEdge = test.edge;
		if (ePals.end() != std::find_if(ePals.begin(), ePals.end(), 
				[tEdge](const EdgePal& dist) { return dist.edge == tEdge; })) {
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
	std::sort(distances.begin(), distances.end(), [](const EdgePal& s1, const EdgePal& s2) {
			return s1.cept < s2.cept || (s1.cept == s2.cept && s1.edge->id < s2.edge->id); });
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
		OpContext* contours = seg->contour->context;
		OpPtAliases& aliases = contours->aliases;
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

FoundIntercept OpWinder::findRayIntercept(OpVector homeTan, float normal, float homeCept) {
	SectRay& ray = home->ray;
	ray.homeTangent = homeTan;
	ray.normal = normal;
	ray.homeCept = homeCept;
	ray.axis = workingAxis;
	Axis perpendicular = !workingAxis;
	float mid = .5;
	float midEnd = .5;
	ray.homeT = OpMath::Ratio(home->startT, home->endT, home->center.t);
	// if find intercept fails, retry some number of times
	// if all retries fail, distinguish between failure cases
	//   if it failed because closest edge was too close, mark pair as unsectable
	ray.firstTry = true;
	do {
		ray.distances.clear();
		ray.distances.emplace_back(home, homeCept, ray.homeT, false);
		targets.reset();
		// start at edge with left equal to or left of center
		while (OpEdge* test = targets.next(homeCept)) {
			FindCept findCept = ray.findIntercept(this, test);
			if (FindCept::ok == findCept)
				continue;
			if (FindCept::retry == findCept)
				goto tryADifferentCenter;
			if (FindCept::addPal == findCept) {
				EdgePal& tDist = ray.distances.back();
				home->addPal(tDist);
				continue;
			}
			if (FindCept::unsortable == findCept)
				goto giveUp;
		}
//		if (ray.addCoinContours(this))  // unmatched coin was found in built distance array
//			continue;
		// start here;
		// if ray distance edge's ray is not computed, return and do that first
		if (ray.incomplete())
			return FoundIntercept::recurse;
		// check if ray distance relies on contour not in targets
		if (ray.addDependentContours(this))
			continue;
		if (ray.distances.size() <= 1) 
			return FoundIntercept::yes;
		if (ray.distances.front().edge == home)
			return FoundIntercept::yes;
		{
			RayOrder rayOrder = ray.checkOrder(home);
			if (RayOrder::ok == rayOrder)
				return FoundIntercept::yes;
			// !!! experiment: allow reversing ray edges that are before home
			if (RayOrder::unordered == rayOrder)
				return FoundIntercept::yes;
		}
	tryADifferentCenter:
		mid /= 2;
		midEnd = midEnd < .5 ? 1 - mid : mid;
		float middle = OpMath::Interp(home->bounds.ltChoice(workingAxis), 
				home->bounds.rbChoice(workingAxis), midEnd);
		float homeMidT = home->curve.center(workingAxis, middle);  // note: 0 to 1 on edge curve
		bool tooMany = mid <= interceptLimit;
		if (OpMath::IsNaN(homeMidT) || tooMany) {  // give it at most eight tries
			if (!home->isUnsectable())
				markUnsortable(tooMany ? Unsortable::tooManyTries : Unsortable::noMidT);
			break;	// give up
		}
		// if find ray intercept can't find, restart with new center, normal, distance, etc.
		// !!! result is not pinned to bounds
		ray.homeCept = homeCept = home->curve.ptAtT(homeMidT).choice(perpendicular);
		OP_ASSERT(!OpMath::IsNaN(homeCept));
		ray.normal = normal = home->curve.ptAtT(homeMidT).choice(workingAxis);
		ray.homeT = homeMidT;
		OP_ASSERT(!OpMath::IsNaN(normal));
	} while (true);
giveUp:
	// give up case: sort and go home
	ray.sort();
	return FoundIntercept::fail;	// nonfatal error (!!! give it a different name!)
}

void OpWinder::markUnsortable(Unsortable unsortable) {
	if (Axis::vertical == workingAxis)
		home->setUnsortable(unsortable);
	else {
		for (RayTarget& target : targets.t) {
			std::vector<OpEdge*>& inY = target.contour->inY;
			if (inY.end() != std::find(inY.begin(), inY.end(), home)) 
				goto foundInY;
		}
		home->setUnsortable(unsortable);
foundInY: ;
	}
	home->rayFail = Axis::vertical == workingAxis ? EdgeFail::vertical : EdgeFail::horizontal;
}

// pass array of edges in parameter; pass same to find ray intercept
ChainFail OpWinder::setSumChain() {
	// see if normal at center point is in direction of ray
	OP_ASSERT(!home->disabled);
	const OpSegment* edgeSeg = home->segment;
	OpVector rayLine = Axis::horizontal == workingAxis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	float midTest = home->center.t;
	OP_ASSERT(home->startT < midTest && midTest < home->endT);
	float normalLimit = home->segment->c.normalLimit();
	float midLo = midTest - home->startT;
	float midHi = home->endT - midTest;
	interceptLimit = home->segment->c.interceptLimit();
	OpVector homeTangent;
	// curve may be flat at center and curved at one end, so don't give up too soon (loop45964)
	auto testTangent = [&homeTangent, edgeSeg, rayLine, normalLimit](float testT) {
		homeTangent = edgeSeg->c.tangent(testT);
		float NxR = homeTangent.normalize().cross(rayLine);
		if (!OpMath::IsFinite(NxR))
			return ChainFail::normalizeOverflow;
		if (fabs(NxR) < normalLimit)
			return ChainFail::normalizeUnderflow;  // nonfatal error
		return ChainFail::none;
	};
	bool firstPass = true;  // don't check 'hi' and 'lo' first time: they are the same
	bool isLinear = home->curve.isLine();  // if curve is line, just check once
	ChainFail chainFail;
	do {  // check in both directions to see if, away from center, curve unflattens
		chainFail = testTangent(midTest);
		if (ChainFail::normalizeUnderflow != chainFail || isLinear)
			break;
		if (!firstPass) {
			chainFail = testTangent(home->endT - midHi);
			if (ChainFail::normalizeUnderflow != chainFail) {
				midTest = home->endT - midHi;
				break;
			}
		}
		midLo /= 2;  // godbolt.org says 'divide by two' is identical to 'multiply by 0.5'
		midTest = home->startT + midLo;
		midHi /= 2;
	} while (std::max(midLo, midHi) > interceptLimit);  // check both since center may not be mid t
	if (ChainFail::normalizeOverflow == chainFail)
		OP_DEBUG_FAIL(*home, chainFail);  // fatal error : cross product returned infinite / nan
	if (ChainFail::normalizeUnderflow == chainFail) {  // nonfatal error -- try vertical instead
		if (Axis::vertical == workingAxis)
			markUnsortable(Unsortable::rayTooShallow);
		return chainFail;
	}
#if 0  // !!! looks like bandaid if above code fails to detect shallow slope ...
	   //     document test and circumstance when this is needed
	float normal = home->center.pt.choice(workingAxis);
	if (normal == home->startPt().choice(workingAxis)
			|| normal == home->endPt().choice(workingAxis)) {
		markUnsortable(Unsortable::noNormal);
		return ChainFail::noNormal;  // nonfatal error
	}
#else
//	OpPoint midPt = edgeSeg->c.ptAtT(midTest);
	OpPoint midPt = home->curve.ptAtT((midTest - home->startT) / (home->endT - home->startT));
	float normal = midPt.choice(workingAxis);
#endif
	// intersect normal with every edge in the direction of ray until we run out 
	Axis perpendicular = !workingAxis;
	targets.build(this);
	float homeCept = midPt.choice(perpendicular);
	FoundIntercept foundIntercept = findRayIntercept(homeTangent, normal, homeCept);
	home->ray.sort();
	if (FoundIntercept::recurse == foundIntercept)	// some found edge is missing ray distances
		return ChainFail::recurse;
	if (FoundIntercept::fail == foundIntercept)
		return ChainFail::failIntercept;
	if (FoundIntercept::overflow == foundIntercept)
		return ChainFail::normalizeOverflow;
	return ChainFail::none;
}

ResolveWinding OpWinder::setWindingByDistance(OpContext* context) {
	// find edge; then walk backwards to first known sum 
	SectRay& ray = home->ray;
	OP_ASSERT(ray.distances.size());
	if (1 == ray.distances.size()) {
		OP_ASSERT(home == ray.distances[0].edge);
		if (home->isUnsectable())  // !!! move this to where unsectable is set?
			home->setUnsortable(Unsortable::homeUnsectable);
		else {
			OpWinding prev(home, WindingSum::dummy);
			// look at direction of edge relative to ray and figure winding/oppWinding contribution
			if (CalcFail::fail == home->addIfUR(ray.axis, ray.distances[0].edgeInsideT, &prev))
				home->setUnsortable(Unsortable::addCalcFail);
			else
				home->setSum(prev  OP_LINE_FILE_PARGS());
		}
		return ResolveWinding::resolved;
	}
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
		float threshold = edge->context()->threshold().choice(!ray.axis);  // use perpendicular
		do {
			const EdgePal& dist = ray.distances[sumIndex];
			OpEdge* previous = dist.edge;
			if (previous->isPal(edge))
				return true;
			if (lastIsEdge && OpMath::Equal(lastCept, dist.cept, threshold))
				return true;
			lastIsEdge = previous == edge;
			if (lastIsEdge) {
				if (OpMath::Equal(lastCept, dist.cept, threshold))
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
	OpContour* winderOwner = home->segment->contour->overlapOwner;
	OpWinding sumWinding(home, WindingSum::dummy);
	int sumIndex = (int) ray.distances.size();
	while (ray.distances[--sumIndex].edge != home) 
		OP_ASSERT(sumIndex > 0);
	float homeT = ray.distances[sumIndex].edgeInsideT;  // used by unsectable, later
	while (--sumIndex >= 0 && (anyPriorPal(ray.distances[sumIndex].edge, sumIndex) 
			|| !ray.distances[sumIndex].edge->sum.isSet() 
			|| RayOrder::unordered == ray.distances[sumIndex].rayOrder))
		;
	if (sumIndex > 0 && !home->isUnsectable() && EdgeFail::none == home->rayFail) {
		RayOrder rayOrder = ray.checkOrder(home);
		// !!! experiment: try allowing unordered outside of home to contribute normally to winding
		if (RayOrder::tooClose == rayOrder)
			return ResolveWinding::retry;
	}
	while (sumIndex >= 0 && RayOrder::unordered == ray.distances[sumIndex].rayOrder)
		--sumIndex;
	if (sumIndex >= 0) {
		EdgePal& sumDistance = ray.distances[sumIndex];
		OpEdge* sumEdge = sumDistance.edge;
		OP_ASSERT(!sumEdge->isUnsectable());
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
// !!! start here;
	// several bugs:
	//  doesn't detect if home/prior are both unsectable
	//  doesn't detect if home/prior have equal cept values
	//  doesn't detect endless swapping recursion
#if 1  // !!! if prior has not had its sum computed, and prior's edge run has entries, recurse
	OpEdge* prior;
	if (-1 == sumIndex) {
		prior = ray.distances[0].edge;
		if (!prior->sum.isSet() && prior->ray.canSetSum(prior)) {
			RayOrder rayOrder = prior->ray.checkOrder(prior);
			if (RayOrder::ok == rayOrder) {
				home = prior;
				setWindingByDistance(context);
				return ResolveWinding::recursed;
			}
		}
	}
#endif
	do {
		OP_ASSERT(sumIndex + 1 < (int) ray.distances.size());
		EdgePal& dist = ray.distances[++sumIndex];
		prior = dist.edge;
		if (home->isUnsectable() && (home == prior || home->isPal(prior)))
			break;
		NormalDirection normDir = prior->normalDirection(ray.axis, dist.edgeInsideT);
		if (NormalDirection::underflow == normDir) {
			prior->setUnsortable(Unsortable::underflow);
			continue;
		}
		if (NormalDirection::downLeft == normDir && !anyPriorPal(prior, sumIndex)
				&& RayOrder::unordered != ray.distances[sumIndex].rayOrder) {
			prior->setSum(sumWinding  OP_LINE_FILE_PARGS());
		}
		if (CalcFail::fail == prior->addSub(winderOwner, ray.axis, dist.edgeInsideT, &sumWinding)) // if d/l sub; if u/r add
			OP_DEBUG_FAIL(*prior, ResolveWinding::fail);
		if (NormalDirection::upRight == normDir && !anyPriorPal(prior, sumIndex)
				&& RayOrder::unordered != ray.distances[sumIndex].rayOrder) {
			prior->setSum(sumWinding  OP_LINE_FILE_PARGS());
		}
	} while (home != prior);
	if (!home->isUnsectable()) {
		if (!home->sum.isSet())
			home->setSum(sumWinding  OP_LINE_FILE_PARGS());
		return ResolveWinding::resolved;
	}
	// if home is unsectable, set its sum winding as if all of its pals' windings were a single edge
	OP_ASSERT(!home->many.isSet());
	// winding must be replaced by all unsectable windings -- however, other unsectables will want 
	//   to see the original winding. This is why 'many' is used. After all sums are computed
	//   replace winding with many.
	home->many.setWind(home->winding);
	for (const auto& pal : home->pals) {
		home->winding.move(context, pal.edge->winding, pal.reversed);
	}
	if (!home->winding.visible(context)) {
		home->setDisabled(OP_LINE_FILE_NPARGS());
//		home->segment->contour->isOpen = true;		// !!! may be required (wait for test case)
//		home->windPal = true;	// !!! doesn't appear to be necessary
	}
	if (CalcFail::fail == home->addIfUR(ray.axis, homeT, &sumWinding))
		home->setUnsortable(Unsortable::addCalcFail2);
	else
		home->setSum(sumWinding  OP_LINE_FILE_PARGS());
	std::swap(home->many, home->winding);  // restore winding, put total of pals in many
	return ResolveWinding::resolved;	   // (will copy many to winding after all many are found)
}

// think about how to write this so it sets the windings for each contour
// probably it should walk the edges in the contour, after setting the inX/inY to the 
// cache of sect edges held by the contour (or the 

FoundWindings OpWinder::setWindings(OpContext* context) {
	// test sum chain for correctness; recompute if prior or next are inconsistent
	for (Axis a : { Axis::horizontal, Axis::vertical }) {
		workingAxis = a;
		std::vector<OpEdge*> edges;
		for (OpContour* contour : context->contours) {
			std::vector<OpEdge*>& in = Axis::horizontal == workingAxis ? contour->inX : contour->inY;
			edges.insert(edges.end(), in.begin(), in.end());
		}
		std::sort(edges.begin(), edges.end(), [a](const OpEdge* s1, const OpEdge* s2) {
			return Axis::horizontal == a ? s1->bounds.left < s2->bounds.left
					: s1->bounds.top < s2->bounds.top;
		});
		for (size_t index = 0; index < edges.size(); ++index) {
			std::vector<OpEdge*> recurse = { edges[index] };
			do {
				home = recurse.back();
				recurse.pop_back();
				if (home->ray.distances.size() && EdgeFail::none == home->rayFail)
					continue;
				if (home->disabled)	// may not be visible in vertical pass
					continue;
				if (home->centerless)
					continue;
				if (EdgeFail::horizontal == home->rayFail && Axis::vertical == workingAxis)
					home->rayFail = EdgeFail::none;
				else if (Unsortable::none != home->isUnsortable)  // may be too small
					break;
				ChainFail chainFail = setSumChain();
				if (ChainFail::recurse == chainFail) {
					recurse.push_back(home);
					for (EdgePal& dist : home->ray.distances) {
						if (dist.edge->ray.distances.empty())
							recurse.push_back(dist.edge);
					}
				}
				if (ChainFail::normalizeOverflow == chainFail)
					OP_DEBUG_FAIL(*home, FoundWindings::fail);
			} while (!recurse.empty());
		}
	}
	for (auto contour : context->contours) {
		for (auto& segment : contour->segments) {
			for (auto& edge : segment.edges) {
				SectRay& ray = edge.ray;
				if (edge.disabled)
					continue;
				if (edge.isUnsectable())
					edge.markPals();
				else
					ray.addPals(&edge);
			}
		}
	}
	// if a pair of pals share an edge, this puts each in the other's pal list
	for (auto contour : context->contours) {
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
					pals.push_back(*reciprocal);
				}
			}
		}
	}
	// sort edges so that largest edges' winding sums are computed first
	// iterate all sect contours and mark them so they aren't inspected again
	// collect edges common to sect contours and compute their sums before moving to inner contours
	// then, iterate the edge size by perimeter
	context->setSortedBounds();
	for (auto sContour : context->sorted) {
		// if edge (later) has horizontal ray, use left most; if vertical, use top most
		auto gather = [sContour](Axis axis) {
			std::vector<OpEdge*> sectsByAxis;
			OP_ASSERT(!sContour->isSorted(axis));
			sContour->setSorted(axis);
			for (auto& segment : sContour->segments) {
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
			std::sort(sectsByAxis.begin(), sectsByAxis.end(), [](const auto& s1, const auto& s2) {
				return s1->bounds.perimeter() > s2->bounds.perimeter(); 
			} );
			return sectsByAxis;
		};
		std::vector<OpEdge*> sectsByX = gather(Axis::horizontal);
		std::vector<OpEdge*> sectsByY = gather(Axis::vertical);
		for (auto sectsBy : { sectsByX, sectsByY }) {
			for (auto edge : sectsBy) {
				if (edge->sum.isSet())
					continue;
				ResolveWinding resolveWinding;
				do {
					home = edge;
					resolveWinding = setWindingByDistance(context);
				} while (ResolveWinding::recursed == resolveWinding);
				if (ResolveWinding::retry == resolveWinding) {
					// if edges in ray are too close, look for a better spot
					workingAxis = home->ray.axis;
					setSumChain();
					// if looking again failed, and if horz axis, try vertical

					// if vertical failed, mark as unsectable
					resolveWinding = setWindingByDistance(context);
					OP_ASSERT(ResolveWinding::retry != resolveWinding);
				}
				if (ResolveWinding::fail == resolveWinding)
					OP_DEBUG_FAIL(*home, FoundWindings::fail);
			}
		}
	}
	for (auto contour : context->contours) {
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
