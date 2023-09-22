// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegment.h"
#include "PathOps.h"

OpJoiner::OpJoiner(OpContours& contours, OpOutPath& p)
	: path(p)
	, linkMatch(EdgeMatch::none)
	, linkPass(LinkPass::none)
	, disabledBuilt(false)
	, disabledPalsBuilt(false) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				addEdge(&edge);
			}
		}
	}
	sort();
}

// look for active unsectable edges that match
// We could accumulate more than one path until we know which one is best.
// Instead, we'll add one but compare it and its pals to see if the path closes.
// If a pal closes, then rewind until the pal chain and given chain match and swap
//  with the pal chain to close the path.
// returns true if more than one unsectable was found
bool OpJoiner::activeUnsectable(const OpEdge* edge, EdgeMatch match, 
        std::vector<FoundEdge>& oppEdges) {
	// !!! incomplete
	return false;
}

void OpJoiner::addEdge(OpEdge* edge) {
	OP_ASSERT(!edge->debugIsLoop());
	if (edge->disabled)
		return;
	if (edge->unsortable)
		unsortables.push_back(edge);
	else if (edge->pals.size())
		unsectByArea.push_back(edge);
	else
		byArea.push_back(edge);
}

void OpJoiner::addToLinkups(OpEdge* edge) {
	OP_ASSERT(!edge->debugIsLoop());
    OpEdge* first = edge->advanceToEnd(EdgeMatch::start);
	OpEdge* next = first;
	OpEdge* last;
	do {
		if (LinkPass::unambiguous == linkPass) {
			OP_ASSERT(next->isActive());
			next->clearActiveAndPals(ZeroReason::none);
		}
		next->lastEdge = nullptr;
		next->inLinkups = true;
		last = next;
		next = next->nextEdge;
	} while (next);
	first->lastEdge = last;
	first->setLinkBounds();
    linkups.l.push_back(first);
}

void OpJoiner::buildDisabled(OpContours& contours) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				if (edge.disabled && !edge.unsortable && !edge.pals.size())
					disabled.push_back(&edge);
			}
		}
	}
	disabledBuilt = true;
}

void OpJoiner::buildDisabledPals(OpContours& contours) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			for (auto& edge : segment.edges) {
				if (edge.disabled && !edge.unsortable && edge.pals.size() && !edge.inOutput)
					disabledPals.push_back(&edge);
			}
		}
	}
	disabledPalsBuilt = true;
}

struct LoopCheck {
	LoopCheck(OpEdge* e, EdgeMatch match) 
		: edge(e) {
		pt = e->flipPtT(match).pt;
	}

	bool operator<(const LoopCheck& rh) const {
		return pt.x < rh.pt.x || (pt.x == rh.pt.x && pt.y < rh.pt.y);
	}

	OpEdge* edge;
	OpPoint pt;
};

// iterate edges to see some pt forms a loop
// if so, detach remaining chain and close loop
// check if any points in next links are in previous links
// !!! TODO : find direction of loop at add 'reverse' param to output if needed
//     direction should consider whether edge normal points to inside or outside
bool OpJoiner::detachIfLoop(OpEdge* edge) {
	std::vector<LoopCheck> edges;
	OpEdge* test = edge;
	// walk forwards to end, keeping one point per edge
	OP_ASSERT(!edge->debugIsLoop());
	while (test) {
		if (edges.end() != std::find_if(edges.begin(), edges.end(), 
				[&test](const LoopCheck& check) {
			return check.edge == test; } )) {
			break;
		}
		edges.emplace_back(test, linkMatch);
		test = EdgeMatch::start == linkMatch ? test->nextEdge : test->priorEdge;
		if (edge == test)
			break;
	}
	if (edge == test) {	// if this forms a loop, there's nothing to detach, return success
		edge->output(path);
#if OP_DEBUG
		const OpEdge* firstEdge = edge;
		do {
			edge->debugOutPath = path.debugID;
			edge = edge->nextEdge;
		} while (firstEdge != edge);
		path.debugNextID(edge);
#endif
		return true;
	}
	// walk backwards to start
	std::sort(edges.begin(), edges.end());
	auto detachEdge = [this](OpEdge* edge, EdgeMatch match) {
		if (OpEdge* detach = EdgeMatch::start == match ? edge->priorEdge : edge->nextEdge) {
			EdgeMatch::start == match ? detach->clearNextEdge() : detach->clearPriorEdge();
			if (!detach->unsortable || detach->nextEdge)
				addToLinkups(detach);	// return front edge
			else
				; // OP_ASSERT(!detach->priorEdge);  // triggered by fuzz763_1 -- is fix needed?
		}
	};
	auto detachNext = [this, detachEdge](OpEdge* test, OpEdge* oppEdge) {
		detachEdge(test, EdgeMatch::end);
		detachEdge(oppEdge, EdgeMatch::start);
		test->setNextEdge(oppEdge);
		oppEdge->setPriorEdge(test);
		test->output(path);
		return true;
	};
	auto detachPrior = [this, detachEdge](OpEdge* test, OpEdge* oppEdge) {
		detachEdge(test, EdgeMatch::start);
		detachEdge(oppEdge, EdgeMatch::end);
		test->setPriorEdge(oppEdge);
		oppEdge->setNextEdge(test);
		test->output(path);
		return true;
	};
	test = edge;
	while ((test = (EdgeMatch::start == linkMatch ? test->priorEdge : test->nextEdge)) && edge != test) {
		LoopCheck testCheck(test, Opposite(linkMatch));
		if (auto bound = std::lower_bound(edges.begin(), edges.end(), testCheck); 
				bound != edges.end() && bound->pt == testCheck.pt)
			return EdgeMatch::start == linkMatch ? detachNext(bound->edge, test) : 
					detachPrior(bound->edge, test);
#if 0
		// wait until loop fails to form because wrong pal was chosen before coding this
		for (auto pal : test->pals) {
			LoopCheck palCheck(pal, Opposite(linkMatch));
			if (auto bound = std::lower_bound(edges.begin(), edges.end(), palCheck);
					bound != edges.end() && !(palCheck < edges.front())) {
				OP_ASSERT(0);
				// If opp edge does not form a loop but pal does,
				// probably need to replace opp with pal.
				// Wait for this to happen for realsies before writing code to handle it.
				return EdgeMatch::start == linkMatch ? detachNext(bound->edge, test) :
						detachPrior(bound->edge, test);
			}
		}
#endif
	}
	return false;
}

bool OpJoiner::linkRemaining() {
	linkPass = LinkPass::unsectInX;
	// match links may add or remove from link ups. Iterate as long as link ups is not empty
	for (auto edge : linkups.l) {
		edge->setLinkBounds();
	}
	OP_DEBUG_CODE(int debugLoopCounter = 0);
    while (linkups.l.size()) {
		// sort to process largest first
		// !!! could optimize to avoid search, but for now, this is the simplest
		linkups.sort();
		OpEdge* lastEdge;
		for (;;) {
			lastEdge = linkups.l.back();
			if (!lastEdge->disabled)  // may be pal whose partner has already been added (loops44i)
				break;
			linkups.l.pop_back();
			if (!linkups.l.size())
				return true;
		}
        if (!matchLinks(lastEdge, true))
			return false;
		OP_DEBUG_CODE(if (++debugLoopCounter < 0) OpDebugOut(""));
    }
	return true;
}

void OpJoiner::linkUnambiguous() {
    // match up edges that have only a single possible prior or next link, and add them to new list
    linkPass = LinkPass::unambiguous;
	OP_DEBUG_VALIDATE_CODE(debugValidate());
    for (auto& edge : byArea) {
		if (edge->disabled)
            continue;   // likely marked as part of a loop below
        if (!edge->isActive())  // check if already saved in linkups
            continue;
		OP_ASSERT(!edge->priorEdge);
		OP_ASSERT(!edge->nextEdge);
		edge->whichEnd = EdgeMatch::start;
		linkMatch = EdgeMatch::start;
		if (!linkUp(edge))
			continue;
		OP_DEBUG_VALIDATE_CODE(debugValidate());
		linkMatch = EdgeMatch::end;
		(void) linkUp(edge->setLastEdge(nullptr));
		OP_DEBUG_VALIDATE_CODE(debugValidate());
    }
}

/* relationship between prev/this/next and whichEnd: (start, end)
   a and b represent points that match; ? represents the other nonmatching end
   prev: (?, a) which:start		this: (a, b) which:start		next: (b, ?) which: start
   prev: (a, ?) which:end		this: (a, b) which:start		next: (b, ?) which: start
   prev: (?, b) which:start		this: (a, b) which:end		    next: (a, ?) which: start
   prev: (b, ?) which:end		this: (a, b) which:end  		next: (a, ?) which: start
   prev: (?, a) which:start		this: (a, b) which:start		next: (?, b) which: end		etc...
*/
// caller clears active flag
// parameter match determines whether link is looked for prior to, or next to edge
// links a single edge or (link of edges) with another edge
// first pass: only allow unambiguous connections; only one choice, matching zero side, etc.
// second pass: check for unambiuous, then allow reversing, pick smallest area, etc.
bool OpJoiner::linkUp(OpEdge* edge) {
	std::vector<FoundEdge> edges;
	OP_ASSERT(!edge->debugIsLoop(WhichLoop::next, LeadingLoop::will));
	const OpSegment* segment = edge->segment;
	bool hasPal = segment->activeAtT(edge, linkMatch, edges);
	hasPal |= segment->activeNeighbor(edge, linkMatch, edges);
	// skip pals should choose the pal that minimizes the output path area
	// if there's not enough info here to do that, the pal choice should be reconsidered
	//   when match links is called
	// !!! maybe the right choice here is the wrong choice later?!
	if (hasPal) {
		// if edges[x] has pals and pal is in linkups, remove edges[x]
		edge->skipPals(linkMatch, edges);
	}
	if (1 != edges.size() || !edges[0].edge->isActive()) {
		if (EdgeMatch::start == linkMatch)
			return true;  // 1) found multiple possibilities, try end
		addToLinkups(edge);
		return false;  // 2) found multiple possibilities (end)
	}
	FoundEdge found = edges.front();
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	edge->linkToEdge(found, linkMatch);
	OP_ASSERT(edge->whichPtT(linkMatch).pt == found.edge->flipPtT(linkMatch).pt);
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	if (detachIfLoop(edge))
		return false; // 4) found loop, nothing leftover; caller to move on to next edge
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	// move to the front or back edge depending on link match
	OpEdge* recurse = found.edge->advanceToEnd(linkMatch);
	return linkUp(recurse);	// 5)  recurse to extend prior or next
}

void OpJoiner::matchLeftover(OpPoint matchPt, const OpEdge* links, 
		const std::vector<OpEdge*>& leftovers, std::vector<FoundEdge>& edges) {
	for (OpEdge* nosort : leftovers) {
		if (nosort->inOutput || nosort->inLinkups)
			continue;
		if (links->hasLinkTo(nosort))
			continue;
		if (matchPt == nosort->start.pt)
			edges.emplace_back(nosort, EdgeMatch::start);
		if (matchPt == nosort->end.pt)
			edges.emplace_back(nosort, EdgeMatch::end);
	}
}

// at this point all singly linked edges have been found
// every active set of links at this point must form a loop
bool OpJoiner::matchLinks(OpEdge* edge, bool popLast) {
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	OpEdge* lastEdge = edge->lastEdge;
	OP_ASSERT(lastEdge);
	OP_ASSERT(EdgeMatch::start == lastEdge->whichEnd || EdgeMatch::end == lastEdge->whichEnd);
	// count intersections equaling end
	// each intersection has zero, one, or two active edges
	std::vector<FoundEdge> found;
	// the only distance that matters is zero. We should never have unexplained gaps
	OpPoint matchPt = lastEdge->whichPtT(EdgeMatch::end).pt;
	// pop last allows first attempt for edge to scan itself, in case it is an unclosed loop
	for (size_t index = 0; index < linkups.l.size(); ++index) {
		OpEdge* linkup = linkups.l[index];
		OP_ASSERT(!linkup->priorEdge);
		OP_ASSERT(!linkup->debugIsLoop());
		if (lastEdge != linkup && linkup->whichPtT().pt == matchPt)
			found.emplace_back(linkup, linkup->whichEnd, index);
		OpEdge* lastLink = linkup->lastEdge;
		OP_ASSERT(lastLink);
		if (lastEdge != lastLink && lastLink->whichPtT(EdgeMatch::end).pt == matchPt)
			found.emplace_back(lastLink, Opposite(lastLink->whichEnd), index);
	}
	if (popLast)
		linkups.l.pop_back();
	// look for (a run of) unsectables / unsortables that close the gap
	OP_ASSERT(!edge->priorEdge);
	OP_ASSERT(!lastEdge->nextEdge);
	// don't match unsectable with a pal that is already in edge link list
	lastEdge->matchUnsectable(EdgeMatch::end, unsectByArea, found, false);
	OP_ASSERT(!lastEdge->debugIsLoop());
	OP_ASSERT(!found.size() || !found.back().edge->debugIsLoop());
	// if nothing found, look for member of linkups that ends in a pal of lastEdge
	// figure out how to write (clear) code to recursively remove an edge, fixing up prior/next/last
	if (lastEdge->pals.size()) {
		for (size_t index = 0; index < linkups.l.size(); ++index) {
			OpEdge* link = linkups.l[index];
			OP_ASSERT(!link->debugIsLoop());
			if (lastEdge->isPal(link) && link->nextEdge 
					&& link->whichPtT(EdgeMatch::end).pt == matchPt) {
				// if found edge is not best, found needs to be added back to linkups.l
				OpEdge* nextLink = link->nextEdge;
				nextLink->setPriorEdge(nullptr);
				nextLink->lastEdge = link->lastEdge;
				link->setNextEdge(nullptr);
				link->lastEdge = link;
				OP_ASSERT(!nextLink->debugIsLoop());
				found.emplace_back(nextLink, nextLink->whichEnd);
				found.back().addBack = true;
				OP_ASSERT(!found.size() || !found.back().edge->debugIsLoop());
			} else {
				OpEdge* linkLast = link->lastEdge;
				OP_ASSERT(linkLast);
				if (lastEdge->isPal(linkLast) && linkLast != link
						&& linkLast->whichPtT(EdgeMatch::start).pt == matchPt) {
					OpEdge* lastPrior = linkLast->priorEdge;
					lastPrior->setNextEdge(nullptr);
					link->lastEdge = lastPrior;
					linkLast->setPriorEdge(nullptr);
					OP_ASSERT(0);  // !!! isn't this supposed to put link in found?
				}
			}
		}
	}
	if (!found.size()) {
		matchLeftover(matchPt, edge, unsortables, found);
		OP_ASSERT(!lastEdge->debugIsLoop());
		OP_ASSERT(!found.size() || !found.back().edge->debugIsLoop());
//	}
	// look for disabled pal that is not in output, and completes loop (cubics14d)
//	if (!found.size()) {
		if (!disabledPalsBuilt)
			buildDisabledPals(*edge->segment->contour->contours);
		matchLeftover(matchPt, edge, disabledPals, found);
		OP_ASSERT(!found.size() || !found.back().edge->debugIsLoop());
	}
#if 1
	if (!found.size()) {
		// its possible that the intersections for very very small edges were missed (skpadspert_net23)
		// before going down the disabled rabbit hole, see if the is a small gap that can be closed
		// look for a pair of intersections with different pt values, but the same t value in 
		//   lastEdge's segment
		// also look for an intersection where the points don't match
		auto checkMissed = [&found, matchPt](OpEdge* test, EdgeMatch whichEnd) {
			float testT = test->whichPtT(whichEnd).t;
			OpContour* contour = test->segment->contour;
			OpIntersection* last = nullptr;
			for (auto sect : test->segment->sects.i) {
				if (sect->ptT.t != testT)
					continue;
				OpIntersection* opp = sect->opp;
				if (sect->ptT.pt != opp->ptT.pt) {
					OpEdge* filler = EdgeMatch::start == whichEnd ? contour->addFiller(sect, opp) 
							: contour->addFiller(opp, sect);
					if (!filler)
						return false;
					found.emplace_back(filler, whichEnd);
				}
				if (!last) {
					last = sect;
					continue;
				}
				if (last->ptT.pt == sect->ptT.pt)
					continue;
				if (matchPt == last->ptT.pt)
					std::swap(last, sect);
				else if (matchPt != sect->ptT.pt)
					continue;
				OpEdge* filler = EdgeMatch::start == whichEnd ? contour->addFiller(sect, last) 
						: contour->addFiller(last, sect);
					if (!filler)
						return false;
				found.emplace_back(filler, whichEnd);
			}
			return true;
		};
//		if (!checkMissed(edge, EdgeMatch::start))  // odd code: expect to only match lastEdge end...
//			return false;
		if (!checkMissed(lastEdge, EdgeMatch::end))
			return false;
	}
#endif
	// if there's no remaining active edges in linkups or unsectables, just close what's left (loops63i)
	auto unsortableCount = [this](const auto edge) {
		return unsortables.size() - edge->countUnsortable();
	};
	if (!found.size() && !linkups.l.size() && !unsortableCount(edge)) {
		OpContour* contour = edge->segment->contour;
		OpIntersection* sect = nullptr;
		for (auto test : edge->segment->sects.i) {
			if (test->ptT != edge->whichPtT())
				continue;
			sect = test;
			break;
		}
		OpIntersection* last = nullptr;
		for (auto test : lastEdge->segment->sects.i) {
			if (test->ptT != lastEdge->whichPtT(EdgeMatch::end))
				continue;
			last = test;
			break;
		}
		OpEdge* filler = contour->addFiller(last, sect);
		if (!filler)
			return false;
		found.emplace_back(filler, EdgeMatch::start);
	}
#if 0
	// if nothing found to this point, see if forcing a very small edge will fill the bill (cubics10u)
	if (!found.size()) {
		OpIntersection* lastEnd = lastEdge->segment->sects.i.back();
		if (edge->whichPtT().pt.isNearly(matchPt)) {
			OpIntersection* edgeStart = edge->segment->sects.i.front();
			OpEdge* filler = edge->segment->contour->addFiller(edgeStart, lastEnd);
			if (!filler)
				return false;
			found.emplace_back(filler, EdgeMatch::end);
		}
		for (size_t index = 0; index < linkups.l.size(); ++index) {
			OpEdge* linkup = linkups.l[index];
			OP_ASSERT(!linkup->priorEdge);
			OP_ASSERT(!linkup->debugIsLoop());
			if (lastEdge != linkup && linkup->whichPtT().pt.isNearly(matchPt)) {
				OpIntersection* linkStart = linkup->segment->sects.i.front();
				OpEdge* filler = edge->segment->contour->addFiller(linkStart, lastEnd);
				if (!filler)
					return false;
				found.emplace_back(filler, EdgeMatch::end);
			}
			OpEdge* lastLink = linkup->lastEdge;
			OP_ASSERT(lastLink);
			if (lastEdge != lastLink && lastLink->whichPtT(EdgeMatch::end).pt == matchPt)
				found.emplace_back(lastLink, Opposite(lastLink->whichEnd, index));
		}
	}

#endif
	if (!found.size())
		lastEdge->matchUnsectable(EdgeMatch::end, unsectByArea, found, true);
	// look for a disabled edge that closes the gap
	// it's likely that this edge is very small, but don't know how to quantify that (yet)
	if (!found.size()) {
		if (!disabledBuilt)
			buildDisabled(*edge->segment->contour->contours);
		matchLeftover(matchPt, edge, disabled, found);
		OP_ASSERT(!found.size() || !found.back().edge->debugIsLoop());
	}
	if (!found.size() && edge->between)	// !!! if this is all that's left, drop it on the floor?
		return true;
#if OP_DEBUG_IMAGE
	if (!found.size()) {
		focus(edge->id);
		oo(10);
		showPoints();
		showValues();
		OpDebugOut("");  // allows setting a breakpoint when joining edge isn't found
	}
#endif
	OP_ASSERT(found.size());
	// if more than one was found, check to see if selected, one makes a complete loop
	// additionally, store in found how far end is from loop
	OpRect bestBounds;
	FoundEdge smallest = found.front();
	if (found.size() > 1) {
		for (auto& foundOne : found) {
			OpEdge* oppEdge = foundOne.edge;
			foundOne.connects = oppEdge->pals.size() || oppEdge->disabled 
					|| (((lastEdge->sum.sum() + oppEdge->sum.sum()) & 1) 
			// e.g., one if last start-to-end connects to found start-to-end (end connects to start)
					== (lastEdge->whichEnd != foundOne.whichEnd));
			// if opp edge contains member of linkups via links, detach from this
			// but remember to reattach if candidate is not best
			if (oppEdge->setLastLink(foundOne.whichEnd) && foundOne.index >= 0) {
				oppEdge->setLinkBounds();
				linkups.l[foundOne.index] = oppEdge;  // link start changed
			}
			for (const OpEdge* eTest = edge; eTest != nullptr; eTest = eTest->nextEdge) {
				for (const OpEdge* fTst = foundOne.edge; fTst != nullptr; fTst = fTst->nextEdge) {
					// look for eTest whichEnd (start) equalling fTst opposite whichEnd (end)
					if (eTest->whichPtT(EdgeMatch::start).pt != fTst->whichPtT(EdgeMatch::end).pt)
						continue;
					foundOne.loops = true;
					goto done;
				}
			}
		done:
			;
		}
		for (int trial = 0; !bestBounds.isFinite() && trial < 2; ++trial) {
			for (const auto& foundOne : found) {
				OpEdge* oppEdge = foundOne.edge;
				// skip edges which flip the fill sum total, implying there is a third edge inbetween
							   // e.g., one if normals point to different fill sums       
				if (!trial && !foundOne.connects)
					continue;
				// choose smallest closed loop -- this is an arbitrary choice
				OpPointBounds testBounds = edge->advanceToEnd(EdgeMatch::start)->setLinkBounds();
				if (!edge->containsLink(oppEdge)) {
					OpEdge* firstEdge = oppEdge;
					while (firstEdge->priorEdge)
						firstEdge = firstEdge->priorEdge;
					testBounds.add(foundOne.index > 0 ? firstEdge->setLinkBounds() : firstEdge->ptBounds);
				}
				if (smallest.loops < foundOne.loops)
					smallest = foundOne;
				else if (smallest.loops == foundOne.loops &&
					!(bestBounds.perimeter() < testBounds.perimeter())) {	// 'not' logic since best = NaN at first
					if (bestBounds.perimeter() == testBounds.perimeter()) {
						// see which end is closer to desired close
						OpPoint start = edge->whichPtT(EdgeMatch::start).pt;
						float testDistance = (start 
								- foundOne.edge->whichPtT(EdgeMatch::end).pt).lengthSquared();
						float bestDistance = (start
								- smallest.edge->whichPtT(EdgeMatch::end).pt).lengthSquared();
						if (testDistance > bestDistance)
							continue;
						// !!! if test equals best, do we need another way to pick the better one?
					}
					bestBounds = testBounds;
					smallest = foundOne;
				}
			}
		}
	}
	// add back found as needed
	for (const auto& foundOne : found) {
		if (foundOne.edge != smallest.edge && foundOne.addBack)
			linkups.l.push_back(foundOne.edge);
	}
	OP_ASSERT(smallest.edge); // !!! if found is not empty, but no edge has the right sum, choose one anyway?
	OpEdge* best = smallest.edge;
	(void) best->setLastLink(smallest.whichEnd);  // make edge suitable for linking to a chain
	if (!best->containsLink(lastEdge)) {
		OP_ASSERT(!best->debugIsLoop());
		OP_ASSERT(!lastEdge->debugIsLoop());
		OP_ASSERT(best->whichPtT(EdgeMatch::start).pt == lastEdge->whichPtT(EdgeMatch::end).pt);
		best->setPriorEdge(lastEdge);
		lastEdge->setNextEdge(best);
		OP_ASSERT(!lastEdge->debugIsLoop());
		lastEdge = best->setLastEdge(lastEdge);
		OP_ASSERT(lastEdge);
	// delete 'smallest.edge' from linkups; entry no longer points to edge link head
		if (smallest.index >= 0) {
			OP_ASSERT((unsigned) smallest.index < linkups.l.size());
			linkups.l.erase(linkups.l.begin() + smallest.index);
		}
	}
	if (best->disabled) {
		auto disabledPos = std::find(disabled.begin(), disabled.end(), best);
		if (disabled.end() != disabledPos)
			disabled.erase(disabledPos);
	}
	// if there is a loop, remove entries in link ups which are output
	linkMatch = EdgeMatch::start;	// since closest prior is set to last edge, use start
	if (detachIfLoop(best))
		return true; // found loop
	edge = edge->advanceToEnd(EdgeMatch::start);
#if OP_DEBUG
	auto index = std::find(debugTrack.begin(), debugTrack.end(), edge);
	if (debugTrack.end() != index) {
		auto oldCount = debugLinks[index - debugTrack.begin()];
		OP_ASSERT(oldCount != edge->debugLinkCount());
	}
	debugTrack.push_back(edge);
	debugLinks.push_back(edge->debugLinkCount());
#endif
	bool result = matchLinks(edge, false);
	OP_DEBUG_CODE(debugTrack.pop_back());
	OP_DEBUG_CODE(debugLinks.pop_back());
	return result;
}

static bool compareSize(const OpEdge* s1, const OpEdge* s2) {
	const OpRect& r1 = s1->ptBounds;
	const OpRect& r2 = s2->ptBounds;
	return r1.width() + r1.height() > r2.width() + r2.height();
}

// sort by size so that tiny edges with poor winding don't run the show
void OpJoiner::sort() {
	std::sort(byArea.begin(), byArea.end(), compareSize);
	std::sort(unsectByArea.begin(), unsectByArea.end(), compareSize);
}

// sort by size to process largest (tail) first
// sort should consider all edges in link
void LinkUps::sort() {
	std::sort(l.begin(), l.end(), [](const auto& s1, const auto& s2) {
		return s1->linkBounds.perimeter() < s2->linkBounds.perimeter(); 
	} );
}