#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegment.h"
#include "PathOps.h"

OpJoiner::OpJoiner(OpContours& contours, OpOutPath& p)
	: path(p)
	, linkMatch(EdgeMatch::none)
	, linkPass(LinkPass::none) {
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
	if (!edge->winding.visible() || EdgeSum::unsortable == edge->sumType)
		return;
	if (edge->many.isSet())
		unsectInX.push_back(edge);
	else
		inX.push_back(edge);
}

void OpJoiner::addToLinkups(OpEdge* edge) {
	OP_ASSERT(!edge->isLoop());
    OpEdge* first = edge->advanceToEnd(EdgeMatch::start);
	OpEdge* next = first;
	OpEdge* last;
	do {
		OP_ASSERT(next->isActive());
		next->clearActiveAndPals();
		next->lastEdge = nullptr;
		next->inOutQueue = true;
		last = next;
		next = next->nextEdge;
	} while (next);
	first->lastEdge = last;
    linkups.push_back(first);
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
bool OpJoiner::detachIfLoop(OpEdge* edge, OpEdge** leftOvers) {
	std::vector<LoopCheck> edges;
	OpEdge* test = edge;
	// walk forwards to end, keeping one point per edge
	while (test) {
		edges.emplace_back(test, linkMatch);
		test = EdgeMatch::start == linkMatch ? test->nextEdge : test->priorEdge;
		if (edge == test)
			break;
	}
	if (edge == test) {	// if this forms a loop, there's nothing to detach, return success
		edge->output(path);
		return true;
	}
	// walk backwards to start
	std::sort(edges.begin(), edges.end());
	auto detachNext = [this, leftOvers](OpEdge* test, OpEdge* oppEdge) {
		if (OpEdge* detach = test->nextEdge) {
			detach->clearPriorEdge();
			*leftOvers = detach;
		}
		test->setNextEdge(oppEdge);
		oppEdge->setPriorEdge(test);
		test->output(path);
		return true;
	};
	auto detachPrior = [this, leftOvers](OpEdge* test, OpEdge* oppEdge) {
		if (OpEdge* detach = test->priorEdge) {
			detach->clearNextEdge();
			*leftOvers = detach;
		}
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
	}
	return false;
}

void OpJoiner::linkUnambiguous() {
    // match up edges that have only a single possible prior or next link, and add them to new list
    linkPass = LinkPass::unambiguous;
    for (auto& leftMost : inX) {
        if (!leftMost->winding.visible())
            continue;   // likely marked as part of a loop below
        if (!leftMost->isActive())  // check if already saved in linkups
            continue;
		OP_ASSERT(!leftMost->priorEdge);
		OP_ASSERT(!leftMost->nextEdge);
		do {
			leftMost->whichEnd = EdgeMatch::start;
			linkMatch = EdgeMatch::start;
			leftMost = linkUp(leftMost);
			if (!leftMost)
				break;
			OP_ASSERT(!leftMost->isLoop());
			linkMatch = EdgeMatch::end;
		} while ((leftMost = linkUp(leftMost)));
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
OpEdge* OpJoiner::linkUp(OpEdge* edge) {
	OP_ASSERT(!edge->isLoop(WhichLoop::next, LeadingLoop::will));
	std::vector<FoundEdge> edges;
	if (LinkPass::unsectInX == linkPass)
		edge->matchUnsectable(linkMatch, &unsectInX, edges);
	else {
		const OpSegment* segment = edge->segment;
		bool hasPal = segment->activeAtT(edge, linkMatch, edges, AllowReversal::no);
		hasPal |= segment->activeNeighbor(edge, linkMatch, edges);
		if (hasPal)
			edge->skipPals(linkMatch, edges);
	}
	if (1 != edges.size()) {
		if (EdgeMatch::start == linkMatch)
			return edge;  // 1) found multiple possibilities, try end
		addToLinkups(edge);
		return nullptr;  // 2) found multiple possibilities (end)
	}
	FoundEdge found = edges.front();
	OP_ASSERT(!found.edge->isLoop());
	edge->linkToEdge(found, linkMatch);
	OP_ASSERT(edge->whichPtT(linkMatch).pt == found.edge->flipPtT(linkMatch).pt);
	OpEdge* leftOvers = nullptr;
	if (detachIfLoop(edge, &leftOvers)) {
		if (leftOvers) {
			if (EdgeMatch::start == linkMatch)
				return leftOvers;	// 3) caller to extend leftover end
			addToLinkups(leftOvers);	// return front edge
		}
		return nullptr; // 4) found loop, nothing leftover; caller to move on to next edge
	}
	// move to the front or back edge depending on link match
	OpEdge* recurse = found.edge->advanceToEnd(linkMatch);
	return linkUp(recurse);	// 5)  recurse to extend prior or next
}

// at this point all singly linked edges have been found
// every active set of links at this point must form a loop
bool OpJoiner::matchLinks(OpEdge* edge) {
	OpEdge* lastEdge = edge->lastEdge;
	OP_ASSERT(lastEdge);
	OP_ASSERT(EdgeMatch::start == lastEdge->whichEnd || EdgeMatch::end == lastEdge->whichEnd);
	// count intersections equaling end
	// each intersection has zero, one, or two active edges
	std::vector<FoundEdge> found;
	const OpSegment* last = lastEdge->segment;
	// should be able to ignore result because pals have already been marked inactive
	(void) last->activeAtT(lastEdge, EdgeMatch::end, found, AllowReversal::yes);
	(void) last->activeNeighbor(edge, EdgeMatch::end, found);
	start here;
	// the only distance that matters is zero. We should never have unexplained gaps
	OpEdge* closest = nullptr;
	EdgeMatch closestEnd = EdgeMatch::none;
	if (!found.size()) {
		OpPoint matchPt = lastEdge->whichPtT(EdgeMatch::end).pt;
		float closestDistance = OpInfinity;
		for (size_t index = 0; index < linkups.size(); ++index) {
			OpEdge* linkup = linkups[index];
			OP_ASSERT(!linkup->priorEdge);
			if (lastEdge != linkup) {
				float distance = (linkup->whichPtT().pt - matchPt).length();
				if (closestDistance > distance) {
					found.clear();
					closestDistance = distance;
					closest = linkup;
				} else if (closestDistance == distance)
					found.emplace_back(linkup, EdgeMatch::none);	// keep ties
			}
			OP_ASSERT(linkup->lastEdge);
			if (lastEdge != linkup->lastEdge) {
				float distance = (linkup->lastEdge->whichPtT(EdgeMatch::end).pt - matchPt).length();
				if (closestDistance > distance) {
					found.clear();
					closestDistance = distance;
					closest = linkup->lastEdge;
					closestEnd = Opposite(linkup->lastEdge->whichEnd);
				} else if (closestDistance == distance)
					found.emplace_back(linkup->lastEdge, Opposite(linkup->lastEdge->whichEnd));	// keep ties
			}
		}
		if (!closest) {	// look for a run of unsectables that close the gap
			OP_ASSERT(!edge->priorEdge);
			OP_ASSERT(!lastEdge->nextEdge);
			if (OpEdge* result = linkUp(edge))
				return result;
		}
#if OP_DEBUG
		if (!closest) {
			focus(edge->id);
			oo(10);
			showPoints();
			showValues();
			OpDebugOut("");
		}
#endif
		OP_ASSERT(closest);
		found.emplace_back(closest, closestEnd);
	}
	OP_ASSERT(found.size());
	OpRect bestBounds;
	closest = nullptr;
	closestEnd = EdgeMatch::none;
	for (int trial = 0; !closest && trial < 2; ++trial) {
		for (const auto& foundOne : found) {
			OpEdge* oppEdge = foundOne.edge;
			// skip edges which flip the fill sum total, implying there is a third edge inbetween
			               // e.g., one if normals point to different fill sums       
			if (!trial && (((lastEdge->sum.sum() + oppEdge->sum.sum()) & 1) 
			// e.g., one if last start-to-end connects to found start-to-end (end connects to start)
					== (lastEdge->whichEnd == foundOne.whichEnd)))
				continue;
			// choose smallest closed loop -- this is an arbitrary choice
			OpPointBounds testBounds = edge->setLinkBounds();
			if (!edge->containsLink(oppEdge))
				testBounds.add(oppEdge->setLinkBounds());
			if (!(bestBounds.area() < testBounds.area())) {	// 'not' logic since best = NaN at first
				bestBounds = testBounds;
				closest = oppEdge;
				closestEnd = foundOne.whichEnd;
			}
		}
	}
	OP_ASSERT(closest); // !!! if found is not empty, but no edge has the right sum, choose one anyway?
	OP_ASSERT(EdgeMatch::end == closestEnd || closest->lastEdge);
	if (EdgeMatch::none != closestEnd)
		closest->setLinkDirection(closestEnd);
	OpEdge* clearClosest = closest;
	do {
		clearClosest->clearActiveAndPals();
		clearClosest = clearClosest->nextEdge;
	} while (clearClosest && clearClosest->isActive());
	OP_ASSERT(closest->whichPtT(EdgeMatch::start).pt == lastEdge->whichPtT(EdgeMatch::end).pt);
	closest->setPriorEdge(lastEdge);
	lastEdge->setNextEdge(closest);
	lastEdge = closest->setLastEdge(lastEdge);
	// delete 'closest' from linkups; entry no longer points to edge link head
	if (auto entry = std::find(linkups.begin(), linkups.end(), closest); linkups.end() != entry)
		linkups.erase(entry);
	OP_ASSERT(lastEdge);
	OpEdge* leftOvers = nullptr;
	linkMatch = EdgeMatch::start;	// since closest prior is set to last edge, use start
	if (detachIfLoop(closest, &leftOvers)) {
		if (leftOvers) {
			leftOvers->setLastEdge();
			leftOvers->setLinkActive();
			if (EdgeSum::unset == leftOvers->sumType)	// skip unsectable, unsortable (for now)
				addToLinkups(leftOvers);	// return front edge
		}
		return true; // found loop
	}
	return matchLinks(edge);
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

// !!! is sorting helpful? would unsorted be just as good?
void OpJoiner::sort() {
	std::sort(inX.begin(), inX.end(), compareXBox);
	std::sort(unsectInX.begin(), unsectInX.end(), compareXBox);
}
