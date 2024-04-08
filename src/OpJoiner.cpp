// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegment.h"
#include "PathOps.h"

void OpLimb::add(OpTree& tree, OpEdge* test, EdgeMatch m, LimbType limbType, size_t limbIndex,
		OpEdge* otherEnd) {
	OP_ASSERT(!test->disabled || test->pals.size() || LimbType::disabled <= limbType);
	OP_ASSERT(!test->hasLinkTo(m) || test->pals.size() || test->unsortable);
	if (test->whichPtT(m).pt != lastPt)
		return;
	if (test->visited)
		return;
	test->visited = true;
	const OpEdge* last = tree.edge->lastEdge;
	OP_ASSERT(last);
	// OP_ASSERT(!test->isPal(last) || LimbType::linked != limbType);  // breaks pentrek10
	// Edge direction and winding are tricky (see description at wind zero declaration.)
	// For first edge (and its last) in storage: if which end is 'end', its wind zero is reversed.
	// 'Test' may need to be reversed to connect, and 'm' may be either end. Wind zero in both cases
	// is computed for the unreversed orientation.
	if (!last->unsortable && LimbType::linked == limbType && !test->unsortable) {
        WindZero zeroSide = test->windZero;
		// if last which end is end, flip last's wind zero (for comparsion, flip zero side);
		// if test m is end, flip zero side; if test which is end, flip zero side
        if ((last->which() == EdgeMatch::start) != (test->which() == m))
            zeroSide = !zeroSide;
        if (last->windZero != zeroSide)
			return;
	}
	OpPointBounds childBounds = test->lastEdge ? test->linkBounds : 
			otherEnd ? otherEnd->linkBounds : test->ptBounds;
	if (parent)  // if not trunk
		childBounds.add(bounds);
	// if this edge added to limb bounds makes perimeter larger than best, skip
	// !!! are their cases where smallest perimeter is not the best test?
	// first, multiple edges with the same start and end point may share perimeters
	// is larger perimeter more desirable if it avoids enclosing another contour?
	// should this use the ray edges to see if test is closer to test edge?
	// note that best may not have ray to edge; e.g., outline of 'O' (edge contains inner contour)
	if (childBounds.perimeter() > tree.bestPerimeter)
		return;
	OpContours& contours = *tree.contour.contours;
	OpLimb* branch = contours.allocateLimb(tree);
	branch->set(tree, test, this, m, limbType, limbIndex, otherEnd, &childBounds);
#if OP_DEBUG
	debugBranches.push_back(branch);
	tree.debugLimbs.push_back(branch);
#endif
}

void OpLimb::set(OpTree& tree, OpEdge* test, const OpLimb* p, EdgeMatch m, LimbType l, 
		size_t index, OpEdge* otherEnd, const OpPointBounds* childBounds) {
	edge = test;
	parent = p;
	linkedIndex = (uint32_t) index;
	match = m;
	type = l;
	if (childBounds)
		bounds = *childBounds;
	if (LimbType::linked != type && LimbType::miswound != type) {
		lastLimb = edge;
		lastPt = lastLimb->whichPtT(!match).pt;
	} else if (EdgeMatch::start == match) {
		lastLimb = edge->lastEdge;
		lastPt = lastLimb->whichPtT(EdgeMatch::end).pt;
	} else {
		lastLimb = otherEnd;
		lastPt = lastLimb->whichPtT(EdgeMatch::start).pt;
	}
	looped = childBounds ? tree.firstPt == lastPt : false;
	if (looped && tree.bestPerimeter > bounds.perimeter()) {
		tree.bestPerimeter = bounds.perimeter();
		tree.bestLimb = this;
	}
	OP_DEBUG_CODE(debugID = test->segment->nextID());
}

void OpLimb::foreach(OpJoiner& join, OpTree& tree, LimbType limbType) {
	if (looped)  // triggered when walking children of trunk 
		return;
	size_t linkupsSize = join.linkups.l.size();
	LimbType linkedLimb = LimbType::miswound <= limbType ? LimbType::miswound : LimbType::linked;
	for (unsigned index = 0; index < linkupsSize; ++index) {
		OpEdge* test = join.linkups.l[index];
		if (test->disabled)
			continue;
		add(tree, test, EdgeMatch::start, linkedLimb, index);
		add(tree, test->lastEdge, EdgeMatch::end, linkedLimb, index, test);
	}
	if (LimbType::linked == limbType)
		return;
	for (const std::vector<OpEdge*>& edges : { join.unsectByArea, join.unsortables } ) {
		for (OpEdge* test : edges) {
			if (test->inLinkups)
				continue;
			add(tree, test, EdgeMatch::start, LimbType::unlinked);
			add(tree, test, EdgeMatch::end, LimbType::unlinked);
		}
	}
	if (LimbType::unlinked == limbType)
		return;
	if (!join.disabledBuilt) {
		join.buildDisabled(*tree.contour.contours);
		for (OpEdge* test : join.disabled) {
			test->unlink();
		}
	}
	for (OpEdge* test : join.disabled) {
		add(tree, test, EdgeMatch::start, LimbType::disabled);
		add(tree, test, EdgeMatch::end, LimbType::disabled);
	}
	if (LimbType::disabled == limbType)
		return;
	if (!join.disabledPalsBuilt)  {
		join.buildDisabledPals(*tree.contour.contours);
		for (OpEdge* test : join.disabledPals) {
			test->unlink();
		}
	}
	for (OpEdge* test : join.disabledPals) {
		add(tree, test, EdgeMatch::start, LimbType::disabledPals);
		add(tree, test, EdgeMatch::end, LimbType::disabledPals);
	}
	if (LimbType::disabledPals == limbType)
		return;
	if (LimbType::miswound == limbType)
		return; 
	// add the smallest gap of: for edge start, and for each end in link ups
	OpVector toStart = lastPt - tree.firstPt;
	// !!! eventually, keep track of gaps to all available edges; for now, only look at loop close
	float lenSq = toStart.lengthSquared();
	if (tree.bestDistance > lenSq) {
		tree.bestDistance = lenSq;
		tree.bestGapLimb = this;
	}
	OP_ASSERT(LimbType::disjoint == limbType);
}

OpTree::OpTree(OpJoiner& join) 
	: limbStorage(nullptr)
	, current(nullptr)
	, contour(*join.edge->segment->contour)
	, edge(join.edge)
	, bestGapLimb(nullptr)
	, bestLimb(nullptr)
	, firstPt(join.edge->whichPtT().pt)
	, bestDistance(OpInfinity)
	, bestPerimeter(OpInfinity)
	, baseIndex(0) 
	, totalUsed(0) {
	for (OpEdge* test : join.linkups.l) {
		test->visited = false;
		test->lastEdge->visited = false;
	}
	limbStorage = contour.contours->resetLimbs();
	OpLimb* trunk = limbStorage->allocate(*this);
	OP_ASSERT(join.linkups.l.back() == join.edge);
	trunk->set(*this, join.edge, nullptr, EdgeMatch::start, LimbType::linked, 
			join.linkups.l.size() - 1, join.edge);
	OP_DEBUG_CODE(debugLimbs.push_back(trunk));
	join.edge->visited = true;
	join.edge->lastEdge->visited = true;
	LimbType limbType = LimbType::linked;
	do {
		switch (limbType) {
			case LimbType::linked:
				break;
			case LimbType::unlinked: 
				for (const std::vector<OpEdge*>& edges : { join.unsectByArea, join.unsortables } )
					for (OpEdge* test : edges)
						join.unlink(test);
				break;
			case LimbType::disabled:
				if (join.disabledBuilt)
					for (OpEdge* test : join.disabled)
						join.unlink(test);
				break;
			case LimbType::disabledPals:
				if (join.disabledPalsBuilt)
					for (OpEdge* test : join.disabledPals)
						join.unlink(test);
				break;
			case LimbType::miswound:
				for (OpEdge* test : join.linkups.l) {
					if (test == join.edge)
						continue;
					test->visited = false;
					test->lastEdge->visited = false;
				}
				break;
			case LimbType::disjoint:
				break;
			default:
				OP_ASSERT(0);
		}
		walker = 0;
		do {
			limbStorage->limb(*this, walker).foreach(join, *this, limbType);
		} while (++walker < totalUsed);
		limbType = (LimbType) ((int) limbType + 1);
		if (LimbType::disjoint < limbType)
			return;  // error if bestLimb == nullptr
	} while (!bestLimb);
}

// used to walk tree in breadth order
OpLimb& OpTree::limb(int index) {
	return contour.contours->limbStorage->limb(*this, index);
}

// join best limb to edge start, then parent to best limb, until lastEdge is found
bool OpTree::join(OpJoiner& join) {
	std::vector<uint32_t> linkupsErasures;
	const OpLimb* bestL = bestLimb;
	OpEdge* best = bestL->edge;
	if (EdgeMatch::end == bestL->match) {	
		(void) best->setLastLink(!best->which()); // make suitable for linking to a chain
		best = best->advanceToEnd(EdgeMatch::start);
	} else if (best != best->lastEdge) {
		(void) best->setLastLink(EdgeMatch::start);
	}
	if (LimbType::linked == bestL->type || LimbType::miswound == bestL->type)
		linkupsErasures.push_back(bestL->linkedIndex);
	do {
		const OpLimb* lastLimb = bestL->parent;
		OpEdge* prior = lastLimb->edge;
		OP_ASSERT(!best->containsLink(prior));
		if (EdgeMatch::end == lastLimb->match) {
			(void) prior->setLastLink(!prior->which());  // make suitable for linking to a chain
			prior = prior->advanceToEnd(EdgeMatch::start);
		} else
			(void) prior->setLastLink(prior->which());
		OpEdge* last = prior->lastEdge;
		OP_ASSERT(best->whichPtT().pt == last->whichPtT(EdgeMatch::end).pt);
		best->setPriorEdge(last);
		last->setNextEdge(best);
		prior->lastEdge = best->lastEdge;
		best->clearLastEdge();
		OP_ASSERT(!last->debugIsLoop());
		if (LimbType::linked == lastLimb->type || LimbType::miswound == lastLimb->type)
			linkupsErasures.push_back(lastLimb->linkedIndex);
		bestL = lastLimb;
		best = bestL->edge;
	} while (bestL->parent);
	OP_TRACK(linkupsErasures);
	std::sort(linkupsErasures.begin(), linkupsErasures.end(), std::greater<int>());
	for (size_t entry : linkupsErasures)
		join.linkups.l.erase(join.linkups.l.begin() + entry);
	join.edge->output(join.path, false);
#if OP_DEBUG
    for (OpLimb* limb : debugLimbs) {
		limb->debugBranches.clear();
	}
	debugLimbs.clear();
#endif
	return true;
}

// caller (in contours) has allocated storage already
OpLimb* OpLimbStorage::allocate(OpTree& tree) {
	++tree.totalUsed;
    return &storage[used++];
}

OpLimb& OpLimbStorage::limb(OpTree& tree, int index) {
	int blockBase = index & ~(ARRAY_COUNT(storage) - 1);
	OpLimbStorage* block;
	if (tree.current && blockBase == tree.baseIndex)
		block = tree.current;
	else {
		block = this;
		int lastBase = tree.totalUsed & ~(ARRAY_COUNT(storage) - 1);
		while (index < lastBase) {
			block = block->nextBlock;
			lastBase -= (int) ARRAY_COUNT(storage);
		}
		tree.current = block;
		tree.baseIndex = blockBase;
	}
	index &= ARRAY_COUNT(storage) - 1;
	return block->storage[index];
}

void OpLimbStorage::reset() {
	used = 0;
	while (nextBlock) {
        OpLimbStorage* save = nextBlock->nextBlock;
        delete nextBlock;
        nextBlock = save;
	}
	nextBlock = nullptr;
}

OpJoiner::OpJoiner(OpContours& contours, OpOutPath& p)
	: path(p)
	, linkMatch(EdgeMatch::none)
	, linkPass(LinkPass::none)
	, edge(nullptr)
	, lastLink(nullptr)
	, disabledBuilt(false)
	, disabledPalsBuilt(false) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			for (auto& e : segment.edges) {
				addEdge(&e);
			}
		}
	}
	sort();
#if OP_DEBUG
	contours.debugJoiner = this;
#endif
    OP_DEBUG_VALIDATE_CODE(debugValidate());
}

bool OpJoiner::setup() {
    if (!byArea.size() && !unsectByArea.size())
        return true;
    sort();  // join up largest edges first
    for (auto e : byArea) {
        e->setActive(true);
		e->clearLinkBounds();  // !!! this may be unnecessary (asserts if necessary)
    }
    for (auto unsectable : unsectByArea) {
        unsectable->setActive(true);
    }
    // although unsortables are marked active, care must be taken since they may or may not
    // be part of the output
    for (auto unsortable : unsortables) {
        unsortable->setActive(true);
    }
#if 0 && OP_DEBUG_IMAGE
    ::clear();
    ::hideSegmentEdges();
    ::hideIntersections();
    debugDraw();
    ::add(unsortables);
    ::showPoints();
    ::showValues();
    ::showTangents();
    ::redraw();
#endif
	return false;
}

void OpJoiner::addEdge(OpEdge* e) {
	OP_ASSERT(!e->debugIsLoop());
	if (e->disabled)
		return;
	e->setWhich(EdgeMatch::start);
#if OP_DEBUG_IMAGE
	e->debugJoin = true;
#endif
	if (e->unsortable)
		unsortables.push_back(e);
	else if (e->pals.size())
		unsectByArea.push_back(e);
	else
		byArea.push_back(e);
}

void OpJoiner::addToLinkups(OpEdge* e) {
	OP_ASSERT(!e->debugIsLoop());
    OpEdge* first = e->advanceToEnd(EdgeMatch::start);
	OpEdge* next = first;
	OpEdge* last;
	do {
		if (LinkPass::remaining != linkPass) {
			OP_ASSERT(next->isActive());
			next->clearActiveAndPals(ZeroReason::none);
		}
		next->clearLastEdge();
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
			for (auto& e : segment.edges) {
				if (!e.disabled || e.unsortable || e.pals.size())
					continue;
				// for the very small, include disabled edges
				if (e.centerless || e.windPal || e.start.pt.soClose(e.end.pt))
					disabled.push_back(&e);
			}
		}
	}
	disabledBuilt = true;
}

void OpJoiner::buildDisabledPals(OpContours& contours) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			for (auto& e : segment.edges) {
				if (e.disabled && !e.unsortable && e.pals.size() && !e.inOutput)
					disabledPals.push_back(&e);
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
bool OpJoiner::detachIfLoop(OpEdge* e, EdgeMatch loopMatch) {
	std::vector<LoopCheck> edges;
	OpEdge* test = e;
	// walk forwards to end, keeping one point per edge
	OP_ASSERT(e && !e->debugIsLoop());
	while (test) {
		if (edges.end() != std::find_if(edges.begin(), edges.end(), 
				[&test](const LoopCheck& check) {
			return check.edge == test; } )) {
			break;
		}
		edges.emplace_back(test, loopMatch);
		test = EdgeMatch::start == loopMatch ? test->nextEdge : test->priorEdge;
		if (e == test)
			break;
	}
	if (e == test) {	// if this forms a loop, there's nothing to detach, return success
		e->output(path, true);
		return true;
	}
	// walk backwards to start
	std::sort(edges.begin(), edges.end());
	auto detachEdge = [this](OpEdge* e, EdgeMatch match) {
		if (OpEdge* detach = EdgeMatch::start == match ? e->priorEdge : e->nextEdge) {
			EdgeMatch::start == match ? detach->clearNextEdge() : detach->clearPriorEdge();
			if (!detach->unsortable || detach->priorEdge || detach->nextEdge)
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
		test->output(path, true);
		return true;
	};
	auto detachPrior = [this, detachEdge](OpEdge* test, OpEdge* oppEdge) {
		detachEdge(test, EdgeMatch::start);
		detachEdge(oppEdge, EdgeMatch::end);
		test->setPriorEdge(oppEdge);
		oppEdge->setNextEdge(test);
		test->output(path, true);
		return true;
	};
	test = e;
	while ((test = (EdgeMatch::start == loopMatch ? test->priorEdge : test->nextEdge)) && e != test) {
		LoopCheck testCheck(test, !loopMatch);
		if (auto bound = std::lower_bound(edges.begin(), edges.end(), testCheck); 
				bound != edges.end() && bound->pt == testCheck.pt)
			return EdgeMatch::start == loopMatch ? detachNext(bound->edge, test) : 
					detachPrior(bound->edge, test);
#if 0
		// wait until loop fails to form because wrong pal was chosen before coding this
		for (auto pal : test->pals) {
			LoopCheck palCheck(pal, !loopMatch);
			if (auto bound = std::lower_bound(edges.begin(), edges.end(), palCheck);
					bound != edges.end() && !(palCheck < edges.front())) {
				OP_ASSERT(0);
				// If opp edge does not form a loop but pal does,
				// probably need to replace opp with pal.
				// Wait for this to happen for realsies before writing code to handle it.
				return EdgeMatch::start == loopMatch ? detachNext(bound->edge, test) :
						detachPrior(bound->edge, test);
			}
		}
#endif
	}
	return false;
}

// start here;
// one thing broken with this overall approach is that very small loops get priority over joining
// large incomplete segments. See if it can be restructured to finish big things first, then use 
// the scale of the big things to see if the small remaining things can be ignored
// first, figure out why the current test fails

bool OpJoiner::linkRemaining(OP_DEBUG_CODE(const OpContours* debugContours)) {
	OP_DEBUG_CONTEXT();
    OP_DEBUG_CODE(debugMatchRay(debugContours));
	linkPass = LinkPass::remaining;
	// match links may add or remove from link ups. Iterate as long as link ups is not empty
	for (auto e : linkups.l) {
		e->setLinkBounds();
	}
	OP_DEBUG_IMAGE_CODE(int debugLoopCounter = 0);
    while (linkups.l.size()) {
		// sort to process largest first
		// !!! could optimize to avoid search, but for now, this is the simplest
		linkups.sort();
		for (;;) {
			edge = linkups.l.back();
			if (!edge->disabled)  // may be pal whose partner has already been added (loops44i)
				break;
			linkups.l.pop_back();
			if (!linkups.l.size())
				return true;
		}
		OP_DEBUG_VALIDATE_CODE(debugValidate());
        if (!matchLinks(true))
			return false;
#if 0 && OP_DEBUG_IMAGE
		colorOut(orange);
#endif
		// contour generated by match links may allow for link up edges to now have a single link
		size_t linkupsIndex = 0;
		while (relinkUnambiguous(linkupsIndex)) {
			++linkupsIndex;
		}
		OP_DEBUG_VALIDATE_CODE(debugValidate());
		OP_DEBUG_IMAGE_CODE(if (++debugLoopCounter < 0) OpDebugOut(""));  // allows seeing loop iteration that failed
    }
	return true;
}

void OpJoiner::linkUnambiguous(LinkPass lp) {
	OP_DEBUG_CONTEXT();
    OP_DEBUG_VALIDATE_CODE(debugValidate());
    // match up edges that have only a single possible prior or next link, and add them to new list
    linkPass = lp;
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	std::vector<OpEdge*>& edges = LinkPass::normal == lp ? byArea : unsectByArea;
    for (auto& e : edges) {
		if (e->disabled)
            continue;   // likely marked as part of a loop below
        if (!e->isActive())  // check if already saved in linkups
            continue;
		OP_ASSERT(!e->priorEdge);
		OP_ASSERT(!e->nextEdge);
		if (LinkPass::unsectable == lp)
			e->setWhich(EdgeMatch::start);
		linkMatch = EdgeMatch::start;
		if (!linkUp(e))
			continue;
		OP_DEBUG_VALIDATE_CODE(debugValidate());
		linkMatch = EdgeMatch::end;
		(void) linkUp(e->setLastEdge());
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
bool OpJoiner::linkUp(OpEdge* e) {
	std::vector<FoundEdge> edges;
	OP_ASSERT(!e->debugIsLoop(EdgeMatch::end, LeadingLoop::will));
	const OpSegment* segment = e->segment;
	bool hadLinkTo = false;
	bool hasPal = segment->activeAtT(e, linkMatch, edges, &hadLinkTo);
	hasPal |= segment->activeNeighbor(e, linkMatch, edges);
    // if oppEdges is count of one and unsortable, don't return any edges (testQuadratic67x)
    if (edges.size() == 1 && edges[0].edge->unsortable /* && hadLinkTo */)
        edges.clear(); // hadLinkTo breaks thread_cubics147521
	// skip pals should choose the pal that minimizes the output path area
	// if there's not enough info here to do that, the pal choice should be reconsidered
	//   when match links is called
	// !!! maybe the right choice here is the wrong choice later?!
    if (edges.size() && hasPal)  // if edges[x] has pals and pal is in linkups, remove edges[x]
		e->skipPals(linkMatch, edges);
	// if edge has pals, and there's a matching unsortable, don't return edge (thread_cubics502920)
	if (1 != edges.size() || !edges[0].edge->isActive()) {
		if (EdgeMatch::start == linkMatch)
			return true;  // 1) found multiple possibilities, try end
		addToLinkups(e);
		return false;  // 2) found multiple possibilities (end)
	}
	FoundEdge foundOne = edges.front();
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	e->linkToEdge(foundOne, linkMatch);
	OP_ASSERT(e->whichPtT(linkMatch).pt == foundOne.edge->flipPtT(linkMatch).pt);
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	if (detachIfLoop(e, linkMatch))
		return false; // 4) found loop, nothing leftover; caller to move on to next edge
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	// move to the front or back edge depending on link match
	OpEdge* recurse = foundOne.edge->advanceToEnd(linkMatch);
	return linkUp(recurse);	// 5)  recurse to extend prior or next
}

// at this point all singly linked edges have been found
// every active set of links at this point must form a loop
// the only distance that matters is zero. We should never have unexplained gaps (ideal, not real)
bool OpJoiner::matchLinks(bool popLast) {
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	OP_ASSERT(!edge->priorEdge);
	lastLink = edge->lastEdge;
	OP_ASSERT(lastLink);
	OP_ASSERT(!lastLink->nextEdge);
	OP_ASSERT(EdgeMatch::start == lastLink->which() || EdgeMatch::end == lastLink->which());
	found.clear();
	matchPt = lastLink->whichPtT(EdgeMatch::end).pt;
	OpTree tree(*this);
#if 0 && OP_DEBUG_VERBOSE
	std::string s = "perimeter:" + STR(tree.bestPerimeter);
	s += " edges:";
	const OpLimb* limb = tree.bestLimb;
	do {
		s += STR(limb->edge->id);
		if (limb->edge->lastEdge && limb->edge != limb->edge->lastEdge)
			s += ".." + STR(limb->edge->lastEdge->id);
		s += " ";
	} while ((limb = limb->parent));
	OpDebugOut(s + "\n");
#endif
	if (!tree.bestLimb) {
		OP_ASSERT(tree.bestGapLimb);
		OpContour* contour = lastLink->segment->contour;
		OpIntersection* startI = edge->findSect(EdgeMatch::start);
		OpIntersection* gapEnd = tree.bestGapLimb->lastLimb->findSect(!tree.bestGapLimb->match);
		OpEdge* filler = contour->addFiller(gapEnd, startI);
		if (filler) {
			OpLimb* branch = contour->contours->allocateLimb(tree);
			branch->set(tree, filler, tree.bestGapLimb, EdgeMatch::start, LimbType::disjoint, 0, 
					nullptr, nullptr);
		#if OP_DEBUG
			const_cast<OpLimb*>(tree.bestGapLimb)->debugBranches.push_back(branch);
			tree.debugLimbs.push_back(branch);
		#endif
			tree.bestLimb = branch;
		}
	}
	OP_ASSERT(tree.bestLimb);
	return tree.join(*this);
}

// check if resolution of link ups left unambiguous edge ends for further linkage
// !!! this is missing a check to see if the matched edge has the correct winding
// at very least, it should have an assert
bool OpJoiner::relinkUnambiguous(size_t link) {
	if (link >= linkups.l.size())
		return false;
	std::vector<size_t> linkupsErasures;
	bool startIsBestMatch = true;
	size_t tIndex = 0;
	edge = linkups.l[link];
	if (link + 1 < linkups.l.size()) { // must have at least two link ups to hook together
		EdgeMatch tMatch;
		auto scanForMatch = [&tMatch, &tIndex, link, this](OpEdge* eEdge, EdgeMatch eMatch) {
			OpPoint edgePt = eEdge->whichPtT(eMatch).pt;
			for (const std::vector<OpEdge*>& edges : { unsectByArea, unsortables } ) {
				auto testUnmatch = [edgePt](OpEdge* test, EdgeMatch match) {
					return test->whichPtT(match).pt == edgePt;
				};
				for (OpEdge* test : edges) {
					if (testUnmatch(test, EdgeMatch::start))
						return false;
					if (testUnmatch(test, EdgeMatch::end))
						return false;
				}
			}
			tMatch = EdgeMatch::none;
			auto test = [&tMatch, &tIndex, edgePt, this](size_t start, size_t end) {
				size_t index = start;
				while (index < end) { // if there is something to hook up to
					auto testMatch = [&tMatch, &tIndex, index, edgePt](OpEdge* test, EdgeMatch match) {
						if (test->whichPtT(match).pt == edgePt) {
							if (tMatch != EdgeMatch::none)
								return false;  // there is more than one match; give up on this end
							tMatch = match;
							tIndex = index;
						}
						return true;
					};
					OpEdge* test = linkups.l[index];
					if (!testMatch(test, EdgeMatch::start))
						return false;
					if (!testMatch(test->lastEdge, EdgeMatch::end))
						return false;
					++index;
				}
				return true;
			};
			if (!test(link + 1, linkups.l.size()))
				return false;
			if (!test(0, link))
				return false;
			return EdgeMatch::none != tMatch;
		};
		// single edge end found which matches; link the two
		auto mergeLinks = [&, this](OpEdge* e, EdgeMatch eMatch, size_t linkIndex) {
			OpEdge* tEdge = linkups.l[tIndex];  // check last edge to see if found is first or last
			OP_ASSERT(!tEdge->priorEdge);
			if (EdgeMatch::start == eMatch) {
				if (EdgeMatch::start == tMatch) {
					if (!tEdge->nextEdge)
						tEdge->setWhich(!tEdge->which());
					else {
						tEdge = tEdge->lastEdge;
						tEdge->setLinkDirection(tMatch);  // reverse links
						linkups.l[tIndex] = tEdge;
					}
				}
				e->setPriorEdge(tEdge->lastEdge);
				tEdge->lastEdge->setNextEdge(e);
				tEdge->lastEdge = e->lastEdge;
				tEdge->setLinkBounds();
				e->clearLastEdge();
			} else {
				if (EdgeMatch::end == tMatch) {
					tEdge = tEdge->lastEdge;
					tEdge->setLinkDirection(tMatch);  // reverse links
				}
				e = e->advanceToEnd(EdgeMatch::start);
				tEdge->setPriorEdge(e->lastEdge);
				e->lastEdge->setNextEdge(tEdge);
				e->lastEdge = tEdge->lastEdge;
				e->setLinkBounds();
				tEdge->clearLastEdge();
			}
			linkupsErasures.push_back(linkIndex);
		};
		if (scanForMatch(edge, EdgeMatch::start))
			mergeLinks(edge, EdgeMatch::start, link);
		else {
			lastLink = edge->lastEdge;
			if (!scanForMatch(lastLink, EdgeMatch::end))
				return true;
			mergeLinks(lastLink, EdgeMatch::end, tIndex);
			startIsBestMatch = false;
		}
	}
	if (detachIfLoop(edge->advanceToEnd(EdgeMatch::start), EdgeMatch::end))
		linkupsErasures.push_back(startIsBestMatch ? tIndex : link);
	if (!linkupsErasures.size())
		return false;
	std::sort(linkupsErasures.begin(), linkupsErasures.end(), std::greater<int>());
	for (size_t entry : linkupsErasures)
		linkups.l.erase(linkups.l.begin() + entry);
	return relinkUnambiguous(link);  // after linking, try again
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

void OpJoiner::unlink(OpEdge* test) {
	if (test->inOutput) {
		test->unlink();
		return;
	}
	OpEdge* first = test->advanceToEnd(EdgeMatch::start);
	if (linkups.l.end() == std::find(linkups.l.begin(), linkups.l.end(), first))
		test->unlink();
}

// sort by size to process largest (tail) first
// sort should consider all edges in link
//start here;
// sort so that first, in addition to being largest, is also on the overall outside border
void LinkUps::sort() {
	OpPointBounds bounds;
	for (auto& linkList : l) {
		bounds.add(linkList->linkBounds);
	}
	auto onBounds = [bounds](const OpEdge* s) {
		return s->linkBounds.left == bounds.left || s->linkBounds.top == bounds.top ||
				s->linkBounds.right == bounds.right || s->linkBounds.bottom == bounds.bottom;
	};
	std::sort(l.begin(), l.end(), [onBounds](const auto& s1, const auto& s2) {
		bool s1OnBounds = onBounds(s1);
		bool s2OnBounds = onBounds(s2);
		return s1OnBounds < s2OnBounds || (s1OnBounds == s2OnBounds 
				&& s1->linkBounds.perimeter() < s2->linkBounds.perimeter()); 
	} );
}
