// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegment.h"
#include "PathOps.h"

void OpLimb::add(OpTree& tree, OpEdge* test, EdgeMatch m, LimbPass limbPass, size_t limbIndex,
		OpEdge* otherEnd) {
	OP_ASSERT(!test->disabled || test->isUnsectable() || test->pals.size() 
			|| LimbPass::disabled <= limbPass);
	OP_ASSERT(!test->hasLinkTo(m) || test->isUnsortable
			|| test->isUnsectable() || test->pals.size());
	if (test->whichPtT(m).pt != lastPtT.pt && LimbPass::unsectPair != limbPass)
		return;
	if (edge == test)
		return;
	if (EdgeMatch::start == m ? test->startSeen : test->endSeen)
		return;
	(EdgeMatch::start == m ? test->startSeen : test->endSeen) = true;
	if (test->isUnsectable() && test->unsectableSeen(m))
		return;
	// compare test wind zero against their parent's last edge wind zero
	OP_ASSERT(lastLimbEdge);
	// OP_ASSERT(!test->isPal(last) || LimbPass::linked != limbPass);  // breaks pentrek10
	// Edge direction and winding are tricky (see description at wind zero declaration.)
	// For first edge (and its last) in storage: if which end is 'end', its wind zero is reversed.
	// 'Test' may need to be reversed to connect, and 'm' may be either end. Wind zero in both cases
	// is computed for the unreversed orientation.
	if (!lastLimbEdge->isUnsortable && LimbPass::linked == limbPass && !test->isUnsortable) {
        WindZero zeroSide = test->windZero;
		// if last which end is end, flip last's wind zero (for comparsion, flip zero side);
		// if test m is end, flip zero side; if test which is end, flip zero side
        if ((lastLimbEdge->which() == EdgeMatch::start) != (test->which() == m))
            zeroSide = !zeroSide;
        if (lastLimbEdge->windZero != zeroSide)
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
	OpContours& contours = *tree.contour->contours;
	OpLimb* branch = contours.allocateLimb(&tree);
	if (LimbPass::unsectPair == limbPass) {
		OpPtT startI = test->whichPtT(m);
		OpPtT gapEnd = lastLimbEdge->whichPtT(lastMatch);
		OpEdge* filler = tree.contour->addFiller(gapEnd, startI);
		filler->setWhich(EdgeMatch::start);
		branch->set(tree, filler, this, EdgeMatch::start, limbPass, limbIndex, nullptr, 
				&filler->ptBounds);
		branch->gapDistance = (filler->startPt() - lastPtT.pt).length();
		return;
	}
	branch->set(tree, test, this, m, limbPass, limbIndex, otherEnd, &childBounds);
}

void OpLimb::set(OpTree& tree, OpEdge* test, OpLimb* p, EdgeMatch m, LimbPass l, 
		size_t index, OpEdge* otherEnd, const OpPointBounds* childBounds) {
	edge = test;
	parent = p;
	linkedIndex = (uint32_t) index;
	gapDistance = 0;
	match = m;
	pass = l;
	if (childBounds)
		bounds = *childBounds;
	if (LimbPass::linked != pass && LimbPass::miswound != pass) {
		lastLimbEdge = edge;
		lastPtT = lastLimbEdge->whichPtT(!match);
		lastMatch = !match;
	} else if (EdgeMatch::start == match) {
		lastLimbEdge = edge->lastEdge;
		lastPtT = lastLimbEdge->whichPtT(EdgeMatch::end);
		lastMatch = EdgeMatch::end;
	} else {
		lastLimbEdge = otherEnd;
		lastPtT = lastLimbEdge->whichPtT(EdgeMatch::start);
		lastMatch = EdgeMatch::start;
	}
	looped = childBounds ? tree.firstPt == lastPtT.pt : false;
	if (looped && tree.bestPerimeter > bounds.perimeter()) {
		tree.bestPerimeter = bounds.perimeter();
		tree.bestLimb = this;
	}
	closeDistance = (lastPtT.pt - tree.firstPt).length();
	if (tree.bestDistance > closeDistance) {
		tree.bestDistance = closeDistance;
		tree.bestGapLimb = this;
	}
	OP_DEBUG_DUMP_CODE(debugID = tree.contour->nextID());
	OP_DEBUG_DUMP_CODE(if (p) p->debugBranches.push_back(this));
}

void OpLimb::foreach(OpJoiner& join, OpTree& tree, LimbPass limbPass) {
	if (looped)  // triggered when walking children of trunk 
		return;
	size_t linkupsSize = join.linkups.l.size();
	LimbPass linkedLimb = LimbPass::miswound <= limbPass ? LimbPass::miswound : LimbPass::linked;
	for (unsigned index = 0; index < linkupsSize; ++index) {
		OpEdge* test = join.linkups.l[index];
		if (test->disabled)
			continue;
		add(tree, test, EdgeMatch::start, linkedLimb, index);
		add(tree, test->lastEdge, EdgeMatch::end, linkedLimb, index, test);
	}
	if (LimbPass::linked == limbPass)
		return;
	for (const std::vector<OpEdge*>& edges : { join.unsectByArea, join.unsortables } ) {
		for (OpEdge* test : edges) {
			if (test->inLinkups)
				continue;
			add(tree, test, EdgeMatch::start, LimbPass::unlinked);
			add(tree, test, EdgeMatch::end, LimbPass::unlinked);
		}
	}
	if (LimbPass::unlinked == limbPass)
		return;
	if (!join.disabledBuilt) {
		join.buildDisabled(*tree.contour->contours);
		for (OpEdge* test : join.disabled) {
			test->unlink();
		}
	}
	for (OpEdge* test : join.disabled) {
		add(tree, test, EdgeMatch::start, LimbPass::disabled);
		add(tree, test, EdgeMatch::end, LimbPass::disabled);
	}
	if (LimbPass::disabled == limbPass)
		return;
	for (OpEdge* test : join.disabledPals) {
		add(tree, test, EdgeMatch::start, LimbPass::disabledPals);
		add(tree, test, EdgeMatch::end, LimbPass::disabledPals);
	}
	if (LimbPass::disabledPals == limbPass)
		return;
	if (LimbPass::miswound == limbPass)
		return; 
	// iterate through edge pals looking for gap that connects lastPt via sect opp
	// unsectable edges do not necessarily point to other unsectable through pals or upairs
	if (lastLimbEdge->isUnsectable()) {
		for (EdgePal& edgePal : lastLimbEdge->uPals) {
			add(tree, edgePal.edge, edgePal.reversed ? !match : match, LimbPass::unsectPair);
		}
	#if 0
		OP_ASSERT(lastLimbEdge->startSect >= 0);
		OP_ASSERT(lastLimbEdge->endSect > 0);
		OpIntersection** first = &lastLimbEdge->segment->sects.i.front() + lastLimbEdge->startSect;
		float tMatch = lastLimbEdge->startT;
		OP_ASSERT((*first)->ptT.t == tMatch);
		OP_DEBUG_CODE(const OpCurve& dbgC = lastLimbEdge->curve);
		OpIntersection** last = &lastLimbEdge->segment->sects.i.back();
		OP_ASSERT(first <= last);
		OP_DEBUG_CODE(bool foundUnsectableEdge = false);
		do {
			OpIntersection* uSect = *first++;
			if (!uSect->unsectID)
				continue;
			OP_ASSERT(uSect->ptT.pt == dbgC.firstPt());
			OpIntersection* oppSect = uSect->opp;
			OpSegment* oppSeg = oppSect->segment;
			OpIntersection** oppFirst = &oppSeg->sects.i.front();
			for (OpEdge& oppEdge : oppSeg->edges) {
				if (!oppEdge.isUnsectable())
					continue;
				OpIntersection** oSectFirst = oppFirst + oppEdge.startSect;
				OpIntersection** oLast = &oppSeg->sects.i.back();
				OP_ASSERT(oSectFirst <= oLast);
				float oTMatch = (*oSectFirst)->ptT.t;
				do {
					OpIntersection* oppUSect = *oSectFirst++;
					if (fabs(uSect->unsectID) == fabs(oppUSect->unsectID)) {
						OP_DEBUG_CODE(foundUnsectableEdge = true);
						add(tree, &oppEdge, oppUSect->unsectID < 0 ? match : !match, LimbPass::unsectPair);
						break;
					}
				} while (oSectFirst <= oLast && (*oSectFirst)->ptT.t == oTMatch);
			}
		} while (first <= last && (*first)->ptT.t == tMatch);
		OP_ASSERT(foundUnsectableEdge);
#endif
	}
#if 0
	for (UnsectableOpp& usectPal : lastLimbEdge->uPairs) {
		OpEdge* palEdge = usectPal.edge;
		EdgeMatch palMatch = usectPal.unsectableID < 0 ? match : !match;
		if (lastPtT.pt == palEdge->whichPtT(palMatch).pt)
			continue;
		add(tree, palEdge, palMatch, LimbPass::unsectPair);
	}
#endif
	if (LimbPass::unsectPair == limbPass)
		return;
	OP_ASSERT(LimbPass::disjoint == limbPass);
}

OpTree::OpTree(OpJoiner& join) 
	: limbStorage(nullptr)
	, current(nullptr)
	, contour(join.edge->segment->contour)
	, bestGapLimb(nullptr)
	, bestLimb(nullptr)
	, firstPt(join.edge->whichPtT().pt)
	, bestDistance(OpInfinity)
	, bestPerimeter(OpInfinity)
	, baseIndex(0) 
	, totalUsed(0) {
	for (OpEdge* test : join.linkups.l) {
		test->startSeen = false;
		test->lastEdge->endSeen = false;
	}
	limbStorage = contour->contours->resetLimbs(OP_DEBUG_DUMP_CODE(this));
	OpLimb* trunk = limbStorage->allocate(*this);
	OP_ASSERT(join.linkups.l.back() == join.edge);
	trunk->set(*this, join.edge, nullptr, EdgeMatch::start, LimbPass::linked, 
			join.linkups.l.size() - 1, join.edge);
	join.edge->startSeen = true;
	join.edge->lastEdge->endSeen = true;
	LimbPass limbPass = LimbPass::linked;
	do {
		initialize(join, limbPass);
		if (LimbPass::disabledPals == limbPass)
			addDisabled(join);
		else {
			walker = 0;
			do {
				limbStorage->limb(*this, walker).foreach(join, *this, limbPass);
			} while (++walker < totalUsed);
		}
		limbPass = (LimbPass) ((int) limbPass + 1);
		if (LimbPass::disjoint < limbPass)
			return;  // error if bestLimb == nullptr
	} while (!bestLimb);
}

// walk the disabled pals from smallest to largest instead of the limbs
// add the least disturbing disabled pal to any limb that matches (that also disturbs least)
// !!! may need to treat regular disabled the same, although pals are more legit ?
void OpTree::addDisabled(OpJoiner& join) {
	if (!join.disabledPalsBuilt) 
		join.buildDisabledPals(*contour->contours);
	for (OpEdge* test : join.disabledPals) {
		test->unlink();
		// check every limb for point match; choose based on limbPass, then bounds
		walker = 0;
		do {
			OpLimb& limb = limbStorage->limb(*this, walker);
			limb.add(*this, test, EdgeMatch::start, LimbPass::disabledPals);
			limb.add(*this, test, EdgeMatch::end, LimbPass::disabledPals);
		} while (++walker < totalUsed);
	}


}

void OpTree::initialize(OpJoiner& join, LimbPass limbPass) {
	switch (limbPass) {
		case LimbPass::linked:
			break;
		case LimbPass::unlinked: 
			for (const std::vector<OpEdge*>& edges : { join.unsectByArea, join.unsortables } )
				for (OpEdge* test : edges)
					join.unlink(test);
			break;
		case LimbPass::unsectPair:
			break;
		case LimbPass::disabled:
			if (join.disabledBuilt)
				for (OpEdge* test : join.disabled)
					join.unlink(test);
			break;
		case LimbPass::disabledPals:
			if (join.disabledPalsBuilt)
				for (OpEdge* test : join.disabledPals)
					join.unlink(test);
			break;
		case LimbPass::miswound:
			for (OpEdge* test : join.linkups.l) {
				if (test == join.edge)
					continue;
				test->startSeen = false;
				test->lastEdge->endSeen = false;
			}
			break;
		case LimbPass::disjoint:
			break;
		default:
			OP_ASSERT(0);
	}
}

// used to walk tree in breadth order
OpLimb& OpTree::limb(int index) {
	return contour->contours->limbStorage->limb(*this, index);
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
	if (LimbPass::linked == bestL->pass || LimbPass::miswound == bestL->pass)
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
		if (LimbPass::linked == lastLimb->pass || LimbPass::miswound == lastLimb->pass)
			linkupsErasures.push_back(lastLimb->linkedIndex);
		bestL = lastLimb;
		best = bestL->edge;
	} while (bestL->parent);
	OP_TRACK(linkupsErasures);
	std::sort(linkupsErasures.begin(), linkupsErasures.end(), std::greater<int>());
	for (size_t entry : linkupsErasures)
		join.linkups.l.erase(join.linkups.l.begin() + entry);
	join.edge->output(false);
	contour->contours->resetLimbs(OP_DEBUG_DUMP_CODE(this));
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
#if OP_DEBUG_DUMP
	auto clearDebugBranches = [](OpLimbStorage* limbs) {
		for (int index = 0; index < limbs->used; ++index) {
			OpLimb& limb = limbs->storage[index];
			limb.debugBranches.clear();
		}
	};
#endif
	while (nextBlock) {
		OP_DEBUG_DUMP_CODE(clearDebugBranches(nextBlock));
        OpLimbStorage* save = nextBlock->nextBlock;
        delete nextBlock;
        nextBlock = save;
	}
	OP_DEBUG_DUMP_CODE(clearDebugBranches(this));
	nextBlock = nullptr;
	used = 0;
}

OpJoiner::OpJoiner(OpContours& contours)
	: linkMatch(EdgeMatch::none)
	, linkPass(LinkPass::none)
	, edge(nullptr)
	, lastLink(nullptr)
	, disabledBuilt(false)
	, disabledPalsBuilt(false) {
	for (auto contour : contours.contours) {
		for (auto& segment : contour->segments) {
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
	if (e->isUnsortable)
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
			next->setActive(false);
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
	for (auto contour : contours.contours) {
		for (auto& segment : contour->segments) {
			for (auto& e : segment.edges) {
				if (!e.disabled || e.isUnsortable || e.pals.size())
					continue;
				// for the very small, include disabled edges
				if (e.centerless || e.windPal || e.startPt().soClose(e.endPt()))
					disabled.push_back(&e);
			}
		}
	}
	disabledBuilt = true;
}

void OpJoiner::buildDisabledPals(OpContours& contours) {
	for (auto contour : contours.contours) {
		for (auto& segment : contour->segments) {
			for (auto& e : segment.edges) {
				if (e.disabled && !e.inOutput && !e.isUnsortable) {
					// !!! test may be overbroad; may need to look at sect and include only
					//     coin + unsect (or add bit in edge to register coin)
					if (e.pals.size() || e.isUnsectable())
						disabledPals.push_back(&e);
				}
			}
		}
	}
	std::sort(disabledPals.begin(), disabledPals.end(), [](OpEdge* a, OpEdge* b)
			{ return a->ptBounds.perimeter() < b->ptBounds.perimeter(); }
	);
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
		e->output(true);
		return true;
	}
	// walk backwards to start
	std::sort(edges.begin(), edges.end());
	auto detachEdge = [this](OpEdge* e, EdgeMatch match) {
		if (OpEdge* detach = EdgeMatch::start == match ? e->priorEdge : e->nextEdge) {
			EdgeMatch::start == match ? detach->clearNextEdge() : detach->clearPriorEdge();
			if (!detach->isUnsortable || detach->priorEdge || detach->nextEdge)
				addToLinkups(detach);	// return front edge
			else
				; // OP_ASSERT(!detach->priorEdge);  // triggered by fuzz763_1 -- is fix needed?
		}
	};
	auto detachNext = [detachEdge](OpEdge* test, OpEdge* oppEdge) 
	{
		detachEdge(test, EdgeMatch::end);
		detachEdge(oppEdge, EdgeMatch::start);
		test->setNextEdge(oppEdge);
		oppEdge->setPriorEdge(test);
		test->output(true);
		return true;
	};
	auto detachPrior = [detachEdge](OpEdge* test, OpEdge* oppEdge) {
		detachEdge(test, EdgeMatch::start);
		detachEdge(oppEdge, EdgeMatch::end);
		test->setPriorEdge(oppEdge);
		oppEdge->setNextEdge(test);
		test->output(true);
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

bool OpJoiner::linkRemaining(OP_DEBUG_CODE(OpContours* debugContours)) {
	OP_DEBUG_CONTEXT();
#if OP_DEBUG_IMAGE
	debugImage();
	showFill();
#endif
    OP_DEBUG_CODE(debugMatchRay(debugContours));
	OP_ASSERT(!TEST_PATH_OP_SKIP_TO_V0 || OP_DEBUG_FAST_TEST);  // break if running last failed fast test
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
#if 1 && OP_DEBUG_IMAGE
		colorOut(orange);
#endif
		// Match links may have output and/or disabled edge in linkups which has prior or next
		// that has not been output. Check for this and adjust linkups as needed.
		size_t linkupsIndex = 0;
		for (; linkupsIndex < linkups.l.size(); ++linkupsIndex) {
			OpEdge* test = linkups.l[linkupsIndex];
			OpEdge* writeNew = nullptr;
			while (test->disabled && test->nextEdge)
				writeNew = test = test->nextEdge;
			if (writeNew) {
				test->priorEdge = nullptr;
				test->lastEdge = linkups.l[linkupsIndex]->lastEdge;
				linkups.l[linkupsIndex] = test;
			}
		}
		// contour generated by match links may allow for link up edges to now have a single link
		linkupsIndex = 0;
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
    if (edges.size() == 1 && edges[0].edge->isUnsortable /* && hadLinkTo */)
        edges.clear(); // hadLinkTo breaks thread_cubics147521
	// skip pals should choose the pal that minimizes the output path area
	// if there's not enough info here to do that, the pal choice should be reconsidered
	//   when match links is called
	// !!! maybe the right choice here is the wrong choice later?!
    if ((edges.size() && hasPal)  // if edges[x] has pals and pal is in linkups, remove edges[x]
	//	e->skipPals(linkMatch, edges);
	// if edge has pals, and there's a matching unsortable, don't return edge (thread_cubics502920)
		|| 1 != edges.size() || !edges[0].edge->isActive()) {
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
	// adding gap edge in unsect pair case
	if (!tree.bestLimb) {
		OpLimb* gap = tree.bestGapLimb;
		OP_ASSERT(gap);
		OpPtT startI = edge->whichPtT(EdgeMatch::start);
		OpPtT gapEnd = gap->lastLimbEdge->whichPtT(!gap->match);
		OpEdge* filler = tree.contour->addFiller(gapEnd, startI);
		OpLimb* branch = tree.contour->contours->allocateLimb(&tree);
		branch->set(tree, filler, gap, EdgeMatch::start, LimbPass::disjoint, 0, nullptr);
		tree.bestLimb = branch;
	}
	return tree.join(*this);
}

// check if resolution of link ups left unambiguous edge ends for further linkage
// !!! this is missing a check to see if the matched edge has the correct winding
// at very least, it should have an assert
bool OpJoiner::relinkUnambiguous(size_t link) {
	if (link >= linkups.l.size())
		return false;
	std::vector<size_t> linkupsErasures;
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
		}
	}
	size_t originalSize = linkups.l.size();
	bool eraseLoops = detachIfLoop(edge->advanceToEnd(EdgeMatch::start), EdgeMatch::end);
	if (linkupsErasures.size()) {
		std::sort(linkupsErasures.begin(), linkupsErasures.end(), std::greater<int>());
		for (size_t entry : linkupsErasures)
			linkups.l.erase(linkups.l.begin() + entry);
	}
	size_t index = linkups.l.size();
	if (eraseLoops && index) {
		do {
			--index;
			if (linkups.l[index]->inOutput)
				linkups.l.erase(linkups.l.begin() + index);
		} while (index);
	}
	if (linkups.l.size() == originalSize)
		return false;
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
