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
	OP_ASSERT(!test->isPal(last) || LimbType::linked != limbType);
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
		lastEdge = edge;
		lastPt = lastEdge->whichPtT(!match).pt;
	} else if (EdgeMatch::start == match) {
		lastEdge = edge->lastEdge;
		lastPt = lastEdge->whichPtT(EdgeMatch::end).pt;
	} else {
		lastEdge = otherEnd;
		lastPt = lastEdge->whichPtT(EdgeMatch::start).pt;
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
	} else
		(void) best->setLastLink(EdgeMatch::start);
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
			(void) prior->setLastLink(EdgeMatch::start);
		OpEdge* last = prior->lastEdge;
		OP_ASSERT(best->whichPtT().pt == last->whichPtT(EdgeMatch::end).pt);
		best->setPriorEdge(last);
		last->setNextEdge(best);
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
	, lastEdge(nullptr)
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
    }
    for (auto unsectable : unsectByArea) {
        unsectable->setActive(true);
    }
    // although unsortables are marked active, care must be taken since they may or may not
    // be part of the output
    for (auto unsortable : unsortables) {
        unsortable->setActive(true);
    }
#if 01 && OP_DEBUG_IMAGE
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

// look for active unsectable edges that match
// We could accumulate more than one path until we know which one is best.
// Instead, we'll add one but compare it and its pals to see if the path closes.
// If a pal closes, then rewind until the pal chain and given chain match and swap
//  with the pal chain to close the path.
// returns true if more than one unsectable was found
#if 0
bool OpJoiner::activeUnsectable(const OpEdge* , EdgeMatch match, 
        std::vector<FoundEdge>& oppEdges) {
	// !!! incomplete
	return false;
}
#endif

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
				if (e.centerless || e.windPal)  // only very small disabled are considered
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

// look for a disabled edge that closes the gap
// it's likely that this edge is very small, but don't know how to quantify that (yet)
void OpJoiner::checkDisabled() {
	if (!disabledBuilt)
		buildDisabled(*edge->contours());
	// need a lookahead method (e.g., testQuadratic75: (simplify))
	// sometimes the best choice is not the one that makes the area smallest
	// if we could look ahead as few as two edges, testQuadratic7 (simplify) for instance
	// could land on correct result
	matchLeftover(disabled);
}

// best gap has the distance from the match point to the next available edge
// if it is very small (?) just use it
// if it is smaller (much smaller?) that unsortable/disabled pal/missed/etc, use it instead?
	// !!! wait for test case to show up before writing code
void OpJoiner::checkGap() {
     if (!OpMath::IsFinite(bestGap.distSq))  // triggered by battleOp21
        return;
    OpVector nextDoor = {
			OpMath::NextLarger(fabsf(matchPt.x)) - fabsf(matchPt.x),
			OpMath::NextLarger(fabsf(matchPt.y)) - fabsf(matchPt.y) };
	// !!! if we add a factor of 4, it fixes battleOp287 without breaking anything else...
	if (nextDoor.lengthSquared() * 4 < bestGap.distSq)
        return;
	OpContour* contour = lastEdge->segment->contour;
	OpIntersection* lastI = lastEdge->findSect(EdgeMatch::end);
	OpIntersection* gapEnd = bestGap.edge->findSect(bestGap.whichEnd);
	OpEdge* filler = contour->addFiller(lastI, gapEnd);
	if (filler)
		found.emplace_back(filler, EdgeMatch::start);
}

// count intersections equaling end
// each intersection has zero, one, or two active edges
void OpJoiner::checkLinkups() {
	for (size_t index = 0; index < linkups.l.size(); ++index) {
		OpEdge* linkup = linkups.l[index];
		OP_ASSERT(!linkup->priorEdge);
		OP_ASSERT(!linkup->debugIsLoop());
//		start here;
		// while linkup or linkup->lastEdge are unsortable, try them, then try the next
		if (lastEdge != linkup) {
			if (linkup->whichPtT().pt == matchPt)
				found.emplace_back(linkup, linkup->which(), index);
			else 
				bestGap.check(&found, linkup, EdgeMatch::start, matchPt); 
		}
		OpEdge* lastLink = linkup->lastEdge;
		OP_ASSERT(lastLink);
		if (lastEdge != lastLink) {
			if (lastLink->whichPtT(EdgeMatch::end).pt == matchPt)
				found.emplace_back(lastLink, !lastLink->which(), index);
			else
				bestGap.check(&found, lastLink, EdgeMatch::end, matchPt);
		}
	}
}

void OpJoiner::checkNothingLeft() {
	// if there's no remaining active edges in linkups or unsectables, just close what's left (loops63i)
	auto unsortableCount = [this](const auto e) {
		int unusedUnsortables = 0;
		for (auto u : unsortables) {
			if (!u->inOutput)
				++unusedUnsortables;
		}
		return unusedUnsortables - e->countUnsortable();
	};
	if (!linkups.l.size() && !unsortableCount(edge)) {
		OpEdge* filler = edge->segment->contour->addFiller(edge, lastEdge);
		if (!filler)
			return;
		found.emplace_back(filler, EdgeMatch::start);
	}
}

bool OpJoiner::checkSectGap() {
	// its possible that the intersections for very very small edges were missed (skpadspert_net23)
	// before going down the disabled rabbit hole, see if the is a small gap that can be closed
	// look for a pair of intersections with different pt values, but the same t value in 
	//   lastEdge's segment
	// also look for an intersection where the points don't match
	auto checkMissed = [&found = found, &matchPt = matchPt](OpEdge* test, EdgeMatch whichEnd) {
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
	return checkMissed(lastEdge, EdgeMatch::end);
}

// chop records the link end that may be later removed
void OpJoiner::checkLinkupsUnsortables() {
	for (size_t index = 0; index < linkups.l.size(); ++index) {
		OpEdge* linkup = linkups.l[index];
		// while linkup or linkup->lastEdge are unsortable, try them, then try the next
		while (linkup->unsortable) {
			linkup = linkup->nextEdge;
			if (!linkup || lastEdge == linkup)
				break;
			if (linkup->whichPtT().pt == matchPt) {
				found.emplace_back(linkup, linkup->which(), index);
				found.back().chop = ChopUnsortable::prior;
				break;
			}
		}
		OpEdge* lastLink = linkups.l[index]->lastEdge;
		while (lastLink->unsortable) {
			lastLink = lastLink->priorEdge;
			if (!lastLink || lastEdge == lastLink)
				break;
			if (lastLink->whichPtT(EdgeMatch::end).pt == matchPt) {
				found.emplace_back(lastLink, !lastLink->which(), index);
				found.back().chop = ChopUnsortable::next;
				break;
			}
		}
	}
}

void OpJoiner::checkUnsectableGap() {    // find unsectable with small gap
	std::vector<FoundEdge> gaps;
	gaps.push_back(bestGap);
	lastEdge->matchUnsectable(EdgeMatch::end, unsectByArea, gaps, AllowPals::no, AllowClose::yes);
}

void OpJoiner::checkUnsortableAndDisabled() {
	OP_ASSERT(!lastEdge->debugIsLoop());
	matchLeftover(unsortables);
// look for disabled pal that is not in output, and completes loop (cubics14d)
	if (!disabledPalsBuilt)
		buildDisabledPals(*edge->contours());
	matchLeftover(disabledPals);
}

// !!! replace this with proper lookahead that sees if any of the found can connect to an regular 
//     edge in another iteration. For now, cheat (a lot) and put in some mumbo-jumbo number to say
//     that one choice is much better than the others...
FoundEdge* OpJoiner::chooseSmallest() {
    FoundEdge* smallest = &found.front();
	OpPoint start = edge->whichPtT(EdgeMatch::start).pt;
	for (int trial = 0; !OpMath::IsFinite(smallest->perimeter) && trial < 2; ++trial) {
		for (auto& foundOne : found) {
			OpEdge* oppEdge = foundOne.edge;
			// skip edges which flip the fill sum total, implying there is a third edge inbetween
							// e.g., one if normals point to different fill sums       
			if (!trial && !foundOne.connects && !foundOne.loops)
				continue;
			// choose smallest closed loop -- this is an arbitrary choice
			OpPointBounds testBounds = edge->advanceToEnd(EdgeMatch::start)->setLinkBounds();
			if (!edge->containsLink(oppEdge)) {
				OpEdge* firstEdge = oppEdge;
				while (firstEdge->priorEdge)
					firstEdge = firstEdge->priorEdge;
				testBounds.add(foundOne.index > 0 ? firstEdge->setLinkBounds() : firstEdge->ptBounds);
			}
			OpEdge* last = foundOne.edge->lastEdge;
			OP_ASSERT(last);
			foundOne.closeSq = (start - last->whichPtT(EdgeMatch::end).pt).lengthSquared();
			foundOne.perimeter = testBounds.perimeter();
			if (&foundOne == &found.front())
				continue;
			if (smallest->loops < foundOne.loops)
				smallest = &foundOne;
			else if (smallest->loops != foundOne.loops)
				continue;
			if (foundOne.closeSq < OpEpsilon * smallest->closeSq
					|| (smallest->closeSq > OpEpsilon * foundOne.closeSq
					&& foundOne.perimeter < smallest->perimeter)) {	
				smallest = &foundOne;
			}
		}
	}
    return smallest;
}

void OpJoiner::detachChoppedEtc() {
	for (auto& foundOne : found) {
		OpEdge* oppEdge = foundOne.edge;
		foundOne.connects = oppEdge->pals.size() || oppEdge->disabled || oppEdge->unsortable
				|| (((lastEdge->sum.sum() + oppEdge->sum.sum()) & 1) 
		// e.g., one if last start-to-end connects to found start-to-end (end connects to start)
				== (lastEdge->which() != foundOne.whichEnd));
		// if opp edge contains member of linkups via links, detach from this
		// but remember to reattach if candidate is not best
		if (ChopUnsortable::none != foundOne.chop) {
			OpEdge*& unsort = ChopUnsortable::prior == foundOne.chop 
					? oppEdge->priorEdge : oppEdge->nextEdge;
			OP_ASSERT(unsort->unsortable);
			OpEdge* u = unsort;
			(ChopUnsortable::prior == foundOne.chop ? u->nextEdge : u->priorEdge) = nullptr;
			u->setActive(true);
			unsort = nullptr;
			oppEdge->setLastEdge(u);
		}
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

#if 0
// if nothing found to this point, see if forcing a very small edge will fill the bill (cubics10u)
bool OpJoiner::forceSmallEdge() {
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
			found.emplace_back(lastLink, !lastLink->whichEnd);
	}
    return true;
}
#endif

bool OpJoiner::lastLastResort() {
	OP_WARNING(edge->contours(), lastResort);
	OpEdge* filler = edge->segment->contour->addFiller(edge, lastEdge);
	if (!filler)
		return false;
	found.emplace_back(filler, EdgeMatch::start);
    return true;
}

// start here;
// one thing broken with this overall approach is that very small loops get priority over joining
// large incomplete segments. See if it can be restructured to finish big things first, then use 
// the scale of the big things to see if the small remaining things can be ignored
// first, figure out why the current test fails

bool OpJoiner::linkRemaining(OP_DEBUG_CODE(const OpContours* debugContours)) {
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
#if 01 && OP_DEBUG_IMAGE
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
		(void) linkUp(e->setLastEdge(nullptr));
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

void OpJoiner::matchLeftover(const std::vector<OpEdge*>& leftovers) {
	for (OpEdge* nosort : leftovers) {
		if (nosort->inOutput || nosort->inLinkups)
			continue;
		if (edge->linksTo(nosort))
			continue;
		if (matchPt == nosort->start.pt)
			found.emplace_back(nosort, EdgeMatch::start);
		if (matchPt == nosort->end.pt)
			found.emplace_back(nosort, EdgeMatch::end);
	}
	OP_ASSERT(!found.size() || !found.back().edge->debugIsLoop());
}

// at this point all singly linked edges have been found
// every active set of links at this point must form a loop
// the only distance that matters is zero. We should never have unexplained gaps (ideal, not real)
bool OpJoiner::matchLinks(bool popLast) {
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	OP_ASSERT(!edge->priorEdge);
	lastEdge = edge->lastEdge;
	OP_ASSERT(lastEdge);
	OP_ASSERT(!lastEdge->nextEdge);
	OP_ASSERT(EdgeMatch::start == lastEdge->which() || EdgeMatch::end == lastEdge->which());
	found.clear();
	matchPt = lastEdge->whichPtT(EdgeMatch::end).pt;
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
		OpContour* contour = lastEdge->segment->contour;
		OpIntersection* startI = edge->findSect(EdgeMatch::start);
		OpIntersection* gapEnd = tree.bestGapLimb->lastEdge->findSect(!tree.bestGapLimb->match);
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
	if (tree.bestLimb)  // !!! always true: here to avoid unused code warning below
		return tree.join(*this);
	// !!! eventually delete below
	bestGap.reset();  // track the smallest gap available, when all else fails (e.g., battleOp21)
	checkLinkups();
	if (edge != lastEdge)
		bestGap.check(&found, edge, EdgeMatch::start, matchPt);
	if (popLast)  // allows first attempt for edge to scan itself, in case it is an unclosed loop
		linkups.l.pop_back();
    if (!found.size())
	    checkLinkupsUnsortables();  // check past unsortables on the ends of each linkup
	// don't match unsectable with a pal that is already in edge link list
	lastEdge->matchUnsectable(EdgeMatch::end, unsectByArea, found, AllowPals::no, AllowClose::no);
// !!! comment out next line as experiment (thread_cubics502920)
//	if (found.size() && found.back().edge->pals.size())  // thread_cubics502920
		matchLeftover(unsortables);
	if (!found.size())
        checkUnsectableGap();
	if (!found.size() && lastEdge->pals.size())
		matchPals();
	if (!found.size())
        checkGap();  // use best gap if very small
	if (!found.size() && !linkups.l.size() && edge->pals.size() && !edge->priorEdge && !edge->nextEdge)
		return true;  // !!! e.g. cr514118, battleOp255: drop on floor (need general solution)
	if (!found.size()) 
        checkUnsortableAndDisabled();
	if (!found.size())
        checkSectGap();
    if (!found.size())
        checkNothingLeft();
#if 0
	if (!found.size())
        if (!forceSmallEdge())
            return false;
#endif
	if (!found.size())
		lastEdge->matchUnsectable(EdgeMatch::end, unsectByArea, found, AllowPals::yes, AllowClose::no);
	if (!found.size())
        checkDisabled();
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
	if (!found.size())
        lastLastResort();
	OP_ASSERT(edge->contours()->debugExpect == OpDebugExpect::unknown
			|| found.size());
	if (!found.size())  // triggered by fuzz763_1c
		return false;
	// if more than one was found, check to see if selected, one makes a complete loop
	// additionally, store in found how far end is from loop
//start here;
// if there is more than one found, bifurcate and try both paths before deciding which is best
// in addition to using the edge that minimizes the total area, if both edges are pals, unsortable, 
// or disabled, choose the edge that gets us back to a normal, desirable (correct zero crossing) edge
	FoundEdge* smallest = &found.front();
	if (found.size() > 1 || ChopUnsortable::none != smallest->chop) {
        detachChoppedEtc();
        smallest = chooseSmallest();
	}
	return hookup(smallest);
}

bool OpJoiner::hookup(FoundEdge* smallest) {
	OP_ASSERT(smallest->edge); // !!! if found is not empty, but no edge has the right sum, choose one anyway?
	OpEdge* best = smallest->edge;
	(void) best->setLastLink(smallest->whichEnd);  // make edge suitable for linking to a chain
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
		if (smallest->index >= 0) {
			OP_ASSERT((unsigned) smallest->index < linkups.l.size());
			linkups.l.erase(linkups.l.begin() + smallest->index);
		}
	}
	if (best->disabled) {
		auto disabledPos = std::find(disabled.begin(), disabled.end(), best);
		if (disabled.end() != disabledPos)
			disabled.erase(disabledPos);
	}
	// if there is a loop, remove entries in link ups which are output
	linkMatch = EdgeMatch::start;	// since closest prior is set to last edge, use start
	if (detachIfLoop(best, linkMatch))
		return true; // found loop
	edge = edge->advanceToEnd(EdgeMatch::start);
	bool result = matchLinks(false);
	return result;
}

// for each joined edge: if it ends in a pal, see if removing the pal would permit connecting to 
// another joined edge
// !!! this fixes a problem that should have never occurred in the first place
//     when the unsectable edge gets added to a linked list, it should have been excluded if another
//     unsectable edge leads to a better outcome (e.g., the other in turn links to a regular edge)
//     once we have sufficient lookahead, this should no longer be necessary
// (currently needed to fix battleOp177)
void OpJoiner::matchPals() {
	OP_ASSERT(!lastEdge->debugIsLoop());
	OP_ASSERT(!found.size() || !found.back().edge->debugIsLoop());
	// look for member of linkups that ends in a pal of lastEdge
	// figure out how to write (clear) code to recursively remove an edge, fixing up prior/next/last
	for (size_t index = 0; index < linkups.l.size(); ++index) {
		OpEdge* link = linkups.l[index];
		OP_ASSERT(!link->debugIsLoop());
		if (!lastEdge->isPal(link))
			continue;
		OpEdge* nextLink = link->nextEdge;
		if (!nextLink)
			continue;
		if (link->whichPtT(EdgeMatch::end).pt != matchPt)
            continue;
		// if found edge is not best, found needs to be added back to linkups.l
		nextLink->setPriorEdge(nullptr);
		nextLink->lastEdge = link->lastEdge;
		link->setNextEdge(nullptr);
		link->lastEdge = link;
		found.emplace_back(nextLink, nextLink->which());
// !!! doesn't this need to check the other end of the linked list as well?
	}
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
				e->lastEdge = nullptr;
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
				tEdge->lastEdge = nullptr;
			}
			linkupsErasures.push_back(linkIndex);
		};
		if (scanForMatch(edge, EdgeMatch::start))
			mergeLinks(edge, EdgeMatch::start, link);
		else {
			lastEdge = edge->lastEdge;
			if (!scanForMatch(lastEdge, EdgeMatch::end))
				return true;
			mergeLinks(lastEdge, EdgeMatch::end, tIndex);
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