// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegment.h"
#include "PathOps.h"

#if OP_DEBUG_DUMP && !TEST_DEFEAT_BREAK
int debugHits = 0;
#endif

void OpLimb::addEach(OpContour& contour, OpTree& tree) {
	OP_DEBUG_CODE(tree.debugAddEach++);
#if 0 && OP_DEBUG_DUMP
	OpDebugOut(STR(tree.debugAddEach) + ": limb[" + STR(id) + "] contour: " + STR(contour.id) 
			+ " pass:" + debugLimbPass(tree.limbPass) + "\n");
#endif
if (looped || deadEnd)  // triggered when walking children of trunk 
		return;
	if (resetPass) {
		tree.limbPass = LimbPass::linked;
		resetPass = false;
	}
	size_t linkupsSize = contour.linkups.l.size();
	LimbPass pass = tree.limbPass;
	LimbPass linkedLimb = LimbPass::miswound <= pass ? LimbPass::miswound : LimbPass::linked;
	for (unsigned index = 0; index < linkupsSize; ++index) {
		OpEdge* test = contour.linkups.l[index];
		if (test->disabled)
			continue;
		tryAdd(tree, test, EdgeMatch::start, linkedLimb, &contour, index);
	}
	size_t endLinksSize = contour.endLinks.l.size();
	for (unsigned endIndex = 0; endIndex < endLinksSize; ++endIndex) {
		OpEdge* test = contour.endLinks.l[endIndex];
		OpEdge* last = test->lastEdge;
		if (last->disabled)
			continue;
		tryAdd(tree, last, EdgeMatch::end, linkedLimb, &contour, endIndex, test);
	}
	if (LimbPass::linked == pass)
		return;
#if OP_DEBUG_DUMP && !TEST_DEFEAT_BREAK
	if (6926 == id && 42 == contour.id) {
		++debugHits;
		OpDebugOut("debugHits: " + STR(debugHits) + "\n");
		if (debugHits == 8) {
#if OP_DEBUG_IMAGE
			playback();
			hideTemporaryEdges();
			colorActive(transparent);
			colorDisabled(transparent);
			colorUnsectables(transparent);
			colorUnsortables(transparent);
			colorLinkups(green);
			OpAssert(0);
#endif
		}
	}
#endif
	for (const std::vector<OpEdge*>& edges : { contour.unsectByArea, contour.unsortables } ) {
		for (OpEdge* test : edges) {
			if (test->inLinkups)
				continue;
			// !!! test here is too soon : check after finding that unsectable extends tree
		#if 0
			bool preferSibling = test->isUnsectable() && tree.preferSibling(this, test);
			if (preferSibling) //  && LimbPass::unsectPair > tree.limbPass)
				continue;
		#endif
			tryAdd(tree, test, EdgeMatch::start, LimbPass::unlinked); 
			tryAdd(tree, test, EdgeMatch::end, LimbPass::unlinked);
		}
	}
	if (LimbPass::unlinked == pass)
		return;
	if (!contour.centerlessBuilt)
		contour.buildCenterless();
	for (OpEdge* test : contour.disabledCenterless) {
		tryAdd(tree, test, EdgeMatch::start, LimbPass::disabledCenterless);
		tryAdd(tree, test, EdgeMatch::end, LimbPass::disabledCenterless);
	}
	if (LimbPass::disabledCenterless == pass)
		return;
	if (!contour.palsBuilt && contour.hasPals) 
		contour.buildPals();
	for (OpEdge* test : contour.disabledPals) {
		tryAdd(tree, test, EdgeMatch::start, LimbPass::disabledPals);
		tryAdd(tree, test, EdgeMatch::end, LimbPass::disabledPals);
	}
	if (LimbPass::disabledPals == pass)
		return;
	if (LimbPass::miswound == pass)
		return; 
	// iterate through edge pals looking for gap that connects lastPt via sect opp
	// unsectable edges do not necessarily point to other unsectable through pals or upairs
	if (lastLimbEdge->isUnsectable()) {
		for (EdgePal& edgePal : lastLimbEdge->pals) {
			if (edgePal.edge->disabled)
				continue;
			if (!edgePal.edge->isUnsectable())
				continue;
			tryAdd(tree, edgePal.edge, edgePal.reversed ? match : !match, LimbPass::unsectPair);
#if 0
			OpEdge* test = edgePal.edge + (EdgeMatch::start == match ? 1 : -1);
			OpSegment* palSeg = edgePal.edge->segment;
			if (&palSeg->edges.front() <= test && test <= &palSeg->edges.back())
				tryAdd(tree, test, edgePal.reversed ? !match : match, LimbPass::unsectPair);
#endif
		}
	}
	if (LimbPass::unsectPair == pass)
		return;
	if (LimbPass::disjoint == pass)
		return;
	if (LimbPass::unlinkedPal == pass)
		return;
	OP_ASSERT(LimbPass::disabledBackwards == pass);
	// check if tree's best distance is small enough with a user-provided multiplier
	// (motivated by loop193532 whose best distance 9.53e-7 exceeds threshold length 8.18e-7;
	//  an alternative would be to find the intersection of edges 198 and 208 -- somehow missed)
	if (tree.gap(tree.bestDistance)) {
		tree.smallGap = true;
		return;
	}
	if (!contour.backwardsBuilt)
		contour.buildBackwards();
	for (OpEdge* test : contour.disabledBackwards) {
		tryAdd(tree, test, EdgeMatch::start, LimbPass::disabledBackwards);
		tryAdd(tree, test, EdgeMatch::end, LimbPass::disabledBackwards);
	}
}

void OpLimb::set(OpTree& tree, OpEdge* test, OpLimb* p, EdgeMatch m, LimbPass l, OpContour* contour,
		size_t index, OpEdge* otherEnd, const OpPointBounds* childBounds) {
	OP_DEBUG_DUMP_CODE(id = tree.context->nextID());
	edge = test;
	parent = p;
	linkedContour = contour;
	linkedIndex = (uint32_t) index;
	gapDistance = 0;
	match = m;
	treePass = l;
	resetPass = true;
	looped = false;
	deadEnd = false;
	if (LimbPass::linked != treePass && LimbPass::miswound != treePass) {
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
	if (childBounds) {
		bounds = *childBounds;
		looped = tree.firstPt == lastPtT.pt;
		const OpLimb* testParent = p;
		while (testParent) {
			if (testParent->lastPtT.pt == lastPtT.pt) {
				deadEnd = true;
				break;
			}
			testParent = testParent->parent;
		}
	}
	if (looped && tree.bestPerimeter > bounds.perimeter()) {
		tree.bestPerimeter = bounds.perimeter();
		tree.bestLimb = this;
	}
	closeDistance = (lastPtT.pt - tree.firstPt).length();
	if (tree.bestDistance > closeDistance) {
		tree.bestDistance = closeDistance;
		tree.bestGapLimb = this;
	}
	OP_DEBUG_DUMP_CODE(if (p) p->debugBranches.push_back(this));
	OP_DEBUG_IMAGE_CODE(tree.debugLimbEdges(edge));
}

OpLimb* OpLimb::tryAdd(OpTree& tree, OpEdge* test, EdgeMatch m, LimbPass limbPass, 
			OpContour* limbContour, size_t limbIndex, OpEdge* otherEnd) {
	OP_ASSERT(!test->disabled || test->isUnsectable() || Unsortable::none != test->isUnsortable 
			|| LimbPass::disabledCenterless <= limbPass);
	OP_ASSERT(!test->hasLinkTo(m) || Unsortable::none != test->isUnsortable || test->disabled 
			|| test->isUnsectable());
	int ccUnsectID = 0;
	OpPoint testPt = test->whichPtT(m).pt;
	if (testPt != lastPtT.pt) {
		if (LimbPass::unsectPair != tree.limbPass)
			return nullptr;
		if (Unsortable::filler == edge->isUnsortable)
			return nullptr;
		// check if filler from test to test's pal can connect to last point
		for (const EdgePal& testPal : test->pals) {
			if (testPal.edge == edge)
				continue;
			OpPoint palPt = testPal.matchPt(m);
			if (palPt == lastPtT.pt) {
				for (OpIntersection* testSect : testPal.edge->unSects) {
					if (testSect->ccUnsectable) {
						ccUnsectID = testSect->unsectID;
						goto connectWithFiller;
					}
				}
				if (testPt == tree.firstPt) {
					ccUnsectID = testPal.unsectID;
					goto connectWithFiller;
				}
			}
		}
		// check if filler added from limb to limb pal can connect to test point
		for (const EdgePal& limbPal : edge->pals) {
			if (limbPal.edge == test)
				continue;
			OpPoint palPt = limbPal.matchPt(lastMatch);
			if (testPt == palPt) {
				for (OpIntersection* palSect : limbPal.edge->unSects) {
					if (palSect->ccUnsectable) {
						ccUnsectID = palSect->unsectID;
						goto connectWithFiller;
					}
				}
			}
		}
		return nullptr;
connectWithFiller:
		if (tree.containsFiller(abs(ccUnsectID)))
			return nullptr;
	}
	if (edge == test)
		return nullptr;
//	if (test->isUnsectable() && LimbPass::unlinkedPal != tree.limbPass && test->unsectableSeen(m))
//		return nullptr;
	if (test->isUnsectable() && tree.preferSibling(this, test))
		return nullptr;
	OP_ASSERT(lastLimbEdge);
	if (LimbPass::miswound == limbPass && lastLimbEdge == test)
		return nullptr;
	if (LimbPass::unsectPair != limbPass && tree.contains(this, test))
		return nullptr;
	bool loopedToFirstPoint = tree.firstPt == testPt;
	if (!loopedToFirstPoint && (EdgeMatch::start == m ? test->startSeen : test->endSeen))
		return nullptr;
	// compare test wind zero against their parent's last edge wind zero
	// OP_ASSERT(!test->isPal(last) || LimbPass::linked != limbPass);  // breaks pentrek10
	// Edge direction and winding are tricky (see description at wind zero declaration.)
	// For first edge (and its last) in storage: if which end is 'end', its wind zero is reversed.
	// 'Test' may need to be reversed to connect, and 'm' may be either end. Wind zero in both cases
	// is computed for the unreversed orientation.
	if (WindZero::unset != lastLimbEdge->windZero && WindZero::unset != test->windZero
			&& (LimbPass::linked == limbPass || LimbPass::miswound == limbPass)
			&& !lastLimbEdge->many.isSet() && Unsortable::filler != lastLimbEdge->isUnsortable) {
		WindZero zeroSide = test->windZero;
		// if last which end is end, flip last's wind zero (for comparsion, flip zero side);
		// if pass is linked: if test m is end, flip zero side; if test which is end, flip zero side
		// if pass is miswound, flip the logic
		if (((lastLimbEdge->which() == match) != (test->which() == m)) 
				== (LimbPass::linked == limbPass))
			zeroSide = !zeroSide;
		if (lastLimbEdge->windZero != zeroSide)
			return nullptr;
	}
	OpPointBounds childBounds = test->lastEdge ? test->linkBounds : 
			otherEnd ? otherEnd->linkBounds : test->bounds;
	if (parent && LimbPass::disjoint != treePass)  // if not trunk
		childBounds.add(bounds);
	if (test->inLinkups)
		(EdgeMatch::start == m ? test->startSeen : test->endSeen) = true;
	// if this edge added to limb bounds makes perimeter larger than best, skip
	// !!! are their cases where smallest perimeter is not the best test?
	// first, multiple edges with the same start and end point may share perimeters
	// is larger perimeter more desirable if it avoids enclosing another contour?
	// should this use the ray edges to see if test is closer to test edge?
	// note that best may not have ray to edge; e.g., outline of 'O' (edge contains inner contour)
	if (childBounds.perimeter() > tree.bestPerimeter)
		return nullptr;
	OpLimb* newParent = this;
	if (LimbPass::unsectPair == tree.limbPass) {
		OpPtT startI = test->whichPtT(m);
		if (lastPtT.pt == startI.pt) 
			return nullptr;
		if (tree.containsFiller(this, lastPtT.pt, startI.pt))
			return nullptr;
		if (tree.bestGapLimb) { 
			float gap = (lastPtT.pt - startI.pt).length();
			if (tree.bestDistance < gap)
				return nullptr;
		}
		OpEdge* filler = tree.addFiller(lastLimbEdge->segment, lastPtT, startI, true);
		filler->setWhich(EdgeMatch::start);
		OpLimb* fillerBranch = tree.makeLimb();
		fillerBranch->set(tree, filler, this, EdgeMatch::start, tree.limbPass, limbContour,
				limbIndex, nullptr, &filler->bounds);
		fillerBranch->edge->ccUnsectID = abs(ccUnsectID);
		fillerBranch->gapDistance = (startI.pt - lastPtT.pt).length();
		if (loopedToFirstPoint)
			return fillerBranch;
		newParent = fillerBranch;
	}
	if (tree.containsParent(this, test, m))
		return nullptr;
	OpLimb* branch = tree.makeLimb();
	// !!! if some upper number of limbs are made, return fail instead of running forever
	//      let caller supply limit?
	branch->set(tree, test, newParent, m, limbPass, limbContour, limbIndex, otherEnd, &childBounds);
	return branch;
}

OpTree::OpTree(OpJoiner& join) 
	: context(join.edge->segment->contour->context)
	, bestGapLimb(nullptr)
	, bestLimb(nullptr)
	, firstPt(join.edge->whichPtT().pt)
	, limbPass(LimbPass::linked)
	, bestDistance(OpInfinity)
	, bestPerimeter(OpInfinity)
	, totalUsed(0) 
	, smallGap(false) {
	id = context->nextID();
	maxLimbs = context->contextCallbacks.maxLimbsFuncPtr ?
			context->contextCallbacks.maxLimbsFuncPtr((PathOpsV0Lib::Context*) context) : 1000;
	OP_DEBUG_CODE(context->debugTree = this);
	OP_ASSERT(join.edge->inLinkups);
	OP_DEBUG_IMAGE_CODE(context->debugLimbClear());
	OP_DEBUG_IMAGE_CODE(debugLimbEdges(join.edge));
// Mark edges' 'seen' as unset in this tree. Later, mark additional contours as they are linked
	OpContour* edgeContour = join.edge->segment->contour;
	for (auto member : edgeContour->members()) {
		member->setSeen(id);
	}
	context->resetLimbs();
	OpLimb* trunk = makeLimb();
	OP_ASSERT(edgeContour->linkups.l.back() == join.edge);
	trunk->set(*this, join.edge, nullptr, EdgeMatch::start, LimbPass::linked, 
			edgeContour, edgeContour->linkups.l.size() - 1, join.edge);
	join.edge->startSeen = true;
	join.edge->lastEdge->endSeen = true;
	// !!! can I know that join.edge never has prior, and is never loop?
	do {
		for (auto member : edgeContour->members()) {
			initialize(*member);
		}
	#if 0 // !!! experiment: try adding disabled pals as regular entries in tree
		if (LimbPass::disabledPals == limbPass) {
			OpLimb* unsectEnd = unsectableLoop();
			if (unsectEnd) {
				addUnsectableLoop(join, unsectEnd);
				return;
			}
			for (auto contour : edgeContour->sects) {
				addDisabled(*contour);
			}
		} else 
	#endif	
		{
			int index = 0;
			do {
				OpLimb& limb = nthLimb(index);
				OpEdge* endEdge = limb.edge;
				if (endEdge->lastEdge)
					endEdge = endEdge->lastEdge;
				else if (endEdge->priorEdge)
					endEdge = endEdge->advanceToEnd(EdgeMatch::start);
				for (auto member : endEdge->segment->contour->members()) {
					if (member->treeID != id)
						member->setSeen(id);
					limb.addEach(*member, *this);
				}
				if (totalUsed > maxLimbs) {
			#if 0 // TEST_ANALYZE // for grshapearcs 
					playback();
//					showLimbs();
					hideDisabled();
			#endif
					context->setError(PathOpsV0Lib::ContextError::tree  
							OP_DEBUG_PARAMS(join.edge->id));
					return;
				}
			} while (++index < totalUsed);
		}
		if (LimbPass::disabledBackwards < ++limbPass)
			return;  // error if bestLimb == nullptr
	} while (!bestLimb);
}

// walk the disabled pals from smallest to largest instead of the limbs
// add the least disturbing disabled pal to any limb that matches (that also disturbs least)
// !!! may need to treat regular disabled the same, although pals are more legit ?
void OpTree::addDisabled(OpContour& contour) {
	for (OpEdge* test : contour.disabledPals) {
		test->unlink();
		// check every limb for point match; choose based on limbPass, then bounds
		int index = 0;
		do {
			OpLimb& limb = nthLimb(index);
			if (limb.looped || limb.deadEnd)  // triggered when walking children of trunk 
				continue;
			if (limb.resetPass) {
				limbPass = LimbPass::none;  // incremented on return to 'linked'
				limb.resetPass = false;
				return;
			}
			auto putPal = [this, &limb, &test](EdgeMatch match) {
				for (OpIntersection* unSect : test->unSects) {
					OpIntersection* unOpp = unSect->opp;
					if (unOpp->ptT.pt == unSect->ptT.pt)
						continue;
					OpEdge* filler = addFiller(unSect->segment, unSect->ptT, unOpp->ptT, false);
					filler->setWhich(EdgeMatch::start);
					OpLimb* branch = makeLimb();
					branch->set(*this, filler, &limb, match, LimbPass::disjoint, 
							nullptr, 0, nullptr);
				}
			};
			if (limb.tryAdd(*this, test, EdgeMatch::start, LimbPass::disabledPals))
				putPal(EdgeMatch::start);
			if (limb.tryAdd(*this, test, EdgeMatch::end, LimbPass::disabledPals))
				putPal(EdgeMatch::end);
		} while (++index < totalUsed);
	}
}

OpEdge* OpTree::addFiller(OpSegment* seg, const OpPtT& ptT1, const OpPtT& ptT2, bool fromCC) {
	if (!fromCC) {
		float fillerLength = (ptT1.pt - ptT2.pt).length();
		if (!gap(fillerLength)) {
			OP_DEBUG_CODE(OpDebugOut("\n" + seg->contour->context->debugData.testname + "\n"));
			OP_DEBUG_DUMP_CODE(::debug());
			OP_DEBUG_CODE(OpDebugOut("\n"));
			OP_DEBUG_DUMP_CODE(dump());
			context->setError(PathOpsV0Lib::ContextError::gap  OP_DEBUG_PARAMS(id));
		}
		OP_ASSERT(OpJoiner::DebugShowImage());
	}
	OpEdge* result = context->addFiller(ptT1, ptT2);
	result->segment = seg;
	return result;
}

void OpTree::addUnsectableLoop(OpJoiner& joiner, OpLimb* end) {
	OpEdge* joinEdge = joiner.edge;
	OpPtT startI = joinEdge->whichPtT(EdgeMatch::start);
	OpEdge* filler = addFiller(end->edge->segment, end->lastPtT, startI, false);
	OpLimb* limb = makeLimb();
	limb->set(*this, filler, end, EdgeMatch::start, LimbPass::disjoint, 
			nullptr, 0, nullptr);
	if (!bestLimb)
		bestLimb = limb;
}

bool OpTree::contains(OpLimb* parent, OpEdge* edge) const {
	OpLimbStorage* limbs = context->limbCurrent;
	while (limbs) {
		int index = limbs->used;
		OP_ASSERT(index > 0);
		do {
			OpLimb* testLimb = &limbs->storage[--index];
			if (testLimb->edge == parent->edge)
				return false;
			if (testLimb->parent->edge != parent->edge)
				continue;
			if (testLimb->edge == edge)
				return true;
		} while (index > 0);
		limbs = limbs->prevBlock;
	}
	return false;
}

bool OpTree::containsParent(OpLimb* parent, OpEdge* edge, EdgeMatch m) const {
	OpLimbStorage* limbs = context->limbCurrent;
	while (limbs) {
		int index = limbs->used;
		OP_ASSERT(index > 0);
		do {
			OpLimb* testLimb = &limbs->storage[--index];
			if (testLimb->parent != parent)
				continue;
			if (testLimb->match != m)
				continue;
			if (testLimb->edge == edge)
				return true;
		} while (index > 0);
		limbs = limbs->prevBlock;
	}
	return false;
}

// !!! maybe more than 1 identical filler is possible ?! wait for test case to implement
// !!! in some rare case, there may be a ton of fillers; if this ever occurs, more filler
// to contour to minimize this search
bool OpTree::containsFiller(OpLimb* parent, OpPoint pt1, OpPoint pt2) const {
	return context->containsFiller(pt1, pt2);
}

bool OpTree::containsFiller(int ccUnsectableID) const {
	return context->containsFiller(ccUnsectableID);
}

bool OpTree::gap(float distance) const {
	PathOpsV0Lib::MaxGap gapFuncPtr = context->contextCallbacks.maxGapFuncPtr;
	float gapFactor = gapFuncPtr ? (*gapFuncPtr)((ContextPtr) context) : 2.f;
	return distance <= context->aliases.thresholdLength * gapFactor;
}

void OpTree::initialize(OpContour& contour) {
	switch (limbPass) {
		case LimbPass::linked:
			break;
		case LimbPass::unlinked: 
			for (const std::vector<OpEdge*>& edges : { contour.unsectByArea, contour.unsortables } )
				for (OpEdge* test : edges)
					contour.unlink(test);
			break;
		case LimbPass::unsectPair:
			break;
		case LimbPass::disabledCenterless:
			if (contour.centerlessBuilt)
				for (OpEdge* test : contour.disabledCenterless)
					contour.unlink(test);
			break;
		case LimbPass::disabledPals:
			if (contour.palsBuilt)
				for (OpEdge* test : contour.disabledPals)
					contour.unlink(test);
			break;
		case LimbPass::miswound:
			break;
		case LimbPass::disjoint:
			break;
		case LimbPass::unlinkedPal:
			break;
		case LimbPass::disabledBackwards:
			if (contour.backwardsBuilt)
				for (OpEdge* test : contour.disabledBackwards)
					contour.unlink(test);
			break;
		default:
			OP_ASSERT(0);
	}
}

// join best limb to edge start, then parent to best limb, until lastEdge is found
bool OpTree::join(OpJoiner& join) {
	std::vector<OpEdge*> linkupsErasures;
	const OpLimb* bestL = bestLimb;
	OpEdge* best = bestL->edge;
	if (EdgeMatch::end == bestL->match) {	
		(void) best->setLastLink(!best->which()); // make suitable for linking to a chain
		best = best->advanceToEnd(EdgeMatch::start);
	} else if (best != best->lastEdge)
		(void) best->setLastLink(EdgeMatch::start);
	if (LimbPass::linked == bestL->treePass || LimbPass::miswound == bestL->treePass) {
		if (EdgeMatch::start == bestL->match) {
			OpEdge* eraseLink = bestL->linkedContour->linkups.l[bestL->linkedIndex];
	#if OP_DEBUG_VALIDATE
			eraseLink->debugScheduledForErasure = true;
	#endif
			linkupsErasures.push_back(eraseLink);
		}
	}
	while (const OpLimb* lastLimb = bestL->parent) {
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
		prior->setLastEdge(best, best->lastEdge, InOutput::yes);
		OP_ASSERT(!last->debugIsLoop());
		if (LimbPass::linked == lastLimb->treePass || LimbPass::miswound == lastLimb->treePass) {
			if (EdgeMatch::start == lastLimb->match) {
				OpEdge* eraseLink = lastLimb->linkedContour->linkups.l[lastLimb->linkedIndex];
				if (linkupsErasures.end() == std::find(linkupsErasures.begin(), 
						linkupsErasures.end(), eraseLink)) {
#if OP_DEBUG_VALIDATE
					eraseLink->debugScheduledForErasure = true;
#endif
					linkupsErasures.push_back(eraseLink);
				}
			}
		}
		bestL = lastLimb;
		best = bestL->edge;
	}
	OP_TRACK(linkupsErasures);
	for (OpEdge* edge : linkupsErasures) {
		OpContour& contour = *edge->segment->contour;
		if (edge != join.edge && edge->lastEdge) {
			OpContour* lastContour = edge->lastEdge->segment->contour;
			for (size_t lastIndex = 0; lastIndex < lastContour->endLinks.l.size(); ++lastIndex) {
				if (lastContour->endLinks.l[lastIndex] == edge) {
					lastContour->endLinks.l.erase(lastContour->endLinks.l.begin() + lastIndex);
					edge->lastEdge = nullptr;
					break;
				}
			}
		}
#if OP_DEBUG_VALIDATE
		OP_ASSERT(edge->debugScheduledForErasure);
		edge->debugScheduledForErasure = false;
#endif
		if (edge->linkHead) {
			for (size_t index = 0; index < contour.linkups.l.size(); ++index) {
				if (contour.linkups.l[index] == edge) {
					contour.linkups.l.erase(contour.linkups.l.begin() + index);
					edge->linkHead = false;
					break;
				}
			}
			OP_ASSERT(!edge->linkHead);
		}
	}
	join.edge->output(false);
	OP_DEBUG_VALIDATE_CODE(join.debugValidate());

	context->resetLimbs();
	context->resetFiller();  // may delete edge that another edge references in prior/next
	return true;
}

// used to walk tree in breadth order
OpLimb& OpTree::nthLimb(int index) {
	return context->nthLimb(index);
}

OpLimb* OpTree::makeLimb() {
	++totalUsed;
	return context->allocateLimb();
}

// !!! this code does not do what is described
// breaks testQuads25988731; disabled to find test case for rewrite
bool OpTree::preferSibling(OpLimb* palParent, OpEdge* edge) {
#if 0
// try siblings to see if they are linkable, and can be extended (preferable)
	OP_ASSERT(palParent);
	// if limb edge and test edge are the only connections, don't check sibling linkage
	int index = totalUsed;
	OP_ASSERT(index > 0);
	for (OpLimb* sib; (sib = &nthLimb(--index)) && sib != palParent; ) {
		if (sib->parent != palParent)
			continue;
		if (sib->edge->isUnsectable() || Unsortable::none != sib->edge->isUnsortable)
			continue;
		std::vector<EdgePal>& dists = sib->edge->ray.distances;
		// if sib ray dists include edge but not edge pal, prefer sib
		if (std::any_of(dists.begin(), dists.end(), [edge](const EdgePal& dist) {
				return dist.edge == edge; }))
			continue;
		if (std::any_of(dists.begin(), dists.end(), [edge](const EdgePal& dist) {
			std::vector<EdgePal>& pals = edge->pals;
			return std::none_of(pals.begin(), pals.end(), [dist](const EdgePal& pal) {
				return dist.edge == pal.edge; });
		}))
			return true;
	}
#endif
	return false;
}

// if disabled pals limb pass is reached, check to see if tree can be closed by connecting unsects
OpLimb* OpTree::unsectableLoop() const {
	OpLimb& trunk = context->nthLimb(0);
	OpSegment* trunkSeg = trunk.edge->segment;
	std::vector<OpIntersection*> startUnsects = trunkSeg->sects.unsectables(firstPt);
	if (startUnsects.empty())
		return nullptr;
	OpLimb* smallest = nullptr;  // return smallest gap
	float bestDist = OpInfinity;
	for (int index = 0; index < totalUsed; ++index) {
		OpLimb& test = context->nthLimb(index);
		OpSegment* testSeg = test.lastLimbEdge->segment;
		if (!testSeg)	// if limb is filler, edge won't have a parent segment
			continue;
		std::vector<OpIntersection*> testUnsects = testSeg->sects.unsectables(test.lastPtT.pt);
		for (OpIntersection* testSect : testUnsects) {
			float testDist = (testSect->ptT.pt - firstPt).length();
			if (testDist > bestDist) 
				continue;
			for (OpIntersection* trunkSect : startUnsects) {
				if (trunkSect->unsectID != testSect->unsectID) 
					continue;
				smallest = &test;
				bestDist = testDist;
				break;
			}
		}
	}
	return smallest;
}

// caller (in contours) has allocated storage already
OpLimb* OpLimbStorage::allocate() {
	OP_ASSERT(used < (int) ARRAY_COUNT(storage));
	OpLimb& result = storage[used++];
	OP_DEBUG_CODE(result = OpLimb());
	return &result;
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
	baseIndex = 0;
	nextBlock = nullptr;
	prevBlock = nullptr;
	used = 0;
}

OpJoiner::OpJoiner(OpContext& contours)
	: context(&contours),
      linkMatch(EdgeMatch::none)
	, linkPass(LinkPass::none)
	, edge(nullptr)
	, lastLink(nullptr)
	OP_DEBUG_PARAMS(debugRecursiveDepth(0)) {
	for (auto contour : contours.contours) {
		for (auto& segment : contour->segments) {
			for (auto& e : segment.edges) {
				if (e.inOutput)
					continue;
				contour->addJoinEdge(this, &e);
			}
		}
		contour->backwardsBuilt = false;
		contour->centerlessBuilt = false;
		contour->palsBuilt = false;
	}
	sort();
	OP_DEBUG_CODE(contours.debugJoiner = this);
	OP_DEBUG_VALIDATE_CODE(debugValidate());
}

// start here;
// one thing broken with this overall approach is that very small loops get priority over joining
// large incomplete segments. See if it can be restructured to finish big things first, then use 
// the scale of the big things to see if the small remaining things can be ignored
// first, figure out why the current test fails

bool OpJoiner::linkRemaining(OpContour* contour) {
	OP_DEBUG_CONTEXT();
	LinkUps& linkups = contour->linkups;
	OP_ASSERT(DebugShowImage());
	linkPass = LinkPass::remaining;
	// match links may add or remove from link ups. Iterate as long as link ups is not empty
	for (auto e : linkups.l) {
		e->setLinkBounds();
	}
	OP_DEBUG_CODE(int debugLoopCounter = 0);
	while (linkups.l.size()) {
		// sort to process largest first
		// !!! could optimize to avoid search, but for now, this is the simplest
		linkups.sort();
		edge = linkups.l.back();
		OP_DEBUG_VALIDATE_CODE(debugValidate());
		if (!matchLinks(true))
			return false;
		// contour generated by match links may allow for link up edges to now have a single link
		RelinkJoins relink = RelinkJoins::uninitialized;
		do {
			for (OpContour* member : contour->members()) {
				size_t linkupsIndex = 0;
				for (;;) {
					relink = member->relinkUnambiguous(this, linkupsIndex);
					if (RelinkJoins::unmatched != relink)
						break;
					++linkupsIndex;
				}
				if (RelinkJoins::unchanged == relink || RelinkJoins::again == relink)
					break;
			}
		} while (RelinkJoins::again == relink);
	#if SHOW_DEBUG_IMAGE && OP_DEBUG_IMAGE
		redraw();
	#endif
		OP_DEBUG_VALIDATE_CODE(debugValidate());
		OP_DEBUG_CODE(if (++debugLoopCounter < 0) OpDebugOut(""));  // allows seeing loop iteration that failed
	}
	return true;
}

OpEdge* OpJoiner::LinkStart(OpEdge* first) {
	OpEdge* edge = first;
	do {
		OpSegment* seg = edge->segment;
		OP_ASSERT(!seg->sects.unsorted);
		OpIntersection* prevSect = seg->sects.i.front();
		OpSegment* opp = prevSect->opp->segment;
		OP_ASSERT(!opp->disabled);
		OpEdge* prevEdge = &opp->edges.back();
		if (!prevEdge->isSimple())
			break;
		if (!opp->simpleEnd(prevEdge) || prevSect->opp != opp->sects.i.back())
			break;
		OP_ASSERT(!prevEdge->nextEdge);
		prevEdge->setNextEdge(edge);
		prevEdge->setWhich(EdgeMatch::start);
		OP_ASSERT(!edge->priorEdge);
		edge->setPriorEdge(prevEdge);
		edge = prevEdge;
		OP_ASSERT(!edge->disabled);
	} while (edge->segment->simpleStart(edge));
	return edge;
}

bool OpJoiner::LinkEnd(OpEdge* first) {
	OpEdge* edge = first;
	do {
		OpSegment* seg = edge->segment;
		OP_ASSERT(!seg->sects.unsorted);
		OpIntersection* nextSect = seg->sects.i.back();
		OpSegment* opp = nextSect->opp->segment;
		OP_ASSERT(!opp->disabled);
		OpEdge* nextEdge = &opp->edges.front();
		if (!nextEdge->isSimple())
			break;
		if (!opp->simpleStart(nextEdge) || nextSect->opp != opp->sects.i.front())
			break;
		if (nextEdge->priorEdge) {
			OP_ASSERT(edge->nextEdge == nextEdge);
			OpEdge* last = first->setLastEdge();
			OP_ASSERT(!last->nextEdge);
			last->setNextEdge(first);
			OP_ASSERT(!first->priorEdge);
			first->setPriorEdge(last);
			first->outputLinkedList(first, true);
			return false;
		}
		OP_ASSERT(!nextEdge->priorEdge);
		nextEdge->setPriorEdge(edge);
		nextEdge->setWhich(EdgeMatch::start);
		OP_ASSERT(!edge->nextEdge);
		edge->setNextEdge(nextEdge);
		if (nextEdge->nextEdge && edge->priorEdge) {
			first->outputLinkedList(first, true);
			return false;
		}
		edge = nextEdge;
		OP_ASSERT(!edge->segment->disabled);
		if (!edge->isSimple())
			break;
	} while (edge->segment->simpleEnd(edge));
	return true;
}

#if 0
bool OpJoiner::linkSimple(OpEdge* first) {
	if (first->priorEdge || first->nextEdge) {
		OP_ASSERT(EdgeMatch::start == first->which());
		addToLinkups(first);
		return true;
	}
	return false;
}
#endif

void OpJoiner::linkUnambiguous(OpContour* contour, LinkPass lp) {
	OP_DEBUG_CONTEXT();
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	// match up edges that have only a single possible prior or next link, and add them to new list
	linkPass = lp;
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	std::vector<OpEdge*>& edges = LinkPass::normal == lp ? contour->byArea : contour->unsectByArea;
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
		OP_DEBUG_CODE(debugRecursiveDepth = 0);
		if (!contour->linkUp(this, e))
			continue;
		OP_DEBUG_VALIDATE_CODE(debugValidate());
		linkMatch = EdgeMatch::end;
		OP_DEBUG_CODE(debugRecursiveDepth = 0);
		OpEdge* lastEdge = e->setLastEdge();
		lastEdge->segment->contour->linkUp(this, lastEdge);
		OP_DEBUG_VALIDATE_CODE(debugValidate());
	}
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
//	found.clear();
	matchPt = lastLink->whichPtT(EdgeMatch::end).pt;
	OpTree tree(*this);
	if (PathOpsV0Lib::ContextError::none != context->error)
		return false;
	// adding gap edge in unsect pair case
	if (!tree.bestLimb) {
		OpLimb* gap = tree.bestGapLimb;
		OP_ASSERT(gap);
		if (!context->errorHandler.errorDispatchFuncPtr
				|| context->errorHandler.errorDispatchFuncPtr(
				PathOpsV0Lib::ContextError::missing, *context, nullptr)) {
			OpPtT startI = edge->whichPtT(EdgeMatch::start);
			OpPtT gapEnd = gap->lastLimbEdge->whichPtT(!gap->match);
			OpEdge* filler = tree.addFiller(gap->lastLimbEdge->segment, gapEnd, startI, false);
			OpLimb* branch = tree.makeLimb();
			branch->set(tree, filler, gap, EdgeMatch::start, LimbPass::disjoint, 
					nullptr, 0, nullptr);
			tree.bestLimb = branch;
		} else {
			tree.bestLimb = gap;
		}
	}
	return tree.join(*this);
}

// sort by size so that tiny edges with poor winding don't run the show
void OpJoiner::sort() {
	for (auto contour : context->contours) {
		contour->joinSort();
	}
}

// sort by size to process largest (tail) first
// sort should consider all edges in link
//start here;
// sort so that first, in addition to being largest, is also on the overall outside border
void LinkUps::sort() {
	OpPointBounds bounds;
	for (auto& linkList : l) {
		OP_ASSERT(linkList->linkBounds.isFinite());
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
