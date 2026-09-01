// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegment.h"
#include "PathOps.h"
#if OP_TEST_RASTER
#include "OpDebugRaster.h"
#endif

#if OP_DEBUG_DUMP && !TEST_DEFEAT_BREAK
int debugHits = 0;
#endif

void OpLimb::addEach(OpContour& contour) {
	OP_DEBUG_CODE(tree->debugAddEach++);
#if 0 && OP_DEBUG_DUMP
	OpDebugOut(STR(tree->debugAddEach) + ": limb[" + STR(id) + "] contour: " + STR(contour.id) 
			+ " pass:" + debugLimbPass(tree->limbPass) + "\n");
#endif
	if (looped || deadEnd)  // triggered when walking children of trunk 
		return;
	size_t linkupsSize = contour.linkups.l.size();
	LimbPass pass = tree->limbPass;
	switch (pass) {
		case LimbPass::linked:
		case LimbPass::miswound:
		case LimbPass::disjoint: {
			for (unsigned index = 0; index < linkupsSize; ++index) {
				OpEdge* test = contour.linkups.l[index];
				if (test->disabled || test->startSeen)
					continue;
				tryAdd(test, EdgeMatch::start, &contour, index);
			}
			size_t endLinksSize = contour.endLinks.l.size();
			for (unsigned endIndex = 0; endIndex < endLinksSize; ++endIndex) {
				OpEdge* test = contour.endLinks.l[endIndex];
				OpEdge* last = test->lastEdge;
				if (last->disabled || last->endSeen)
					continue;
				tryAdd(last, EdgeMatch::end, &contour, endIndex, test);
			}
			} break;
		case LimbPass::unlinked:
			for (const std::vector<OpEdge*>& edges : { contour.unsectByArea, contour.unsortables } ) {
				for (OpEdge* test : edges) {
					if (test->inLinkups)
						continue;
					if (!test->startSeen)
						tryAdd(test, EdgeMatch::start); 
					if (!test->endSeen)
						tryAdd(test, EdgeMatch::end);
				}
			}
			break;
		case LimbPass::unsectPair:
			for (EdgePal& edgePal : lastLimbEdge->pals) {
				OpEdge* test = edgePal.edge;
				if (test->disabled)
					continue;
				if (!test->hasPals())
					continue;
				EdgeMatch palMatch = edgePal.reversed ? match : !match;
				if (EdgeMatch::start == palMatch ? !test->startSeen : !test->endSeen)
					tryAdd(test, palMatch);
			}
			break;
		case LimbPass::coinPals:
			if (!contour.coincPalsBuilt)
				contour.buildCoincPals();
			for (OpEdge* test : contour.coincPals) {
				if (!test->startSeen)
					tryAdd(test, EdgeMatch::start);
				if (!test->endSeen)
					tryAdd(test, EdgeMatch::end);
			}
			break;
		case LimbPass::disabledCenterless:
			if (!contour.centerlessBuilt)
				contour.buildCenterless();
			for (OpEdge* test : contour.disabledCenterless) {
				if (!test->startSeen)
					tryAdd(test, EdgeMatch::start);
				if (!test->endSeen)
					tryAdd(test, EdgeMatch::end);
			}
			break;
		case LimbPass::disabledPals:
			if (!contour.palsBuilt && contour.hasPals) 
				contour.buildPals();
			for (OpEdge* test : contour.disabledPals) {
				// a pair of disabled pals may form an edge where they are of unequal lengths
				// so: add filler edge from test to test' pal (respecting pals 'reversed')
				if (!test->startSeen)
					tryAdd(test, EdgeMatch::start);
				if (!test->endSeen)
					tryAdd(test, EdgeMatch::end);
			}
			break;
		case LimbPass::unlinkedPal:
			for (EdgePal& pal : edge->pals) {
				std::vector<OpPoint> palPts = pal.edge->collectMatch(pal.reversed ? !match : match);
				if (ptsMatch(match, palPts))
					continue;
				pal.edge->setUnsetWhich();
				OpEdge* filler = tree->addFiller(pal.edge->segment, edge->whichCurvePt(match),
						pal.edge->whichCurvePt(pal.reversed ? !match : match), FillerGap::no);
				filler->curve.start = edge->curve.whichAlias(match);
				filler->curve.end = pal.edge->curve.whichAlias(pal.reversed ? !match : match);
				filler->setWhich(EdgeMatch::start);
				OpLimb* branch = tree->makeLimb();
				branch->set(filler, (OpLimb*) parent, match, nullptr, 0, nullptr);
			}
			break;
		case LimbPass::smallEdge:
			for (OpEdge* test : contour.smallEdges) {
				if (!test->startSeen)
					tryAdd(test, EdgeMatch::start);
				if (!test->endSeen)
					tryAdd(test, EdgeMatch::end);
			}
			break;
		case LimbPass::disabledBackwards:
	// check if tree's best distance is small enough with a user-provided multiplier
	// (motivated by loop193532 whose best distance 9.53e-7 exceeds threshold length 8.18e-7;
	//  an alternative would be to find the intersection of edges 198 and 208 -- somehow missed)
			if (contour.context->allowError(PathOpsV0Lib::ContextError::missing, &edge->curve.c))
				return;
			if (tree->gap(tree->bestDistance)) {
				tree->smallGap = true;
				return;
			}
			if (!contour.backwardsBuilt)
				contour.buildBackwards();
			for (OpEdge* test : contour.disabledBackwards) {
				if (!test->startSeen)
					tryAdd(test, EdgeMatch::start);
				if (!test->endSeen)
					tryAdd(test, EdgeMatch::end);
			}
			break;
		case LimbPass::disabled:
			if (!contour.disabledBuilt)
				contour.buildDisabled();
			for (OpEdge* test : contour.disabledEdges) {
				if (!test->startSeen)
					tryAdd(test, EdgeMatch::start);
				if (!test->endSeen)
					tryAdd(test, EdgeMatch::end);
			}
			break;
		default:
			OP_ASSERT(0);
	}
}

bool OpLimb::parentSeen(OpEdge* test) const {
	const OpLimb* ancestor = parent;
	while (ancestor) {
		if (ancestor->edge == test)
			return true;
		ancestor = ancestor->parent;
	}
	return false;
}

bool OpLimb::ptsMatch(EdgeMatch limbEnd, const std::vector<OpPoint>& testPts) const {
	OP_ASSERT(EdgeMatch::start == limbEnd || EdgeMatch::end == limbEnd);
	const std::vector<OpPoint>& pts = EdgeMatch::start == limbEnd ? firstPts : lastPts;
	for (OpPoint pt : pts) {
		for (OpPoint testPt : testPts) {
			if (pt == testPt)
				return true;
		}
	}
	return false;
}

bool OpLimb::ptsMatch(EdgeMatch limbEnd, const OpLimb* test, EdgeMatch testEnd) const {
	OP_ASSERT(EdgeMatch::start == testEnd || EdgeMatch::end == testEnd);
	const std::vector<OpPoint>& testPts = 
			EdgeMatch::start == testEnd ? test->firstPts : test->lastPts;
	return ptsMatch(limbEnd, testPts);
}

void OpLimb::set(OpEdge* test, OpLimb* p, EdgeMatch m, OpContour* contour,
		size_t index, OpEdge* otherEnd, const OpPointBounds* childBounds) {
	OP_DEBUG_DUMP_CODE(id = tree->context->nextID());
	edge = test;
	firstPts = edge->collectMatch(m);
	parent = p;
	linkedContour = contour;
	linkedIndex = (uint32_t) index;
	match = m;
	treePass = tree->limbPass;
	looped = false;
	deadEnd = false;
	if (LimbPass::linked != treePass && LimbPass::miswound != treePass) {
		lastLimbEdge = edge;
		lastMatch = !match;
	} else if (EdgeMatch::start == match) {
		lastLimbEdge = edge->lastEdge;
		lastMatch = EdgeMatch::end;
	} else {
		lastLimbEdge = otherEnd;
		lastMatch = EdgeMatch::start;
	}
	lastPts = lastLimbEdge->collectMatch(lastMatch, &lastT);
	looped = tree->firstMatch(lastPts);
	closeDistance = tree->firstDistance(lastPts[0]);
	if (childBounds) {
		limbBounds = *childBounds;
		const OpLimb* testParent = p;
		while (testParent) {
			if (testParent->ptsMatch(EdgeMatch::start, this, EdgeMatch::end)) {
				deadEnd = true;
				break;
			}
			testParent = testParent->parent;
		}
	}
    // !!! start here
    // cut test uses winding frame-fill instead of perimeter to choose best child
    // cut test needs to be given each possibility and then have a way to provide
    // selection akin to best-perimeter once all possible choices are known...
    if (looped) {
        PathOpsV0Lib::CurveOutput outFunc = tree->context->contextCallbacks.bestLoopFuncPtr;
        if (outFunc) {
        	bool kept = false;
            const OpLimb* limb = this;
            bool ptFirst = true;
	        do {
                OpEdge* outEdge = limb->edge;
                if (EdgeMatch::end == limb->match) {
                    outEdge = outEdge->lastEdge;
                    do {
	                    OpCurve copy(outEdge->curve.c, Rotated::no);
                        copy.reverse();
                        OpEdge* next = outEdge->priorEdge;
                        kept |= PathOpsV0Lib::WindKeep::Discard != copy.bestLoop(outFunc, 
								outEdge->winding.w, ptFirst, !!next  OP_DEBUG_PARAMS(outEdge->id));
                        ptFirst = false;
                        outEdge = next;
                    } while (outEdge && !kept);
                } else {
                    do {
                        OpEdge* next = outEdge->nextEdge;
                        kept |= PathOpsV0Lib::WindKeep::Discard != outEdge->curve.bestLoop(outFunc,
                                outEdge->winding.w, ptFirst, !!next  OP_DEBUG_PARAMS(outEdge->id));
                        outEdge = next;
                    } while (outEdge && !kept);
                }
            } while ((limb = limb->parent) && !kept);
            if (kept)
                tree->bestLimb = this;
        } else if (tree->bestPerimeter > limbBounds.perimeter()) {
		    tree->bestPerimeter = limbBounds.perimeter();
		    tree->bestLimb = this;
        }
	}
	if (tree->bestDistance > closeDistance) {
		tree->bestDistance = closeDistance;
		tree->bestGapLimb = this;
	}
	OP_DEBUG_DUMP_CODE(if (p) p->debugBranches.push_back(this));
}

OpLimb* OpLimb::tryAdd(OpEdge* test, EdgeMatch m, OpContour* limbContour, size_t limbIndex, 
			OpEdge* otherEnd) {
	OP_ASSERT(!test->disabled || test->hasPals() || !test->isSortable()
			|| LimbPass::coinPals <= tree->limbPass);
	OP_ASSERT(!test->hasLinkTo(m) || !test->isSortable() || test->disabled 
			|| test->hasPals() || test->smallTRange);
	// !!! future optimization : keep all possible end points with edge, or pass limb instead of
	std::vector<OpPoint> testStarts = test->collectMatch(m);
	if (!ptsMatch(EdgeMatch::end, testStarts))
		return nullptr;
	if (edge == test)
		return nullptr;
	if (test->hasPals() && tree->preferSibling(this, test))
		return nullptr;
	OP_ASSERT(lastLimbEdge);
	if (LimbPass::miswound == tree->limbPass && lastLimbEdge == test)
		return nullptr;
	if ((test->startSeen || test->endSeen) && parentSeen(test))
		return nullptr;
	// compare test wind zero against their parent's last edge wind zero
	// OP_ASSERT(!test->isPal(last) || LimbPass::linked != limbPass);  // breaks pentrek10
	// Edge direction and winding are tricky (see description at wind zero declaration.)
	// For first edge (and its last) in storage: if which end is 'end', its wind zero is reversed.
	// 'Test' may need to be reversed to connect, and 'm' may be either end. Wind zero in both cases
	// is computed for the unreversed orientation.
	if (WindZero::unset != lastLimbEdge->windZero && WindZero::unset != test->windZero
			&& (LimbPass::linked == tree->limbPass || LimbPass::miswound == tree->limbPass)
			&& lastLimbEdge->pals.empty()
			&& Unsortable::filler != lastLimbEdge->unsortable) {
		WindZero zeroSide = test->windZero;
		// if last which end is end, flip last's wind zero (for comparsion, flip zero side);
		// if pass is linked: if test m is end, flip zero side; if test which is end, flip zero side
		// if pass is miswound, flip the logic
		if (((lastLimbEdge->which() == match) != (test->which() == m)) 
				== (LimbPass::linked == tree->limbPass))
			zeroSide = !zeroSide;
		if (lastLimbEdge->windZero != zeroSide)
			return nullptr;
	}
	OpPointBounds childBounds = test->lastEdge ? test->linkBounds : 
			otherEnd ? otherEnd->linkBounds : test->bounds();
//	start here;
	// Look for parent with multiple children. See if a sibling ends at the same point as test.
	// Select the shorter? option with the smaller bounds?
	// !!! check if adding this child to parent makes the bounds bigger than some other child?
	//     are we checking to see if both children ended up at the same point?
	//     
	if (parent && LimbPass::disjoint != treePass)  // if not trunk
		childBounds.add(limbBounds);
	if (LimbPass::linked == tree->limbPass)
		(EdgeMatch::start == m ? test->startSeen : test->endSeen) = true;
	// if this edge added to limb bounds makes perimeter larger than best, skip
	// !!! are their cases where smallest perimeter is not the best test?
	// first, multiple edges with the same start and end point may share perimeters
	// is larger perimeter more desirable if it avoids enclosing another contour?
	// should this use the ray edges to see if test is closer to test edge?
	// note that best may not have ray to edge; e.g., outline of 'O' (edge contains inner contour)
	if (childBounds.perimeter() > tree->bestPerimeter)
		return nullptr;
#if 0  // breaks quad test (unknown #) next time, record test it fixes...
	if (LimbPass::disabledCenterless == limbPass) 
		test->setWhich(m);
#endif
	if (LimbPass::unlinked == tree->limbPass 
			|| LimbPass::coinPals == tree->limbPass 
			|| LimbPass::disabledCenterless == tree->limbPass 
			|| LimbPass::disabledPals == tree->limbPass
			|| LimbPass::smallEdge == tree->limbPass
			|| LimbPass::disabledBackwards == tree->limbPass 
			)
		(EdgeMatch::start == m ? test->startSeen : test->endSeen) = true;
	OpLimb* branch = tree->makeLimb();
	branch->set(test, this, m, limbContour, limbIndex, otherEnd, &childBounds);
	return branch;
}

#if OP_DEBUG_DUMP
OpTree::OpTree(DumpSerialization , OpContext* c)
    : context(c)
	, bestGapLimb(nullptr)
	, bestLimb(nullptr)
	, bestDistance(OpNaN)
	, bestPerimeter(OpNaN)
	, maxLimbs(0) 
	, totalUsed(0) 
    , id(0)
	, limbPass(LimbPass::uninitialized)
//	, disabled(false)
	, smallGap(false) {
}
#endif

OpTree::OpTree(OpEdge* edge) 
	: context(edge->segment->contour->context)
	, trunk(nullptr)
	, bestGapLimb(nullptr)
	, bestLimb(nullptr)
	, bestDistance(OpInfinity)
	, bestPerimeter(OpInfinity)
	, totalUsed(0) 
	, limbPass(LimbPass::linked)
//	, disabled(false)
	, smallGap(false) {
	passIndex.push_back(0);
	OP_ASSERT((size_t) limbPass == passIndex.size());
	passIndex.push_back(0);
	id = context->nextID();
	maxLimbs = context->contextCallbacks.maxLimbsFuncPtr ?
			context->contextCallbacks.maxLimbsFuncPtr((PathOpsV0Lib::Context*) context) : 100000;
	OP_DEBUG_CODE(context->debugTree = this);
	OP_ASSERT(edge->inLinkups);
}

OpEdge* OpTree::addFiller(OpSegment* seg, OpPoint pt1, OpPoint pt2, FillerGap fillGap) {
	if (FillerGap::yes == fillGap) {
		float fillerLength = (pt1 - pt2).length();
		if (!gap(fillerLength)) {
			OP_DEBUG_CODE(OpDebugOut("\n" + context->debugData.testname + "\n"));
			OP_DEBUG_DUMP_CODE(dump());
			context->setError(PathOpsV0Lib::ContextError::gap  OP_DEBUG_PARAMS(id));
			// !!! dump file here?
		}
	}
	OpEdge* result = context->addFiller(pt1, pt2, seg);
	return result;
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

float OpTree::firstDistance(OpPoint pt) const {
	OP_ASSERT(trunk);
	float minDistanceSq = OpInfinity;
	for (OpPoint firstPt : trunk->firstPts) {
		float distanceSq = (firstPt - pt).lengthSquared();
		minDistanceSq = std::min(minDistanceSq, distanceSq);
	}
	return sqrtf(minDistanceSq);
}

bool OpTree::firstMatch(const std::vector<OpPoint>& pts) const {
	OP_ASSERT(trunk);
	for (OpPoint firstPt : trunk->firstPts) {
		for (OpPoint pt : pts) {
			if (firstPt == pt)
				return true;
		}
	}
	return false;
}

bool OpTree::gap(float distance) const {
	PathOpsV0Lib::ContextValue gapFuncPtr = context->contextCallbacks.maxGapFuncPtr;
	float gapFactor = gapFuncPtr ? (*gapFuncPtr)((ContextPtr) context) : 4.f;
	return distance <= context->thresholdLength * gapFactor;
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
		case LimbPass::coinPals:
			if (contour.coincPalsBuilt)
				for (OpEdge* test : contour.coincPals)
					contour.unlink(test);
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
		case LimbPass::unlinkedPal:
			break;
		case LimbPass::miswound:
			break;
		case LimbPass::disjoint:
			break;
		case LimbPass::smallEdge:
			break;
		case LimbPass::disabledBackwards:
			if (contour.backwardsBuilt)
				for (OpEdge* test : contour.disabledBackwards)
					contour.unlink(test);
			break;
		case LimbPass::disabled:
			if (contour.centerlessBuilt)
				for (OpEdge* test : contour.disabledEdges)
					contour.unlink(test);
			break;
		default:
			OP_ASSERT(0);
	}
}

// join best limb to edge start, then parent to best limb, until lastEdge is found
bool OpTree::join(OpJoiner& join) {
	std::vector<OpEdge*> linkupsErasures;
	OP_DEBUG_DUMP_CODE(context->debugErasures = &linkupsErasures);
	const OpLimb* bestL = bestLimb;
	OpEdge* best = bestL->edge;
	if (EdgeMatch::end == bestL->match) {
        OP_ASSERT(EdgeMatch::none != best->which()  // !!! assert may be unnecessary; make sure disabled is correct choice
                || (best->disabled && (LimbPass::disabledBackwards == bestL->treePass
				|| LimbPass::disabledCenterless == bestL->treePass
				|| LimbPass::coinPals == bestL->treePass
				|| LimbPass::disabled == bestL->treePass))
				|| (best->smallTRange && LimbPass::smallEdge == bestL->treePass)
			);
        EdgeMatch which = EdgeMatch::none != best->which() ? best->which() : bestL->match;
		(void) best->setLastLink(!which); // make suitable for linking to a chain
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
			EdgeMatch priorEnd = EdgeMatch::none == prior->which() ? EdgeMatch::end : !prior->which();
			(void) prior->setLastLink(priorEnd);  // make suitable for linking to a chain
			prior = prior->advanceToEnd(EdgeMatch::start);
		} else
			(void) prior->setLastLink(prior->which());
		OpEdge* last = prior->lastEdge;
//		OP_ASSERT(bestL->ptsMatch(EdgeMatch::start, lastLimb, EdgeMatch::end));
//		OP_ASSERT(best->whichSect().pt == last->whichSect(EdgeMatch::end).pt || best->disabled);
		best->setPriorEdge(last);
		last->setNextEdge(best);
		prior->setLast(best, best->lastEdge, InOutput::yes);
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
	// close path unless caller allows disjoint results (by allowing context error missing) 
	bool allowGaps = context->allowError(PathOpsV0Lib::ContextError::missing, &join.edge->curve.c);
	EdgeOutput edgeOutput(context, join.edge, !allowGaps);
	OP_DEBUG_DUMP_CODE(context->dumpFile("treeJoined"));
	for (OpEdge* edge : linkupsErasures) {
        if (!edge->inOutput) {
            edge->updateLastEdge();
            continue;
        }
		if (edge != join.edge && edge->lastEdge)
            edge->clearLast(/* InOutput::yes */);
#if OP_DEBUG_VALIDATE
		OP_ASSERT(edge->debugScheduledForErasure);
		edge->debugScheduledForErasure = false;
#endif
		if (edge->linkHead) {
            edge->segment->contour->removeLink(edge);
			OP_ASSERT(!edge->linkHead);
		}
	}
	OP_DEBUG_DUMP_CODE(context->debugErasures = nullptr);
	OP_DEBUG_VALIDATE_CODE(join.debugValidate());
	context->resetLimbs();
	return true;
}

// used to walk tree in breadth order
OpLimb& OpTree::nthLimb(int index) {
	return context->nthLimb(index);
}

OpLimb* OpTree::makeLimb() {
	++totalUsed;
	return context->allocateLimb(this);
}

// Mark edges' 'seen' as unset in this tree. Later, mark additional contours as they are linked
bool OpTree::makeTrunk(OpEdge* edge) {
	OpContour* edgeContour = edge->segment->contour;
	for (OpContour* member : edgeContour->members()) {
		member->setSeen(id);
	}
	context->resetLimbs();
	trunk = makeLimb();
	OP_ASSERT(edgeContour->linkups.l.back() == edge);
	trunk->set(edge, nullptr, EdgeMatch::start, edgeContour, edgeContour->linkups.l.size() - 1, 
			edge);
	edge->startSeen = true;
	edge->lastEdge->endSeen = true;
	if (bestLimb)
		return true;
	// !!! can I know that edge never has prior, and is never loop?
	for (;;) {
		for (OpContour* member : edgeContour->members()) {
			initialize(*member);
		}
		int index = passIndex[(int) limbPass];
		int baseUsed = totalUsed;
		// !!! old: (LimbPass::linked == limbPass ? totalUsed : passIndex[(int) limbPass - 1])
		while (index < totalUsed) {
			OpLimb& limb = nthLimb(index);
			OpEdge* endEdge = limb.edge;
			if (endEdge->lastEdge)
				endEdge = endEdge->lastEdge;
			else if (endEdge->priorEdge)
				endEdge = endEdge->advanceToEnd(EdgeMatch::start);
			for (OpContour* member : endEdge->segment->contour->members()) {
				if (member->treeID != id)
					member->setSeen(id);
				limb.addEach(*member);
			}
			if (totalUsed > maxLimbs) {
				setError(edge, index);
				return false;
			}
			++index;
		}
		if (bestLimb)
			break;
		passIndex[(int) limbPass] = totalUsed;
		if (LimbPass::linked != limbPass && totalUsed > baseUsed)
			limbPass = LimbPass::linked;
		else {
			if (LimbPass::disabled <= limbPass)
				return false;
			if ((int) ++limbPass >= (int) passIndex.size())
				passIndex.push_back(0);
		}
	}
	return true;
}

bool OpTree::exhausted() {
	// if ineligible edges make up some percentage of the tree, discard it
	const OpLimb* test = bestGapLimb;
	float enabledLength = 0;
	float disabledLength = 0;
	do {
		(test->edge->disabled ? disabledLength : enabledLength) += 
				test->edge->curve.callerBounds().perimeter();
		test = test->parent;
	} while (test);
	PathOpsV0Lib::ContextValue fun = context->contextCallbacks.enabledRatioFuncPtr;
	float enabledRatio = fun ? (*fun)((ContextPtr) context) : 1;  // !!! have no idea what this should be
	if (disabledLength && enabledLength / disabledLength >= enabledRatio)
		return false;
	test = bestGapLimb;
	do {
		if (test->edge->linkHead)
			test->edge->segment->contour->removeLink(test->edge);
		test = test->parent;
	} while (test);
	return true;
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

void OpTree::setError(OpEdge* edge, int index) {
#if OP_DEBUG_DUMP
	std::string s;
	for (int i = index; i < totalUsed; ++i) {
		const OpLimb& test = nthLimb(i);
		s += STR(i) + " " + test.debugDump(defaultLevel, defaultBase) + "\n";
#if 0
		std::string sa;
		const OpLimb* a = test.parent ? test.parent->parent : nullptr;
		if (!a)
			continue;
		while (a) {
			sa += "[l:" + STR((int) a->linkedIndex) + " e:" + STR(a->edge->id) + "] ";
			a = a->parent;
		}
		sa.pop_back();
		s += " ancestors: " + sa + "\n";
#endif
	}
	OpDebugFormat(s);
	context->debugData.defeatDumps = false;
	context->dumpFile("treeError");
	OpNop();
#endif
	context->setError(PathOpsV0Lib::ContextError::tree  
			OP_DEBUG_PARAMS(edge->id));

}

#if 0
// if disabled pals limb pass is reached, check to see if tree can be closed by connecting unsects
OpLimb* OpTree::unsectableLoop() const {
	OpLimb& trunk = context->nthLimb(0);
	OpSegment* trunkSeg = trunk.edge->segment;
	OpPoint trunkPt = trunk.edge->whichSect().pt;
	std::vector<OpIntersection*> startUnsects = trunkSeg->sects.unsectables(trunkPt);
	if (startUnsects.empty())
		return nullptr;
	OpLimb* smallest = nullptr;  // return smallest gap
	float bestDist = OpInfinity;
	for (int index = 0; index < totalUsed; ++index) {
		OpLimb& test = context->nthLimb(index);
		OpSegment* testSeg = test.lastLimbEdge->segment;
		if (!testSeg)	// if limb is filler, edge won't have a parent segment
			continue;
		std::vector<OpIntersection*> testUnsects = testSeg->sects.unsectables(test.lastPts[0]);
		for (OpIntersection* testSect : testUnsects) {
			float testDist = (testSect->ptT.pt - trunkPt).length();
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
#endif

// caller (in contours) has allocated storage already
OpLimb* OpLimbStorage::allocate(OpTree* tree) {
	OP_ASSERT(used < (int) ARRAY_COUNT(storage));
	OpLimb& result = storage[used++];
//	OP_DEBUG_CODE(result = OpLimb());  // !!! may be needed but comment why
	result.tree = tree;
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

#if OP_DEBUG_DUMP
OpJoiner::OpJoiner(DumpSerialization , OpContext* c) 
    : context(c)
    , linkMatch(EdgeMatch::none)
	, linkPass(LinkPass::none)
	, edge(nullptr)
	, lastLink(nullptr) 
    OP_DEBUG_PARAMS(debugRecursiveDepth(0)) {
}

#endif

OpJoiner::OpJoiner(OpContext& contours)
	: context(&contours)
    , linkMatch(EdgeMatch::none)
	, linkPass(LinkPass::none)
	, edge(nullptr)
	, lastLink(nullptr)
	OP_DEBUG_PARAMS(debugRecursiveDepth(0)) {
    for (auto contour : contours.contours) {
		for (auto& segment : contour->segments) {
			for (auto& e : segment.edgeList) {
				if (e.smallTRange)
					contour->addSmallEdge(&e);
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

#if OP_DEBUG
OpJoiner::~OpJoiner() {
    context->debugJoiner = nullptr;
}
#endif

// start here;
// one thing broken with this overall approach is that very small loops get priority over joining
// large incomplete segments. See if it can be restructured to finish big things first, then use 
// the scale of the big things to see if the small remaining things can be ignored
// first, figure out why the current test fails

bool OpJoiner::linkRemaining(OpContour* contour) {
	LinkUps& linkups = contour->linkups;
    OP_DEBUG_DUMP_CODE(context->dumpFile(__func__));
	linkPass = LinkPass::remaining;
	// match links may add or remove from link ups. Iterate as long as link ups is not empty
	for (auto e : linkups.l) {
		e->setLinkBounds();
	}
	OP_DEBUG_CODE(int debugLoopCounter = 0);
	while (linkups.l.size()) {
		// sort to process largest of left/top first
		// !!! could optimize to avoid search, but for now, this is the simplest
		linkups.sort(context);
		edge = linkups.l.back();
		// if largest is small, and all edges it links to are small, don't link it
		auto edgeIsSmall = [this]() {
			if (!edge->smallTRange) 
				return false;
			OpEdge* nextEdge = edge;
			while ((nextEdge = nextEdge->nextEdge)) {
				if (!nextEdge->smallTRange)
					return false;
			}
			return true;
		};
		if (edgeIsSmall())
			return true;
		OP_DEBUG_VALIDATE_CODE(debugValidate());
        if (!edge->lastEdge) {
            OP_ASSERT(!edge->priorEdge);  // !!! leftover tail after loop was output?
            OP_ASSERT(!edge->nextEdge);
            OP_ASSERT(!edge->inOutput);
            edge->setLast(edge, edge, InOutput::no);
        }
		if (!matchLinks(contour, true))
			return false;
		// Match links may have generated new filler edges. Check if remaining edge in linkups
		// is nearly the same as a generated edge; if so, mark it unlinked so joiner can exit 
		// without using it.
		// !!! Not sure how well this scales. Start out only allowing unlinked linkable edges.
		if (context->fillerStorage) {
			std::vector<OpEdge*> erasures;
			std::vector<OpEdge*> singleLinks;
			for (OpContour* member : contour->members()) {
				for (size_t index = 0; index < member->linkups.l.size(); ++index) {
					OpEdge* linkup = member->linkups.l[index];
					OP_ASSERT(!linkup->priorEdge);
					if (linkup->nextEdge)
						continue;
					singleLinks.push_back(linkup);
				}
			}
			for (;;) {  // assume there are possibly many fillers, but few unlinked linkables
				OpEdge* fillerEdge = context->fillerStorage->edgeIndex(fillerIndex);
				if (!fillerEdge)
					break;
				OpRect fillerBounds = fillerEdge->curve.callerBounds();
				for (OpEdge* linkup : singleLinks) {
					OpRect linkupBounds = linkup->curve.callerBounds();
					if (!fillerBounds.nearlyContains(linkupBounds, context->threshold))
						continue;
					if (!linkupBounds.nearlyContains(fillerBounds, context->threshold))
						continue;
					erasures.push_back(linkup);  // filler and linkup are nearly the same
				}
				++fillerIndex;
			}
			if (!erasures.empty()) {
				contour->eraseLinks(erasures);
				std::vector<OpEdge*>& unsortables = contour->unsortables;
				unsortables.insert(unsortables.end(), erasures.begin(), erasures.end());
			}
		}
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
//		OP_ASSERT(!opp->disabled);  // !!! may be disabled if tiny -- segment points no longer moved
        if (opp->disabled)
            break;
		OpEdge* prevEdge = &opp->edgeList.back();
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
//		OP_ASSERT(!opp->disabled);  // !!! may be disabled if tiny -- segment points no longer moved
        if (opp->disabled)
            break;
		OpEdge* nextEdge = &opp->edgeList.front();
		if (!nextEdge->isSimple())
			break;
		if (!opp->simpleStart(nextEdge) || nextSect->opp != opp->sects.i.front())
			break;
		if (nextEdge->priorEdge) {
			OP_ASSERT(edge->nextEdge == nextEdge);
			OpEdge* last = first->updateLastEdge();
			OP_ASSERT(!last->nextEdge);
			last->setNextEdge(first);
			OP_ASSERT(!first->priorEdge);
			first->setPriorEdge(last);
			first->outputLinkedList();
			return false;
		}
		OP_ASSERT(!nextEdge->priorEdge);
		nextEdge->setPriorEdge(edge);
		nextEdge->setWhich(EdgeMatch::start);
		OP_ASSERT(!edge->nextEdge);
		edge->setNextEdge(nextEdge);
		if (nextEdge->nextEdge && edge->priorEdge) {
			first->outputLinkedList();
			return false;
		}
		edge = nextEdge;
		OP_ASSERT(!edge->segment->disabled);
		if (!edge->isSimple())
			break;
	} while (edge->segment->simpleEnd(edge));
	return true;
}

void OpJoiner::linkUnambiguous(OpContour* contour, LinkPass lp) {
#if OP_DEBUG_DUMP
	if (context->debugData.dumpUnambiguous)
    	context->dumpFile(__func__);
#endif
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	// match up edges that have only a single possible prior or next link, and add them to new list
	linkPass = lp;
	std::vector<OpEdge*>& edges = LinkPass::normal == lp ? contour->byArea : contour->unsectByArea;
	for (auto& e : edges) {
		if (e->disabled)
			continue;   // likely marked as part of a loop below
		if (e->smallTRange)
			continue;
		if (!e->isActive())  // check if already saved in linkups
			continue;
		if (e->inOutput)  // !!! added for testQuads3130081 ; unsure why it wasn't a condition all along
			continue;
		if (!e->priorEdge) {
			if (LinkPass::unsectable == lp)
				e->setWhich(EdgeMatch::start);
			linkMatch = EdgeMatch::start;
			OP_DEBUG_CODE(debugRecursiveDepth = 0);
			contour->linkUp(this, e);
		}
		{
			OP_DEBUG_VALIDATE_CODE(debugValidate());
			linkMatch = EdgeMatch::end;
			OP_DEBUG_CODE(debugRecursiveDepth = 0);
			if (!e->inOutput) {
				OpEdge* last = e->updateLastEdge();
				if (!last->inOutput)
					last->segment->contour->linkUp(this, last);
			}
			OP_DEBUG_VALIDATE_CODE(debugValidate());
		}
	}
}

// at this point all singly linked edges have been found
// every active set of links at this point must form a loop
// the only distance that matters is zero. We should never have unexplained gaps (ideal, not real)
bool OpJoiner::matchLinks(OpContour* contour, bool popLast) {
	OP_DEBUG_VALIDATE_CODE(debugValidate());
	OP_ASSERT(!edge->priorEdge);
	lastLink = edge->lastEdge;
	OP_ASSERT(lastLink);
	OP_ASSERT(!lastLink->nextEdge);
	OP_ASSERT(EdgeMatch::start == lastLink->which() || EdgeMatch::end == lastLink->which());
//	found.clear();
//	matchPt = lastLink->whichSect(EdgeMatch::end).pt;
	OpTree tree(edge);
	bool treeFailed = !tree.makeTrunk(edge) && tree.exhausted();
	OP_DEBUG_DUMP_CODE(context->dumpFile("treeMade"));
	if (treeFailed)
		return false;
	if (PathOpsV0Lib::ContextError::none != context->error)
		return false;
	// adding gap edge in unsect pair case
#if OP_TEST_RASTER
    OpPoint firstPt;
    OpPoint lastPt;
#endif
	if (!tree.bestLimb) {
		OpLimb* gap = tree.bestGapLimb;
		OP_ASSERT(gap);
		if (context->allowError(PathOpsV0Lib::ContextError::missing, &edge->curve.c)) {
			OpPtT startI = edge->whichSect();
			OpPtT gapEnd = gap->lastLimbEdge->whichSect(!gap->match);
			bool usectLink = unsectableLink(contour, startI.pt, gapEnd.pt);
			OpEdge* filler = tree.addFiller(gap->lastLimbEdge->segment, gapEnd.pt, startI.pt, 
					usectLink ? FillerGap::no : FillerGap::yes);
			OpLimb* branch = tree.makeLimb();
			branch->set(filler, gap, EdgeMatch::start, nullptr, 0, nullptr);
			branch->treePass = LimbPass::disjoint;
			tree.bestLimb = branch;
		} else {
			tree.bestLimb = gap;
		}
	} else {
#if OP_TEST_RASTER
        firstPt = edge->whichCallerPt();
        OpEdge* lastEdge = tree.bestLimb->lastLimbEdge;
        lastPt = EdgeMatch::end == (EdgeMatch::none == lastEdge->which()
                ? tree.bestLimb->match : lastEdge->which(tree.bestLimb->match)) 
                ? lastEdge->curve.callerFirst() : lastEdge->curve.callerLast();
#endif
    }
	bool result = tree.join(*this);
#if OP_TEST_RASTER
    if (firstPt != lastPt)
        context->addRasterFiller(firstPt, lastPt, edge->segment);
#endif
    return result;
}

// sort by size so that tiny edges with poor winding don't run the show
void OpJoiner::sort() {
	for (auto contour : context->contours) {
		contour->joinSort();
	}
}

// scan links to see if their ends are the same as other edges unsectable ends
bool OpJoiner::unsectableLink(OpContour* contour, OpPoint start, OpPoint end) {
	for (const std::vector<OpEdge*>& list : { contour->disabledPals, contour->unsortables } ) {
		for (OpEdge* test : list) {
			OpPoint s = test->startPt();
			OpPoint e = test->endPt();
			bool sMatch = s == start || e == start;
			bool eMatch = s == end || e == end;
			if (!sMatch && !eMatch)
				continue;
			for (EdgePal& testPal : test->pals) {
				OpPoint ps = testPal.edge->startPt();
				OpPoint pe = testPal.edge->endPt();
				if (sMatch && (ps == end || pe == end))
					return true;
				if (eMatch && (ps == start || pe == start))
					return true;
			}
		}
	}
	return false;
}

// sort by size to process largest of left (tail) first
// sort should consider all edges in link
void LinkUps::sort(OpContext* context) {
	float leftMost = OpInfinity;
	for (auto& linkList : l) {
		OP_ASSERT(linkList->linkBounds.isFinite());
		leftMost = std::min(linkList->linkBounds.left, leftMost);
	}
	PathOpsV0Lib::ContextValue scaleFuncPtr = context->contextCallbacks.linkupScaleFuncPtr;
	float scale = scaleFuncPtr ? (*scaleFuncPtr)((ContextPtr) context) : 1024.f;  // !!! wild guess : testCubics3465653 requires 400K or smaller
	std::sort(l.begin(), l.end(), [leftMost, scale](const auto& s1, const auto& s2) {
		// if much smaller, ignore leftmost
		float s1perimeter = s1->linkBounds.perimeter();
		float s2perimeter = s2->linkBounds.perimeter();
		if (s1perimeter * scale < s2perimeter)
			return true;
		if (s2perimeter * scale < s1perimeter)
			return false;
        bool s1Left = s1->linkBounds.left == leftMost;
        bool s2Left = s2->linkBounds.left == leftMost;
		return s1Left < s2Left || (s1Left == s2Left && s1perimeter < s2perimeter); 
	} );
}
