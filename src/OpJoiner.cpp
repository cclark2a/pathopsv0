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
	for (const std::vector<OpEdge*>& edges : { contour.unsectByArea, contour.unsortables } ) {
		for (OpEdge* test : edges) {
			if (test->inLinkups)
				continue;
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
		OpLimb* dpStart = tryAdd(tree, test, EdgeMatch::start, LimbPass::disabledPals);
		// a pair of disabled pals may form an edge where they are of unequal lengths
		// so: add filler edge from test to test' pal (respecting pals 'reversed')
		if (dpStart)
			tryPal(tree, this, dpStart, EdgeMatch::start);
		OpLimb* dpEnd = tryAdd(tree, test, EdgeMatch::end, LimbPass::disabledPals);
		if (dpEnd)
			tryPal(tree, this, dpEnd, EdgeMatch::end);
	}
	if (LimbPass::disabledPals == pass)
		return;
	if (LimbPass::miswound == pass)
		return; 
	// iterate through edge pals looking for gap that connects lastPt via sect opp
	// unsectable edges do not necessarily point to other unsectable through pals or upairs
	for (EdgePal& edgePal : lastLimbEdge->pals) {
		if (edgePal.edge->disabled)
			continue;
		if (!edgePal.edge->hasPals())
			continue;
		tryAdd(tree, edgePal.edge, edgePal.reversed ? match : !match, LimbPass::unsectPair);
#if 0
		OpEdge* test = edgePal.edge + (EdgeMatch::start == match ? 1 : -1);
		OpSegment* palSeg = edgePal.edge->segment;
		if (&palSeg->edges.front() <= test && test <= &palSeg->edges.back())
			tryAdd(tree, test, edgePal.reversed ? !match : match, LimbPass::unsectPair);
#endif
	}
	if (LimbPass::unsectPair == pass)
		return;
	if (LimbPass::disjoint == pass)
		return;
	if (LimbPass::unlinkedPal == pass)
		return;
	for (OpEdge* test : contour.smallEdges) {
		tryAdd(tree, test, EdgeMatch::start, LimbPass::smallEdge);
		tryAdd(tree, test, EdgeMatch::end, LimbPass::smallEdge);
	}
	if (LimbPass::smallEdge == pass)
		return;
//	if (LimbPass::alternateEnd == pass)
//		return;
	if (contour.context->allowError(PathOpsV0Lib::ContextError::missing, &edge->curve.c))
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

void OpLimb::set(OpTree& tree, OpEdge* test, OpLimb* p, EdgeMatch m, LimbPass l, OpContour* contour,
		size_t index, OpEdge* otherEnd, const OpPointBounds* childBounds) {
	OP_DEBUG_DUMP_CODE(id = tree.context->nextID());
	edge = test;
	firstPts = edge->collectMatch(m);
	parent = p;
	linkedContour = contour;
	linkedIndex = (uint32_t) index;
	match = m;
	treePass = l;
	resetPass = true;
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
	looped = tree.firstMatch(lastPts);
	closeDistance = tree.firstDistance(lastPts[0]);
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
        PathOpsV0Lib::CurveOutput outFunc = tree.context->contextCallbacks.bestLoopFuncPtr;
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
                tree.bestLimb = this;
        } else if (tree.bestPerimeter > limbBounds.perimeter()) {
		    tree.bestPerimeter = limbBounds.perimeter();
		    tree.bestLimb = this;
        }
	}
	if (tree.bestDistance > closeDistance) {
		tree.bestDistance = closeDistance;
		tree.bestGapLimb = this;
	}
	OP_DEBUG_DUMP_CODE(if (p) p->debugBranches.push_back(this));
	OP_DEBUG_IMAGE_CODE_OLD(tree.debugLimbEdges(edge));
}

OpLimb* OpLimb::tryAdd(OpTree& tree, OpEdge* test, EdgeMatch m, LimbPass limbPass, 
			OpContour* limbContour, size_t limbIndex, OpEdge* otherEnd) {
	OP_ASSERT(!test->disabled || test->hasPals() || !test->isSortable() 
			|| LimbPass::disabledCenterless <= limbPass);
	OP_ASSERT(!test->hasLinkTo(m) || !test->isSortable() || test->disabled 
			|| test->hasPals() || test->smallTRange);
	// !!! future optimization : keep all possible end points with edge, or pass limb instead of
	std::vector<OpPoint> testStarts = test->collectMatch(m);
	if (!ptsMatch(EdgeMatch::end, testStarts))
		return nullptr;
	if (edge == test)
		return nullptr;
	if (test->hasPals() && tree.preferSibling(this, test))
		return nullptr;
	OP_ASSERT(lastLimbEdge);
	if (LimbPass::miswound == limbPass && lastLimbEdge == test)
		return nullptr;
	if (LimbPass::unsectPair != limbPass && tree.contains(this, test))
		return nullptr;
	std::vector<OpPoint> testEnds = test->collectMatch(!m);
	bool loopedToFirstPoint = tree.firstMatch(testEnds);
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
#if OP_EDGE_PAL_MANY
			&& !lastLimbEdge->palMany.isSet() 
#else
			&& lastLimbEdge->pals.empty()
#endif
			&& Unsortable::filler != lastLimbEdge->unsortable) {
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
			otherEnd ? otherEnd->linkBounds : test->bounds();
//	start here;
	// Look for parent with multiple children. See if a sibling ends at the same point as test.
	// Select the shorter? option with the smaller bounds?
	// !!! check if adding this child to parent makes the bounds bigger than some other child?
	//     are we checking to see if both children ended up at the same point?
	//     
	if (parent && LimbPass::disjoint != treePass)  // if not trunk
		childBounds.add(limbBounds);
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
	if (tree.containsParent(this, test, m))
		return nullptr;
#if 0  // breaks quad test (unknown #) next time, record test it fixes...
	if (LimbPass::disabledCenterless == limbPass) 
		test->setWhich(m);
#endif
	OpLimb* branch = tree.makeLimb();
	branch->set(tree, test, newParent, m, limbPass, limbContour, limbIndex, otherEnd, &childBounds);
	return branch;
}

void OpLimb::tryPal(OpTree& tree, OpLimb* parent, OpLimb* limb, EdgeMatch m) {
	for (EdgePal& edgePal : limb->edge->pals) {
		std::vector<OpPoint> palPts = edgePal.edge->collectMatch(edgePal.reversed ? !m : m);
		if (limb->ptsMatch(m, palPts))
			continue;
		// if pal point is not close to test point, add filler between the two
		OpEdge* filler = tree.addFiller(edgePal.edge->segment, limb->edge->whichCurvePt(m),
				edgePal.edge->whichCurvePt(edgePal.reversed ? !m : m), FillerGap::no);
		filler->curve.start = limb->edge->curve.whichAlias(m);
		filler->curve.end = edgePal.edge->curve.whichAlias(edgePal.reversed ? !m : m);
		filler->setWhich(EdgeMatch::start);
		OpLimb* branch = tree.makeLimb();
		branch->set(tree, filler, parent, m, LimbPass::disjoint, nullptr, 0, nullptr);
	}
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
	, disabled(false)
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
	, disabled(false)
	, smallGap(false) {
	id = context->nextID();
	maxLimbs = context->contextCallbacks.maxLimbsFuncPtr ?
			context->contextCallbacks.maxLimbsFuncPtr((PathOpsV0Lib::Context*) context) : 1000;
	OP_DEBUG_CODE(context->debugTree = this);
	OP_ASSERT(edge->inLinkups);
	OP_DEBUG_IMAGE_CODE_OLD(context->debugLimbClear());
	OP_DEBUG_IMAGE_CODE_OLD(debugLimbEdges(edge));
}

#if 0
	/* 
		before allowing backwards, look to see if a connecting edge:
		- is not available (not in linked list)
		- start matches the current end
		- end differs from iEnd
		- length is proportional to end / iEnd?
	 */
void OpTree::addAlternateEnd() {
	std::vector<FoundEdge> foundEdges;
	if (!totalUsed)
		return;
	OpLimb& lastLimb = nthLimb(totalUsed - 1);
	OpEdge* lastEdge = lastLimb.lastLimbEdge;
#if 1
	if (!lastEdge->alternateEnd)
		return;
#endif
	const OpSegment* segment = lastEdge->segment;
	EdgeMatch end = !lastEdge->which();
	segment->activeAtT(lastEdge, end, MatchZero::no, foundEdges);
	segment->activeNeighbor(lastEdge, end, AllowLinked::yes, foundEdges);
	for (const FoundEdge& foundEdge : foundEdges) {
		OpEdge* test = foundEdge.edge;
		EdgeMatch foundEnd = foundEdge.neighborEnd;
		if (EdgeMatch::none == foundEnd)
			continue;
		// found edge was not found in earlier passes because it is in linked list
		OP_ASSERT(test->priorEdge);
		OpPoint otherEnd = EdgeMatch::start == foundEnd ? test->endPt() : test->startPt();
		OpPoint curveEnd = test->curve.whichPt(!foundEnd);
		if (otherEnd == curveEnd || !otherEnd.isFinite())
			continue;
		// find opposite point matching found edge end
		OpPoint complementEnd;
		for (const OpIntersection* sect : foundEdge.edge->segment->sects.i) {
			if (sect->ptT.pt != otherEnd)
				continue;
			for (const OpEdge& edge : sect->opp->segment->edges) {
				if (edge.startPt() == otherEnd) {
					complementEnd = edge.curve.firstPt();
					break;
				}
				if (edge.endPt() == otherEnd) {
					complementEnd = edge.curve.lastPt();
					break;
				}
			}
			OpPoint lastEdgePt = lastEdge->whichCurvePt(EdgeMatch::end);
			if (containsFiller(&lastLimb, lastEdgePt, complementEnd))
				continue;
			if (!complementEnd.isFinite() || complementEnd == curveEnd) 
				continue;
		#if 0  // done when creating edge sets alternate end bit
			// consider line from last limb end to complement end
			// does its length compared to the sect points distance match?
			float sectDistance = (curveEnd - complementEnd).length();
			float fillerLength = (lastEdgePt - complementEnd).length();
			float ratio = fillerLength / sectDistance;
			PathOpsV0Lib::CurveConst altEndFuncPtr = 
					context->callback(lastEdge->curve.c.type).maxAlternateEndFuncPtr;
			float maxAltEnd = altEndFuncPtr ? (*altEndFuncPtr)(lastEdge->curve.c) : 4.0f;
			if (ratio > maxAltEnd)
				continue;
		#endif
			OpEdge* filler = addFiller(sect->opp->segment, lastEdgePt, complementEnd, 
					true);
			filler->setWhich(EdgeMatch::start);
			OpLimb* branch = makeLimb();
			OpRect fillerBounds = filler->bounds();
			branch->set(*this, filler, &lastLimb, lastLimb.match, LimbPass::alternateEnd, 
					nullptr, 0, nullptr, &fillerBounds);
			limbPass = LimbPass::none;
			return;
		}
	}
}
#endif

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
					OpEdge* filler = addFiller(unSect->segment, unSect->ptT.pt, unOpp->ptT.pt, 
							FillerGap::yes);
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

#if 0
void OpTree::addUnsectableLoop(OpJoiner& joiner, OpLimb* end) {
	OpEdge* joinEdge = joiner.edge;
	OpPtT startI = joinEdge->whichSect();
	OpEdge* filler = addFiller(end->edge->segment, { end->lastPts[0], end->lastT }, startI, false);
	OpLimb* limb = makeLimb();
	limb->set(*this, filler, end, EdgeMatch::start, LimbPass::disjoint, 
			nullptr, 0, nullptr);
	if (!bestLimb)
		bestLimb = limb;
}
#endif

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
		case LimbPass::smallEdge:
			break;
//		case LimbPass::alternateEnd:
//			break;
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
	OP_DEBUG_DUMP_CODE(context->debugErasures = &linkupsErasures);
	const OpLimb* bestL = bestLimb;
	OpEdge* best = bestL->edge;
	if (EdgeMatch::end == bestL->match) {
        OP_ASSERT(EdgeMatch::none != best->which()  // !!! assert may be unnecessary; make sure disabled is correct choice
                || (best->disabled && (LimbPass::disabledBackwards == bestL->treePass
				|| LimbPass::disabledCenterless == bestL->treePass))
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
	OP_DEBUG_DUMP_CODE(context->dumpFile("tree"));
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
	// in dump mode, this does not release filler which is in output, for raster debugging
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

// Mark edges' 'seen' as unset in this tree. Later, mark additional contours as they are linked
void OpTree::makeTrunk(OpEdge* edge) {
	OpContour* edgeContour = edge->segment->contour;
	for (OpContour* member : edgeContour->members()) {
		member->setSeen(id);
	}
	context->resetLimbs();
	trunk = makeLimb();
	OP_ASSERT(edgeContour->linkups.l.back() == edge);
	trunk->set(*this, edge, nullptr, EdgeMatch::start, LimbPass::linked, 
			edgeContour, edgeContour->linkups.l.size() - 1, edge);
	edge->startSeen = true;
	edge->lastEdge->endSeen = true;
	if (bestLimb)
		return;
	// !!! can I know that edge never has prior, and is never loop?
	do {
		for (OpContour* member : edgeContour->members()) {
			initialize(*member);
		}
		int index = 0;
		do {
			OpLimb& limb = nthLimb(index);
			OpEdge* endEdge = limb.edge;
			if (endEdge->lastEdge)
				endEdge = endEdge->lastEdge;
			else if (endEdge->priorEdge)
				endEdge = endEdge->advanceToEnd(EdgeMatch::start);
			for (OpContour* member : endEdge->segment->contour->members()) {
				if (member->treeID != id)
					member->setSeen(id);
				limb.addEach(*member, *this);
			}
			if (totalUsed > maxLimbs) {
				OP_DEBUG_DUMP_CODE(context->dumpFile("treeError"));
				context->setError(PathOpsV0Lib::ContextError::tree  
						OP_DEBUG_PARAMS(edge->id));
				return;
			}
		} while (++index < totalUsed);
		if (LimbPass::disabledBackwards < ++limbPass) {
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
			disabled = disabledLength && enabledLength / disabledLength < enabledRatio;
			if (!disabled)
				return;
			test = bestGapLimb;
			do {
				if (test->edge->linkHead)
					test->edge->segment->contour->removeLink(test->edge);
				test = test->parent;
			} while (test);
			return;  // error if bestLimb == nullptr
		}
	} while (!bestLimb);
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
			for (auto& e : segment.edges) {
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
    OP_DEBUG_DUMP_CODE(context->dumpFile("linkRemaining"));
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
		if (!matchLinks(contour, true))
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
//		OP_ASSERT(!opp->disabled);  // !!! may be disabled if tiny -- segment points no longer moved
        if (opp->disabled)
            break;
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
//		OP_ASSERT(!opp->disabled);  // !!! may be disabled if tiny -- segment points no longer moved
        if (opp->disabled)
            break;
		OpEdge* nextEdge = &opp->edges.front();
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
    OP_DEBUG_DUMP_CODE(context->dumpFile("linkUnambiguous"));
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
	tree.makeTrunk(edge);
	if (tree.disabled)
		return false;
	if (PathOpsV0Lib::ContextError::none != context->error)
		return false;
	// adding gap edge in unsect pair case
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

void LinkUps::clear() {
    l.clear();
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
