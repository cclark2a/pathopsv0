// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "PathOps.h"

#if OP_DEBUG_IMAGE
#include "OpDebugColor.h"
#endif

// opp and contour share at least one coincident segment or edge; makes opp member of contour set
void OpContour::addMerge(OpContour* opp) {
	if (this == opp)
		return;
	if (merges.end() == std::find(merges.begin(), merges.end(), opp))
		merges.push_back(opp);
	std::vector<OpContour*>& oMerges = opp->merges;
	if (oMerges.end() == std::find(oMerges.begin(), oMerges.end(), this))
		oMerges.push_back(this);
}

// !!! disabled assuming new approach will find intersecting contours even if outside sects
// !!! If edge is disabled, but its winding was transferred to another edge (potentially in another 
// !!! contour) remember that to check to see if coin edge should also be added. (fuzz763_1823)
void OpContour::addEdges() {
	for (auto& segment : segments) {
		for (auto& edge : segment.edges) {
			if (edge.disabled)
				continue;
			if (edge.bounds.height())
				inX.push_back(&edge);
			if (edge.bounds.width())
				inY.push_back(&edge);
		}
	}
	std::sort(inX.begin(), inX.end(), [](const OpEdge* s1, const OpEdge* s2) {
			return s1->bounds.left < s2->bounds.left; });
	std::sort(inY.begin(), inY.end(), [](const OpEdge* s1, const OpEdge* s2) {
			return s1->bounds.top < s2->bounds.top; });
}

#if 0  // !!! disabled (see above)
// If an edge is disabled, but its winding was transferred to another edge (potentially in another 
// contour) add the coincident edge to the list. (fuzz763_1823)
// !!! this avoids adding the same edge more than once, at the expense of scanning all edges
//     if this is a performance concern, defer duplicate check until after sort
void OpContour::addCoinEdges() {
	for (auto& segment : segments) {
		for (auto& edge : segment.edges) {
			if (!edge.disabled)
				continue;
			for (CoinPal& coinPal : edge.coinPals) {
				OpSegment* coinSeg = coinPal.opp;
				if (sects.end() != std::find(sects.begin(), sects.end(), coinSeg->contour))
					continue;
				for (auto& cEdge : coinSeg->edges) {
					if (cEdge.ptBounds.height() 
							&& inX.end() == std::find(inX.begin(), inX.end(), &cEdge))
						inX.push_back(&cEdge);
					if (cEdge.ptBounds.width() 
							&& inY.end() == std::find(inY.begin(), inY.end(), &cEdge))
						inY.push_back(&cEdge);
				}
			}
		}
	}
}
#endif

void OpContour::addLast(OpEdge* edge) {
	OP_ASSERT(edge->lastEdge);
	OP_ASSERT(edge->lastEdge->segment->contour == this);
	OP_ASSERT(!edge->priorEdge);
#if OP_DEBUG
	for (OpEdge* test : endLinks.l) {
		OP_ASSERT(test->lastEdge->segment->contour == this);
		OP_ASSERT(test != edge);
		OP_ASSERT(!test->priorEdge);
		OP_ASSERT(test->lastEdge);
	}
#endif
	endLinks.l.push_back(edge);
}

void OpContour::addJoinEdge(OpJoiner* joiner, OpEdge* e) {
	OP_ASSERT(!e->inOutput);
	OP_ASSERT(e->segment->contour == this);
	if (e->priorEdge || e->nextEdge)
		return;
	if (e->disabled)
		return;
	e->setWhich(EdgeMatch::start);
#if OP_DEBUG_IMAGE
	e->debugJoin = true;
#endif
	if (Unsortable::none != e->isUnsortable || e->isUnsectable()) {
		OP_DEBUG_VALIDATE_CODE(e->debugValidate());
		unsortables.push_back(e);
		return;
	}
	OpSegment* seg = e->segment;
	OP_ASSERT(!seg->disabled);
	OP_ASSERT(e->isSimple());
	if (seg->simpleEnd(e) && !OpJoiner::LinkEnd(e))  // returns false if loop was formed
		return;
	if (seg->simpleStart(e))
		e = OpJoiner::LinkStart(e);  // returns new first edge
	if (!e)  // loop
		return;
	OP_ASSERT(!e->priorEdge);
	if (!e->nextEdge) {
		OP_ASSERT(byArea.end() == std::find(byArea.begin(), byArea.end(), e));
		byArea.push_back(e);
		return;
	}
	OP_ASSERT(linkups.l.end() == std::find(linkups.l.begin(), linkups.l.end(), e));
	OpEdge* last = e->updateLastEdge();
	if (e->startPt() == last->endPt()) {
		OP_ASSERT(!last->nextEdge);
		last->setNextEdge(e);
		OP_ASSERT(!e->priorEdge);
		e->setPriorEdge(last);
		e->outputLinkedList();
	} else {
//		e->segment->contour->pushLinkup(e);
		addToLinkups(joiner, e);
		e->setLinkBounds();
	}
}

void OpContour::addToLinkups(OpJoiner* joiner, OpEdge* e) {
	OP_ASSERT(!e->debugIsLoop());
	OpEdge* first = e->advanceToEnd(EdgeMatch::start);
	OpEdge* next = first;
	OpEdge* last;
	do {
		if (LinkPass::remaining != joiner->linkPass && LinkPass::none != joiner->linkPass) {
//			OP_ASSERT(next->isActive());
			next->setActive(false);
		}
		next->clearLast(/* InOutput::no */);
		next->inLinkups = true;
		last = next;
		next = next->nextEdge;
	} while (next);
	first->setLast(first, last, InOutput::no);
	OP_ASSERT(first->linkBounds.isFinite());
#if OP_DEBUG_VALIDATE
#if OP_DEBUG_DUMP
	if (first->debugScheduledForErasure)
		context->dumpFile("addToLinkups debugScheduledForErasure");
#endif
	OP_ASSERT(!first->debugScheduledForErasure);
#endif
	first->segment->contour->linkups.l.push_back(first);	// !!! call pushlinkup?
	first->linkHead = true;
}

void OpContour::buildBackwards() {
	for (auto& segment : segments) {
		for (auto& e : segment.edges) {
			if (e.disabled && Unsortable::none == e.isUnsortable && !e.isUnsectable()
					&& !e.centerless && !e.coinPals.size())
				disabledBackwards.push_back(&e);
		}
	}
	backwardsBuilt = true;
}

void OpContour::buildCenterless() {
	// example that needs small factor: testQuads18787007
//	OpVector threshold = contours.threshold() * OpMath::smallJoinerFactor;
	for (auto& segment : segments) {
		for (auto& e : segment.edges) {
			if (!e.disabled || Unsortable::none != e.isUnsortable || e.isUnsectable())
				continue;
			// for the very small, include disabled edges
			// !!! this also tested on windPal, but non-extended tests don't need it
			if (e.centerless || e.coinPals.size()) // entire segment is not coincident; partial is
				disabledCenterless.push_back(&e);
		}
	}
	centerlessBuilt = true;
}

void OpContour::buildPals() {
	for (auto& segment : segments) {
		for (auto& e : segment.edges) {
			if (e.disabled && !e.inOutput && Unsortable::none == e.isUnsortable) {
				// !!! test may be overbroad; may need to look at sect and include only
				//     coin + unsect (or add bit in edge to register coin)
				if (e.isUnsectable()) {
					disabledPals.push_back(&e);
					e.setWhich(EdgeMatch::start);
				}
			}
		}
	}
	std::sort(disabledPals.begin(), disabledPals.end(), [](OpEdge* a, OpEdge* b)
			{ return a->bounds.perimeter() < b->bounds.perimeter(); }
	);
	palsBuilt = true;
}

void OpContour::clear() {
    sorted.clear();
    overlaps.clear();
    merges.clear();
    inX.clear();
    inY.clear();
    byArea.clear();
    unsectByArea.clear();
    disabledBackwards.clear();
    disabledCenterless.clear();
    disabledPals.clear();
    unsortables.clear();
    linkups.clear();
    endLinks.clear();
    overlapBounds.clear();
    bounds.clear();
    init();
}

void OpContour::clearEdges() {
	for (auto& segment : segments) {
		for (auto& e : segment.edges) {
            e.inOutput = false;
        }
    }
}

void OpContour::clearSegments() {
	for (auto& segment : segments) {
	    segment.winding = segment.contour->winding();
        segment.init();
        segment.sects.clear();
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
bool OpContour::detachIfLoop(OpJoiner* joiner, OpEdge* e, EdgeMatch loopMatch) {
    if (context->windingCallbacks.windingWoundFuncPtr)
        return false;
    std::vector<LoopCheck> edges;
	OpEdge* test = e;
	// walk forwards to end, keeping one point per edge
	OP_ASSERT(e /* && !e->debugIsLoop() */ );  // !!! do not understand
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
        EdgeOutput edgeOutput(context, e, true);
		OP_DEBUG_VALIDATE_CODE(joiner->debugValidate());
		return true;
	}
	// walk backwards to start
	std::sort(edges.begin(), edges.end());
	auto detachEdge = [this, joiner](OpEdge* e, EdgeMatch match) 
	{
		if (OpEdge* detach = EdgeMatch::start == match ? e->priorEdge : e->nextEdge) {
			EdgeMatch::start == match ? detach->clearNextEdge() : detach->clearPriorEdge();
			if (Unsortable::none == detach->isUnsortable || detach->priorEdge || detach->nextEdge)
				addToLinkups(joiner, detach);	// return front edge
		}
	};
	auto detachNext = [detachEdge](OpEdge* test, OpEdge* oppEdge) 
	{
		detachEdge(test, EdgeMatch::end);
		detachEdge(oppEdge, EdgeMatch::start);
		test->setNextEdge(oppEdge);
		oppEdge->setPriorEdge(test);
        EdgeOutput edgeOutput(test->context(), test, true);
		return true;
	};
	auto detachPrior = [detachEdge](OpEdge* test, OpEdge* oppEdge) {
		detachEdge(test, EdgeMatch::start);
		detachEdge(oppEdge, EdgeMatch::end);
		test->setPriorEdge(oppEdge);
		oppEdge->setNextEdge(test);
        EdgeOutput edgeOutput(test->context(), test, true);
		return true;
	};
	test = e;
	while ((test = (EdgeMatch::start == loopMatch ? test->priorEdge : test->nextEdge)) && e != test) {
		LoopCheck testCheck(test, !loopMatch);
		if (auto bound = std::lower_bound(edges.begin(), edges.end(), testCheck); 
				bound != edges.end() && bound->pt == testCheck.pt)
			return EdgeMatch::start == loopMatch ? detachNext(bound->edge, test) : 
					detachPrior(bound->edge, test);
	}
	OP_DEBUG_VALIDATE_CODE(joiner->debugValidate());
	return false;
}

// !!! bare minimum to fix cubic129075 (experiment)
bool OpContour::disabledPal(OpPoint a, OpPoint b) const {
	OP_ASSERT(a != b);
	for (OpEdge* edge : disabledPals) {
		OP_ASSERT(edge->startPt() != edge->endPt());
		if (edge->startPt() != a && edge->startPt() != b)
			continue;
		if (edge->endPt() != a && edge->endPt() != b)
			continue;
		return true;
	}
	return false;
}

// !!! reverse return bool : now true if no edges to join (reverse caller also)
bool OpContour::joinSetup() {
	if (!byArea.size() && !unsectByArea.size() && !linkups.l.size())
		return true;
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
	return false;
}

static bool compareSize(const OpEdge* s1, const OpEdge* s2) {
	const OpRect& r1 = s1->bounds;
	const OpRect& r2 = s2->bounds;
	return r1.width() + r1.height() > r2.width() + r2.height();
}

void OpContour::joinSort() {
	std::sort(byArea.begin(), byArea.end(), compareSize);
	std::sort(unsectByArea.begin(), unsectByArea.end(), compareSize);
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
bool OpContour::linkUp(OpJoiner* joiner, OpEdge* e) {
	for (;;) {
		OP_ASSERT(++joiner->debugRecursiveDepth < 630);	// !!! set to deepest test
		EdgeMatch linkMatch = joiner->linkMatch;
		std::vector<FoundEdge> foundEdges;
		OP_ASSERT(!e->debugIsLoop(EdgeMatch::end, LeadingLoop::will));
		const OpSegment* segment = e->segment;
		bool hasPal = segment->activeAtT(e, linkMatch, MatchZero::yes, foundEdges);
		hasPal |= segment->activeNeighbor(e, linkMatch, AllowLinked::no, foundEdges);
		// if oppEdges is count of one and unsortable, don't return any edges (testQuadratic67x)
		if (foundEdges.size() == 1 && Unsortable::none != foundEdges[0].edge->isUnsortable /* && hadLinkTo */)
			foundEdges.clear(); // hadLinkTo breaks thread_cubics147521
		// skip pals should choose the pal that minimizes the output path area
		// if there's not enough info here to do that, the pal choice should be reconsidered
		//   when match links is called
		// !!! maybe the right choice here is the wrong choice later?!
		if ((foundEdges.size() && hasPal)  // if edges[x] has pals and pal is in linkups, remove edges[x]
		//	e->skipPals(linkMatch, edges);
		// if edge has pals, and there's a matching unsortable, don't return edge (thread_cubics502920)
				|| 1 != foundEdges.size() || !foundEdges[0].edge->isActive()) {
			if (EdgeMatch::start == linkMatch)
				return true;  // 1) found multiple possibilities, try end
			e->segment->contour->addToLinkups(joiner, e);
			return false;  // 2) found multiple possibilities (end)
		}
		FoundEdge foundOne = foundEdges.front();
		OP_DEBUG_VALIDATE_CODE(joiner->debugValidate());
		e->linkToEdge(foundOne, linkMatch);
		#if 0  // !!! this assert is wrong : see if a test fails if it is ignored
		OP_ASSERT(e->compareMatch(linkMatch, foundOne.edge, 
				linkMatch == foundOne.edge->which() ? EdgeMatch::end : EdgeMatch::start));
		#endif
//		OP_ASSERT(e->whichSect(linkMatch).pt.isNearly(foundOne.edge->flipPtT(linkMatch).pt,
//                context->threshold()));  // !!! old pre-collect code
//		OP_DEBUG_VALIDATE_CODE(joiner->debugValidate());  // can't validate; loop isn't processed
		if (detachIfLoop(joiner, e, linkMatch))
			return false; // 4) found loop, nothing leftover; caller to move on to next edge
		OP_DEBUG_VALIDATE_CODE(joiner->debugValidate());
		// move to the front or back edge depending on link match
		e = foundOne.edge->advanceToEnd(linkMatch);  // 5)  recurse to extend prior or next
	}
}

// check if resolution of link ups left unambiguous edge ends for further linkage
// !!! this is missing a check to see if the matched edge has the correct winding
// at very least, it should have an assert
RelinkJoins OpContour::relinkUnambiguous(OpJoiner* joiner, size_t link) {
	if (link >= linkups.l.size())
		return RelinkJoins::done;
	size_t tIndex = 0;
	std::vector<OpEdge*> linkupsErasures;
	OpContour* tContour = nullptr;
	joiner->edge = linkups.l[link];
	OpEdge* edge = joiner->edge;
	{ // must have at least two link ups to hook together
		EdgeMatch tMatch;
		auto scanForMatch = [&tMatch, &tIndex, link, this, &tContour](OpEdge* eEdge, 
				OpContour* eContour, EdgeMatch eMatch) {
			OpPoint edgePt = eEdge->whichSect(eMatch).pt;
			auto testUnmatch = [edgePt](OpEdge* test, EdgeMatch match) {
				return test->whichSect(match).pt == edgePt;
			};
			tMatch = EdgeMatch::none;
			for (OpContour* member : eContour->members()) {
				for (const std::vector<OpEdge*>& edges : { member->unsectByArea, member->unsortables } ) {
					for (OpEdge* test : edges) {
						if (testUnmatch(test, EdgeMatch::start))
							return false;
						if (testUnmatch(test, EdgeMatch::end))
							return false;
					}
				}
				for (size_t index = 0; index < member->linkups.l.size(); ++index) {
					if (index == link && member == this)
						continue;
					auto testMatch = [&tMatch, &tIndex, &tContour, index, member, edgePt]
							(OpEdge* test, EdgeMatch match) {
						if (test->whichSect(match).pt == edgePt) {
							if (tMatch != EdgeMatch::none)
								return false;  // there is more than one match; give up on this end
							tMatch = match;
							tIndex = index;
							tContour = member;
						}
						return true;
					};
					OpEdge* test = member->linkups.l[index];
					if (!testMatch(test, EdgeMatch::start))
						return false;
					if (!testMatch(test->lastEdge, EdgeMatch::end))
						return false;
				}
			}
			return EdgeMatch::none != tMatch;
		};
		// single edge end found which matches; link the two
		auto mergeLinks = [&tIndex, &tMatch, &tContour, &linkupsErasures](
				OpEdge* e, EdgeMatch eMatch, OpContour* linkContour, size_t linkIndex) {
			OpEdge* tEdge = tContour->linkups.l[tIndex];
			OP_ASSERT(!tEdge->priorEdge);
			if (EdgeMatch::start == eMatch) {
				if (EdgeMatch::start == tMatch) {
					if (!tEdge->nextEdge)
						tEdge->setWhich(!tEdge->which());
					else {
						tEdge = tEdge->lastEdge;
						tEdge->setLinkDirection(tMatch, &linkupsErasures, InOutput::no);  // reverse links
						tEdge->setLinkBounds();
						tContour->setLinkEdge(tEdge, tIndex);
					}
				}
				e->setPriorEdge(tEdge->lastEdge);
				tEdge->lastEdge->setNextEdge(e);
				tEdge->setLast(e, e->lastEdge, InOutput::no);
			} else {
				if (EdgeMatch::end == tMatch) {
					tEdge = tEdge->lastEdge;
					tEdge->setLinkDirection(tMatch, &linkupsErasures, InOutput::no);  // reverse links
				}
				e = e->advanceToEnd(EdgeMatch::start);
				tEdge->setPriorEdge(e->lastEdge);
				e->lastEdge->setNextEdge(tEdge);
				e->setLast(tEdge, tEdge->lastEdge, InOutput::no);
			}
			OpEdge* eraseEdge = linkContour->linkups.l[linkIndex];
#if OP_DEBUG_VALIDATE
			eraseEdge->debugScheduledForErasure = true;
#endif
			linkupsErasures.push_back(eraseEdge);
		};
		if (scanForMatch(edge, this, EdgeMatch::start))
			mergeLinks(edge, EdgeMatch::start, this, link);
		else {
			joiner->lastLink = edge->lastEdge;
			OpEdge* lastLink = joiner->lastLink;
			if (!scanForMatch(lastLink, lastLink->segment->contour, EdgeMatch::end))
				return RelinkJoins::unmatched;
			mergeLinks(lastLink, EdgeMatch::end, tContour, tIndex);
		}
	}
	context->linkErased = false;
	bool somethingWasErased = false;
	detachIfLoop(joiner, edge->advanceToEnd(EdgeMatch::start), EdgeMatch::end);
	if (linkupsErasures.size()) {
		for (OpEdge* entry : linkupsErasures) {
			if (!entry->linkHead)
				continue;
			std::vector<OpEdge*>& links = entry->segment->contour->linkups.l;
			OP_ASSERT(entry->debugScheduledForErasure);
			OP_DEBUG_CODE(entry->debugScheduledForErasure = false);
			entry->linkHead = false;
			for (size_t index = 0; index < links.size(); ++index) {
				if (links[index] == entry) {
					links.erase(links.begin() + index);
					somethingWasErased = true;
					break;
				}
			}
		}
	}
	if (!somethingWasErased && !context->linkErased)
		return RelinkJoins::unchanged;
	return RelinkJoins::again;
}

// !!! this had incomplete code that cared about 'InOutput' but didn't do anything with it
//     removing that for now...
void OpContour::removeLast(OpEdge* edge  /*, InOutput inOut */) {
	for (size_t index = 0; index < endLinks.l.size(); ++index) {
		OpEdge* test = endLinks.l[index];
		if (edge == test) {
			endLinks.l.erase(endLinks.l.begin() + index);
//			edge->clearLastEdge();
			return;
		}
	}
#if 0
    if (edge->inOutput || InOutput::yes == inOut)
		return;
	OP_ASSERT(0);
#endif
}

void OpContour::removeLink(OpEdge* edge) {
	for (size_t index = 0; index < linkups.l.size(); ++index) {
		OpEdge* test = linkups.l[index];
		if (edge == test) {
			linkups.l.erase(linkups.l.begin() + index);
		    edge->linkHead = false;
			return;
		}
	}
	OP_ASSERT(0);
}

void OpContour::pushLinkup(OpEdge* edge) {
	OP_ASSERT(edge->segment->contour == this);
	OP_ASSERT(!edge->priorEdge);
#if OP_DEBUG
	for (OpEdge* test : linkups.l) {
		OP_ASSERT(test->segment->contour == this);
		OP_ASSERT(test != edge);
		OP_ASSERT(!test->priorEdge);
		OP_ASSERT(test->lastEdge);
	}
#endif
	OP_ASSERT(!edge->debugScheduledForErasure); 
	linkups.l.push_back(edge);
	edge->inLinkups = true;
	edge->linkHead = true;
}

void OpContour::setLinkEdge(OpEdge* link, size_t index) {
	OpContour* newContour = link->segment->contour;
	OP_ASSERT(!link->debugScheduledForErasure); 
	if (this != newContour)
		newContour->linkups.l.push_back(link);	//!!! call pushLinkup instead?
	else {
		linkups.l[index]->linkHead = false;
		linkups.l[index] = link;
	}
	link->inLinkups = true;
	link->linkHead = true;
}

#if 0
OpIntersection* OpContour::addEdgeSect(const OpPtT& t, OpSegment* seg  
		OP_LINE_FILE_DEF(const OpEdge* edge, const OpEdge* oEdge)) {
	OpIntersection* next = contours->allocateIntersection();
	next->set(t, seg  OP_LINE_FILE_CALLER(edge->id, oEdge->id));
	return next;
}
#endif

OpIntersection* OpContour::addSegSect(const OpPtT& t, OpSegment* seg  
		OP_LINE_FILE_DEF(const OpSegment* oSeg)) {
	OpIntersection* next = context->allocateIntersection();
	next->set(t, seg  OP_LINE_FILE_CALLER(seg->id, oSeg->id));
	return next;
}

OpIntersection* OpContour::addUnsect(const OpPtT& t, OpSegment* seg, int uID, MatchEnds unsectEnd
		OP_LINE_FILE_DEF(const OpSegment* oSeg)) {
	OpIntersection* next = context->allocateIntersection();
	next->set(t, seg  OP_LINE_FILE_CALLER(seg->id, oSeg->id));
	OP_ASSERT(MatchEnds::both != unsectEnd);
	next->setUnsect(uID, unsectEnd);
	return next;
}

bool OpContour::fixCCSects() {
	for (auto& segment : segments) {
		if (segment.disabled)
			continue;
		int safetyCount = 10;
		while (segment.fixCCSects() && --safetyCount)
			;
		if (!safetyCount)
			return context->setError(PathOpsV0Lib::ContextError::loop  OP_DEBUG_PARAMS(segment.id));
	}
	return true;
}

void OpContour::init(OpContext* ctxt, PathOpsV0Lib::WindingData wind, size_t size) {
    init();
	context = ctxt;
	id = ctxt->nextID();
    windingStorage.resize(size);
    std::memcpy(&windingStorage.front(), wind, size);
#if OP_TEST
	debugWinding = OpWinding(this, wind, size);
#endif
#if OP_DEBUG_IMAGE
    int used = ctxt->contourStorage->used;
    if (1 == used)
        debugColor = blue;
    else if (2 == used)
        debugColor = red;
    else
        debugColor = debugColorArray[id % debugColorArray.size()].first;
#endif
}

void OpContour::init() {
	overlapOwner = nullptr;
	treeID = 0;  // tracks if contour has been initialized in this tree's context (for edge 'seen')
	backwardsBuilt = false;
	centerlessBuilt = false;
	hasPals = false;
	palsBuilt = false;
	disabled = false;
	overlapsMerged = false;
}

bool OpContour::isEmpty() {
#if OP_DEBUG_IMAGE
	OP_ASSERT(segments.empty() || !debugCurveData.empty());
	return debugCurveData.empty();
#endif
	return segments.empty();
}

int OpContour::nextID() const {
//    if (93 == contours->uniqueID + 1)
//        OpDebugOut("");
	return context->nextID();
}


void OpContour::setSeen(int tree_id) {
	treeID = tree_id;
	for (OpEdge* test : linkups.l) {
		test->startSeen = false;
	}
	for (OpEdge* test : endLinks.l) {
		test->lastEdge->endSeen = false;
	}
}

void OpContour::unlink(OpEdge* test) {
	if (test->inOutput) {
		test->unlink();
		return;
	}
	OpEdge* first = test->advanceToEnd(EdgeMatch::start);
	if (linkups.l.end() == std::find(linkups.l.begin(), linkups.l.end(), first))
		test->unlink();
}

std::vector<OpEdge*>& OpContour::windingEdges(Axis axis) {
	OP_ASSERT(overlapOwner == this || (inX.empty() && inY.empty()));
	return Axis::horizontal == axis ? overlapOwner->inX : overlapOwner->inY;
}

#if 0
SegmentIterator::SegmentIterator(OpContext* c)
	: contourIterator(c)
	, contourIter(c)
	, segIndex(-1) 
	OP_DEBUG_PARAMS(debugEnded(false)) {
}

OpSegment* SegmentIterator::next() {
	OpSegment* s;
	OP_ASSERT(!debugEnded);
	do {
		if (++segIndex >= (*contourIter)->segments.size()) {
			segIndex = 0;
			do {
				++contourIter;
				if (!(contourIterator.end() != contourIter)) {
					OP_DEBUG_CODE(debugEnded = true);
					return nullptr;
				}
			} while (!(*contourIter)->segments.size());
		}
		s = &(*contourIter)->segments[segIndex];
	} while (s->disabled);
	return s;
}
#endif

OpContourIter::OpContourIter(OpContext* context) {
	storage = context->contourStorage;
	contourIndex = 0;
}

#if 0 && OP_DEBUG
void OpContour::addDebugContourData(PathOpsV0Lib::DebugContourData data, 
        PathOpsV0Lib::DebugContourType type) {
    PathOpsV0Lib::DebugContourData& contourData = debugContourData[(size_t) type];
    contourData.size = data.size;
	if (!data.size) {
		contourData.data = nullptr;
		return;
	}
	contourData.data = context->allocateCallerData(data.size);
	std::memcpy(contourData.data, data.data, data.size);
}
#endif
