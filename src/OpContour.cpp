// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "PathOps.h"

char* OpContext::allocateCallerData(size_t size) {
	if (!callerStorage)
		callerStorage = new CallerDataStorage;
	if (callerStorage->used + size > sizeof(callerStorage->storage)) {
		CallerDataStorage* next = new CallerDataStorage;
		next->next = callerStorage;
		callerStorage = next;
	}
	char* result = &callerStorage->storage[callerStorage->used];
	size_t alignSize = alignof(void*);  // !!! allow caller to specify this?
	size_t alignPart = size % alignSize;
	if (alignPart)
		size += alignSize - alignPart;  // round up to next alignment
	callerStorage->used += size;
	return result;
}

#if OP_DEBUG
void OpContour::addDebugContourData(PathOpsV0Lib::DebugContourData data) {
	if (!data.size) {
		debugCaller.data = nullptr;
		debugCaller.size = 0;
		return;
	}
	debugCaller.data = context->allocateCallerData(data.size);
	std::memcpy(debugCaller.data, data.data, data.size);
	debugCaller.size = data.size;  // !!! don't know if size is really needed ...
}

void OpContext::addDebugContextData(PathOpsV0Lib::DebugContextData data) {
	if (!data.size) {
		debugContextData.data = nullptr;
		debugContextData.size = 0;
		return;
	}
	debugContextData.data = allocateCallerData(data.size);
	std::memcpy(debugContextData.data, data.data, data.size);
	debugContextData.size = data.size;  // !!! don't know if size is really needed ...
}
#endif

#if WINDER_CONTOUR_EXPERIMENT
// !!! disabled assuming new approach will find intersecting contours even if outside sects
// !!! If edge is disabled, but its winding was transferred to another edge (potentially in another 
// !!! contour) remember that to check to see if coin edge should also be added. (fuzz763_1823)
bool OpContour::addEdges() {
	bool addCoin = false;
	for (auto& segment : segments) {
		for (auto& edge : segment.edges) {
			if (edge.disabled) {
	#if 0
				if (!addCoin) {
					for (CoinPal& coinPal : edge.coinPals) {
						addCoin |= sects.end() == std::find(sects.begin(), sects.end(), 
								coinPal.opp->contour);
					}
				}
	#endif
				continue;
			}
			if (edge.ptBounds.height())
				inX.push_back(&edge);
			if (edge.ptBounds.width())
				inY.push_back(&edge);
		}
	}
	std::sort(inX.begin(), inX.end(), [](const OpEdge* s1, const OpEdge* s2) {
		return s1->ptBounds.left < s2->ptBounds.left;
	});
	std::sort(inY.begin(), inY.end(), [](const OpEdge* s1, const OpEdge* s2) {
		return s1->ptBounds.top < s2->ptBounds.top;
	});
	return addCoin;
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

std::vector<OpEdge*>& OpContour::windingEdges(Axis axis) {
	if (winderOwner != this) {
		OP_ASSERT(inX.empty() && inY.empty());
		return winderOwner->windingEdges(axis);
	}
	return Axis::horizontal == axis ? inX : inY;
}

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

void OpContour::removeLast(OpEdge* edge, InOutput inOut) {
	for (size_t index = 0; index < endLinks.l.size(); ++index) {
		OpEdge* test = endLinks.l[index];
		if (edge == test) {
			endLinks.l.erase(endLinks.l.begin() + index);
			edge->lastEdge = nullptr;
			return;
		}
	}
	if (edge->inOutput || InOutput::yes == inOut)
		return;
	OP_ASSERT(0);
}

void OpContour::removeLink(OpEdge* edge) {
	for (size_t index = 0; index < linkups.l.size(); ++index) {
		OpEdge* test = linkups.l[index];
		if (edge == test) {
			linkups.l.erase(linkups.l.begin() + index);
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

#endif

#if 0
OpIntersection* OpContour::addEdgeSect(const OpPtT& t, OpSegment* seg  
		OP_LINE_FILE_DEF(const OpEdge* edge, const OpEdge* oEdge)) {
	OpIntersection* next = contours->allocateIntersection();
	next->set(t, seg  OP_LINE_FILE_CALLER(edge->id, oEdge->id));
	return next;
}
#endif

OpIntersection* OpContour::addCoinSect(const OpPtT& t, OpSegment* seg, int cID, MatchEnds coinEnd
		OP_LINE_FILE_DEF(const OpSegment* oSeg)) {
	OP_ASSERT(MatchEnds::both != coinEnd);
	OpIntersection* next = context->allocateIntersection();
	next->set(t, seg  OP_LINE_FILE_CALLER(seg->id, oSeg->id));
	next->setCoin(cID, coinEnd);  // 0 if no coincidence; negative if coincident pairs are reversed
	return next;
}

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

int OpContour::nextID() const {
//    if (93 == contours->uniqueID + 1)
//        OpDebugOut("");
	return context->nextID();
}


#if WINDER_CONTOUR_EXPERIMENT
void OpContour::setSeen(int tree_id) {
	treeID = tree_id;
	for (OpEdge* test : linkups.l) {
		test->startSeen = false;
	}
	for (OpEdge* test : endLinks.l) {
		test->lastEdge->endSeen = false;
	}
}
#endif

// end of contour; start of contours

bool OpPtAliases::add(OpPoint original, OpPoint alias) {
	OP_ASSERT(original.isFinite());
	OP_ASSERT(alias.isFinite());
	OP_ASSERT(original != alias);
	for (OpPtAlias& test : maps) {
		if (original == test.alias)
			return false;
		if (test.original == original && test.alias == alias)
			return true;
		OP_ASSERT(test.original != alias);
	}
	maps.push_back({original, alias});
	for (OpPoint pt : aliases) {
		if (pt == alias)
			return true;
	}
	aliases.push_back(alias);
	return true;
}

bool OpPtAliases::contains(OpPoint aliased) const {
	OP_ASSERT(aliased.isFinite());
	for (OpPoint pt : aliases) {
		if (pt == aliased)
			return true;
	}
	return false;
}

OpPoint OpPtAliases::existing(OpPoint match) const {
	OP_ASSERT(match.isFinite());
	for (const OpPtAlias& test : maps) {
		if (test.original == match)
			return test.alias;
	}
	return match;
}

#if 0  // there can be more than one. don't know when this behavior is desired
OpPoint OpPtAliases::find(OpPoint aliased) const {
	OP_ASSERT(aliased.isFinite());
	for (const OpPtAlias& test : maps) {
		if (test.alias == aliased)
			return test.original;
	}
	return OpPoint();
}
#endif

bool OpPtAliases::isSmall(OpPoint pt1, OpPoint pt2) {
	OP_ASSERT(pt1.isFinite());
	OP_ASSERT(pt2.isFinite());
	if (pt1.isNearly(pt2, threshold)) {
		if (contains(pt1))
			add(pt2, pt1);
		else if (contains(pt2) || existing(pt1) != existing(pt2))
			add(pt1, pt2);
		return true;
	}
	auto match = [this](OpPoint pt) -> SegPt {
		if (!maps.size())
			return { pt, PtType::noMatch };
		if (contains(pt))
			return { pt, PtType::isAlias };
		for (OpPtAlias& test : maps) {
			if (test.original == pt)
				return { test.alias, PtType::original };
		}
		for (OpPoint alias : aliases) {
			if (pt.isNearly(alias, threshold)) {
				add(pt, alias);
				return { alias, PtType::original };
			}
		}
#if 0
		for (OpPtAlias& test : maps) {
			if (pt.isNearly(test.original, threshold)) {
				add(pt, test.alias);
				return { test.alias, PtType::original };
			}
		}
#endif
		return { pt, PtType::noMatch };
	};
	SegPt match1 = match(pt1);
	SegPt match2 = match(pt2);
	OP_ASSERT(match1.pt != match2.pt 
		|| ((PtType::noMatch == match1.ptType) == (PtType::noMatch == match2.ptType)));
	return PtType::noMatch != match1.ptType && PtType::noMatch != match2.ptType 
			&& match1.pt == match2.pt;
}

void OpPtAliases::remap(OpPoint oldAlias, OpPoint newAlias) {
	OP_ASSERT(oldAlias.isFinite());
	OP_ASSERT(newAlias.isFinite());
	for (OpPtAlias& test : maps) {
		if (test.alias == oldAlias)
			test.alias = newAlias;
	}
	for (size_t index = 0; index < aliases.size(); ++index) {
		if (aliases[index] == oldAlias) {
			aliases.erase(aliases.begin() + index);
			break;
		}
	}
	add(oldAlias, newAlias);
}

SegPt OpPtAliases::addIfClose(OpPoint match) {
	OP_ASSERT(match.isFinite());
	for (OpPoint alias : aliases) {
		if (match == alias)
			return { alias, PtType::isAlias };
		if (match.isNearly(alias, threshold)) {
			add(match, alias);
			return { alias, PtType::original };
		}
	}
	for (const OpPtAlias& alias : maps) {
		if (alias.original.isNearly(match, threshold)) {
			add(match, alias.alias);
			return { alias.alias, PtType::original };
		}
	}
	return { match, PtType::noMatch };
}

OpContext::OpContext()
	: errorHandler({nullptr})
	, ccStorage(nullptr)
	, curveDataStorage(nullptr)
	, contourStorage(nullptr)
	, fillerStorage(nullptr)
	, sectStorage(nullptr)
	, limbStorage(nullptr)
	, limbCurrent(nullptr)
	, callerStorage(nullptr)
	, error(PathOpsV0Lib::ContextError::none)
	, fatalError(false)
	, uniqueID(0) 
	, outputOne(false)
	OP_DEBUG_PARAMS(debugData(false)) {
#if OP_DEBUG_VALIDATE
	debugValidateEdgeIndex = 0;
	debugValidateJoinerIndex = 0;
#endif
#if OP_DEBUG
	debugCurveCurve = nullptr;
	debugJoiner = nullptr;
	debugTree = nullptr;
	debugOutputID = 0;
	debugErrorID = 0;
	debugOppErrorID = 0;
	debugExpect = OpDebugExpect::unknown;
	debugInPathOps = false;
	debugInClearEdges = false;
	debugCheckLastEdge = false;
	debugFailOnEqualCepts = false;
	OP_DEBUG_DUMP_CODE(debugDumpInit = false);
#endif
#if TEST_RASTER
	rasterEnabled = false;
#endif
}

OpContext::~OpContext() {
	release(ccStorage);
	while (curveDataStorage) {
		CurveDataStorage* next = curveDataStorage->next;
		delete curveDataStorage;
		curveDataStorage = next;
	}
	while (contourStorage) {
		OpContourStorage* next = contourStorage->next;
		delete contourStorage;
		contourStorage = next;
	}
	release(fillerStorage);
	while (sectStorage) {
		OpSectStorage* next = sectStorage->next;
		delete sectStorage;
		sectStorage = next;
	}
	if (limbStorage) {
		limbStorage->reset();
		delete limbStorage;
	}
	while (callerStorage) {
		CallerDataStorage* next = callerStorage->next;
		delete callerStorage;
		callerStorage = next;
	}
#if OP_DEBUG
	debugInPathOps = false;
	debugInClearEdges = false;
#endif
#if OP_DEBUG_DUMP
	if (debugDumpInit) {
		delete debugCurveCurve;
		delete debugJoiner;
	}
#endif
}

bool OpContext::addAlias(OpPoint pt, OpPoint alias) {
	   if (!aliases.add(pt, alias)) {
		   remapPts(pt, alias);
		   return false;
	   }
	   return true;
}

OpEdge* OpContext::addFiller(const OpPtT& start, const OpPtT& end) {
	void* block = allocateEdge(fillerStorage);
	// note: start t may be greater than end t (for filler only)
	OpEdge* filler = new(block) OpEdge(this, start, end  OP_LINE_FILE_PARGS());
	return filler;
}

OpContour* OpContext::allocateContour() {
	if (!contourStorage)
		contourStorage = new OpContourStorage;
	if (contourStorage->used == ARRAY_COUNT(contourStorage->storage)) {
		OpContourStorage* next = new OpContourStorage;
		next->next = contourStorage;
		contourStorage = next;
	}
	return &contourStorage->storage[contourStorage->used++];
}

OpEdge* OpContext::allocateEdge(OpEdgeStorage*& edgeStorage) {
	if (!edgeStorage)
		edgeStorage = new OpEdgeStorage;
	if (edgeStorage->used == ARRAY_COUNT(edgeStorage->storage)) {
		OpEdgeStorage* next = new OpEdgeStorage;
		next->next = edgeStorage;
		edgeStorage = next;
	}
	return &edgeStorage->storage[edgeStorage->used++];
}

PathOpsV0Lib::CurveData* OpContext::allocateCurveData(size_t size) {
	if (!curveDataStorage)
		curveDataStorage = new CurveDataStorage;
	if (curveDataStorage->used + size > sizeof(curveDataStorage->storage)) {
		CurveDataStorage* next = new CurveDataStorage;
		next->next = curveDataStorage;
		curveDataStorage = next;
	}
	return curveDataStorage->curveData(size);
}

OpIntersection* OpContext::allocateIntersection() {
	if (!sectStorage)
		sectStorage = new OpSectStorage;
	if (sectStorage->used == ARRAY_COUNT(sectStorage->storage)) {
		OpSectStorage* next = new OpSectStorage;
		OP_ASSERT(!next->next);
		next->next = sectStorage;
		sectStorage = next;
	}
	return &sectStorage->storage[sectStorage->used++];
}

OpLimb* OpContext::allocateLimb() {
	if (limbStorage->used == ARRAY_COUNT(limbStorage->storage)) {
		OpLimbStorage* next = new OpLimbStorage;
		next->nextBlock = limbStorage;
		next->baseIndex = limbStorage->baseIndex + ARRAY_COUNT(limbStorage->storage);
		limbStorage->prevBlock = next;
		limbStorage = next;
	}
	return limbStorage->allocate();
}

PathOpsV0Lib::WindingData* OpContext::allocateWinding(size_t size) {
	void* result = allocateCallerData(size);
	return (PathOpsV0Lib::WindingData*) result;
}

// build list of linked edges
// if they are closed, done
// if not, match up remainder
// make sure normals point same way
// prefer smaller assembled contours
// returns true on success
bool OpContext::assemble() {
	OpJoiner joiner(*this);
#if WINDER_CONTOUR_EXPERIMENT
	bool linkableFound = false;
	for (auto contour : contours) {
		linkableFound |= !contour->joinSetup();  // !!! reverse return (now: true == no linkable edges)
	}
	if (!linkableFound) {
		initOutOnce();
		return true;
	}
	for (LinkPass linkPass : { LinkPass::normal, LinkPass::unsectable } ) {
		for (auto contour : contours) {
			joiner.linkUnambiguous(contour, linkPass);
		}
		bool remaining = false;
#if OP_DEBUG
		for (auto contour : contours) {
			contour->debugMatchRay();
		}
#endif
		// sort contours so that first edge is on the outside
		for (auto contour : sorted) {
			remaining |= !joiner.linkRemaining(contour);
			if (fatalError)
				return false;
		}
		if (!remaining)
			return true;
	}
#else
	if (joiner.setup()) {
		initOutOnce();
		return true;
	}
	for (LinkPass linkPass : { LinkPass::normal, LinkPass::unsectable } ) {
		joiner.linkUnambiguous(linkPass);
		if (joiner.linkRemaining(OP_DEBUG_CODE(this)))
			return true;
		if (fatalError)
			return false;
	}
#endif
	return false;
}

bool OpContext::containsFiller(OpPoint start, OpPoint end) const {
	if (!fillerStorage)
		return false;
	return fillerStorage->contains(start, end);
}

void OpContext::disableSmallSegments() {
	SegmentIterator segIterator(this);
	while (OpSegment* seg = segIterator.next()) {
		seg->disableSmall();
	}
}

void OpContext::initOutOnce() {
	if (outputOne)
		return;
	PathOpsV0Lib::EmptyCallerPath emptyPath = contextCallbacks.emptyCallerPathFuncPtr;
	if (emptyPath)
		(*emptyPath)(callerOutput);
	outputOne = true;
}

#if 01  // breaks skpagentxsites_com55 (add what test this is required for...
void OpContext::markInCoincidence() {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			if (segment.hasCoin)
				segment.sects.markInCoincidence();
		}
	}
}
#endif

OpLimb& OpContext::nthLimb(int index) {
	int blockBase = index & ~(ARRAY_COUNT(limbStorage->storage) - 1);
	if (!limbCurrent || limbCurrent->baseIndex != blockBase) {
		limbCurrent = limbStorage;
		while (limbCurrent->baseIndex != blockBase) {
			OP_ASSERT(limbCurrent->nextBlock);
			limbCurrent = limbCurrent->nextBlock;
		}
	}
	index &= ~blockBase;
	return limbCurrent->storage[index];
}

void OpContext::resetLimbs() {
	if (!limbStorage)
		limbStorage = new OpLimbStorage;
	limbStorage->reset();
}

void OpContext::opsInit() {
	setThreshold();
	OpContourIterator iterator(this);
	for (OpContourIter iter = iterator.begin(); iter != iterator.end(); ++iter) {
		OpContour* contour = *iter;
		if (contour->isEmpty())
			continue;
		contours.push_back(contour);
	}
	normalize();  // collect extremes, map all from 0 to 1, map <= epsilon to zero
	for (OpContour* contour : contours) {
		for (const OpSegment& segment : contour->segments) {
			if (segment.disabled)
				continue;
			contour->bounds.add(segment.ptBounds);
		}
		if (contour->bounds.isFinite())
			maxBounds.add(contour->bounds);
		else
			contour->disabled = true;
	}
	for (size_t outer = 0; outer < contours.size(); ++outer) {
		OpContour* oContour = contours[outer];
		if (oContour->disabled)
			continue;
		oContour->winderOwner = oContour;
		for (size_t inner = outer; inner < contours.size(); ++inner) {
			OpContour* iContour = contours[inner];
			if (iContour->disabled)
				continue;
			if (oContour->bounds.intersects(iContour->bounds)) {
				oContour->sects.push_back(iContour);
				if (oContour != iContour)
					iContour->sects.push_back(oContour);
			}
		}
	}
	for (OpContour* contour : contours) {
		sort(contour->sects.begin(), contour->sects.end(), [](OpContour* a, OpContour* b) {
			return a->id < b->id;
		});
	}
#if WINDER_CONTOUR_EXPERIMENT
	for (size_t index = 0; index < contours.size(); ++index) {
		OpContour* contour = contours[index];
		if (contour->winderOwner != contour)
			continue;
		for (OpContour* sect : contour->sects) {
			contour->sectBounds.add(sect->bounds);
			if (sect->id <= contour->id)
				continue;
			if (sect->sects == contour->sects) {
				sect->winderOwner = contour;
				continue;
			}
		}
	}
#endif
#if TEST_RASTER
	if (rasterEnabled)
		rasterOutput.init();
#endif
}

// If successive runs of the same input are flaky, check to see if identical ids are generated.
// To do this, insert OP_DEBUG_COUNT(*this, _some_identifer_); after every callout.  
// This will compare the dumps of contours and contents to detect when something changed.
// The callouts are removed when not in use as they are not maintained and reduce readability.
// !!! OP_DEBUG_COUNT was unintentionally deleted at some point. Hopefully it is in git history...
bool OpContext::pathOps() {
	OpSegments segments(*this);
	segments.findCoincidences();
	debugValidateIntersections();
	OpSegments sortedSegments(*this);
	sortedSegments.initInX();
	debugValidateIntersections();
	if (empty()) {
		contextCallbacks.emptyCallerPathFuncPtr(callerOutput);
		OP_DEBUG_SUCCESS(*this, true);
	}
	if (FoundIntersections::fail == sortedSegments.findIntersections())
		return setError(PathOpsV0Lib::ContextError::intersection  
				OP_DEBUG_PARAMS(sortedSegments.debugFailSegID));
	debugValidateIntersections();
	if (errorHandler.errorDispatchFuncPtr && !errorHandler.errorDispatchFuncPtr(
			PathOpsV0Lib::ContextError::missing, (PathOpsV0Lib::Context*) this, nullptr)) {
		addDisjointIntersections();
	}
	disableSmallSegments();  // moved points may allow disabling some segments
	if (empty()) {
		contextCallbacks.emptyCallerPathFuncPtr(callerOutput);  // no existing tests exercises
		OP_DEBUG_SUCCESS(*this, true);
	}
	sortIntersections();
	fixCCSects();  // curve-curve intersections may have enough error to put sect list out of order
	sortIntersections();
	findMissingEnds();  // moved pts may require looking in aliases for an end match
	betweenIntersections();  // fill in intersections in coin runs that are missing in other coins
	sortIntersections();
	markInCoincidence();
	makeEdges();
	makeCoins();
	sortIntersections();
	transferCoins();
	makePals();  // edges too close to each other to sort or precisely intersect

	// made edges may include lines that are coincident with other edges. Undetected for now...
//    windCoincidences();  // for segment h/v lines, compute their winding considering coincidence
	OpWinder windingEdges(*this);
	FoundWindings foundWindings = windingEdges.setWindings(this);  // walk edges, compute windings
	if (FoundWindings::fail == foundWindings)
		OP_DEBUG_FAIL(*this, false);  // no existing tests exercises
	OP_DEBUG_DUMP_CODE(debugContext = "apply");
	apply();  // suppress edges which don't meet op criteria
	if (!assemble())
		OP_DEBUG_FAIL(*this, false);
	// !!! missing final step to reverse order of contours as winding rule requires
	// this should be driven by user choices since the engine itself can't know the winding rule
	// it does require all output contours to be completed first. Perhaps the link-to-path
	// step should be removed from assemble or placed at the end of assemble, so the reverse
	// link loop can be decided once all loops are known
	// !!! for now, set Skia adapter to create evenodd fills
	OP_DEBUG_SUCCESS(*this, true);
}

void OpContext::release(OpEdgeStorage*& edgeStorage) {
	while (edgeStorage) {
		OpEdgeStorage* next = edgeStorage->next;
		delete edgeStorage;
		edgeStorage = next;
	}
}

OpPoint OpContext::remapPts(OpPoint oldAlias, OpPoint newAlias) {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			segment.remap(oldAlias, newAlias);
		}
	}
	aliases.remap(oldAlias, newAlias);
	return newAlias;
}

// seems too complicated to reuse multiple edge storage blocks, so delete all but first
#if 0
void OpContext::reuse(OpEdgeStorage* edgeStorage) {
	if (!edgeStorage)
		return;
	OpEdgeStorage* next = edgeStorage->next;
	while (next) {
		OpEdgeStorage* following = next->next;
		delete next;
		next = following;
	}
	edgeStorage->reuse();
}
#endif

bool OpContext::setError(PathOpsV0Lib::ContextError e  OP_DEBUG_PARAMS(int eID, int oID)) {
	if (fatalError)
		return false;
#if OP_DEBUG
	OP_ASSERT(debugData.limitContours <= 0);  // break when debugging limited number of contours
	if (PathOpsV0Lib::ContextError::finite != e 
			&& PathOpsV0Lib::ContextError::toVertical != e
			&& PathOpsV0Lib::ContextError::gap != e)
		OpDebugOut("fatal error in " + debugData.testname + "\n");
#endif
	fatalError = PathOpsV0Lib::ContextError::gap != e;
	if (!fatalError && PathOpsV0Lib::ContextError::none != error)
		return false;
	error = e;
	OP_DEBUG_CODE(debugErrorID = eID);
	OP_DEBUG_CODE(debugOppErrorID = oID);
	return false;
}

// if one contour is entirely to the left or above another, put it first
//     if x-bounds or y-bounds do intersect, and
//     if contour 1 right/bottom < contour 2 left/top, traverse contour 1 before contour 2
// otherwise, return smaller first  !!! why?
void OpContext::setSortedBounds() {
	OP_ASSERT(sorted.empty());
	for (OpContour* contour : contours) {
		OP_ASSERT(!contour->isSorted(Axis::horizontal));
		OP_ASSERT(!contour->isSorted(Axis::vertical));
		if (contour->bounds.isEmpty())
			continue;
		OP_ASSERT(contour->bounds.isFinite());
		sorted.push_back(contour);
	}
	std::sort(sorted.begin(), sorted.end(), [](const OpContour* a, const OpContour* b) {
		bool xOverlaps = a->bounds.right >= b->bounds.left && b->bounds.right >= a->bounds.left;
		bool yOverlaps = a->bounds.bottom >= b->bounds.top && b->bounds.bottom >= a->bounds.top;
		if (xOverlaps && !yOverlaps) 
			return a->bounds.top < b->bounds.top;
		if (!xOverlaps && yOverlaps) 
			return a->bounds.left < b->bounds.left;
		return a->bounds.perimeter() < b->bounds.perimeter();
	});
}

void OpContext::setThreshold() {
	auto threshold = [](float left, float right) {
		return std::max({1.f, fabsf(left), fabsf(right), right - left}) * OpEpsilon;
	};
	aliases.threshold = { threshold(maxBounds.left, maxBounds.right),
			threshold(maxBounds.top, maxBounds.bottom) };
	aliases.thresholdLength = aliases.threshold.length();
}

void OpContext::sortIntersections() {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			segment.sects.sort();
		}
	}
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			segment.sects.mergeNear(aliases);
		}
	}
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			segment.sects.sort();
		}
	}
}

bool OpContext::debugFail() const {
#if OP_DEBUG
	return OpDebugExpect::unknown == debugExpect || OpDebugExpect::fail == debugExpect;
#else
	return false;
#endif
}

#if OP_DEBUG
bool OpContext::debugSuccess() const {
	return OpDebugExpect::unknown == debugExpect || OpDebugExpect::success == debugExpect;
}
#endif

#if OP_DEBUG_VALIDATE
void OpContext::debugValidateIntersections() {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			segment.sects.debugValidate();
		}
	}
}
#endif

SegmentIterator::SegmentIterator(OpContext* c)
	: contours(c)
	, contourIterator(c)
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

OpContourIter::OpContourIter(OpContext* contours) {
	storage = contours->contourStorage;
	contourIndex = 0;
}

