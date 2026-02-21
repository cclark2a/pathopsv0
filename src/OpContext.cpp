// (c) 2025, Cary Clark cclark2@gmail.com
#include "OpContext.h"
#include "OpCurveCurve.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "DebugOps.h"
#include "PathOps.h"

OpContext::OpContext() {
    PathOpsV0Lib::SetCurveCallbacks((PathOpsV0Lib::Context*)(this), 0, { } );
#if OP_DEBUG
    PathOpsV0Lib::SetDebugCurveCallbacks((PathOpsV0Lib::Context*)(this), 0, { } );
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
    clear();
	while (callerStorage) {
		CallerDataStorage* next = callerStorage->next;
		delete callerStorage;
		callerStorage = next;
	}
#if OP_TEST_RASTER
	while (rasterStorage) {
		CallerDataStorage* next = rasterStorage->next;
		delete rasterStorage;
		rasterStorage = next;
	}
#endif
#if OP_DEBUG
	debugInPathOps = false;
	debugInClearEdges = false;
#endif
#if OP_DEBUG_DUMP
	if (debugDumpInit) {
		delete debugCurveCurve;
		delete debugJoiner;
		delete debugTree;
	}
#endif
}

OpEdge* OpContext::addFiller(OpPoint start, OpPoint end, OpSegment* parent) {
	void* block = allocateEdge(fillerStorage  OP_DEBUG_PARAMS("fillerStorage"));
	OpEdge* filler = new(block) OpEdge(this, start, end  OP_LINE_FILE_PARGS());
	filler->segment = parent;
	return filler;
}

void OpContext::addUserData(PathOpsV0Lib::ContextUserData contextUserData) {
    userData.push_back(contextUserData);
}

#if 0
// !!! incomplete
// Need to record curve/curve or line/curve raw associations of interecting pts 
// (before meet-in-the-middle). Intent is to find one point to reprsent all intersections
// where three or more curves intersect or nearly intersect.
void OpContext::aliasIntersections() {
	for (OpContour* contour : contours) {
		std::vector<OpIntersection*> sects;
		OpPointBounds bounds;
		// collect all intersections not on either end of segment
		for (OpSegment& segment : contour->segments) {
			for (OpIntersection* sect : segment.sects.i) {
				if (0 == sect->ptT.t || 1 == sect->ptT.t)
					continue;
				sects.push_back(sect);
				bounds.add(sect->ptT.pt);
			}
		}
		if (!bounds.isFinite())
			continue;
		std::sort(sects.begin(), sects.end(), []
				(const OpIntersection* s1, const OpIntersection* s2) {
				return s1->ptT.pt.x < s2->ptT.pt.x || (s1->ptT.pt.x == s2->ptT.pt.x 
				&& s1->ptT.pt.y < s2->ptT.pt.y); } ); 
		// custom binary search range (can't see how to use standard library...)
		auto checkSearch = [sects](int lo, OpPoint check) {
			int hi = (int) sects.size() - 1;
			do {
				int mid = (lo + hi) / 2;
				OP_ASSERT(0 <= mid && mid < (int) sects.size());
				const OpPoint test = sects[mid]->ptT.pt;
//				if (test == check)
//					return mid;
				if (test.x < check.x || (test.x == check.x && test.y < check.y))
					lo = mid + 1;
				else
					hi = mid - 1;
			} while (lo <= hi);			
			return lo;
		};
		auto checkNear = [this, sects, checkSearch, bounds](OpPoint check) {
			OpPointBounds checkRange { check - threshold, check + threshold };
			if (!checkRange.intersects(bounds))
				return;
			int lo = checkSearch(0, { checkRange.left, checkRange.top } );
			int hi = checkSearch(lo, { checkRange.right, checkRange.bottom } );
			// lo is first larger than left/top; hi is first larger than right/bottom
			for (int index = lo; index < hi; ++index) {
				if (!checkRange.contains(sects[index]->ptT.pt))
					continue;
				if (check == sects[index]->ptT.pt)
					continue;
				OpNop();
			}
			// !!! incomplete, more code goes here
		};
		// iterate through all contours that intersect, looking for close points
		for (OpContour* testContour : contour->members()) {
			for (OpSegment& testSegment : testContour->segments) {
				if (!testSegment.c.aliasBounds().intersects(bounds))
					continue;
				checkNear(testSegment.c.start);
				checkNear(testSegment.c.end);
			}
		}
	}
}
#endif

uint8_t* OpContext::allocateCallerData(size_t size  OP_DEBUG_RASTER_PARAMS(bool raster)) {
#if OP_TEST_RASTER
	CallerDataStorage*& storage = raster ? rasterStorage : callerStorage;
	if (!raster)
		OpNop();
#else
	CallerDataStorage*& storage = callerStorage;
#endif
	if (!storage)
		storage = new CallerDataStorage;
	size_t alignSize = alignof(void*);  // !!! allow caller to specify this?
	size_t alignPart = size % alignSize;
	if (alignPart)
		size += alignSize - alignPart;  // round up to next alignment
	if (storage->used + size > sizeof(storage->storage)) {
		CallerDataStorage* next = new CallerDataStorage;
		next->next = storage;
		storage = next;
	}
	uint8_t* result = &storage->storage[storage->used];
	storage->used += size;
	return result;
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

OpEdge* OpContext::allocateEdge(OpEdgeStorage*& edgeStorage  OP_DEBUG_PARAMS(std::string name)) {
	if (!edgeStorage) {
		edgeStorage = new OpEdgeStorage;
        OP_DEBUG_DUMP_CODE(edgeStorage->debugName = name);
    }
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
	if (!limbStorage)
		limbStorage = new OpLimbStorage;
	if (limbStorage->used == ARRAY_COUNT(limbStorage->storage)) {
		OpLimbStorage* next = new OpLimbStorage;
		next->nextBlock = limbStorage;
		next->baseIndex = limbStorage->baseIndex + ARRAY_COUNT(limbStorage->storage);
		limbStorage->prevBlock = next;
		limbStorage = next;
	}
	return limbStorage->allocate();
}

PathOpsV0Lib::WindingData* OpContext::allocateWinding(size_t size  
		OP_DEBUG_RASTER_PARAMS(bool usedByRaster)) {
	uint8_t* result = allocateCallerData(size  OP_DEBUG_RASTER_PARAMS(usedByRaster));
	return (PathOpsV0Lib::WindingData*) result;
}

bool OpContext::allowError(PathOpsV0Lib::ContextError error, PathOpsV0Lib::Curve* c) {
	return errorHandler.errorDispatchFuncPtr && (*errorHandler.errorDispatchFuncPtr)(error, c);
}

WindingCondition OpContext::apply() {
    allDiscarded = true;
    allKept = true;
	for (auto contour : contours) {
		if (WindingCondition windingCondition = contour->apply())
            return windingCondition;
	}
    if (PathOpsV0Lib::WindingShort windingShort = windingCallbacks.windingShortAllFuncPtr) {
       if (allDiscarded != allKept) {
            PathOpsV0Lib::WindKeep keep = allDiscarded ? PathOpsV0Lib::WindKeep::Discard
                    : PathOpsV0Lib::WindKeep::Start;
            if (WindingCondition condition = (*windingShort)((ContextPtr) this, keep))
                return condition;
        }
        return -1;
    }
	return 0;
}

// build list of linked edges
// if they are closed, done
// if not, match up remainder
// make sure normals point same way
// prefer smaller assembled contours
// returns true on success
bool OpContext::assemble() {
	setSortedBounds();
	OpJoiner joiner(*this);
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
		for (auto contour : sortedContours) {
			remaining |= !joiner.linkRemaining(contour);
			if (fatalError)
				return false;
		}
		if (!remaining)
			return true;
	}
	return false;
}

void OpContext::clear() {
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
}

bool OpContext::containsFiller(OpPoint start, OpPoint end) const {
	if (!fillerStorage)
		return false;
	return fillerStorage->containsPts(start, end);
}

bool OpContext::containsFiller(int ccUnsectableID) const {
	if (!fillerStorage)
		return false;
	return fillerStorage->contains(ccUnsectableID);
}

bool OpContext::containsPals(OpEdge* edge, int totalLimbs) {
	for (const EdgePal& edgePal : edge->pals) {
		int index = 0;
		do {
			OpLimb& limb = nthLimb(index);
			if (limb.edge == edgePal.edge)
				return true;
		} while (++index < totalLimbs);
	}
	return false;
}

int OpContext::curveIndex(int nativeType) const {
    int count = (int) nativeCurveTypes.size();
    if (nativeType < count && nativeCurveTypes[nativeType] == nativeType)
        return nativeType;
    for (int index = 0; index < count; ++index) {
        if (nativeType == nativeCurveTypes[index]) {
            return index;
        }
    }
    return 0;
}

#if 0
void OpContext::demotePalLinks() {
	for (auto contour : contours) {
		if (!contour->hasPals)
			continue;
		for (auto& segment : contour->segments) {
			if (!segment.hasPals)
				continue;
			segment.demotePalLinks();
		}
	}
}
#endif

void OpContext::disableSmallSegments() {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			segment.disableSmall();
		}
	}
}

void OpContext::initOutOnce() {
	if (outputOne)
		return;
	PathOpsV0Lib::EmptyCallerPath emptyPath = contextCallbacks.emptyCallerPathFuncPtr;
	if (emptyPath)
		(*emptyPath)((ContextPtr) this);
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

void OpContext::opsInit() {
    if (initialized || PathOpsV0Lib::ContextError::none != error)
        return;
    initialized = true;
    if (windingSet) {
        clearSegments();
        contours.clear();
        clear();
        sortedContours.clear();
        fillerStorage = nullptr;
        sectStorage = nullptr;
        limbStorage = nullptr;
        limbCurrent = nullptr;
    }
    setThreshold();
	OpContourIterator iterator(this);
	for (OpContourIter iter = iterator.begin(); iter != iterator.end(); ++iter) {
		OpContour* contour = *iter;
		if (contour->isEmpty())
			continue;
        if (windingSet)
            contour->clear();
		contours.push_back(contour);
	}
    windingSet = false;
	zeroSmall();  // map <= threshold to zero, find small curves (disabled later)
	for (OpContour* contour : contours) {
		for (OpSegment& segment : contour->segments) {
			if (segment.disabled)
				continue;
			segment.c.isLine();  // defer until after threshold is set
			contour->bounds.add(segment.c.aliasBounds());
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
		oContour->overlapOwner = oContour;
		for (size_t inner = outer; inner < contours.size(); ++inner) {
			OpContour* iContour = contours[inner];
			if (iContour->disabled)
				continue;
			if (oContour->bounds.intersects(iContour->bounds)) {
				oContour->overlaps.push_back(iContour);
				if (oContour != iContour)
					iContour->overlaps.push_back(oContour);
			}
		}
	}
	for (OpContour* contour : contours) {  // sort so contours can find indentical intersection sets
		std::sort(contour->overlaps.begin(), contour->overlaps.end(), [](OpContour* a, OpContour* b) {
				return a->id < b->id; });
	}
	for (size_t index = 0; index < contours.size(); ++index) {
		OpContour* contour = contours[index];
		if (contour->overlapOwner != contour)
			continue;
		contour->overlapBounds = contour->bounds;
		for (OpContour* member : contour->overlaps) {
			if (member != contour) {
				contour->overlapBounds.add(member->bounds);
				if (member->overlaps == contour->overlaps) {
					member->overlapOwner = contour;  // if identical, point to master
					member->overlaps.clear();  //  and remove duplicate data
					OP_ASSERT(member->overlapBounds.isEmpty());
				}
			}
		}
	}
	normalize();  // if points are close, alias them
}

// If successive runs of the same input are flaky, check to see if identical ids are generated.
// To do this, insert OP_DEBUG_COUNT(*this, _some_identifer_); after every callout.  
// This will compare the dumps of contours and contents to detect when something changed.
// The callouts are removed when not in use as they are not maintained and reduce readability.
// !!! OP_DEBUG_COUNT was unintentionally deleted at some point. Hopefully it is in git history...
WindingCondition OpContext::pathOps() {
	opsInit();
	if (!windingSet) {
        auto checkEmpty = [this]() {
	        if (empty()) {
                PathOpsV0Lib::EmptyCallerPath emptyPath = contextCallbacks.emptyCallerPathFuncPtr;
                if (emptyPath)
		            (*emptyPath)((ContextPtr) this);
		        OP_ASSERT(debugSuccess());  // break to verify that this is correct
                return true;
	        }
            return false;
        };
        windingSet = true;
        OpSegments segments(*this);
	    segments.findCoincidences();
	    debugValidateIntersections();
	    OpSegments sortedSegments(*this);
	    sortedSegments.initInX();
		OP_DEBUG_DUMP_CODE(dumpFile("sortedSegments"));
	    debugValidateIntersections();
	    if (checkEmpty())
		    return 0;
	    if (FoundIntersections::fail == sortedSegments.findIntersections())
		    return setError(PathOpsV0Lib::ContextError::intersection  
				    OP_DEBUG_PARAMS(sortedSegments.debugFailSegID));
	    debugValidateIntersections();
	    if (allowError(PathOpsV0Lib::ContextError::missing))
		    addDisjointIntersections();
//		aliasIntersections();  // merge all intersections that are close together
	    sortIntersections();
//		tripleSect(); // if three or more segments intersect, make the points the same 
//	    disableSmallSegments();  // moved points may allow disabling some segments
	    if (checkEmpty())
		    return 0;  // no existing tests exercises
	    sortIntersections();
	    if (!fixCCSects())  // curve-curve intersections may have enough error to put sect list out of order
		    OP_DEBUG_FAIL(*this, -1);
	    sortIntersections();
//	    findMissingEnds();  // moved pts may require looking in aliases for an end match
	    manyCoincidences();  // fill in intersections in coin runs that are missing in other coins
	    sortIntersections();
	    betweenCoincidence();  // fill in intersections in coin runs that are missing in other coins
	    sortIntersections();
	    markInCoincidence();
		OP_DEBUG_DUMP_CODE(dumpFile("markInCoincidence"));
	    makeEdges();
	    makeCoins();
	    sortIntersections();
	    transferCoins();
	    makePals();  // edges too close to each other to sort or precisely intersect
	    rebuildOverlaps(); // add coincident contours to intersecting contour bounds arrays
		OP_DEBUG_DUMP_CODE(dumpFile("rebuildOverlaps"));

	    // made edges may include lines that are coincident with other edges. Undetected for now...
    //    windCoincidences();  // for segment h/v lines, compute their winding considering coincidence
	    FoundWindings foundWindings = OpWinder::SetWindings(*this);  // walk edges, compute windings
		OP_DEBUG_DUMP_CODE(dumpFile("foundWindings"));
	    if (FoundWindings::fail == foundWindings)
		    OP_DEBUG_FAIL(*this, -1);  // no existing tests exercises
    } else {
        sortedContours.clear();
        clearContours();
        clearEdges();
    }
	WindingCondition windingCondition = apply();  // suppress edges which don't meet op criteria
//	demotePalLinks();  // mark edges that connect pal ends as unsortable so assembly can ignore them
	if (!windingCondition && !assemble())
		OP_DEBUG_FAIL(*this, -1);
	// !!! missing final step to reverse order of contours as winding rule requires
	// this should be driven by user choices since the engine itself can't know the winding rule
	// it does require all output contours to be completed first. Perhaps the link-to-path
	// step should be removed from assemble or placed at the end of assemble, so the reverse
	// link loop can be decided once all loops are known
	// !!! for now, set Skia adapter to create evenodd fills
	OP_DEBUG_SUCCESS(*this, std::max(0, windingCondition));
}

static void addMerges(OpContour* contour, OpContour* mergeOwner, 
		std::vector<OpContour*>& visited) {
	std::vector<OpContour*> mergesCopy = mergeOwner->merges;
	for (OpContour* merge : mergesCopy) {
		if (visited.end() != std::find(visited.begin(), visited.end(), merge))
			continue;
		visited.push_back(merge);
		addMerges(contour, merge, visited);
	}
	if (contour == mergeOwner)
		return;
	std::vector<OpContour*>& merges = mergeOwner->merges;
	if (merges.end() == std::find(merges.begin(), merges.end(), contour))
		merges.push_back(contour);
	std::vector<OpContour*>& backs = contour->merges;
	if (backs.end() == std::find(backs.begin(), backs.end(), mergeOwner))
		backs.push_back(mergeOwner);
}

// after all coincidence has been found, rebuild overlaps
void OpContext::rebuildOverlaps() {
	// update merges to find multiple step coincident contour extensions
	bool hasMerges = false;
	for (OpContour* contour : contours) {
		if (contour->merges.empty())
			continue;
		std::vector<OpContour*> visited = { contour };
		addMerges(contour, contour, visited);
		hasMerges = true;
	}
	if (!hasMerges)
		return;
	// update overlaps from full merges
	for (OpContour* contour : contours) {
		if (contour != contour->overlapOwner)
			continue;
		std::vector<OpContour*> adds;
		std::vector<OpContour*>& overlaps = contour->overlaps;
		for (OpContour* member : overlaps) {
			for (OpContour* inner : member->merges) {
				if (overlaps.end() != std::find(overlaps.begin(), overlaps.end(), inner))
					continue;
				if (adds.end() != std::find(adds.begin(), adds.end(), inner))
					continue;
				adds.push_back(inner);
			}
		}
		if (adds.empty())
			continue;
		contour->overlapsMerged = true;
		for (OpContour* add : adds) {
			overlaps.push_back(add);
			contour->overlapBounds.add(add->bounds);
		}
		std::sort(overlaps.begin(), overlaps.end(), [](OpContour* a, OpContour* b) {
				return a->id < b->id; });
	}
	// if contour gained new overlaps, check for common ownership
	for (OpContour* contour : contours) {
		if (!contour->overlapsMerged)
			continue;
		for (OpContour* inner : contour->overlaps) {
			if (contour == inner)
				continue;
			if (inner->overlaps != contour->overlaps)
				continue;
			inner->overlapOwner = contour;
			inner->overlaps.clear();
			for (OpContour* dups : contours) {
				if (dups->overlapOwner == inner)
					dups->overlapOwner = contour;
			}
		}
	}
}

void OpContext::release(OpEdgeStorage*& edgeStorage) {
	while (edgeStorage) {
		OpEdgeStorage* next = edgeStorage->next;
		delete edgeStorage;
		edgeStorage = next;
	}
}

void OpContext::resetFiller() {
#if OP_DEBUG_DUMP
	if (fillerStorage)
		fillerStorage->debugRelease();
#else
	release(fillerStorage);
	fillerStorage = nullptr;
#endif
}

void OpContext::resetLimbs() {
	if (!limbStorage)
		limbStorage = new OpLimbStorage;
	limbStorage->reset();
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
	OP_ASSERT(sortedContours.empty());
	for (OpContour* contour : contours) {
	//	OP_ASSERT(!contour->isSorted(Axis::horizontal));
	//	OP_ASSERT(!contour->isSorted(Axis::vertical));
		if (contour->bounds.isEmpty())
			continue;
		OP_ASSERT(contour->bounds.isFinite());
		sortedContours.push_back(contour);
	}
	std::sort(sortedContours.begin(), sortedContours.end(), [](const OpContour* a, const OpContour* b) {
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
	auto thresh = [](float left, float right) {
		return std::max({1.f, fabsf(left), fabsf(right), right - left}) * OpEpsilon;
	};
	threshold = { thresh(maxBounds.left, maxBounds.right),
			thresh(maxBounds.top, maxBounds.bottom) };
	thresholdLength = threshold.length();
}

void OpContext::sortIntersections() {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
            if (segment.disabled)
                continue;
			segment.sects.sort();
		}
	}
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
            if (segment.disabled)
                continue;
			segment.sects.mergeNear(contour->aliases);
		}
	}
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
            if (segment.disabled)
                continue;
			segment.sects.sort();
		}
	}
}

PathOpsV0Lib::ContextUserData OpContext::findUserData(PathOpsV0Lib::UserDataType type) {
    for (PathOpsV0Lib::ContextUserData& user : userData) {
        if (user.type == type)
            return user;
    }
    OP_ASSERT(0);
    return { nullptr, 0, PathOpsV0Lib::UserDataType::none };
}


#if OP_DEBUG
#if 0
void OpContext::addDebugContextData(PathOpsV0Lib::DebugContextData data, 
        PathOpsV0Lib::DebugContextType type) {
    PathOpsV0Lib::DebugContextData& contextData = debugContextData[(size_t) type];
    contextData.size = data.size;
	if (!data.size) {
        contextData.data = nullptr;
		return;
	}
	contextData.data = allocateCallerData(data.size);
	std::memcpy(contextData.data, data.data, data.size);
}

PathOpsV0Lib::DebugContextData& OpContext::debugGetContextData(PathOpsV0Lib::DebugContextType type) {
    OP_ASSERT((size_t) type < debugContextData.size());
    return debugContextData[(size_t) type];
}
#endif

bool OpContext::debugFail() const {
	return OpDebugExpect::unknown == debugExpect || OpDebugExpect::fail == debugExpect;
}

bool OpContext::debugSuccess() const {
	return true;  // !!! I suppose I should do something if it is expected to fail ?
}

#if OP_DEBUG_VALIDATE
void OpContext::debugValidateIntersections() {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			if (!segment.disabled)
				segment.sects.debugValidate();
		}
	}
}
#endif

#endif
