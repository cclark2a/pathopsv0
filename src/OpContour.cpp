// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
#include "PathOps.h"

char* CallerDataStorage::Allocate(size_t size, CallerDataStorage** callerStoragePtr) {
    if (!*callerStoragePtr)
        *callerStoragePtr = new CallerDataStorage;
    CallerDataStorage* callerStorage = *callerStoragePtr;
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

void OpContour::addCallerData(PathOpsV0Lib::AddContour data) {
    caller.data = CallerDataStorage::Allocate(data.size, &contours->callerStorage);
    std::memcpy(caller.data, data.data, data.size);
    caller.size = data.size;  // !!! don't know if size is really needed ...
}

void OpContours::addCallerData(PathOpsV0Lib::AddContext data) {
    caller.data = CallerDataStorage::Allocate(data.size, &callerStorage);
    std::memcpy(caller.data, data.data, data.size);
    caller.size = data.size;  // !!! don't know if size is really needed ...
}

OpIntersection* OpContour::addEdgeSect(const OpPtT& t, OpSegment* seg  
        OP_LINE_FILE_DEF(const OpEdge* edge, const OpEdge* oEdge)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg  OP_LINE_FILE_CALLER(edge->id, oEdge->id));
    return next;
}

OpEdge* OpContour::addFiller(OpEdge* edge, OpEdge* lastEdge) {
	// break this off into its own callable function
	// as a last, last resort: do this rather than returning false
	OpContour* contour = edge->segment->contour;
	OpIntersection* sect = nullptr;
	// !!! is there an edge method that does this?
	for (auto test : edge->segment->sects.i) {
		if (test->ptT != edge->whichPtT())
			continue;
		sect = test;
		break;
	}
	OP_ASSERT(sect);
	OpIntersection* last = nullptr;
	for (auto test : lastEdge->segment->sects.i) {
		if (test->ptT != lastEdge->whichPtT(EdgeMatch::end))
			continue;
		last = test;
		break;
	}
	OP_ASSERT(last);
	return contour->addFiller(last, sect);
}

OpEdge* OpContour::addFiller(OpIntersection* start, OpIntersection* end) {
    if (contours->fillerStorage && contours->fillerStorage->contains(start, end))
        return nullptr;  // !!! when does this happen? what is the implication? e.g. fuzz433
    void* block = contours->allocateEdge(contours->fillerStorage);
    OpEdge* filler = new(block) OpEdge(start->segment, start->ptT, end->ptT
            OP_LINE_FILE_PARAMS(start, end));
    OP_DEBUG_CODE(filler->debugFiller = true);
    filler->setDisabled(OP_LINE_FILE_NPARAMS());
    return filler;
}

OpIntersection* OpContour::addCoinSect(const OpPtT& t, OpSegment* seg, int cID, MatchEnds coinEnd
        OP_LINE_FILE_DEF(const OpSegment* oSeg)) {
	OP_ASSERT(MatchEnds::both != coinEnd);
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg  OP_LINE_FILE_CALLER(seg->id, oSeg->id));
	next->setCoin(cID, coinEnd);  // 0 if no coincidence; negative if coincident pairs are reversed
    return next;
}

OpIntersection* OpContour::addSegSect(const OpPtT& t, OpSegment* seg  
        OP_LINE_FILE_DEF(const OpSegment* oSeg)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg  OP_LINE_FILE_CALLER(seg->id, oSeg->id));
    return next;
}

OpIntersection* OpContour::addUnsect(const OpPtT& t, OpSegment* seg, int uID, MatchEnds unsectEnd
        OP_LINE_FILE_DEF(const OpSegment* oSeg)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg  OP_LINE_FILE_CALLER(seg->id, oSeg->id));
	OP_ASSERT(MatchEnds::both != unsectEnd);
	next->setUnsect(uID, unsectEnd);
    return next;
}

int OpContour::nextID() const {
//    if (93 == contours->uniqueID + 1)
//        OpDebugOut("");
    return ++contours->uniqueID;
}

// end of contour; start of contours

OpContours::OpContours()
    : caller({nullptr, 0}) 
    , ccStorage(nullptr)
    , curveDataStorage(nullptr)
    , contourStorage(nullptr)
    , contours(this)
    , fillerStorage(nullptr)
    , sectStorage(nullptr)
    , limbStorage(nullptr)
    , callerStorage(nullptr)
    , uniqueID(0) {
#if OP_DEBUG_VALIDATE
    debugValidateEdgeIndex = 0;
    debugValidateJoinerIndex = 0;
#endif
#if OP_DEBUG
    debugCurveCurve = nullptr;
    debugJoiner = nullptr;
    debugExpect = OpDebugExpect::unknown;
    debugInPathOps = false;
    debugInClearEdges = false;
    debugCheckLastEdge = false;
    debugFailOnEqualCepts = false;
    OP_DEBUG_DUMP_CODE(debugDumpInit = false);
#endif
#if OP_DEBUG_DUMP
    dumpTree = nullptr;
#endif
}

OpContours::~OpContours() {
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

void OpContours::addAlias(OpPoint pt, OpPoint alias) {
    for (OpPtAlias& a : aliases) {
        if (a.pt == pt && a.alias == alias)
            return;
    }
    aliases.push_back({pt, alias});
}

OpContour* OpContours::allocateContour() {
    if (!contourStorage)
        contourStorage = new OpContourStorage;
    if (contourStorage->used == ARRAY_COUNT(contourStorage->storage)) {
        OpContourStorage* next = new OpContourStorage;
        next->next = contourStorage;
        contourStorage = next;
    }
    return &contourStorage->storage[contourStorage->used++];
}

OpEdge* OpContours::allocateEdge(OpEdgeStorage*& edgeStorage) {
    if (!edgeStorage)
        edgeStorage = new OpEdgeStorage;
    if (edgeStorage->used == ARRAY_COUNT(edgeStorage->storage)) {
        OpEdgeStorage* next = new OpEdgeStorage;
        OP_ASSERT(!next->next);
        next->next = edgeStorage;
        edgeStorage = next;
    }
    return &edgeStorage->storage[edgeStorage->used++];
}

PathOpsV0Lib::CurveData* OpContours::allocateCurveData(size_t size) {
    if (!curveDataStorage)
        curveDataStorage = new CurveDataStorage;
    if (curveDataStorage->used + size > sizeof(curveDataStorage->storage)) {
        CurveDataStorage* next = new CurveDataStorage;
        next->next = curveDataStorage;
        curveDataStorage = next;
    }
    return curveDataStorage->curveData(size);
}

OpIntersection* OpContours::allocateIntersection() {
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

OpLimb* OpContours::allocateLimb(OpTree* tree) {
    OP_DEBUG_DUMP_CODE(dumpTree = tree);
    if (limbStorage->used == ARRAY_COUNT(limbStorage->storage)) {
        OpLimbStorage* next = new OpLimbStorage;
        next->nextBlock = limbStorage;
        limbStorage->prevBlock = next;
        limbStorage = next;
    }
    return limbStorage->allocate(*tree);
}

PathOpsV0Lib::WindingData* OpContours::allocateWinding(size_t size) {
    void* result = CallerDataStorage::Allocate(size, &callerStorage);
    return (PathOpsV0Lib::WindingData*) result;
}

void OpContours::disableSmallSegments() {
    SegmentIterator segIterator(this);
    while (OpSegment* seg = segIterator.next()) {
        if (seg->disabled)
            continue;
        if (aliases.end() != std::find_if(aliases.begin(), aliases.end(), 
                [seg](OpPtAlias& a) {
            return (a.pt == seg->c.c.data->start || a.alias == seg->c.c.data->start)
                    && (a.pt == seg->c.c.data->end || a.alias == seg->c.c.data->end);
        })) {
            seg->setDisabled(OP_LINE_FILE_NPARAMS());
        }
    }
}

OpLimbStorage* OpContours::resetLimbs(OP_DEBUG_DUMP_CODE(OpTree* tree)) {
    OP_DEBUG_DUMP_CODE(dumpTree = tree);
    if (!limbStorage)
        limbStorage = new OpLimbStorage;
    limbStorage->reset();
    return limbStorage;
}

// build list of linked edges
// if they are closed, done
// if not, match up remainder
// make sure normals point same way
// prefer smaller assembled contours
// returns true on success
bool OpContours::assemble() {
    OpJoiner joiner(*this);
    if (joiner.setup())
        return true;
    for (LinkPass linkPass : { LinkPass::normal, LinkPass::unsectable } ) {
        joiner.linkUnambiguous(linkPass);
        if (joiner.linkRemaining(OP_DEBUG_CODE(this)))
            return true;
    }
    return false;
}

bool OpContours::debugFail() const {
#if OP_DEBUG
    return OpDebugExpect::unknown == debugExpect || OpDebugExpect::fail == debugExpect;
#else
    return false;
#endif
}

// If successive runs of the same input are flaky, check to see if identical ids are generated.
// To do this, insert OP_DEBUG_COUNT(*this, _some_identifer_); after every callout.  
// This will compare the dumps of contours and contents to detect when something changed.
// The callouts are removed when not in use as they are not maintained and reduce readability.
// !!! OP_DEBUG_COUNT was unintentionally deleted at some point. Hopefully it is in git history...
bool OpContours::pathOps() {
    OpSegments::FindCoincidences(this);
#if OP_DEBUG_VALIDATE
    debugValidateIntersections();
#endif
    OpSegments sortedSegments(*this);
#if OP_DEBUG_VALIDATE
    debugValidateIntersections();
#endif
    if (!sortedSegments.inX.size()) {
        contextCallBacks.emptyNativePathFuncPtr(callerOutput);
        OP_DEBUG_SUCCESS(*this, true);
    }
    if (FoundIntersections::fail == sortedSegments.findIntersections())
        return false;  // !!! fix this to record for Error()
#if OP_DEBUG_VALIDATE
    debugValidateIntersections();
#endif
    disableSmallSegments();  // moved points may allow disabling some segments
    if (empty()) {
        contextCallBacks.emptyNativePathFuncPtr(callerOutput);
        OP_DEBUG_SUCCESS(*this, true);
    }
    sortIntersections();
    makeEdges();

    // made edges may include lines that are coincident with other edges. Undetected for now...
    windCoincidences();  // for segment h/v lines, compute their winding considering coincidence
    OpWinder windingEdges(*this, EdgesToSort::byCenter);
    FoundWindings foundWindings = windingEdges.setWindings(this);  // walk edges, compute windings
    if (FoundWindings::fail == foundWindings)
        OP_DEBUG_FAIL(*this, false);
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
#if 0 && OP_DEBUG_IMAGE
    showResult();
#endif
    OP_DEBUG_SUCCESS(*this, true);
}

void OpContours::release(OpEdgeStorage*& edgeStorage) {
    while (edgeStorage) {
        OpEdgeStorage* next = edgeStorage->next;
        delete edgeStorage;
        edgeStorage = next;
    }
}

void OpContours::reuse(OpEdgeStorage* edgeStorage) {
    OpEdgeStorage* next = edgeStorage;
    while (next) {
        next->used = 0;
        next = next->next;
    }
}

void OpContours::sortIntersections() {
    for (auto contour : contours) {
        for (auto& segment : contour->segments) {
            segment.sects.sort();
        }
    }
    for (auto contour : contours) {
        for (auto& segment : contour->segments) {
            segment.sects.mergeNear();
        }
    }
}

#if OP_DEBUG
void OpContour::debugComplete() {
    caller.data = nullptr;   // should always get initialized by OpContours::addCallerData
    caller.size = 0;
    id = nextID();
}

bool OpContours::debugSuccess() const {
    return OpDebugExpect::unknown == debugExpect || OpDebugExpect::success == debugExpect;
}
#endif

#if OP_DEBUG_VALIDATE
void OpContours::debugValidateIntersections() {
    for (auto contour : contours) {
        for (auto& segment : contour->segments) {
            segment.sects.debugValidate();
        }
    }
}
#endif

SegmentIterator::SegmentIterator(OpContours* c)
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

OpContourIter::OpContourIter(OpContours* contours) {
    storage = contours->contourStorage;
	contourIndex = 0;
}

