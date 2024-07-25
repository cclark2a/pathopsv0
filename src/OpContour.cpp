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

#if !OP_TEST_NEW_INTERFACE
bool OpContour::addClose() {
    if (!segments.size())
        return false;
    OpPoint curveStart = segments.front().c.pts[0];
    OpPoint lastPoint = segments.back().c.lastPt();
    if (lastPoint != curveStart) {
        LinePts linePts = {{ lastPoint, curveStart }};
        segments.emplace_back(linePts, this  
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
    }
    return true;
}

bool OpContour::addConic(OpPoint pts[3], float weight) {
    OpMath::ZeroTiny(pts, 3);
    if (pts[0] == pts[2])   // !!! should be fill only, not frame
        return true;
    OpTightBounds bounds;
    OpConic conic(pts, weight);
    if (!bounds.calcBounds(conic))
        return false;
    std::vector<ExtremaT> extrema = bounds.findExtrema(pts[0], pts[2]);
    if (!extrema.size()) {
        OpCurve whole(pts, weight, OpType::conic);
        if (whole.isLinear()) {
            LinePts linePts = {{ whole.pts[0], whole.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else
            segments.emplace_back(whole, OpType::conic, this  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        return true;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[2], 1 }  OP_DEBUG_PARAMS(SectReason::endPt));
        OpCurve partPts = conic.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear()) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else 
            segments.emplace_back(partPts, OpType::conic, this
                    OP_DEBUG_PARAMS(start.reason, end.reason));
        start = end;
    }
    return true;
}

bool OpContour::addCubic(OpPoint pts[4]) {
    OpMath::ZeroTiny(pts, 4);
    // reduction to point if pt 0 equals pt 3 complicated since it requires pts 1, 2 be linear..
    if (pts[0] == pts[3]) { // !!! detect possible degenerate to code from actual test data
        if (pts[1] == pts[0] && pts[2] == pts[0])
            return true;
    }
    OpTightBounds bounds;
    OpCubic cubic(pts);
    if (!bounds.calcBounds(cubic))
        return false;
    std::vector<ExtremaT> extrema = bounds.findExtrema(pts[0], pts[3]);

    if (!extrema.size()) {
        OpCurve whole(pts, OpType::cubic);
        if (whole.isLinear()) {
            LinePts linePts = {{ whole.pts[0], whole.pts[3] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this  
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else
            segments.emplace_back(whole, OpType::cubic, this  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        return true;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[3], 1 }  OP_DEBUG_PARAMS(SectReason::endPt));
        OpCurve partPts = cubic.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear()) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[3] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this  OP_DEBUG_PARAMS(start.reason, end.reason));
        } else
            segments.emplace_back(partPts, OpType::cubic, this  
                    OP_DEBUG_PARAMS(start.reason, end.reason));
        start = end;
    }
    return true;
}
#endif

OpIntersection* OpContour::addEdgeSect(const OpPtT& t, OpSegment* seg  
        OP_LINE_FILE_DEF(SectReason reason, const OpEdge* edge, const OpEdge* oEdge)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg  OP_LINE_FILE_CALLER(reason, edge->id, oEdge->id));
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
            OP_LINE_FILE_PARAMS(EdgeMaker::filler, start, end));
    OP_DEBUG_CODE(filler->debugFiller = true);
    filler->setDisabled(OP_DEBUG_CODE(ZeroReason::filler));
    return filler;
}

#if !OP_TEST_NEW_INTERFACE
void OpContour::addLine(OpPoint pts[2]) {
    OpMath::ZeroTiny(pts, 2);
    if (pts[0] == pts[1])   // !!! should be fill only, not frame
        return;
    LinePts linePts = {{ pts[0], pts[1] }};
    segments.emplace_back(linePts, this  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
}
#endif

OpIntersection* OpContour::addCoinSect(const OpPtT& t, OpSegment* seg, int cID, MatchEnds coinEnd
        OP_LINE_FILE_DEF(SectReason reason, const OpSegment* oSeg)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg  OP_LINE_FILE_CALLER(reason, seg->id, oSeg->id));
	next->coincidenceID = cID;  // 0 if no coincidence; negative if coincident pairs are reversed
	OP_ASSERT(MatchEnds::both != coinEnd);
	next->coinEnd = coinEnd;
    return next;
}

OpIntersection* OpContour::addSegSect(const OpPtT& t, OpSegment* seg  
        OP_LINE_FILE_DEF(SectReason reason, const OpSegment* oSeg)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg  OP_LINE_FILE_CALLER(reason, seg->id, oSeg->id));
    return next;
}

OpIntersection* OpContour::addUnsect(const OpPtT& t, OpSegment* seg, int uID, MatchEnds unsectEnd
        OP_LINE_FILE_DEF(SectReason reason, const OpSegment* oSeg)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg  OP_LINE_FILE_CALLER(reason, seg->id, oSeg->id));
	next->unsectID = uID;
	OP_ASSERT(MatchEnds::both != unsectEnd);
	next->unsectEnd = unsectEnd;
    return next;
}

#if !OP_TEST_NEW_INTERFACE
bool OpContour::addQuad(OpPoint pts[3]) {
    OpMath::ZeroTiny(pts, 3);
    if (pts[0] == pts[2])   // !!! should be fill only, not frame
        return true;
    OpTightBounds bounds;
    OpQuad quad(pts);
    if (!bounds.calcBounds(quad))
        return false;
    std::vector<ExtremaT> extrema = bounds.findExtrema(pts[0], pts[2]);
    if (!extrema.size()) {
        OpCurve whole(pts, OpType::quad);
        if (whole.isLinear()) {
            LinePts linePts = {{ whole.pts[0], whole.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this  
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else
            segments.emplace_back(whole, OpType::quad, this  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        return true;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[2], 1 }  OP_DEBUG_PARAMS(SectReason::startPt));
        OpCurve partPts = quad.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear()) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this  
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else 
            segments.emplace_back(partPts, OpType::quad, this  
                    OP_DEBUG_PARAMS(start.reason, end.reason));
        start = end;
    }
    return true;
}
#endif

int OpContour::nextID() const {
//    if (93 == contours->uniqueID + 1)
//        OpDebugOut("");
    return ++contours->uniqueID;
}

// end of contour; start of contours

#if !OP_TEST_NEW_INTERFACE
static const OpOperator OpInverse[+OpOperator::ReverseSubtract + 1][2][2] {
    //                inside minuend                                  outside minuend
    //  inside subtrahend     outside subtrahend         inside subtrahend   outside subtrahend
    { { OpOperator::Subtract, OpOperator::Intersect }, { OpOperator::Union, OpOperator::ReverseSubtract } },
    { { OpOperator::Intersect, OpOperator::Subtract }, { OpOperator::ReverseSubtract, OpOperator::Union } },
    { { OpOperator::Union, OpOperator::ReverseSubtract }, { OpOperator::Subtract, OpOperator::Intersect } },
    { { OpOperator::ExclusiveOr, OpOperator::ExclusiveOr }, { OpOperator::ExclusiveOr, OpOperator::ExclusiveOr } },
    { { OpOperator::ReverseSubtract, OpOperator::Union }, { OpOperator::Intersect, OpOperator::Subtract } },
};
#endif

OpContours::OpContours()
    : 
#if !OP_TEST_NEW_INTERFACE
     leftIn(nullptr)
    , rightIn(nullptr)
    , opIn(OpOperator::Intersect)
    , 
#endif
      ccStorage(nullptr)
    , curveDataStorage(nullptr)
    , contourStorage(nullptr)
    , contours(this)
    , fillerStorage(nullptr)
    , sectStorage(nullptr)
    , limbStorage(nullptr)
    , callerStorage(nullptr)
#if !OP_TEST_NEW_INTERFACE
    , left(OpFillType::unset)
    , right(OpFillType::unset)
#endif
    , uniqueID(0) 
    , caller({nullptr, 0}) {
#if OP_DEBUG_VALIDATE
    debugValidateEdgeIndex = 0;
    debugValidateJoinerIndex = 0;
#endif
#if OP_DEBUG
    debugCurveCurve = nullptr;
    debugJoiner = nullptr;
#if !OP_TEST_NEW_INTERFACE
    debugResult = nullptr;
#endif
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

OpContours::OpContours(OpInPath& l, OpInPath& r, OpOperator op) 
    : OpContours() {
#if !OP_TEST_NEW_INTERFACE
    leftIn = &l;
    rightIn = &r;
    opIn = op;
    opOperator = OpInverse[+op][l.isInverted()][r.isInverted()];
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
#if !OP_TEST_NEW_INTERFACE
        delete debugLeft;
        delete debugRight;
        delete debugResult;
#endif
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

#if !OP_TEST_NEW_INTERFACE
OpContour* OpContours::addMove(OpContour* last, OpOperand operand , const OpPoint pts[1]) {
    // !!! if frame paths are supported, don't add close unless fill is set
    if (last->addClose())
        return makeContour(operand);
    // keep point as its own segment in the future?
    return contours.back();
}
#endif

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
            seg->setDisabled(OP_DEBUG_CODE(ZeroReason::isPoint));
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
#if OP_TEST_NEW_INTERFACE
bool OpContours::assemble() 
#else
bool OpContours::assemble(OpOutPath& path) 
#endif
{
#if OP_TEST_NEW_INTERFACE
    OpJoiner joiner(*this);
#else
    OpJoiner joiner(*this, path);  // collect active edges and sort them
#endif
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

#if !OP_TEST_NEW_INTERFACE
static const bool OutInverse[+OpOperator::ReverseSubtract + 1][2][2] {
    { { false, false }, { true, false } },  // diff
    { { false, false }, { false, true } },  // sect
    { { false, true }, { true, true } },    // union
    { { false, true }, { true, false } },   // xor
    { { false, true }, { false, false } },  // rev diff
};
#endif

// If successive runs of the same input are flaky, check to see if identical ids are generated.
// To do this, insert OP_DEBUG_COUNT(*this, _some_identifer_); after every callout.  
// This will compare the dumps of contours and contents to detect when something changed.
// The callouts are removed when not in use as they are not maintained and reduce readability.
// !!! OP_DEBUG_COUNT was unintentionally deleted at some point. Hopefully it is in git history...
#if OP_TEST_NEW_INTERFACE
bool OpContours::pathOps()
#else
bool OpContours::pathOps(OpOutPath& result)
#endif
{
#if !OP_TEST_NEW_INTERFACE
    if (!build(*leftIn, OpOperand::left))  // builds monotonic segments, and adds 0/1 sects
        OP_DEBUG_FAIL(*this, false);
    if (!build(*rightIn, OpOperand::right))
        OP_DEBUG_FAIL(*this, false);
//  finishAll();  // !!! no longer needed
//  setBounds();    // !!! check to see if this is used
    OpSegments sortedSegments(*this);
    if (!sortedSegments.inX.size()) {
        result.setEmpty();
        OP_DEBUG_SUCCESS(*this, true);
    }
    sortedSegments.findCoincidences();  // check for exact curves and full lines
    if (FoundIntersections::fail == sortedSegments.findIntersections())
        return false;  // triggered by fuzzhang_1
    if (contours.empty())
        OP_DEBUG_SUCCESS(*this, true);
#endif
    {
        OpSegments::FindCoincidences(this);
#if OP_DEBUG_VALIDATE
        debugValidateIntersections();
#endif
        OpSegments sortedSegments(*this);
#if OP_DEBUG_VALIDATE
        debugValidateIntersections();
#endif
        if (!sortedSegments.inX.size()) {
            contextCallBacks.emptyNativePath(callerOutput);
            OP_DEBUG_SUCCESS(*this, true);
        }
        if (FoundIntersections::fail == sortedSegments.findIntersections())
            return false;  // !!! fix this to record for Error()
#if OP_DEBUG_VALIDATE
        debugValidateIntersections();
#endif
        disableSmallSegments();  // moved points may allow disabling some segments
        if (empty()) {
            contextCallBacks.emptyNativePath(callerOutput);
            OP_DEBUG_SUCCESS(*this, true);
        }
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
#if OP_TEST_NEW_INTERFACE
    if (!assemble())
        OP_DEBUG_FAIL(*this, false);
#else
    if (!assemble(result))
        OP_DEBUG_FAIL(*this, false);
    bool inverseFill = OutInverse[+opOperator][leftIn->isInverted()][rightIn->isInverted()];
    result.setInverted(inverseFill);
#endif
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

