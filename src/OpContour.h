// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpContour_DEFINED
#define OpContour_DEFINED

#include "OpJoiner.h"
#include "OpTightBounds.h"
#include <vector>
#if OP_DEBUG
#include <atomic>
#endif

enum class EdgeMatch : int8_t;
struct FoundEdge;
struct OpContourIterator;
struct OpContourStorage;
struct OpCurveCurve;
struct OpJoiner;

enum class OpFillType {
    winding = -1,
    unset = 0,
    evenOdd = 1
};

enum class WindState {
    zero,	    // edge is outside filled area on both sides
    flipOff,	// edge moves from in to out
    flipOn,	    // edge moves from out to in
    one		    // edge is inside filled area on both sides
};

struct OpContours;
struct OpInPath;

struct OpContour {
    OpContour(OpContours* c, OpOperand op)
        : contours(c)
        , operand(op) {
#if OP_DEBUG
        debugComplete();
#endif
    }

    bool addClose();
    bool addConic(OpPoint pts[3], float weight);
    bool addCubic(OpPoint pts[4]);
    OpIntersection* addEdgeSect(const OpPtT& , OpSegment* seg
           OP_LINE_FILE_DEF(SectReason , const OpEdge* edge, const OpEdge* oEdge));
    OpEdge* addFiller(OpEdge* edge, OpEdge* lastEdge);
    OpEdge* addFiller(OpIntersection* start, OpIntersection* end);
    OpIntersection* addCoinSect(const OpPtT& , OpSegment* seg, int cID, MatchEnds 
            OP_LINE_FILE_DEF(SectReason , const OpSegment* oSeg));
    OpIntersection* addSegSect(const OpPtT& , OpSegment* seg
            OP_LINE_FILE_DEF(SectReason , const OpSegment* oSeg));
    OpIntersection* addUnsect(const OpPtT& , OpSegment* seg, int uID, MatchEnds 
            OP_LINE_FILE_DEF(SectReason , const OpSegment* oSeg));
    void addLine(OpPoint pts[2]);
    bool addQuad(OpPoint pts[3]);

    void apply() {
        for (auto& segment : segments) {
            segment.apply();
        }
    }

    void finish();

    void makeEdges() {
        for (auto& segment : segments) {
            segment.makeEdges();
        }
    }

    int nextID() const;

#if 0  // !!! disable until use case appears
    void setBounds() {
        for (auto& segment : segments) {
            ptBounds.add(segment.ptBounds);
        }
    }
#endif

    void windCoincidences() {
        for (auto& segment : segments) {
            segment.windCoincidences();
        }
    }

#if OP_DEBUG
    void debugComplete();
#endif
#if OP_DEBUG_DUMP
    OpContour() {}
    DUMP_DECLARATIONS
    #define OP_X(Thing) \
    void dump##Thing() const;
    SEGMENT_DETAIL
    EDGE_OR_SEGMENT_DETAIL
    #undef OP_X
#endif

    OpContours* contours;
    std::vector<OpSegment> segments;
    PathOpsV0Lib::ContourCallBacks callBacks;

//    OpPointBounds ptBounds;
    OpOperand operand; // first or second argument to a binary operator
#if OP_DEBUG
    int id;
#endif
};

struct OpContourStorage {
	OpContourStorage()
		: next(nullptr)
		, used(0) {
	}
#if OP_DEBUG_DUMP
	size_t debugCount() const;
	OpContour* debugFind(int id) const;
	OpContour* debugIndex(int index) const;
	static void DumpSet(const char*& , OpContours* );
	DUMP_DECLARATIONS
#endif

	OpContourStorage* next;
	OpContour storage[2];
	int used;
};

struct OpContourIter {
    OpContourIter(OpContours* contours);
    
    void moveToEnd() {
        OP_ASSERT(!contourIndex);
        while (storage->next) {
            contourIndex += storage->used;
            storage = storage->next;
        }
        contourIndex += storage->used;
    }

    bool operator!=(OpContourIter rhs) { 
		return contourIndex != rhs.contourIndex; 
	}

    OpContour* operator*() {
        OP_ASSERT(storage && contourIndex < storage->used);
        return &storage->storage[contourIndex]; 
	}

    void operator++() { 
		if (++contourIndex >= storage->used) {
            contourIndex = 0;
            storage = storage->next;
            OP_ASSERT(storage);
        }
	}

    OpContourStorage* storage;
	int contourIndex;
};

struct OpContourIterator {
    OpContourIterator(OpContours* c) 
        : cachedEnd(c)
        , contours(c) {
    }

    OpContour* back() {
        OpContourIter iter = end();
        iter.contourIndex -= 1;
        return *iter;
    }

    OpContourIter begin() {
        OpContourIter iter(contours);
        return iter;
    }

    OpContourIter end() {
        if (!cachedEnd.contourIndex)
            cachedEnd.moveToEnd();
        return cachedEnd; 
    }

	bool empty() { return !(begin() != end()); }

    OpContourIter cachedEnd;
    OpContours* contours;
};

struct SegmentIterator {
    SegmentIterator(OpContours* );
    OpSegment* next();

    OpContours* contours;
    OpContourIterator contourIterator;
    OpContourIter contourIter;
    size_t segIndex;
    OP_DEBUG_CODE(bool debugEnded);
};

struct OpContours {
    OpContours(OpInPath& left, OpInPath& right, OpOperator op);
    OpContours();
    ~OpContours();
    OpContour* addMove(OpContour* , OpOperand , const OpPoint pts[1]);
    OpContour* allocateContour();
    PathOpsV0Lib::CurveData* allocateCurveData(size_t );
    void* allocateEdge(OpEdgeStorage*& );
    OpIntersection* allocateIntersection();
    OpLimb* allocateLimb(OpTree* );
    PathOpsV0Lib::WindingData* allocateWinding(size_t );

    void apply() {
        for (auto contour : contours) {
            contour->apply();
        }
    }

    bool assemble(OpOutPath& );
    bool build(OpInPath& path, OpOperand operand);   // provided by graphics implementation

    PathOpsV0Lib::CallBacks& callBack(OpType type) {
        return callBacks[(int) type - 1];
    }

//    WindingData* copySect(const OpWinding& );  // !!! add a separate OpWindingStorage for temporary blocks?

    void finishAll();

    int leftFillTypeMask() const {
        return (int) left;
    }

    OpContour* makeContour(OpOperand operand) {
        OpContour* contour = allocateContour();
        contour->contours = this;
        contour->operand = operand;
        return contour;
    }

    void makeEdges() {
       OP_DEBUG_CODE(debugInClearEdges = true);
       for (auto contour : contours) {
            contour->makeEdges();
        }
       OP_DEBUG_CODE(debugInClearEdges = false);
    }

    bool pathOps(OpOutPath& result);
    void release(OpEdgeStorage*& );
    OpLimbStorage* resetLimbs(OpTree* tree);
    void reuse(OpEdgeStorage* );

    int rightFillTypeMask() const {
        return (int) right;
    }

#if 0  // !!! disable until use case appears
    void setBounds() {
        for (auto& contour : contours) {
            contour.setBounds();
        }
    }
#endif

    void setFillType(OpOperand operand, OpFillType exor) {
        (OpOperand::left == operand ? left : right) = exor;
    }

    void sortIntersections();

    void windCoincidences() {
        for (auto contour : contours) {
            contour->windCoincidences();
        }
    }

    // sum is accumulated fill clockwise from center tangent
// if sum is zero and edge winding is non-zero, edge 'flips' winding from zero to non-zero
// if sum is non-zero and edge winding equals sum, edge 'flips' winding from non-zero to zero
    WindState windState(int wind, int sum, OpOperand operand) {
#if OP_DEBUG
        OpFillType fillType = OpOperand::left == operand ? left : right;
        OP_ASSERT(wind == (wind & (int) fillType));
        OP_ASSERT(sum == (sum & (int) fillType));
#endif
        if (!wind)
            return sum ? WindState::one : WindState::zero;
        if (sum)
            return wind == sum ? WindState::flipOff : WindState::one;
        return WindState::flipOn;
    }

    bool debugFail() const;
#if OP_DEBUG
    void addDebugWarning(OpDebugWarning );
    void debugRemap(int oldRayMatch, int newRayMatch);
    bool debugSuccess() const;
#endif
#if OP_DEBUG_DUMP
    void debugCompare(std::string s);
    void dumpResolve(OpContour*& contourRef);
    void dumpResolve(const OpEdge*& );
    void dumpResolve(OpEdge*& );
    void dumpResolve(OpIntersection*& );
    void dumpResolve(const OpLimb*& limbRef);
    void dumpResolve(OpSegment*& );
    #include "OpDebugDeclarations.h"
#endif

    OpInPath* leftIn;
    OpInPath* rightIn;
    OpOperator opIn;
#if !OP_TEST_NEW_INTERFACE
    // this allows contours to move when vector grows. Better to allocate it ourselves so reference
    // returned to caller doesn't change if a new contour is created.
    std::vector<OpContour> contours;
#endif
    // these are pointers instead of inline values because the storage with empty slots is first
    OpEdgeStorage* ccStorage;
    CurveDataStorage* curveDataStorage;
    OpContourStorage* contourStorage;
    OpContourIterator contours;
    OpEdgeStorage* fillerStorage;
    OpSectStorage* sectStorage;
    OpLimbStorage* limbStorage;
    WindingDataStorage* windingStorage;
    OpFillType left;
    OpFillType right;
    OpOperator opOperator;
    int uniqueID;  // used for object id, unsectable id, coincidence id
    bool newInterface;

// new interface ..
    std::vector<PathOpsV0Lib::CallBacks> callBacks;
#if OP_DEBUG_VALIDATE
    int debugValidateEdgeIndex;
    int debugValidateJoinerIndex;
#endif
#if OP_DEBUG
    OpCurveCurve* debugCurveCurve;
    OpJoiner* debugJoiner;
    OpOutPath* debugResult;
    std::vector<OpDebugWarning> debugWarnings;
    std::string debugTestname;
    OpDebugExpect debugExpect;
    bool debugInPathOps;
    bool debugInClearEdges;
    bool debugCheckLastEdge;
    bool debugFailOnEqualCepts;
#endif
#if OP_DEBUG_DUMP
    OpInPath* debugLeft;
    OpInPath* debugRight;
	OpTree* dumpTree;
	int dumpCurve1;
	int dumpCurve2;
    int debugBreakDepth;
    bool debugDumpInit;   // if true, created by dump init
#endif
};

#endif