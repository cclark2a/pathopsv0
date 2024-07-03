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

struct OpContours;
struct OpInPath;

struct CallerDataStorage {
	CallerDataStorage()
		: next(nullptr)
		, used(0) {
	}

	static void* Allocate(size_t size, CallerDataStorage** );
#if OP_DEBUG_DUMP
	size_t debugCount() const;
	std::string debugDump(std::string label, DebugLevel l, DebugBase b) const;
	uint8_t* debugIndex(int index) const;
	static void DumpSet(const char*& str, OpContours* , DumpStorage );
	DUMP_DECLARATIONS
#endif

	CallerDataStorage* next;
	int used;
	uint8_t callerData[2048];	// !!! size is arbitrary guess -- should measure and do better
};

struct OpContour {
#if !OP_TEST_NEW_INTERFACE
    OpContour(OpContours* c, OpOperand op)
        : contours(c)
        , operand(op) {
#if OP_DEBUG
        debugComplete();
#endif
    }
#endif

    void addCallerData(PathOpsV0Lib::AddContour callerData);
#if !OP_TEST_NEW_INTERFACE
    bool addClose();
    bool addConic(OpPoint pts[3], float weight);
    bool addCubic(OpPoint pts[4]);
#endif
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
#if !OP_TEST_NEW_INTERFACE
    OpContour() {}
#endif
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
#if OP_TEST_NEW_INTERFACE
    PathOpsV0Lib::AddContour caller;
#else
    OpOperand operand; // first or second argument to a binary operator
#endif
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
    OpContourIter()
        : storage(nullptr)
        , contourIndex(0) {
    }

    OpContourIter(OpContours* contours);
    
    bool operator!=(OpContourIter rhs) { 
		return storage != rhs.storage || contourIndex != rhs.contourIndex; 
	}

    OpContour* operator*() {
        OP_ASSERT(storage && contourIndex < storage->used);
        return &storage->storage[contourIndex]; 
	}

    void operator++() {
        OP_ASSERT(storage && contourIndex < storage->used); 
		if (++contourIndex >= storage->used) {
            contourIndex = 0;
            storage = storage->next;
        }
	}

    void back() {
        OP_ASSERT(storage);
        while (OpContourStorage* next = storage->next) {
            storage = next;
        }
        OP_ASSERT(storage->used);
        contourIndex = storage->used - 1;
    }

    OpContourStorage* storage;
	int contourIndex;
};

struct OpContourIterator {
    OpContourIterator(OpContours* c) 
        : contours(c) {
    }

    OpContour* back() {
        OpContourIter iter;
        iter.back();
        return *iter;
    }

    OpContourIter begin() {
        return OpContourIter(contours);
    }

    OpContourIter end() {
        return OpContourIter(); 
    }

	bool empty() { return !(begin() != end()); }

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
    void addCallerData(PathOpsV0Lib::AddContext callerData);
#if !OP_TEST_NEW_INTERFACE
    OpContour* addMove(OpContour* , OpOperand , const OpPoint pts[1]);
#endif
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
#if !OP_TEST_NEW_INTERFACE
    bool build(OpInPath& path, OpOperand operand);   // provided by graphics implementation
#endif

    PathOpsV0Lib::CurveCallBacks& callBack(OpType type) {
        return callBacks[(int) type - 1];
    }

//    WindingData* copySect(const OpWinding& );  // !!! add a separate OpWindingStorage for temporary blocks?

#if !OP_TEST_NEW_INTERFACE
    int leftFillTypeMask() const {
        return (int) left;
    }
#endif

#if OP_TEST_NEW_INTERFACE
    OpContour* makeContour() {
        OpContour* contour = allocateContour();
        contour->contours = this;
        OP_DEBUG_CODE(contour->debugComplete());
        return contour;
    }
#else
    OpContour* makeContour(OpOperand operand) {
        OpContour* contour = allocateContour();
        contour->contours = this;
        contour->operand = operand;
        OP_DEBUG_CODE(contour->debugComplete());
        return contour;
    }
#endif

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

#if !OP_TEST_NEW_INTERFACE
    int rightFillTypeMask() const {
        return (int) right;
    }
#endif

#if 0  // !!! disable until use case appears
    void setBounds() {
        for (auto& contour : contours) {
            contour.setBounds();
        }
    }
#endif

#if !OP_TEST_NEW_INTERFACE
    void setFillType(OpOperand operand, OpFillType exor) {
        (OpOperand::left == operand ? left : right) = exor;
    }
#endif

    void sortIntersections();

    void windCoincidences() {
        for (auto contour : contours) {
            contour->windCoincidences();
        }
    }

#if !OP_TEST_NEW_INTERFACE
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
#endif

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

#if !OP_TEST_NEW_INTERFACE
    OpInPath* leftIn;
    OpInPath* rightIn;
    OpOperator opIn;
#endif
    // these are pointers instead of inline values because the storage with empty slots is first
    OpEdgeStorage* ccStorage;
    CurveDataStorage* curveDataStorage;
    OpContourStorage* contourStorage;
    OpContourIterator contours;
    OpEdgeStorage* fillerStorage;
    OpSectStorage* sectStorage;
    OpLimbStorage* limbStorage;
    CallerDataStorage* callerStorage;
#if !OP_TEST_NEW_INTERFACE
    OpFillType left;
    OpFillType right;
    OpOperator opOperator;
#endif
    int uniqueID;  // used for object id, unsectable id, coincidence id
    bool newInterface;

// new interface ..
    std::vector<PathOpsV0Lib::CurveCallBacks> callBacks;
//    PathOpsV0Lib::ContextCallBacks contextCallBacks;
    PathOpsV0Lib::PathOutput callerOutput;
    PathOpsV0Lib::AddContext caller;
#if OP_DEBUG_VALIDATE
    int debugValidateEdgeIndex;
    int debugValidateJoinerIndex;
#endif
#if OP_DEBUG
    OpCurveCurve* debugCurveCurve;
    OpJoiner* debugJoiner;
#if !OP_TEST_NEW_INTERFACE
    OpOutPath* debugResult;
#endif
    std::vector<OpDebugWarning> debugWarnings;
    std::string debugTestname;
    OpDebugExpect debugExpect;
    bool debugInPathOps;
    bool debugInClearEdges;
    bool debugCheckLastEdge;
    bool debugFailOnEqualCepts;
#endif
#if OP_DEBUG_DUMP
#if !OP_TEST_NEW_INTERFACE
    OpInPath* debugLeft;
    OpInPath* debugRight;
#endif
	OpTree* dumpTree;
	int dumpCurve1;
	int dumpCurve2;
    int debugBreakDepth;
    bool debugDumpInit;   // if true, created by dump init
#endif
};

#endif