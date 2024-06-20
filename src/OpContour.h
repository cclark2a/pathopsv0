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

struct SegmentIterator {
    SegmentIterator(OpContours* );
    OpSegment* next();

    OpContours* contours;
    size_t contourIndex;
    size_t segIndex;
};

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

struct OpContours {
    OpContours(OpInPath& left, OpInPath& right, OpOperator op);
    OpContours();
    ~OpContours();
    OpContour* addMove(OpContour* , OpOperand , const OpPoint pts[1]);
    PathOpsV0Lib::CurveData* allocateCurveData(size_t );
    void* allocateEdge(OpEdgeStorage*& );
    OpIntersection* allocateIntersection();
    OpLimb* allocateLimb(OpTree* );
    WindingData* allocateWinding(size_t );

    void apply() {
        for (auto& contour : contours) {
            contour.apply();
        }
    }

    bool assemble(OpOutPath& );
    bool build(OpInPath& path, OpOperand operand);   // provided by graphics implementation

    PathOpsV0Lib::CallBacks& callBack(OpType type) {
        return callBacks[(int) type - 1];
    }

    WindingData* copySect(const OpEdge* , const OpWinding& );  // !!! add a separate OpWindingStorage for temporary blocks?

    void finishAll();

    int leftFillTypeMask() const {
        return (int) left;
    }

    OpContour* makeContour(OpOperand operand) {
        contours.emplace_back(this, operand);
        return &contours.back();
    }

    void makeEdges() {
       OP_DEBUG_CODE(debugInClearEdges = true);
       for (auto& contour : contours) {
            contour.makeEdges();
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
        for (auto& contour : contours) {
            contour.windCoincidences();
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
    void debugRemap(int oldRayMatch, int newRayMatch) const;
    bool debugSuccess() const;
#endif
#if OP_DEBUG_DUMP
    void debugCompare(std::string s) const;
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
    std::vector<OpContour> contours;
    // !!! should some of these be by value to avoid the initial allocation?
    OpEdgeStorage* ccStorage;
    CurveDataStorage* curveDataStorage;
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