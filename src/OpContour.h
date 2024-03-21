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
    flipOff,	// edge moves either from in to out
    flipOn,	    // edge moves either from out to in
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

    void setBounds() {
        for (auto& segment : segments) {
            ptBounds.add(segment.ptBounds);
        }
    }

    void sortIntersections() {
        for (auto& segment : segments) {
            segment.sects.sort();
        }
    }

    void windCoincidences() {
        for (auto& segment : segments) {
            segment.windCoincidences();
        }
    }

#if OP_DEBUG
    void debugComplete();
#endif
#if OP_DEBUG_DUMP
    DUMP_DECLARATIONS
    #define OP_X(Thing) \
    void dump##Thing() const;
    SEGMENT_DETAIL
    EDGE_OR_SEGMENT_DETAIL
    #undef OP_X
#endif
#if OP_DEBUG_IMAGE
    void draw() const;
#endif

    OpContours* contours;
    std::vector<OpSegment> segments;
    OpPointBounds ptBounds;
    OpOperand operand; // first or second argument to a binary operator
#if OP_DEBUG
    int id;
#endif
};

struct OpContours {
    OpContours(OpInPath& left, OpInPath& right, OpOperator op);

    ~OpContours() {
        release(ccStorage);
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
#if OP_DEBUG
        debugInPathOps = false;
        debugInClearEdges = false;
#endif
    }

    OpContour* addMove(OpContour* , OpOperand , const OpPoint pts[1]);
    void* allocateEdge(OpEdgeStorage*& );
    OpIntersection* allocateIntersection();
    OpLimb* allocateLimb(OpTree& );

    void apply() {
        for (auto& contour : contours) {
            contour.apply();
        }
    }

    bool assemble(OpOutPath );
    bool build(OpInPath path, OpOperand operand);   // provided by graphics implementation

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

    bool pathOps(OpOutPath result);
    void release(OpEdgeStorage*& );
    OpLimbStorage* resetLimbs();
    void reuse(OpEdgeStorage* );

    int rightFillTypeMask() const {
        return (int) right;
    }

    void setBounds() {
        for (auto& contour : contours) {
            contour.setBounds();
        }
    }

    void setFillType(OpOperand operand, OpFillType exor) {
        (OpOperand::left == operand ? left : right) = exor;
    }

    void sortIntersections() {
        for (auto& contour : contours) {
            contour.sortIntersections();
        }
    }

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
    std::string debugDumpHex(std::string label) const;
    #include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
    void draw() const;
#endif

    OpInPath& leftIn;
    OpInPath& rightIn;
    OpOperator opIn;
    std::vector<OpContour> contours;
    OpEdgeStorage* ccStorage;
    OpEdgeStorage* fillerStorage;
    OpSectStorage* sectStorage;
    OpLimbStorage* limbStorage;
    OpFillType left;
    OpFillType right;
    OpOperator opOperator;
    int uniqueID;  // used for object id, unsectable id, coincidence id
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
};

#endif