#ifndef OpContour_DEFINED
#define OpContour_DEFINED

#include "OpSegment.h"
#include "OpTightBounds.h"
#include <vector>

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
    void addConic(const OpPoint pts[3], float weight);
    void addCubic(const OpPoint pts[4]);
    OpIntersection* addEdgeSect(const OpPtT& t, OpSegment* seg
            OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
            const OpEdge* edge, const OpEdge* oEdge));
    OpIntersection* addSegSect(const OpPtT& t, OpSegment* seg, SectFlavor , int cID
            OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
            const OpSegment* oSeg));
    void addLine(const OpPoint pts[2]);
    OpContour* addMove(const OpPoint pts[1]);
    void addQuad(const OpPoint pts[3]);

    void apply() {
        for (auto& segment : segments) {
            segment.apply();
        }
    }

#if 0
    void calcBounds() {
        for (auto& segment : segments) {
            segment.calcBounds();
        }
    }
#endif

    void finish();

#if 0
    void intersectEdge() {
        for (auto& segment : segments) {
            segment.intersectEdge();
        }
    }
#endif

    void makeEdges() {
        for (auto& segment : segments) {
            segment.makeEdges();
        }
    }

#if 0
    void missingCoincidence() {
        for (auto& segment : segments) {
            segment.missingCoincidence();
        }
    }
#endif

#if 0
    bool resolveCoincidence() {
        for (auto& segment : segments) {
            // need watchdog here so it doesn't loop forever
            int watchDog = 1000;   // !!! not sure how to compute maximum possible loopage
            while (!segment.resolveCoincidence())
                if (!--watchDog)
                    return false;
        }
        return true;
    }
#endif

#if 0
    void resolvePoints() {
        for (auto& segment : segments) {
            if (!segment.intersections.size())  // size may be zero if coincident
                continue;
            segment.resolvePoints();
        }
    }
#endif

    void setBounds() {
        for (auto& segment : segments) {
            ptBounds.add(segment.ptBounds);
        }
    }

    void sortIntersections() {
        for (auto& segment : segments) {
            segment.sortIntersections();
        }
    }

#if OP_DEBUG
    void debugComplete();
#endif
#if OP_DEBUG_DUMP
    std::string debugDump() const;
    void dump() const;
    void dumpDetail() const;
    void dumpFull() const;
    void dumpHex() const;
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
        do {
            OpSectStorage* next = sectStorage->next;
            delete sectStorage;
            sectStorage = next;
        } while (sectStorage);
#if OP_DEBUG
        debugInPathOps = false;
        debugInClearEdges = false;
#endif
    }

    OpIntersection* allocateIntersection();

    void apply() {
        for (auto& contour : contours) {
            contour.apply();
        }
    }

    bool build(OpInPath path, OpOperand operand);   // provided by graphics implementation

#if 0
    void calcBounds() {
        for (auto& contour : contours) {
            contour.calcBounds();
        }
    }
#endif

    bool closeGap(OpEdge* last, OpEdge* first);
    void finishAll();

#if 0
    void intersectEdge() {
        for (auto& contour : contours) {
            contour.intersectEdge();
        }
    }
#endif

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

#if 0
    void missingCoincidence() {
        for (auto& contour : contours) {
            contour.missingCoincidence();
        }
    }
#endif

#if 0
    bool resolveCoincidence() {
        for (auto& contour : contours) {
            if (!contour.resolveCoincidence())
                return false;
        }
        return true;
    }
#endif

#if 0
    void resolvePoints() {
        for (auto& contour : contours) {
            contour.resolvePoints();
        }
    }
#endif

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

#if OP_DEBUG
    bool debugFail() const;
    bool debugSuccess() const;
#endif
#if OP_DEBUG_DUMP
    void debugCompare(std::string s) const;
    std::string debugDumpHex(std::string label) const;
    void dumpCount(std::string label) const;
    DEBUG_COMMON_DECLARATIONS();
    DUMP_COMMON_DECLARATIONS();
    FIND_COMMON_DECLARATIONS(const, const;)
#endif
#if OP_DEBUG_IMAGE
    void draw() const;
#endif
    std::vector<OpContour> contours;
    std::vector<OpEdge*> unsortables;
    OpSectStorage* sectStorage;
    OpFillType left;
    OpFillType right;
    OpOperator _operator;
    int coincidenceID = 0; // not debug since it is required for coin disambiguation
    int id = 0;
#if OP_DEBUG
    bool debugInPathOps;
    bool debugInClearEdges;
    OpDebugExpect debugExpect;
#endif
};

#endif