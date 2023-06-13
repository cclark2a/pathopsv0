#ifndef OpContour_DEFINED
#define OpContour_DEFINED

#include "OpSegment.h"
#include "OpSegmentBuilder.h"
#include "OpTightBounds.h"
#include <list>
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

struct OpContour {
    OpContour(OpContours* c, OpOperand op)
        : contours(c)
        , operand(op) {
#if OP_DEBUG
        debugComplete();
#endif
    }

    void addClose(OpPoint pt1, OpPoint pt2) {
        CurvePts curvePts = {{ pt1, pt2 }, 1 };
        segments.emplace_back(curvePts, lineType, this
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
    }

    void addConic(const OpPoint pts[3], float weight) {
        if (pts[0] == pts[3])
            return addMove(pts);
        OpSegmentBuilder::AddConic(this, pts, weight);
    }

    void addCubic(const OpPoint pts[4]) {
        // reduction to point if pt 0 equals pt 3 complicated since it requires pts 1, 2 be linear..
        assert(pts[0] != pts[3]); // !!! detect possible degenerate to code from actual test data
        OpSegmentBuilder::AddCubic(this, pts);
    }

    OpIntersection* addIntersection(const OpPtT& t, OpSegment* seg, SectFlavor , int cID
            OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
            const OpIntersection* master, const OpEdge* edge, const OpEdge* oEdge));

    OpIntersection* addIntersection(const OpPtT& t, OpSegment* seg, SectFlavor , int cID
            OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
            const OpSegment* dSeg, const OpSegment* oSeg));

    void addLine(const OpPoint pts[2]) {
        if (pts[0] == pts[1])
            return addMove(pts);
        CurvePts curvePts = {{ pts[0], pts[1] }, 1 };
        segments.emplace_back(curvePts, lineType, this
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
    }

    void addMove(const OpPoint pts[1]) {
        CurvePts curvePts = {{ pts[0] }, 1 };
        segments.emplace_back(curvePts, pointType, this
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
    }

    void addQuad(const OpPoint pts[3]) {
        if (pts[0] == pts[2])
            return addMove(pts);
        OpSegmentBuilder::AddQuad(this, pts);
    }

    void apply() {
        for (auto& segment : segments) {
            segment.apply();
        }
    }

    void calcBounds() {
        for (auto& segment : segments) {
            segment.calcBounds();
        }
    }

    void intersectEdge() {
        for (auto& segment : segments) {
            segment.intersectEdge();
        }
    }

    void makeEdges() {
        for (auto& segment : segments) {
            segment.makeEdges();
        }
    }

    void missingCoincidence() {
        for (auto& segment : segments) {
            if (pointType == segment.c.type)
                continue;
            segment.missingCoincidence();
        }
    }

    bool resolveCoincidence() {
        for (auto& segment : segments) {
            if (pointType == segment.c.type)
                continue;
            // need watchdog here so it doesn't loop forever
            int watchDog = 1000;   // !!! not sure how to compute maximum possible loopage
            while (!segment.resolveCoincidence())
                if (!--watchDog)
                    return false;
        }
        return true;
    }

#if 0
    void resolvePoints() {
        for (auto& segment : segments) {
            if (pointType == segment.c.type)
                continue;
            if (!segment.intersections.size())  // size may be zero if coincident
                continue;
            segment.resolvePoints();
        }
    }
#endif

    void setBounds() {
        for (auto& segment : segments) {
            ptBounds.add(segment.ptBounds);
            tightBounds.add(segment.tightBounds);
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
    void dump() const;
    void dumpDetail() const;
    void dumpFull() const;
    void dumpHex() const;
#endif
#if OP_DEBUG_IMAGE
    void draw() const;
#endif

    OpContours* contours;
    std::list<OpSegment> segments;
    OpPointBounds ptBounds;
    OpTightBounds tightBounds;
    OpOperand operand; // first or second argument to a binary operator
#if OP_DEBUG
    int id;
#endif
};

struct OpContours {
    OpContours(OpOperator op)
        : _operator(op) {
        sectStorage = new OpSectStorage;
    }

    ~OpContours() {
        do {
            OpSectStorage* next = sectStorage->next;
            delete sectStorage;
            sectStorage = next;
        } while (sectStorage);
    }

    OpIntersection* allocateIntersection();

    void apply() {
        for (auto& contour : contours) {
            contour.apply();
        }
    }

    void calcBounds() {
        for (auto& contour : contours) {
            contour.calcBounds();
        }
    }

    bool closeGap(OpEdge* last, OpEdge* first);

    void intersectEdge() {
        for (auto& contour : contours) {
            contour.intersectEdge();
        }
    }

    int leftFillTypeMask() {
        return (int) left;
    }

    void makeEdges() {
        for (auto& contour : contours) {
            contour.makeEdges();
        }
    }

    void missingCoincidence() {
        for (auto& contour : contours) {
            contour.missingCoincidence();
        }
    }

    bool resolveCoincidence() {
        for (auto& contour : contours) {
            if (!contour.resolveCoincidence())
                return false;
        }
        return true;
    }

#if 0
    void resolvePoints() {
        for (auto& contour : contours) {
            contour.resolvePoints();
        }
    }
#endif

    int rightFillTypeMask() {
        return (int) right;
    }

    void setBounds() {
        for (auto& contour : contours) {
            contour.setBounds();
        }
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
        assert(wind == (wind & (int) fillType));
        assert(sum == (sum & (int) fillType));
#endif
        if (!wind)
            return sum ? WindState::one : WindState::zero;
        if (sum)
            return wind == sum ? WindState::flipOff : WindState::one;
        return WindState::flipOn;
    }

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
    std::list<OpContour> contours;
    std::vector<OpEdge*> unsortables;
    OpSectStorage* sectStorage;
    OpFillType left;
    OpFillType right;
    OpOperator _operator;
    int coincidenceID = 0; // not debug since it is required for coin disambiguation
    int id = 0;
};

#endif