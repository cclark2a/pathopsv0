#ifndef OpContour_DEFINED
#define OpContour_DEFINED

#include "OpSegment.h"
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

    void addClose(const OpPoint& pt1, const OpPoint& pt2) {
        segments.emplace_back(pt1, pt2, this);
    }

    void addConic(const OpPoint pts[3], float weight) {
        if (pts[0] == pts[3])
            return addMove(pts);
        segments.emplace_back(pts, weight, this);
    }

    void addCubic(const OpPoint pts[4]) {
        // reduction to point if pt 0 equals pt 3 complicated since it requires pts 1, 2 be linear..
        assert(pts[0] != pts[3]); // !!! detect possible degenerate to code from actual test data
        segments.emplace_back(pts, cubicType, this);
    }

    void addLine(const OpPoint pts[2]) {
        if (pts[0] == pts[1])
            return addMove(pts);
        segments.emplace_back(pts, lineType, this);
    }

    void addMove(const OpPoint pts[1]) {
        segments.emplace_back(pts, pointType, this);
    }

    void addQuad(const OpPoint pts[3]) {
        if (pts[0] == pts[2])
            return addMove(pts);
        segments.emplace_back(pts, quadType, this);
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

    void findCoincidences() {
        for (auto& segment : segments) {
            segment.findCoincidences();
        }
    }

    void findSegmentExtrema() {
        for (auto& segment : segments) {
            segment.findExtrema();
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

    void matchIntersections() {
        for (auto& segment : segments) {
            segment.matchIntersections();
        }
    }

    void resolveCoincidence() {
        for (auto& segment : segments) {
            if (pointType == segment.c.type)
                continue;
            segment.resolveCoincidence();
        }
    }

    void resolvePoints() {
        for (auto& segment : segments) {
            if (pointType == segment.c.type)
                continue;
            segment.resolvePoints();
        }
    }

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

    void findCoincidences() {
        for (auto& contour : contours) {
            contour.findCoincidences();
        }
    }

    void findSegmentExtrema() {
        for (auto& contour : contours) {
            contour.findSegmentExtrema();
        }
    }

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

    void matchIntersections() {
        for (auto& contour : contours) {
            contour.matchIntersections();
        }
    }

    void resolveCoincidence() {
        for (auto& contour : contours) {
            contour.resolveCoincidence();
        }
    }

    void resolvePoints() {
        for (auto& contour : contours) {
            contour.resolvePoints();
        }
    }

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
    OpFillType left;
    OpFillType right;
    OpOperator _operator;
    int coincidenceID = 0; // not debug since it is required for coin disambiguation
    int id = 0;
};

#endif