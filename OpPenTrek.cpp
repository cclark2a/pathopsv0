/*
 *  Copyright Pentrek Inc, 2023
 */

#include "OpContour.h"
#include "OpEdge.h"
#include "PathOps.h"

#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_PENTREK

#ifdef PENTREK_FOR_REAL
#include "include/path.h"
#include "include/path_builder.h"
#else
namespace pentrek {
enum class PathVerb { move, line, quad, cubic, close };
enum class PathFillType { evenodd };
class Point {
public:
    Point(float , float) {}
};
class PointsData {
public:
    void* data() { return nullptr; }
};
class Points {
public:
    PointsData pts;
    PathVerb vrb;
};
class PathObject {
public:
    void* get() { return nullptr; }
};
class PathIter {
public:
    bool isDone() { return false; }
    Points next() { return Points(); }
};
class Path {
public:
    PathFillType fillType() const { return PathFillType::evenodd; }
    PathIter iter() const { return PathIter(); }
    PathObject Circle(Point , float ) { return PathObject(); }
    void TestOps();
};
class BuilderObject {
public:
    void dump() {}
};
class PathBuilder {
public:
    void close() {}
    void cubic(Point ,  Point , Point ) {}
    BuilderObject* detach() { return new BuilderObject(); }
    void line(Point ) {}
    void move(Point ) {}
    void quad(Point ,  Point ) {}
    void rewind() {}
};
}
#endif

using namespace pentrek;

// Porting InPath

#ifdef PENTREK_FOR_REAL
void OpDebugOut(const std::string& str) {
    printf("pathops: %s\n", str.c_str());
}
#else
bool OpOutPath::debugIsEmpty(void) const { return false; }
void dmp(struct OpOutPath const &) {}
#endif

bool OpInPath::isInverted() const {
    return false;   // pentrek does not support inverse filltype
}

bool OpContours::build(OpInPath ipath, OpOperand operand) {
    auto src = (const Path*)ipath.externalReference;

    setFillType(operand, src->fillType() == PathFillType::evenodd ? OpFillType::evenOdd
                                                                  : OpFillType::winding);
    OpContour* head = makeContour(operand);

    auto iter = src->iter();
    while (!iter.isDone()) {
        auto rec = iter.next();
        auto pts = (const OpPoint*)rec.pts.data();
        switch (rec.vrb) {
            case PathVerb::move:
                head = addMove(head, operand, pts);
                break;
            case PathVerb::line:
                head->addLine(pts);
                break;
            case PathVerb::quad:
                head->addQuad(pts);
                break;
            case PathVerb::cubic:
                head->addCubic(pts);
                break;
            case PathVerb::close:
                break;
        }
    }
    head->addClose();
    return true;
}

// Porting OutPath

void OpOutPath::setInverted(bool wasInverted) {
    printf("setInverted(%d) ignored!\n", wasInverted);
}

void OpOutPath::setEmpty() {
    auto builder = (PathBuilder*)externalReference;
    builder->rewind();
}

void OpEdge::output(OpOutPath opath) {
    auto to_pk = [](OpPoint p) { return Point{p.x, p.y}; };

    auto sink = (PathBuilder*)opath.externalReference;

    sink->move(to_pk(whichPtT().pt));
    const OpEdge* firstEdge = this;
    OpEdge* edge = this;
    do {
        auto endPt = to_pk((EdgeMatch::start == edge->whichEnd ? edge->end : edge->start).pt);
        OpType type = edge->type();
        if (OpType::line == type)
            sink->line(endPt);
        else {
            auto ctrlPt0 = to_pk(edge->ctrlPts[0]);
            if (OpType::quad == type) {
                sink->quad(ctrlPt0, endPt);
            } else if (OpType::conic == type) {
                OP_ASSERT(false);
            } else {
                OP_ASSERT(OpType::cubic == type);
                auto ctrlPt1 = to_pk(edge->ctrlPts[1]);
                if (EdgeMatch::end == edge->whichEnd) {
                    std::swap(ctrlPt0, ctrlPt1);
                }
                sink->cubic(ctrlPt0, ctrlPt1, endPt);
            }
        }
        edge = edge->nextOut();
    } while (firstEdge != edge);
    sink->close();
}

////////////////////

void Path::TestOps() {
    auto p0 = Path::Circle({100, 100}, 50);
    auto p1 = Path::Circle({120, 100}, 50);
    OpInPath op1(p0.get());
    OpInPath op2(p1.get());

    PathBuilder bu;
    OpOutPath opOut(&bu);

    PathOps(op1, op2, OpOperator::Intersect, opOut);

    auto res = bu.detach();
    res->dump();
}

#endif
