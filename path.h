/*
 *  Copyright Pentrek Inc, 2022
 */

#ifndef _pentrek_path_h_
#define _pentrek_path_h_

#include "include/matrix.h"
#include "include/rect.h"
#include "include/span.h"
#include <vector>

namespace pentrek {

enum class PathVerb : uint8_t {
    move, line, quad, cubic, close
};

enum class PathDirection : uint8_t {
    ccw, cw
};

enum class PathFillType : uint8_t {
    winding, evenodd
};

class Path {
    static constexpr PathFillType kDefFillType = PathFillType::winding;

public:
    std::vector<Point> m_points;
    std::vector<PathVerb> m_verbs;
    PathFillType fillType = kDefFillType;

    Path() = default;
    Path(Span<Point>, Span<PathVerb>, PathFillType = kDefFillType);
    Path(const Path&) = default;
    Path(Path&&) = default;
    ~Path() = default;

    Path& operator=(const Path&) = default;

    bool empty() const { return m_points.size() == 0; }
    Span<const Point> points() const { return m_points; }
    Span<const PathVerb> verbs() const { return m_verbs; }

    Rect bounds() const;

    void move(Point);
    void line(Point);
    void quad(Point, Point);
    void cubic(Point, Point, Point);
    void close();

    void addLine(Point a, Point b) { this->move(a); this->line(b); }
    void addRect(const Rect&, PathDirection = PathDirection::ccw);
    void addOval(const Rect&, PathDirection = PathDirection::ccw);
    void addCircle(Point, float, PathDirection = PathDirection::ccw);
    void addPoly(Span<const Point>, bool doClose);
    void addPath(Span<const Point>, Span<const PathVerb>);
    void addPath(const Path&);

    Path transform(const Matrix&) const;
    void transformInPlace(const Matrix&);
    
    static Path FromRect(const Rect&, PathDirection = PathDirection::ccw);
    static Path FromOval(const Rect&, PathDirection = PathDirection::ccw);
    static Path FromPoly(Span<const Point>, bool doClose);

    bool hitTest(Point, float radius = 1);
    bool hitTest(const IRect&) const;

    template <typename M, typename L, typename Q, typename C, typename X>
    void visit(M m, L l, Q q, C c, X x) const {
        const Point* movePt = nullptr;
        const Point* p = m_points.data();
        for (auto v : m_verbs) {
            switch (v) {
                case PathVerb::move:
                    m(p);
                    movePt = p;
                    p += 1;
                break;
                case PathVerb::line:  l(p); p += 1; break;
                case PathVerb::quad:  q(p); p += 2; break;
                case PathVerb::cubic: c(p); p += 3; break;
                case PathVerb::close:
                    assert(movePt != nullptr);
                    x(p[-1], *movePt);
                    break;
            }
        }
        assert(p == m_points.data() + m_points.size());
    }
};

static inline Path operator*(const Matrix& m, const Path& p) {
    return p.transform(m);
}

static inline int points_for_verb(PathVerb v) {
    static const uint8_t gVerbPointCount[] = { 1, 1, 2, 3, 0 };
    return gVerbPointCount[(unsigned)v];
}

// Return <#points, #verbs> for the first contour in these spans. Will stop
// after the first contour, even if there are more than 1 in vergs.
// - returns <0,0> if the verb span is empty
// - asserts the verb span begins with PathVerb::move (if not empty)
//
std::pair<int, int> count_contour_pts_vbs(Span<const Point>, Span<const PathVerb>);

/////////////////////////////////

struct CtrlPoint {
    Point prev, curr, next;
};

void path_add_ctrlpoints(Path* dst, Span<const CtrlPoint>, bool doClose);
Path path_from_ctrlpoints(Span<const CtrlPoint>, bool doClose);

std::vector<CtrlPoint> mikemull_ctrlpoints(Span<const Point>,
                                           float tanScale, bool doClose, bool doC2 = true);

Path mikemull(Span<const Point>, float tanScale, bool doClose, bool doC2 = true);

} // namespace

#endif
