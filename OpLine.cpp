#include "OpCurve.h"

int OpLine::axisRawHit(Axis axis, float axisIntercept, rootCellar& cepts) const {
    const float* ptr = pts[0].asPtr(axis);
    float min = std::min(ptr[0], ptr[2]);
    float max = std::max(ptr[0], ptr[2]);
    if (min > axisIntercept || axisIntercept > max)
        return 0;
    if (min == max) {   // coincident line values are computed later
        OP_DEBUG_CODE(cepts[0] = OpNaN);
        OP_DEBUG_CODE(cepts[1] = OpNaN);
        return 2;
    }
    cepts[0] = (axisIntercept - ptr[0]) / (ptr[2] - ptr[0]);
    return 1;
}

int OpLine::axisRayHit(Axis offset, float axisIntercept, rootCellar& cepts) const {
    int realRoots = axisRawHit(offset, axisIntercept, cepts);
    if (2 == realRoots)
        return 2;
    int foundRoots = OpMath::KeepValidTs(cepts, realRoots);
    return foundRoots;
}

float OpLine::interp(XyChoice offset, float t) const {
    const float* ptr = &pts[0].x + +offset;
    if (0 == t)
        return ptr[0];
    if (1 == t)
        return ptr[2];
    return OpMath::Interp(ptr[0], ptr[2], t);
}

int OpLine::rawIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const {
    if (line[0].x == line[1].x)
        return axisRawHit(Axis::vertical, line[0].x, cepts);
    if (line[0].y == line[1].y)
        return axisRawHit(Axis::horizontal, line[0].y, cepts);
    OpLine rotated;
    toVertical(line, rotated);
    return rotated.axisRawHit(Axis::vertical, 0, cepts);
}

int OpLine::rayIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const {
    int realRoots = rawIntersect(line, cepts);
    if (2 == realRoots)
        return 2;
    int foundRoots = OpMath::KeepValidTs(cepts, realRoots);
    return foundRoots;
}

OpVector OpLine::normal(float t) const {
    return { pts[0].y - pts[1].y, pts[1].x - pts[0].x };
}

OpPoint OpLine::ptAtT(float t) const {
    if (0 == t)
        return pts[0];
    if (1 == t)
        return pts[1];
    return (1 - t) * pts[0] + t * pts[1];
}

OpVector OpLine::tangent(float t) const {
    return pts[1] - pts[0];
}
