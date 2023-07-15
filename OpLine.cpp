#include "OpCurve.h"

OpRoots OpLine::axisRawHit(Axis axis, float axisIntercept) const {
    const float* ptr = pts[0].asPtr(axis);
    if (!OpMath::Between(ptr[0], axisIntercept, ptr[2]))
        return OpRoots();
    // strict equality fails for denomalized numbers
    // if (ptr[2] == ptr[0]) {
    if (fabsf(ptr[2] - ptr[0]) <= OpEpsilon)   // coincident line values are computed later
        return OpRoots(OpNaN, OpNaN);
    return OpRoots((axisIntercept - ptr[0]) / (ptr[2] - ptr[0]));
}

float OpLine::interp(XyChoice offset, float t) const {
    const float* ptr = &pts[0].x + +offset;
    if (0 == t)
        return ptr[0];
    if (1 == t)
        return ptr[2];
    return OpMath::Interp(ptr[0], ptr[2], t);
}

OpRoots OpLine::rawIntersect(const LinePts& line) const {
    if (line.pts[0].x == line.pts[1].x)
        return axisRawHit(Axis::vertical, line.pts[0].x);
    if (line.pts[0].y == line.pts[1].y)
        return axisRawHit(Axis::horizontal, line.pts[0].y);
    OpCurve rotated = toVertical(line);
    return rotated.asLine().axisRawHit(Axis::vertical, 0);
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

OpVector OpLine::tangent() const {
    return pts[1] - pts[0];
}
