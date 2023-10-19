// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurve.h"

OpRoots OpConic::axisRawHit(Axis offset, float axisIntercept) const {
    OpQuadCoefficients coeff = coefficients(offset, axisIntercept);
    return OpMath::QuadRootsReal(coeff.a, coeff.b, coeff.c - axisIntercept);
}

OpQuadCoefficients OpConic::coefficients(Axis axis, float intercept) const {
    const float* ptr = pts[0].asPtr(axis);
    float a = ptr[4];
    float b = ptr[2] * weight - intercept * weight + intercept;
    float c = ptr[0];
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    return { a, 2 * b, c };
}

float OpConic::denominator(float t) const {
    float B = 2 * (weight - 1);
    float C = 1;
    float A = -B;
    return (A * t + B) * t + C;
}

OpQuadCoefficients OpConic::derivative_coefficients(XyChoice offset) const {
    const float* ptr = &pts[0].x + +offset;
    float P20 = ptr[4] - ptr[0];
    float P10 = ptr[2] - ptr[0];
    float wP10 = weight * P10;
    float a = weight * P20 - P20;
    float b = P20 - 2 * wP10;
    float c = wP10;
    return { a, b, c };
}

OpRoots OpConic::extrema(XyChoice offset) const {
    OpQuadCoefficients dc = derivative_coefficients(offset);
    OpRoots roots = OpMath::QuadRootsInteriorT(dc.a, dc.b, dc.c);
    // In extreme cases, the number of roots returned can be 2. Pathops
    // will fail later on, so there's no advantage to plumbing in an error
    // return here.
    OP_ASSERT(0 == roots.count || 1 == roots.count);   // !!! I wanna see the extreme case...
    return roots;
};

OpRoots OpConic::rawIntersect(const LinePts& line) const {
    if (line.pts[0].x == line.pts[1].x)
        return axisRawHit(Axis::vertical, line.pts[0].x);
    if (line.pts[0].y == line.pts[1].y)
        return axisRawHit(Axis::horizontal, line.pts[0].y);
    OpCurve rotated = toVertical(line);
    OpRoots result = rotated.asConic().axisRawHit(Axis::vertical, 0);
    // for thread_circles104483, edges 113 and 117 fail to find intersection; check for error here
    for (size_t index = 0; index < result.count; ++index) {
        float t = result.roots[index];
        if (0 > t || t > 1)
            continue;
        OpPoint vertPt = rotated.asConic().ptAtT(t);
        if (fabsf(vertPt.x) >= RAW_INTERSECT_LIMIT) {
            result.rawIntersectFailed = true;
            break;
        }
    }
    return result;
}

bool OpConic::monotonic(XyChoice offset) const {
    return this->asConicQuad().monotonic(offset);
}

OpVector OpConic::normal(float t) const {
    OpVector tan = tangent(t);
    return { -tan.dy, tan.dx };
}

OpPoint OpConic::numerator(float t) const {
    OpPoint pt1w = pts[1] * weight;
    OpPoint C = pts[0];
    OpPoint A = pts[2] - 2 * pt1w + C;
    OpPoint B = 2 * (pt1w - C);
    return (A * t + B) * t + C;
}

OpPoint OpConic::ptAtT(float t) const {
    if (t == 0)
        return pts[0];
    if (t == 1)
        return pts[2];
    return numerator(t) / denominator(t);
}

OpCurve OpConic::subDivide(OpPtT ptT1, OpPtT ptT2) const {
    OpCurve result;
    result.type = OpType::conic;
    result.pts[0] = ptT1.pt;
    result.pts[2] = ptT2.pt;
    if (0 == ptT1.t && 1 == ptT2.t) {
        result.pts[1] = pts[1];
        result.weight = weight;
        return result;
    }
    OpPoint a;
    float az = 1;
    if (ptT1.t == 0) {
        a = ptT1.pt;
    } else if (ptT1.t != 1) {
        a = numerator(ptT1.t);
        az = denominator(ptT1.t);
    } else {
        a = ptT2.pt;
    }
    OpPoint c;
    float cz = 1;
    if (ptT2.t == 0) {
        c = ptT1.pt;
    } else if (ptT2.t != 1) {
        c = numerator(ptT2.t);
        cz = denominator(ptT2.t);
    } else {
        c = ptT2.pt;
    }
    float midT = (ptT1.t + ptT2.t) / 2;
    OpPoint d = numerator(midT);
    float dz = denominator(midT);
    OpPoint b = 2 * d - (a + c) / 2;
    float bz = 2 * dz - (az + cz) / 2;
    // if bz is 0, weight is 0, control point has no effect: any value will do
    float bzNonZero = !bz ? 1 : bz;
    result.pts[1] = b / bzNonZero;
    result.pts[1].pin(result.pts[0], result.pts[2]);
    result.weight = bz / sqrtf(az * cz);
    return result;
}

float OpConic::tangent(XyChoice offset, float t) const {
    OpQuadCoefficients coeff = derivative_coefficients(offset);
    return t * (t * coeff.a + coeff.b) + coeff.c;
}

OpVector OpConic::tangent(float t) const {
    return { tangent(XyChoice::inX, t), tangent(XyChoice::inY, t) };
}

float OpConic::tAtXY(float t1, float t2, XyChoice xy, float goal) const {
    OpPair endCheck = xyAtT( { t1, t2 }, xy );
    if (!OpMath::Between(endCheck.s, goal, endCheck.l))
        return OpNaN;
    float mid = (t1 + t2) * .5;
    float step = (mid - t1) * .5;
    while (step > OpEpsilon) {
        OpPair test = { mid - step, mid + step };
        OpPair x = xyAtT(test, xy);
        bool ordered = x.s < x.l;
        if (ordered ? goal < x.s : goal > x.s)
            mid = test.s;
        else if (ordered ? goal > x.l : goal < x.l)
            mid = test.l;
        step = step * .5;
    }
    return mid;
}

// given a pair of t values, return a pair of x values
// !!! implementation is non-optimal; but see if it works at all, and wonder, who uses conics?
OpPair OpConic::xyAtT(OpPair t, XyChoice xy) const {
    return { ptAtT(t.s).choice(xy), ptAtT(t.l).choice(xy) };
}
