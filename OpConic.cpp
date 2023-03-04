#include "OpCurve.h"

int OpConic::axisRawHit(Axis offset, float axisIntercept, rootCellar& cepts) const {
    OpQuadCoefficients coeff = coefficients(offset, axisIntercept);
    return OpMath::QuadRootsReal(coeff.a, coeff.b, coeff.c - axisIntercept, cepts);
}

#if 0
int OpConic::axisRayHit(Axis offset, float axisIntercept, rootCellar& cepts) const {
    OpQuadCoefficients coeff = coefficients(offset, axisIntercept);
    return OpMath::QuadRootsValidT(coeff.a, coeff.b, coeff.c - axisIntercept, cepts);
}
#endif

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

int OpConic::extrema(XyChoice offset, rootCellar& t) const {
    OpQuadCoefficients dc = derivative_coefficients(offset);
    int roots = OpMath::QuadRootsInteriorT(dc.a, dc.b, dc.c, t);
    // In extreme cases, the number of roots returned can be 2. Pathops
    // will fail later on, so there's no advantage to plumbing in an error
    // return here.
    assert(0 == roots || 1 == roots);   // !!! I wanna see the extreme case...
    return roots;
};

int OpConic::rawIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const {
    if (line[0].x == line[1].x)
        return axisRawHit(Axis::vertical, line[0].x, cepts);
    if (line[0].y == line[1].y)
        return axisRawHit(Axis::horizontal, line[0].y, cepts);
    OpConic rotated;
    toVertical(line, rotated);
    return rotated.axisRawHit(Axis::vertical, 0, cepts);
}

int OpConic::rayIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const {
    if (line[0].x == line[1].x)
        return axisRayHit(Axis::vertical, line[0].x, cepts);
    if (line[0].y == line[1].y)
        return axisRayHit(Axis::horizontal, line[0].y, cepts);
    OpConic rotated;
    toVertical(line, rotated);
    return rotated.axisRayHit(Axis::vertical, 0, cepts);
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

void OpConic::subDivide(OpPtT ptT1, OpPtT ptT2, std::array<OpPoint, 4>& dst, float* w) const {
    dst[0] = ptT1.pt;
    dst[2] = ptT2.pt;
    if (0 == ptT1.t && 1 == ptT2.t) {
        dst[1] = pts[1];
        *w = weight;
        return;
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
    dst[1] = b / bzNonZero;
    dst[1].pin(dst[0], dst[2]);
    *w = bz / sqrtf(az * cz);
}

float OpConic::tangent(XyChoice offset, float t) const {
    OpQuadCoefficients coeff = derivative_coefficients(offset);
    return t * (t * coeff.a + coeff.b) + coeff.c;
}

OpVector OpConic::tangent(float t) const {
    return { tangent(XyChoice::inX, t), tangent(XyChoice::inY, t) };
}
