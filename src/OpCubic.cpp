// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurve.h"
#include "OpDebugRecord.h"
#include <cmath>

OpRoots OpCubic::axisRawHit(Axis axis, float axisIntercept) const {
    OpCubicCoefficients coeff = coefficients(axis);
    coeff.d -= axisIntercept;
    return OpMath::CubicRootsReal(coeff.a, coeff.b, coeff.c, coeff.d);
}

OpCubicCoefficients OpCubic::coefficients(Axis axis) const {
    const float* ptr = pts[0].asPtr(axis);
    OpCubicFloatType A = ptr[6];   // d
    OpCubicFloatType B = ptr[4] * 3;  // 3*c
    OpCubicFloatType C = ptr[2] * 3;  // 3*b
    OpCubicFloatType D = ptr[0];   // a
    A -= D - C + B;     // A =   -a + 3*b - 3*c + d
    B += 3 * D - 2 * C; // B =  3*a - 6*b + 3*c
    C -= 3 * D;         // C = -3*a + 3*b
    return { A, B, C, D };
}

OpRoots OpCubic::extrema(XyChoice offset) const {
    const float* ptr = &pts[0].x + +offset;
    float a = ptr[0];
    float b = ptr[2];
    float c = ptr[4];
    float d = ptr[6];
    float A = d - a + 3 * (b - c);
    float B = 2 * (a - b - b + c);
    float C = b - a;
    return OpMath::QuadRootsInteriorT(A, B, C);  // don't keep roots ~0, ~1
}

OpRoots OpCubic::inflections() const {
    OpPoint A = pts[1] - pts[0];
    OpPoint B = pts[2] - 2 * pts[1] + pts[0];
    OpPoint C = pts[3] + 3 * (pts[1] - pts[2]) - pts[0];
    return OpMath::QuadRootsInteriorT(B.x * C.y - B.y * C.x, A.x * C.y - A.y * C.x,
            A.x * B.y - A.y * B.x);  // don't keep roots ~0, ~1
}

OpPoint OpCubic::interp(float t) const {
    OpPoint ab = OpMath::Interp(pts[0], pts[1], t);
    OpPoint bc = OpMath::Interp(pts[1], pts[2], t);
    OpPoint cd = OpMath::Interp(pts[2], pts[3], t);
    OpPoint abc = OpMath::Interp(ab, bc, t);
    OpPoint bcd = OpMath::Interp(bc, cd, t);
    OpPoint abcd = OpMath::Interp(abc, bcd, t);
    return abcd;
}

bool OpCubic::monotonic(XyChoice offset) const {
    const float* ptr = &pts[0].x + +offset;
    return OpMath::Between(ptr[0], ptr[2], ptr[6])
        && OpMath::Between(ptr[0], ptr[4], ptr[6]);
}

OpVector OpCubic::normal(float t) const {
    OpVector tan = tangent(t);
    return { -tan.dy, tan.dx };
}

OpPoint OpCubic::ptAtT(float t) const {
    if (0 == t)
        return pts[0];
    if (1 == t)
        return pts[3];
    float one_t = 1 - t;
    float one_t2 = one_t * one_t;
    float a = one_t2 * one_t;
    float b = 3 * one_t2 * t;
    float t2 = t * t;
    float c = 3 * one_t * t2;
    float d = t2 * t;
    return a * pts[0] + b * pts[1] + c * pts[2] + d * pts[3];
}

#if 0 // unused
OpPoint OpCubic::doublePtAtT(float t) const {
    if (0 == t)
        return pts[0];
    if (1 == t)
        return pts[3];
    double one_t = 1. - t;
    double one_t2 = one_t * one_t;
    double a = one_t2 * one_t;
    double b = 3 * one_t2 * t;
    double t2 = t * t;
    double c = 3 * one_t * t2;
    double d = t2 * t;
    return {
        (float)(a * pts[0].x + b * pts[1].x + c * pts[2].x + d * pts[3].x),
        (float)(a * pts[0].y + b * pts[1].y + c * pts[2].y + d * pts[3].y)
    };
}
#endif

void OpCubic::pinCtrls(XyChoice offset) {
    OP_DEBUG_CODE(float orig1 = pts[1].choice(offset));
    OP_DEBUG_CODE(float orig2 = pts[2].choice(offset));
    *pts[1].asPtr(offset) = OpMath::Pin(pts[0].choice(offset), pts[1].choice(offset), pts[3].choice(offset));
    *pts[2].asPtr(offset) = OpMath::Pin(pts[0].choice(offset), pts[2].choice(offset), pts[3].choice(offset));
#if OP_DEBUG
    if (orig1 != pts[1].choice(offset))
        *debugOriginalCtrls[0].asPtr(offset) = orig1;
    if (orig2 != pts[2].choice(offset))
        *debugOriginalCtrls[1].asPtr(offset) = orig2;
#endif
}

OpRoots OpCubic::rawIntersect(const LinePts& line) const {
    if (line.pts[0].x == line.pts[1].x)
        return axisRawHit(Axis::vertical, line.pts[0].x);
    if (line.pts[0].y == line.pts[1].y)
        return axisRawHit(Axis::horizontal, line.pts[0].y);
    OpCurve rotated = toVertical(line);
    OpRoots result = rotated.asCubic().axisRawHit(Axis::vertical, 0);
    // for thread_cubics8753, edges 54 and 55 fail to find intersection; check for error here
    for (size_t index = 0; index < result.count; ++index) {
        float t = result.roots[index];
        if (0 > t || t > 1)
            continue;
        OpPoint vertPt = rotated.asCubic().ptAtT(t);
#if OP_DEBUG_RECORD
        OpDebugRecord(rotated, t, vertPt, result);
#endif
        if (fabsf(vertPt.x) >= RAW_INTERSECT_LIMIT) {
            result.rawIntersectFailed = true;
            break;
        }
    }
    return result;
}

OpCurve OpCubic::subDivide(OpPtT ptT1, OpPtT ptT2) const {
    OpCurve result;
    result.type = OpType::cubic;
    result.pts[0] = ptT1.pt;
    result.pts[3] = ptT2.pt;
    result.weight = 1;
    if (ptT1.t == 0 && ptT2.t == 1) {
        result.pts[1] = pts[1];
        result.pts[2] = pts[2];
    } else {
        OpPoint a = interp(ptT1.t);
        OpPoint e = interp((ptT1.t * 2 + ptT2.t) / 3);
        OpPoint f = interp((ptT1.t + ptT2.t * 2) / 3);
        OpPoint d = interp(ptT2.t);
        OpVector m = e * 27 - a * 8 - d;
        OpVector n = f * 27 - a - d * 8;
        /* b = */ result.pts[1] = (m * 2 - n) / 18;
        /* c = */ result.pts[2] = (n * 2 - m) / 18;
#if 0   // !!! EXPERIMENT
        // the above may compute control points on either side of the end points line
        // pinning the control points to the bounds won't prevent this
        // likely better to turn a line without control points in this case
        // Or, leave the curve alone and return state so that resulting edge is unsortable
        // It is not sufficient to return line whenever t range is sufficiently small;
        // !!! this may effect curves that should be left alone; need to add visual debugging

        OpVector ctrlVs[4] = { 
            result.pts[1] - ptT1.pt, result.pts[1] - ptT2.pt, 
            result.pts[2] - ptT1.pt, result.pts[2] - ptT2.pt 
        };
        auto doubleCross = [](OpPoint c, OpPoint s, OpPoint e) {
            double cvsx = (double) c.x - s.x;
            double cvsy = (double) c.y - s.y;
            double cvex = (double) c.x - e.x;
            double cvey = (double) c.y - e.y;
            return cvsx * cvey - cvsy * cvex;
        };
        float c0x1 = ctrlVs[0].cross(ctrlVs[1]);
        double dc0x1 = doubleCross(result.pts[1], ptT1.pt, ptT2.pt);
        float c2x3 = ctrlVs[2].cross(ctrlVs[3]);
        double dc2x3 = doubleCross(result.pts[2], ptT1.pt, ptT2.pt);
        if (c0x1 * c2x3 < 0 || dc0x1 * dc2x3 < 0) {
            result.type = OpType::line;
            result.pts[1] = ptT2.pt;
            return result;
        }
#endif
        result.pts[1].pin(result.pts[0], result.pts[3]);
        result.pts[2].pin(result.pts[0], result.pts[3]);
    }
    return result;
}

OpVector OpCubic::tangent(float t) const {
    // !!! this does not handle if t == 0 and pt[0] == pt[1] or if t == 1 and pt[2] == pt[3]
    // don't think this is needed for pathops, but is used by debugging (image drawing)
    // for now, put the additional logic there
    // !!! document why this needs to be double (include example test requiring it)
    auto tangent = [this](XyChoice offset, double t) {
        const float* ptr = &pts[0].x + +offset;
        double one_t = 1 - t;
        double a = ptr[0];
        double b = ptr[2];
        double c = ptr[4];
        double d = ptr[6];
        return (float) (3 * ((b - a) * one_t * one_t + 2 * (c - b) * t * one_t + (d - c) * t * t));
    };
    return { tangent(XyChoice::inX, t), tangent(XyChoice::inY, t) };
}

// given a pair of t values, return a pair of x values
OpPair OpCubic::xAtT(OpPair t) const {
    OpPair one_t = 1 - t;
    OpPair one_t2 = one_t * one_t;
    OpPair a = one_t2 * one_t;
    OpPair b = 3 * one_t2 * t;
    OpPair t2 = t * t;
    OpPair c = 3 * one_t * t2;
    OpPair d = t2 * t;
    return a * pts[0].x + b * pts[1].x + c * pts[2].x + d * pts[3].x;
}
