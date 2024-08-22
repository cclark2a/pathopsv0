// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"
#if OP_RELEASE_TEST

#ifdef _WIN32
#pragma optimize( "", off )
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "OpContour.h"
#include "OpCurve.h"
#include "PathOps.h"
#include "skia/SkiaPaths.h"  // for curve types

enum class ClipToBounds {
    noClip,
    clip
};

constexpr uint32_t debugOpBlack = 0xFF000000;

struct DebugOpPoint {
    DebugOpPoint() 
        : x(OpNaN)
        , y(OpNaN) {
    }

    DebugOpPoint(double xIn, double yIn) 
        : x(xIn)
        , y(yIn) {
    }

    void operator+=(DebugOpPoint v) {
        x += v.x;
        y += v.y;
    }

    void operator-=(DebugOpPoint v) {
        x -= v.x;
        y -= v.y;
    }

    void operator*=(double s) {
        x *= s;
        y *= s;
    }

    void operator/=(double s) {
        x /= s;
        y /= s;
    }

    DebugOpPoint operator+(DebugOpPoint v) {
        DebugOpPoint result = *this;
        result += v;
        return result;
    }

    DebugOpPoint operator-(DebugOpPoint v) {
        DebugOpPoint result = *this;
        result -= v;
        return result;
    }

    DebugOpPoint operator*(double s) {
        DebugOpPoint result = *this;
        result *= s;
        return result;
    }

    friend DebugOpPoint operator*(double a, const DebugOpPoint& b) {
        return { a * b.x, a * b.y };
    }

    DebugOpPoint operator/(double s) {
        DebugOpPoint result = *this;
        result /= s;
        return result;
    }

    bool operator==(DebugOpPoint v) {
        return x == v.x && y == v.y;
    }

    double choice(Axis axis) const {
        return Axis::vertical == axis ? x : y;
    }

    double x;
    double y;
};

struct DebugColorPt : DebugOpPoint {
    DebugColorPt(double xIn, double yIn, uint32_t c) 
        : DebugOpPoint(xIn, yIn)
        , t(OpNaN)
        , color(c)
        , sprite(DebugSprite::diamond) {
    }

    DebugColorPt(double xIn, double yIn, float tIn, uint32_t c) 
        : DebugOpPoint(xIn, yIn)
        , t(tIn)
        , color(c)
        , sprite(DebugSprite::diamond) {
    }

    DebugColorPt(DebugOpPoint pt, double tIn, uint32_t c) 
        : DebugOpPoint(pt)
        , t(tIn)
        , color(c)
        , sprite(DebugSprite::diamond) {
    }

    DebugColorPt(double xIn, double yIn, double tIn, uint32_t c, DebugSprite sprite) 
        : DebugOpPoint(xIn, yIn)
        , t(tIn)
        , color(c)
        , sprite(sprite) {
    }

    double t;    // for display only
    uint32_t color;
    DebugSprite sprite;
};

struct DebugOpRect {
    bool ptInRect(DebugOpPoint pt) const {
        return left <= pt.x + OpEpsilon 
                && pt.x - OpEpsilon <= right 
                && top <= pt.y + OpEpsilon 
                && pt.y - OpEpsilon <= bottom;
    }

    void reset() {
        left = top = right = bottom = OpNaN;
    }

    double left;
    double top;
    double right;
    double bottom;
};

struct DebugOpRoots {
    DebugOpRoots() 
        : count(0) {
    }

    DebugOpRoots(float one)
        : count(1) {
        roots[0] = one;
    }

    DebugOpRoots(float one, float two) {
        count = 1 + (one != two);
        roots[0] = one;
        roots[1] = two;
    }

    DebugOpRoots keepValidTs() {
        size_t foundRoots = 0;
        for (int index = 0; index < count; ++index) {
            double tValue = roots[index];
            if (tValue != tValue || 0 > tValue || tValue > 1)
                continue;
            for (size_t idx2 = 0; idx2 < foundRoots; ++idx2) {
                if (roots[idx2] == tValue) {
                    goto notUnique;
                }
            }
            roots[foundRoots++] = tValue;
        notUnique:
            ;
        }
        count = foundRoots;
        return *this;
    }

    int count;
    std::array<double, 5> roots;
};

constexpr double PI = 3.1415926535897931;

struct DebugOpMath {
    static DebugOpRoots CubicRootsReal(double A, double B, double C, double D) {
        if (0 == A)
            return QuadRootsReal(B, C, D);
        if (0 == D) {  // 0 is one root
            DebugOpRoots roots = QuadRootsReal(A, B, C);
            for (int i = 0; i < roots.count; ++i) {
                if (0 == roots.roots[i])
                    return roots;
            }
            roots.roots[roots.count++] = 0;
            return roots;
        }
        if (0 == A + B + C + D) {  // 1 is one root
            DebugOpRoots roots = QuadRootsReal(A, A + B, -D);
            for (int i = 0; i < roots.count; ++i) {
                if (1 == roots.roots[i]) {
                    return roots;
                }
            }
            roots.roots[roots.count++] = 1;
            return roots;
        }
        double invA = 1 / A;
        double a = B * invA;
        double b = C * invA;
        double c = D * invA;
        double a2 = a * a;
        double Q = (a2 - b * 3) / 9;
        double R = (2 * a2 * a - 9 * a * b + 27 * c) / 54;
        double R2 = R * R;
        double Q3 = Q * Q * Q;
        double R2MinusQ3 = R2 - Q3;
        double adiv3 = a / 3;
        double r;
        DebugOpRoots roots;
        double* rootPtr = &roots.roots[0];
        if (R2MinusQ3 < 0) {   // we have 3 real roots
            // the divide/root can, due to finite precisions, be slightly outside of -1...1
            double theta = acos(std::max(std::min(1., R / sqrt(Q3)), -1.));
            double neg2RootQ = -2 * sqrt(Q);
            r = neg2RootQ * cos(theta / 3) - adiv3;
            *rootPtr++ = r;
            r = neg2RootQ * cos((theta + 2 * PI) / 3) - adiv3;
            if (roots.roots[0] != r)
                *rootPtr++ = r;
            r = neg2RootQ * cos((theta - 2 * PI) / 3) - adiv3;
            if (roots.roots[0] != r && (rootPtr - &roots.roots[0] == 1 || roots.roots[1] != r))
                *rootPtr++ = r;
        } else {  // we have 1 real root
            double sqrtR2MinusQ3 = sqrt(R2MinusQ3);
            // !!! need to rename this 'A' something else; since parameter is also 'A'
            A = fabs(R) + sqrtR2MinusQ3;
            A = std::cbrt(A);
            if (R > 0)
                A = -A;
            if (A != 0)
                A += Q / A;
            r = A - adiv3;
            *rootPtr++ = r;
            if (R2 == Q3) {
                r = -A / 2 - adiv3;
                if (roots.roots[0] != r)
                    *rootPtr++ = r;
            }
        }
        roots.count = static_cast<int>(rootPtr - &roots.roots[0]);
        return roots;
    }

    static DebugOpRoots CubicRootsValidT(double A, double B, double C, double D) {
        return CubicRootsReal(A, B, C, D).keepValidTs();
    }

    static double Interp(double A, double B, double t) {
        return A + (B - A) * t;
    }

    static DebugOpPoint Interp(DebugOpPoint A, DebugOpPoint B, double t) {
        return A + (B - A) * t;
    }

    static bool IsNaN(double x) {
        return !(x == x);
    }

    static DebugOpRoots QuadRootsReal(double A, double B, double C) {
        if (0 == A) {
            if (0 == B) {
                if (C == 0)
                    return DebugOpRoots();
                return DebugOpRoots(0);
            }
            return DebugOpRoots(-C / B);
        }
        const double p = B / (2 * A);
        const double q = C / A;
        /* normal form: x^2 + px + q = 0 */
        const double p2 = p * p;
        if (p2 < q)
            return DebugOpRoots();
        double sqrtl = sqrt(p2 - q);
        return DebugOpRoots(sqrtl - p, -sqrtl - p);
    }

    static DebugOpRoots QuadRootsValidT(double A, double B, double C) {
        return QuadRootsReal(A, B, C).keepValidTs();
    }
};

struct DebugOpQuad;
struct DebugOpConic;
struct DebugOpCubic;

int nextContourID = 0;

struct DebugOpCurve {
    DebugOpCurve()
        : size(0)
        , weight(1)
        , type(PathOpsV0Lib::CurveType::no)
        , id(0)
        , pathContour(0)
        , color(debugOpBlack) {
    }
    const DebugOpQuad& asQuad() const;
    const DebugOpConic& asConic() const;
    const DebugOpCubic& asCubic() const;
    DebugOpRoots axisRayHit(Axis offset, double axisIntercept) const;
    void mapTo(OpCurve& ) const;
    int pointCount() const { return static_cast<int>(type) + (type < PathOpsV0Lib::CurveType::conic); }
    DebugOpPoint ptAtT(double t) const;
    DebugOpRoots rayIntersect(const OpDebugRay& ) const;
    void rectCurves(std::vector<DebugOpCurve>& bounded) const;
    void subDivide(double a, double b, DebugOpCurve& dest) const;
    bool tInRect(double t, const DebugOpRect& bounds) const;

    DebugOpPoint pts[4];
    size_t size; // size of original curve data, not double points
    double weight;
    PathOpsV0Lib::CurveType type;
    int id;     // edge or segment
    int pathContour;
    uint32_t color;
};

struct DebugOpQuadCoefficients {
    double a;
    double b;
    double c;
};

struct DebugOpQuad : DebugOpCurve {
    DebugOpRoots axisRayHit(Axis axis, double axisIntercept) const {
        DebugOpQuadCoefficients coeff = coefficients(axis);
        coeff.c -= axisIntercept;
        return DebugOpMath::QuadRootsValidT(coeff.a, coeff.b, coeff.c);
    }

    DebugOpQuadCoefficients coefficients(Axis axis) const {
        double a = pts[2].choice(axis);
        double b = pts[1].choice(axis);
        double c = pts[0].choice(axis);
        a += c - 2 * b;    // A = a - 2*b + c
        b -= c;            // B = -(b - c)
        return { a, 2 * b, c };
    }

    DebugOpPoint ptAtT(double t) const {
        if (0 == t) {
            return pts[0];
        }
        if (1 == t) {
            return pts[2];
        }
        double one_t = 1 - t;
        double a = one_t * one_t;
        double b = 2 * one_t * t;
        double c = t * t;
        return a * pts[0] + b * pts[1] + c * pts[2];
    }

    void subDivide(double a, double b, DebugOpCurve& dst) const {
        dst.pts[0] = ptAtT(a);
        dst.pts[2] = ptAtT(b);
        dst.pts[1] = 2 * ptAtT((a + b) / 2) - (dst.pts[0] + dst.pts[2]) / 2;
    }
};

struct DebugOpConic : DebugOpCurve {
    DebugOpRoots axisRayHit(Axis offset, double axisIntercept) const {
        DebugOpQuadCoefficients coeff = coefficients(offset, axisIntercept);
        return DebugOpMath::QuadRootsValidT(coeff.a, coeff.b, coeff.c - axisIntercept);
    }

    DebugOpQuadCoefficients coefficients(Axis axis, double intercept) const {
        double a = pts[2].choice(axis);
        double b = pts[1].choice(axis) * weight - intercept * weight + intercept;
        double c = pts[0].choice(axis);
        a += c - 2 * b;    // A = a - 2*b + c
        b -= c;            // B = -(b - c)
        return { a, 2 * b, c };
    }

    double denominator(double t) const {
        double B = 2 * (weight - 1);
        double C = 1;
        double A = -B;
        return (A * t + B) * t + C;
    }

    DebugOpPoint numerator(double t) const {
        DebugOpPoint pt1w = pts[1];
        pt1w *= weight;
        DebugOpPoint C = pts[0];
        DebugOpPoint A = pts[2];
        A += (-2 * pt1w + C);
        DebugOpPoint B = 2 * (pt1w - C);
        return (A * t + B) * t + C;
    }

    DebugOpPoint ptAtT(double t) const {
        return numerator(t) / denominator(t);
    }

    void subDivide(double t1, double t2, DebugOpCurve& dst) const {
        dst.pts[0] = ptAtT(t1);
        dst.pts[2] = ptAtT(t2);
        DebugOpPoint a = numerator(t1);
        double az = denominator(t1);
        DebugOpPoint c = numerator(t2);
        double cz = denominator(t2);
        double midT = (t1 + t2) / 2;
        DebugOpPoint d = numerator(midT);
        double dz = denominator(midT);
        DebugOpPoint b = 2 * d - (a + c) / 2;
        double bz = 2 * dz - (az + cz) / 2;
        // if bz is 0, weight is 0, control point has no effect: any value will do
        double bzNonZero = 0 == bz ? 1 : bz;
        dst.pts[1] = b / bzNonZero;
        dst.weight = bz / sqrt(az * cz);
    }
};

struct DebugOpCubicCoefficients {
    double a;
    double b;
    double c;
    double d;
};

struct DebugOpCubic : DebugOpCurve {
    DebugOpRoots axisRayHit(Axis offset, double axisIntercept) const {
        DebugOpCubicCoefficients coeff = coefficients(offset);
        coeff.d -= axisIntercept;
        return DebugOpMath::CubicRootsValidT(coeff.a, coeff.b, coeff.c, coeff.d);
    }

    DebugOpCubicCoefficients coefficients(Axis axis) const {
        double A = pts[3].choice(axis);   // d
        double B = pts[2].choice(axis) * 3;  // 3*c
        double C = pts[1].choice(axis) * 3;  // 3*b
        double D = pts[0].choice(axis);   // a
        A -= D - C + B;     // A =   -a + 3*b - 3*c + d
        B += 3 * D - 2 * C; // B =  3*a - 6*b + 3*c
        C -= 3 * D;         // C = -3*a + 3*b
        return { A, B, C, D };
    }

    DebugOpPoint interp(double t) const {
        DebugOpPoint ab = DebugOpMath::Interp(pts[0], pts[1], t);
        DebugOpPoint bc = DebugOpMath::Interp(pts[1], pts[2], t);
        DebugOpPoint cd = DebugOpMath::Interp(pts[2], pts[3], t);
        DebugOpPoint abc = DebugOpMath::Interp(ab, bc, t);
        DebugOpPoint bcd = DebugOpMath::Interp(bc, cd, t);
        DebugOpPoint abcd = DebugOpMath::Interp(abc, bcd, t);
        return abcd;
    }

    DebugOpPoint ptAtT(double t) const {
        double one_t = 1 - t;
        double one_t2 = one_t * one_t;
        double a = one_t2 * one_t;
        double b = 3 * one_t2 * t;
        double t2 = t * t;
        double c = 3 * one_t * t2;
        double d = t2 * t;
        return a * pts[0] + b * pts[1] + c * pts[2] + d * pts[3];
    }

    void subDivide(double t1, double t2, DebugOpCurve& dst) const {
        dst.pts[0] = ptAtT(t1);
        dst.pts[3] = ptAtT(t2);
        DebugOpPoint a = interp(t1);
        DebugOpPoint e = interp((t1 * 2 + t2) / 3);
        DebugOpPoint f = interp((t1 + t2 * 2) / 3);
        DebugOpPoint d = interp(t2);
        DebugOpPoint m = e * 27 - a * 8 - d;
        DebugOpPoint n = f * 27 - a - d * 8;
        /* b = */ dst.pts[1] = (m * 2 - n) / 18;
        /* c = */ dst.pts[2] = (n * 2 - m) / 18;
    }
};

#if OP_DEBUG_IMAGE
const DebugOpQuad& DebugOpCurve::asQuad() const { return *static_cast<const DebugOpQuad*>(this); }
const DebugOpConic& DebugOpCurve::asConic() const { return *static_cast<const DebugOpConic*>(this); }
const DebugOpCubic& DebugOpCurve::asCubic() const { return *static_cast<const DebugOpCubic*>(this); }

DebugOpRoots DebugOpCurve::axisRayHit(Axis axis, double axisIntercept) const {
    switch (type) {
    case PathOpsV0Lib::CurveType::line: {
        double denominator = pts[1].choice(axis) - pts[0].choice(axis);
        DebugOpRoots roots(0 == denominator ? OpNaN : (axisIntercept - pts[0].choice(axis)) / denominator);
        return roots.keepValidTs();
    }
    case PathOpsV0Lib::CurveType::quad: return asQuad().axisRayHit(axis, axisIntercept);
    case PathOpsV0Lib::CurveType::conic: return asConic().axisRayHit(axis, axisIntercept);
    case PathOpsV0Lib::CurveType::cubic: return asCubic().axisRayHit(axis, axisIntercept);
    default:
        OP_ASSERT(0);
    }
    return 0;
}

DebugOpRoots DebugOpCurve::rayIntersect(const OpDebugRay& ray) const {
    if (ray.useAxis)
        return axisRayHit(ray.axis, ray.value);
    DebugOpCurve rotated = *this;
    double adj = (double) ray.pts.pts[1].x - ray.pts.pts[0].x;
    double opp = (double) ray.pts.pts[1].y - ray.pts.pts[0].y;
    for (int n = 0; n < pointCount(); ++n) {
        double vdx = (double) pts[n].x - ray.pts.pts[0].x;
        double vdy = (double) pts[n].y - ray.pts.pts[0].y;
        rotated.pts[n].x = (float) (vdy * adj - vdx * opp);
        rotated.pts[n].y = (float) (vdy * opp + vdx * adj);
    }
    return rotated.axisRayHit(Axis::vertical, 0);
}

DebugOpPoint DebugOpCurve::ptAtT(double t) const {
    switch(type) {
    case PathOpsV0Lib::CurveType::line: return DebugOpMath::Interp(pts[0], pts[1], t);    
    case PathOpsV0Lib::CurveType::quad: return asQuad().ptAtT(t);
    case PathOpsV0Lib::CurveType::conic: return asConic().ptAtT(t);
    case PathOpsV0Lib::CurveType::cubic: return asCubic().ptAtT(t);
    default:
        OP_ASSERT(0);
        return DebugOpPoint();
    }
}

double debugZoom = 0;
constexpr int debugMargin = 2;
constexpr struct {
    int x;
    int y;
} debugBitmapBounds { bitmapWH - debugMargin * 2, bitmapWH - debugMargin * 2 };
DebugOpPoint debugCenter { debugBitmapBounds.x / 2, debugBitmapBounds.y / 2 };
DebugOpRect setBounds;

void DebugOpResetFocus() {
    debugZoom = 0;
    debugCenter = { debugBitmapBounds.x / 2, debugBitmapBounds.y / 2 };
    setBounds.reset();
}

double DebugOpGetCenterX() {
    return debugCenter.x;
}

double DebugOpGetCenterY() {
    return debugCenter.y;
}

double DebugOpGetOffsetX() {
    return debugBitmapBounds.x / 2 + debugMargin;
}

double DebugOpGetOffsetY() {
    return debugBitmapBounds.y / 2 + debugMargin;
}

double DebugOpGetZoomScale() {
    return std::pow(2, debugZoom / 8);
}

static DebugOpRect ZoomToRect() {
    double scale = DebugOpGetZoomScale() * 2;
    double radiusX = debugBitmapBounds.x / scale;
    double radiusY = debugBitmapBounds.y / scale;
    return { debugCenter.x - radiusX, debugCenter.y - radiusY, 
            debugCenter.x + radiusX, debugCenter.y + radiusY };
}

void DebugOpBounds(double& left, double& top, double& right, double& bottom) {
    DebugOpRect dRect = ZoomToRect();
    left = dRect.left;
    top = dRect.top;
    right = dRect.right;
    bottom = dRect.bottom;
}

void DebugOpScreenBounds(int& left, int&top, int& right, int& bottom) {
    left = top = debugMargin;
    right = bottom = bitmapWH - debugMargin;
}

void DebugOpRecord(FILE* recordFile) {
    fprintf(recordFile, "debugZoom: %.*g\n", DBL_DECIMAL_DIG, debugZoom);
    fprintf(recordFile, "debugCenter: %.*g, %.*g\n", DBL_DECIMAL_DIG, debugCenter.x, 
            DBL_DECIMAL_DIG, debugCenter.y);
}

void DebugOpResetBounds() {
    setBounds = ZoomToRect();
}

bool DebugOpCurve::tInRect(double t, const DebugOpRect& bounds) const {
    DebugOpPoint pt = ptAtT(t);
    return bounds.ptInRect(pt);
}

// generate curve scaled from one rect to another
void DebugOpCurve::rectCurves(std::vector<DebugOpCurve>& bounded) const {
    DebugOpRect bounds = ZoomToRect();
    DebugOpRoots lefts = axisRayHit(Axis::vertical, bounds.left);
    DebugOpRoots tops = axisRayHit(Axis::horizontal, bounds.top);
    DebugOpRoots rights = axisRayHit(Axis::vertical, bounds.right);
    DebugOpRoots bottoms = axisRayHit(Axis::horizontal, bounds.bottom);
    std::vector<double> cepts;
    if (tInRect(0, bounds))
        cepts.push_back(0);
    for (int index = 0; index < lefts.count; ++index) {
        DebugOpPoint pt = ptAtT(lefts.roots[index]);
        if (bounds.top <= pt.y + OpEpsilon && pt.y - OpEpsilon <= bounds.bottom)
            cepts.push_back(lefts.roots[index]);
    }
    for (int index = 0; index < tops.count; ++index) {
        DebugOpPoint pt = ptAtT(tops.roots[index]);
        if (bounds.left < pt.x + OpEpsilon && pt.x - OpEpsilon < bounds.right)
            cepts.push_back(tops.roots[index]);
    }
    for (int index = 0; index < rights.count; ++index) {
        DebugOpPoint pt = ptAtT(rights.roots[index]);
        if (bounds.top <= pt.y + OpEpsilon && pt.y - OpEpsilon <= bounds.bottom)
            cepts.push_back(rights.roots[index]);
    }
    for (int index = 0; index < bottoms.count; ++index) {
        DebugOpPoint pt = ptAtT(bottoms.roots[index]);
        if (bounds.left < pt.x + OpEpsilon && pt.x - OpEpsilon < bounds.right)
            cepts.push_back(bottoms.roots[index]);
    }
    if (tInRect(1, bounds))
        cepts.push_back(1);
    if (cepts.size() < 2)
        return;
    std::sort(cepts.begin(), cepts.end());
    double first = cepts[0];
    for (double c : cepts) {
        if (first == c)
            continue;
        if (c < 0)
            continue;
        if (first >= 1)
            break;
        double mid = DebugOpMath::Interp(first, c, .5);
        if (tInRect(mid, bounds)) {
            bounded.push_back(DebugOpCurve()); 
            DebugOpCurve& part = bounded.back();
            subDivide(first, c, part);
            part.id = id;
            part.pathContour = pathContour;
            part.size = size;
            part.type = type;
            part.color = color;
        }
        first = c;
    }
}

void DebugOpCurve::subDivide(double a, double b, DebugOpCurve& dest) const {
    switch (type) {
    case PathOpsV0Lib::CurveType::line: 
        dest.pts[0] = ptAtT(a); 
        dest.pts[1] = ptAtT(b); 
        return;
    case PathOpsV0Lib::CurveType::quad: 
        return asQuad().subDivide(a, b, dest);
    case PathOpsV0Lib::CurveType::conic: 
        return asConic().subDivide(a, b, dest);
    case PathOpsV0Lib::CurveType::cubic: 
        return asCubic().subDivide(a, b, dest);
    default:
        OP_ASSERT(0);
    }
}

#include "OpSegment.h"
#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"
#endif

std::vector<DebugOpCurve> debugLines;
std::vector<DebugOpCurve> debugSegments;
std::vector<DebugOpCurve> debugEdges;
std::vector<DebugOpCurve> debugFills;
std::vector<DebugOpCurve> debugHighlights;
std::vector<DebugOpCurve> debugInputs;
std::vector<DebugOpCurve> debugOutputs;
std::vector<DebugOpCurve> debugPaths;
std::vector<DebugColorPt> debugPoints;  // used for path end points

OpPoint DebugOpMap(DebugOpPoint dPt) {
    double z = DebugOpGetZoomScale();
    return OpPoint( (float) ((dPt.x - debugCenter.x) * z + debugBitmapBounds.x / 2 + debugMargin),
        (float) ((dPt.y - debugCenter.y) * z + debugBitmapBounds.y / 2 + debugMargin));
}

void DebugOpCurve::mapTo(OpCurve& c) const {
    c.c.data = debugGlobalContours->allocateCurveData(size);
    c.c.size = size;
    c.c.data->start = DebugOpMap(pts[0]);
    int endIndex;
    switch (type) {
        case PathOpsV0Lib::CurveType::line:
            endIndex = 1;
            break;
        case PathOpsV0Lib::CurveType::quad: {
            OpPoint ctrl = DebugOpMap(pts[1]);
            OP_ASSERT(size == offsetof(PathOpsV0Lib::CurveData, optionalAdditionalData) 
                    + sizeof ctrl);
            std::memcpy(c.c.data->optionalAdditionalData, &ctrl, sizeof ctrl);
            endIndex = 2;
            } break;
        case PathOpsV0Lib::CurveType::conic: {
            OpPoint ctrl = DebugOpMap(pts[1]);
            float floatWeight = (float) weight;
            char* dst = c.c.data->optionalAdditionalData;
            OP_ASSERT(size == offsetof(PathOpsV0Lib::CurveData, optionalAdditionalData) 
                    + sizeof ctrl + sizeof floatWeight);
            std::memcpy(dst, &ctrl, sizeof ctrl);
            dst += sizeof ctrl;
            std::memcpy(dst, &floatWeight, sizeof floatWeight);
            endIndex = 2;
            } break;
        case PathOpsV0Lib::CurveType::cubic: {
            OpPoint ctrls[2] { DebugOpMap(pts[1]), DebugOpMap(pts[2]) };
            OP_ASSERT(size == offsetof(PathOpsV0Lib::CurveData, optionalAdditionalData) 
                    + sizeof ctrls);
            std::memcpy(c.c.data->optionalAdditionalData, ctrls, sizeof ctrls);
            endIndex = 3;
            } break;
        default:
            OP_ASSERT(0);  // unimplemented
            return;
    }
    c.c.data->end = DebugOpMap(pts[endIndex]);
    c.c.type = type;
    c.contours = debugGlobalContours;
    return;
}

enum class UnsectType {
    none,
    edge,
    opp
};

enum class DrawEdgeType {
    normal,
    highlight
};

// !!! haven't decided how I want to abstract this : for now, just reference directly
#include "OpDebugColor.h"

void DebugOpDrawEdges(std::vector<DebugOpCurve>& curves, DrawEdgeType edgeType) {
    float strokeWidth = DrawEdgeType::normal == edgeType ? 0 : 5;
    SkPath path;
    uint32_t last = black;
    for (auto& curve : curves) {
        uint32_t color = black;
        const OpEdge* edge = findEdge(curve.id);
        if (edge)
            color = edge->debugColor;
        else
            OpDebugOut("edge " + STR(curve.id) + " not found\n");
        if (last != color) {
            if (!path.isEmpty()) {
                OpDebugImage::drawDoublePath(path, last, strokeWidth);
                path.reset();
            }
            last = color;
        }
        OpCurve c;
        curve.mapTo(c);
        OpDebugImage::addToPath(c, path);
    }
    OpDebugImage::drawDoublePath(path, last, strokeWidth);
}

void DebugOpDraw(std::vector<DebugOpCurve>& curves) {
    SkPath path;
    uint32_t color = black;
    for (auto& curve : curves) {
        OpCurve c;
        curve.mapTo(c);
        OpDebugImage::addToPath(c, path);
        color = curve.color;
    }
    OpDebugImage::drawDoublePath(path, color);
}

void DebugOpFill(std::vector<DebugOpCurve>& curves) {
    SkPath path;
    uint32_t color = black;
    for (auto& curve : curves) {
        OpCurve c;
        curve.mapTo(c);
        OpDebugImage::addToPath(c, path);
        color = curve.color;
    }
    OpDebugImage::drawDoubleFill(path, color);
}

void DebugOpDraw(const std::vector<OpDebugRay>& lines) {
    debugLines.clear();
    DebugOpRect bounds = ZoomToRect();
    for (auto& line : lines) {
        DebugOpCurve curve;
        curve.size = 2 * sizeof(OpPoint);
        curve.weight = 1;
        curve.type = PathOpsV0Lib::CurveType::line;
        curve.color = red;
        if (line.useAxis) {
            if (Axis::horizontal == line.axis) {
                if (bounds.top > line.value || line.value > bounds.bottom)
                    continue;
                curve.pts[0] = { bounds.left, line.value };
                curve.pts[1] = { bounds.right, line.value };
            } else {
                if (bounds.left > line.value || line.value > bounds.right)
                    continue;
                curve.pts[0] = { line.value, bounds.top };
                curve.pts[1] = { line.value, bounds.bottom };
            }
            debugLines.push_back(curve);
            continue;
        }
        int outIndex = 0;
        DebugOpPoint len { line.pts.pts[1].x - line.pts.pts[0].x, line.pts.pts[1].y - line.pts.pts[0].y };
        float leftY = line.pts.pts[0].y + len.y * (bounds.left - line.pts.pts[0].x) / len.x;
        if (bounds.top <= leftY && leftY < bounds.bottom)
            curve.pts[outIndex++] = { bounds.left, leftY };
        float topX = line.pts.pts[0].x + len.x * (bounds.top - line.pts.pts[0].y) / len.y;
        if (bounds.left <= topX && topX < bounds.right)
            curve.pts[outIndex++] = { bounds.top, topX };
        float rightY = line.pts.pts[0].y + len.y * (bounds.right - line.pts.pts[0].x) / len.x;
        if (bounds.top < rightY && rightY <= bounds.bottom)
            curve.pts[outIndex++] = { bounds.right, rightY };
        float bottomX = line.pts.pts[0].x + len.x * (bounds.bottom - line.pts.pts[0].y) / len.y;
        if (bounds.left < bottomX && bottomX <= bounds.right)
            curve.pts[outIndex++] = { bounds.bottom, bottomX };
        if (2 == outIndex)
            debugLines.push_back(curve);
        else if (0 != outIndex)
            OpDebugOut("unexpected ray bounds\n");
    }
    DebugOpDraw(debugLines);
}

static double curveWeight(const OpCurve& curve) {
    // !!! haven't decided how to support this through callbacks
    if (PathOpsV0Lib::CurveType::conic == curve.c.type) {
        char* dst = curve.c.data->optionalAdditionalData;
        dst += sizeof(OpPoint);
        float floatWeight;
        std::memcpy(&floatWeight, dst, sizeof floatWeight);
        return (double) floatWeight;
    }
    return 1.;
}

void DebugOpBuild(const OpSegment& seg, std::vector<DebugOpCurve>& debugSegs) {
    DebugOpCurve curve;
    for (int i = 0; i < seg.c.pointCount(); ++i)
        curve.pts[i] = { seg.c.hullPt(i).x, seg.c.hullPt(i).y } ;
    curve.weight = curveWeight(seg.c);
    curve.size = seg.c.c.size;
    curve.type = seg.c.c.type;
    curve.id = seg.id;
    curve.color = black;  // !!! fix once segments carry debug colors
    curve.rectCurves(debugSegs);
}

void DebugOpBuild(const DebugColorPt& dPt) {
    DebugOpRect bounds = ZoomToRect();
    if (!bounds.ptInRect(dPt))
        return;
    if (debugPoints.end() == std::find(debugPoints.begin(), debugPoints.end(), dPt))
        debugPoints.push_back(dPt);
}

void DebugOpBuild(const OpSegment& seg, const OpDebugRay& ray) {
    DebugOpCurve curve;
    for (int i = 0; i < seg.c.pointCount(); ++i)
        curve.pts[i] = { seg.c.hullPt(i).x, seg.c.hullPt(i).y } ;
    curve.weight = curveWeight(seg.c);
    curve.size = seg.c.c.size;
    curve.type = seg.c.c.type;
    DebugOpRoots roots = curve.rayIntersect(ray);
    for (int index = 0; index < roots.count; ++index) {
        DebugColorPt pt { curve.ptAtT(roots.roots[index]), roots.roots[index], black };
        DebugOpBuild(pt);
    }
}

void DebugOpClearSegments() {
    debugSegments.clear();
}

void DebugOpAdd(const OpSegment* segment) {
    DebugOpBuild(*segment, debugSegments);
}

void DebugOpDrawSegments() {
    DebugOpDraw(debugSegments);
}

DebugOpCurve OpEdge::debugSetCurve() const {
    DebugOpCurve dCurve;
    for (int i = 0; i < curve.pointCount(); ++i)
        dCurve.pts[i] = { curve.hullPt(i).x, curve.hullPt(i).y } ;
    // !!! missing conic weight for now
    dCurve.size = curve.c.size;
    dCurve.weight = curveWeight(curve);
    dCurve.type = curve.c.type;
    dCurve.id = id;
    dCurve.color = debugColor;
    return dCurve;
}

void DebugOpBuild(const OpEdge& edge, std::vector<DebugOpCurve>& debugEs) {
    DebugOpCurve curve = edge.debugSetCurve();
    curve.rectCurves(debugEs);
}

void DebugOpBuild(const OpEdge& edge, const OpDebugRay& ray) {
    DebugOpCurve curve = edge.debugSetCurve();
    DebugOpRoots roots = curve.rayIntersect(ray);
    for (int index = 0; index < roots.count; ++index) {
        DebugColorPt pt = { curve.ptAtT(roots.roots[index]), 
            edge.startT + (edge.endT - edge.startT) * roots.roots[index], black };
        DebugOpBuild(pt);
    }
}

void DebugOpBuild(Axis axis, float normal, float cept) {
    DebugColorPt pt = Axis::horizontal == axis 
        ? DebugColorPt(cept, normal, black) : DebugColorPt(normal, cept, black);
    DebugOpBuild(pt);
}

OpPoint DebugOpPtToPt(OpPoint src) {
    return DebugOpMap(DebugOpPoint(src.x, src.y));
}

void DebugOpClearEdges() {
    debugEdges.clear();
}

void DebugOpClearHighlight() {
    debugHighlights.clear();
}

void DebugOpAdd(const OpEdge* edge) {
    DebugOpBuild(*edge, debugEdges);
}

void DebugOpAddHighlight(const OpEdge* edge) {
    DebugOpBuild(*edge, debugHighlights);
}

void DebugOpDrawEdges() {
    DebugOpDrawEdges(debugEdges, DrawEdgeType::normal);
}

void DebugOpDrawHighlight() {
    DebugOpDrawEdges(debugHighlights, DrawEdgeType::highlight);
}

void DebugOpDraw(const std::vector<OpEdge>& edges) {
    for (auto edge : edges)
        DebugOpBuild(edge, debugEdges);
    DebugOpDrawEdges(debugEdges, DrawEdgeType::normal);
}

void DebugOpDraw(const std::vector<const OpEdge*>& edges) {
    for (auto edge : edges)
        DebugOpBuild(*edge, debugEdges);
    DebugOpDrawEdges(debugEdges, DrawEdgeType::normal);
}

void DebugOpHighlight(const std::vector<const OpEdge*>& edges) {
    for (auto edge : edges)
        DebugOpBuild(*edge, debugHighlights);
    DebugOpDrawEdges(debugHighlights, DrawEdgeType::highlight);
}

void DebugOpBuild(const SkPath& path, std::vector<DebugOpCurve>& debugPs, ClipToBounds clip,
        uint32_t color) {
    SkPath::RawIter iter(path);
    SkPoint curveStart {0, 0};
    SkPath::Verb verb;
    SkPoint lastPoint {0, 0};
    bool hasLastPoint = false;
    DebugOpCurve curve;
    do {
        SkPoint pts[4];
        verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            curve.pathContour = ++nextContourID;
            // !!! if frame paths are supported, don't add close unless fill is set
            if (hasLastPoint && lastPoint != curveStart) {
                curve.pts[0] = { lastPoint.fX, lastPoint.fY } ; 
                curve.pts[1] = { curveStart.fX, curveStart.fY } ; 
                curve.type = PathOpsV0Lib::CurveType::line;
                curve.color = color;
                if (ClipToBounds::clip == clip)
                    curve.rectCurves(debugPs);
                else
                    debugPs.emplace_back(curve);
                hasLastPoint = false;
            }
            curveStart = pts[0];
            continue;
        case SkPath::kLine_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.size = 2 * sizeof(OpPoint);
            curve.type = PathOpsV0Lib::CurveType::line;
            curve.color = color;
            if (ClipToBounds::clip == clip)
                curve.rectCurves(debugPs);
            else
                debugPs.emplace_back(curve);
            lastPoint = pts[1];
            hasLastPoint = true;
            break;
        case SkPath::kQuad_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.size = 3 * sizeof(OpPoint);
            curve.type = PathOpsV0Lib::CurveType::quad;
            curve.color = color;
            if (ClipToBounds::clip == clip)
                curve.rectCurves(debugPs);
            else
                debugPs.emplace_back(curve);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kConic_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.size = 3 * sizeof(OpPoint) + sizeof(float);
            curve.weight = iter.conicWeight();
            curve.type = PathOpsV0Lib::CurveType::conic;
            curve.color = color;
            if (ClipToBounds::clip == clip)
                curve.rectCurves(debugPs);
            else
                debugPs.emplace_back(curve);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kCubic_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.pts[3] = { pts[3].fX, pts[3].fY } ; 
            curve.size = 4 * sizeof(OpPoint);
            curve.type = PathOpsV0Lib::CurveType::cubic;
            curve.color = color;
            if (ClipToBounds::clip == clip)
                curve.rectCurves(debugPs);
            else
                debugPs.emplace_back(curve);
            lastPoint = pts[3];
            hasLastPoint = true;
            break;
        case SkPath::kClose_Verb:
            break;
        case SkPath::kDone_Verb:
            break;
        }
    } while (verb != SkPath::kDone_Verb);
    if (hasLastPoint && lastPoint != curveStart) {
        curve.pts[0] = { lastPoint.fX, lastPoint.fY } ; 
        curve.pts[1] = { curveStart.fX, curveStart.fY } ; 
        curve.type = PathOpsV0Lib::CurveType::line;
        curve.color = color;
        if (ClipToBounds::clip == clip)
            curve.rectCurves(debugPs);
        else
            debugPs.emplace_back(curve);
    }
}

void DebugOpBuild(const SkPath& path, const struct OpDebugRay& ray) {
    auto axisSect = [&](const DebugOpCurve& curve) {  // lambda
        DebugOpRoots roots = curve.rayIntersect(ray);
        for (int index = 0; index < roots.count; ++index) {
            DebugColorPt pt = { curve.ptAtT(roots.roots[index]), roots.roots[index], black };
            DebugOpBuild(pt);
        }
    };
    SkPath::RawIter iter(path);
    SkPoint curveStart {0, 0};
    SkPath::Verb verb;
    SkPoint lastPoint {0, 0};
    bool hasLastPoint = false;
    DebugOpCurve curve;
    curve.pathContour = ++nextContourID;
    do {
        SkPoint pts[4];
        verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            // !!! if frame paths are supported, don't add close unless fill is set
            if (hasLastPoint && lastPoint != curveStart) {
                curve.pts[0] = { lastPoint.fX, lastPoint.fY } ; 
                curve.pts[1] = { curveStart.fX, curveStart.fY } ; 
                curve.size = 2 * sizeof(OpPoint);
                curve.type = PathOpsV0Lib::CurveType::line;
                axisSect(curve);
                hasLastPoint = false;
            }
            curveStart = pts[0];
            curve.pathContour = ++nextContourID;
            continue;
        case SkPath::kLine_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.size = 2 * sizeof(OpPoint);
            curve.type = PathOpsV0Lib::CurveType::line;
            axisSect(curve);
            lastPoint = pts[1];
            hasLastPoint = true;
            break;
        case SkPath::kQuad_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.size = 3 * sizeof(OpPoint);
            curve.type = PathOpsV0Lib::CurveType::quad;
            axisSect(curve);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kConic_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.weight = iter.conicWeight();
            curve.size = 3 * sizeof(OpPoint) + sizeof(float);
            curve.type = PathOpsV0Lib::CurveType::conic;
            axisSect(curve);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kCubic_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.pts[3] = { pts[3].fX, pts[3].fY } ; 
            curve.size = 4 * sizeof(OpPoint);
            curve.type = PathOpsV0Lib::CurveType::cubic;
            axisSect(curve);
            lastPoint = pts[3];
            hasLastPoint = true;
            break;
        case SkPath::kClose_Verb:
            break;
        case SkPath::kDone_Verb:
            break;
        }
    } while (verb != SkPath::kDone_Verb);
    if (hasLastPoint && lastPoint != curveStart) {
        curve.pts[0] = { lastPoint.fX, lastPoint.fY } ; 
        curve.pts[1] = { curveStart.fX, curveStart.fY } ; 
        curve.size = 2 * sizeof(OpPoint);
        curve.type = PathOpsV0Lib::CurveType::line;
        axisSect(curve);
    }
}

void DebugOpDraw(const std::vector<const SkPath*>& paths) {
    debugPaths.clear();
    for (auto& path : paths)
        if (path)
            DebugOpBuild(*path, debugPaths, ClipToBounds::clip, blue);
    DebugOpDraw(debugPaths);
}

void DebugOpClearPoints() {
    debugPoints.clear();
}

void DebugOpBuild(OpPoint pt) {
    DebugColorPt dPt { pt.x, pt.y, black };
    DebugOpBuild(dPt);
}

void DebugOpBuild(OpPoint pt, float t, bool opp) {
    DebugColorPt dPt { pt.x, pt.y, t, opp ? blue : darkGreen };
    DebugOpBuild(dPt);
}

void DebugOpBuild(OpPoint pt, bool opp) {
    DebugOpBuild(pt, OpNaN, opp);
}

void DebugOpBuild(OpPoint pt, float t, DebugSprite sprite) {
    DebugColorPt dPt { pt.x, pt.y, t, darkBlue, sprite };
    DebugOpBuild(dPt);
}

void DebugOpClearInputs() {
    debugInputs.clear();
}

#if 0
void DebugOpAdd(const OpInPath& input) {
    if (input.externalReference)
        DebugOpBuild(*(SkPath*)input.externalReference, debugInputs, ClipToBounds::clip, blue);
}

void DebugOpFill(const OpInPath& input, uint32_t color) {
    if (!input.externalReference)
        return;
    debugFills.clear();
    DebugOpBuild(*(SkPath*)input.externalReference, debugFills, ClipToBounds::noClip,
            color);
    DebugOpFill(debugFills);
}
#endif

void DebugOpDrawInputs() {
    DebugOpDraw(debugInputs);
}

#if 0
void DebugOpDraw(const OpOutPath* output, uint32_t color) {
    if (output->externalReference)
        DebugOpBuild(*(SkPath*)output->externalReference, debugOutputs, ClipToBounds::clip, color);
    DebugOpDraw(debugOutputs);
}
#endif

void DebugOpDrawArrowHead() {
//    SkPath path;

}

struct ColorPath {
    ColorPath(uint32_t c)
        : color(c) {
    }
    SkPath path;
    uint32_t color;
};

void DebugOpDrawSprites() {
    std::vector<ColorPath> colorPaths;
    for (auto& point : debugPoints) {
        OpPoint pt = DebugOpMap(point);
        auto colorPathIter = std::find_if(colorPaths.begin(), colorPaths.end(),
                [&point](ColorPath& path) { return path.color == point.color; });
 //       SkPath path;
        if (colorPaths.end() == colorPathIter) {
            colorPaths.emplace_back(point.color);
            colorPathIter = colorPaths.end() - 1;
        }
        if (DebugSprite::circle == point.sprite)
            OpDebugImage::addCircleToPath(pt, colorPathIter->path);
        else if (DebugSprite::diamond == point.sprite)
            OpDebugImage::addDiamondToPath(pt, colorPathIter->path);
        else if (DebugSprite::square == point.sprite)
            OpDebugImage::addSquareToPath(pt, colorPathIter->path);
        else if (DebugSprite::triangle == point.sprite)
            OpDebugImage::addTriangleToPath(pt, colorPathIter->path);
        else
            OpDebugOut("unknown sprite (%d)" + STR_E(point.sprite) + "\n");
            
    }
    for (auto& cp : colorPaths)
        OpDebugImage::drawDoublePath(cp.path, cp.color);
}

void DebugOpDrawT(bool inHex) {
    for (auto& point : debugPoints) {
        if (OpMath::IsNaN(point.t))
            continue;
        OpPoint pt = DebugOpMap(point);
        std::string ptStr = inHex ? OpDebugDumpHex(point.t) : STR((float) point.t);
        (void) OpDebugImage::drawValue(pt, ptStr);
    }
}

void DebugOpDrawValue(bool inHex) {
    for (auto& point : debugPoints) {
        OpPoint pt = DebugOpMap(point);
        std::string ptStr = "(";
        ptStr += inHex ? OpDebugDumpHex(point.x) : STR((float) point.x);
        ptStr += ", ";
        ptStr += inHex ? OpDebugDumpHex(point.y) : STR((float) point.y);
        ptStr += ")";
        (void) OpDebugImage::drawValue(pt, ptStr);
    }
}

// these edges are splits created when intersecting a pair of curves
void DebugOpDrawEdgeID(const OpEdge* edge, uint32_t color) {
    std::vector<DebugOpCurve> drawn;
    DebugOpBuild(*edge, drawn);
    for (auto& drawnEdge : drawn) {
        OpCurve curve;
        drawnEdge.mapTo(curve);
        OpPoint midTPt = curve.ptAtT(.5);
        if (OpDebugImage::drawValue(midTPt, STR(edge->id), color))
            break;
    }
}

void DebugOpDrawEdgeControlLines(const OpEdge* edge, uint32_t color) {
    if (!edge->segment)
        return;
    int ptCount = edge->curve.pointCount();
    if (ptCount <= 2)
        return;
    for (int index = 0; index < ptCount - 1; ++index) {
        DebugOpCurve src;
        src.pts[0] = { edge->curve.hullPt(index).x, edge->curve.hullPt(index).y };
        src.pts[1] = { edge->curve.hullPt(index + 1).x, edge->curve.hullPt(index + 1).y };
        src.weight = 1;
        src.type = PathOpsV0Lib::CurveType::line;
        src.id = edge->id;
        src.color = color;
        OpCurve dst;
        src.mapTo(dst);
        OpDebugImage::drawCurve(dst, color);
    }
}

void DebugOpDrawEdgeEndToEnd(const OpEdge* edge, uint32_t color) {
    DebugOpCurve src;
    src.pts[0] = { edge->startPt().x, edge->startPt().y } ;
    src.pts[1] = { edge->endPt().x, edge->endPt().y } ;
    src.weight = 1;
    src.size = 2 * sizeof(OpPoint);
    src.type = PathOpsV0Lib::CurveType::line;
    src.id = edge->id;
    src.color = color;
    OpCurve dst;
    src.mapTo(dst);
    OpDebugImage::drawCurve(dst, color);
}

void DebugOpDrawEdgeNormal(const OpEdge* edge, uint32_t color) {
    std::vector<DebugOpCurve> drawn;
    DebugOpBuild(*edge, drawn);
    for (auto& drawnEdge : drawn) {
        OpCurve curve;
        drawnEdge.mapTo(curve);
	    OpVector norm = curve.normal(.66f).normalize() * 15;
        if (!norm.isFinite() || norm == OpVector{ 0, 0 }) {
		    OpDebugOut("overflow on edge " + STR(edge->id) + "\n");
		    return;
	    }
        OpPoint midTPt = curve.ptAtT(.66f);
        if (OpDebugImage::drawEdgeNormal(norm, midTPt, edge->id, color))
            break;
    }
}

void DebugOpDrawEdgeTangent(const OpEdge* edge, uint32_t color) {
    std::vector<DebugOpCurve> drawn;
    DebugOpBuild(*edge, drawn);
    for (auto& drawnEdge : drawn) {
        OpCurve curve;
        drawnEdge.mapTo(curve);
        if (curve.c.data->start.isNearly(curve.c.data->end))
            continue;
        OpVector tan = curve.tangent(.33f).normalize() * 15;
        if (EdgeMatch::end == edge->which()) {
            tan = -tan;
            color = red;
        }
        if (!tan.isFinite() || tan == OpVector{ 0, 0 }) {
		    OpDebugOut("overflow on edge " + STR(edge->id) + "\n");
		    return;
	    }
        OpPoint midTPt = curve.ptAtT(.33f);
        if (OpDebugImage::drawTangent(tan, midTPt, edge->id, color))
            break;
    }
}

void DebugOpDrawSegmentTangent(const OpSegment* seg, uint32_t color) {
    std::vector<DebugOpCurve> drawn;
    DebugOpBuild(*seg, drawn);
    for (auto& drawnSeg : drawn) {
        OpCurve curve;
        drawnSeg.mapTo(curve);
        if (curve.c.data->start.isNearly(curve.c.data->end))
            continue;
        OpVector tan = curve.tangent(.42f).normalize() * 15;
        if (!tan.isFinite() || tan == OpVector{ 0, 0 }) {
		    OpDebugOut("overflow on seg " + STR(seg->id) + "\n");
		    return;
	    }
        OpPoint midTPt = curve.ptAtT(.42f);
        if (OpDebugImage::drawTangent(tan, midTPt, seg->id, color))
            break;
    }
}

void DebugOpDrawEdgeWinding(const OpEdge* edge, uint32_t color) {
    std::vector<DebugOpCurve> drawn;
    DebugOpBuild(*edge, drawn);
    for (auto& drawnEdge : drawn) {
        OpCurve curve;
        drawnEdge.mapTo(curve);
        if (OpDebugImage::drawEdgeWinding(curve, edge, color))
            break;
    }
}

void DebugOpDrawIntersectionID(const OpIntersection* sect, std::vector<int>& ids) {
#if OP_DEBUG
    if (ids.end() != std::find(ids.begin(), ids.end(), sect->id))
        return;
    ids.push_back(sect->id);
    OpPoint mapped = DebugOpPtToPt(sect->ptT.pt);
    (void) OpDebugImage::drawValue(mapped, STR(sect->id));
#endif
}

void DebugOpDrawSegmentID(const OpSegment* segment, std::vector<int>& ids) {
        if (ids.end() != std::find(ids.begin(), ids.end(), segment->id))
            return;
        ids.push_back(segment->id);
        std::vector<DebugOpCurve> drawn;
        DebugOpBuild(*segment, drawn);
        for (auto& drawnSeg : drawn) {
            OpCurve curve;
            drawnSeg.mapTo(curve);
            OpPoint midTPt = curve.ptAtT(.5);
            if (OpDebugImage::drawValue(midTPt, STR(segment->id), segment->disabled ? red : black))
                break;
        }
}

void DebugOpDrawPointID(const OpSegment* segment, std::vector<int>& ids) {
    // !!! incomplete
}

void DebugOpAddBounds(double left, double top, double right, double bottom) {
    if (!OpMath::IsNaN(setBounds.left)) {
        left = std::min(left, setBounds.left);
        top = std::min(top, setBounds.top);
        right = std::max(right, setBounds.right);
        bottom = std::max(bottom, setBounds.bottom);
    }
    DebugOpSetBounds(left, top, right, bottom);
}

void DebugOpSetBounds(double left, double top, double right, double bottom) {
    setBounds = { left, top, right, bottom };
    debugCenter.x = (left + right) / 2;
    debugCenter.y = (top + bottom) / 2;
    double width = right - left;
    double height = bottom - top;
    double screenWidth = debugBitmapBounds.x;
    double screenHeight = debugBitmapBounds.y;
    double minScale = std::min(screenWidth / width, screenHeight / height);
    DebugOpSetZoomScale(minScale);
}

void DebugOpSetCenter(double x, double y) {
    debugCenter = { x, y };
}

void DebugOpOffsetZoom(double dz) {
    debugZoom += dz;
}

void DebugOpSetZoom(double z) {
    debugZoom = z;
}

void DebugOpSetZoomScale(double z) {
    debugZoom = std::log2(z) * 8;
}

void DebugOpOffsetCenter(double dx, double dy) {
    debugCenter.x += dx * debugBitmapBounds.x;
    debugCenter.y += dy * debugBitmapBounds.y;
}

double DebugOpTranslate(double s) {
    return s / 4 / DebugOpGetZoomScale();
}

#endif

#endif
