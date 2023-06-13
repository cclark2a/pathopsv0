#include "OpDebug.h"
#if OP_RELEASE_TEST

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "OpCurve.h"
#include "OpDebugDouble.h"
#include "PathOps.h"

#if OP_DEBUG_IMAGE
#ifdef _WIN32
#pragma optimize( "", off )
#endif
#endif

enum class DebugColor {
    black,
    red,
    darkGreen,
    blue
};

enum class ClipToBounds {
    noClip,
    clip
};

struct DebugOpPoint {
    DebugOpPoint() 
        : x(OpNaN)
        , y(OpNaN)
        , t(OpNaN)
        , color(DebugColor::black) {
    }

    DebugOpPoint(double _x, double _y) 
        : x(_x)
        , y(_y)
        , t(OpNaN)
        , color(DebugColor::black) {
    }

    DebugOpPoint(double _x, double _y, DebugColor c) 
        : x(_x)
        , y(_y)
        , t(OpNaN)
        , color(c) {
    }

    DebugOpPoint(double _x, double _y, float _t, DebugColor c) 
        : x(_x)
        , y(_y)
        , t(_t)
        , color(c) {
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
    float t;    // for display only
    DebugColor color;
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
        : weight(1)
        , type(OpType::noType)
        , id(0)
        , pathContour(0) {
    }
    const DebugOpQuad& asQuad() const;
    const DebugOpConic& asConic() const;
    const DebugOpCubic& asCubic() const;
    DebugOpRoots axisRayHit(Axis offset, double axisIntercept) const;
    void mapTo(OpCurve& ) const;
    int pointCount() const { return static_cast<int>(type) + (type < conicType); }
    DebugOpPoint ptAtT(double t) const;
    DebugOpRoots rayIntersect(const OpDebugRay& ) const;
    void rectCurves(std::vector<DebugOpCurve>& bounded) const;
    void subDivide(double a, double b, DebugOpCurve& dest) const;
    bool tInRect(double t, const DebugOpRect& bounds) const;
    DebugOpPoint pts[4];
    double weight;
    OpType type;
    int id;     // edge or segment
    int pathContour;
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

void OpCubicPtAtT(const OpCubic& c, float f, OpPoint pt) {
    DebugOpCubic dCubic;
    for (int value = 0; value < 8; ++value)
        *(&dCubic.pts[0].x + value) = *(&c.pts[0].x + value);
    DebugOpPoint dPt = dCubic.ptAtT(f);
    pt.x = (float) dPt.x;
    pt.y = (float) dPt.y;
}

DebugOpRoots OpCubicAxisRayHit(const OpCubic& c, Axis offset, float axisIntercept) {
    DebugOpCubic dCubic;
    for (int value = 0; value < 8; ++value)
        *(&dCubic.pts[0].x + value) = *(&c.pts[0].x + value);
    return dCubic.axisRayHit(offset, axisIntercept);
}

#if OP_DEBUG_IMAGE
const DebugOpQuad& DebugOpCurve::asQuad() const { return *static_cast<const DebugOpQuad*>(this); }
const DebugOpConic& DebugOpCurve::asConic() const { return *static_cast<const DebugOpConic*>(this); }
const DebugOpCubic& DebugOpCurve::asCubic() const { return *static_cast<const DebugOpCubic*>(this); }

DebugOpRoots DebugOpCurve::axisRayHit(Axis axis, double axisIntercept) const {
    switch (type) {
    case pointType: return 0;
    case lineType: {
        double denominator = pts[1].choice(axis) - pts[0].choice(axis);
        DebugOpRoots roots(0 == denominator ? OpNaN : (axisIntercept - pts[0].choice(axis)) / denominator);
        return roots.keepValidTs();
    }
    case quadType: return asQuad().axisRayHit(axis, axisIntercept);
    case conicType: return asConic().axisRayHit(axis, axisIntercept);
    case cubicType: return asCubic().axisRayHit(axis, axisIntercept);
    default:
        assert(0);
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
    case pointType: return pts[0];
    case lineType: return DebugOpMath::Interp(pts[0], pts[1], t);    
    case quadType: return asQuad().ptAtT(t);
    case conicType: return asConic().ptAtT(t);
    case cubicType: return asCubic().ptAtT(t);
    default:
        assert(0);
        return DebugOpPoint();
    }
}

double debugZoom = 0;
constexpr double debugMargin = 2;
DebugOpPoint debugBitmapBounds { bitmapWH - debugMargin * 2, bitmapWH - debugMargin * 2 };
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

void DebugOpRecord(FILE* recordFile) {
    fprintf(recordFile, "debugZoom: %g\n", debugZoom);
    fprintf(recordFile, "debugCenter: %g, %g\n", debugCenter.x, debugCenter.y);
    fprintf(recordFile, "setBounds: %g, %g, %g, %g\n", 
            setBounds.left, setBounds.top, setBounds.right, setBounds.bottom);
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
        if (bounds.top <= pt.y && pt.y <= bounds.bottom)
            cepts.push_back(lefts.roots[index]);
    }
    for (int index = 0; index < tops.count; ++index) {
        DebugOpPoint pt = ptAtT(tops.roots[index]);
        if (bounds.left < pt.x && pt.x < bounds.right)
            cepts.push_back(tops.roots[index]);
    }
    for (int index = 0; index < rights.count; ++index) {
        DebugOpPoint pt = ptAtT(rights.roots[index]);
        if (bounds.top <= pt.y && pt.y <= bounds.bottom)
            cepts.push_back(rights.roots[index]);
    }
    for (int index = 0; index < bottoms.count; ++index) {
        DebugOpPoint pt = ptAtT(bottoms.roots[index]);
        if (bounds.left < pt.x && pt.x < bounds.right)
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
            part.type = type;
        }
        first = c;
    }
}

void DebugOpCurve::subDivide(double a, double b, DebugOpCurve& dest) const {
    switch (type) {
    case pointType: 
        dest.pts[0] = ptAtT(a); 
        return;
    case lineType: 
        dest.pts[0] = ptAtT(a); 
        dest.pts[1] = ptAtT(b); 
        return;
    case quadType: 
        return asQuad().subDivide(a, b, dest);
    case conicType: 
        return asConic().subDivide(a, b, dest);
    case cubicType: 
        return asCubic().subDivide(a, b, dest);
    default:
        assert(0);
    }
}

#include "OpSegment.h"
#include "include/core/SkColor.h"
#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"

std::vector<DebugOpCurve> debugLines;
std::vector<DebugOpCurve> debugSegments;
std::vector<DebugOpCurve> debugEdges;
std::vector<DebugOpCurve> debugFills;
std::vector<DebugOpCurve> debugInputs;
std::vector<DebugOpCurve> debugOutputs;
std::vector<DebugOpCurve> debugPaths;
std::vector<DebugOpPoint> debugPoints;  // used for path end points

OpPoint DebugOpMap(DebugOpPoint dPt) {
    double z = DebugOpGetZoomScale();
    return OpPoint( (float) ((dPt.x - debugCenter.x) * z + debugBitmapBounds.x / 2 + debugMargin),
        (float) ((dPt.y - debugCenter.y) * z + debugBitmapBounds.y / 2 + debugMargin));
}

void DebugOpCurve::mapTo(OpCurve& c) const {
    for (int i = 0; i < 4; ++i) {
        c.pts[i] = DebugOpMap(pts[i]);
    }
    c.weight = (float) weight;
    c.type = type;
}

void DebugOpDraw(std::vector<DebugOpCurve>& curves, SkColor color = SK_ColorBLACK) {
    SkPath path;
    for (auto& curve : curves) {
        OpCurve c;
        curve.mapTo(c);
        OpDebugImage::addToPath(c, path);
    }
    OpDebugImage::drawDoublePath(path, color);
}

void DebugOpFill(std::vector<DebugOpCurve>& curves, SkColor color = SK_ColorBLACK) {
    SkPath path;
    for (auto& curve : curves) {
        OpCurve c;
        curve.mapTo(c);
        OpDebugImage::addToPath(c, path);
    }
    OpDebugImage::drawDoubleFill(path, color);
}

void DebugOpDraw(const std::vector<OpDebugRay>& lines) {
    debugLines.clear();
    DebugOpRect bounds = ZoomToRect();
    for (auto& line : lines) {
        DebugOpCurve curve;
        curve.weight = 1;
        curve.type = lineType;
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
    DebugOpDraw(debugLines, SK_ColorRED);
}

void DebugOpBuild(const OpSegment& seg, std::vector<DebugOpCurve>& debugSegs) {
    DebugOpCurve curve;
    for (int i = 0; i < 4; ++i)
        curve.pts[i] = { seg.c.pts[i].x, seg.c.pts[i].y } ;
    curve.weight = seg.c.weight;
    curve.type = seg.c.type;
    curve.id = seg.id;
    curve.rectCurves(debugSegs);
}

void DebugOpBuild(const DebugOpPoint& dPt) {
    DebugOpRect bounds = ZoomToRect();
    if (!bounds.ptInRect(dPt))
        return;
    if (debugPoints.end() == std::find(debugPoints.begin(), debugPoints.end(), dPt))
        debugPoints.push_back(dPt);
}

void DebugOpBuild(const OpSegment& seg, const OpDebugRay& ray) {
    DebugOpCurve curve;
    for (int i = 0; i < 4; ++i)
        curve.pts[i] = { seg.c.pts[i].x, seg.c.pts[i].y } ;
    curve.weight = seg.c.weight;
    curve.type = seg.c.type;
    DebugOpRoots roots = curve.rayIntersect(ray);
    for (int index = 0; index < roots.count; ++index) {
        DebugOpPoint pt = curve.ptAtT(roots.roots[index]);
        pt.t = roots.roots[index];
        pt.color = DebugColor::black;
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

void DebugOpBuild(const OpEdge& edge, std::vector<DebugOpCurve>& debugEs) {
    DebugOpCurve curve;
    OpEdge copy(edge);
    const OpCurve& c = copy.setCurve();
    for (int i = 0; i < 4; ++i)
        curve.pts[i] = { c.pts[i].x, c.pts[i].y } ;
    curve.weight = c.weight;
    curve.type = c.type;
    curve.id = edge.id;
    curve.rectCurves(debugEs);
}

void DebugOpBuild(const OpEdge& edge, const OpDebugRay& ray) {
    DebugOpCurve curve;
    OpEdge copy(edge);
    const OpCurve& c = copy.setCurve();
    for (int i = 0; i < 4; ++i)
        curve.pts[i] = { c.pts[i].x, c.pts[i].y } ;
    curve.weight = c.weight;
    curve.type = c.type;
    curve.id = edge.id;
    DebugOpRoots roots = curve.rayIntersect(ray);
    for (int index = 0; index < roots.count; ++index) {
        DebugOpPoint pt = curve.ptAtT(roots.roots[index]);
        pt.t = edge.start.t + (edge.end.t - edge.start.t) * roots.roots[index];
        pt.color = DebugColor::black;
        DebugOpBuild(pt);
    }
}

OpPoint DebugOpPtToPt(OpPoint src) {
    return DebugOpMap(DebugOpPoint(src.x, src.y));
}

void DebugOpClearEdges() {
    debugEdges.clear();
}

void DebugOpAdd(const OpEdge* edge) {
    DebugOpBuild(*edge, debugEdges);
}

void DebugOpDrawEdges() {
    DebugOpDraw(debugEdges);
}

void DebugOpDraw(const std::vector<OpEdge>& edges) {
    for (auto edge : edges)
        DebugOpBuild(edge, debugEdges);
    DebugOpDraw(debugEdges);
}

void DebugOpDraw(const std::vector<const OpEdge*>& edges) {
    for (auto edge : edges)
        DebugOpBuild(*edge, debugEdges);
    DebugOpDraw(debugEdges);
}

void DebugOpBuild(const SkPath& path, std::vector<DebugOpCurve>& debugPs, ClipToBounds clip) {
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
                curve.type = OpType::lineType;
                if (ClipToBounds::clip == clip)
                    curve.rectCurves(debugPs);
                else
                    debugPs.emplace_back(curve);
                hasLastPoint = false;
            }
            curveStart = pts[0];
            curve.pathContour = ++nextContourID;
            continue;
        case SkPath::kLine_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.type = OpType::lineType;
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
            curve.type = OpType::quadType;
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
            curve.weight = iter.conicWeight();
            curve.type = OpType::conicType;
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
            curve.type = OpType::cubicType;
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
        curve.type = OpType::lineType;
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
            DebugOpPoint pt = curve.ptAtT(roots.roots[index]);
            pt.t = roots.roots[index];
            pt.color = DebugColor::black;
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
                curve.type = OpType::lineType;
                axisSect(curve);
                hasLastPoint = false;
            }
            curveStart = pts[0];
            curve.pathContour = ++nextContourID;
            continue;
        case SkPath::kLine_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.type = OpType::lineType;
            axisSect(curve);
            lastPoint = pts[1];
            hasLastPoint = true;
            break;
        case SkPath::kQuad_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.type = OpType::quadType;
            axisSect(curve);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kConic_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.weight = iter.conicWeight();
            curve.type = OpType::conicType;
            axisSect(curve);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kCubic_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.pts[3] = { pts[3].fX, pts[3].fY } ; 
            curve.type = OpType::cubicType;
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
        curve.type = OpType::lineType;
        axisSect(curve);
    }
}

void DebugOpDraw(const std::vector<const SkPath*>& paths) {
    debugPaths.clear();
    for (auto& path : paths)
        if (path)
            DebugOpBuild(*path, debugPaths, ClipToBounds::clip);
    DebugOpDraw(debugPaths, SK_ColorBLUE);
}

void DebugOpClearPoints() {
    debugPoints.clear();
}

void DebugOpBuild(OpPoint pt) {
    DebugOpPoint dPt { pt.x, pt.y, DebugColor::black };
    DebugOpRect bounds = ZoomToRect();
    if (!bounds.ptInRect(dPt))
        return;
    if (debugPoints.end() == std::find(debugPoints.begin(), debugPoints.end(), dPt))
        debugPoints.push_back(dPt);
}

void DebugOpBuild(OpPoint pt, float t, bool opp) {
    DebugOpPoint dPt { pt.x, pt.y, t, opp ? DebugColor::blue : DebugColor::darkGreen };
    DebugOpRect bounds = ZoomToRect();
    if (!bounds.ptInRect(dPt))
        return;
    if (debugPoints.end() == std::find(debugPoints.begin(), debugPoints.end(), dPt))
        debugPoints.push_back(dPt);
}

void DebugOpBuild(OpPoint pt, bool opp) {
    DebugOpBuild(pt, OpNaN, opp);
}

void DebugOpClearInputs() {
    debugInputs.clear();
}

void DebugOpAdd(const OpInPath& input) {
    if (input.skPath)
        DebugOpBuild(*input.skPath, debugInputs, ClipToBounds::clip);
}

void DebugOpFill(const OpInPath& input, uint32_t color) {
    if (!input.skPath)
        return;
    debugFills.clear();
    DebugOpBuild(*input.skPath, debugFills, ClipToBounds::noClip);
    DebugOpFill(debugFills, color);
}

void DebugOpDrawInputs() {
    DebugOpDraw(debugInputs, SK_ColorBLUE);
}

void DebugOpDraw(const std::vector<OpOutPath>& outputs) {
    debugOutputs.clear();
    for (auto& output : outputs)
        if (output.skPath)
            DebugOpBuild(*output.skPath, debugOutputs, ClipToBounds::clip);
    DebugOpDraw(debugOutputs, SK_ColorBLUE);
}

void DebugOpDrawArrowHead() {
    SkPath path;

}

void DebugOpDrawDiamond() {
    SkPath blackPath;
    SkPath bluePath;
    SkPath darkGreenPath;
    for (auto& point : debugPoints) {
        OpPoint pt = DebugOpMap(point);
        SkPath& path = DebugColor::black == point.color ? blackPath :
                DebugColor::blue == point.color ? bluePath : darkGreenPath;
        OpDebugImage::addDiamondToPath(pt, path);
    }
    OpDebugImage::drawDoublePath(blackPath, SK_ColorBLACK);
    OpDebugImage::drawDoublePath(bluePath, SK_ColorBLUE, true);
    OpDebugImage::drawDoublePath(darkGreenPath, 0x80008000, true);
}

void DebugOpDrawT(bool inHex, int precision) {
    for (auto& point : debugPoints) {
        if (OpMath::IsNaN(point.t))
            continue;
        OpPoint pt = DebugOpMap(point);
        std::string ptStr = inHex ? OpDebugDumpHex(point.t) : 
                OpDebugToString((float) point.t, precision);
        (void) OpDebugImage::drawValue(pt, ptStr);
    }
}

bool DebugOpHasT() {
    for (auto& point : debugPoints) {
        if (!OpMath::IsNaN(point.t))
            return true;
    }
    return false;
}

void DebugOpDrawValue(bool inHex, int precision) {
    for (auto& point : debugPoints) {
        OpPoint pt = DebugOpMap(point);
        std::string ptStr = "(";
        ptStr += inHex ? OpDebugDumpHex(point.x) : OpDebugToString((float) point.x, precision);
        ptStr += ", ";
        ptStr += inHex ? OpDebugDumpHex(point.y) : OpDebugToString((float) point.y, precision);
        ptStr += ")";
        (void) OpDebugImage::drawValue(pt, ptStr);
    }
}

void DebugOpDrawEdgeIDs(const std::vector<const OpEdge*>& edges, std::vector<int>& ids) {
    for (auto edge : edges)
        DebugOpDrawEdgeID(edge, ids, edge->winding.visible() ? SK_ColorBLACK : SK_ColorRED);
}

// these edges are splits created when intersecting a pair of curves
void DebugOpDrawEdgeID(const OpEdge* edge, std::vector<int>& ids, uint32_t color) {
    if (ids.end() != std::find(ids.begin(), ids.end(), edge->id))
        return;
    ids.push_back(edge->id);
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

void DebugOpDrawEdgeNormal(const OpEdge* edge, std::vector<int>& ids, uint32_t color) {
    if (ids.end() != std::find(ids.begin(), ids.end(), edge->id))
        return;
    ids.push_back(edge->id);
    std::vector<DebugOpCurve> drawn;
    DebugOpBuild(*edge, drawn);
    for (auto& drawnEdge : drawn) {
        OpCurve curve;
        drawnEdge.mapTo(curve);
	    bool overflow;
	    OpVector norm = curve.normal(.66f).normalize(&overflow) * 15;
	    if (overflow) {
		    OpDebugOut("overflow on edge " + STR(edge->id) + "\n");
		    return;
	    }
        OpPoint midTPt = curve.ptAtT(.66f);
        if (OpDebugImage::drawEdgeNormal(norm, midTPt, edge->id, color))
            break;
    }
}

void DebugOpDrawEdgeWinding(const OpEdge* edge, std::vector<int>& ids, uint32_t color) {
    if (ids.end() != std::find(ids.begin(), ids.end(), edge->id))
        return;
    ids.push_back(edge->id);
    std::vector<DebugOpCurve> drawn;
    DebugOpBuild(*edge, drawn);
    for (auto& drawnEdge : drawn) {
        OpCurve curve;
        drawnEdge.mapTo(curve);
	    bool overflow;
	    OpVector norm = curve.normal(.58f).normalize(&overflow) * 15;
	    if (overflow) {
		    OpDebugOut("overflow on edge " + STR(edge->id) + "\n");
		    return;
	    }
        OpPoint midTPt = curve.ptAtT(.58f);
        if (OpDebugImage::drawEdgeWinding(norm, midTPt, edge, color))
            break;
    }
}

void DebugOpDrawIntersectionID(const OpIntersection* sect, std::vector<int>& ids) {
    if (ids.end() != std::find(ids.begin(), ids.end(), sect->id))
        return;
    ids.push_back(sect->id);
    OpPoint mapped = DebugOpPtToPt(sect->ptT.pt);
    (void) OpDebugImage::drawValue(mapped, STR(sect->id));
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
            if (OpDebugImage::drawValue(midTPt, STR(segment->id), segment->winding.visible()
                    && OpType::pointType != segment->c.type ? SK_ColorBLACK : SK_ColorRED))
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
