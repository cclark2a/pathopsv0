#include "OpDebug.h"
#if OP_RELEASE_TEST

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "OpCurve.h"
#include "OpDebugDouble.h"

#if OP_DEBUG_IMAGE
#pragma optimize( "", off )
#endif

enum class DebugColor {
    black,
    red,
    darkGreen,
    blue
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

typedef std::array<double, 5> debugRootCellar;

constexpr double PI = 3.1415926535897931;

struct DebugOpMath {
    static int CubicRootsReal(double A, double B, double C, double D, debugRootCellar& s) {
        if (0 == A)
            return QuadRootsReal(B, C, D, s);
        if (0 == D) {  // 0 is one root
            int num = QuadRootsReal(A, B, C, s);
            for (int i = 0; i < num; ++i) {
                if (0 == s[i])
                    return num;
            }
            s[num++] = 0;
            return num;
        }
        if (0 == A + B + C + D) {  // 1 is one root
            int num = QuadRootsReal(A, A + B, -D, s);
            for (int i = 0; i < num; ++i) {
                if (1 == s[i]) {
                    return num;
                }
            }
            s[num++] = 1;
            return num;
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
        double* roots = &s[0];
        if (R2MinusQ3 < 0) {   // we have 3 real roots
            // the divide/root can, due to finite precisions, be slightly outside of -1...1
            double theta = acos(std::max(std::min(1., R / sqrt(Q3)), -1.));
            double neg2RootQ = -2 * sqrt(Q);
            r = neg2RootQ * cos(theta / 3) - adiv3;
            *roots++ = r;
            r = neg2RootQ * cos((theta + 2 * PI) / 3) - adiv3;
            if (s[0] != r)
                *roots++ = r;
            r = neg2RootQ * cos((theta - 2 * PI) / 3) - adiv3;
            if (s[0] != r && (roots - &s[0] == 1 || s[1] != r))
                *roots++ = r;
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
            *roots++ = r;
            if (R2 == Q3) {
                r = -A / 2 - adiv3;
                if (s[0] != r) {
                    *roots++ = r;
                }
            }
        }
        return static_cast<int>(roots - &s[0]);
    }

    static int CubicRootsValidT(double A, double B, double C, double D, debugRootCellar& s) {
        int realRoots = CubicRootsReal(A, B, C, D, s);
        int foundRoots = KeepValidTs(s, realRoots);
        return foundRoots;
    }

    static double Interp(double A, double B, double t) {
        return A + (B - A) * t;
    }

    static DebugOpPoint Interp(DebugOpPoint A, DebugOpPoint B, double t) {
        return A + (B - A) * t;
    }

    static int KeepValidTs(debugRootCellar& s, int realRoots) {
        size_t foundRoots = 0;
        for (int index = 0; index < realRoots; ++index) {
            double tValue = s[index];
            if (IsNaN(tValue) || 0 > tValue || tValue > 1)
                continue;
            for (size_t idx2 = 0; idx2 < foundRoots; ++idx2) {
                if (s[idx2] == tValue) {
                    goto notUnique;
                }
            }
            s[foundRoots++] = tValue;
        notUnique:
            ;
        }
        return foundRoots;
    }

    static bool IsNaN(double x) {
        return !(x == x);
    }

    static int QuadRootsReal(double A, double B, double C, debugRootCellar & s) {
        if (!A) {
            if (0 == B) {
                s[0] = 0;
                return C == 0;
            }
            s[0] = -C / B;
            return 1;
        }
        const double p = B / (2 * A);
        const double q = C / A;
        /* normal form: x^2 + px + q = 0 */
        const double p2 = p * p;
        if (p2 < q)
            return 0;
        double sqrtl = sqrt(p2 - q);
        s[0] = sqrtl - p;
        s[1] = -sqrtl - p;
        return 1 + (s[0] != s[1]);
    }

    static int QuadRootsValidT(double A, double B, double C, debugRootCellar& s) {
        int realRoots = QuadRootsReal(A, B, C, s);
        int foundRoots = KeepValidTs(s, realRoots);
        return foundRoots;
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
    int axisRayHit(Axis offset, double axisIntercept, debugRootCellar& cepts) const;
    void mapTo(OpCurve& ) const;
    DebugOpPoint ptAtT(double t) const;
    void rectCurves(std::vector<DebugOpCurve>& bounded);
    void subDivide(double a, double b, DebugOpCurve& dest);
    bool tInRect(double t, const DebugOpRect& bounds);
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
    int axisRayHit(Axis axis, double axisIntercept, debugRootCellar& cepts) const {
        DebugOpQuadCoefficients coeff = coefficients(axis);
        coeff.c -= axisIntercept;
        return DebugOpMath::QuadRootsValidT(coeff.a, coeff.b, coeff.c, cepts);
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
    int axisRayHit(Axis offset, double axisIntercept, debugRootCellar& cepts) const {
        DebugOpQuadCoefficients coeff = coefficients(offset, axisIntercept);
        return DebugOpMath::QuadRootsValidT(coeff.a, coeff.b, coeff.c - axisIntercept, cepts);
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
    int axisRayHit(Axis offset, double axisIntercept, debugRootCellar& cepts) const {
        DebugOpCubicCoefficients coeff = coefficients(offset);
        coeff.d -= axisIntercept;
        return DebugOpMath::CubicRootsValidT(coeff.a, coeff.b, coeff.c, coeff.d, cepts);
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

void OpCubicPtAtT(const OpCubic& c, float f, OpPoint& pt) {
    DebugOpCubic dCubic;
    for (int value = 0; value < 8; ++value)
        *(&dCubic.pts[0].x + value) = *(&c.pts[0].x + value);
    DebugOpPoint dPt = dCubic.ptAtT(f);
    pt.x = (float) dPt.x;
    pt.y = (float) dPt.y;
}

void OpCubicAxisRayHit(const OpCubic& c, Axis offset, float axisIntercept, 
        std::array<float, 5>& cepts, int& roots) {
    DebugOpCubic dCubic;
    for (int value = 0; value < 8; ++value)
        *(&dCubic.pts[0].x + value) = *(&c.pts[0].x + value);
    debugRootCellar dCepts;
    roots = dCubic.axisRayHit(offset, axisIntercept, dCepts);
    for (int index = 0; index < roots; ++index)
        cepts[index] = (float) dCepts[index];
}

#if OP_DEBUG_IMAGE
const DebugOpQuad& DebugOpCurve::asQuad() const { return *static_cast<const DebugOpQuad*>(this); }
const DebugOpConic& DebugOpCurve::asConic() const { return *static_cast<const DebugOpConic*>(this); }
const DebugOpCubic& DebugOpCurve::asCubic() const { return *static_cast<const DebugOpCubic*>(this); }

int DebugOpCurve::axisRayHit(Axis axis, double axisIntercept, debugRootCellar& cepts) const {
    switch (type) {
    case pointType: return 0;
    case lineType: {
        double denominator = pts[1].choice(axis) - pts[0].choice(axis);
        cepts[0] = 0 == denominator ? OpNaN : (axisIntercept - pts[0].choice(axis)) / denominator;
        return DebugOpMath::KeepValidTs(cepts, 1);
    }
    case quadType: return asQuad().axisRayHit(axis, axisIntercept, cepts);
    case conicType: return asConic().axisRayHit(axis, axisIntercept, cepts);
    case cubicType: return asCubic().axisRayHit(axis, axisIntercept, cepts);
    default:
        assert(0);
    }
    return 0;
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
DebugOpPoint debugBitmapBounds = { bitmapWH - debugMargin * 2, bitmapWH - debugMargin * 2 };
DebugOpPoint debugCenter = { debugBitmapBounds.x / 2, debugBitmapBounds.y / 2 };
DebugOpRect setBounds;

void DebugOpResetFocus() {
    debugZoom = 0;
    debugCenter = { debugBitmapBounds.x / 2, debugBitmapBounds.y / 2 };
    setBounds.reset();
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

bool DebugOpCurve::tInRect(double t, const DebugOpRect& bounds) {
    DebugOpPoint pt = ptAtT(t);
    return bounds.ptInRect(pt);
}

// generate curve scaled from one rect to another
void DebugOpCurve::rectCurves(std::vector<DebugOpCurve>& bounded) {
//    OpDebugBreak(this, 387, true);
    DebugOpRect bounds = ZoomToRect();
    debugRootCellar lefts, tops, rights, bottoms;
    int leftRoots = axisRayHit(Axis::vertical, bounds.left, lefts);
    int topRoots = axisRayHit(Axis::horizontal, bounds.top, tops);
    int rightRoots = axisRayHit(Axis::vertical, bounds.right, rights);
    int bottomRoots = axisRayHit(Axis::horizontal, bounds.bottom, bottoms);
    std::vector<double> cepts;
    if (tInRect(0, bounds))
        cepts.push_back(0);
    for (int index = 0; index < leftRoots; ++index) {
        DebugOpPoint pt = ptAtT(lefts[index]);
        if (bounds.top <= pt.y && pt.y <= bounds.bottom)
            cepts.push_back(lefts[index]);
    }
    for (int index = 0; index < topRoots; ++index) {
        DebugOpPoint pt = ptAtT(tops[index]);
        if (bounds.left < pt.x && pt.x < bounds.right)
            cepts.push_back(tops[index]);
    }
    for (int index = 0; index < rightRoots; ++index) {
        DebugOpPoint pt = ptAtT(rights[index]);
        if (bounds.top <= pt.y && pt.y <= bounds.bottom)
            cepts.push_back(rights[index]);
    }
    for (int index = 0; index < bottomRoots; ++index) {
        DebugOpPoint pt = ptAtT(bottoms[index]);
        if (bounds.left < pt.x && pt.x < bounds.right)
            cepts.push_back(bottoms[index]);
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

void DebugOpCurve::subDivide(double a, double b, DebugOpCurve& dest) {
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
std::vector<DebugOpCurve> debugNormals;
std::vector<DebugOpCurve> debugSegments;
std::vector<DebugOpCurve> debugEdges;
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

void DebugOpDraw(const std::vector<OpRay>& lines) {
    debugLines.clear();
    DebugOpRect bounds = ZoomToRect();
    for (auto& line : lines) {
        DebugOpCurve curve;
        curve.weight = 1;
        curve.type = lineType;
        if (Axis::horizontal == line.axis) {
            if (bounds.top > line.value || line.value > bounds.bottom)
                continue;
            curve.pts[0] = { bounds.left, line.value } ;
            curve.pts[1] = { bounds.right, line.value } ;
        } else {
            if (bounds.left > line.value || line.value > bounds.right)
                continue;
            curve.pts[0] = { line.value, bounds.top } ;
            curve.pts[1] = { line.value, bounds.bottom } ;
        }
        debugLines.push_back(curve);
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

void DebugOpBuild(const OpSegment& seg, Axis axis, float ray) {
    DebugOpCurve curve;
    for (int i = 0; i < 4; ++i)
        curve.pts[i] = { seg.c.pts[i].x, seg.c.pts[i].y } ;
    curve.weight = seg.c.weight;
    curve.type = seg.c.type;
    debugRootCellar cepts;
    int roots = curve.axisRayHit(axis, (double) ray, cepts);
    for (int index = 0; index < roots; ++index) {
        DebugOpPoint pt = curve.ptAtT(cepts[index]);
        pt.t = cepts[index];
        pt.color = DebugColor::black;
        DebugOpBuild(pt);
    }
}

void DebugOpDraw(const std::vector<const OpSegment*>& segments) {
    debugSegments.clear();
    for (auto seg : segments)
        DebugOpBuild(*seg, debugSegments);
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

void DebugOpBuild(const OpEdge& edge, Axis axis, float ray) {
    DebugOpCurve curve;
    OpEdge copy(edge);
    const OpCurve& c = copy.setCurve();
    for (int i = 0; i < 4; ++i)
        curve.pts[i] = { c.pts[i].x, c.pts[i].y } ;
    curve.weight = c.weight;
    curve.type = c.type;
    curve.id = edge.id;
    debugRootCellar cepts;
    int roots = curve.axisRayHit(axis, (double) ray, cepts);
    for (int index = 0; index < roots; ++index) {
        DebugOpPoint pt = curve.ptAtT(cepts[index]);
        pt.t = edge.start.t + (edge.end.t - edge.start.t) * cepts[index];
        pt.color = DebugColor::black;
        DebugOpBuild(pt);
    }
}

void DebugOpPtToPt(const OpPoint& src, OpPoint& dst) {
    dst = DebugOpMap(DebugOpPoint(src.x, src.y));
}

void DebugOpClearEdges() {
    debugEdges.clear();
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

void DebugOpBuild(const SkPath& path, std::vector<DebugOpCurve>& debugPs) {
    SkPath::RawIter iter(path);
    SkPoint curveStart = {0, 0};
    SkPath::Verb verb;
    SkPoint lastPoint = {0, 0};
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
                curve.rectCurves(debugPs);
                hasLastPoint = false;
            }
            curveStart = pts[0];
            curve.pathContour = ++nextContourID;
            continue;
        case SkPath::kLine_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.type = OpType::lineType;
            curve.rectCurves(debugPs);
            lastPoint = pts[1];
            hasLastPoint = true;
            break;
        case SkPath::kQuad_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.type = OpType::quadType;
            curve.rectCurves(debugPs);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kConic_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.weight = iter.conicWeight();
            curve.type = OpType::conicType;
            curve.rectCurves(debugPs);
            lastPoint = pts[2];
            hasLastPoint = true;
            break;
        case SkPath::kCubic_Verb:
            curve.pts[0] = { pts[0].fX, pts[0].fY } ; 
            curve.pts[1] = { pts[1].fX, pts[1].fY } ; 
            curve.pts[2] = { pts[2].fX, pts[2].fY } ; 
            curve.pts[3] = { pts[3].fX, pts[3].fY } ; 
            curve.type = OpType::cubicType;
            curve.rectCurves(debugPs);
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
        curve.rectCurves(debugPs);
    }
}

void DebugOpBuild(const SkPath& path, Axis axis, float ray) {
    auto axisSect = [&](const DebugOpCurve& curve) {  // lambda
        debugRootCellar cepts;
        int roots = curve.axisRayHit(axis, (double) ray, cepts);
        for (int index = 0; index < roots; ++index) {
            DebugOpPoint pt = curve.ptAtT(cepts[index]);
            pt.t = cepts[index];
            pt.color = DebugColor::black;
            DebugOpBuild(pt);
        }
    };
    SkPath::RawIter iter(path);
    SkPoint curveStart = {0, 0};
    SkPath::Verb verb;
    SkPoint lastPoint = {0, 0};
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
            DebugOpBuild(*path, debugPaths);
    DebugOpDraw(debugPaths);
}

void DebugOpClearPoints() {
    debugPoints.clear();
}

void DebugOpBuild(const OpPoint& pt) {
    DebugOpPoint dPt = { pt.x, pt.y, DebugColor::black };
    DebugOpRect bounds = ZoomToRect();
    if (!bounds.ptInRect(dPt))
        return;
    if (debugPoints.end() == std::find(debugPoints.begin(), debugPoints.end(), dPt))
        debugPoints.push_back(dPt);
}

void DebugOpBuild(const OpPoint& pt, float t, bool opp) {
    DebugOpPoint dPt = { pt.x, pt.y, t, opp ? DebugColor::blue : DebugColor::darkGreen };
    DebugOpRect bounds = ZoomToRect();
    if (!bounds.ptInRect(dPt))
        return;
    if (debugPoints.end() == std::find(debugPoints.begin(), debugPoints.end(), dPt))
        debugPoints.push_back(dPt);
}

void DebugOpBuild(const OpPoint& pt, bool opp) {
    DebugOpBuild(pt, OpNaN, opp);
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
    OpDebugImage::drawDoublePath(blackPath);
    OpDebugImage::drawDoublePath(bluePath, SK_ColorBLUE);
    OpDebugImage::drawDoublePath(darkGreenPath, 0x80008000);
}

void DebugOpDrawT(bool inHex, int precision) {
    for (auto& point : debugPoints) {
        if (OpMath::IsNaN(point.t))
            continue;
        OpPoint pt = DebugOpMap(point);
        std::string ptStr = inHex ? OpDebugDumpHex(point.t) : 
                OpDebugToString((float) point.t, precision);
        OpDebugImage::drawValue(pt, ptStr);
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
        OpDebugImage::drawValue(pt, ptStr);
    }
}

void DebugOpDrawEdgeIDs(const std::vector<const OpEdge*>& edges, std::vector<int>& ids) {
    for (auto edge : edges) {
        if (ids.end() != std::find(ids.begin(), ids.end(), edge->id))
            continue;
        ids.push_back(edge->id);
        std::vector<DebugOpCurve> drawn;
        DebugOpBuild(*edge, drawn);
        for (auto& drawnEdge : drawn) {
            OpCurve curve;
            drawnEdge.mapTo(curve);
            OpPoint midTPt = curve.ptAtT(.5);
            OpDebugImage::drawValue(midTPt, STR(edge->id), 
                    edge->winding.visible() ? SK_ColorBLACK : SK_ColorRED);
        }
    }
}

// these edges are splits created when intersecting a pair of curves
void DebugOpDrawEdgeIDs(const std::vector<OpEdge>& edges, std::vector<int>& ids, bool opp) {
    for (const auto& edge : edges) {
        if (ids.end() != std::find(ids.begin(), ids.end(), edge.id))
            continue;
        ids.push_back(edge.id);
        std::vector<DebugOpCurve> drawn;
        DebugOpBuild(edge, drawn);
        for (auto& drawnEdge : drawn) {
            OpCurve curve;
            drawnEdge.mapTo(curve);
            OpPoint midTPt = curve.ptAtT(.5);
            OpDebugImage::drawValue(midTPt, STR(edge.id),       // dark green
                    edge.winding.visible() ? opp ? SK_ColorBLUE : 0xFF008000 : SK_ColorRED);
        }
    }
}

void DebugOpDrawIntersectionIDs(const std::vector<const OpIntersection*>& intersections,
        std::vector<int>& ids) {
    for (auto sect : intersections) {
        if (ids.end() != std::find(ids.begin(), ids.end(), sect->id))
            continue;
        ids.push_back(sect->id);
        OpPoint mapped;
        DebugOpPtToPt(sect->ptT.pt, mapped);
        OpDebugImage::drawValue(mapped, STR(sect->id));
    }
}

void DebugOpDrawSegmentIDs(const std::vector<const OpSegment*>& segments, std::vector<int>& ids) {
    for (auto seg : segments) {
        if (ids.end() != std::find(ids.begin(), ids.end(), seg->id))
            continue;
        ids.push_back(seg->id);
        std::vector<DebugOpCurve> drawn;
        DebugOpBuild(*seg, drawn);
        for (auto& drawnSeg : drawn) {
            OpCurve curve;
            drawnSeg.mapTo(curve);
            OpPoint midTPt = curve.ptAtT(.5);
            OpDebugImage::drawValue(midTPt, STR(seg->id));
        }
    }
}

void DebugOpDrawPointIDs(const std::vector<const OpSegment*>& segments, std::vector<int>& ids) {
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
