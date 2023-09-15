#ifndef OpMath_DEFINED
#define OpMath_DEFINED

#include <algorithm>
#include <array>
#include <limits>

#ifndef _WIN32
#include <math.h>
#endif

#include "OpDebug.h"
#include "OpDebugColor.h"
#include "OpDebugDouble.h"
#include "OpDebugDump.h"
#include "OpDebugImage.h"

struct SkPoint;

// !!! move to OpTypes.h one day
template <typename T, size_t N> char (&ArrayCountHelper(T (&array)[N]))[N];
#define ARRAY_COUNT(array) (sizeof(ArrayCountHelper(array)))

typedef double OpCubicFloatType;

// always assume a maximum of (and reserve space for) five roots
// lines, cubics, and quads only need 2 but reserving three simplifies things,
// just as all curves reserve 4 points, even though all but cubics need 2 or 3
// then add two more in case error in root finding misses roots at zero and one
struct OpRoots {
    OpRoots() 
        : count(0) {
    }

    OpRoots(int ) = delete; // disallow old pattern that returned number of roots

    OpRoots(float one)
        : count(1) {
        roots[0] = one;
    }

    OpRoots(float one, float two) {
        count = 1 + (one != two);
        roots[0] = one;
        roots[1] = two;
    }

    // testing only
#if OP_RELEASE_TEST
    OpRoots(float one, float two, float three) {
        count = 3;
        roots[0] = one;
        roots[1] = two;
        roots[2] = three;
    }
#endif

    void add(float root) {
        OP_ASSERT(count < roots.size());
        roots[count++] = root;
    }

    void addEnd(float root) {
        if (contains(root))
            return;
        if (replaceClosest(root))
            return;
        add(root);
    }

    bool contains(float check) const {
        for (size_t index = 0; index < count; ++index) {
            if (check == roots[index])
                return true;
        }
        return false;
    }

    float get(unsigned index) {
        OP_ASSERT(index < count);
        return roots[index];
    }

    OpRoots keepValidTs(float start = 0, float end = 1);
    OpRoots keepInteriorTs(float start = 0, float end = 1);

    float* last() {
        OP_ASSERT(count > 0);
        return &roots[count - 1];
    }

    void prioritize01();

    bool replaceClosest(float root);

    void sort() {
        std::sort(roots.begin(), roots.begin() + count);
    }

#ifdef OP_DEBUG_DUMP
    void dump() const;
	void dumpDetail() const;
    void dumpHex() const;
#endif

    std::array<float, 5> roots;
    size_t count;
};

constexpr auto OpPI = 3.14159265f;
constexpr auto OpInfinity = std::numeric_limits<float>::infinity();
constexpr auto OpNaN = std::numeric_limits<float>::quiet_NaN();
constexpr auto OpMax = std::numeric_limits<int>::max();
constexpr auto OpEpsilon = std::numeric_limits<float>::epsilon();

struct OpPoint;
struct OpRect;

enum class XyChoice : uint8_t {
    inX,
    inY,
    inZ     // used by conics
};

inline int operator+(XyChoice a) {
    return static_cast<int>(a);
}

// for other enums, neither would be valued at zero for struct initialization
// here, axis is used as a component index for (x, y), so '-1' is used for neither
enum class Axis : int8_t {
    neither = -1,   // set when axis parameter is passed but has no meaning
    vertical,       // a vertical axis has a value in x; or axis is positive in y (top to bottom)
    horizontal,     // a horizontal axis has a value in y; or axis is positive in x (left to right)
    up,             // used sparsely to denote a vertical axis that points bottom to top
    left,           // used sparsely to denote a horizontal axis that points right to left
};

inline int operator+(Axis a) {
    OP_ASSERT(Axis::vertical == a || Axis::horizontal == a);
    return static_cast<int>(a);
}

inline Axis operator-(Axis a) {
    OP_ASSERT(Axis::vertical == a || Axis::horizontal == a);
    return static_cast<Axis>((int) a + 2);
}

inline Axis operator!(Axis a) {
    OP_ASSERT(Axis::vertical == a || Axis::horizontal == a);
    return static_cast<Axis>(!static_cast<int>(a));
}

// xychoice and axis are two ways of saying the same thing; often, one can be cast to the other
inline Axis toAxis(XyChoice choice) {
    OP_ASSERT(XyChoice::inX == choice || XyChoice::inY == choice);
    return static_cast<Axis>(choice);
}

inline XyChoice toXyChoice(Axis a) {
    OP_ASSERT(Axis::vertical == a || Axis::horizontal == a);
    return static_cast<XyChoice>(a);
}

struct OpVector {
    OpVector()
        : dx(OpNaN)
        , dy(OpNaN) {
    }

    OpVector(OpPoint );

    OpVector(float x, float y) {
        dx = x;
        dy = y;
    }

    OpVector operator-() const {
        return { -dx, -dy };
    }

    void operator+=(OpVector v) {
        dx += v.dx;
        dy += v.dy;
    }

    void operator-=(OpVector v) {
        dx -= v.dx;
        dy -= v.dy;
    }

    void operator/=(float s) {
        dx /= s;
        dy /= s;
    }

    void operator/=(OpVector v) {
        dx /= v.dx;
        dy /= v.dy;
    }

    void operator*=(float s) {
        dx *= s;
        dy *= s;
    }

    OpVector operator+(OpVector v) {
        OpVector result = *this;
        result += v;
        return result;
    }

    OpVector operator-(OpVector v) {
        OpVector result = *this;
        result -= v;
        return result;
    }

    OpVector operator*(float s) {
        OpVector result = *this;
        result *= s;
        return result;
    }

    friend OpVector operator*(float a, const OpVector& b) {  // must be const ref to disambiguate
        return { a * b.dx, a * b.dy };
    }

    OpVector operator/(float s) {
        OpVector result = *this;
        result /= s;
        return result;
    }

    OpVector operator/(OpVector v) {
        OpVector result = *this;
        result /= v;
        return result;
    }

    float dot(OpVector a) const {
        return dx * a.dx + dy * a.dy;
    }

    float cross(OpVector a) const {
        return dx * a.dy - dy * a.dx;
    }

    float choice(XyChoice xyChoice) const {
        OP_ASSERT(XyChoice::inZ != xyChoice);
        return *(&dx + +xyChoice);
    }

    bool isFinite() const;

    float length() const {
        return sqrtf(lengthSquared());
    }

    float lengthSquared() const {
        return dx * dx + dy * dy;
    }

    OpVector normalize(bool* overflow);

#if OP_DEBUG_DUMP
    std::string debugDump() const;
    std::string debugDumpHex() const;
#endif

    float dx;
    float dy;
};

struct OpPoint {
    OpPoint() 
        : x(OpNaN)
        , y(OpNaN) {
    }

    OpPoint(float xIn, float yIn) 
        : x(xIn)
        , y(yIn) {
    }

    OpPoint(OpVector v) 
        : x(v.dx)
        , y(v.dy) {
    }

    bool isFinite() const;

    friend OpVector operator-(OpPoint a, OpPoint b) {
        return { a.x - b.x, a.y - b.y };
    }

    friend OpVector operator-(OpVector a, OpPoint b) {
        return { a.dx - b.x, a.dy - b.y };
    }

    friend OpPoint operator+(const OpPoint& a, OpVector b) {  // must be const ref to disambiguate
        return { a.x + b.dx, a.y + b.dy };
    }

    friend bool operator==(OpPoint a, OpPoint b) {
        return a.x == b.x && a.y == b.y;
    }

    friend bool operator!=(OpPoint a, OpPoint b) {
        return a.x != b.x || a.y != b.y;
    }

    void operator+=(OpVector v) {
        x += v.dx;
        y += v.dy;
    }

    void operator+=(OpPoint v) {
        x += v.x;
        y += v.y;
    }

    void operator-=(OpVector v) {
        x -= v.dx;
        y -= v.dy;
    }

    void operator*=(float s) {
        x *= s;
        y *= s;
    }

    void operator/=(float s) {
        x /= s;
        y /= s;
    }

    OpPoint operator+(OpVector v) {
        OpPoint result = *this;
        result += v;
        return result;
    }

    OpPoint operator+(OpPoint v) {
        OpPoint result = *this;
        result += v;
        return result;
    }

    OpPoint operator-(OpVector v) {
        OpPoint result = *this;
        result -= v;
        return result;
    }

    OpPoint operator*(float s) {
        OpPoint result = *this;
        result *= s;
        return result;
    }

    friend OpPoint operator*(float a, const OpPoint& b) {  // must be const ref to disambiguate
        return { a * b.x, a * b.y };
    }

    friend OpPoint operator*(const OpPoint& b, float a) {  // must be const ref to disambiguate
        return { a * b.x, a * b.y };
    }

    OpPoint operator/(float s) {
        OpPoint result = *this;
        result /= s;
        return result;
    }

    const float* asPtr(Axis axis) const {
        return &x + +axis;
    }
    
    const float* asPtr(XyChoice xyChoice) const {
        return &x + +xyChoice;
    }

    float* asPtr(XyChoice xyChoice) {
        return &x + +xyChoice;
    }

    static bool Between(OpPoint start, OpPoint mid, OpPoint end);

    float choice(Axis axis) const {
        return *asPtr(axis);
    }

    float choice(XyChoice xyChoice) const {
        OP_ASSERT(XyChoice::inZ != xyChoice);
        return *asPtr(xyChoice);
    }

    bool isNearly(OpPoint test) const {
        return nexttowardf(test.x, x) == x && nexttowardf(test.y, y) == y;
    }

    void pin(const OpPoint , const OpPoint );
    void pin(const OpRect& );
    SkPoint toSkPoint() const;

    void zeroTiny() {  // set denormalized inputs to zero
        if (fabsf(x) < OpEpsilon)
            x = 0;
        if (fabsf(y) < OpEpsilon)
            y = 0;
    }

#if OP_DEBUG_DUMP
    OpPoint(const char*& );
    std::string debugDump() const;
    std::string debugDumpHex() const;
    void dump() const;
	void dumpDetail() const;
    void dumpHex() const;
#endif

    float x;
    float y;
};

#if OP_DEBUG_DUMP
struct OpHexPoint : OpPoint {
    OpHexPoint(int32_t xIn, int32_t yIn) {
        x = OpDebugBitsToFloat(xIn);
        y = OpDebugBitsToFloat(yIn);
    }
};
#endif

inline OpVector::OpVector(OpPoint pt)
    : dx(pt.x)
    , dy(pt.y) {
}

struct OpRect {
    OpRect()
        : left(OpNaN)
        , top(OpNaN) 
        , right(OpNaN)
        , bottom(OpNaN) {
    }

    OpRect(float l, float t, float r, float b)
        : left(l)
        , top(t)
        , right(r)
        , bottom(b) {
    }

    friend bool operator==(OpRect a, OpRect b) {
        return a.left == b.left && a.top == b.top && a.right == b.right && a.bottom == b.bottom; }
    friend bool operator!=(OpRect a, OpRect b) {
        return a.left != b.left || a.top != b.top || a.right != b.right || a.bottom != b.bottom; }
    OpPoint center() const { 
        return { (left + right) / 2, (top + bottom) / 2 }; }
    bool isFinite() const;
    float height() const { 
        return bottom - top; }

    bool intersects(const OpRect& r) const {
#if OP_DEBUG_VALIDATE
        debugValidate();
        r.debugValidate();
#endif
        return r.left <= right && left <= r.right && r.top <= bottom && top <= r.bottom;
    }

    float ltChoice(Axis axis) const { 
        return *(&left + +axis);  }
    float perimeter() const { 
        return width() + height(); }
    float rbChoice(Axis axis) const {
        return *(&right + +axis); }
    float width() const { 
        return right - left; }

#if OP_DEBUG_DUMP
    virtual ~OpRect() {}
    OpRect(const OpRect& r) = default;
    OpRect& operator=(const OpRect& ) = default;
    virtual std::string debugDump() const;
    virtual std::string debugDumpHex() const;
    virtual void dump() const;
    virtual void dumpDetail() const;
    virtual void dumpHex() const;
#endif

#if OP_DEBUG_VALIDATE
    void debugValidate() const {
        OP_ASSERT(left <= right);
        OP_ASSERT(top <= bottom);
    }
#endif

    float left;
    float top;
    float right;
    float bottom;
};

struct OpPtT {
    OpPtT()
        : t(OpNaN) {
    }

    OpPtT(OpPoint ptIn, float tIn)
        : pt(ptIn)
        , t(tIn) {
    }

    friend bool operator==(OpPtT a, OpPtT b) {
        return a.pt == b.pt && a.t == b.t;
    }

    friend bool operator!=(OpPtT a, OpPtT b) {
        return a.pt != b.pt || a.t != b.t;
    }

    static void MeetInTheMiddle(OpPtT& a, OpPtT& b){
        OpPoint mid;
        if (0 == a.t || 1 == a.t)
            mid = a.pt;
        else if (0 == b.t || 1 == b.t)
            mid = b.pt;
        else
            mid = (a.pt + b.pt) / 2;
        a.pt = mid;
        b.pt = mid;
    }

#if OP_DEBUG_DUMP
    OpPtT(const char*& );
    std::string debugDump() const;
    std::string debugDumpHex() const;
    void dump() const;
	void dumpDetail() const;
    void dumpHex() const;
#endif

    OpPoint pt;
    float t;
};

#if OP_DEBUG_DUMP
struct OpHexPtT : OpPtT {
    OpHexPtT(OpHexPoint ptIn, int32_t tIn) {
        pt = OpHexPoint(ptIn);
        t = OpDebugBitsToFloat(tIn);
    }
};
#endif

struct OpMath {
    // returns true if (a <= b <= c) || (a >= b >= c)
    static bool Between(float a, float b, float c) {
        return (a - b) * (c - b) <= 0;
    }

//    static float CubeRoot(float);
    static OpRoots CubicRootsReal(OpCubicFloatType A, OpCubicFloatType B, OpCubicFloatType C,
            OpCubicFloatType D);

    static OpRoots CubicRootsValidT(OpCubicFloatType A, OpCubicFloatType B, OpCubicFloatType C,
            OpCubicFloatType D) {
        return CubicRootsReal(A, B, C, D).keepValidTs();
    }

    static float Interp(float A, float B, float t) {
        return A + (B - A) * t;
    }

    static OpPoint Interp(OpPoint A, OpPoint B, float t) {
        return A + (B - A) * t;
    }

    // !!! could optimize with float bits trick
    static bool IsFinite(float x) {
        return OpInfinity != x && -OpInfinity != x && !OpMath::IsNaN(x);
    }

    static bool IsInt(float x) {
        OP_ASSERT(Between(0, x, 1));
        return (int) x == x;
    }

    static bool IsNaN(float x) {
        return !(x == x);
    }

    static float Pin(float a, float b, float c);

    static float PinT(float t) {
        return t < 0 ? 0 : t > 1 ? 1 : t;
    }

    static OpRoots QuadRootsReal(float A, float B, float C) {
        if (0 == A) {
            if (0 == B) {
                if (C == 0)
                    return OpRoots();
                return OpRoots(0.f);
            }
            return OpRoots(-C / B);
        }
        const float p = B / (2 * A);
        const float q = C / A;
        /* normal form: x^2 + px + q = 0 */
        const float p2 = p * p;
        if (p2 < q)
            return OpRoots();
        float sqrtl = sqrtf(p2 - q);
        return OpRoots(sqrtl - p, -sqrtl - p);
    }

    static OpRoots QuadRootsValidT(float A, float B, float C) {
        return QuadRootsReal(A, B, C).keepValidTs();
    }

    static OpRoots QuadRootsDouble(float A, float B, float C) {
        if (0 == A) {
            if (0 == B) {
                if (C == 0)
                    return OpRoots();
                return OpRoots(0.f);
            }
            return OpRoots(-C / B);
        }
        const double p = B / (2 * A);
        const double q = C / A;
        /* normal form: x^2 + px + q = 0 */
        const double p2 = p * p;
        if (p2 < q)
            return OpRoots();
        double sqrtl = sqrt(p2 - q);
        return OpRoots((float) (sqrtl - p), (float) (-sqrtl - p));
    }

    // single precision quad root is not enough for some use cases.
    // example: finding extrema of cubic (3, 0, -2/3, 1) (see loop9)
    static OpRoots QuadRootsInteriorT(float A, float B, float C) {
        return QuadRootsDouble(A, B, C).keepInteriorTs();
    }

    static void ZeroTiny(OpPoint* pts, size_t count) {
        for (size_t index = 0; index < count; ++index)
            pts[index].zeroTiny();
    }

#if OP_DEBUG
    static void DebugCompare(float a, float b);

    static void DebugCompare(OpPoint a, OpPoint b) {
        DebugCompare(a.x, b.x);
        DebugCompare(a.y, b.y);
    }
#endif
};

#endif
