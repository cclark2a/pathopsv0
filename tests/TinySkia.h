// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef TinySkia_DEFINED
#define TinySkia_DEFINED

#include "OpMath.h"
#include "SkiaEnumSkPathOp.h"  // !!! remove this eventually

struct OpContext;

#if OP_DEBUG
#define SkDEBUGCODE(x) x
#else
#define SkDEBUGCODE(x)
#endif
#define SK_INIT_TO_AVOID_WARNING    = 0

inline float SkBits2Float(int32_t i) {
	return OpDebugBitsToFloat(i); }

inline int SkScalarRoundToInt(float f) {
	return (int) floorf(f + 0.5f); }

inline float SkDoubleToScalar(double d) {
	return (float) d; }

// random must be exactly the same as Skia for the same test values to be generated
struct SkRandom {
    SkRandom() { init(0); }

    uint32_t nextU() {
        fK = kKMul * (fK & 0xffff) + (fK >> 16);
        fJ = kJMul * (fJ & 0xffff) + (fJ >> 16);
        return (((fK << 16) | (fK >> 16)) + fJ);
    }

	int nextRangeU(uint32_t min, uint32_t max) { 
        uint32_t range = max - min + 1;
        if (0 == range)
            return nextU();
        else
            return min + nextU() % range;
	}

    void init(uint32_t seed) {
        fK = NextLCG(seed);
        if (0 == fK)
            fK = NextLCG(fK);
        fJ = NextLCG(fK);
        if (0 == fJ)
            fJ = NextLCG(fJ);
    }

    static uint32_t NextLCG(uint32_t seed) { return kMul*seed + kAdd; }

	enum {
        kMul = 1664525,
        kAdd = 1013904223,
        kKMul = 30345,
        kJMul = 18000,
    };

    uint32_t fK;
    uint32_t fJ;
};

struct SkPoint {
    friend bool operator!=(SkPoint a, SkPoint b) {
        return a.fX != b.fX || a.fY != b.fY;
    }
	float fX;
	float fY;
};

struct SkMatrix {
	static SkMatrix MakeAll(float scaleX, float skewX,  float transX,
                            float skewY,  float scaleY, float transY,
                            float pers0, float pers1, float pers2);
	void mapPoints(OpPoint* pts, int count) const;
	float m[2][3];
};

struct SkRect {
	static SkRect MakeWH(float w, float h) {
		return { 0, 0, w, h }; }
	void setLTRB(float left, float top, float right, float bottom) {
        fLeft = left; fTop = top; fRight = right; fBottom = bottom; }

	float fLeft;
	float fTop;
	float fRight;
	float fBottom;
};

enum class TinyOps {
	simplify = -1,
	difference,  // numbered as zero agrees with historic Skia
	intersect,
	unite,  // union conflicts
	exclusiveOr,  // xor conflicts
	reverseDifference
};

inline TinyOps operator++(TinyOps& tinyOp) {
	return tinyOp = (TinyOps) ((int) tinyOp + 1);
}

enum class SkPathFillType {
	kWinding,
	kEvenOdd,
	kInverseWinding,
	kInverseEvenOdd,
};

enum class SkPathDirection {
	kCW,
	kCCW,
};

enum class TinyType {
	line,
	quad,
	conic,
	cubic,
};

struct TinyCurve {
	OpPoint lastPt() const;
	size_t pointCount() const;

	OpPoint pts[4];
	float weight;
	TinyType type;
};

class SkPath {
public:
	enum Verb {
		kMove_Verb,
		kLine_Verb,
		kQuad_Verb,
		kConic_Verb,
		kCubic_Verb,
		kClose_Verb,
		kDone_Verb
	};
	struct RawIter {
		RawIter(const SkPath& );
		float conicWeight() const { return w; }
		Verb next(SkPoint* );

		const SkPath& path;
		OpPoint first;
		size_t index;
		float w;
		bool nextClose;
		bool nextMove;
	};

	void addCircle(float , float , float , SkPathDirection dir = SkPathDirection::kCW);
	void addPath(SkPath const& p);
	void addPath(SkPath const& p, const SkMatrix& m);
	void addRect(float, float, float, float, SkPathDirection dir = SkPathDirection::kCW);
	void addRect(const SkRect& rect, SkPathDirection dir = SkPathDirection::kCW) {
		addRect(rect.fLeft, rect.fTop, rect.fRight, rect.fBottom, dir); }
	void addRoundRect(const SkRect& rect, float rx, float ry,
			SkPathDirection dir = SkPathDirection::kCW);
	void arcTo(const SkRect& , float startAngle, float sweepAngle, bool forceMoveTo);
	bool isInverseFillType() const { return SkPathFillType::kInverseWinding == fFillType
			|| SkPathFillType::kInverseEvenOdd == fFillType; }
	SkRect getBounds() const;
	void reset();
	bool isEmpty() const;
	bool isFinite() const;
	const SkPath& makeTransform(SkMatrix const & m);
	void moveTo(SkPoint p) { return moveTo(p.fX, p.fY); }
	void moveTo(float,float);
	void lineTo(SkPoint p) { return lineTo(p.fX, p.fY); }
	void lineTo(float,float);
	void quadTo(SkPoint p1,SkPoint p2) { return quadTo(p1.fX, p1.fY, p2.fX, p2.fY); }
	void quadTo(float,float,float,float);
	void conicTo(SkPoint p1,SkPoint p2, float w) { return conicTo(p1.fX, p1.fY, p2.fX, p2.fY, w); }
	void conicTo(float,float,float,float,float);
	void cubicTo(SkPoint p1,SkPoint p2,SkPoint p3) { return cubicTo(p1.fX, p1.fY, p2.fX, p2.fY, p3.fX, p3.fY); }
	void cubicTo(float,float,float,float,float,float);
	void close();
	void offset(float,float);
	SkPathFillType getFillType() const { return fFillType; }
	void setFillType(SkPathFillType f) { fFillType = f; }
	void toggleInverseFillType() { fFillType = (SkPathFillType) ((int) fFillType ^ 2); }

	void transform(const SkMatrix& matrix, SkPath* dst = nullptr);
#if OP_DEBUG_SERIALIZE
	std::string debugDumpCommon(bool hex, std::string callPrefix) const;
	void dump() const;
	void dumpHex() const;
	void dumpCommon(bool hex, std::string callPrefix) const;
	std::string fillTypeStr() const;
#endif

	std::vector<TinyCurve> path;
	// OpContext* contours;
	// mutable SkRect bounds;
	OpPoint last { 0, 0 };
	size_t firstIndex = 0;
	SkPathFillType fFillType = SkPathFillType::kWinding;
};

// !!! don't think I'll implement these!
//inline bool Op(const SkPath& one, const SkPath& two, SkPathOp op, SkPath* result) { return true; }
//inline bool Simplify(const SkPath& path, SkPath* result) { return true; }

struct SkParsePath {
	static void FromSVGString(const char*, SkPath* ) {}  // incomplete
};

inline std::string dumpSkPath(const SkPath* p, bool hex, std::string prefix) {
#if OP_DEBUG_SERIALIZE
	return p->debugDumpCommon(hex, prefix);
#else
    return "";
#endif
}

typedef float SkScalar;
typedef SkPoint SkVector;
inline float SkIntToScalar(int x) { return (float) x; }
constexpr float SK_ScalarInfinity = OpInfinity;
constexpr float SK_ScalarNegativeInfinity = -OpInfinity;
constexpr float SK_ScalarNaN = OpNaN;
constexpr float SK_ScalarMin = (float) -OpMax;
constexpr float SK_ScalarMax = (float) OpMax;
#define SkASSERT(x) OP_ASSERT(x)
#endif
