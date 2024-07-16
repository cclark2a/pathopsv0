// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef TinySkia_DEFINED
#define TinySkia_DEFINED

#include "OpCurve.h"

inline float SkBits2Float(int32_t i) {
	return OpDebugBitsToFloat(i); }

inline int SkScalarRoundToInt(float f) {
	return (int) floorf(f + 0.5); }

inline float SkDoubleToScalar(double d) {
	return (float) d; }

struct SkRandom {
	int nextRangeU(int, int);
};

constexpr uint32_t SK_ColorWHITE = 0xFFFFFFFF;
constexpr uint32_t SK_ColorBLACK = 0xFF000000;
constexpr uint32_t SK_ColorRED =   0xFFFF0000;
constexpr uint32_t SK_ColorBLUE =  0xFF00FF00;
constexpr uint32_t SK_ColorGREEN = 0xFF0000FF;

inline uint8_t SkColorGetR(uint32_t c) { return 0; }
inline uint8_t SkColorGetG(uint32_t c) { return 0; }
inline uint8_t SkColorGetB(uint32_t c) { return 0; }

struct SkPoint {
	static SkPoint Make(float x, float y) { 
		return { x, y }; }

    friend bool operator==(SkPoint a, SkPoint b) {
        return a.fX == b.fX && a.fY == b.fY;
    }

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
	void reset();
	void setScale(float, float);
	void setRotate(float);
	void setRotate(float deg, float px, float py);
	void preScale(float, float);
	void preTranslate(float, float);
	void postTranslate(float, float);
	void mapPoints(SkPoint * ,int)const;
	void mapPoints(OpPoint * ,int)const;
	void setConcat(const SkMatrix&, const SkMatrix& );

	float m[2][3];
};

struct SkRect {
	static SkRect MakeWH(float w, float h) {
		return { 0, 0, w, h }; }

	float centerX() const { 
		return (fLeft + fRight) / 2; }
	float centerY() const { 
		return (fTop + fBottom) / 2; }
	bool contains(struct SkRect const &);
	float height() const { 
		return fBottom - fTop; }
	void inset(float, float) {}
	bool intersect(struct SkRect const &);
	void join(struct SkRect const &);
	void offset(float dx, float dy) {
		fLeft += dx; fTop += dy; fRight += dx; fBottom += dy; }
	float width() const { 
		return fRight - fLeft; }

	float fLeft;
	float fTop;
	float fRight;
	float fBottom;
};

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
	void addPath(const SkPath& );
	void addPath(const SkPath& , const SkMatrix& );
	void addRect(float, float, float, float, SkPathDirection dir = SkPathDirection::kCW);
	void arcTo(const SkRect& , float startAngle, float sweepAngle, bool forceMoveTo);
	bool isInverseFillType() const { return SkPathFillType::kInverseWinding == fFillType
			|| SkPathFillType::kInverseEvenOdd == fFillType; }
	const SkRect& getBounds() const;
	void reset();
	bool isEmpty() const;
	void moveTo(SkPoint p) { return moveTo(p.fX, p.fY); }
	void moveTo(float,float);
	void lineTo(SkPoint p) { return lineTo(p.fX, p.fY); }
	void lineTo(float,float);
	void rLineTo(float,float);
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
	const SkPath& makeTransform(SkMatrix const &);
	void toggleInverseFillType() { fFillType = (SkPathFillType) ((int) fFillType ^ 2); }

	void transform(const SkMatrix& matrix, SkPath* dst = nullptr);
    void updateBoundsCache() const {
        getBounds(); }
	void dump() const;
	void dumpHex() const;
	void dumpCommon(bool) const;

	std::vector<OpCurve> path;
	mutable SkRect bounds;
	OpPoint last;
	size_t firstIndex;
	SkPathFillType fFillType;
};

struct SkImageInfo {
	static SkImageInfo MakeN32Premul(int w, int h) {
		SkImageInfo i;
		i.width = w;
		i.height = h;
		return i;
	}

	int width;
	int height;
};

class SkBitmap {
public:
	SkBitmap()
		: pixels(nullptr) {
		info.width = 0;
		info.height = 0;
	}

	~SkBitmap() {
		delete pixels;
	}

	void allocPixels(SkImageInfo const &);

	void allocN32Pixels(int w, int h) { 
		info = SkImageInfo::MakeN32Premul(w, h); 
		allocPixels(info); 
	}

	uint32_t* getAddr32(int x, int y) {
		return &pixels[y * info.width + x]; }

	uint32_t getColor(int, int);
	int width() const { return info.width; }

	SkImageInfo info;
	uint32_t* pixels;
};

// !!! I haven't decided if I'm going to use this or not
struct SkRegion {
	enum Op {

	};

	void getBoundaryPath(SkPath* ) {}
	void setRect(int , int , int , int ) {}
	void setPath(const SkPath& , const SkRegion& ) {}
	void op(const SkRegion& , const SkRegion& , Op ) {}
	void setRect(SkRect ) {}
};

enum class SkBlendMode {
	kPlus
};

struct SkPaint {
	enum Style {
		kStroke_Style,
		kFill_Style
	};

	void setAlpha(uint8_t a) {
		color = (color & 0x00FFFFFF) ; }
	void setAntiAlias(bool) {}
	void setStyle(Style s) { 
		style = s; }
	void setBlendMode(SkBlendMode b) {
		blendMode = b; }
	void setColor(uint32_t c) {
		color = c; }
	void setStrokeWidth(float sw) {
		strokeWidth = sw; }

	uint32_t color;
	float strokeWidth;
	Style style;
	SkBlendMode blendMode;
};

struct SkString {
	SkString() {}
	SkString(std::string s) { string = s; }
	SkString(const char* s) { string = s; }
	void appendf(const char format[], ...);
	const char* c_str() const { return string.c_str(); }
	void printf(const char format[], ...);

	std::string string;
};

enum SkTextEncoding {
	kUTF8
};

struct SkFont {
	SkFont(void*, float s,float,float);
	float getSize() const { return fSize; }
	void setSize(float s) { fSize = s; }
	float measureText(void const *,uint64_t,SkTextEncoding, SkRect*) const;

	float fSize;
};

struct SkCanvas {
	SkCanvas(SkBitmap& b)
		: bitmap(b) {
		m.emplace_back();
		m.back().reset();
	}

	void clear(uint32_t c) {
		int size = bitmap.info.width * bitmap.info.height;
		uint32_t* p = bitmap.pixels;
		while (size--) {
			*p++ = c;
		}
	}

	void save();
	void translate(float , float );
	void restore();
	void rotate(float deg, float x, float y);
	void drawColor(uint32_t c) { 
		clear(c); }
	void drawLine(float x1, float y1, float x2, float y2, const SkPaint&);
	void drawPath(class SkPath const &, const SkPaint&);
	void drawString(SkString , float , float , const SkFont& , const SkPaint&);

	SkBitmap& bitmap;
	std::vector<SkMatrix> m;
};

enum SkPathOp {
    kDifference_SkPathOp,         //!< subtract the op path from the first path
    kIntersect_SkPathOp,          //!< intersect the two paths
    kUnion_SkPathOp,              //!< union (inclusive-or) the two paths
    kXOR_SkPathOp,                //!< exclusive-or the two paths
    kReverseDifference_SkPathOp,  //!< subtract the first path from the op path
};

// !!! don't think I'll implement these!
inline bool Op(const SkPath& one, const SkPath& two, SkPathOp op, SkPath* result) { return true; }
inline bool Simplify(const SkPath& path, SkPath* result) { return true; }

struct SkOpGlobalState {
	static bool DebugRunFail() { return true; }
};

struct SkPathOpsDebug {
	static const char* OpStr(SkPathOp op) { return ""; }
};

typedef float SkScalar;
typedef SkPoint SkVector;
inline float SkIntToScalar(int x) { return (float) x; }
constexpr float SK_ScalarInfinity = OpInfinity;
constexpr float SK_ScalarNegativeInfinity = -OpInfinity;
constexpr float SK_ScalarNaN = OpNaN;
constexpr float SK_ScalarMin = -OpMax;
constexpr float SK_ScalarMax = OpMax;
#define SK_ASSERT(x) OP_ASSERT(x)
#define SkASSERT(x) OP_ASSERT(x)
#endif
