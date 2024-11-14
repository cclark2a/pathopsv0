// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"

#if OP_TINY_SKIA

#if !OP_DEBUG_FAST_TEST
OP_DEBUG_CODE(extern bool debugUseAlt);
#endif

extern void threadablePathOpTest(int id, const SkPath& a, const SkPath& b, 
        SkPathOp op, std::string testname, bool v0MayFail, bool skiaMayFail, bool mayDiffer);

void alt_cubicOp130a() {
SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(SkBits2Float(0x40a00000), SkBits2Float(0x40c00000));  // 5, 6
path.quadTo(SkBits2Float(0x4089d89e), SkBits2Float(0x40b5c670), SkBits2Float(0x40676275), SkBits2Float(0x405c6701));  // 4.30769253f, 5.68047333f, 3.61538434f, 3.44378686f
path.quadTo(SkBits2Float(0x4033b13a), SkBits2Float(0x3f331fd0), SkBits2Float(0x40000000), SkBits2Float(0x3f800000));  // 2.80769205f, 0.69970417f, 2, 1
path.lineTo(SkBits2Float(0x40a00000), SkBits2Float(0x40c00000));  // 5, 6
path.close();
path.moveTo(SkBits2Float(0x40a00000), SkBits2Float(0x40c00000));  // 5, 6
path.quadTo(SkBits2Float(0x3f948dde), SkBits2Float(0x40204c6b), SkBits2Float(0x406c5fda), SkBits2Float(0x406771a6));  // 1.16057944f, 2.50466418f, 3.69335032f, 3.61631155f
path.quadTo(SkBits2Float(0x40b8f154), SkBits2Float(0x408fdbb9), SkBits2Float(0x40c00000), SkBits2Float(0x40800000));  // 5.779459f, 4.49557161f, 6, 4
SkPath a = path;
path.reset();
path.setFillType(SkPathFillType::kWinding);
path.moveTo(SkBits2Float(0x00000000), SkBits2Float(0x40400000));  // 0, 3
path.close();
SkPath b = path;
threadablePathOpTest(0, a, b, kDifference_SkPathOp, __FUNCTION__, false, false, false);
}

void alt_loop1asQuad() {
SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(SkBits2Float(0x00000000), SkBits2Float(0x3f800000));  // 0, 1
path.quadTo(SkBits2Float(0x3e811de3), SkBits2Float(0x40128dc7), SkBits2Float(0xbe0ed315), SkBits2Float(0x4037d06b));  // 0.252181143f, 2.2899034f, -0.139477089f, 2.87209582f
path.quadTo(SkBits2Float(0xbf6ee1b5), SkBits2Float(0x40702d7e), SkBits2Float(0xbeed02de), SkBits2Float(0x405ff76a));  // -0.933131516f, 3.75277662f, -0.4629125f, 3.49947596f
path.quadTo(SkBits2Float(0xbdc7b335), SkBits2Float(0x4047fbf9), SkBits2Float(0x410d5555), SkBits2Float(0x40155556));  // -0.097509779f, 3.12475419f, 8.83333302f, 2.33333349f
path.lineTo(SkBits2Float(0x00000000), SkBits2Float(0x3f800000));  // 0, 1
path.close();
SkPath a = path;
path.reset();
path.setFillType(SkPathFillType::kWinding);
path.moveTo(SkBits2Float(0x3f800000), SkBits2Float(0x40a00000));  // 1, 5
path.quadTo(SkBits2Float(0xc02563fd), SkBits2Float(0x4073fc80), SkBits2Float(0x3f96952c), SkBits2Float(0x403b3855));  // -2.5842278f, 3.81228638f, 1.17642736f, 2.92531323f
path.quadTo(SkBits2Float(0x40a98186), SkBits2Float(0x3fff192c), SkBits2Float(0x00000000), SkBits2Float(0x3f800000));  // 5.29706097f, 1.99295568f, 0, 1
path.lineTo(SkBits2Float(0x3f800000), SkBits2Float(0x40a00000));  // 1, 5
path.close();
SkPath b = path;
threadablePathOpTest(0, a, b, kIntersect_SkPathOp, __FUNCTION__, false, false, false);
}

#define SkPathOpsCubic_DEFINED

struct SkDPoint {
    double fX;
    double fY;

    static bool ApproximatelyEqual(const SkPoint& a, const SkPoint& b) {
	 OpPoint oa { a.fX, a.fY };
	 OpPoint ob { b.fX, b.fY };
	 return oa.isNearly(ob, OpVector(OpEpsilon * 8, OpEpsilon * 8));
	}

    SkPoint asSkPoint() const {
        SkPoint pt { (float) fX, (float) fY };
        return pt;
    }
};

struct SkDQuad {
    static const int kPointCount = 3;
    SkDPoint fPts[kPointCount];

	SkDPoint& operator[](int index) {
		return fPts[index];
	}

};

struct SkDCubicPair;

struct SkDCubic {
    static const int kPointCount = 4;
    SkDPoint fPts[kPointCount];

    SkDCubicPair chopAt(double t) const;

    static int ComplexBreak(const SkPoint pts[4], SkScalar* t) {
#if !OP_DEBUG_FAST_TEST
		OP_DEBUG_CODE(debugUseAlt = true);
#endif
		// intentionally unimplemented
		// tests that call this are substituted by name
		return 0;
	}

    void debugSet(const SkDPoint* pts) {
		memcpy(fPts, pts, sizeof(fPts));
	}

	int findInflections(double tValues[2]) const {
#if !OP_DEBUG_FAST_TEST
		OP_DEBUG_CODE(debugUseAlt = true);
#endif
		// intentionally unimplemented
		// tests that call this are substituted by name
		tValues[0] = 1;
		tValues[1] = 0;
		return 0 == fPts[0].fX ? 2 : 1;
	}

    void set(const SkPoint* pts) {
		for (int index = 0; index < kPointCount; ++index)
			fPts[index] = { pts[index].fX, pts[index].fY };
	}

    SkDQuad toQuad() const {
		SkDQuad result;
#if !OP_DEBUG_FAST_TEST
		OP_DEBUG_CODE(debugUseAlt = true);
#endif
		// intentionally unimplemented
		// tests that call this are substituted by name
		result.fPts[0] = { 0, 0 };
		return result;
	}
};

struct SkDCubicPair {
    SkDPoint pts[7];

    SkDCubic first() const {
        return (const SkDCubic&) pts[0];
	}

	SkDCubic second() const {
        return (const SkDCubic&) pts[3];
	}
};

SkDCubicPair SkDCubic::chopAt(double t) const {
	SkDCubicPair result;
#if !OP_DEBUG_FAST_TEST
	OP_DEBUG_CODE(debugUseAlt = true);
#endif
	// intentionally unimplemented
	// tests that call this are substituted by name
	result.pts[0] = { 0, 0 };
	return result;
}

#else

#include "src/pathops/SkPathOpsQuad.h"

#endif

struct CubicPts {
    static const int kPointCount = 4;
    SkDPoint fPts[kPointCount];
};

#include "tests/PathOpsOpTest.cpp"

void run_op_tests(skiatest::Reporter* reporter) {
    reporter->subname = "fail";
    test_PathOpsFailOp(reporter);
    reporter->subname = "main";
    test_PathOpsOp(reporter);
    reporter->subname = "rep";
    test_PathOpsRepOp(reporter);
}
