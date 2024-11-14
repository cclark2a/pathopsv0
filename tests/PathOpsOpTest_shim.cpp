// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"

#if !OP_DEBUG_FAST_TEST
OP_DEBUG_CODE(extern bool debugUseAlt);
#endif

extern void threadablePathOpTest(int id, const SkPath& a, const SkPath& b, 
        SkPathOp op, std::string testname, bool v0MayFail, bool skiaMayFail, bool mayDiffer);

#if OP_TINY_SKIA

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

// some tests call internal routines to construct the tests. Code to do that is commented out below
// save the results to test without Skia
void alt_loops58iAsQuads() {
	SkPath qPath, qPathB;
	qPath.setFillType(SkPathFillType::kWinding);
	qPath.moveTo(SkBits2Float(0x40000000), SkBits2Float(0x40400000));  // 2, 3
	qPath.quadTo(SkBits2Float(0x401423e4), SkBits2Float(0x406953ed), SkBits2Float(0x401ab099), SkBits2Float(0x407a4cf5));  // 2.31469059f, 3.64574742f, 2.41702867f, 3.91094708f
	qPath.quadTo(SkBits2Float(0x40234949), SkBits2Float(0x4084f34f), SkBits2Float(0x40282a44), SkBits2Float(0x40817b4d));  // 2.55134797f, 4.15470076f, 2.62757969f, 4.04630136f
	qPath.quadTo(SkBits2Float(0x402c4259), SkBits2Float(0x406ca08e), SkBits2Float(0x406aaaaa), SkBits2Float(0x40400000));  // 2.69154954f, 3.69729948f, 3.66666651f, 3
	qPath.lineTo(SkBits2Float(0x40000000), SkBits2Float(0x40400000));  // 2, 3
	qPath.close();
	qPathB.setFillType(SkPathFillType::kWinding);
	qPathB.moveTo(SkBits2Float(0x40400000), SkBits2Float(0x40a00000));  // 3, 5
	qPathB.quadTo(SkBits2Float(0x402afdab), SkBits2Float(0x4094cda7), SkBits2Float(0x402a3b68), SkBits2Float(0x408a332d));  // 2.67173266f, 4.65010405f, 2.65987587f, 4.31874704f
	qPathB.quadTo(SkBits2Float(0x40297925), SkBits2Float(0x407f3165), SkBits2Float(0x402eba1e), SkBits2Float(0x406d8ba6));  // 2.64801908f, 3.9873898f, 2.73010969f, 3.71164846f
	qPathB.quadTo(SkBits2Float(0x403551ef), SkBits2Float(0x4058f2a8), SkBits2Float(0x402f3b43), SkBits2Float(0x404ca11a));  // 2.83312583f, 3.38981056f, 2.73799205f, 3.19733286f
	qPathB.quadTo(SkBits2Float(0x40292498), SkBits2Float(0x40404f8c), SkBits2Float(0x40000000), SkBits2Float(0x40400000));  // 2.64285851f, 3.00485516f, 2, 3
	qPathB.lineTo(SkBits2Float(0x40400000), SkBits2Float(0x40a00000));  // 3, 5
	qPathB.close();
threadablePathOpTest(0, qPath, qPathB, kDifference_SkPathOp, __FUNCTION__, false, false, false);
}

// note: qPathB includes path_edit
void alt_loops59iasQuads() {
	SkPath qPath, qPathB;
	qPath.setFillType(SkPathFillType::kWinding);
	qPath.moveTo(SkBits2Float(0x00000000), SkBits2Float(0x40c00000));  // 0, 6
	qPath.quadTo(SkBits2Float(0x3f1b96b0), SkBits2Float(0x408840ee), SkBits2Float(0x3fcb5e5d), SkBits2Float(0x4056611e));  // 0.607768059f, 4.25792599f, 1.58881724f, 3.34967756f
	qPath.quadTo(SkBits2Float(0x400c86e3), SkBits2Float(0x402d005c), SkBits2Float(0x400bfc38), SkBits2Float(0x4017f211));  // 2.19573283f, 2.70314693f, 2.18726921f, 2.37414956f
	qPath.quadTo(SkBits2Float(0x400b8106), SkBits2Float(0x3ffd786e), SkBits2Float(0x3d8b3f64), SkBits2Float(0x3ff5fd97));  // 2.17974997f, 1.98023772f, 0.0679920018f, 1.92180145f
	qPath.quadTo(SkBits2Float(0xc002cd10), SkBits2Float(0x3ff37e21), SkBits2Float(0xc0f00000), SkBits2Float(0x40000000));  // -2.04376602f, 1.90228665f, -7.5f, 2
	qPath.lineTo(SkBits2Float(0x00000000), SkBits2Float(0x40c00000));  // 0, 6
	qPath.close();
	qPathB.setFillType(SkPathFillType::kWinding);
	qPathB.moveTo(SkBits2Float(0x3f800000), SkBits2Float(0x40000000));  // 1, 2
	qPathB.quadTo(SkBits2Float(0x401024ee), SkBits2Float(0x3ff726a3), SkBits2Float(0x4020d631), SkBits2Float(0x3ff45865));  // 2.25225401f, 1.9308666f, 2.51307321f, 1.90894759f
	qPathB.quadTo(SkBits2Float(0x40277dae), SkBits2Float(0x3ff35f88), SkBits2Float(0x40277f46), SkBits2Float(0x3ff382ae));  // 2.61704588f, 1.90135288f, 2.61714315f, 1.90242553f
	qPathB.quadTo(SkBits2Float(0x4027729b), SkBits2Float(0x3ff3a064), SkBits2Float(0x40231275), SkBits2Float(0x3ff4a754));  // 2.61636996f, 1.90333223f, 2.54800153f, 1.91135645f
	qPathB.quadTo(SkBits2Float(0x3ff31613), SkBits2Float(0x3ff945f3), SkBits2Float(0x3d744df7), SkBits2Float(0x40180ccf));  // 1.89911115f, 1.94744718f, 0.0596446656f, 2.37578177f
	qPathB.quadTo(SkBits2Float(0xbfdb46c8), SkBits2Float(0x4032b59d), SkBits2Float(0xc009b640), SkBits2Float(0x406fd9bc));  // -1.71309757f, 2.79233479f, -2.15174866f, 3.74766445f
	qPathB.quadTo(SkBits2Float(0xc00b1652), SkBits2Float(0x40932b41), SkBits2Float(0x00000000), SkBits2Float(0x40c00000));  // -2.17323732f, 4.59903002f, 0, 6
	qPathB.lineTo(SkBits2Float(0x3f800000), SkBits2Float(0x40000000));  // 1, 2
	qPathB.close();
threadablePathOpTest(0, qPath, qPathB, kDifference_SkPathOp, __FUNCTION__, false, false, false);
}

void alt_loops33iAsQuads() {
	SkPath qPath, qPathB;
	qPath.setFillType(SkPathFillType::kWinding);
	qPath.moveTo(SkBits2Float(0x40000000), SkBits2Float(0x40c00000));  // 2, 6
	qPath.quadTo(SkBits2Float(0x3fdc176b), SkBits2Float(0x408cfc33), SkBits2Float(0x40165063), SkBits2Float(0x408d01de));  // 1.71946466f, 4.40578604f, 2.34865642f, 4.40647793f
	qPath.quadTo(SkBits2Float(0x402ecec6), SkBits2Float(0x40926cbc), SkBits2Float(0x400eb7e0), SkBits2Float(0x40b95142));  // 2.73137045f, 4.57577324f, 2.22997284f, 5.79116917f
	qPath.quadTo(SkBits2Float(0x3fa58352), SkBits2Float(0x40e035c9), SkBits2Float(0xc0955555), SkBits2Float(0x40f55555));  // 1.29307008f, 7.00656557f, -4.66666651f, 7.66666651f
	qPath.lineTo(SkBits2Float(0x40000000), SkBits2Float(0x40c00000));  // 2, 6
	qPath.close();
	qPathB.setFillType(SkPathFillType::kWinding);
	qPathB.moveTo(SkBits2Float(0x3f800000), SkBits2Float(0x40000000));  // 1, 2
	qPathB.quadTo(SkBits2Float(0x4036158d), SkBits2Float(0x40809bed), SkBits2Float(0x402dd839), SkBits2Float(0x40a4eb97));  // 2.84506536f, 4.01903391f, 2.71632218f, 5.15375853f
	qPathB.quadTo(SkBits2Float(0x401b176d), SkBits2Float(0x40b77e0f), SkBits2Float(0x3fe90c46), SkBits2Float(0x40c32cda));  // 2.4233048f, 5.73413801f, 1.82068706f, 6.09922504f
	qPathB.quadTo(SkBits2Float(0x3e8026ad), SkBits2Float(0x40d819b4), SkBits2Float(0xbddd1e84), SkBits2Float(0x40d73150));  // 0.250295073f, 6.75313759f, -0.10796836f, 6.72476959f
	qPathB.quadTo(SkBits2Float(0xbddea6f2), SkBits2Float(0x40d342c7), SkBits2Float(0x40000000), SkBits2Float(0x40c00000));  // -0.10871686f, 6.60190153f, 2, 6
	qPathB.lineTo(SkBits2Float(0x3f800000), SkBits2Float(0x40000000));  // 1, 2
	qPathB.close();
threadablePathOpTest(0, qPath, qPathB, kDifference_SkPathOp, __FUNCTION__, false, false, false);
}

void alt_loops40iAsQuads() {
	SkPath qPath, qPathB;
	qPath.setFillType(SkPathFillType::kWinding);
	qPath.moveTo(SkBits2Float(0x40400000), SkBits2Float(0x40a00000));  // 3, 5
	qPath.quadTo(SkBits2Float(0x3fe7c712), SkBits2Float(0x40a00000), SkBits2Float(0x3fcf6f5d), SkBits2Float(0x40a3e513));  // 1.81076264f, 5, 1.62058604f, 5.12171316f
	qPath.quadTo(SkBits2Float(0x3fc63354), SkBits2Float(0x40a8c36a), SkBits2Float(0x3fe43516), SkBits2Float(0x40bf2895));  // 1.54844141f, 5.27385426f, 1.78287005f, 5.97370386f
	qPath.quadTo(SkBits2Float(0x401ee97f), SkBits2Float(0x40e1994b), SkBits2Float(0x40400000), SkBits2Float(0x41300000));  // 2.48300147f, 7.04996252f, 3, 11
	qPath.lineTo(SkBits2Float(0x40400000), SkBits2Float(0x40a00000));  // 3, 5
	qPath.close();
	qPathB.setFillType(SkPathFillType::kWinding);
	qPathB.moveTo(SkBits2Float(0x00000000), SkBits2Float(0x40a00000));  // 0, 5
	qPathB.quadTo(SkBits2Float(0x3f5bf108), SkBits2Float(0x40a1c272), SkBits2Float(0x3fb6b7b4), SkBits2Float(0x40b784e3));  // 0.859146595f, 5.054986f, 1.42748117f, 5.73497152f
	qPathB.quadTo(SkBits2Float(0x3fff76e3), SkBits2Float(0x40cd4755), SkBits2Float(0x40150342), SkBits2Float(0x40e1ec73));  // 1.99581563f, 6.41495752f, 2.32832384f, 7.06011343f
	qPathB.quadTo(SkBits2Float(0x402dcd14), SkBits2Float(0x40f55555), SkBits2Float(0x403722c9), SkBits2Float(0x40f2d9b1));  // 2.71564198f, 7.66666651f, 2.86149812f, 7.58907366f
	qPathB.quadTo(SkBits2Float(0x40400000), SkBits2Float(0x40ead772), SkBits2Float(0x40400000), SkBits2Float(0x40a00000));  // 3, 7.33879948f, 3, 5
	qPathB.lineTo(SkBits2Float(0x00000000), SkBits2Float(0x40a00000));  // 0, 5
	qPathB.close();
threadablePathOpTest(0, qPath, qPathB, kDifference_SkPathOp, __FUNCTION__, false, false, false);
}

void alt_cubicOp114asQuad() {
	SkPath qPath, qPathB;
	qPath.setFillType(SkPathFillType::kWinding);
	qPath.moveTo(SkBits2Float(0x00000000), SkBits2Float(0x3f800000));  // 0, 1
	qPath.quadTo(SkBits2Float(0x3ec23c99), SkBits2Float(0x3fedac0b), SkBits2Float(0x3eb0c3c4), SkBits2Float(0x40046269));  // 0.379368573f, 1.85681283f, 0.345243573f, 2.06850648f
	qPath.quadTo(SkBits2Float(0x3eb3f74b), SkBits2Float(0x400a816c), SkBits2Float(0x3ebf6d5a), SkBits2Float(0x400b6717));  // 0.351496071f, 2.16414928f, 0.373881161f, 2.1781671f
	qPath.quadTo(SkBits2Float(0x3ecbbd95), SkBits2Float(0x400b9eb0), SkBits2Float(0x3f03fc0b), SkBits2Float(0x4008358c));  // 0.397930771f, 2.18156052f, 0.51556462f, 2.12826824f
	qPath.quadTo(SkBits2Float(0x3f300097), SkBits2Float(0x40012a8c), SkBits2Float(0x3fad68ad), SkBits2Float(0x3fe55a48));  // 0.687509f, 2.01822186f, 1.35475695f, 1.79181767f
	qPath.quadTo(SkBits2Float(0x40016888), SkBits2Float(0x3fc85f78), SkBits2Float(0x40600000), SkBits2Float(0x3faaaaab));  // 2.02200508f, 1.56541348f, 3.5f, 1.33333337f
	qPath.lineTo(SkBits2Float(0x00000000), SkBits2Float(0x3f800000));  // 0, 1
	qPath.close();
	qPathB.setFillType(SkPathFillType::kWinding);
	qPathB.moveTo(SkBits2Float(0x3f800000), SkBits2Float(0x40400000));  // 1, 3
	qPathB.quadTo(SkBits2Float(0x3ea9d29a), SkBits2Float(0x40290609), SkBits2Float(0x3ef6e1a3), SkBits2Float(0x4015b696));  // 0.331684887f, 2.64099336f, 0.482190222f, 2.33926916f
	qPathB.quadTo(SkBits2Float(0x3f21f855), SkBits2Float(0x40026723), SkBits2Float(0x3f80b8d8), SkBits2Float(0x3fe5846a));  // 0.632695496f, 2.03754497f, 1.00564098f, 1.79310346f
	qPathB.quadTo(SkBits2Float(0x3fb0d529), SkBits2Float(0x3fc370e6), SkBits2Float(0x3fae797d), SkBits2Float(0x3faa0fcb));  // 1.38150513f, 1.52688289f, 1.36308253f, 1.32860696f
	qPathB.quadTo(SkBits2Float(0x3fa6f27f), SkBits2Float(0x3f90aeb1), SkBits2Float(0x00000000), SkBits2Float(0x3f800000));  // 1.30427539f, 1.13033116f, 0, 1
	qPathB.lineTo(SkBits2Float(0x3f800000), SkBits2Float(0x40400000));  // 1, 3
	qPathB.close();
threadablePathOpTest(0, qPath, qPathB, kDifference_SkPathOp, __FUNCTION__, false, false, false);
}

struct CubicPts {
    static const int kPointCount = 4;
    SkDPoint fPts[kPointCount];
};

inline void CubicPathToQuads(const SkPath& cubicPath, SkPath* quadPath) {
#if !OP_DEBUG_FAST_TEST
		OP_DEBUG_CODE(debugUseAlt = true);
#endif
	// intentionally left unimplemented
}

#include "tests/PathOpsOpTest.cpp"

void run_op_tests(skiatest::Reporter* reporter) {
    reporter->subname = "fail";
    test_PathOpsFailOp(reporter);
    reporter->subname = "main";
    test_PathOpsOp(reporter);
    reporter->subname = "rep";
    test_PathOpsRepOp(reporter);
}
