// (c) 2023, Cary Clark cclark2@gmail.com
#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"
#endif

#include "SkiaPaths.h"
#if TEST_RASTER
#include "OpDebugRaster.h"
#endif

static const SkPathOp inverseOpMapping[5][2][2] {
    {{kDifference_SkPathOp, kIntersect_SkPathOp}, {kUnion_SkPathOp, kReverseDifference_SkPathOp }},
    {{kIntersect_SkPathOp, kDifference_SkPathOp}, {kReverseDifference_SkPathOp, kUnion_SkPathOp }},
    {{kUnion_SkPathOp, kReverseDifference_SkPathOp}, {kDifference_SkPathOp, kIntersect_SkPathOp }},
    {{kXOR_SkPathOp, kXOR_SkPathOp }, {kXOR_SkPathOp, kXOR_SkPathOp}},
    {{kReverseDifference_SkPathOp, kUnion_SkPathOp}, {kIntersect_SkPathOp, kDifference_SkPathOp }},
};

/* Given a PathOps operator, and if the operand fills are inverted, return the equivalent operator
   treating the operands as non-inverted.
 */
SkPathOp MapInvertedSkPathOp(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted) {
    return inverseOpMapping[(int) op][leftOperandIsInverted][rightOperandIsInverted];
}

static const bool outInverse[5][2][2] {
    { { false, false }, { true, false } },  // diff
    { { false, false }, { false, true } },  // sect
    { { false, true }, { true, true } },    // union
    { { false, true }, { true, false } },   // xor
    { { false, true }, { false, false } },  // rev diff
};

/* Given a PathOps operator, and if the operand fills are inverted, return true if the output is
   inverted.
 */
bool SkPathOpInvertOutput(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted) {
    return outInverse[(int) op][leftOperandIsInverted][rightOperandIsInverted];
}

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/ConicBezier.h"
#include "curves/CubicBezier.h"
#include "curves/BinaryWinding.h"
#include "curves/UnaryWinding.h"

using namespace PathOpsV0Lib;

void skiaOutput(Curve c, bool firstPt, bool lastPt, PathOutput output) {
    SkPath::Verb type = (SkPath::Verb) c.type; 
    SkPath& skpath = *(SkPath*)(output);
    if (firstPt) {
		skpath.setFillType(SkPathFillType::kEvenOdd);
        skpath.moveTo(c.data->start.x, c.data->start.y);
	}
    switch (type) {
        case SkPath::kLine_Verb:
            skpath.lineTo(c.data->end.x, c.data->end.y);
            break;
        case SkPath::kQuad_Verb: {
            OpPoint ctrlPt = quadControlPt(c);
            skpath.quadTo(ctrlPt.x, ctrlPt.y, c.data->end.x, c.data->end.y);
            break;
        }
        case SkPath::kConic_Verb: {
            PointWeight ctrl(c);
            skpath.conicTo(ctrl.pt.x, ctrl.pt.y, c.data->end.x, c.data->end.y, ctrl.weight);
            break;
        }
        case SkPath::kCubic_Verb: {
            CubicControls ctrls(c);
            skpath.cubicTo(ctrls.pts[0].x, ctrls.pts[0].y, ctrls.pts[1].x, ctrls.pts[1].y,
                    c.data->end.x, c.data->end.y);
            break;
        }
        default:
            OP_ASSERT(0);
    }
    if (lastPt)
        skpath.close();
}

void SetSkiaCurveCallbacks(Context* context) {
    SetCurveCallbacks(context, SkPath::kLine_Verb, { } );
    quadCallbacks(context, SkPath::kQuad_Verb);
    conicCallbacks(context, SkPath::kConic_Verb);
    cubicCallbacks(context, SkPath::kCubic_Verb);
    OP_DEBUG_CODE(SetSkiaCurveCallbacksDebug(context));
}

void emptySkPathFunc(PathOutput output) {
    SkPath* skOutput = (SkPath*) output;
    skOutput->reset();
	skOutput->setFillType(SkPathFillType::kEvenOdd);
}

PathOpsV0Lib::CurveType setSkiaLineType(PathOpsV0Lib::Context* , PathOpsV0Lib::Curve ) {
    return SkPath::kLine_Verb;
}

void SetSkiaContextCallbacks(Context* context) {
    OP_DEBUG_CODE(if (DebugAnalyze(context)) return);
    SetContextCallbacks(context, { skiaOutput, setSkiaLineType, emptySkPathFunc });
}

Contour* SetSkiaSimplifyCallbacks(Context* context, Winding winding,
        bool isWindingFill  OP_DEBUG_PARAMS(const SkPath& path)) {
    Contour* contour = CreateContour(context, winding);
    WindingAdd addFunc = isWindingFill ? unaryWindingAddFunc : unaryEvenOddFunc;
    WindingAdd subtractFunc = isWindingFill ? unaryWindingSubtractFunc : unaryEvenOddFunc;
    SetWindingCallbacks(context, { addFunc, unaryWindingKeepFunc, unaryWindingVisibleFunc, 
			unaryWindingZeroFunc, subtractFunc });
    OP_DEBUG_CODE(SetSkiaSimplifyCallbacksDebug(context, contour, path));
    return contour;
}

void SetSkiaOpContextCallbacks(Context* context, SkPathOp op, BinaryWindType windType) {
    WindingKeep operatorFunc = nullptr;
    switch (op) {
        case kDifference_SkPathOp: operatorFunc = binaryWindingDifferenceFunc; break;
        case kIntersect_SkPathOp: operatorFunc = binaryWindingIntersectFunc; break;
        case kUnion_SkPathOp: operatorFunc = binaryWindingUnionFunc; break;
        case kXOR_SkPathOp: operatorFunc = binaryWindingExclusiveOrFunc; break;
        case kReverseDifference_SkPathOp: operatorFunc = binaryWindingReverseDifferenceFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd addFunc = nullptr;
    switch (windType) {
        case BinaryWindType::evenOdd: addFunc = binaryEvenOddFunc; break;
        case BinaryWindType::windLeft: addFunc = binaryWindingAddLeftFunc; break;
        case BinaryWindType::windRight: addFunc = binaryWindingAddRightFunc; break;
        case BinaryWindType::windBoth: addFunc = binaryWindingAddFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd subtractFunc = nullptr;
    switch (windType) {
        case BinaryWindType::evenOdd: break;
        case BinaryWindType::windLeft: subtractFunc = binaryWindingSubtractLeftFunc; break;
        case BinaryWindType::windRight: subtractFunc = binaryWindingSubtractRightFunc; break;
        case BinaryWindType::windBoth: subtractFunc = binaryWindingSubtractFunc; break;
        default: OP_ASSERT(0);
    }
    SetWindingCallbacks(context, { addFunc, operatorFunc, binaryWindingVisibleFunc, 
			binaryWindingZeroFunc, subtractFunc });
    OP_DEBUG_CODE(SetSkiaOpContextCallbacksDebug(context, op));
}

Contour* SetSkiaOpContourCallbacks(Context* context, Winding winding,
        BinaryOperand operand  OP_DEBUG_PARAMS(const SkPath& path)) {
    Contour* contour = CreateContour(context, winding);
    OP_DEBUG_CODE(SetSkiaOpContourCallbacksDebug(context, contour, operand, path));
    return contour;
}

void AddSkiaPath(Context* context, Contour* contour, const SkPath& path
        OP_DEBUG_PARAMS(AddDebugContour addDebugContour)) {
	if (!path.isFinite()) {  // raw iter treats non-finite path as empty
		SetError(context, ContextError::finite);
		return;
	}
    SkPath::RawIter iter(path);
    OpPoint closeLine[2] = {{0, 0}, {0, 0}};  // initialize so first move doesn't add close line
    for (;;) {
        SkPoint pts[4];
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            if (closeLine[0] != closeLine[1])
                Add(contour, { closeLine, sizeof(closeLine), SkPath::kLine_Verb } );
            closeLine[0] = closeLine[1] = { pts[0].fX, pts[0].fY };
            pts[1] = pts[0];
			contour = Clone(contour);
            OP_DEBUG_CODE(addDebugContour.add(contour));
            break;
        case SkPath::kLine_Verb:
            if (pts[0] != pts[1])
                Add(contour, { (OpPoint*) pts, sizeof(SkPoint) * 2, SkPath::kLine_Verb } );
            closeLine[0] = { pts[1].fX, pts[1].fY };
            break;
        case SkPath::kQuad_Verb:
            AddQuads(contour, { (OpPoint*) pts, sizeof(SkPoint) * 3, SkPath::kQuad_Verb } );
            closeLine[0] = { pts[2].fX, pts[2].fY };
            break;
        case SkPath::kConic_Verb:
            pts[3].fX = iter.conicWeight(); // !!! hacky
            AddConics(contour, { (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    SkPath::kConic_Verb } );
            closeLine[0] = { pts[2].fX, pts[2].fY };
            break;
        case SkPath::kCubic_Verb:
            AddCubics(contour, { (OpPoint*) pts, sizeof(SkPoint) * 4, SkPath::kCubic_Verb } );
            closeLine[0] = { pts[3].fX, pts[3].fY };
            break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1])
                Add(contour, { closeLine, sizeof(closeLine), SkPath::kLine_Verb } );
            if (SkPath::kDone_Verb == verb)
                return;
            closeLine[0] = closeLine[1];
            break;
        default:
            OP_ASSERT(0);
        }
    }
}

// return true if some point is very large but no value is inf or nan.
bool VeryLargeSkiaPath(const SkPath& path) {
	bool veryLarge = false;
	float large = 1e38f;
	auto checkPt = [large, &veryLarge](SkPoint pt) {
		veryLarge |= fabsf(pt.fX) >= large || fabsf(pt.fY) >= large;
	};
	auto checkPts = [checkPt](SkPoint* pts, size_t count) {
		for (size_t index = 1; index <= count; ++index)
			checkPt(pts[index]);
	};
    SkPath::RawIter iter(path);
    for (;;) {
        SkPoint pts[4];
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            checkPt(pts[0]);
            break;
        case SkPath::kLine_Verb:
            checkPt(pts[1]);
            break;
        case SkPath::kQuad_Verb:
			checkPts(pts, 2);
            break;
        case SkPath::kConic_Verb:
			checkPts(pts, 2);
			veryLarge |= fabsf(iter.conicWeight()) >= large;
            break;
        case SkPath::kCubic_Verb:
			checkPts(pts, 3);
            break;
        case SkPath::kClose_Verb:
			break;
        case SkPath::kDone_Verb:
            return veryLarge;
        default:
            OP_ASSERT(0);
        }
	}
	return veryLarge;
}
