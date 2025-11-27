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

WindKeep skiaOutput(Output o) {
    const Curve& c = o.curve;
    bool firstPt = (int) o.attribute & (int) LoopAttribute::first;
    bool lastPt = (int) o.attribute & (int) LoopAttribute::last;
    ContextUserData data = UserData(c.context, UserDataType::outPath);
    SkPath& skpath = *(SkPath*) data.data;
    SkPath::Verb type = (SkPath::Verb) c.type; 
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
    return WindKeep::Discard;
}

void SetSkiaCurveCallbacks(Context* context) {
    lineCallbacks(context, SkPath::kLine_Verb);
    quadCallbacks(context, SkPath::kQuad_Verb);
    conicCallbacks(context, SkPath::kConic_Verb);
    cubicCallbacks(context, SkPath::kCubic_Verb);
}

void PathOpsV0Lib::emptySkPathFunc(Context* context) {
    ContextUserData data = UserData(context, UserDataType::outPath);
    SkPath* skOutput = (SkPath*) data.data;
    skOutput->reset();
	skOutput->setFillType(SkPathFillType::kEvenOdd);
}

void SetSkiaContextCallbacks(Context* context) {
#if TEST_ANALYZE
    if (DebugAnalyze(context))
        return;
#endif
    SetContextCallbacks(context, { skiaOutput, emptySkPathFunc });
}

Contour* SetSkiaSimplifyCallbacks(Context* context, WindingData data, size_t size,
        bool isWindingFill  OP_DEBUG_PARAMS(const SkPath* pathPtr)) {
    Contour* contour = CreateContour(context, data, size);
    WindingAdd addFunc = isWindingFill ? unaryAddFunc : unaryEvenOddFunc;
    WindingAdd subtractFunc = isWindingFill ? unarySubtractFunc : unaryEvenOddFunc;
    SetWindingCallbacks(context, { addFunc, unaryKeepFunc, subtractFunc });
    OP_DEBUG_CODE(if (pathPtr) SetSkiaSimplifyCallbacksDebug(context, contour, *pathPtr));
    return contour;
}

void SetSkiaOpContextCallbacks(Context* context, SkPathOp op, BinaryWindType windType) {
    WindingKeep operatorFunc = nullptr;
    switch (op) {
        case kDifference_SkPathOp: operatorFunc = binaryDifferenceFunc; break;
        case kIntersect_SkPathOp: operatorFunc = binaryIntersectFunc; break;
        case kUnion_SkPathOp: operatorFunc = binaryUnionFunc; break;
        case kXOR_SkPathOp: operatorFunc = binaryExclusiveOrFunc; break;
        case kReverseDifference_SkPathOp: operatorFunc = binaryReverseDifferenceFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd addFunc = nullptr;
    switch (windType) {
        case BinaryWindType::evenOdd: addFunc = binaryEvenOddFunc; break;
        case BinaryWindType::windLeft: addFunc = binaryAddLeftFunc; break;
        case BinaryWindType::windRight: addFunc = binaryAddRightFunc; break;
        case BinaryWindType::windBoth: addFunc = binaryAddFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd subtractFunc = nullptr;
    switch (windType) {
        case BinaryWindType::evenOdd: break;
        case BinaryWindType::windLeft: subtractFunc = binarySubtractLeftFunc; break;
        case BinaryWindType::windRight: subtractFunc = binarySubtractRightFunc; break;
        case BinaryWindType::windBoth: subtractFunc = binarySubtractFunc; break;
        default: OP_ASSERT(0);
    }
    SetWindingCallbacks(context, { addFunc, operatorFunc, subtractFunc });
//    OP_DEBUG_CODE(SetSkiaOpContextCallbacksDebug(context, op));
}

Contour* SetSkiaOpContourCallbacks(Context* context, PathOpsV0Lib::WindingData windingData, 
		size_t size, BinaryOperand operand  OP_DEBUG_PARAMS(const SkPath* pathPtr)) {
    Contour* contour = CreateContour(context, windingData, size);
    OP_DEBUG_CODE(if (pathPtr) SetSkiaOpContourCallbacksDebug(context, contour, operand, *pathPtr));
    return contour;
}

void AddSkiaPath(Context* context, Contour* contour, const SkPath& path
        /* OP_DEBUG_PARAMS(AddDebugContour* addDebugPtr) */) {
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
                AddLine(contour, { context, closeLine, sizeof(closeLine), SkPath::kLine_Verb } );
            closeLine[0] = closeLine[1] = { pts[0].fX, pts[0].fY };
            pts[1] = pts[0];
			contour = Clone(contour);
//            OP_DEBUG_CODE(if (addDebugPtr) addDebugPtr->add(contour));
            break;
        case SkPath::kLine_Verb:
            AddLine(contour, { context, (OpPoint*) pts, sizeof(SkPoint) * 2, SkPath::kLine_Verb } );
            closeLine[0] = { pts[1].fX, pts[1].fY };
            break;
        case SkPath::kQuad_Verb:
            AddQuads(contour, { context, (OpPoint*) pts, sizeof(SkPoint) * 3, SkPath::kQuad_Verb } );
            closeLine[0] = { pts[2].fX, pts[2].fY };
            break;
        case SkPath::kConic_Verb:
            pts[3].fX = iter.conicWeight(); // !!! hacky
            AddConics(contour, { context, (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    SkPath::kConic_Verb } );
            closeLine[0] = { pts[2].fX, pts[2].fY };
            break;
        case SkPath::kCubic_Verb:
            AddCubics(contour, { context, (OpPoint*) pts, sizeof(SkPoint) * 4, SkPath::kCubic_Verb } );
            closeLine[0] = { pts[3].fX, pts[3].fY };
            break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1])
                AddLine(contour, { context, closeLine, sizeof(closeLine), SkPath::kLine_Verb } );
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
