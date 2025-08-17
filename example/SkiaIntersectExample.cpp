// (c) 2025, Cary Clark cclark2@gmail.com

#include "curves/BinaryWinding.h"
#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkPath.h"
#endif
#include "port/SkiaPaths.h"

using namespace PathOpsV0Lib;

static void IntersectPath(const SkPath& onePath, const SkPath& twoPath, SkPath* out) {
    Context* context = CreateContext();
    SetSkiaContextCallbacks(context);
    SetSkiaCurveCallbacks(context);
    auto isWindingFill = [](const SkPath& path) {
        return SkPathFillType::kWinding == path.getFillType()
                || SkPathFillType::kInverseWinding == path.getFillType();
    }; 
    bool oneIsWinding = isWindingFill(onePath);
    bool twoIsWinding = isWindingFill(twoPath);
    BinaryWindType windType = oneIsWinding && twoIsWinding ? BinaryWindType::windBoth
            : oneIsWinding ? BinaryWindType::windLeft : twoIsWinding ? BinaryWindType::windRight
            : BinaryWindType::evenOdd;
	SetSkiaOpContextCallbacks(context, kIntersect_SkPathOp, windType);
    int oneData[] = { 1, 0 };
    PathOpsV0Lib::Winding oneWinding { oneData, sizeof(oneData) };
    Contour* one = SetSkiaOpContourCallbacks(context, oneWinding, BinaryOperand::left);
    AddSkiaPath(context, one, onePath);
    int twoData[] = { 0, 1 };
    PathOpsV0Lib::Winding twoWinding { twoData, sizeof(twoData) };
    Contour* two = SetSkiaOpContourCallbacks(context, twoWinding, BinaryOperand::right);
    AddSkiaPath(context, two, twoPath);
	PathOutput pathOutput = out;
	Resolve(context, pathOutput);
    DeleteContext(context);
}

void SkiaIntersectExample() {
    SkPath one, two, intersection;
    one.moveTo(0, 0);
    one.quadTo(1, 0, 3, 2);
    one.lineTo(1, 3);
    two.moveTo(1, 0);
    two.lineTo(2, 1);
    two.quadTo(2, 2, 1, 3);
    IntersectPath(one, two, &intersection);
    intersection.dump();
}

OP_TINY_MAIN(SkiaIntersectExample)  // main() for cmake
