// (c) 2025, Cary Clark cclark2@gmail.com

#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkPath.h"
#endif
#include "port/SkiaPaths.h"

using namespace PathOpsV0Lib;

static void SimplifyPath(const SkPath& path, SkPath* out) {
    Context* context = CreateContext((ContextUserData*) out);
    SetSkiaContextCallbacks(context);
    SetSkiaCurveCallbacks(context);
    int simplifyData[] = { 1 };
    bool isWindingFill = SkPathFillType::kWinding == path.getFillType()
            || SkPathFillType::kInverseWinding == path.getFillType();
    Contour* simplify = SetSkiaSimplifyCallbacks(context, 
            simplifyData, sizeof(simplifyData), isWindingFill);
    AddSkiaPath(context, simplify, path);
	Resolve(context);
    DeleteContext(context);
}

void SkiaSimplifyExample() {
    SkPath path, simplified;
    path.setFillType(SkPathFillType::kWinding);
    path.moveTo(0, 0);
    path.quadTo(1, 0, 3, 2);
    path.lineTo(1, 3);
    path.moveTo(1, 0);
    path.lineTo(2, 1);
    path.quadTo(2, 2, 1, 3);
    SimplifyPath(path, &simplified);
    simplified.dump();
}

OP_TINY_MAIN(SkiaSimplifyExample)  // main() for cmake
