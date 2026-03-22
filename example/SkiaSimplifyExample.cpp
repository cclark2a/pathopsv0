// (c) 2025, Cary Clark cclark2@gmail.com

#include "TinySkia.h"
#include "port/SkiaPaths.h"

using namespace PathOpsV0Lib;

static void SimplifyPath(const SkPath& path, SkPath* out) {
    Context* context = CreateContext();
    ContextUserData data { out, sizeof(out), UserDataType::outPath };
    AddUserData(context, data);
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
#if OP_DEBUG_SERIALIZE
    simplified.dump();
#endif
}

OP_TINY_MAIN(SkiaSimplifyExample)  // main() for cmake
