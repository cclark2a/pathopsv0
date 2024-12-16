// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOps.h"
#include "SkiaEnumSkPathOp.h"

class SkPath;

namespace PathOpsV0Lib {

enum class CurveType : int {
    no,
    line,
    quad,
    conic,
    cubic
};

enum class BinaryOperand : int;
enum class BinaryWindType : int;

}

SkPathOp MapInvertedSkPathOp(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted);
bool SkPathOpInvertOutput(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted);

void SetSkiaContextCallBacks(PathOpsV0Lib::Context* );
void SetSkiaCurveCallBacks(PathOpsV0Lib::Context* );
PathOpsV0Lib::Contour* SetSkiaSimplifyCallBacks(PathOpsV0Lib::Context* , bool isWindingFill
        OP_DEBUG_PARAMS(const SkPath& ));
PathOpsV0Lib::Contour* SetSkiaOpCallBacks(PathOpsV0Lib::Context* , SkPathOp op, 
		PathOpsV0Lib::BinaryOperand , PathOpsV0Lib::BinaryWindType  OP_DEBUG_PARAMS(const SkPath&));
void AddSkiaPath(PathOpsV0Lib::Context* , PathOpsV0Lib::AddWinding , const SkPath& path);

#if TEST_ANALYZE
// used to break down tests that are too large to debug directly
void AddDebugSkiaPath(PathOpsV0Lib::Context* , PathOpsV0Lib::AddWinding , const SkPath& path);
#endif
#if !OP_TINY_SKIA
std::string dumpSkPath(const SkPath* path, bool inHex);
#endif

#if OP_DEBUG_IMAGE
void debugLineAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugQuadAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugConicAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugCubicAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
#endif
