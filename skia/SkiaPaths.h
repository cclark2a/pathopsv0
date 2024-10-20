// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOpsTypes.h"

#if !OP_TINY_SKIA
#include "include/pathops/SkPathOps.h"
#else
enum SkPathOp : unsigned int;
#endif

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
void AddSkiaPath(PathOpsV0Lib::AddWinding , const SkPath& path);

#if OP_DEBUG_IMAGE
void debugLineAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugQuadAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugConicAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugCubicAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
#endif
