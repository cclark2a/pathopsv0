// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOpsTypes.h"
#include "include/pathops/SkPathOps.h"

class SkPath;

namespace PathOpsV0Lib {
enum class BinaryOperand;
}

struct SkiaSimplifyContourData {
	PathOpsV0Lib::Operation unusedOperation;
	PathOpsV0Lib::Operand unusedOperand;
    OP_DEBUG_CODE(const SkPath* pathPtr);
	OP_DEBUG_IMAGE_CODE(bool drawNativePath);
};

struct SkiaOpContourData {
    SkPathOp op;
    PathOpsV0Lib::BinaryOperand operand;
    OP_DEBUG_CODE(const SkPath* pathPtr);
	OP_DEBUG_IMAGE_CODE(bool drawNativePath);
};

SkPathOp MapInvertedSkPathOp(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted);
bool SkPathOpInvertOutput(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted);

void SetSkiaContextCallBacks(PathOpsV0Lib::Context* context);
void SetSkiaCurveCallBacks(PathOpsV0Lib::Context* );
PathOpsV0Lib::Contour* SetSkiaSimplifyCallBacks(PathOpsV0Lib::Context* , bool isWindingFill 
		OP_DEBUG_PARAMS(const SkPath& ));
PathOpsV0Lib::Contour* SetSkiaOpCallBacks(PathOpsV0Lib::Context* , SkPathOp op, PathOpsV0Lib::BinaryOperand ,
		bool isWindingFill  OP_DEBUG_PARAMS(const SkPath& ));
void AddSkiaPath(PathOpsV0Lib::AddWinding , const SkPath& path);

#if OP_DEBUG_IMAGE
void debugLineAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugQuadAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugConicAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
void debugCubicAddToSkPath(PathOpsV0Lib::Curve , SkPath& );
#endif
