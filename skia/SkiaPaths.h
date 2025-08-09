// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef SkiaPaths_DEFINED
#define SkiaPaths_DEFINED

#include "DebugOpsTypes.h"
#include "PathOps.h"
#include "SkiaEnumSkPathOp.h"

class SkPath;

namespace PathOpsV0Lib {

enum class BinaryOperand : int;
enum class BinaryWindType : int;

}

#if OP_DEBUG
struct UnaryContour {
    const SkPath* pathPtr;
    int contourIndex;
	bool drawNativePath;
};

struct BinaryContour : public UnaryContour {
	PathOpsV0Lib::BinaryOperand operand;
};
#endif

SkPathOp MapInvertedSkPathOp(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted);
bool SkPathOpInvertOutput(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted);

void SetSkiaContextCallbacks(PathOpsV0Lib::Context* );
void SetSkiaCurveCallbacks(PathOpsV0Lib::Context* );
PathOpsV0Lib::Contour* SetSkiaSimplifyCallbacks(PathOpsV0Lib::Context* , PathOpsV0Lib::Winding , 
		bool isWindingFill  OP_DEBUG_PARAMS(const SkPath& ));
void SetSkiaOpContextCallbacks(PathOpsV0Lib::Context* , SkPathOp , PathOpsV0Lib::BinaryWindType );
PathOpsV0Lib::Contour* SetSkiaOpContourCallbacks(PathOpsV0Lib::Context* , PathOpsV0Lib::Winding , 
		PathOpsV0Lib::BinaryOperand  OP_DEBUG_PARAMS(const SkPath&));
void AddSkiaPath(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* , const SkPath& 
        OP_DEBUG_PARAMS(UnaryContour& , size_t , PathOpsV0Lib::DebugContourType ));
bool VeryLargeSkiaPath(const SkPath& );

#if TEST_ANALYZE
// used to break down tests that are too large to debug directly
void AddDebugSkiaPath(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* , const SkPath& path);
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

#endif
