// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef SkiaPathsDebug_DEFINED
#define SkiaPathsDebug_DEFINED

#include "DebugOpsTypes.h"
#include "PathOpsTypes.h"

#if OP_DEBUG

class SkPath;

namespace PathOpsV0Lib {

enum class BinaryOperand : int;
enum class BinaryWindType : int;

}

struct UnaryContour {
    UnaryContour()
        : pathPtr(nullptr)
        , contourIndex(0)
        , drawNativePath(false) {
    }

    UnaryContour(const SkPath* path) 
        : pathPtr(path)
        , contourIndex(0)
        , drawNativePath(false) {
    }

    const SkPath* pathPtr;
    int contourIndex;
	bool drawNativePath;
};

struct BinaryContour : public UnaryContour {
	PathOpsV0Lib::BinaryOperand operand;
};

struct AddDebugContour {
    void add(PathOpsV0Lib::Contour* );

    UnaryContour& debugData;
    size_t debugSize;
    PathOpsV0Lib::DebugContourType debugContourType;
};

void SetSkiaSimplifyCallbacksDebug(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* , 
        const SkPath& );
void SetSkiaOpContextCallbacksDebug(PathOpsV0Lib::Context* , SkPathOp );
void SetSkiaOpContourCallbacksDebug(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* ,
        PathOpsV0Lib::BinaryOperand , const SkPath& );

#if TEST_ANALYZE
// used to break down tests that are too large to debug directly
void AddDebugSkiaPath(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* , const SkPath& path);
#endif

#if !OP_TINY_SKIA
std::string dumpSkPath(const SkPath* path, bool inHex);
#endif

#endif
#endif
