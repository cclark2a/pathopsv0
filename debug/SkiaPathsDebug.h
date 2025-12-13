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

#if OP_DEBUG_DUMP
// std::string dumpUnaryContourFunc(DebugContourData , DebugLevel , DebugBase );
// std::string dumpBinaryContourFunc(DebugContourData caller, DebugLevel , DebugBase );
std::string dumpBinaryContextFunc(DebugContextData caller, DebugLevel , DebugBase );
#endif
#if 0
void* debugSimplifyPathFunc(DebugContourData );
bool debugSimplifyGetDrawFunc(DebugContourData );
void debugSimplifySetDrawFunc(DebugContourData , bool draw);
void* debugOpPathFunc(DebugContourData );
bool debugOpGetDrawFunc(DebugContourData );
void debugOpSetDrawFunc(DebugContourData , bool draw);
bool debugOpSetIsOppFunc(DebugContourData , int opp);
#endif

std::string dumpSkiaOutPath(Context* context);

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

#if 0
struct AddDebugContour {
    void add(PathOpsV0Lib::Contour* );

    UnaryContour& debugData;
    size_t debugSize;
    PathOpsV0Lib::DebugContourType debugContourType;
};
#endif

void SetSkiaSimplifyCallbacksDebug(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* , 
        const SkPath& );
// void SetSkiaOpContextCallbacksDebug(PathOpsV0Lib::Context* , SkPathOp );
void SetSkiaOpContourCallbacksDebug(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* ,
        PathOpsV0Lib::BinaryOperand , const SkPath& );

#if TEST_ANALYZE
// used to break down tests that are too large to debug directly
bool DebugAnalyze(PathOpsV0Lib::Context* );
void AddDebugSkiaPath(PathOpsV0Lib::Context* , PathOpsV0Lib::Contour* , const SkPath& path);
#endif

#if !OP_TINY_SKIA
std::string dumpSkPath(const SkPath* path, bool inHex);
#endif

// !!! unsure if visual debugger needs these or not in the long run; for now, disable
#if 0 && OP_DEBUG_DUMP && OP_DEBUG
#define DEBUG_SKIAPATH_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(dumpUnaryContourFunc), \
    OP_TAGGED_FUNCTION(dumpBinaryContourFunc), \
    OP_TAGGED_FUNCTION(dumpBinaryContextFunc), \
    OP_TAGGED_FUNCTION(debugSimplifyPathFunc), \
    OP_TAGGED_FUNCTION(debugSimplifyGetDrawFunc), \
    OP_TAGGED_FUNCTION(debugSimplifySetDrawFunc), \
    OP_TAGGED_FUNCTION(debugOpPathFunc), \
    OP_TAGGED_FUNCTION(debugOpGetDrawFunc), \
    OP_TAGGED_FUNCTION(debugOpSetDrawFunc), \
    OP_TAGGED_FUNCTION(debugOpSetIsOppFunc), \

#endif


#endif
#endif
