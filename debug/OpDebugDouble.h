// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebugDouble_DEFINED
#define OpDebugDouble_DEFINED

//#ifndef _WIN32
#include <float.h>
//#endif

namespace PathOpsV0Lib {
struct Curve;
struct ColorCurve;
}

enum class Axis : int8_t ;
struct ColorCurve;
struct OpContour;
struct OpEdge;
class SkPath;
struct OpIntersection;
struct OpInPath;
struct OpOutPath;
struct OpPoint;
// struct OpPointBounds;
struct OpRect;
struct OpDebugRay;
struct OpSegment;

enum class DebugSprite {
    circle,
    diamond,
    square,
    triangle,
};


void DebugOpAdd(const OpSegment* );
void DebugOpAdd(const OpEdge* );
void DebugOpAdd(const OpInPath& );
void DebugOpAddBounds(double, double, double, double);
void DebugOpAddHighlight(const OpEdge* );
void DebugOpBounds(double& left, double& top, double& right, double& bottom);
void DebugOpBuild(OpPoint , uint32_t color);
void DebugOpBuild(OpPoint , uint32_t color, bool opp);
void DebugOpBuild(OpPoint , uint32_t color, float t, bool opp);
void DebugOpBuild(OpPoint , uint32_t color, float t, DebugSprite );
void DebugOpBuild(Axis axis, float normal, float cept);
void DebugOpBuild(const OpEdge& , const OpDebugRay& );
void DebugOpBuild(const OpSegment& , const OpDebugRay& );
void DebugOpBuild(const SkPath& , const OpDebugRay& );
void DebugOpBuild(const PathOpsV0Lib::ColorCurve& );
void DebugCurveBuild(const PathOpsV0Lib::ColorCurve& , const OpDebugRay& );
void DebugOpClearEdges();
void DebugOpClearHighlight();
void DebugOpClearInputs();
void DebugOpClearPoints();
void DebugOpClearSegments();
void DebugOpDraw(const OpOutPath* , uint32_t color);
void DebugOpDraw(const std::vector<OpRect>& );
void DebugOpDraw(const std::vector<OpDebugRay>& );
void DebugOpDraw(const std::vector<PathOpsV0Lib::ColorCurve>& );
void DebugOpDraw(const std::vector<OpEdge>& );
void DebugOpDraw(const std::vector<const OpEdge*>& );
void DebugOpDraw(const std::vector<const SkPath*>& );
void DebugOpDrawContourID(const OpContour* contour, std::vector<int>& ids);
void DebugOpDrawCurveControlLines(const PathOpsV0Lib::ColorCurve& , uint32_t color);
void DebugOpDrawCurveEndToEnd(const PathOpsV0Lib::ColorCurve& , uint32_t color);
void DebugOpDrawCurveNormal(const PathOpsV0Lib::ColorCurve& , uint32_t color);
void DebugOpDrawCurveTangent(const PathOpsV0Lib::ColorCurve& , uint32_t color);
void DebugOpDrawEdgeControlLines(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeEndToEnd(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeID(const OpEdge* , uint32_t color, bool drawLimbs);
void DebugOpDrawEdgeNormal(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeTangent(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeWinding(const OpEdge* , uint32_t color);
void DebugOpDrawEdges();
void DebugOpDrawHighlight();
void DebugOpDrawInputs();
void DebugOpDrawSegmentControlLines(const OpSegment* , uint32_t color);
void DebugOpDrawSegmentEndToEnd(const OpSegment* , uint32_t color);
void DebugOpDrawSegmentNormal(const OpSegment* , uint32_t color);
void DebugOpDrawSegmentTangent(const OpSegment* , uint32_t color);
void DebugOpDrawSegments();
void DebugOpDrawSprites();
void DebugOpDrawIntersectionID(const OpIntersection* , std::vector<int>& ids);
void DebugOpDrawPointID(const OpSegment* , std::vector<int>& ids);
void DebugOpDrawSegmentID(const OpSegment* , std::vector<int>& ids);
void DebugOpDrawT(bool inHex);
void DebugOpDrawValue(bool inHex);
void DebugOpEdgeCenter(const OpEdge* , OpPoint );
void DebugOpFill(const OpInPath& , uint32_t color);
double DebugOpGetCenterX();
double DebugOpGetCenterY();
double DebugOpGetOffsetX();
double DebugOpGetOffsetY();
double DebugOpGetZoomScale(); // zoom factor (pow of zoom number)
void DebugOpHighlight(const std::vector<const OpEdge*>& );
void DebugOpOffsetCenter(double dx, double dy);
void DebugOpOffsetZoom(double dz);
OpPoint DebugOpPtToPt(OpPoint src);
void DebugOpRecord(FILE* recordFile);
void DebugOpResetBounds();
void DebugOpResetFocus();
void DebugOpScreenBounds(int& left, int&top, int& right, int& bottom);
void DebugOpSetBounds(double, double, double, double);
void DebugOpSetCenter(double x, double y);
void DebugOpSetZoom(double z);	// zoom number (log2 of zoom factor)
void DebugOpSetZoomScale(double z);	// zoom factor (pow of zoom number)
double DebugOpTranslate(double s);

#endif
