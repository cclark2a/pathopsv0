// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebugDouble_DEFINED
#define OpDebugDouble_DEFINED

#include <vector>
//#ifndef _WIN32
#include <float.h>
//#endif

enum class Axis : int8_t ;
struct OpEdge;
class SkPath;
struct OpCubic;
struct OpIntersection;
struct OpInPath;
struct OpOutPath;
struct OpPoint;
struct OpPointBounds;
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
void DebugOpBuild(OpPoint );
#if OP_TEST_NEW_INTERFACE
void DebugOpBuild(OpPoint , uint32_t color);
void DebugOpBuild(OpPoint , float t, uint32_t color);
#else
void DebugOpBuild(OpPoint , bool opp);
void DebugOpBuild(OpPoint , float t, bool opp);
#endif
void DebugOpBuild(OpPoint , float t, DebugSprite );
void DebugOpBuild(Axis axis, float normal, float cept);
void DebugOpBuild(const OpEdge& , const struct OpDebugRay& );
void DebugOpBuild(const OpSegment& , const struct OpDebugRay& );
void DebugOpBuild(const SkPath& , const struct OpDebugRay& );
void DebugOpClearEdges();
void DebugOpClearHighlight();
void DebugOpClearInputs();
void DebugOpClearPoints();
void DebugOpClearSegments();
void DebugOpDraw(const OpOutPath* , uint32_t color);
void DebugOpDraw(const std::vector<OpDebugRay>& );
void DebugOpDraw(const std::vector<OpEdge>& );
void DebugOpDraw(const std::vector<const OpEdge*>& );
void DebugOpDraw(const std::vector<const SkPath*>& );
void DebugOpDrawEdgeControlLines(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeEndToEnd(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeID(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeNormal(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeTangent(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeWinding(const OpEdge* , uint32_t color);
void DebugOpDrawEdges();
void DebugOpDrawHighlight();
void DebugOpDrawInputs();
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

// used by testing
void OpCubicPtAtT(const OpCubic& , float, OpPoint );
void OpCubicAxisRayHit(const OpCubic& c, Axis offset, float axisIntercept, std::array<float, 5>& cepts, int& roots);

#endif
