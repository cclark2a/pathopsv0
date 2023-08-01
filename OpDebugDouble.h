#ifndef OpDebugDouble_DEFINED
#define OpDebugDouble_DEFINED

#include <vector>

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

void DebugOpAdd(const OpSegment* );
void DebugOpAdd(const OpEdge* );
void DebugOpAdd(const OpInPath& );
void DebugOpAddBounds(double, double, double, double);
void DebugOpAddHighlight(const OpEdge* );
void DebugOpBounds(double& left, double& top, double& right, double& bottom);
void DebugOpBuild(OpPoint );
void DebugOpBuild(OpPoint , bool opp);
void DebugOpBuild(OpPoint , float t, bool opp);
void DebugOpBuild(const OpEdge& , const struct OpDebugRay& );
void DebugOpBuild(const OpSegment& , const struct OpDebugRay& );
void DebugOpBuild(const SkPath& , const struct OpDebugRay& );
void DebugOpClearEdges();
void DebugOpClearHighlight();
void DebugOpClearInputs();
void DebugOpClearPoints();
void DebugOpClearSegments();
void DebugOpDraw(const OpOutPath* );
void DebugOpDraw(const std::vector<OpDebugRay>& );
void DebugOpDraw(const std::vector<OpEdge>& );
void DebugOpDraw(const std::vector<const OpEdge*>& );
void DebugOpDraw(const std::vector<const SkPath*>& );
void DebugOpDrawDiamond();
void DebugOpDrawEdges();
void DebugOpDrawHighlight();
void DebugOpDrawInputs();
void DebugOpDrawSegments();
void DebugOpDrawEdgeID(const OpEdge* , uint32_t color);
void DebugOpDrawIntersectionID(const OpIntersection* , std::vector<int>& ids);
void DebugOpDrawPointID(const OpSegment* , std::vector<int>& ids);
void DebugOpDrawSegmentID(const OpSegment* , std::vector<int>& ids);
void DebugOpDrawT(bool inHex, int precision);
void DebugOpDrawValue(bool inHex, int precision);
void DebugOpDrawEdgeNormal(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeTangent(const OpEdge* , uint32_t color);
void DebugOpDrawEdgeWinding(const OpEdge* , uint32_t color);
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
void DebugOpSetBounds(double, double, double, double);
void DebugOpSetCenter(double x, double y);
void DebugOpSetZoom(double z);	// zoom number (log2 of zoom factor)
void DebugOpSetZoomScale(double z);	// zoom factor (pow of zoom number)
double DebugOpTranslate(double s);

// used by testing
void OpCubicPtAtT(const OpCubic& , float, OpPoint );
void OpCubicAxisRayHit(const OpCubic& c, Axis offset, float axisIntercept, std::array<float, 5>& cepts, int& roots);

#endif
