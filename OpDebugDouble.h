#ifndef OpDebugDouble_DEFINED
#define OpDebugDouble_DEFINED

#include <vector>

enum class Axis : uint8_t ;
struct OpEdge;
class SkPath;
struct OpCubic;
struct OpIntersection;
struct OpPoint;
struct OpPointBounds;
struct OpRay;
struct OpSegment;

void DebugOpAddBounds(double, double, double, double);
void DebugOpBounds(double& left, double& top, double& right, double& bottom);
void DebugOpClearEdges();
void DebugOpClearPoints();
void DebugOpBuild(const OpPoint& );
void DebugOpBuild(const OpPoint& , bool opp);
void DebugOpBuild(const OpPoint& , float t, bool opp);
void DebugOpBuild(const OpEdge& , Axis, float ray);
void DebugOpBuild(const OpSegment& , Axis, float ray);
void DebugOpBuild(const SkPath& , Axis, float ray);
void DebugOpDraw(const std::vector<OpRay>& );
void DebugOpDraw(const std::vector<OpEdge>& );
void DebugOpDraw(const std::vector<const OpEdge*>& );
void DebugOpDraw(const std::vector<const SkPath*>& );
void DebugOpDraw(const std::vector<const OpSegment*>& );
void DebugOpDrawDiamond();
void DebugOpDrawEdgeIDs(const std::vector<OpEdge>& , std::vector<int>& ids, bool opp);
void DebugOpDrawEdgeIDs(const std::vector<const OpEdge*>& , std::vector<int>& ids);
void DebugOpDrawIntersectionIDs(const std::vector<const OpIntersection*>& , std::vector<int>& ids);
void DebugOpDrawPointIDs(const std::vector<const OpSegment*>& , std::vector<int>& ids);
void DebugOpDrawSegmentIDs(const std::vector<const OpSegment*>& , std::vector<int>& ids);
void DebugOpDrawT(bool inHex, int precision);
void DebugOpDrawValue(bool inHex, int precision);
void DebugOpEdgeCenter(const OpEdge* , OpPoint& );
double DebugOpGetZoomScale(); // zoom factor (pow of zoom number)
bool DebugOpHasT();
void DebugOpOffsetCenter(double dx, double dy);
void DebugOpOffsetZoom(double dz);
void DebugOpPtToPt(const OpPoint& src, OpPoint& dst);
void DebugOpResetFocus();
void DebugOpSetBounds(double, double, double, double);
void DebugOpSetCenter(double x, double y);
void DebugOpSetZoom(double z);	// zoom number (log2 of zoom factor)
void DebugOpSetZoomScale(double z);	// zoom factor (pow of zoom number)
double DebugOpTranslate(double s);

// used by testing
void OpCubicPtAtT(const OpCubic& , float, OpPoint& );
void OpCubicAxisRayHit(const OpCubic& c, Axis offset, float axisIntercept, std::array<float, 5>& cepts, int& roots);

#endif
