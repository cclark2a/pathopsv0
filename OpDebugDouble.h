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
void DebugOpClearPoints();
void DebugOpBuild(const OpPoint& );
void DebugOpBuild(const OpPoint& , bool opp);
void DebugOpBuild(const OpPoint& , float t, bool opp);
void DebugOpDraw(const std::vector<OpRay>& );
void DebugOpDraw(const std::vector<const OpEdge*>& );
void DebugOpDraw(const std::vector<const SkPath*>& );
void DebugOpDraw(const std::vector<const OpSegment*>& );
void DebugOpDrawDiamond();
void DebugOpDrawEdgeIDs(const std::vector<OpEdge>& , std::vector<int>& ids, bool opp);
void DebugOpDrawEdgeIDs(const std::vector<const OpEdge*>& , std::vector<int>& ids);
void DebugOpDrawFocus();
void DebugOpDrawIntersectionIDs(const std::vector<const OpIntersection*>& , std::vector<int>& ids);
void DebugOpDrawPaths();
void DebugOpDrawPointIDs(const std::vector<const OpSegment*>& , std::vector<int>& ids);
void DebugOpDrawSegmentIDs(const std::vector<const OpSegment*>& , std::vector<int>& ids);
void DebugOpDrawT(bool inHex);
void DebugOpDrawValue(bool inHex);
void DebugOpEdgeCenter(const OpEdge* , OpPoint& );
void DebugOpOffsetCenter(double dx, double dy);
void DebugOpOffsetZoom(double dz);
void DebugOpPtToPt(const OpPoint& src, OpPoint& dst);
void DebugOpResetFocus();
void DebugOpSetBounds(double, double, double, double);
void DebugOpSetCenter(double x, double y);
void DebugOpSetZoom(double z);
double DebugOpTranslate(double s);
void DebugOpBounds(double& left, double& top, double& right, double& bottom);

// used by testing
void OpCubicPtAtT(const OpCubic& , float, OpPoint& );
void OpCubicAxisRayHit(const OpCubic& c, Axis offset, float axisIntercept, std::array<float, 5>& cepts, int& roots);

#endif
