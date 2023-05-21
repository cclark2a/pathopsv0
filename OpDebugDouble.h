#ifndef OpDebugDouble_DEFINED
#define OpDebugDouble_DEFINED

#include <vector>

enum class Axis : uint8_t ;
struct OpEdge;
class SkPath;
struct OpCubic;
struct OpIntersection;
struct OpInPath;
struct OpOutPath;
struct OpPoint;
struct OpPointBounds;
struct OpRay;
struct OpSegment;

void DebugOpAddBounds(double, double, double, double);
void DebugOpBounds(double& left, double& top, double& right, double& bottom);
void DebugOpClearEdges();
void DebugOpClearPoints();
void DebugOpClearSegments();
void DebugOpAdd(const OpSegment* );
void DebugOpAdd(const OpEdge* );
void DebugOpAdd(const OpInPath&);
void DebugOpBuild(OpPoint );
void DebugOpBuild(OpPoint , bool opp);
void DebugOpBuild(OpPoint , float t, bool opp);
void DebugOpBuild(const OpEdge& , const struct OpRay& );
void DebugOpBuild(const OpSegment& , const struct OpRay& );
void DebugOpBuild(const SkPath& , const struct OpRay& );
void DebugOpClearInputs();
void DebugOpDraw(const std::vector<OpOutPath>& );
void DebugOpDraw(const std::vector<OpRay>& );
void DebugOpDraw(const std::vector<OpEdge>& );
void DebugOpDraw(const std::vector<const OpEdge*>& );
void DebugOpDraw(const std::vector<const SkPath*>& );
void DebugOpDrawInputs();
void DebugOpDrawEdges();
void DebugOpDrawSegments();
void DebugOpDrawDiamond();
void DebugOpDrawEdgeID(const OpEdge* , std::vector<int>& ids, uint32_t color);
void DebugOpDrawIntersectionIDs(const std::vector<const OpIntersection*>& , std::vector<int>& ids);
void DebugOpDrawPointID(const OpSegment* , std::vector<int>& ids);
void DebugOpDrawSegmentID(const OpSegment* , std::vector<int>& ids);
void DebugOpDrawT(bool inHex, int precision);
void DebugOpDrawValue(bool inHex, int precision);
void DebugOpEdgeCenter(const OpEdge* , OpPoint );
double DebugOpGetZoomScale(); // zoom factor (pow of zoom number)
bool DebugOpHasT();
void DebugOpOffsetCenter(double dx, double dy);
void DebugOpOffsetZoom(double dz);
void DebugOpPtToPt(OpPoint src, OpPoint dst);
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
