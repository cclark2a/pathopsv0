#ifndef OpDebugImage_DEFINED
#define OpDebugImage_DEFINED

#if OP_DEBUG_IMAGE

#include <string>
#include <vector>

struct LinePts;
struct LinkUps;
struct OpContour;
struct OpContours;
struct OpCurve;
struct OpDebugRay;
struct OpEdge;
struct OpInPath;
struct OpIntersection;
struct OpLine;
struct OpOutPath;
struct OpPoint;
struct OpPointBounds;
struct OpPtT;
struct OpRect;
struct OpSegment;
struct OpVector;
enum class Axis : int8_t;

constexpr int bitmapWH = 1000;
static_assert(bitmapWH / 2 * 2 == bitmapWH);	// must be multiple of 2

// it should not be necessary to call these implementation functions directly
struct OpDebugImage {
	static void add(const OpEdge* );
	static void add(const OpSegment* );
	static void add(Axis axis, float value);
	static void add(const OpDebugRay& );

	static void addArrowHeadToPath(const OpLine& , class SkPath& );
	static void addDiamondToPath(OpPoint , class SkPath& );
	static void addToPath(const OpCurve& , class SkPath& );
	static void center(int id, bool add);
	static void clearIntersections();
	static void clearLines();
	static void clearLocalEdges();
	static void clearScreen();
	static void drawDoubleCenter(OpPoint , bool add);
	static void drawDoubleFocus();
	static void drawDoubleFocus(const OpRect& , bool add);
	static void drawGrid();
	static void drawDoubleFill(const class SkPath& path, uint32_t color = 0xFF000000, 
			bool stroke = false);
	static void drawDoublePath(const class SkPath& path, uint32_t color = 0xFF000000, 
			float strokeWidth = 0);
	static bool drawEdgeNormal(OpVector norm, OpPoint midTPt, int edgeID, 
			uint32_t color = 0xFF000000);
	static bool drawEdgeTangent(OpVector tan, OpPoint midTPt, int edgeID, 
			uint32_t color = 0xFF000000);
	static bool drawEdgeWinding(OpVector norm, OpPoint midTPt, const OpEdge* edge, uint32_t color);
	static void drawPath(const class SkPath& path, uint32_t color = 0xFF000000);
	static void drawPoints();
	static bool drawValue(OpPoint pt, std::string ptStr, uint32_t color = 0xFF000000);
	static void find(int id, OpPointBounds* , OpPoint* );
	static void focus(int id, bool add);
	static void focusEdges();
	static void init(const OpInPath& left, const OpInPath& right);
};

// call these inline or from the immediate window while debugging
extern void add(const std::vector<OpEdge>& );  // to draw edge list built from intersections
extern void add(const std::vector<OpEdge*>& ); // to draw unsortables
extern void addFocus(int id);
extern void addFocus(const OpContour& );
extern void addFocus(const OpContours& );
extern void addFocus(const OpEdge& );
extern void addFocus(const OpIntersection& );
extern void addFocus(const OpPoint& );
extern void addFocus(const OpPtT& );
extern void addFocus(const OpRect& );
extern void addFocus(const OpSegment& );
extern void addFocus(const OpContour* );
extern void addFocus(const OpContours* );
extern void addFocus(const OpEdge* );
extern void addFocus(const OpIntersection* );
extern void addFocus(const OpPoint* );
extern void addFocus(const OpPtT* );
extern void addFocus(const OpRect* );
extern void addFocus(const OpSegment* );
extern void center();
extern void center(int id);
extern void center(float x, float y);
extern void center(const OpContour& );
extern void center(const OpContours& );
extern void center(const OpEdge& );
extern void center(const OpIntersection& );
extern void center(const OpRect& );
extern void center(const OpPoint& );
extern void center(const OpPtT& );
extern void center(const OpSegment& );
extern void center(const OpContour* );
extern void center(const OpContours* );
extern void center(const OpEdge* );
extern void center(const OpIntersection* );
extern void center(const OpRect* );
extern void center(const OpPoint* );
extern void center(const OpPtT* );
extern void center(const OpSegment* );
extern void clear();
extern void clearLines();
extern void draw(const std::vector<OpEdge>& );  // to draw edge list built from intersections
extern void draw(const std::vector<OpEdge*>& ); // to draw unsortables
extern void highlight(const LinkUps& ); // to draw assemble linkups
extern void highlightLinked(const OpEdge& );
extern void highlightLinked(const OpEdge* );
extern void draw(Axis , float );	// horizontal or vertical ray
extern void draw(const LinePts& );	// arbitrary angled ray
extern void draw();  // draw all current state
extern void focus(int id);
extern void focus(const OpContour& );
extern void focus(const OpContours& );
extern void focus(const OpEdge& );
extern void focus(const OpRect& );
extern void focus(const OpSegment& );
extern void focus(const OpContour* );
extern void focus(const OpContours* );
extern void focus(const OpEdge* );
extern void focus(const OpRect* );
extern void focus(const OpSegment* );
extern void gridCenter(int x, int y);
extern void gridLines(int );
extern void gridStep(float dxy);
extern void help();
extern void playback();
extern void precision(int );
extern void record();
extern void redraw();
extern void resetFocus();
extern void textSize(float );

#define MASTER_LIST \
OP_X(Bounds) \
OP_X(Centers) \
OP_X(Chains) \
OP_X(Coincidences) \
OP_X(Controls) \
OP_X(Edges) \
OP_X(Fill) \
OP_X(Grid) \
OP_X(Guides) \
OP_X(Hex) \
OP_X(Highlight) \
OP_X(IDs) \
OP_X(Intersections) \
OP_X(Left) \
OP_X(Lines) \
OP_X(Normals) \
OP_X(Operands) \
OP_X(Outputs) \
OP_X(Paths) \
OP_X(Points) \
OP_X(Right) \
OP_X(SegmentEdges) \
OP_X(Segments) \
OP_X(Sums) \
OP_X(Tangents) \
OP_X(TemporaryEdges) \
OP_X(Ts) \
OP_X(Values) \
OP_X(Windings)

#define OP_X(Thing) \
	extern void hide##Thing(); \
	extern void show##Thing(); \
	extern void toggle##Thing();
	MASTER_LIST
#undef OP_X

extern void u(float );
extern void u();
extern void d(float );
extern void d();
extern void l(float );
extern void l();
extern void r(float );
extern void r();
extern void i(float );
extern void i();
extern void oo(float );
extern void oo();

#endif

#endif
