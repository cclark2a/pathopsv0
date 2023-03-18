#ifndef OpDebugImage_DEFINED
#define OpDebugImage_DEFINED

#if OP_DEBUG_IMAGE

#include <array>
#include <string>
#include <vector>

struct OpCurve;
struct OpEdge;
struct OpInPath;
struct OpIntersection;
struct OpLine;
struct OpOutPath;
struct OpPoint;
struct OpPointBounds;
struct OpPtT;
struct OpRay;
struct OpSegment;
enum class Axis : uint8_t;
typedef const OpPointBounds* ConstOpPointBoundsPtr;
typedef const OpPoint* ConstOpPointPtr;

constexpr int bitmapWH = 1000;

// it should not be necessary to call these implementation functions directly
struct OpDebugImage {
	static void add(const OpEdge* );
	static void add(const OpIntersection* );
	static void add(const OpSegment* );
	static void add(Axis axis, float value);
	static void add(const OpRay& );

	static void addArrowHeadToPath(const OpLine& , class SkPath& );
	static void addDiamondToPath(OpPoint , class SkPath& );
	static void addToPath(const OpCurve& , class SkPath& );
	static void center(int id, bool add);
	static void clearIntersections();
	static void clearLines();
	static void clearScreen();
	static void clearTemporaryEdges();
	static void drawDoubleCenter(OpPoint , bool add);
	static void drawDoubleFocus();
	static void drawDoubleFocus(const OpPointBounds& , bool add);
	static void drawGrid();
	static void drawDoublePath(const class SkPath& path, uint32_t color = 0xFF000000, bool stroke = false);
	static void drawEdgeNormals(const std::vector<const OpEdge*>& );
	static void drawEdgeNormals(const std::vector<OpEdge>& );
	static void drawEdgeWindings(const std::vector<const OpEdge*>& );
	static void drawEdgeWindings(const std::vector<OpEdge>& );
	static void drawLines();
	static void drawPath(const class SkPath& path);
	static void drawPoints();
	static void drawValue(OpPoint pt, std::string ptStr, uint32_t color = 0xFF000000);
	static void find(int id, ConstOpPointBoundsPtr* ,ConstOpPointPtr* );
	static void focus(int id, bool add);
	static void focusEdges();
	static void init(const OpInPath& left, const OpInPath& right);
};

// call these inline or from the immediate window while debugging
extern void addEdges();
extern void addIntersections();
extern void addFocus(int id);
extern void addSegments();
extern void center(int id);
extern void center(float x, float y);
extern void center(const OpPoint& );
extern void center(const OpPtT& );
extern void clear();
extern void clearLines();

extern void draw(const std::vector<OpEdge>& );  // to draw edge list built from intersections
extern void draw(const std::vector<OpEdge*>& ); // to draw assemble linkups
extern void draw(Axis , float );	// horizontal or vertical ray
// commented out because OpPoint isn't defined in this context (this draw is in OpCurve.h)
// extern void draw(const std::array<OpPoint, 2>& );	// arbitrary angled ray
extern void draw();  // draw all current state

extern void focus(int id);
extern void gridCenter(int x, int y);
extern void gridLines(int );
extern void gridStep(float dxy);
extern void help();
extern void precision(int );
extern void resetFocus();

#define HIDE_SHOW_DECLARATION(Thing) \
extern void hide##Thing(); \
extern void show##Thing(); \
extern void toggle##Thing()

HIDE_SHOW_DECLARATION(Arrows);
HIDE_SHOW_DECLARATION(Bounds);
HIDE_SHOW_DECLARATION(Centers);
HIDE_SHOW_DECLARATION(Chains);
HIDE_SHOW_DECLARATION(Coincidences);
HIDE_SHOW_DECLARATION(Controls);
HIDE_SHOW_DECLARATION(Edges);
HIDE_SHOW_DECLARATION(Grid);
HIDE_SHOW_DECLARATION(Hex);
HIDE_SHOW_DECLARATION(IDs);
HIDE_SHOW_DECLARATION(Intersections);
HIDE_SHOW_DECLARATION(Left);
HIDE_SHOW_DECLARATION(Lines);
HIDE_SHOW_DECLARATION(Normals);
HIDE_SHOW_DECLARATION(Operands);
HIDE_SHOW_DECLARATION(Paths);
HIDE_SHOW_DECLARATION(Points);
HIDE_SHOW_DECLARATION(Right);
HIDE_SHOW_DECLARATION(SegmentEdges);
HIDE_SHOW_DECLARATION(Segments);
HIDE_SHOW_DECLARATION(Sums);
HIDE_SHOW_DECLARATION(Tangents);
HIDE_SHOW_DECLARATION(TemporaryEdges);
HIDE_SHOW_DECLARATION(Ts);
HIDE_SHOW_DECLARATION(Values);
HIDE_SHOW_DECLARATION(Windings);

#undef HIDE_SHOW_DECLARATION

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
