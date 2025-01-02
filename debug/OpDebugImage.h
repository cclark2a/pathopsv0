// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebugImage_DEFINED
#define OpDebugImage_DEFINED

#define GENERATE_COLOR_FILES 0
void OpDebugGenerateColorFiles();

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
struct OpOutPath;
struct OpPoint;
struct OpPointBounds;
struct OpPtT;
struct OpRect;
struct OpSegment;
struct OpVector;
struct SkRect;
enum class Axis : int8_t;

constexpr int bitmapWH = 1000;
constexpr uint32_t debugBlack = 0xFF000000;
static_assert(bitmapWH / 2 * 2 == bitmapWH);	// must be multiple of 2
class SkBitmap& bitmapRef();

// it should not be necessary to call these implementation functions directly
struct OpDebugImage {
	static void add(Axis axis, float value);
	static void add(const OpDebugRay& );
	static void add(const OpPtT& );
	static void addArrowHeadToPath(const LinePts& , class SkPath& );
	static void addCircleToPath(OpPoint , class SkPath& );
	static void addDiamondToPath(OpPoint , class SkPath& );
	static void addSquareToPath(OpPoint , class SkPath& );
	static void addTriangleToPath(OpPoint , class SkPath& );
	static void addToPath(const OpCurve& , class SkPath& );
	static bool bitsAreBlank(bool allowIntersect, SkRect& test);
	static void center(int id, bool add);
	static void clearIntersections();
	static void clearLines();
	static void clearPoints();
	static void clearScreen();
	static bool drawCurve(OpCurve& , uint32_t color = debugBlack);
	static void drawDoubleCenter(OpPoint , bool add);
	static void drawDoubleFocus();
	static void drawDoubleFocus(const OpRect& , bool add);
	static void drawGrid();
	static void drawGuide(const SkRect& test, OpPoint pt, uint32_t color);
	static void drawDoubleFill(const class SkPath& path, uint32_t color = debugBlack, 
			bool stroke = false);
	static void drawDoublePath(const class SkPath& path, uint32_t color = debugBlack, 
			float strokeWidth = 0);
	static bool drawEdgeNormal(OpVector norm, OpPoint midTPt, int edgeID, 
			uint32_t color = debugBlack);
	static bool drawEdgeWinding(const OpCurve& , const OpEdge* , uint32_t color);
	static void drawPath(const class SkPath& path, uint32_t color = debugBlack);
	static void drawPoints();
	static void drawRaster();
	static bool drawTangent(OpVector tan, OpPoint midTPt, int edgeID, 
			uint32_t color = debugBlack);
	static bool drawValue(OpPoint pt, std::string ptStr, uint32_t color = debugBlack);
	static bool drawWinding(const OpCurve& , std::string left, std::string right,
			float normSign, uint32_t color);
	static bool find(int id, OpPointBounds* , OpPoint* );
	static OpPoint find(int id, float t);
//	static std::vector<const OpEdge*> find(int id);
	static void focus(int id, bool add);
	static void focusEdges();
	static void init();
	static void playback(FILE* );
	static void record(FILE* );
};

inline uint32_t OpDebugAlphaColor(uint32_t alpha, uint32_t color) {
	return (alpha << 24) | (color & 0x00FFFFFF);
}

// call these inline or from the immediate window while debugging
extern void add(std::vector<OpEdge>& );  // to draw edge list built from intersections
extern void add(std::vector<OpEdge*>& ); // to draw unsortables
extern void addFocus(int id);
extern void addFocus(const OpContour& );
extern void addFocus(const OpContours& );
extern void addFocus(const OpEdge& );
extern void addFocus(const OpIntersection& );
extern void addFocus(const OpPoint& );  // pass by reference; VS fails by value in immediate window
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
extern void addFocusLink(int id);
extern void addFocusLink(const OpEdge& );
extern void addFocusLink(const OpEdge* );
extern void ctr();
extern void ctr(int id);
extern void ctr(float x, float y);
extern void ctr(const OpContour& );
extern void ctr(const OpContours& );
extern void ctr(const OpEdge& );
extern void ctr(const OpIntersection& );
extern void ctr(const OpRect& );
extern void ctr(const OpPoint& );
extern void ctr(const OpPtT& );
extern void ctr(const OpSegment& );
extern void ctr(const OpContour* );
extern void ctr(const OpContours* );
extern void ctr(const OpEdge* );
extern void ctr(const OpIntersection* );
extern void ctr(const OpRect* );
extern void ctr(const OpPoint* );
extern void ctr(const OpPtT* );
extern void ctr(const OpSegment* );
extern void clear();
extern void clearLines();
extern void clearPoints();
extern void color(int id);
extern void color(int id, uint32_t color);
extern void colorLink(int id, uint32_t color = 0xAbeBeBad);
extern void colorLink(const OpEdge& , uint32_t color = 0xAbeBeBad);
extern void colorLink(const OpEdge* , uint32_t color = 0xAbeBeBad);
extern void debugImage();
extern void draw(std::vector<OpEdge>& );  // to draw edge list built from intersections
extern void draw(std::vector<OpEdge*>& ); // to draw unsortables
extern void draw(Axis , float );	// horizontal or vertical ray
extern void draw(const LinePts& );	// arbitrary angled ray
extern void draw(const OpPoint& );
extern void draw(const OpPtT& );   // draw point (ignores t)
extern void draw(const OpPoint* );
extern void draw(const OpPtT* );
extern void draw(float , float );
extern void drawHex(uint32_t , uint32_t );
extern void drawT(int id, float );
extern void drawT(int id, const OpPtT& );
extern void drawT(int id, const OpPtT* );
extern void drawT(const OpSegment&, float );
extern void drawT(const OpSegment&, const OpPtT& );
extern void drawT(const OpSegment&, const OpPtT* );
extern void drawT(const OpSegment*, float );
extern void drawT(const OpSegment*, const OpPtT& );
extern void drawT(const OpSegment*, const OpPtT* );
extern void drawT(const OpEdge&, float );
extern void drawT(const OpEdge&, const OpPtT& );
extern void drawT(const OpEdge&, const OpPtT* );
extern void drawT(const OpEdge*, float );
extern void drawT(const OpEdge*, const OpPtT& );
extern void drawT(const OpEdge*, const OpPtT* );
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
extern void focusEdges();
extern void focusLink(int id);
extern void focusLink(const OpEdge& );
extern void focusLink(const OpEdge* );
extern void focusSegments();
extern void gridCenter(int x, int y);
extern void gridLines(int );
extern void gridStep(float dxy);
extern void help();
extern void limbs(int );
extern void precision(int );  // number of fractional decimals; -1 is unset
extern void redraw();  // draw all current state
extern void resetFocus();
extern void showEpsilon(bool );  // show values smaller than 100 * OpEpsilon as eps
extern void smallFloats(bool );  // set to false to show sub-epsilon values as ~0
extern void textSize(float );

#if OP_DEBUG_VERBOSE
extern void drawDepth(int level);  // during curve-curve intersection, select edges at given level 
#endif

#define MASTER_LIST \
OP_X(Bounds) \
OP_X(Centers) \
OP_X(Coincidences) \
OP_X(ControlLines) \
OP_X(Controls) \
OP_X(EdgesOut) \
OP_X(EndToEnd) \
OP_X(Fill) \
OP_X(Grid) \
OP_X(Guides) \
OP_X(Hex) \
OP_X(Hulls) \
OP_X(IDs) \
OP_X(Intersections) \
OP_X(Lines) \
OP_X(Normals) \
OP_X(PathsOut) \
OP_X(Points) \
OP_X(Raster) \
OP_X(Rays) \
OP_X(Result) \
OP_X(Segments) \
OP_X(Sums) \
OP_X(Tangents) \
OP_X(Ts) \
OP_X(Values) \
OP_X(Windings)

#define EDGE_BOOL_LIST \
OP_X(Edges) \
OP_X(Disabled) \
OP_X(Join) \
OP_X(Linkups) \
OP_X(SegmentEdges) \
OP_X(TemporaryEdges) \

#define EDGE_BOOL_LIST2 \
OP_X(Edges, false) \
OP_X(Disabled, !edge->disabled) \
OP_X(Join, !edge->debugJoin) \
OP_X(Linkups, !edge->inLinkups) \
OP_X(SegmentEdges, edge->debugJoin || edgeIter.isFiller || edgeIter.isCurveCurve) \
OP_X(TemporaryEdges, !edgeIter.isFiller && !edgeIter.isCurveCurve) \

#define ALIAS_LIST \
OP_X(EdgeRuns) \
OP_X(Operands) \
OP_X(Limbs) \
OP_X(Tree)

#define CALLOUT_LIST \
OP_X(Left) \
OP_X(Right)

#define OP_X(Thing) \
	extern void hide##Thing(); \
	extern void show##Thing(); \
	extern void toggle##Thing();
	MASTER_LIST
	EDGE_BOOL_LIST
	ALIAS_LIST
	CALLOUT_LIST
#undef OP_X

#define COLOR_LIST \
OP_X(Active) \
OP_X(Disabled) \
OP_X(Edges) \
OP_X(Limbs) \
OP_X(Linkups) \
OP_X(Opp) \
OP_X(Out) \
OP_X(PathsOut) \
OP_X(Segments) \
OP_X(Unsectables) \
OP_X(Unsortables)

#define COLOR_LIST2 \
OP_X(Active, edge->active_impl) \
OP_X(Disabled, edge->disabled) \
OP_X(Edges, true) \
OP_X(Linkups, edge->inLinkups) \
OP_X(Unsectables, edge->isUnsectable()) \
OP_X(Unsortables, Unsortable::none != edge->isUnsortable)

#define OP_X(Thing) \
	extern void color##Thing(); \
	extern void color##Thing(uint32_t color); \
	extern void color##Thing(uint8_t alpha, uint32_t color); \
	extern void uncolor##Thing();
	COLOR_LIST
#undef OP_X

extern uint32_t OP_DEBUG_MULTICOLORED;

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

namespace PathOpsV0Lib {
	struct Curve;
}

extern void debugLineAddToSkPath(PathOpsV0Lib::Curve c, class SkPath& path);
extern void debugQuadAddToSkPath(PathOpsV0Lib::Curve c, class SkPath& path);
extern void debugConicAddToSkPath(PathOpsV0Lib::Curve c, class SkPath& path);
extern void debugCubicAddToSkPath(PathOpsV0Lib::Curve c, class SkPath& path);

#endif

#endif
