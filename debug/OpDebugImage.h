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
struct OpLine;
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

// it should not be necessary to call these implementation functions directly
struct OpDebugImage {
	static void add(Axis axis, float value);
	static void add(const OpDebugRay& );
	static void add(const OpPtT& );
	static void addArrowHeadToPath(const OpLine& , class SkPath& );
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
	static bool drawCurve(const OpCurve& , uint32_t color = debugBlack);
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
	static bool drawTangent(OpVector tan, OpPoint midTPt, int edgeID, 
			uint32_t color = debugBlack);
	static bool drawValue(OpPoint pt, std::string ptStr, uint32_t color = debugBlack);
	static bool drawWinding(const OpCurve& , std::string left, std::string right,
			float normSign, uint32_t color);
	static bool find(int id, OpPointBounds* , OpPoint* );
	static OpPoint find(int id, float t);
	static std::vector<const OpEdge*> find(int id);
	static void focus(int id, bool add);
	static void focusEdges();
	static void init();
	static void playback(FILE* );
	static void record(FILE* );
};

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
extern void draw(std::vector<OpEdge>& );  // to draw edge list built from intersections
extern void draw(std::vector<OpEdge*>& ); // to draw unsortables
extern void draw(Axis , float );	// horizontal or vertical ray
extern void draw(const LinePts& );	// arbitrary angled ray
extern void draw(const OpLine& );	// arbitrary angled ray
extern void draw(const OpPoint& );
extern void draw(const OpPtT& );   // draw point (ignores t)
extern void draw(const OpPoint* );
extern void draw(const OpPtT* );
extern void draw(float , float );
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
extern void gridCenter(int x, int y);
extern void gridLines(int );
extern void gridStep(float dxy);
extern void help();
extern void hideSegmentEdges();  // not in master list; immediately changes edge state
extern void hideTemporaryEdges();
extern void precision(int );
extern void redraw();  // draw all current state
extern void resetFocus();
extern void showSegmentEdges();  // not in master list; immediately changes edge state
extern void showTemporaryEdges();
extern void textSize(float );
extern void toggleSegmentEdges();  // not in master list; immediately changes edge state
extern void toggleTemporaryEdges();

#if OP_DEBUG_VERBOSE
extern void depth(int level);  // during curve-curve intersection, select edges at given level 
#endif

#define MASTER_LIST \
OP_X(Bounds) \
OP_X(Centers) \
OP_X(Coincidences) \
OP_X(ControlLines) \
OP_X(Controls) \
OP_X(EdgesOut) \
OP_X(Edges) \
OP_X(EndToEnd) \
OP_X(Grid) \
OP_X(Guides) \
OP_X(Hex) \
OP_X(Hulls) \
OP_X(IDs) \
OP_X(Intersections) \
OP_X(Left) \
OP_X(Lines) \
OP_X(Normals) \
OP_X(PathsOut) \
OP_X(Points) \
OP_X(Rays) \
OP_X(Result) \
OP_X(Right) \
OP_X(Segments) \
OP_X(Sums) \
OP_X(Tangents) \
OP_X(Ts) \
OP_X(Values) \
OP_X(Windings)

#define ALIAS_LIST \
OP_X(Fill) \
OP_X(In) \
OP_X(Operands)

#define OP_X(Thing) \
	extern void hide##Thing(); \
	extern void show##Thing(); \
	extern void toggle##Thing();
	MASTER_LIST
	ALIAS_LIST
#undef OP_X

#define COLOR_LIST \
OP_X(Active) \
OP_X(Between) \
OP_X(Disabled) \
OP_X(Linkups) \
OP_X(Opp) \
OP_X(Out) \
OP_X(PathsOut) \
OP_X(Unsectables) \
OP_X(Unsortables)

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

#endif

#endif
