// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebugDump_DEFINED
#define OpDebugDump_DEFINED

#if OP_DEBUG_DUMP

#include <vector>

#define DUMP_DECLARATIONS \
std::string debugDump(DebugLevel , DebugBase ) const; \
void dump() const; \
void dump(DebugLevel, DebugBase ) const; \
void dumpHex() const;

#define DUMP_DECLARATIONS_OVERRIDE \
std::string debugDump(DebugLevel , DebugBase ) const override; \
void dump() const override; \
void dump(DebugLevel, DebugBase ) const override; \
void dumpHex() const override;

// removed OP_X(ExtremaT) for now
// removed OP_X(LoopCheck) for now
#define VECTOR_STRUCTS \
OP_X(CcCenter) \
OP_X(CcSects) \
OP_X(CoinPair) \
OP_X(EdgeDistance) \
OP_X(FoundEdge) \
OP_X(HullSect) \
OP_X(OpContour) \
OP_X(OpEdge) \
OP_X(OpIntersection) \
OP_X(OpPtT) \
OP_X(OpSegment) \
OP_X(SoClose)

#define OP_STRUCTS \
OP_X(LinePts) \
OP_X(LinkUps) \
OP_X(OpContours) \
OP_X(OpCurve) \
OP_X(OpCurveCurve) \
OP_X(OpEdgeStorage) \
OP_X(OpIntersections) \
OP_X(OpInPath) \
OP_X(OpJoiner) \
OP_X(OpLimb) \
OP_X(OpOutPath) \
OP_X(OpPoint) \
OP_X(OpPointBounds) \
OP_X(OpRect) \
OP_X(OpRootPts) \
OP_X(OpRoots) \
OP_X(OpSegments) \
OP_X(OpTightBounds) \
OP_X(OpTree) \
OP_X(OpVector) \
OP_X(OpWinder) \
OP_X(SectRay)

#define OP_X(Thing) \
	struct Thing;
	VECTOR_STRUCTS
#undef OP_X

#define VECTOR_PTRS \
OP_X(OpEdge*) \
OP_X(const OpEdge*) \
OP_X(OpIntersection*) \
OP_X(OpSegment*)

#define OP_X(Thing) \
	extern void dmp(const std::vector<Thing>* ); \
	extern void dmp(const std::vector<Thing>& ); \
	extern void dmpHex(const std::vector<Thing>* ); \
	extern void dmpHex(const std::vector<Thing>& );
	VECTOR_STRUCTS
	VECTOR_PTRS
#undef OP_X

#define OP_X(Thing) \
	extern void dmp(const struct Thing* ); \
	extern void dmp(const struct Thing& ); \
	extern void dmpHex(const struct Thing& ); \
	extern void dmpHex(const struct Thing* );
	VECTOR_STRUCTS
	OP_STRUCTS
#undef OP_X

#define DUMP_GROUP \
OP_X(Active) \
OP_X(Contours) \
OP_X(Edges) \
OP_X(Intersections) \
OP_X(Sects) \
OP_X(Segments)

#define OP_X(Thing) \
	extern void dmp##Thing();
	DUMP_GROUP
#undef OP_X

#define DEBUG_DUMP \
OP_X(Detail) \
OP_X(Edges) \
OP_X(Full) \
OP_X(ID) \
OP_X(Intersections) \
OP_X(Link) \
OP_X(Points) \
OP_X(Winding)

#define DEBUG_DUMP_ID_DEFINITION(OWNER, ID) \
	std::string OWNER::debugDumpID() const { \
		return std::to_string(ID); \
	}

#define DUMP_POINT \
OP_X(Match)

#define DETAIL_POINTS \
OP_X(Match, Intersection) \
OP_X(Match, Point) \
OP_X(Match, PtT)

#define OP_X(Thing, Struct) \
extern void dmp##Thing(const Op##Struct* ); \
extern void dmp##Thing(const Op##Struct& );
DETAIL_POINTS
#undef OP_X

#define EDGE_OR_SEGMENT_DETAIL \
OP_X(End) \
OP_X(Start)

#define EDGE_DETAIL \
OP_X(Center) \
OP_X(Link) \
OP_X(Points)

#define OP_X(Thing) \
extern void dmp##Thing(int ID); \
extern void dmp##Thing(const OpEdge* ); \
extern void dmp##Thing(const OpEdge& );
EDGE_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X

#define SEGMENT_DETAIL \
OP_X(Full) \
OP_X(SegmentEdges) \
OP_X(SegmentIntersections) \
OP_X(SegmentSects) \
OP_X(SoClose)

#define OP_X(Thing) \
extern void dmp##Thing(int ID); \
extern void dmp##Thing(const OpContour* ); \
extern void dmp##Thing(const OpContour& ); \
extern void dmp##Thing(const OpSegment* ); \
extern void dmp##Thing(const OpSegment& );
SEGMENT_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X

#ifdef OP_HAS_FMA
#define DUMP_BY_DUMPID \
OP_X(dmp, dump) \
OP_X(dmpHex, dumpHex)
#endif
#ifdef OP_NO_FMA
#define DUMP_BY_DUMPID \
OP_X(dmpid, dump) \
OP_X(dmpHex, dumpHex)
#endif
#define OP_X(Global, Method) \
	extern void Global(int id);
DUMP_BY_DUMPID
#undef OP_X

#define DUMP_BY_ID \
OP_X(Hex)

extern void dmpActive();
extern void dmpCoincidences();
extern void dmpCoins();
extern void dmpDisabled();
extern void dmpEdges();
extern void dmpInOutput();
extern void dmpIntersections();
extern void dmpJoin();
extern void dmpSects();
extern void dmpSegments();
extern void dmpSegments();
extern void dmpSoClose();
extern void dmpUnsectable();
extern void dmpUnsortable();

#if OP_DEBUG_VERBOSE
extern void dmpDepth(int level);  // curve-curve intermediate edges created at some recursive depth
extern void dmpDepth();  // curve-curve intermediate edges at all depths 
#endif

extern std::vector<const OpIntersection*> findCoincidence(int id);
extern const OpContour* findContour(int id);
extern const OpEdge* findEdge(int id);
extern std::vector<const OpEdge*> findEdgeOutput(int id);
extern std::vector<const OpEdge*> findEdgeRayMatch(int id);
extern std::vector<const OpEdge*> findEdgeUnsectable(int id);
extern const OpIntersection* findIntersection(int id);
extern const OpLimb* findLimb(int id);
extern std::vector<const OpIntersection*> findSectUnsectable(int id);
extern const OpSegment* findSegment(int id);

enum class DebugBase {
    dec,
    hex,
	hexdec,
};

enum class DebugLevel {
	brief,
	normal,
	detailed,
    error      // displays uninitialized and error conditions like nan and infinities
};

// for typing in immediate window as parameters to dmpBase
// commented out here to avoid declaration shadowing, but defined for real at bottom of cpp file
// const int dec = 0;
// const int hex = 1;
// const int hexdec = 2;

// for typing in immediate window as parameters to dmpLevel
// commented out here to avoid declaration shadowing, but defined for real at bottom of cpp file
// const int brief = 0;
// const int normal = 1;
// const int detailed = 2;

extern void dmpBase(int );  // set to dec, hex, hexdec
extern void dmpClosest(const OpCurveCurve& , const OpPoint& );
extern std::string debugDumpColor(uint32_t c);
extern void dmpColor(uint32_t );
extern void dmpColor(const OpEdge* );
extern void dmpColor(const OpEdge& );
extern void dmpFilters();  // returns current filter settings
extern void dmpHex(float );
extern void dmpLevel(int  );  // set to brief, normal, detailed
extern void dmpPlayback(FILE* );
extern void dmpRecord(FILE * );
extern void dmpT(int ID, float t);
extern void dmpT(const OpEdge* s, float t);
extern void dmpT(const OpSegment* s, float t);
extern void dmpWidth(int );  // max chars before inserting linefeed

enum class EF;
typedef EF EdgeFilter;
extern void addAlways(EdgeFilter);
extern void clearAlways(EdgeFilter);
extern void addFilter(EdgeFilter);
extern void clearFilter(EdgeFilter);

// !!! working around laptop compiler bug; testing new w/o breaking old...
extern void dp(const OpEdge* );
extern void dp(const OpEdge& );
extern void dp(int id);

#endif

#endif
