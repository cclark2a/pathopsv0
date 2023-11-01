// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebugDump_DEFINED
#define OpDebugDump_DEFINED

#if OP_DEBUG_DUMP

#include <vector>

// removed OP_X(ExtremaT) for now
// removed OP_X(LoopCheck) for now
#define VECTOR_STRUCTS \
OP_X(CoinPair) \
OP_X(EdgeDistance) \
OP_X(FoundEdge) \
OP_X(OpContour) \
OP_X(OpEdge) \
OP_X(OpIntersection) \
OP_X(OpSegment)

#define OP_STRUCTS \
OP_X(LinkUps) \
OP_X(OpContours) \
OP_X(OpCurve) \
OP_X(OpCurveCurve) \
OP_X(OpEdgeStorage) \
OP_X(OpIntersections) \
OP_X(OpJoiner) \
OP_X(OpOutPath) \
OP_X(OpPtT) \
OP_X(OpPoint) \
OP_X(OpPointBounds) \
OP_X(OpRect) \
OP_X(OpSegments) \
OP_X(OpTightBounds) \
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
	extern void dmpDetail(const std::vector<Thing>* ); \
	extern void dmp(const std::vector<Thing>& ); \
	extern void dmpDetail(const std::vector<Thing>& );
	VECTOR_STRUCTS
	VECTOR_PTRS
#undef OP_X

#define OP_X(Thing) \
	extern void dmp(const struct Thing* ); \
	extern void dmp(const struct Thing& ); \
	extern void dmpDetail(const struct Thing* ); \
	extern void dmpDetail(const struct Thing& );
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
OP_X(EdgesDetail) \
OP_X(Full) \
OP_X(FullDetail) \
OP_X(Hex) \
OP_X(ID) \
OP_X(Intersections) \
OP_X(IntersectionsDetail) \
OP_X(Link) \
OP_X(LinkDetail) \
OP_X(Winding)

#define DEBUG_DUMP_ID_DEFINITION(OWNER, ID) \
	std::string OWNER::debugDumpID() const { \
		return std::to_string(ID); \
	}

#define DUMP_POINT \
OP_X(Match) \
OP_X(MatchDetail)

#define DETAIL_POINTS \
OP_X(Match, Intersection) \
OP_X(MatchDetail, Intersection) \
OP_X(Match, Point) \
OP_X(MatchDetail, Point) \
OP_X(Match, PtT) \
OP_X(MatchDetail, PtT) 

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
OP_X(CenterHex) \
OP_X(Link) \
OP_X(LinkDetail)

#define OP_X(Thing) \
extern void dmp##Thing(int ID); \
extern void dmp##Thing(const OpEdge* ); \
extern void dmp##Thing(const OpEdge& );
EDGE_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X

#define SEGMENT_DETAIL \
OP_X(Full) \
OP_X(FullDetail) \
OP_X(SegmentEdges) \
OP_X(SegmentIntersections) \
OP_X(SegmentSects)

#define OP_X(Thing) \
extern void dmp##Thing(int ID); \
extern void dmp##Thing(const OpContour* ); \
extern void dmp##Thing(const OpContour& ); \
extern void dmp##Thing(const OpSegment* ); \
extern void dmp##Thing(const OpSegment& );
SEGMENT_DETAIL
EDGE_OR_SEGMENT_DETAIL
#undef OP_X

#define DUMP_HEX \
OP_X(OpContour) \
OP_X(OpCurve) \
OP_X(OpEdge) \
OP_X(OpIntersection) \
OP_X(OpPoint) \
OP_X(OpPointBounds) \
OP_X(OpPtT) \
OP_X(OpRect) \
OP_X(OpRoots) \
OP_X(OpSegment) \
OP_X(OpTightBounds) \
OP_X(OpVector) \
OP_X(SectRay)

#define OP_X(Thing) \
	extern void dmpHex(const struct Thing& ); \
	extern void dmpHex(const struct Thing* );
DUMP_HEX
#undef OP_X

#ifdef OP_HAS_FMA
#define DUMP_BY_DUMPID \
OP_X(dmp, dump) \
OP_X(dmpDetail, dumpDetail) \
OP_X(dmpHex, dumpHex)
#endif
#ifdef OP_NO_FMA
#define DUMP_BY_DUMPID \
OP_X(dmpid, dump) \
OP_X(dmpDetail, dumpDetail) \
OP_X(dmpHex, dumpHex)
#endif
#define OP_X(Global, Method) \
	extern void Global(int id);
DUMP_BY_DUMPID
#undef OP_X

#define DUMP_BY_ID \
OP_X(Detail) \
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
extern void dmpUnsectable();
extern void dmpUnsortable();

extern std::vector<const OpIntersection*> findCoincidence(int id);
extern const OpContour* findContour(int id);
extern const OpEdge* findEdge(int id);
extern std::vector<const OpEdge*> findEdgeOutput(int id);
extern std::vector<const OpEdge*> findEdgeRayMatch(int id);
extern std::vector<const OpEdge*> findEdgeUnsectable(int id);
extern const OpIntersection* findIntersection(int id);
extern std::vector<const OpIntersection*> findSectUnsectable(int id);
extern const OpSegment* findSegment(int id);

enum class DebugLevel {
	brief,
	normal,
	detailed
};

// !!! will likely macro-tize these as pattern emerges
extern std::string debugDump(const EdgeDistance& , DebugLevel );
extern std::string debugDumpHex(const EdgeDistance& , DebugLevel );
extern std::string debugDump(const SectRay& , DebugLevel );
extern std::string debugDumpHex(const SectRay& , DebugLevel );

extern std::string debugDumpColor(uint32_t c);
extern void dmpColor(uint32_t );
extern void dmpColor(const OpEdge* );
extern void dmpColor(const OpEdge& );
extern void dmpHex(float );

#endif

#endif
