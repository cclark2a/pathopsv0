#ifndef OpDebugDump_DEFINED
#define OpDebugDump_DEFINED

#if OP_DEBUG_DUMP

#include <vector>

// removed OP_X(CoinPair) for now
// removed OP_X(ExtremaT) for now
// removed OP_X(LoopCheck) for now
// removed OP_X(MissingIntersection) for now
#define VECTOR_STRUCTS \
OP_X(EdgeDistance) \
OP_X(FoundEdge) \
OP_X(OpContour) \
OP_X(OpEdge) \
OP_X(OpIntersection) \
OP_X(OpSegment)

#define OP_STRUCTS \
OP_X(LinkUps) \
OP_X(OpContours) \
OP_X(OpCurveCurve) \
OP_X(OpJoiner) \
OP_X(OpOutPath) \
OP_X(OpPtT) \
OP_X(OpPoint) \
OP_X(OpPointBounds) \
OP_X(OpRect) \
OP_X(OpSegments) \
OP_X(OpTightBounds) \
OP_X(OpWinder)

#define OP_X(Thing) \
	struct Thing;
	VECTOR_STRUCTS
	OP_STRUCTS
#undef OP_X

#define VECTOR_PTRS \
OP_X(OpEdge*) \
OP_X(const OpEdge*) \
OP_X(OpIntersection*) \
OP_X(OpSegment*)

#define OP_X(Thing) \
	extern void dump(const std::vector<Thing>* ); \
	extern void dumpDetail(const std::vector<Thing>* ); \
	extern void dump(const std::vector<Thing>& ); \
	extern void dumpDetail(const std::vector<Thing>& );
	VECTOR_STRUCTS
	VECTOR_PTRS
#undef OP_X

#define OP_X(Thing) \
	extern void dump(const Thing* ); \
	extern void dump(const Thing& );
	OP_STRUCTS
#undef OP_X

#define DETAIL_STRUCTS \
OP_X(Link, Edge) \
OP_X(Full, Segment)

#define OP_X(Thing, Struct) \
extern void dump##Thing(const Op##Struct* ); \
extern void dump##Thing(const Op##Struct& );
DETAIL_STRUCTS
#undef OP_X

extern void dumpMatch(const OpPoint* );
extern void dumpMatch(const OpPtT* );
extern void dumpMatch(const OpPoint& );
extern void dumpMatch(const OpPtT& );
extern void dumpMatchDetail(const OpPoint* );
extern void dumpMatchDetail(const OpPtT* );
extern void dumpMatchDetail(const OpPoint& );
extern void dumpMatchDetail(const OpPtT& );

#define DUMP_GROUP \
OP_X(Active) \
OP_X(Contours) \
OP_X(Edges) \
OP_X(Intersections) \
OP_X(Sects) \
OP_X(Segments)

#define OP_X(Thing) \
	extern void dump##Thing();
	DUMP_GROUP
#undef OP_X

#define DEBUG_DUMP \
OP_X(Detail) \
OP_X(Edges) \
OP_X(Full) \
OP_X(Hex) \
OP_X(ID) \
OP_X(Intersections) \
OP_X(Link) \
OP_X(LinkDetail) \
OP_X(Winding)

#define DEBUG_DUMP_ID_DEFINITION(OWNER, ID) \
	std::string OWNER::debugDumpID() const { \
		return std::to_string(ID); \
	}

#define DUMP_BY_ID \
OP_X(Coin) \
OP_X(Coincidence) \
OP_X(Detail) \
OP_X(End) \
OP_X(Full) \
OP_X(Hex) \
OP_X(Link) \
OP_X(LinkDetail) \
OP_X(SegmentEdges) \
OP_X(SegmentIntersections) \
OP_X(SegmentSects) \
OP_X(Start) \
OP_X(Winding)

#define DUMP_POINT \
OP_X(Match) \
OP_X(MatchDetail)

#if 0  // replacement for DUMP_STRUCT_DEFINITIONS(OWNER)
		// ... don't know how to do this ...
#define OP_X(Thing) \
	DUMP_STRUCT_DEF_ID(OWNER, Thing)
	DUMP_BY_ID
#undef OP_X
#endif

#define DUMP_STRUCT_DEFINITION(OWNER, METHOD) \
	void OWNER::METHOD() const { \
		debugGlobalContours->METHOD(); \
	}

#define DUMP_STRUCT_DEF_ID(OWNER, METHOD) \
	void OWNER::METHOD(int ID) const { \
		debugGlobalContours->METHOD(ID); \
	}

#define DUMP_STRUCT_DEF_POINT(OWNER, METHOD) \
	void OWNER::METHOD(const OpPoint& pt) const { \
		debugGlobalContours->METHOD(pt); \
	}

#define DUMP_STRUCT_DEFINITIONS(OWNER) \
	DUMP_STRUCT_DEF_ID(OWNER, dump) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpCoin) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpCoincidence) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpDetail) \
	DUMP_STRUCT_DEFINITION(OWNER, dumpEdges) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpEnd) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpFull) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpHex) \
	DUMP_STRUCT_DEFINITION(OWNER, dumpIntersections) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpLink) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpLinkDetail) \
	DUMP_STRUCT_DEF_POINT(OWNER, dumpMatch) \
	DUMP_STRUCT_DEF_POINT(OWNER, dumpMatchDetail) \
	DUMP_STRUCT_DEFINITION(OWNER, dumpSects) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpSegmentEdges) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpSegmentIntersections) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpSegmentSects) \
	DUMP_STRUCT_DEFINITION(OWNER, dumpSegments) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpStart) \
	DUMP_STRUCT_DEF_ID(OWNER, dumpWinding) \

#define DUMP_GLOBAL_DEFINITION(global_function) \
	void global_function() { \
		debugGlobalContours->global_function(); \
	}

#define DUMP_GLOBAL_DEF_ID(global_function) \
	void global_function(int id) { \
		debugGlobalContours->global_function(id); \
	}

#define DUMP_GLOBAL_DEF_POINT(global_function) \
	void global_function(const OpPoint& pt) { \
		debugGlobalContours->global_function(pt); \
	}

#define DUMP_GLOBAL_DEFINITIONS() \
	DUMP_GLOBAL_DEF_ID(dump) \
	DUMP_GLOBAL_DEFINITION(dumpActive) \
	DUMP_GLOBAL_DEF_ID(dumpCoin) \
	DUMP_GLOBAL_DEF_ID(dumpCoincidence) \
	DUMP_GLOBAL_DEF_ID(dumpDetail) \
	DUMP_GLOBAL_DEFINITION(dumpEdges) \
	DUMP_GLOBAL_DEF_ID(dumpEnd) \
	DUMP_GLOBAL_DEF_ID(dumpFull) \
	DUMP_GLOBAL_DEF_ID(dumpHex) \
	DUMP_GLOBAL_DEFINITION(dumpIntersections) \
	DUMP_GLOBAL_DEF_ID(dumpLink) \
	DUMP_GLOBAL_DEF_ID(dumpLinkDetail) \
	DUMP_GLOBAL_DEF_POINT(dumpMatch) \
	DUMP_GLOBAL_DEF_POINT(dumpMatchDetail) \
	DUMP_GLOBAL_DEFINITION(dumpSects) \
	DUMP_GLOBAL_DEF_ID(dumpSegmentEdges) \
	DUMP_GLOBAL_DEF_ID(dumpSegmentIntersections) \
	DUMP_GLOBAL_DEF_ID(dumpSegmentSects) \
	DUMP_GLOBAL_DEFINITION(dumpSegments) \
	DUMP_GLOBAL_DEF_ID(dumpStart) \
	DUMP_GLOBAL_DEF_ID(dumpWinding) \

#define DUMP_GLOBAL_DECLARATION(global_function) \
	extern void global_function();

#define DUMP_GLOBAL_DECL_ID(global_function) \
	extern void global_function(int id);

#define DUMP_GLOBAL_DECL_POINT(global_function) \
	extern void global_function(const OpPoint& pt);

#define DUMP_GLOBAL_DECLARATIONS() \
	DUMP_GLOBAL_DECL_ID(dump) \
	DUMP_GLOBAL_DECL_ID(dumpCoin) \
	DUMP_GLOBAL_DECL_ID(dumpCoincidence) \
	DUMP_GLOBAL_DECL_ID(dumpDetail) \
	DUMP_GLOBAL_DECLARATION(dumpEdges) \
	DUMP_GLOBAL_DECL_ID(dumpEnd) \
	DUMP_GLOBAL_DECL_ID(dumpFull) \
	DUMP_GLOBAL_DECL_ID(dumpHex) \
	DUMP_GLOBAL_DECLARATION(dumpIntersections) \
	DUMP_GLOBAL_DECL_ID(dumpLink) \
	DUMP_GLOBAL_DECL_ID(dumpLinkDetail) \
	DUMP_GLOBAL_DECL_POINT(dumpMatch) \
	DUMP_GLOBAL_DECL_POINT(dumpMatchDetail) \
	DUMP_GLOBAL_DECLARATION(dumpSects) \
	DUMP_GLOBAL_DECL_ID(dumpSegmentEdges) \
	DUMP_GLOBAL_DECL_ID(dumpSegmentIntersections) \
	DUMP_GLOBAL_DECL_ID(dumpSegmentSects) \
	DUMP_GLOBAL_DECLARATION(dumpSegments) \
	DUMP_GLOBAL_DECL_ID(dumpStart) \
	DUMP_GLOBAL_DECL_ID(dumpWinding) \

DUMP_GLOBAL_DECLARATIONS()

#define FIND_COMMON_DECLARATIONS(pre_const, post) \
	pre_const OpIntersection* findCoin(int id) post \
	pre_const OpContour* findContour(int id) post \
	pre_const OpIntersection* findCoincidence(int id) post \
	pre_const OpEdge* findEdge(int id) post \
	pre_const OpIntersection* findIntersection(int id) post \
	pre_const OpSegment* findSegment(int id) post

FIND_COMMON_DECLARATIONS(extern const, ;)

#endif

#endif
