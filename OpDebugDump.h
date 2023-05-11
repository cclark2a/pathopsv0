#ifndef OpDebugDump_DEFINED
#define OpDebugDump_DEFINED

#if OP_DEBUG_DUMP

#include <vector>

struct OpContour;
struct OpEdge;
struct OpIntersection;
struct OpOutPath;
struct OpPoint;
struct OpPtT;
struct OpSegment;

extern void dump(const std::vector<OpEdge>& );  // to dump edge list built from intersections
extern void dump(const std::vector<OpEdge*>& ); // to dump assemble linkups
extern void dump(const std::vector<const OpEdge*>& ); // to dump debug image edges
extern void dump(const std::vector<OpSegment*>& ); // to dump segment intersection pairs
extern void dump(const OpOutPath& );

extern void dump(const OpEdge*);
extern void dump(const OpEdge&);
extern void dump(const OpPtT*);
extern void dump(const OpPtT&);
extern void dump(const OpPoint*);
extern void dump(const OpPoint&);

#define DEBUG_COMMON_DECLARATIONS() \
    std::string debugDump() const; \
	std::string debugDumpDetail() const; \
	std::string debugDumpEdges() const; \
	std::string debugDumpFull() const; \
	std::string debugDumpHex() const; \
    std::string debugDumpID() const; \
	std::string debugDumpIntersections() const; \
	std::string debugDumpLink() const; \
	std::string debugDumpLinkDetail() const; \
	std::string debugDumpSum() const; \
	std::string debugDumpSumDetail() const; \
	std::string debugDumpWinding() const; \

#define DEBUG_DUMP_ID_DEFINITION(_owner, _id) \
	std::string _owner::debugDumpID() const { \
		return std::to_string(_id); \
	}

#define DUMP_COMMON_DECLARATIONS() \
	void dump(int id) const; \
    void dumpCoin(int id) const; \
    void dumpCoincidence(int id) const; \
	void dumpDetail(int id) const; \
	void dumpEdges() const; \
	void dumpEnd(int id) const; \
	void dumpFull(int id) const; \
	void dumpHex(int id) const; \
	void dumpIntersections() const; \
    void dumpLink(int id) const; \
    void dumpLinkDetail(int id) const; \
	void dumpMatch(const OpPoint& ) const; \
	void dumpSects() const; \
	void dumpSegmentEdges(int id) const; \
	void dumpSegmentIntersections(int id) const; \
	void dumpSegmentSects(int id) const; \
	void dumpSegments() const; \
	void dumpStart(int id) const; \
    void dumpSum() const; \
    void dumpSum(int id) const; \
    void dumpSumDetail() const; \
    void dumpSumDetail(int id) const; \
	void dumpWinding(int id) const; \

#define DUMP_IMPL_DECLARATIONS() \
	void dump() const; \
    void dumpCoin() const; \
    void dumpCoincidence() const; \
	void dumpDetail() const; \
	void dumpEnd() const; \
	void dumpFull() const; \
	void dumpHex() const; \
    void dumpLink() const; \
    void dumpLinkDetail() const; \
	void dumpSegmentEdges() const; \
	void dumpSegmentIntersections() const; \
	void dumpSegmentSects() const; \
	void dumpStart() const; \
	void dumpWinding() const; \

#define DUMP_STRUCT_DEFINITION(_owner, _method) \
	void _owner::_method() const { \
		debugGlobalContours->_method(); \
	}

#define DUMP_STRUCT_DEF_ID(_owner, _method) \
	void _owner::_method(int ID) const { \
		debugGlobalContours->_method(ID); \
	}

#define DUMP_STRUCT_DEF_MATCH(_owner, _method) \
	void _owner::_method(const OpPoint& pt) const { \
		debugGlobalContours->_method(pt); \
	}

#define DUMP_STRUCT_DEFINITIONS(_owner) \
	DUMP_STRUCT_DEF_ID(_owner, dump) \
	DUMP_STRUCT_DEF_ID(_owner, dumpCoin) \
	DUMP_STRUCT_DEF_ID(_owner, dumpCoincidence) \
	DUMP_STRUCT_DEF_ID(_owner, dumpDetail) \
	DUMP_STRUCT_DEFINITION(_owner, dumpEdges) \
	DUMP_STRUCT_DEF_ID(_owner, dumpEnd) \
	DUMP_STRUCT_DEF_ID(_owner, dumpFull) \
	DUMP_STRUCT_DEF_ID(_owner, dumpHex) \
	DUMP_STRUCT_DEFINITION(_owner, dumpIntersections) \
	DUMP_STRUCT_DEF_ID(_owner, dumpLink) \
	DUMP_STRUCT_DEF_ID(_owner, dumpLinkDetail) \
	DUMP_STRUCT_DEF_MATCH(_owner, dumpMatch) \
	DUMP_STRUCT_DEFINITION(_owner, dumpSects) \
	DUMP_STRUCT_DEF_ID(_owner, dumpSegmentEdges) \
	DUMP_STRUCT_DEF_ID(_owner, dumpSegmentIntersections) \
	DUMP_STRUCT_DEF_ID(_owner, dumpSegmentSects) \
	DUMP_STRUCT_DEFINITION(_owner, dumpSegments) \
	DUMP_STRUCT_DEF_ID(_owner, dumpStart) \
	DUMP_STRUCT_DEF_ID(_owner, dumpSum) \
	DUMP_STRUCT_DEF_ID(_owner, dumpSumDetail) \
	DUMP_STRUCT_DEF_ID(_owner, dumpWinding) \

#define DUMP_GLOBAL_DEFINITION(global_function) \
	void global_function() { \
		debugGlobalContours->global_function(); \
	}

#define DUMP_GLOBAL_DEF_ID(global_function) \
	void global_function(int id) { \
		debugGlobalContours->global_function(id); \
	}

#define DUMP_GLOBAL_DEF_MATCH(global_function) \
	void global_function(const OpPoint& pt) { \
		debugGlobalContours->global_function(pt); \
	}

#define DUMP_GLOBAL_DEFINITIONS() \
	DUMP_GLOBAL_DEF_ID(dump) \
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
	DUMP_GLOBAL_DEF_MATCH(dumpMatch) \
	DUMP_GLOBAL_DEFINITION(dumpSects) \
	DUMP_GLOBAL_DEF_ID(dumpSegmentEdges) \
	DUMP_GLOBAL_DEF_ID(dumpSegmentIntersections) \
	DUMP_GLOBAL_DEF_ID(dumpSegmentSects) \
	DUMP_GLOBAL_DEFINITION(dumpSegments) \
	DUMP_GLOBAL_DEF_ID(dumpStart) \
	DUMP_GLOBAL_DEF_ID(dumpSum) \
	DUMP_GLOBAL_DEF_ID(dumpSumDetail) \
	DUMP_GLOBAL_DEF_ID(dumpWinding) \

#define DUMP_GLOBAL_DECLARATION(global_function) \
	extern void global_function();

#define DUMP_GLOBAL_DECL_ID(global_function) \
	extern void global_function(int id);

#define DUMP_GLOBAL_DECL_MATCH(global_function) \
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
	DUMP_GLOBAL_DECL_MATCH(dumpMatch) \
	DUMP_GLOBAL_DECLARATION(dumpSects) \
	DUMP_GLOBAL_DECL_ID(dumpSegmentEdges) \
	DUMP_GLOBAL_DECL_ID(dumpSegmentIntersections) \
	DUMP_GLOBAL_DECL_ID(dumpSegmentSects) \
	DUMP_GLOBAL_DECLARATION(dumpSegments) \
	DUMP_GLOBAL_DECL_ID(dumpStart) \
	DUMP_GLOBAL_DECL_ID(dumpSum) \
	DUMP_GLOBAL_DECL_ID(dumpSumDetail) \
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
