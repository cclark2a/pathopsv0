#ifndef OpDebugDump_DEFINED
#define OpDebugDump_DEFINED

#if OP_DEBUG_DUMP

#include <vector>

struct OpContour;
struct OpEdge;
struct OpIntersection;
struct OpOutPath;
struct OpSegment;

extern void dump(const std::vector<OpEdge>& );  // to dump edge list built from intersections
extern void dump(const std::vector<OpEdge*>& ); // to dump assemble linkups
extern void dump(const std::vector<const OpEdge*>& ); // to dump debug image edges
extern void dump(const std::vector<OpSegment*>& ); // to dump segment intersection pairs
extern void dump(const OpOutPath& );

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
	std::string debugDumpSumDetail() const \

#define DEBUG_DUMP_ID_DEFINITION(_owner, _id) \
	std::string _owner::debugDumpID() const { \
		return std::to_string(_id); \
	}

#define DUMP_COMMON_DECLARATIONS() \
	void dump() const; \
	void dump(int id) const; \
    void dumpCoin(int id) const; \
    void dumpCoincidence(int id) const; \
	void dumpDetail(int id) const; \
	void dumpEdges() const; \
	void dumpEnd(int id) const; \
	void dumpFull(int id) const; \
	void dumpHex() const; \
	void dumpHex(int id) const; \
	void dumpIntersections() const; \
    void dumpLink(int id) const; \
    void dumpLinkDetail(int id) const; \
	void dumpMatch(OpPoint) const; \
	void dumpSects() const; \
	void dumpSegments() const; \
	void dumpStart(int id) const; \
    void dumpSum(int id) const; \
    void dumpSumDetail(int id) const; \

#define DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, _method) \
	void _owner::_method(int ID) const { \
		if (!_sizecheck)	\
			return;	\
		const OpSegment* s = _segment;	\
		s->contour->contours->_method(ID); \
	}

#define DUMP_STRUCT_DEFINITIONS(_owner, _sizecheck, _segment) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dump) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpCoin) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpCoincidence) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpDetail) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpEnd) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpFull) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpHex) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpLink) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpLinkDetail) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpStart) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpSum) \
	DUMP_STRUCT_DEFINITION(_owner, _sizecheck, _segment, dumpSumDetail) \

#define DUMP_GLOBAL_DEFINITION(global_function) \
	void global_function(int id) { \
		debugGlobalContours->global_function(id); \
	}

#define DUMP_GLOBAL_DEFINITIONS() \
	DUMP_GLOBAL_DEFINITION(dump) \
	DUMP_GLOBAL_DEFINITION(dumpCoin) \
	DUMP_GLOBAL_DEFINITION(dumpCoincidence) \
	DUMP_GLOBAL_DEFINITION(dumpDetail) \
	DUMP_GLOBAL_DEFINITION(dumpEnd) \
	DUMP_GLOBAL_DEFINITION(dumpFull) \
	DUMP_GLOBAL_DEFINITION(dumpHex) \
	DUMP_GLOBAL_DEFINITION(dumpLink) \
	DUMP_GLOBAL_DEFINITION(dumpLinkDetail) \
	DUMP_GLOBAL_DEFINITION(dumpStart) \
	DUMP_GLOBAL_DEFINITION(dumpSum) \
	DUMP_GLOBAL_DEFINITION(dumpSumDetail) \

#define DUMP_GLOBAL_DECLARATION(global_function) \
	extern void global_function(int id);

#define DUMP_GLOBAL_DECLARATIONS() \
	DUMP_GLOBAL_DECLARATION(dump) \
	DUMP_GLOBAL_DECLARATION(dumpCoin) \
	DUMP_GLOBAL_DECLARATION(dumpCoincidence) \
	DUMP_GLOBAL_DECLARATION(dumpDetail) \
	DUMP_GLOBAL_DECLARATION(dumpEnd) \
	DUMP_GLOBAL_DECLARATION(dumpFull) \
	DUMP_GLOBAL_DECLARATION(dumpHex) \
	DUMP_GLOBAL_DECLARATION(dumpLink) \
	DUMP_GLOBAL_DECLARATION(dumpLinkDetail) \
	DUMP_GLOBAL_DECLARATION(dumpStart) \
	DUMP_GLOBAL_DECLARATION(dumpSum) \
	DUMP_GLOBAL_DECLARATION(dumpSumDetail) \

DUMP_GLOBAL_DECLARATIONS()

#define DUMP_MULTIPLE_DEFINITION(_owner, _sizecheck, _segment, _method) \
	void _owner::_method() const { \
		if (!_sizecheck)	\
			return;	\
		const OpSegment* s = _segment;	\
		s->contour->contours->_method(); \
	}

#define DUMP_MULTIPLE_DEFINITIONS(_owner, _sizecheck, _segment) \
	DUMP_MULTIPLE_DEFINITION(_owner, _sizecheck, _segment, dumpEdges) \
	DUMP_MULTIPLE_DEFINITION(_owner, _sizecheck, _segment, dumpSegments) \
	DUMP_MULTIPLE_DEFINITION(_owner, _sizecheck, _segment, dumpIntersections)

#define DUMP_MATCH_DEFINITION(_owner, _sizecheck, _segment, _method) \
	void _owner::_method(OpPoint pt) const { \
		if (!_sizecheck)	\
			return;	\
		const OpSegment* s = _segment;	\
		s->contour->contours->_method(pt); \
	}

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
