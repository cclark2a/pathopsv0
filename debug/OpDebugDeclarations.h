// (c) 2023, Cary Clark cclark2@gmail.com
// Including this in a struct provides debug methods in that struct's context
// to dump data specific to that struct and outside that struct.

// Other structs can be referred to be pointer, reference or ID.

// Arrays (vectors) of structs can be dumped as well.

// All Macros below are defined in OpDebugDump.h

std::string debugDump() const;
std::string debugDump(DebugLevel, DebugBase ) const;
void dump() const;
void dump(int ID) const;

#define OP_X(Thing) \
	std::string debugDump##Thing() const;
	DEBUG_DUMP
#undef OP_X
#define OP_X(Thing) \
	void dump##Thing(int ID) const;
	DUMP_BY_ID
#undef OP_X
#define OP_X(Thing) \
	void dump##Thing() const;
	DUMP_GROUP
	DUMP_BY_ID
#undef OP_X
#define OP_X(Thing) \
	void dump##Thing(const struct OpPoint& ) const; \
	void dump##Thing(const struct OpPoint* ) const; \
	void dump##Thing(const struct OpPtT& ) const; \
	void dump##Thing(const struct OpPtT* ) const;
	DUMP_POINT
#undef OP_X
#define OP_X(Thing) \
	void dump(const std::vector<Thing>& ) const; \
	void dump(const std::vector<Thing>* ) const; \
	void dumpDetail(const std::vector<Thing>& ) const; \
	void dumpDetail(const std::vector<Thing>* ) const;
	VECTOR_STRUCTS
	VECTOR_PTRS
#undef OP_X
#define OP_X(Thing) \
	void dump(const Thing& ) const; \
	void dump(const Thing* ) const; \
	void dumpDetail(const Thing& ) const; \
	void dumpDetail(const Thing* ) const;
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void dump(const struct Thing& ) const; \
	void dump(const struct Thing* ) const; \
	void dumpDetail(const struct Thing& ) const; \
	void dumpDetail(const struct Thing* ) const;
	OP_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void dump##Thing(const struct OpEdge& ) const; \
	void dump##Thing(const struct OpEdge* ) const;
	EDGE_DETAIL
#undef OP_X
#define OP_X(Thing) \
	void dump##Thing(const struct OpSegment& ) const; \
	void dump##Thing(const struct OpSegment* ) const;
	SEGMENT_DETAIL
#undef OP_X
