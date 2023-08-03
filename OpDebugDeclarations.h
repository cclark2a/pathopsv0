// Including this in a struct provides debug methods in that struct's context
// to dump data specific to that struct and outside that struct.

// Other structs can be referred to be pointer, reference or ID.

// Arrays (vectors) of structs can be dumped as well.

// All Macros below are defined in OpDebugDump.h

std::string debugDump() const;
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
	void dump##Thing(const OpPoint* ) const; \
	void dump##Thing(const OpPtT* ) const; \
	void dump##Thing(const OpPoint& ) const; \
	void dump##Thing(const OpPtT& ) const;
	DUMP_POINT
#undef OP_X
#define OP_X(Thing) \
	void dump(const std::vector<Thing>& ) const; \
	void dumpDetail(const std::vector<Thing>& ) const;
	VECTOR_STRUCTS
	VECTOR_PTRS
#undef OP_X
#define OP_X(Thing) \
	void dump(const Thing& ) const; \
	void dump(const Thing* ) const; \
	void dumpDetail(const Thing& ) const; \
	void dumpDetail(const Thing* ) const;
	VECTOR_STRUCTS
	OP_STRUCTS
#undef OP_X
#define OP_X(Thing, Struct) \
	void dump##Thing(const Op##Struct* ) const; \
	void dump##Thing(const Op##Struct& ) const;
	DETAIL_STRUCTS
#undef OP_X
