// (c) 2023, Cary Clark cclark2@gmail.com
// Including this in a struct provides debug methods in that struct's context
// to dump data specific to that struct and outside that struct.

// Other structs can be referred to be pointer, reference or ID.

// Arrays (vectors) of structs can be dumped as well.

// All Macros below are defined in OpDebugDump.h

#define OP_X(Thing) \
	void OWNER::dump(const std::vector<Thing>& things) const { \
		dmp(things); \
	} \
	void OWNER::dump(const std::vector<Thing>* things) const { \
		dmp(*things); \
	}
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump##Thing() const { \
		dmp##Thing(); \
	}
	DUMP_GROUP
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump##Thing(int ID) const { \
		dmp##Thing(ID); \
	}
	DUMP_BY_ID
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump##Thing(const OpPtT& p) const { \
		dmp##Thing(p.pt); \
	} \
	void OWNER::dump##Thing(const OpPtT* p) const { \
		dmp##Thing(p->pt); \
	} \
	void OWNER::dump##Thing(const OpPoint& p) const { \
		dmp##Thing(p); \
	} \
	void OWNER::dump##Thing(const OpPoint* p) const { \
		dmp##Thing(*p); \
	}
	DUMP_POINT
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump(const std::vector<Thing>& things) const { \
		dmp(things); \
	} \
	void OWNER::dump(const std::vector<Thing>* things) const { \
		dmp(*things); \
	}
	VECTOR_PTRS
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump(const Thing& thing) const { \
		dmp(thing); \
	} \
	void OWNER::dump(const Thing* thing) const { \
		dmp(*thing); \
	}
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump(const struct Thing& thing) const { \
		dmp(thing); \
	} \
	void OWNER::dump(const struct Thing* thing) const { \
		dmp(*thing); \
	}
	OP_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump##Thing(const struct OpEdge& thing) const { \
		dmp##Thing(thing); \
	} \
	void OWNER::dump##Thing(const struct OpEdge* thing) const { \
		dmp##Thing(*thing); \
	}
	EDGE_DETAIL
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump##Thing(const struct OpSegment& thing) const { \
		dmp##Thing(thing); \
	} \
	void OWNER::dump##Thing(const struct OpSegment* thing) const { \
		dmp##Thing(*thing); \
	}
	SEGMENT_DETAIL
#undef OP_X

