// Including this in a struct provides debug methods in that struct's context
// to dump data specific to that struct and outside that struct.

// Other structs can be referred to be pointer, reference or ID.

// Arrays (vectors) of structs can be dumped as well.

// All Macros below are defined in OpDebugDump.h

void OWNER::dump() const {
    ::dump(*this);
}

void OWNER::dump(int ID) const {
	::dump(ID);
}

#define OP_X(Thing) \
	void OWNER::dump(const std::vector<Thing>& things) const { \
		::dump(things); \
	} \
	void OWNER::dumpDetail(const std::vector<Thing>& things) const { \
		::dumpDetail(things); \
	}
	VECTOR_STRUCTS
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump##Thing(int ID) const { \
		::dump##Thing(ID); \
	}
	DUMP_BY_ID
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump##Thing(const OpPoint& p) const { \
		::dump##Thing(p); \
	}
	DUMP_POINT
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump(const std::vector<Thing>& things) const { \
		::dump(things); \
	} \
	void OWNER::dumpDetail(const std::vector<Thing>& things) const { \
		::dumpDetail(things); \
	}
	VECTOR_PTRS
#undef OP_X
#define OP_X(Thing) \
	void OWNER::dump(const Thing& thing) const { \
		thing.dump(); \
	} \
	void OWNER::dump(const Thing* thing) const { \
		thing->dump(); \
	} \
	void OWNER::dumpDetail(const Thing& thing) const { \
		thing.dumpDetail(); \
	} \
	void OWNER::dumpDetail(const Thing* thing) const { \
		thing->dumpDetail(); \
	}
	VECTOR_STRUCTS
	OP_STRUCTS
#undef OP_X
#define OP_X(Thing, Struct) \
	void OWNER::dump##Thing(const Op##Struct* thing) const { \
		::dump##Thing(thing); \
	} \
	void OWNER::dump##Thing(const Op##Struct& thing) const { \
		::dump##Thing(thing); \
	}
	DETAIL_STRUCTS
#undef OP_X
