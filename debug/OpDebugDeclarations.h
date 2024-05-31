// (c) 2023, Cary Clark cclark2@gmail.com
// Including this in a struct provides debug methods in that struct's context
// to dump data specific to that struct and outside that struct.

// Other structs can be referred to be pointer, reference or ID.

// Arrays (vectors) of structs can be dumped as well.

// All Macros below are defined in OpDebugDump.h

std::string debugDump(DebugLevel, DebugBase ) const;
void dump() const;
void dump(DebugLevel, DebugBase ) const;
void dumpResolveAll(OpContours* );
void dumpSet(const char*& );

#define OP_X(Thing) \
	void dump##Thing() const;
	DUMP_BY_ID
#undef OP_X
