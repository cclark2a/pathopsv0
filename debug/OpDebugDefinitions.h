// (c) 2023, Cary Clark cclark2@gmail.com
// Including this in a struct provides debug methods in that struct's context
// to dump data specific to that struct and outside that struct.

// Other structs can be referred to be pointer, reference or ID.

// Arrays (vectors) of structs can be dumped as well.

// All Macros below are defined in OpDebugDump.h

#define OP_X(Thing) \
	void dmp##Thing(const OWNER& ); \
\
	void dmpBrief(const OWNER& o) { \
		OpDebugFormat(o.debugDump(DebugLevel::brief, defaultBase)); } \
	void dmpDetailed(const OWNER& o) { \
		OpDebugFormat(o.debugDump(DebugLevel::detailed, defaultBase)); } \
	void dmpHex(const OWNER& o) { \
		OpDebugFormat(o.debugDump(defaultLevel, DebugBase::hex)); } \
\
	void OWNER::dump##Thing() const { \
		::dmp##Thing(*this); \
	}
	DUMP_BY_ID
#undef OP_X
