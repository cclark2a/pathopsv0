
#if OP_DEBUG_COMPARE
#include <string>

struct OpEdge;

struct OpDebugCompare {
	OpDebugCompare(const char* );
	void edges(const OpEdge* , const OpEdge*);
	void close();

	std::string str;
	char* buffer;
	const char* bufferPtr = nullptr;
	const char* filename;
};

#endif
