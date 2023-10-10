// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebug.h"

#if OP_DEBUG_COMPARE
#include "OpContour.h"
#include "OpEdge.h"

OpDebugCompare::OpDebugCompare(const char* f) {
	filename = f;
	buffer = nullptr;
	FILE* file = fopen(filename, "rb");
	if (!file)
		return;
	int seek = fseek(file, 0, SEEK_END);
	assert(!seek);
	long bufferSize = ftell(file);
	fclose(file);
	file = fopen(filename, "rb");
	buffer = (char*)malloc(bufferSize);
	bufferPtr = buffer;
	fread(buffer, 1, bufferSize, file);
	fclose(file);
}

void OpDebugCompare::close() {
	FILE* file = fopen(filename, "w");
	size_t result = fwrite(str.c_str(), 1, str.length(), file);
	assert(result == str.length());
	fclose(file);
	fflush(file);
	free(buffer);
}

void OpDebugCompare::edges(const OpEdge* edge, const OpEdge* opp) {
	std::string nextStr = "e:" + STR(edge->id) + " o:" + STR(opp->id)
		+ " id:" + STR(edge->contour->contours->id);
	if (bufferPtr) {
		const char* limit = strchr(bufferPtr, '\n');
		int lf = 0;
		if ('\r' == limit[-1])
			++lf;
		std::string debugFileStr(bufferPtr, limit - bufferPtr - lf);
		assert(nextStr == debugFileStr);
		bufferPtr = limit + 1;
	}
	str += nextStr + "\n";
}

#endif
