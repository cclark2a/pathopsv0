#ifndef OpJoiner_DEFINED
#define OpJoiner_DEFINED

#include "OpMath.h"

struct OpContours;
struct OpEdge;
struct OpOutPath;

enum class LinkPass {
	none,
	unambiguous,
	unsectInX
};

struct OpJoiner {
	OpJoiner(OpContours& contours, OpOutPath& );
	bool activeUnsectable(const OpEdge* edge, EdgeMatch match, std::vector<FoundEdge>& oppEdges);
	void addEdge(OpEdge* );
	void addToLinkups(OpEdge* edge);
	bool detachIfLoop(OpEdge* edge);
	bool linkRemaining();
	void linkUnambiguous();
	bool linkUp(OpEdge* edge);
	bool matchLinks(OpEdge* edge, bool popLast);
	void sort();

#if OP_DEBUG
	int debugActive() const;
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
	void dump() const;
	DUMP_COMMON_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
	void debugDraw() const;
#endif

	OpOutPath& path;	// !!! move op joiner into op contours to eliminate reference?
	std::vector<OpEdge*> inX;
	std::vector<OpEdge*> unsectInX;
    std::vector<OpEdge*>& unsortable;
	std::vector<OpEdge*> linkups;
	EdgeMatch linkMatch;
	LinkPass linkPass;
};

#endif
