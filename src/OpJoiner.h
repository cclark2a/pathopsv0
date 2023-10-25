// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpJoiner_DEFINED
#define OpJoiner_DEFINED

#include "OpSegment.h"

struct OpContours;
struct OpOutPath;

enum class LinkPass {
	none,
	unambiguous,
	unsectInX
};

struct LinkUps {
	void sort();
#if OP_DEBUG_DUMP
	void dump() const;
	void dumpDetail() const;
#endif

	std::vector<OpEdge*> l;
};

struct OpJoiner {
	OpJoiner(OpContours& contours, OpOutPath& );
	bool activeUnsectable(const OpEdge* , EdgeMatch , std::vector<FoundEdge>& oppEdges);
	void addEdge(OpEdge* );
	void addToLinkups(OpEdge* );
	void buildDisabled(OpContours& );
	void buildDisabledPals(OpContours& );
    void checkDisabled();
    void checkGap();
    void checkLinkups();
    void checkNothingLeft();
    bool checkSectGap();
    void checkUnsectableGap();
    void checkUnsortableAndDisabled();
    void checkUnables();
    FoundEdge chooseSmallest();
    void detachChoppedEtc();
	bool detachIfLoop(OpEdge* );
    bool forceSmallEdge();
    bool lastLastResort();
	bool linkRemaining();
	void linkUnambiguous();
	bool linkUp(OpEdge* );
	void matchLeftover(OpPoint , const OpEdge* links, const std::vector<OpEdge*>& leftovers, 
			std::vector<FoundEdge>& );
	bool matchLinks(bool popLast);
    void matchPals();
	void sort();
#if OP_DEBUG
	int debugActive() const;
	void debugMatchRay(OP_DEBUG_CODE(OpContours* contours));
#endif
#if OP_DEBUG_VALIDATE
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	void debugDraw() const;
#endif

	OpOutPath& path;	// !!! move op joiner into op contours to eliminate reference?
	std::vector<OpEdge*> byArea;
	std::vector<OpEdge*> unsectByArea;
	std::vector<OpEdge*> disabled;
	std::vector<OpEdge*> disabledPals;
    std::vector<OpEdge*> unsortables;
	std::vector<FoundEdge> found;
    FoundEdge bestGap;
	LinkUps linkups;  // a structure to allow data specific debugging / dumping
	EdgeMatch linkMatch;
	LinkPass linkPass;
	const OpEdge* baseUnsectable;
	OpEdge* edge;
	OpEdge* lastEdge;
	OpPoint matchPt;
	bool disabledBuilt;
	bool disabledPalsBuilt;

#if OP_DEBUG
	std::vector<OpEdge*> debugTrack;
	std::vector<int> debugLinks;
#endif
};

#endif
