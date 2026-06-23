// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSegments_DEFINED
#define OpSegments_DEFINED

#include "OpMath.h"

#define DEFER_COIN_CHECK 1  // experiment: wait until points are merged to check for coincidence

struct OpContour;
struct OpContext;
struct OpSegment;
enum class FoundIntersections;
enum class IntersectResult;

struct DeferredCoinSect {
	OpIntersection* segStart;
	OpIntersection* segEnd;
};

struct DeferredCoinEnd {
	OpSegment* seg;
	OpSegment* opp;
};

struct OpSegments {
	OpSegments(OpContext& );
    void addEndMatches(OpContour* seg, OpContour* opp);
	std::vector<OpIntersection*> addEndMatches(OpSegment* seg, OpSegment* opp);
	FoundIntersections addLineCurveIntersection(OpSegment* opp, OpSegment* seg, 
			std::vector<OpIntersection*>& matchingSects);
	void checkCoins();  // coincident check after finding sects and merging their points
	void findCoincidence(OpContour* seg, OpContour* opp);
	bool findCoincidence(OpSegment* seg, OpSegment* opp);
	void findCoincidences();
	FoundIntersections findIntersections();
	void findIntersection(OpContour* seg, OpContour* opp);
	bool findIntersection(OpSegment* seg, OpSegment* opp);
	void initInX();
#if !DEFER_COIN_CHECK
	IntersectResult lineCoincidence(OpSegment* seg, OpSegment* opp);
#endif
	static void SwapEndMatches(std::vector<OpIntersection*>& );

	std::vector<DeferredCoinSect> deferredCoinSects;
	std::vector<DeferredCoinEnd> deferredCoinEnds;
	OpContext& context;
	FoundIntersections found;
	OP_DEBUG_CODE(int debugFailSegID);
	OP_DEBUG_CODE(int debugFailOppID);
};

#endif
