// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include <utility>

void CcCurves::clear() {
	for (auto edge : c) {
		edge->hulls.clear();
		edge->ccOverlaps = false;
	}
}

// note that this doesn't create an opportunity for a zero distance; it uses edge run as temporary
bool CcCurves::checkMid(size_t index) {
	EdgeRun& eS = runs[index];
	EdgeRun& eE = runs[index + 1];
	EdgeRun mid;
	float midT = OpMath::Average(eS.edgePtT.t, eE.edgePtT.t);
	const OpSegment* seg = eS.edge->segment;
	mid.edgePtT = seg->c.ptTAtT(midT);
	mid.oppPtT = Dist(seg, mid.edgePtT, eS.oppEdge->segment);
	mid.oppDist = mid.setOppDist(seg);
	return OpMath::InUnsorted(eS.oppDist, mid.oppDist, eE.oppDist, seg->threshold().length());
}

#if 0
OpPtT CcCurves::closest(OpPoint pt) const {
	OpPoint oBest;
	float oBestDistSq = OpInfinity;
	auto bestPt = [&oBest, &oBestDistSq, pt](OpPoint testPt) {
		float distSq = (testPt - pt).lengthSquared();
		if (oBestDistSq > distSq) {
			oBestDistSq = distSq;
			oBest = testPt;
		}
	};
	// scan opposite curves for closest point
	OpPoint oLast;
	for (OpEdge* oppEdge : c) {
		OpPoint startPt = oppEdge->startPt();
		if (oLast != startPt)
			bestPt(startPt);
		OpPoint endPt = oppEdge->endPt();
		bestPt(endPt);
		oLast = endPt;
	}
	return { oBest, oBestDistSq };
};
#endif

// !!! this was lineIntersect which could miss if normal line points away from seg
//     but it was changed without fixing the root bug, so may make things less stable ...
OpPtT CcCurves::Dist(const OpSegment* seg, const OpPtT& segPtT, const OpSegment* opp) {
	OpVector normal = seg->c.normal(segPtT.t);
	if (!normal.isFinite())
		return OpPtT(SetToNaN::dummy);
	LinePts normLine { segPtT.pt - normal, segPtT.pt + normal };
	OpRoots roots = opp->c.rayIntersect(normLine, MatchEnds::none);
	float bestSq = OpInfinity;
	OpPtT bestPtT(SetToNaN::dummy);
	for (size_t index = 0; index < roots.count; ++index) {
		OpPtT oppPtT = opp->c.ptTAtT(roots.roots[index]);
		float distSq = (segPtT.pt - oppPtT.pt).lengthSquared();
		if (bestSq > distSq) {
			bestSq = distSq;
			bestPtT = oppPtT;
		}
	}
	return bestPtT;
}

float EdgeRun::setOppDist(const OpSegment* segment) {
	if (OpMath::IsNaN(edgePtT.t))
		return OpNaN;
	OpVector oppV = edgePtT.pt - oppPtT.pt;
	float dist = oppV.length();
	if (OpMath::IsNaN(dist))
		return OpNaN;
	OpVector normal = segment->c.normal(edgePtT.t);
	float nDotOpp = normal.dot(oppV);
	if (nDotOpp < -OpEpsilon)
		dist = -dist;
	else if (nDotOpp <= OpEpsilon)
		dist = 0;
	return dist;
}

void EdgeRun::set(OpEdge* e, const OpSegment* oppSeg, EdgeMatch match) {
	edge = e;
	oppEdge = const_cast<OpEdge*>(&oppSeg->edges[0]);  // !!! don't know about this...
	edgePtT = EdgeMatch::start == match ? edge->start() : edge->end();
	oppPtT = CcCurves::Dist(edge->segment, edgePtT, oppSeg);
	if (OpMath::IsFinite(oppPtT.t))
		oppDist = setOppDist(edge->segment);
	else
		oppDist = OpNaN;
	fromFoundT = false;
	byZero = false;
	OP_DEBUG_CODE(debugBetween = 1);
}

bool EdgeRun::inDeleted(CcCurves* curves, CcCurves* oppCurves) const {
	return curves->deletedT(edgePtT.t) || oppCurves->deletedT(oppPtT.t);
}

void CcCurves::addEdgeRun(OpEdge* edge, const OpSegment* oppSeg, EdgeMatch match) {
	EdgeRun run;
	run.set(edge, oppSeg, match);
	if (OpMath::IsNaN(run.oppDist))
		return;
	EdgeRun* runStart = runs.size() ? &runs.front() : nullptr;
	// !!! could binary search
	size_t index = 1;
	for (; index < runs.size(); ++index) {
		EdgeRun* runEnd = &runs[index];
		if (runStart->edgePtT.t <= run.edgePtT.t && run.edgePtT.t < runEnd->edgePtT.t
				&& OpMath::Between(runStart->oppDist, run.oppDist, runEnd->oppDist)) {
			OP_DEBUG_CODE(++runStart->debugBetween);
			if (runStart->oppDist * runEnd->oppDist > 0)
				return;
		}
		if (run.edgePtT.t < runStart->edgePtT.t)
			break;
		runStart = runEnd;
	}
	// we care if run encompasses run-1, run-2 or run+1, run+2
		// in these cases run replaces run-1 or run+1
	// we care if sgn(run.oppDist) != sgn((run-1).oppDist) or sgn((run+1).oppDist)
	// we care if run.oppDist == 0 (!!! or is nearly 0?)
	--index;
	if (runs.size() > index + 1) {
		if (OpMath::Between(run.oppDist, runs[index].oppDist, runs[index + 1].oppDist)
				&& run.oppDist * runs[index].oppDist > 0
				&& runs[index].oppDist * runs[index + 1].oppDist > 0) {
			OP_DEBUG_CODE(run.debugBetween += runs[index].debugBetween);
			runs[index] = run;
			return;
		}
	}
	if (index > 1) {
		if (OpMath::Between(runs[index - 2].oppDist, runs[index - 1].oppDist, run.oppDist)
				&& run.oppDist * runs[index - 1].oppDist > 0
				&& runs[index - 2].oppDist * runs[index - 1].oppDist > 0) {
			OP_DEBUG_CODE(runs[index - 2].debugBetween += 1);
			OP_DEBUG_CODE(run.debugBetween = runs[index - 1].debugBetween);
			runs[index - 1] = run;
			return;
		}
	}
	if (1 == runs.size() && run.edgePtT.t > runStart->edgePtT.t)
		++index;
	runs.insert(runs.begin() + index, run);
}

bool CcCurves::deletedT(float t) const {
	for (const CutRangeT& test : deleted) {
		if (test.lo.t <= t && t <= test.hi.t)
			return true;
	}
	return false;
}

std::vector<CutRangeT> CcCurves::findGaps() const {
	std::vector<CutRangeT> gaps;
	size_t index = 0;
	while (index < c.size() && !c[index]->ccOverlaps)
		++index;
	if (index >= c.size())
		return gaps;
	OpPtT last = c[index]->end();
	while (++index < c.size()) {
		const OpEdge* edgePtr = c[index];
		OP_ASSERT(edgePtr->startT >= last.t);  // if not sorted, fix
		if (!edgePtr->ccOverlaps)
			continue;
		if (edgePtr->startT != last.t)
			gaps.push_back({ last, edgePtr->start() });
		last = edgePtr->end();
	}
	return gaps;
}

#if 0
int CcCurves::groupCount() const {
	if (!c.size())
		return 0;
	const OpEdge* last = c[0];
	int result = 1;
	for (size_t index = 1; index < c.size(); ++index) {
		const OpEdge* edge = c[index];
		OP_ASSERT(edge->startT >= last->endT);  // if not sorted, fix
		if (!edge->ccOverlaps)
			continue;					 // !!! below condition not quite right...
		if (edge->startT != last->endT /* && (!edge->ccStart || !last->ccEnd) */ )
			++result;
		last = edge;
	}
	return result;
}
#endif

void CcCurves::initialEdgeRun(OpEdge* edge, const OpSegment* oppSeg) {
	for (EdgeMatch match : { EdgeMatch::start, EdgeMatch::end } ) {
		EdgeRun run;
		run.set(edge, oppSeg, match);
		if (OpMath::IsNaN(run.oppDist))
			continue;
		runs.push_back(run);
	}
}

void CcCurves::markToDelete(float loT, float hiT) {
	for (auto edgePtr : c) {
		if (edgePtr->startT >= loT && hiT >= edgePtr->endT)
			edgePtr->ccOverlaps = false;
	}
}

int CcCurves::overlaps() const {
	int count = 0;
	for (auto curve : c)
		count += curve->ccOverlaps;
	return count;
}

#if 0
float CcCurves::perimeter() const {
	if (!c.size())
		return 0;
	OpPointBounds r = c[0]->ptBounds;
	for (size_t index = 1; index < c.size(); ++index)
		r.add(c[index]->ptBounds);
	return r.perimeter();
}
#endif

// snip out the curve 16 units to the side of the intersection point, to prevent another closeby
// intersection from also getting recorded. The units maybe t values, or may be x/y distances.
void CcCurves::snipAndGo(const OpSegment* segment, const OpPtT& ptT, OpPoint oppPt, 
		const OpSegment* oppSeg) {
	// snip distance must be large enough to differ in x/y and in t
	CutRangeT tRange = segment->c.cutRange(ptT, oppPt, 0, 1);
	OP_ASSERT(tRange.lo.t < tRange.hi.t);
	// remove part or all of edges that overlap tRange
	deleted.push_back(tRange);
	snipRange(segment, tRange.lo, tRange.hi, oppSeg);
}

void CcCurves::snipRange(const OpSegment* segment, const OpPtT& lo, const OpPtT& hi, 
		const OpSegment* oppSeg) {
	CcCurves snips;
	OpContours* contours = segment->contour->contours;
	auto addSnip = [contours](const OpEdge* edge, const OpPtT& start, const OpPtT& end) {
		void* block = contours->allocateEdge(contours->ccStorage);
		OpEdge* newE = new(block) OpEdge(edge, start, end  OP_LINE_FILE_PARGS());
		newE->ccOverlaps = true;
		return newE;
	};
	for (OpEdge* edge : c) {
		if (edge->startT >= hi.t || edge->endT <= lo.t) {
			OP_ASSERT(!edge->disabled);
			snips.c.push_back(edge);
			continue;
		}
		OpVector threshold = contours->threshold();
		if (edge->startT < lo.t && !edge->start().isNearly(lo, threshold)) {
			OpEdge* snipE = addSnip(edge, edge->start(), lo);
			snipE->ccStart = edge->ccStart;
			snipE->ccSmall = edge->ccSmall;
			snipE->ccEnd = true;
			addEdgeRun(snipE, oppSeg, EdgeMatch::end);
			OP_ASSERT(!snipE->disabled);
			snips.c.push_back(snipE);
		}
		if (edge->endT > hi.t && !hi.isNearly(edge->end(), threshold)) {
			OpEdge* snipS = addSnip(edge, hi, edge->end());
			snipS->ccStart = true;
			snipS->ccEnd = edge->ccEnd;
			snipS->ccLarge = edge->ccLarge;
			addEdgeRun(snipS, oppSeg, EdgeMatch::start);
			OP_ASSERT(!snipS->disabled);
			snips.c.push_back(snipS);
		}
	}
	snips.c.swap(c);
}

OpCurveCurve::OpCurveCurve(OpSegment* s, OpSegment* o)
	: contours(s->contour->contours)
	, seg(s)
	, opp(o)
	, depth(0)
	, uniqueLimits_impl(-1)
	, unsplitables(0)
	, addedPoint(false)
	, rotateFailed(false)
	, sectResult(false)
	, lastDepthReduced(false)
	, foundGap(false)
	, splitHullFail(false)
{
#if OP_DEBUG_DUMP
	++debugCall;
	debugLocalCall = debugCall;  // copied so value is visible in debugger
	contours->debugCurveCurve = this;
#endif
	contours->reuse(contours->ccStorage);
	maxSignSwap = contours->contextCallBacks.maxSignSwapFuncPtr(s->c.c, o->c.c);
	maxSplits = contours->contextCallBacks.maxSplitsFuncPtr(s->c.c, o->c.c);
	maxDepth = contours->contextCallBacks.maxDepthFuncPtr(s->c.c, o->c.c);

	matchRev = seg->matchEnds(opp);
	smallTFound = MatchEnds::start & matchRev.match;
	largeTFound = MatchEnds::end & matchRev.match;
	splitMid = smallTFound || largeTFound;
	seg->edges.clear();
	opp->edges.clear();
	OpPtT segS {seg->c.firstPt(), 0 };
	OpPtT segE {seg->c.lastPt(), 1 };
	OpPtT oppS {opp->c.firstPt(), 0 };
	OpPtT oppE {opp->c.lastPt(), 1 };
	if (matchRev.reversed)
		std::swap(oppS, oppE);
	if (smallTFound) {
		FoundLimits smT { nullptr, nullptr, segS, oppS, true, false   // no edges
				OP_LINE_FILE_STRUCT() }; 
		limits.push_back(std::move(smT));
	}
	if (largeTFound) {
		FoundLimits lgT { nullptr, nullptr, segE, oppE, true, false  // no edges
				OP_LINE_FILE_STRUCT() };
		limits.push_back(std::move(lgT));
	}
	// if ends of segments already touch, exclude from made edge
	seg->makeEdge(OP_LINE_FILE_NPARGS());
	if (matchRev.reversed)
		std::swap(oppS, oppE);
	opp->makeEdge(OP_LINE_FILE_NPARGS());
	OpEdge* edge = &seg->edges.back();
	edge->ccStart = edge->ccSmall = smallTFound;
	edge->ccEnd = edge->ccLarge = largeTFound;
	OP_ASSERT(!edge->disabled);
	edgeCurves.oppCurves = &oppCurves;
	oppCurves.oppCurves = &edgeCurves;
	edgeCurves.c.push_back(edge);
	edgeCurves.initialEdgeRun(edge, opp);
	OpEdge* oppEdge = &opp->edges.back();
	OP_ASSERT(!oppEdge->disabled);
	oppCurves.c.push_back(oppEdge);
	oppCurves.initialEdgeRun(oppEdge, seg);
	if (matchRev.reversed)
		std::swap(smallTFound, largeTFound);
	oppEdge->ccStart = oppEdge->ccSmall = smallTFound;
	oppEdge->ccEnd = oppEdge->ccLarge = largeTFound;
	if (smallTFound) {
		limits.front().parentEdge = edge;
		limits.front().parentOpp = oppEdge;
	}
	if (largeTFound) {
		limits.back().parentEdge = edge;
		limits.back().parentOpp = oppEdge;
	}
}

void OpCurveCurve::addEdgeRun(OpEdge* edge, CurveRef curveRef, EdgeMatch match) {
	CcCurves& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	curves.addEdgeRun(edge, CurveRef::edge == curveRef ? opp : seg, match);
}

void OpCurveCurve::addIntersection(OpEdge* edge, OpEdge* oppEdge) {
	recordSect(edge, oppEdge, edge->start(), oppEdge->start()  
			OP_LINE_FILE_PARGS());
	snipEdge = edge->start();
	snipOpp = oppEdge->start();
}

enum class IsOpp {
	no,
	yes
};

enum class IsCoin {
	no,
	yes
};

struct IdEnds {
	int id;
	MatchEnds matchEnds;
};

struct SectDuo {
	OpIntersection* s;
	OpIntersection* o;
	bool alreadySet;
};

bool OpCurveCurve::addUnsectable(const OpPtT& edgeStart, const OpPtT& edgeEnd,
		const OpPtT& oppStart, const OpPtT& oppEnd) {
	OpVector threshold = contours->threshold();
	OpPtT eStart = edgeStart;
	OpPtT eEnd = edgeEnd;
	OpPtT oStart = oppStart;
	OpPtT oEnd = oppEnd;
	if (eStart.isNearly(oStart, threshold))
		OpPtT::MeetInTheMiddle(eStart, oStart);
	if (eEnd.isNearly(oEnd, threshold))
		OpPtT::MeetInTheMiddle(eEnd, oEnd);
	if (eStart.isNearly(eEnd, threshold))
		return false;
	if (oStart.isNearly(oEnd, threshold))
		return false;
	MatchReverse match { MatchEnds::start, oStart.t > oEnd.t };
	IsCoin isCoin = eStart.pt == (match.reversed ? oEnd.pt : oStart.pt) 
			&& eEnd.pt == (match.reversed ? oStart.pt : oEnd.pt) ? IsCoin::yes : IsCoin::no;
	auto setSectPair = [isCoin, this](const OpPtT& ePt, const OpPtT& oPt) {
		SectDuo result;
		result.s = seg->sects.contains(ePt, opp);
		result.o = opp->sects.contains(oPt, seg);
		if (!result.s != !result.o) {
			if (result.o)
				result.s = result.o->opp;
			else
				result.o = result.s->opp;
		}
		result.alreadySet = result.s 
				&& (IsCoin::yes == isCoin ? result.s->coincidenceID : result.s->unsectID);
		return result;
	};
	SectDuo sect1 = setSectPair(eStart, oStart);
	if (sect1.alreadySet)
		return false;
	SectDuo sect2 = setSectPair(eEnd, oEnd);
	if (sect2.alreadySet)
		return false;
	if (sect1.s && sect1.s == sect2.s)
		return false;
	if (sect1.o && sect1.o == sect2.o)
		return false;
	int usectID = seg->nextID();
	auto idEnds = [usectID, &match, isCoin](IsOpp isOpp) {
		IdEnds idEnds {
			match.reversed && (IsOpp::yes == isOpp || IsCoin::yes == isCoin) ? -usectID : usectID,
			IsOpp::yes == isOpp && match.reversed ? !match.match : match.match };
		return idEnds;
	};
	auto setSect = [isCoin, idEnds](OpIntersection* sect, IsOpp isOpp) {
		IdEnds ie = idEnds(isOpp);
		if (IsCoin::yes == isCoin)
			sect->setCoin(ie.id, ie.matchEnds);
		else
			sect->setUnsect(ie.id, ie.matchEnds);
	};
	auto addSect = [isCoin, idEnds](OpSegment* segs, OpSegment* opps, const OpPtT& start, 
			IsOpp isOpp  OP_LINE_FILE_ARGS()) {
		IdEnds ie = idEnds(isOpp);
		if (IsCoin::yes == isCoin)
			return segs->addCoin(start, ie.id, ie.matchEnds, opps  OP_LINE_FILE_PARGS());
		return segs->addUnsectable(start, ie.id, ie.matchEnds, opps  OP_LINE_FILE_PARGS());
	};
	auto addPair = [this, addSect, setSect](SectDuo sPair, const OpPtT& ePt, const OpPtT& oPt) {
		if (sPair.s) {
			setSect(sPair.s, IsOpp::no);
			setSect(sPair.o, IsOpp::yes);
		} else {
			sPair.s = addSect(seg, opp, ePt, IsOpp::no  OP_LINE_FILE_PARGS());
			sPair.o = addSect(opp, seg, oPt, IsOpp::yes  OP_LINE_FILE_PARGS());
			sPair.s->pair(sPair.o);
		}
	};
	addPair(sect1, eStart, oStart);
	match.match = MatchEnds::end;
	addPair(sect2, eEnd, oEnd);
	addedPoint = true;
	return true; 
}

// if after breaking runs spacially on both edge and opp into two runs
//  and one run is connected to already found intersections, remove that run
// return true if edges connected to small and large t are marked for removal (not overlapping)
bool OpCurveCurve::checkForGaps() {
	if (!edgeCurves.c.size())
		return false;
	if ((!smallTFound || !edgeCurves.c[0]->ccOverlaps || edgeCurves.c[0]->startT) 
			&& (!largeTFound || !edgeCurves.c.back()->ccOverlaps || 1 != edgeCurves.c.back()->endT))
		return false;
	OP_ASSERT(edgeCurves.c.size() && oppCurves.c.size());
	std::vector<CutRangeT> edgeGaps = edgeCurves.findGaps();
	if (edgeGaps.size() < (size_t) (smallTFound + largeTFound))  // require 2 gaps if sm && lg
		return false;
	std::vector<CutRangeT> oppGaps = oppCurves.findGaps();
	if (oppGaps.size() < (size_t) (smallTFound + largeTFound))
		return false;
	if (smallTFound) {
		OpPointBounds eGapBounds { edgeGaps[0].lo.pt, edgeGaps[0].hi.pt };
		size_t oIndex = matchRev.reversed ? oppGaps.size() - 1 : 0;
		OpPointBounds oGapBounds { oppGaps[oIndex].lo.pt, oppGaps[oIndex].hi.pt };
		if (eGapBounds.overlaps(oGapBounds))
			edgeCurves.markToDelete(0, edgeGaps[0].lo.t);
		if (matchRev.reversed)
			oppCurves.markToDelete(oppGaps[oIndex].hi.t, 1);
		else
			oppCurves.markToDelete(0, oppGaps[0].lo.t);
	}
	if (largeTFound) {
		OpPointBounds eGapBounds { edgeGaps.back().lo.pt, edgeGaps.back().hi.pt };
		size_t oIndex = matchRev.reversed ? 0 : oppGaps.size() - 1;
		OpPointBounds oGapBounds { oppGaps[oIndex].lo.pt, oppGaps[oIndex].hi.pt };
		if (eGapBounds.overlaps(oGapBounds))
			edgeCurves.markToDelete(edgeGaps.back().hi.t, 1);
		if (matchRev.reversed)
			oppCurves.markToDelete(0, oppGaps[0].lo.t);
		else
			oppCurves.markToDelete(oppGaps[oIndex].hi.t, 1);
	}
	return true;
}

bool OpCurveCurve::setSnipFromLimits(size_t oldCount) {
	if (oldCount >= limits.size())
		return false;
	FoundLimits limit = limits[oldCount];
	snipEdge = limit.seg;
	snipOpp = limit.opp;
	return true;
}

bool OpCurveCurve::checkSect() {
	for (auto edgePtr : edgeCurves.c) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		OpPtT edgeStart = edge.start();
		OpPtT edgeEnd = edge.end();
		OpVector threshold = contours->threshold();
		bool edgeDone = edgeStart.isNearly(edgeEnd, threshold);
		for (auto oppPtr : oppCurves.c) {
			auto& oppEdge = *oppPtr;
			if (!oppEdge.ccOverlaps)
				continue;
			OpPtT oppStart = oppEdge.start();
			OpPtT oppEnd = oppEdge.end();
			// check end condition
			if (edgeDone && (oppStart.isNearly(oppEnd, threshold))) {
				addIntersection(edgePtr, oppPtr);
				return true;
			} 
			if (ifExactly(edge, edgeStart, oppEdge, oppStart)
					|| ifExactly(edge, edgeEnd, oppEdge, oppStart)
					|| ifExactly(edge, edgeStart, oppEdge, oppEnd)
					|| ifExactly(edge, edgeEnd, oppEdge, oppEnd))
				return true;
			if (ifNearly(edge, edgeStart, oppEdge, oppStart)
					|| ifNearly(edge, edgeEnd, oppEdge, oppStart)
					|| ifNearly(edge, edgeStart, oppEdge, oppEnd)
					|| ifNearly(edge, edgeEnd, oppEdge, oppEnd))
				return true;
		}
	}
	return false;
}

// Scan through opposite curves and see if check point is inside deleted bounds. If so, use a
// different (but close by if possible) point to split the curve.
bool OpCurveCurve::checkSplit(float loT, float hiT, CurveRef which, OpPtT& checkPtT) const {
	OP_ASSERT(loT <= checkPtT.t && checkPtT.t <= hiT);
	const std::vector<OpEdge*>& oCurves = CurveRef::edge == which ? oppCurves.c : edgeCurves.c;
	const OpCurve& eCurve = CurveRef::edge == which ? seg->c : opp->c;
	float startingT = checkPtT.t;
	float deltaT = OpEpsilon;
	int attempts = 0;
	OpPtT original = checkPtT;
	const int maxAttempts = 16;  // !!! arbitrary guess
	do {
		// check edge cases (e.g., checkPtT.pt == oCurve end) first
		for (OpEdge* oCurve : oCurves) {
			if (oCurve->ccOverlaps && oCurve->ptBounds.contains(checkPtT.pt))
				return original != checkPtT;
		}
		// check for gap between original and edge list, and between edges in edge list
		const OpEdge& oEdge = CurveRef::edge == which ? opp->edges[0] : seg->edges[0];
		OpPtT delLo = oEdge.start();
		auto checkBounds = [checkPtT](const OpPtT& lo, const OpPtT& hi) {
			OpPointBounds delBounds(lo.pt, hi.pt);
			return delBounds.contains(checkPtT.pt);
		};
		for (OpEdge* oCurve : oCurves) {
			OpPtT delHi = oCurve->start();
			OP_ASSERT(delLo.t <= delHi.t);
			if (delLo.t < delHi.t && checkBounds(delLo, delHi))  // there's a gap between edges
				goto tryAgain;
			delLo = oCurve->end();
			if (!oCurve->ccOverlaps && checkBounds(delHi, delLo))  // check edge without overlap
				goto tryAgain;
		}
		if (delLo.t == oEdge.endT || !checkBounds(delLo, oEdge.end()))  // check last list / original
			return original != checkPtT;
	tryAgain:
		checkPtT.t = startingT + deltaT;
		if (loT > checkPtT.t || checkPtT.t > hiT) {
			checkPtT.t = startingT - deltaT;
			if (loT > checkPtT.t || checkPtT.t > hiT) {
				deltaT /= -2;
				continue;
			}
		}
		checkPtT.pt = eCurve.ptAtT(checkPtT.t);
		deltaT *= -2;
	} while (++attempts < maxAttempts);
//	OP_ASSERT(0);  // decide what to do when assert fires
	checkPtT = original;
	return false;
}

void OpCurveCurve::checkUnsplitables() {
	for (OpEdge* eCurve : edgeCurves.c) {
		if (!eCurve->isUnsplitable)
			continue;
		for (OpEdge* oCurve : oppCurves.c) {
			if (!oCurve->isUnsplitable)
				continue;
			bool hullsIntersect = false;
			OpPtT ePt = eCurve->center;
			for (HullSect& hull : eCurve->hulls.h) {
				if (hull.opp == oCurve) {
					ePt = hull.sect;
					hullsIntersect = true;
					break;
				}
			}
			OpPtT oPt = oCurve->center;
			for (HullSect& hull : oCurve->hulls.h) {
				if (hull.opp == eCurve) {
					oPt = hull.sect;
					hullsIntersect = true;
					break;
				}
			}
			if (hullsIntersect) {
				recordSect(eCurve, oCurve, ePt, oPt  OP_LINE_FILE_PARGS());
				eCurve->ccOverlaps = false;
				oCurve->ccOverlaps = false;
				return;
			}
		}
	}
}

// Divide only by geometric midpoint. Midpoint is determined by endpoint intersection, or
// curve bounds if no intersection is found.
// try several approaches:
// 1) call existing code for baseline (bounds split on geometric middle, find curve root with line)
// 2) rotate curve to opp start/end line and use newton method to find intersection
//    if we're going to rotate, take advantage and do another bounds check
// 3) if line/line intersects, divide at that point
// 4) if line/line intersects, divide at corresponding ts
// 5) split geometric until t doesn't change (expect to be slow)
// 6) binary search on t (expect to be slower)
SectFound OpCurveCurve::divideAndConquer() {
	OP_DEBUG_CONTEXT();
	OP_ASSERT(1 == edgeCurves.c.size());
	OP_ASSERT(1 == oppCurves.c.size());
//	int splitCount = 0;
	// !!! start here;
	// looks like recursion is getting stuck trimming tiny bits from end instead of dividing in the
	// middle and/or detecting that it ought to look to see if the run is largely unsectable. It
	// should be trying in more than one place to find intersections somehow

	// !!! testQuads5721199 segments 2 and 5 share (0, 0) but iterate depth to 24 to see if they
	// intersect a second time. Not sure what to do...
	for (depth = 1; depth < maxDepth; ++depth) {
//		edgeCurves.endDist(seg, opp);
//		oppCurves.endDist(opp, seg);
		OP_ASSERT(debugShowImage(true));
		bool snipEm = false;
		if (!setOverlaps())
			return SectFound::fail;
		if (checkForGaps() || (splitMid && !endsOverlap()))
			splitMid = false;
		int edgeOverlaps = edgeCurves.overlaps();
		int oppOverlaps = oppCurves.overlaps();
		if (!edgeOverlaps || !oppOverlaps) {
			if (depth > 8) // !!! number arbitrary, depth is 9 for testQuads3993265
				return SectFound::noOverlapDeep;
			return limits.size() ? SectFound::add : SectFound::no;
		}
		if (edgeOverlaps >= maxSplits || oppOverlaps >= maxSplits)
			return SectFound::maxOverlaps;  // more code required
		if (checkSect())
			snipEm = true;
		else {
			size_t limitCount = limits.size();
			setHulls(CurveRef::edge);
			setHulls(CurveRef::opp);
			snipEm = setSnipFromLimits(limitCount);
		}
		// if there is more than one crossover, look for unsectable
		size_t limitsSize = uniqueLimits();
//		start here;
		// !!! add: while there are live edges that crossover the dist axis from + to -, continue
		if (limitsSize > 0 && !lastDepthReduced 
				&& limitsSize > std::max(1U, (unsigned) (smallTFound + largeTFound))) {
			if (!reduceDistFlipped())
				return SectFound::add;
			lastDepthReduced = true;
			if (!snipEm)
				continue;
		}
#if OP_DEBUG && OP_DEBUG_VERBOSE  // save state prior to split and delete
		debugSaveState();
#endif
		// If any intersection is found, already found or not, remove piece around 
		// both edge and opp so that remaining edges can be checked for intersection
		if (!snipEm) {
			CcCurves eSplits, oSplits;
			size_t limitCount = limits.size();
			unsplitables = 0;
			if (!splitHulls(CurveRef::edge, eSplits))
				return SectFound::fail;
			if (!splitHulls(CurveRef::opp, oSplits))
				return SectFound::fail;
			edgeCurves.c.swap(eSplits.c);
			oppCurves.c.swap(oSplits.c);
			if (!edgeCurves.c.size()) {
				if (splitHullFail)  // split hulls failed to split -- runs crossing axis is sect
					return SectFound::fail;  // note that this is very conservative and narrow
				return limits.size() ? SectFound::add : SectFound::no;
			}
			// !!! if edge curves has unsplittable and opp curves has unsplittable, 
			// add limit and remove both
			if (1 < unsplitables)
				checkUnsplitables();
			snipEm = setSnipFromLimits(limitCount);
		}
		if (snipEm) {
			edgeCurves.snipAndGo(seg, snipEdge, snipOpp.pt, opp);
			oppCurves.snipAndGo(opp, snipOpp, snipEdge.pt, seg);
		}
	}
//	OP_ASSERT(0);  // !!! if this occurs likely more code is needed
	return SectFound::fail;
}

// return true if either small t or large t belong to edge that is still available
bool OpCurveCurve::endsOverlap() const {
	if (!edgeCurves.c.size() || !oppCurves.c.size())
		return false;
	if (largeTFound) {
		const OpEdge* last = edgeCurves.c.back();
		if (last->ccLarge && last->ccOverlaps)
			return true;
		last = matchRev.reversed ? oppCurves.c[0] : oppCurves.c.back();
		if (last->ccLarge && last->ccOverlaps)
			return true;
	}
	if (smallTFound) {
		const OpEdge* last = edgeCurves.c[0];
		if (last->ccSmall && last->ccOverlaps)
			return true;
		last = matchRev.reversed ? oppCurves.c.back() : oppCurves.c[0];
		if (last->ccSmall && last->ccOverlaps)
			return true;
	}
	return false;
}

// (assuming its already been determined that the edge pair is likely partially unsectable
// binary search pair of curves
// look for range of unsectables and snip remaining more aggressively
// if one end t is already found, add it in to calculus of finding range of unsectable
// in any case, check the found range to see if the midpoint is also plausibly unsectable
// before treating the entire range as unsectable
// either: generate one unsectable range, or two points
// (unsectable range + one point or two unsectable ranges may be better, but wait for test case)

// rework this to keep unsectable ranges in curve-curve until all intersections are found, and then
// output intersections and intersection ranges
// start by changing curve-curve constructor to record range for small and large t found, if any
void OpCurveCurve::findUnsectable() {
	// start with existing found point
	// separate limits into unsectable and regular
	// continue as long as unsectable until entire unsectable range is found
	// then, skip regular if they are from found t
	OP_ASSERT(limits.size() || smallTFound || largeTFound);
	// find limit of where curve pair are nearly coincident
	std::sort(limits.begin(), limits.end(), [](const FoundLimits& a, const FoundLimits& b) {
			return a.seg.t < b.seg.t; });
	// mark unsectables with opposite t values that are not ordered
	// !!! start out conservative and only mark limits outside of first/last
	if (limits.size() > 2) {
		float frontOppT = limits.front().opp.t;
		float backOppT = limits.back().opp.t;
		for (size_t index = 1; index < limits.size() - 1; ++index) {
			FoundLimits& limit = limits[index];
			if (!OpMath::Between(frontOppT, limit.opp.t, backOppT))
				limit.oppOutOfOrder = true;
		}
	}
	// check midpoints to see if they are also on the curve
	size_t unsectLo = 0;
	size_t unsectHi = 0;
	auto addSect = [this, &unsectLo, &unsectHi]() {
		FoundLimits& limit = limits[unsectLo];
		if (unsectLo < unsectHi
				&& addUnsectable(limit.seg, limits[unsectHi].seg, limit.opp, limits[unsectHi].opp))
			return;
		if (limit.fromFoundT)
			return;
		OpPtT::MeetInTheMiddle(limit.seg, limit.opp);
		if (seg->sects.contains(limit.seg, opp))
			return;
		if (opp->sects.contains(limit.opp, seg))
			return;
		OpIntersection* sect = seg->addSegSect(limit.seg, opp  OP_LINE_FILE_PARGS());
		OpIntersection* oSect = opp->addSegSect(limit.opp, seg  OP_LINE_FILE_PARGS());
		sect->pair(oSect);
		addedPoint = true;
	};
	float lastT = limits[0].seg.t;
	for (size_t index = 1; index < limits.size(); ++index) {
		FoundLimits& limit = limits[index];
		if (limit.oppOutOfOrder)
			continue;
		float t = limit.seg.t;
		OP_ASSERT(t > lastT);
		float midT = OpMath::Average(lastT, t);
		OpPtT midPtT = seg->c.ptTAtT(midT);
		OpPtT oppTest = CcCurves::Dist(seg, midPtT, opp);
		OpVector oppdist = midPtT.pt - oppTest.pt;
		if (oppdist.lengthSquared() <= OpEpsilon) // !!! epsilon is unfounded guess
			unsectHi = index;
		else {
			addSect();
			unsectLo = index;
		}
		lastT = t;
	}
	addSect();
}

bool OpCurveCurve::ifExactly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& oppEdge, const OpPtT& oppPtT) {
	if (edgePtT.pt != oppPtT.pt)
		return false;
	if (edge.ccStart && edge.startT == edgePtT.t)
		return false;
	if (edge.ccEnd && edge.endT == edgePtT.t)
		return false;
	recordSect(&edge, &oppEdge, edgePtT, oppPtT  OP_LINE_FILE_PARGS());
	snipEdge = edgePtT;
	snipOpp = oppPtT;
	return true;
}

bool OpCurveCurve::ifNearly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& oppEdge, const OpPtT& oppPtT) {
	OpVector threshold = contours->threshold();
	if (!edgePtT.pt.isNearly(oppPtT.pt, threshold))
		return false;
	if (edge.ccStart && edge.start().isNearly(edgePtT, threshold))
		return false;
	if (edge.ccEnd && edge.end().isNearly(edgePtT, threshold))
		return false;
	recordSect(&edge, &oppEdge, edgePtT, oppPtT  OP_LINE_FILE_PARGS());
	snipEdge = edgePtT;
	snipOpp = oppPtT;
	return true;
}

bool OpCurveCurve::alreadyInLimits(const OpEdge* edge, const OpEdge* oEdge, float t) {
	for (FoundLimits& limit : limits) {
		if (limit.parentEdge == edge && limit.parentOpp == oEdge)
			return true;  // reverse case already recorded
		if (OpMath::EqualT(limit.seg.t, t))
			return true;  // already recorded
	}
#if 0  // this breaks testQuads5635157; instead, disallow t values in deleted ranges
// if this edge/opEdge is a subset of an existing limit, replace it
	for (size_t index = 0; index < limits.size(); ++index) {
		FoundLimits& limit = limits[index];
		if (limit.parentEdge->segment != edge->segment)
			continue;
		OP_ASSERT(limit.parentOpp->segment == oEdge->segment);
		if (limit.parentEdge->startT > edge->startT)
			continue;
		if (limit.parentEdge->endT < edge->endT)
			continue;
		limits.erase(limits.begin() + index);  // caller replaces limit with more precise data
		break;
	}
#else
	CcCurves& curves = edge->segment == seg ? edgeCurves : oppCurves;
	if (curves.deletedT(t))
		return true;
#endif
	return false;  // caller adds to limit
}

bool OpCurveCurve::betweenLimits(const OpEdge* edge, const OpEdge* oEdge, float lo, float hi) {
	for (FoundLimits& limit : limits) {
		if (limit.parentEdge == edge && limit.parentOpp == oEdge)
			return true;  // reverse case already recorded
		if (OpMath::Between(lo, 
				edge->segment == limit.parentEdge->segment ? limit.opp.t : limit.seg.t, hi))
			return true;  // already recorded
	}
	return false;
}

// defer meet in the middle stuff until all intersections are found
void OpCurveCurve::recordSect(OpEdge* edge, OpEdge* oEdge, const OpPtT& edgePtT, const OpPtT& oppPtT
			OP_LINE_FILE_ARGS()) {
	if (alreadyInLimits(edge, oEdge, edgePtT.t))
		return;
	FoundLimits newLimit { edge, oEdge, edgePtT, oppPtT, false, false  OP_LINE_FILE_SCALLER() };
	limits.push_back(std::move(newLimit));
	uniqueLimits_impl = -1;
}

// remove edges that do not change distance sign in run

bool OpCurveCurve::reduceDistFlipped() {
	CcCurves edgeSplits, oppSplits;
	CcCurves* curves = &edgeCurves;
	bool swap = false;
	do {
		EdgeRun* lower = nullptr;
		EdgeRun* upper = nullptr;
		size_t lodex = OpMax;
		size_t hidex = OpMax;
		auto keepRun = [&lower, &upper, this](CcCurves& splits) {
			void* block = contours->allocateEdge(contours->ccStorage);
			OpEdge* split = new(block) OpEdge(lower->edge, lower->edgePtT.t, 
					upper->edgePtT.t  OP_LINE_FILE_PARGS());
			split->ccOverlaps = true;
			OP_ASSERT(!split->disabled);
			splits.c.push_back(split);
		};
		auto markByZero = [&lodex, &hidex, &lower, &upper, &curves]() {
			lower = &curves->runs[lodex];
			upper = OpMax == hidex ? nullptr : &curves->runs[hidex];
			// only disable prior if the mid point is equal to or smaller than its distance
			// mark prior two edges to disable sign compare
			if (lodex > 1 && curves->checkMid(lodex - 1)) {
				curves->runs[lodex - 1].byZero = true;
			}
			if (OpMax == hidex)
				hidex = lodex;
			// mark next two edges to disable sign compare
			if (hidex + 2 < curves->runs.size() && curves->checkMid(hidex))
				curves->runs[hidex + 1].byZero = true;
			lodex = hidex = OpMax;
		};
		for (size_t index = 0; index < curves->runs.size(); ++index) {
			EdgeRun& run = curves->runs[index];
			if (fabsf(run.oppDist) <= OpEpsilon)
				(OpMax == lodex ? lodex : hidex) = index;
			else if (OpMax != lodex)
				markByZero();
		}
		if (OpMax != lodex)
			markByZero();
		// second pass: add edges with distances that switch signs but are not adjacent to very small
		for (auto& run : curves->runs) {
			upper = &run;
			if (lower && lower->oppDist * run.oppDist < 0 // switches sides
					&& fabsf(lower->oppDist) > OpEpsilon && fabsf(run.oppDist) > OpEpsilon
					&& !lower->byZero && !run.byZero
					&& (lower->edge == upper->edge  // skip if gap between last run and this run
					|| lower->edge->endT == upper->edge->startT))
				keepRun(swap ? oppSplits : edgeSplits);
			lower = upper;
		}
		curves = &oppCurves;
	} while ((swap = !swap));
	if (edgeSplits.c.size()) {
		edgeSplits.deleted = edgeCurves.deleted;
		oppSplits.deleted = oppCurves.deleted;
		edgeCurves = edgeSplits;
		oppCurves = oppSplits;
		edgeCurves.oppCurves = &oppCurves;
		oppCurves.oppCurves = &edgeCurves;
		return true;
	}
	return false;
}

bool OpCurveCurve::rotatedIntersect(OpEdge& edge, OpEdge& oppEdge, bool sharesPoint) {
	LinePts edgePts { edge.startPt(), edge.endPt() };
	const OpCurve& edgeRotated = edge.setVertical(edgePts, MatchEnds::start);
	rotateFailed |= !edgeRotated.isFinite();
	MatchReverse match = oppEdge.matchEnds(edgePts);
	const OpCurve& oppRotated = oppEdge.setVertical(edgePts, match.match);
	rotateFailed |= !oppRotated.isFinite();
	OpPointBounds eRotBounds = edgeRotated.ptBounds();
	OpPointBounds oRotBounds = oppRotated.ptBounds();
	if (rotateFailed || !eRotBounds.intersects(oRotBounds))
		return false;
	// !!! can one have no area (e.g. horz or vert) and the other not?
	return !sharesPoint || !eRotBounds.hasArea() || !oRotBounds.hasArea() 
			|| eRotBounds.areaOverlaps(oRotBounds);
}

SectFound OpCurveCurve::runsToLimits() {
	size_t oldLimits = limits.size();
	bool swap = false;
	auto addIfNew = [this, &swap](EdgeRun* run) {
		FoundLimits lowerLimit { run->edge, run->oppEdge, run->edgePtT, run->oppPtT, 
				run->fromFoundT, false  OP_LINE_FILE_STRUCT() };
		if (swap) {
			std::swap(lowerLimit.parentEdge, lowerLimit.parentOpp);
			std::swap(lowerLimit.seg, lowerLimit.opp);
		}
		if (!alreadyInLimits(lowerLimit.parentEdge, lowerLimit.parentOpp, lowerLimit.seg.t))
			limits.push_back(std::move(lowerLimit));
	};
	EdgeRun* lower, * lastUpper, * upper;
	CcCurves* curves = &edgeCurves;
	CcCurves* oCurves = &oppCurves;
	auto addLimit = [this, &lower, &lastUpper, &upper, &curves, &oCurves, addIfNew]() {
		OP_ASSERT(!upper || lower->edge->segment == upper->edge->segment);
		if (upper && betweenLimits(lower->edge, lower->oppEdge, lower->oppPtT.t, upper->oppPtT.t))
			return;
		if (!lower->inDeleted(curves, oCurves) && lower != lastUpper 
				&& (!upper || fabsf(lower->oppDist) <= fabsf(upper->oppDist)))
			addIfNew(lower);
		if (upper && !upper->inDeleted(curves, oCurves) 
				&& fabsf(lower->oppDist) > fabsf(upper->oppDist)) {
			addIfNew(upper);
			lastUpper = upper;
		}
	};
	// first pass: add edges with very small distances
	size_t lodex, hidex;
	auto markByZero = [&lodex, &hidex, &lower, &upper, &curves, addLimit]() {
			lower = &curves->runs[lodex];
			upper = OpMax == hidex ? nullptr : &curves->runs[hidex];
			addLimit();
			// only disable prior if the mid point is equal to or smaller than its distance
			// mark prior two edges to disable sign compare
			if (lodex > 1 && curves->checkMid(lodex - 1)) {
				curves->runs[lodex - 1].byZero = true;
			}
			if (OpMax == hidex)
				hidex = lodex;
			// mark next two edges to disable sign compare
			if (hidex + 2 < curves->runs.size() && curves->checkMid(hidex))
				curves->runs[hidex + 1].byZero = true;
			lodex = hidex = OpMax;
	};
	// scan both edge curves and opp curves
	do {
		lower = nullptr;
		lastUpper = nullptr;
		upper = nullptr;
		lodex = OpMax;
		hidex = OpMax;
		for (size_t index = 0; index < curves->runs.size(); ++index) {
			EdgeRun& run = curves->runs[index];
			if (fabsf(run.oppDist) <= OpEpsilon)
				(OpMax == lodex ? lodex : hidex) = index;
			else if (OpMax != lodex)
				markByZero();
		}
		if (OpMax != lodex)
			markByZero();
		// second pass: add edges with distances that switch signs but are not adjacent to very small
		for (auto& run : curves->runs) {
			upper = &run;
			if (lower && lower->oppDist * run.oppDist < 0 
					&& fabsf(lower->oppDist) > OpEpsilon && fabsf(run.oppDist) > OpEpsilon
				// !!! very large dist sign swaps should be ignored
				//     need to determine what this range should be
					&& fabsf(lower->oppDist) < OpEpsilon * maxSignSwap 
					&& fabsf(run.oppDist) < OpEpsilon * maxSignSwap
					&& !lower->byZero && !run.byZero)  // switches sides
				addLimit();
			lower = upper;
		}
		curves = &oppCurves;
	} while ((swap = !swap));
	return limits.size() == oldLimits ? SectFound::no : SectFound::add;
}

// finds intersections of opp edge's hull with edge, and stores them in edge's hulls
// returns true if found intersection is true curve curve intersection (or at least, close enough)
void OpCurveCurve::setHullSects(OpEdge& edge, OpEdge& oppEdge, CurveRef curveRef) {
	int ptCount = oppEdge.curve.pointCount();
	LinePts oppPts;
	oppPts.pts[1] = oppEdge.curve.firstPt();
	OpVector threshold = contours->threshold();
	for (int index = 1; index <= ptCount; ++index) {
		oppPts.pts[0] = oppPts.pts[1];
		int endHull = index < ptCount ? index : 0;
		oppPts.pts[1] = oppEdge.curve.hullPt(endHull);
		if (oppPts.pts[0].isNearly(oppPts.pts[1], threshold))
			continue;
		// since curve/curve intersection works by keeping overlapping edge bounds, it should
		// use edge, not segment, to find hull intersections
		OpRootPts septs = edge.curve.lineIntersect(oppPts);
		for (size_t inner = 0; inner < septs.count; ++inner) {
			OpPtT sectPtT = septs.ptTs[inner];
			// set to secttype endhull iff computed point is equal to or nearly an end point
			SectType sectType;
			if ((1 == index || 0 == endHull) 
					&& sectPtT.pt.isNearly(oppPts.pts[0], threshold)) {
				sectType = SectType::endHull;
				sectPtT.pt = oppPts.pts[0];
			} else if ((ptCount - 1 == index || 0 == endHull) 
					&& sectPtT.pt.isNearly(oppPts.pts[1], threshold)) {
				sectType = SectType::endHull;
				sectPtT.pt = oppPts.pts[1];
			} else
				sectType = endHull ? SectType::controlHull : SectType::midHull;
			sectPtT.t = OpMath::Interp(edge.startT, edge.endT, sectPtT.t);
			OP_ASSERT(edge.startT <= sectPtT.t && sectPtT.t <= edge.endT);
			// if pt is close to existing hull sect, and both are not end, record intersection
			if (edge.hulls.add(sectPtT, contours->threshold(), sectType, &oppEdge)) {
				OpSegment* oSeg = oppEdge.segment;
				OpPtT oppPtT { oSeg->c.ptTAtT(oSeg->findValidT(0, 1, sectPtT.pt))};
				if (!oppPtT.pt.isFinite())
					return;
				if (CurveRef::edge == curveRef)
					recordSect(&edge, &oppEdge, sectPtT, oppPtT  OP_LINE_FILE_PARGS());
				else
					recordSect(&oppEdge, &edge, oppPtT, sectPtT  OP_LINE_FILE_PARGS());
				return;
			}
		}
	}
}

// if ref is edge, records all intersections of edges with all hulls of opp
void OpCurveCurve::setHulls(CurveRef curveRef) {
	std::vector<OpEdge*>& eCurves = CurveRef::edge == curveRef ? edgeCurves.c : oppCurves.c;
	std::vector<OpEdge*>& oCurves = CurveRef::edge == curveRef ? oppCurves.c : edgeCurves.c;
	for (auto edgePtr : eCurves) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		for (auto oppPtr : oCurves) {
			auto& oppEdge = *oppPtr;
			if (!oppEdge.ccOverlaps)
				continue;
			if (!splitMid || edge.isLine())
				setHullSects(edge, oppEdge, curveRef);
		}
	}
}

#if 0
// !!! need to debug test case to get unsectable range right
void OpCurveCurve::setIntersections() {
	if (!limits.size())
		return;
	for (FoundLimits& limit : limits) {
		if (limit.fromFoundT)
			continue;
		OpIntersection* sect = seg->addEdgeSect(limit.seg  
				OP_LINE_FILE_PARAMS(limit.parentEdge, limit.parentOpp));
		OpIntersection* oSect = opp->addEdgeSect(limit.opp  
				OP_LINE_FILE_PARAMS(limit.parentEdge, limit.parentOpp));
		sect->pair(oSect);
		addedPoint = true;
	}
}
#endif

bool OpCurveCurve::setOverlaps() {
	edgeCurves.clear();
	oppCurves.clear();
	for (auto edgePtr : edgeCurves.c) {
		auto& edge = *edgePtr;
		for (auto oppPtr : oppCurves.c) {
			auto& oppEdge = *oppPtr;
			// !!! used to be close bounds; simple bounds required for testQuads1883885
			if (depth > 1 && !edge.bounds().intersects(oppEdge.bounds()))
				continue;
			bool sharesPoint = (edge.ccStart || edge.ccEnd) && (oppEdge.ccStart || oppEdge.ccEnd);
			// if bounds have common edge only and already share point, they don't intersect
			if (sharesPoint) {
				// !!! can one have no area (e.g. horz or vert) and the other not?
				if (edge.bounds().hasArea() && oppEdge.bounds().hasArea() &&
						!edge.bounds().areaOverlaps(oppEdge.bounds()))
					continue;
			}
			if (!rotatedIntersect(edge, oppEdge, sharesPoint))
				continue;
			if (!rotatedIntersect(oppEdge, edge, sharesPoint))
				continue;
			oppEdge.ccOverlaps = true;
			edge.ccOverlaps = true;
		}
	}
	return !rotateFailed;
}

bool OpCurveCurve::splitDownTheMiddle(const OpEdge& edge, const OpPtT& edgeMid, CurveRef curveRef,
			CcCurves& splits) {
	// !!!? while edgeMid is in a deleted bounds, bump edgeMidT
	//      (this isn't necessarily in the intersection hull)
	//      wait until this is necessary to make it work
	// checkSplit(edge.startT, edge.endT, which, edgeMid);
	if (OpMath::EqualT(edge.startT, edgeMid.t))
		return false;
	if (OpMath::EqualT(edgeMid.t, edge.endT))
		return false;
	if (edge.centerless)
		return false;
	OP_ASSERT(edge.startT < edgeMid.t);
	OP_ASSERT(edgeMid.t < edge.endT);
	CcCurves& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	const OpSegment* oppSeg = CurveRef::edge == curveRef ? opp : seg;
	void* blockL = contours->allocateEdge(contours->ccStorage);
	OpEdge* splitLeft = new(blockL) OpEdge(&edge, edgeMid, NewEdge::isLeft  
			OP_LINE_FILE_PARGS());
	if (!splitLeft->disabled) {
		splitLeft->ccOverlaps = true;
		splitLeft->ccStart = edge.ccStart;
		splitLeft->ccSmall = edge.ccSmall;
		OP_ASSERT(!splitLeft->disabled);
		splits.c.push_back(splitLeft);
		curves.addEdgeRun(splitLeft, oppSeg, EdgeMatch::end);
	}
	void* blockR = contours->allocateEdge(contours->ccStorage);
	OpEdge* splitRight = new(blockR) OpEdge(&edge, edgeMid, NewEdge::isRight  
			OP_LINE_FILE_PARGS());
	if (!splitRight->disabled) {
		splitRight->ccOverlaps = true;
		splitRight->ccEnd = edge.ccEnd;
		splitRight->ccLarge = edge.ccLarge;
		OP_ASSERT(!splitRight->disabled);
		splits.c.push_back(splitRight);
	}
	return true;
}

// If edge has hull points that define split but opp does not, opp may be entirely inside edge hull.
// Modify below to keep opp corresponding to kept edge split to handle this case.
// where should the opp be stored?

// if end hull index == -1, discard both sides of sect (cutout via exact or nearby sect)
// if end hull >= 0, look for sect through curve
bool OpCurveCurve::splitHulls(CurveRef which, CcCurves& splits) {
	OpVector threshold = contours->threshold();
	const CcCurves& curves = CurveRef::edge == which ? edgeCurves : oppCurves;
	for (auto edgePtr : curves.c) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		if (edge.start().isNearly(edge.center, threshold) 
				|| edge.center.isNearly(edge.end(), threshold)) {
			edge.isUnsplitable = true;
			++unsplitables;
			splits.c.push_back(edgePtr);  // caller will split with snip and go
			if (CurveRef::edge == which)
				splitHullFail = true;
			continue;
		}
		if (CurveRef::edge == which)
			splitHullFail = false;
		OpHulls& hulls = edge.hulls;
		if (!hulls.h.size() || splitMid) {
			if (!splitDownTheMiddle(edge, edge.center, which, splits))
				return false;
			continue;
		}
		if (2 <= hulls.h.size()) {  // see if hulls are close enough to define an intersection
			hulls.sort(smallTFound);
	//		size_t limitCount = limits.size();
			for (size_t index = 1; index < hulls.h.size(); ++index) {
				if (!hulls.sectCandidates((int) index, edge))
					continue;
				OpDebugOut("!!! splitHulls fail:"  OP_DEBUG_CODE(+ contours->debugData.testname)
					+ std::string("\n"));
			#if 0
				OP_ASSERT(hulls.h[index - 1].opp);
				OpEdge& oEdge = const_cast<OpEdge&>(*hulls.h[index - 1].opp);
				OpPtT oPtT;
				OpPtT hull1Sect = hulls.h[index - 1].sect;
				oPtT.t = oEdge.segment->findValidT(0, 1, hull1Sect.pt);
				if (OpMath::IsNaN(oPtT.t))	{
					OP_ASSERT(0);  // !!! was return false; debug to see if this can continue
					continue;
				}
				oPtT.pt = oEdge.segment->c.ptAtT(oPtT.t);
				if (!hulls.closeEnough(index - 1, edge, oEdge, &oPtT, &hull1Sect))
					continue;
				if (CurveRef::opp == which)
					recordSect(&oEdge, edgePtr, oPtT, hull1Sect  OP_LINE_FILE_PARGS());
				else 
					recordSect(edgePtr, &oEdge, hull1Sect, oPtT  OP_LINE_FILE_PARGS());
	//			curves.snipAndGo(edge.segment, hull1Sect);
	//			oCurves.snipAndGo(oEdge.segment, oPtT);
			#endif
			}
	#if 0
			if (limitCount < limits.size()) {
				OP_ASSERT(!edgePtr->disabled);
				splits.c.push_back(edgePtr);  // caller will split with snip and go
				continue;
			}
	#endif
		}
	#if 0
		int midCount = 0;
		for (const HullSect hull : hulls.h) {
			if (edge.startT <= hull.sect.t && hull.sect.t <= edge.endT)
				++midCount;
		}
		if (!midCount) {
			OpPtT cutHere;
			for (const HullSect hull : hulls.h) {
		#if 0
				if (OpMath::EqualT(hull.sect.t, edge.startT)) 
					cutHere = edge.segment->c.cut(hull.sect, edge.startT, edge.endT, 1);
				else if (OpMath::EqualT(hull.sect.t, edge.endT)) 
					cutHere = edge.segment->c.cut(hull.sect, edge.startT, edge.endT, -1);
				else
		#endif
					continue;
				OP_ASSERT(edge.startT < cutHere.t && cutHere.t < edge.endT);
				OP_ASSERT(!OpMath::EqualT(edge.startT, cutHere.t));
				OP_ASSERT(!OpMath::EqualT(cutHere.t, edge.endT));
			}
			if (!OpMath::IsNaN(cutHere.t)) {
				if (!splitDownTheMiddle(edge, cutHere, which, splits))
					return false;
				continue;
			}
			OP_ASSERT(0);  // !!! looks broken; fix with test case (or delete if never called)
		}
	#endif
		OP_DEBUG_VALIDATE_CODE(hulls.debugValidate());
		hulls.add(edge.start(), threshold, SectType::endHull);
		hulls.add(edge.end(), threshold, SectType::endHull);
		hulls.nudgeDeleted(edge, *this, which);
		size_t splitCount = splits.c.size();
		for (size_t index = 0; index + 1 < hulls.h.size(); ) {
			const HullSect& hullLo = hulls.h[index];
			const HullSect& hullHi = hulls.h[++index];
			if (edge.ccStart && edge.start().isNearly(hullHi.sect, threshold))
				continue;
			if (edge.ccEnd && edge.end().isNearly(hullLo.sect, threshold))
				continue;
			if (OpMath::EqualT(hullLo.sect.t, hullHi.sect.t))
				continue;
			void* block = contours->allocateEdge(contours->ccStorage);
			OpEdge* split = new(block) OpEdge(&edge, hullLo.sect.t, 
					hullHi.sect.t  OP_LINE_FILE_PARGS());
			if (split->disabled)
				continue;
			split->ccOverlaps = true;
			split->ccStart = edge.ccStart && edge.start().isNearly(split->start(), threshold);
			if (split->ccStart)
				split->ccSmall = edge.ccSmall;
			split->ccEnd = edge.ccEnd && edge.start().isNearly(split->end(), threshold);
			if (split->ccEnd)
				split->ccLarge = edge.ccLarge;
			if (split->endT < edge.endT)
				addEdgeRun(split, which, EdgeMatch::end);
			splits.c.push_back(split);
		}
		if (splits.c.size() - splitCount < 2) {
			splits.c.resize(splitCount); // this may leave an edge was never used ... not the end of the world
			if (!splitDownTheMiddle(edge, edge.center, which, splits))  // divide edge in two
				return false;
		}
	}
	return true;
}

size_t OpCurveCurve::uniqueLimits() {
	if (limits.size() <= 1)
		uniqueLimits_impl = (int) limits.size();
	if (uniqueLimits_impl >= 0)
		return uniqueLimits_impl;
	std::sort(limits.begin(), limits.end(), [](FoundLimits& a, FoundLimits& b) {
		return a.seg.t < b.seg.t;
	} );
	size_t result = 1;
	const FoundLimits* last = &limits[0];
	float lastDistSq = (last->seg.pt - last->opp.pt).lengthSquared();
	OpVector threshold = contours->threshold();
	for (size_t index = 1; index < limits.size(); ++index) {
		const FoundLimits* limit = &limits[index];
		bool soClose = last->seg.isNearly(limit->seg, threshold);
		soClose |= last->opp.isNearly(limit->opp, threshold);
		float distSq = (limit->seg.pt - limit->opp.pt).lengthSquared();
		if (!soClose) {
			OpPtT midPtT = seg->c.ptTAtT(OpMath::Average(last->seg.t, limits[index].seg.t));
			OpPtT midOpp = CcCurves::Dist(seg, midPtT, opp);
			float midDist = (midPtT.pt - midOpp.pt).lengthSquared();
			if (midDist > lastDistSq && midDist > distSq)
				++result;
		}
		last = &limits[index];
		lastDistSq = distSq;
	}
	uniqueLimits_impl = (int) result;
	return result;
}

