// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include <utility>

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

bool EdgeRun::inDeleted(CcCurves* curves, CcCurves* oppCurves) const {
	return curves->deletedT(edgePtT.t) || oppCurves->deletedT(oppPtT.t);
}

void EdgeRun::set(OpEdge* edge, OpEdge* opp, EdgeMatch match) {
//	runEdge = edge;
//	runOpp = opp;
	edgePtT = EdgeMatch::start == match ? edge->start() : edge->end();
	oppPtT = edge->segment->distance(edgePtT, opp->segment);
	if (OpMath::IsFinite(oppPtT.t)) {
		setOppDist(edge->segment);
		if (EdgeMatch::start == match) {
			edge->startDist = oppDist;
			edge->startOpp = oppPtT;
		} else {
			edge->endDist = oppDist;
			edge->endOpp = oppPtT;
		}
	} else
		oppDist = OpNaN;
	fromFoundT = false;
	byZero = false;
	OP_DEBUG_CODE(debugBetween = 1);
}

// start here;
// if distance is zero, that defines an intersection (which may or may not have already been found)
// record intersection, and snip out (by marking as deleted?)
void EdgeRun::setOppDist(const OpSegment* segment) {
	if (OpMath::IsNaN(edgePtT.t)) {
		oppDist = OpNaN;
		return;
	}
	OpVector oppV = edgePtT.pt - oppPtT.pt;
	oppDist = oppV.length();
	if (OpMath::IsNaN(oppDist))
		return;
	if (oppDist <= segment->thresholdLength()) {
		oppDist = 0;
//		OpAssert(OpMath::NearlyEndT(edgePtT.t) && OpMath::NearlyEndT(oppPtT.t));
		return;
	}
	OpVector normal = segment->c.normal(edgePtT.t);
	float nDotOpp = normal.dot(oppV);
	if (nDotOpp < -OpEpsilon)
		oppDist = -oppDist;
	else if (nDotOpp <= OpEpsilon)
		oppDist = 0;
}

/* main cases: (where smaller, larger are distances) existing1=1, existing2=2, n=newDistance
   if existing changes sign, insert
   if existing same sign, replace high with higher, low with lower, or discard
   if new run distance is zero, return runs' pointer to that new run  
*/
EdgeRun* CcCurves::addEdgeRun(OpEdge* edge, OpEdge* opp, EdgeMatch match) {
	EdgeRun run;
	run.set(edge, opp, match);
	if (OpMath::IsNaN(run.oppDist))
		return nullptr;
	// skip adding new run if it is between existing run
	// otherwise, find run before, and its index
	// binary search
	int lo = 0;  // position in runs where new run is to be inserted
	if (!runs.empty()) {
		OP_DEBUG_VALIDATE_CODE(debugValidate());
		int hi = (int) runs.size() - 1;
		do {
			int mid = (lo + hi) / 2;
			EdgeRun& test = runs[mid];
			if (test.edgePtT.t == run.edgePtT.t)
				return nullptr;
			if (test.edgePtT.t < run.edgePtT.t)
				lo = mid + 1;
			else
				hi = mid - 1;
		} while (lo <= hi);
	}
	// binary search first, then add / modify run as needed
	if (0 < lo && lo < (int) runs.size()) {
		EdgeRun& runStart = runs[lo - 1];
		EdgeRun& runEnd = runs[lo];
		OP_ASSERT(runStart.edgePtT.t < run.edgePtT.t && run.edgePtT.t < runEnd.edgePtT.t);
		if (runStart.oppDist * runEnd.oppDist >= 0) {  // true if start, end have same sign
			if (OpMath::Between(runStart.oppDist, run.oppDist, runEnd.oppDist)) {
				OP_DEBUG_CODE(++runStart.debugBetween);
				return nullptr;
			}
			bool startSmaller = fabs(runStart.oppDist) < fabs(runEnd.oppDist);
			bool runSmaller = fabs(run.oppDist) < fabs(runs[lo - startSmaller].oppDist); 
			OP_DEBUG_CODE(run.debugBetween += runs[lo - (runSmaller == startSmaller)].debugBetween);
			EdgeRun* runPtr = &runs[lo - (runSmaller == startSmaller)];
			*runPtr = run;
			return run.oppDist ? nullptr : runPtr;
		}
		// whichever of start or end has same sign as run
		//  if start or end is between run dist and following dist, replace start or end
		bool runSign = run.oppDist * runStart.oppDist > 0;  // check if signs are the same
		if (runSign ? lo > 1 : lo + 1 < (int) runs.size()) {
			EdgeRun& runNear = runSign ? runStart : runEnd;
			EdgeRun& runFar = runSign ? runs[lo - 2] : runs[lo + 1];
			if (runNear.oppDist * runFar.oppDist > 0  // signs are the same
					&& OpMath::Between(runFar.oppDist, runNear.oppDist, run.oppDist)) {
				OP_DEBUG_CODE(run.debugBetween += runNear.debugBetween);
				runNear = run;
				return run.oppDist ? nullptr : &runNear;
			}
		}
	} else if (runs.size() > 1) {
		if (0 == lo) {
			EdgeRun& runStart = runs[0];
			if (runStart.oppDist * run.oppDist >= 0) {
				EdgeRun& runEnd = runs[1];
				if (runStart.oppDist * runEnd.oppDist >= 0
						&& OpMath::Between(run.oppDist, runStart.oppDist, runEnd.oppDist)) {
					OP_DEBUG_CODE(run.debugBetween += runStart.debugBetween);
					runStart = run;
					return run.oppDist ? nullptr : &runStart;
				}
			}
		} else {
			OP_ASSERT(lo == (int) runs.size());
			EdgeRun& runStart = runs[runs.size() - 2];
			if (runStart.oppDist * run.oppDist >= 0) {
				EdgeRun& runEnd = runs.back();
				if (runStart.oppDist * runEnd.oppDist >= 0
						&& OpMath::Between(runStart.oppDist, runEnd.oppDist, run.oppDist)) {
					OP_DEBUG_CODE(run.debugBetween += runEnd.debugBetween);
					runEnd = run;
					return run.oppDist ? nullptr : &runEnd;
				}
			}
		}
	}
	runs.insert(runs.begin() + lo, run);  // runs are sorted by edge pt t
	OP_DEBUG_VALIDATE_CODE(debugValidate());
#if 0
static int dump_count;
	OpDebugOut(STR(++dump_count) + ": ");
	dmp(runs);
	OpDebugOut("\n");
#endif
	return run.oppDist ? nullptr : &runs.front() + lo;
}

// note that this doesn't create an opportunity for a zero distance; it uses edge run as temporary
bool CcCurves::checkMid(OpSegment* seg, OpSegment* opp, size_t index) {
	EdgeRun& eS = runs[index];
	EdgeRun& eE = runs[index + 1];
	EdgeRun mid;
	float midT = OpMath::Average(eS.edgePtT.t, eE.edgePtT.t);
	mid.edgePtT = seg->c.ptTAtT(midT);
	mid.oppPtT = seg->distance(mid.edgePtT, opp);
	mid.setOppDist(seg);
	// ok if start oppDist > end.oppDist
	return OpMath::InUnsorted(eS.oppDist, mid.oppDist, eE.oppDist, seg->threshold().length());
}

void CcCurves::checkSigns(OpSegment* opp) {
	if (c.empty())
		return;
	OpSegment* seg = c[0]->segment;
	for (OpEdge* curve : c) {
		if (!(curve->startDist * curve->endDist < 0))  // allow nan
			continue;
		if (OpMath::IsNaN(curve->startOpp.t) || OpMath::IsNaN(curve->endOpp.t))
			continue;
		// use distance to weight middle of binary search
		float startT = curve->startT;
		float endT = curve->endT;
		float startDist = curve->startDist;
		float endDist = curve->endDist;
		EdgeRun run;
		do {
			float rangeT = endT - startT;
			float absStart = fabs(startDist);
			float absEnd = fabs(endDist);
			float rangeDist = absStart + absEnd;
			float weight, midT;
			if (absStart < absEnd) {
				weight = absStart / rangeDist;
				midT = rangeT * weight + startT;
				if (startT == midT)
					break;
			} else {
				weight = absEnd / rangeDist;
				midT = endT - rangeT * weight;
				if (endT == midT)
					break;
			}

			run.edgePtT = seg->c.ptTAtT(midT);
			run.oppPtT = seg->distance(run.edgePtT, opp);
			run.setOppDist(seg);
			float error = run.oppDist;
			if (0 == error) 
				break;
			if (OpMath::IsNaN(error)) 
				break;
			// !!! when estimated zero point misses, use the error (distance) to make a better guess
			// adjust the end further away, to cut down on the possible t range as much as possible

			// !!! if weight < .5, run pt is closer to start: move end close to guess

			if (startDist * error < 0) {	// guess too large
				endDist = error;
				endT = midT;
			} else {
				OP_ASSERT(endDist * error < 0);
				startDist = error;
				startT = midT;
			}
		} while (startT + OpEpsilon < endT);
		if (OpMath::IsNaN(run.oppDist)) 
			continue;
		OpPoint midPt = OpPoint::Average(run.edgePtT.pt, run.oppPtT.pt);
		float edgeLen = (run.edgePtT.pt - midPt).length();
		float oppLen = (run.oppPtT.pt - midPt).length();
		float edgeFactor = edgeLen / seg->thresholdLength();
		float oppFactor = oppLen / seg->thresholdLength();
		if (edgeFactor > 1 || oppFactor > 1)
			continue;
		run.edgePtT.pt = midPt;
		run.oppPtT.pt = midPt;
		if (seg->sects.contains(run.edgePtT, opp))
			continue;
		if (opp->sects.contains(run.oppPtT, seg))
			continue;
		OpIntersection* sect = seg->addSegBase(run.edgePtT  OP_LINE_FILE_PARAMS(opp));
		OpIntersection* oSect = opp->addSegBase(run.oppPtT  OP_LINE_FILE_PARAMS(seg));
		OP_ASSERT(sect);
		OP_ASSERT(oSect);
		sect->pair(oSect);
	}
}

void CcCurves::clear() {
	for (auto edge : c) {
		edge->hulls.clear();
		edge->ccOverlaps = false;
	}
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

void CcCurves::initialEdgeRun(OpEdge* edge, OpEdge* opp) {
	OP_ASSERT(runs.empty());
	for (EdgeMatch match : { EdgeMatch::start, EdgeMatch::end } ) {
		EdgeRun run;
		run.set(edge, opp, match);
		if (OpMath::IsNaN(run.oppDist))
			continue;
		runs.push_back(run);
	}
}

// returns true if caller should split edges down the middle
bool CcCurves::lopSided(size_t priorCount, float maxBias) const {
	if (c.size() < 2 + priorCount) 
		return true;
	float major = 0;
	bool doubleMajor = false;
	for (OpEdge* test : c) {
		float range = test->endT - test->startT;
		if (major < range) {
			major = range;
			doubleMajor = false;
		} else if (major == range)
			doubleMajor = true;
	}
	if (doubleMajor)
		return false;
	float minor = 0;
	for (OpEdge* test : c) {
		float range = test->endT - test->startT;
		if (minor < range && range < major)
			minor = range;
	}
	return minor < maxBias * major;
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

// snip out the curve 16 units to the side of the intersection point, to prevent another closeby
// intersection from also getting recorded. The units maybe t values, or may be x/y distances.
void CcCurves::snipAndGo(const OpSegment* segment, const OpPtT& ptT, OpPoint oppPt, OpEdge* opp) {
	// snip distance must be large enough to differ in x/y and in t
	CutRangeT tRange = segment->c.cutRange(ptT, oppPt, 0, 1);
	OP_ASSERT(tRange.lo.t < tRange.hi.t);
	// remove part or all of edges that overlap tRange
	deleted.push_back(tRange);
	snipRange(segment, tRange.lo, tRange.hi, opp);
}

void CcCurves::snipRange(const OpSegment* segment, const OpPtT& lo, const OpPtT& hi, OpEdge* opp) {
	CcCurves snips;
	OpContext* context = segment->contour->context;
	auto addSnip = [context](const OpEdge* edge, const OpPtT& start, const OpPtT& end) {
		void* block = context->allocateEdge(context->ccStorage);
		OpEdge* newE = new(block) OpEdge(edge, start, end  OP_LINE_FILE_PARGS());
		newE->ccInit(true);
		return newE;
	};
	for (OpEdge* edge : c) {
		if (edge->startT >= hi.t || edge->endT <= lo.t) {
			OP_ASSERT(!edge->disabled);
			snips.c.push_back(edge);
			continue;
		}
		OpVector threshold = context->threshold();
		if (edge->startT < lo.t && !edge->start().isNearly(lo, threshold)) {
			OpEdge* snipE = addSnip(edge, edge->start(), lo);
			snipE->ccStart = edge->ccStart;
			snipE->ccSmall = edge->ccSmall;
			snipE->ccEnd = true;
			addEdgeRun(snipE, opp, EdgeMatch::end);
			OP_ASSERT(!snipE->disabled);
			snips.c.push_back(snipE);
		}
		if (edge->endT > hi.t && !hi.isNearly(edge->end(), threshold)) {
			OpEdge* snipS = addSnip(edge, hi, edge->end());
			snipS->ccStart = true;
			snipS->ccEnd = edge->ccEnd;
			snipS->ccLarge = edge->ccLarge;
			addEdgeRun(snipS, opp, EdgeMatch::start);
			OP_ASSERT(!snipS->disabled);
			snips.c.push_back(snipS);
		}
	}
	snips.c.swap(c);
}

OpCurveCurve::OpCurveCurve(OpSegment* s, OpSegment* o)
	: context(s->contour->context)
	, seg(s)
	, opp(o)
	, depth(0)
	, uniqueLimits_impl(-1)
	, unsplitables(0)
	, boundedEdgeFailed(false)
	, overlap(false)
	, rotateFailed(false)
	, sectResult(false)
	, lastDepthReduced(false)
	, foundGap(false)
	, splitHullFail(false)
{
#if OP_DEBUG_DUMP
	++debugCall;
	debugLocalCall = debugCall;  // copied so value is visible in debugger
	context->debugCurveCurve = this;
#endif
//	contours->reuse(contours->ccStorage);  // !!! consider adding starting block of edges to context
	PathOpsV0Lib::ContextCallbacks& cb = context->contextCallbacks;
	maxSignSwap = cb.maxSignSwapFuncPtr ? cb.maxSignSwapFuncPtr(s->c.c, o->c.c) : 131072.f;
	maxSignSwap *= context->aliases.thresholdLength;
	float maxOverlap = cb.maxOverlapFuncPtr ? cb.maxOverlapFuncPtr(s->c.c, o->c.c) : 1.5f;
	maxSplitBias = cb.maxSplitBiasFuncPtr ? cb.maxSplitBiasFuncPtr(s->c.c, o->c.c) : .1f;  // guess
	maxCheckSplit = cb.maxCheckSplitFuncPtr ? cb.maxCheckSplitFuncPtr(s->c.c, o->c.c) : 16;
	maxDeep = cb.maxDeepFuncPtr ? cb.maxDeepFuncPtr(s->c.c, o->c.c) : 64;
	maxShallow = cb.maxShallowFuncPtr ? cb.maxShallowFuncPtr(s->c.c, o->c.c) : 8;
	maxSplits = cb.maxSplitsFuncPtr ? cb.maxSplitsFuncPtr(s->c.c, o->c.c) : 8;
	maxBoundedEdge = cb.maxBoundedEdgeFuncPtr ? cb.maxBoundedEdgeFuncPtr(s->c.c) : 2.0f;
	maxBoundedT = cb.maxBoundedTFuncPtr ? cb.maxBoundedTFuncPtr(s->c.c) : 8.f;
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
	// !!! undone: if ends of segments already touch, exclude from made edge
	// take the bounds of both segments
	// limit the edge t range to the opposite bounding rectangle
	OpPointBounds curveSect = seg->ptBounds.intersect(opp->ptBounds);
	OpVector sectWH = curveSect.widthHeight() * maxOverlap;
	if (sectWH.dx < 0 || sectWH.dy < 0)
		return;
	auto makeEdge = [sectWH, &curveSect, this](OpSegment* segment) {
		OpVector segWH = segment->ptBounds.widthHeight();
		OpEdge* parent;
		if (sectWH < segWH && !sectWH.axisAligned())
			parent = boundedEdge(segment, curveSect, matchRev.match  OP_LINE_FILE_PARGS());
		else {
			segment->makeEdge(OP_LINE_FILE_NPARGS());
			parent = &segment->edges.back();
		}
		return parent;
	};
	if (!(parentEdge = makeEdge(seg)))
		return;
	parentEdge->ccStart = parentEdge->ccSmall = smallTFound;
	parentEdge->ccEnd = parentEdge->ccLarge = largeTFound;
	OP_ASSERT(!parentEdge->disabled);
	edgeCurves.oppCurves = &oppCurves;
	edgeCurves.c.push_back(parentEdge);
	// end of seg; start of opp
	if (!(parentOpp = makeEdge(opp)))
		return;
	overlap = true;
	if (matchRev.reversed)
		std::swap(smallTFound, largeTFound);
	parentOpp->ccStart = parentOpp->ccSmall = smallTFound;
	parentOpp->ccEnd = parentOpp->ccLarge = largeTFound;
	OP_ASSERT(!parentOpp->disabled);
	oppCurves.oppCurves = &edgeCurves;
	oppCurves.c.push_back(parentOpp);
	edgeCurves.initialEdgeRun(parentEdge, parentOpp);
	oppCurves.initialEdgeRun(parentOpp, parentEdge);
	// end of common function
	if (smallTFound) {
		limits.front().parentEdge = parentEdge;
		limits.front().parentOpp = parentOpp;
	}
	if (largeTFound) {
		limits.back().parentEdge = parentEdge;
		limits.back().parentOpp = parentOpp;
	}
}

EdgeRun* OpCurveCurve::addEdgeRun(OpEdge* edge, CurveRef curveRef, EdgeMatch match) {
	CcCurves& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	return curves.addEdgeRun(edge, CurveRef::edge == curveRef ? parentOpp : parentEdge, match);
}

void OpCurveCurve::addIntersection(OpEdge* edge, OpEdge* oppEdge) {
	recordSect(edge, oppEdge, edge->start(), oppEdge->start()  
			OP_LINE_FILE_PARGS());
	snipEdge = edge->start();
	snipOpp = oppEdge->start();
}

bool OpCurveCurve::addUnsectable(const OpPtT& edgeStart, const OpPtT& edgeEnd,
		const OpPtT& oppStart, const OpPtT& oppEnd) {
	OpVector threshold = context->threshold();
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
		else {
			sect->setUnsect(ie.id, ie.matchEnds);
			sect->ccUnsectable = true;
		}
	};
	auto addSect = [isCoin, idEnds](OpSegment* segs, OpSegment* opps, const OpPtT& start, 
			IsOpp isOpp  OP_LINE_FILE_ARGS()) {
		IdEnds ie = idEnds(isOpp);
		OpIntersection* result;
		if (IsCoin::yes == isCoin)
			result = segs->addCoin(start, ie.id, ie.matchEnds, opps  OP_LINE_FILE_CARGS());
		else {
			result = segs->addUnsectable(start, ie.id, ie.matchEnds, opps  OP_LINE_FILE_CARGS());
			result->ccUnsectable = true;
		}
		result->ccSect = true;
		segs->sects.hasCCSects = true;
		return result;
	};
	auto addPair = [this, addSect, setSect](SectDuo sPair, const OpPtT& ePtT, const OpPtT& oPtT) {
		if (sPair.s) {
			setSect(sPair.s, IsOpp::no);
			setSect(sPair.o, IsOpp::yes);
		} else {
			sPair.s = addSect(seg, opp, ePtT, IsOpp::no  OP_LINE_FILE_PARGS());
			sPair.o = addSect(opp, seg, oPtT, IsOpp::yes  OP_LINE_FILE_PARGS());
			sPair.s->pair(sPair.o);
		}
	};
	addPair(sect1, eStart, oStart);
	match.match = MatchEnds::end;
	addPair(sect2, eEnd, oEnd);
	return true; 
}

OpEdge* OpCurveCurve::boundedEdge(OpSegment* segm, const OpPointBounds& sectBounds, 
		MatchEnds match  OP_LINE_FILE_ARGS()) {
	// while segment crosses at most two sect bounds' sides, all four must be checked
	const OpCurve& c = segm->c;
	// This could use points at found t; but error in finding axis roots can cause points to miss 
	// bounds. Instead, compute t range, and at the end, outset it to account for error; then
	// compute the points from the liberal t values
	float minT = sectBounds.contains(segm->c.firstPt()) ? 0 : OpNaN;
	float maxT = sectBounds.contains(segm->c.lastPt()) ? 1 : OpNaN;
	auto saveBest = [&minT, &maxT](float root) {
		if (!(root >= minT)) // works if min t is nan
			minT = root;
		if (!(root <= maxT)) // works if max t is nan
			maxT = root;
	};
	auto saveRoots = [c, sectBounds, saveBest, this](OpRoots roots, XyChoice choice) {
		if (RootFail::rootIsNaN == roots.fail) {
			boundedEdgeFailed = true;
			return;
		}
		if (!roots.count())
			return;
		OpVector threshold = context->threshold() * maxBoundedEdge;
		for (float root : roots.roots) {
			OpPoint pt = c.ptAtT(root);
			if (XyChoice::inX == choice) {
				if ((sectBounds.left <= pt.x && pt.x <= sectBounds.right)
						|| OpMath::Equal(sectBounds.left, pt.x, threshold.dx)
						|| OpMath::Equal(sectBounds.right, pt.x, threshold.dx))
					saveBest(root);
			} else if ((sectBounds.top <= pt.y && pt.y <= sectBounds.bottom)
						|| OpMath::Equal(sectBounds.top, pt.y, threshold.dy)
						|| OpMath::Equal(sectBounds.bottom, pt.y, threshold.dy))
				saveBest(root);
		}
	};
	if (segm->ptBounds.left < sectBounds.left) {
		OP_ASSERT(segm->ptBounds.right >= sectBounds.left);
		saveRoots(c.axisRayHit(Axis::vertical, sectBounds.left), XyChoice::inY);
	}
	if (segm->ptBounds.top < sectBounds.top) {
		OP_ASSERT(segm->ptBounds.bottom >= sectBounds.top);
		saveRoots(c.axisRayHit(Axis::horizontal, sectBounds.top), XyChoice::inX);
	}
	if (segm->ptBounds.right > sectBounds.right) {
		OP_ASSERT(segm->ptBounds.left <= sectBounds.right);
		saveRoots(c.axisRayHit(Axis::vertical, sectBounds.right), XyChoice::inY);
	}
	if (segm->ptBounds.bottom > sectBounds.bottom) {
		OP_ASSERT(segm->ptBounds.top <= sectBounds.bottom);
		saveRoots(c.axisRayHit(Axis::horizontal, sectBounds.bottom), XyChoice::inX);
	}
	if (boundedEdgeFailed)
		return nullptr;
//	if (!validRoots)
//		return nullptr;
#if 0
	//  debugger function determines error required for axisRayHit to 
	//  move from given result to result on the other side of the target ray
	//  note: can't call all the time: if edges do not intersect, assert may be triggered 
	OP_DEBUG_CODE(debugBoundedEdge(segm, sectBounds, minT, "in"));
	OP_DEBUG_CODE(debugBoundedEdge(segm, sectBounds, maxT, "ax"));
#endif
	minT = std::max(0.f, minT - OpEpsilon * maxBoundedT);
	maxT = std::min(1.f, maxT + OpEpsilon * maxBoundedT);
	if (!(minT < maxT))  // condition returns null if either is nan
		return nullptr;
	// outset t values account for error introduced in caller's root finding
	OpPtT minPtT = segm->c.ptTAtT(minT);
	OpPtT maxPtT = segm->c.ptTAtT(maxT);
	if (minPtT.pt.isNearly(maxPtT.pt, context->threshold()))
		return nullptr;
	void* block = context->allocateEdge(context->ccStorage);
	OpEdge* result = new(block) OpEdge(segm, minPtT, maxPtT  OP_LINE_FILE_PARGS());
	result->ccInit(false);
	return result;
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

bool OpCurveCurve::checkSect() {
	for (auto edgePtr : edgeCurves.c) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		OpPtT edgeStart = edge.start();
		OpPtT edgeEnd = edge.end();
		OpVector threshold = context->threshold();
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
	do {
		// check edge cases (e.g., checkPtT.pt == oCurve end) first
		for (OpEdge* oCurve : oCurves) {
			if (oCurve->ccOverlaps && oCurve->bounds.contains(checkPtT.pt))
				return original != checkPtT;
		}
		// check for gap between original and edge list, and between edges in edge list
		const OpEdge& oEdge = CurveRef::edge == which ? *parentOpp : *parentEdge;
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
	} while (++attempts < maxCheckSplit);
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
			for (const HullSect& hull : eCurve->hulls.h) {
				if (hull.opp == oCurve) {
					ePt = hull.sect;
					hullsIntersect = true;
					break;
				}
			}
			OpPtT oPt = oCurve->center;
			for (const HullSect& hull : oCurve->hulls.h) {
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
	// !!! testQuads5721199 segments 2 and 5 share (0, 0) but iterate depth to 24 to see if they
	// intersect a second time. Not sure what to do...
	for (depth = 1; depth < maxDeep; ++depth) {
		OP_ASSERT(debugShowImage(true));
		bool snipEm = false;
		if (!setOverlaps())
			return SectFound::fail;
		if (checkForGaps() || (splitMid && !endsOverlap()))
			splitMid = false;
		int edgeOverlaps = edgeCurves.overlaps();
		int oppOverlaps = oppCurves.overlaps();
		if (!edgeOverlaps || !oppOverlaps) {
			if (depth > maxShallow)  // default may not be correct (depth is 9 for testQuads3993265)
				return SectFound::noOverlapDeep;
			return limits.size() ? SectFound::add : SectFound::no;
		}
		if (edgeOverlaps >= maxSplits || oppOverlaps >= maxSplits) {
			// find crossing for every curve that has an opposite distance that changes sign
			edgeCurves.checkSigns(opp);
			oppCurves.checkSigns(seg);
			return SectFound::maxOverlaps;
		}
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
			// if edge curves and opp curves have unsplittables, add limit and remove both
			if (1 < unsplitables)
				checkUnsplitables();
			snipEm = setSnipFromLimits(limitCount);
		}
		if (snipEm) {
			edgeCurves.snipAndGo(seg, snipEdge, snipOpp.pt, parentOpp);
			oppCurves.snipAndGo(opp, snipOpp, snipEdge.pt, parentEdge);
		}
	}
	return SectFound::fail;  // soft fail (ignored)
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
		OpIntersection* sect = seg->addSegBase(limit.seg  OP_LINE_FILE_PARAMS(opp));
		OpIntersection* oSect = opp->addSegBase(limit.opp  OP_LINE_FILE_PARAMS(seg));
		sect->pair(oSect);
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
		OpPtT oppTest = seg->distance(midPtT, opp);
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
#if 0
	// !!! do this earlier when max splits is exceeded
	// check edge runs for ranges, not yet recorded, where start and end distance change sign
	auto checkRuns = [this](CcCurves& curves, CurveRef curveRef) {
		for (OpEdge* curve : curves.c) {
			if (!(curve->startDist * curve->endDist < 0))  // allow nan
				continue;
			for (FoundLimits& limit : limits) {
				float limitT = CurveRef::edge == curveRef ? limit.seg.t : limit.opp.t;
				if (curve->startT <= limitT && limitT <= curve->endT)
					goto next;
			}
			addUnsectable(curve->start(), curve->end(), curve->startOpp, curve->endOpp);
	next:
			;
		}
	};
	checkRuns(edgeCurves, CurveRef::edge);
	checkRuns(oppCurves, CurveRef::opp);
#endif
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
	OpVector threshold = context->threshold();
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

bool OpCurveCurve::alreadyInLimits(const OpEdge* edge, const OpEdge* oEdge, 
		const OpPtT& edgePtT, const OpPtT& oppPtT) {
	for (FoundLimits& limit : limits) {
#if 0  // !!! this test is undoubtably required, but as written is too broad, dropping valid
	   //     intersections (e.g., loop8452, segments 8 & 4 intersect multiple times)
		if (limit.parentEdge == edge && limit.parentOpp == oEdge)
			return true;  // reverse case already recorded
#endif
		if (OpMath::EqualT(limit.seg.t, edgePtT.t))
			return true;  // already recorded
		OpPointBounds bounds(limit.seg.pt, limit.opp.pt);
		if (bounds.contains(edgePtT.pt))
			return true;
		if (bounds.contains(oppPtT.pt))
			return true;
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
	if (curves.deletedT(edgePtT.t))
		return true;
#endif
	return false;  // caller adds to limit
}

bool OpCurveCurve::betweenLimits(OpSegment* segment, float lo, float hi) {
	for (FoundLimits& limit : limits) {
// !!! this test may be still required: but, trying to remove edge from edge runs
//		if (limit.parentEdge == edge && limit.parentOpp == oEdge)
//			return true;  // reverse case already recorded
		if (OpMath::Between(lo, 
				segment == limit.parentEdge->segment ? limit.opp.t : limit.seg.t, hi))
			return true;  // already recorded
	}
	return false;
}

// defer meet in the middle stuff until all intersections are found
void OpCurveCurve::recordSect(OpEdge* edge, OpEdge* oEdge, const OpPtT& edgePtT, const OpPtT& oppPtT
			OP_LINE_FILE_ARGS()) {
	if (alreadyInLimits(edge, oEdge, edgePtT, oppPtT))
		return;
	FoundLimits newLimit { edge, oEdge, edgePtT, oppPtT, false, false  OP_LINE_FILE_SCALLER() };
	limits.push_back(std::move(newLimit));
	uniqueLimits_impl = -1;
}

// remove edges that do not change distance sign in run

bool OpCurveCurve::reduceDistFlipped() {
	CcCurves edgeSplits, oppSplits;
	CcCurves* curves = &edgeCurves;
	OpSegment* segment = seg;
	OpSegment* oSegment = opp;
	bool swap = false;
	do {
		EdgeRun* lower = nullptr;
		EdgeRun* upper = nullptr;
		size_t lodex = OpMax;
		size_t hidex = OpMax;
		auto keepRun = [&segment, &lower, &upper, this](CcCurves& splits) {
			void* block = context->allocateEdge(context->ccStorage);
			OpEdge* split = new(block) OpEdge(segment, lower->edgePtT.t, 
					upper->edgePtT.t  OP_LINE_FILE_PARGS());
			split->ccInit(true);
			OP_ASSERT(!split->disabled);
			splits.c.push_back(split);
		};
		auto markByZero = [&segment, &oSegment, &lodex, &hidex, &lower, &upper, &curves]() {
			lower = &curves->runs[lodex];
			upper = OpMax == hidex ? nullptr : &curves->runs[hidex];
			// only disable prior if the mid point is equal to or smaller than its distance
			// mark prior two edges to disable sign compare
			if (lodex > 1 && curves->checkMid(segment, oSegment, lodex - 1)) {
				curves->runs[lodex - 1].byZero = true;
			}
			if (OpMax == hidex)
				hidex = lodex;
			// mark next two edges to disable sign compare
			if (hidex + 2 < curves->runs.size() && curves->checkMid(segment, oSegment, hidex))
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
		CcCurves& splits = swap ? oppSplits : edgeSplits;
		for (auto& run : curves->runs) {
			upper = &run;
			if (lower && lower->oppDist * run.oppDist < 0 // switches sides
					&& fabsf(lower->oppDist) > OpEpsilon && fabsf(run.oppDist) > OpEpsilon
					&& !lower->byZero && !run.byZero
			#if 0  // !!! add test and rationale why this is required
				   // since runs are kept after edges are trimmed or divided, below logic is
				   // insufficient. Notably, comparing runEdge or runEdge values isn't meaningful
					&& (lower->runEdge == upper->runEdge  // skip if gap between last run and this run
					|| lower->runEdge->endT == upper->runEdge->startT)
			#endif		
					)
				keepRun(splits);
			lower = upper;
		}
		// additionally, keep curves if their start and end distances change signs
		std::vector<OpEdge*> toAdd;
		for (OpEdge* curve : curves->c) {
			if (!(curve->startDist * curve->endDist < 0))  // allow nan
				continue;
			for (OpEdge* split : splits.c) {
				if ((split->startT <= curve->startT && curve->startT < split->endT)
						|| (split->startT < curve->endT && curve->endT <= split->endT))
					goto nextCurve;
			}
			toAdd.push_back(curve);
	nextCurve:
			;
		}
		if (!toAdd.empty()) {
			splits.c.insert(splits.c.end(), toAdd.begin(), toAdd.end());
			std::sort(splits.c.begin(), splits.c.end(), [](OpEdge* a, OpEdge* b) {
				return a->startT < b->startT;
			});
		}
		curves = &oppCurves;
		std::swap(segment, oSegment);
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
		FoundLimits lowerLimit { parentEdge, parentOpp, run->edgePtT, run->oppPtT, 
				run->fromFoundT, false  OP_LINE_FILE_STRUCT() };
		if (swap) {
			std::swap(lowerLimit.parentEdge, lowerLimit.parentOpp);
			std::swap(lowerLimit.seg, lowerLimit.opp);
		}
		if (!alreadyInLimits(lowerLimit.parentEdge, lowerLimit.parentOpp, 
				lowerLimit.seg, lowerLimit.opp))
			limits.push_back(std::move(lowerLimit));
	};
	EdgeRun* lastUpper;
	CcCurves* curves = &edgeCurves;
	CcCurves* oCurves = &oppCurves;
	OpSegment* segment = seg;
	OpSegment* oSegment = opp;
	auto addLimit = [this, &segment, &lastUpper, &curves, &oCurves, addIfNew](
			EdgeRun* lower, EdgeRun* upper) {
//		OP_ASSERT(!upper || lower->runEdge->segment == upper->runEdge->segment);
		if (upper && betweenLimits(segment, lower->oppPtT.t, upper->oppPtT.t))
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
	auto markByZero = [&segment, &oSegment, &lodex, &hidex, &curves, addLimit]() {
			EdgeRun* lower = &curves->runs[lodex];
			EdgeRun* upper = OpMax == hidex ? nullptr : &curves->runs[hidex];
			addLimit(lower, upper);
			// only disable prior if the mid point is equal to or smaller than its distance
			// mark prior two edges to disable sign compare
			if (lodex > 1 && curves->checkMid(segment, oSegment, lodex - 1)) {
				curves->runs[lodex - 1].byZero = true;
			}
			if (OpMax == hidex)
				hidex = lodex;
			// mark next two edges to disable sign compare
			if (hidex + 2 < curves->runs.size() && curves->checkMid(segment, oSegment, hidex))
				curves->runs[hidex + 1].byZero = true;
			lodex = hidex = OpMax;
	};
	// scan both edge curves and opp curves
	do {
		lastUpper = nullptr;
		lodex = OpMax;
		hidex = OpMax;
		for (size_t index = 0; index < curves->runs.size(); ++index) {
			EdgeRun& run = curves->runs[index];
			if (0 == run.oppDist)
				(OpMax == lodex ? lodex : hidex) = index;
			else if (OpMax != lodex)
				markByZero();
		}
		if (OpMax != lodex)
			markByZero();
		// second pass: add edges with distances that switch signs but are not adjacent to very small
		EdgeRun* lower = nullptr;
		for (auto& run : curves->runs) {
			if (lower && lower->oppDist * run.oppDist < 0 
					&& lower->oppDist && run.oppDist
				// !!! very large dist sign swaps should be ignored
				//     need to determine what this range should be
					&& fabsf(lower->oppDist) < maxSignSwap 
					&& fabsf(run.oppDist) < maxSignSwap
					&& !lower->byZero && !run.byZero) { // switches sides
				// check if dot of opposite normal at oppPtT.t switches signs (loop146475)
				OpVector lowerOppNormal = oSegment->c.normal(lower->oppPtT.t).normalize();
				OpVector upperOppNormal = oSegment->c.normal(run.oppPtT.t).normalize();
				OpVector lowerDistV = (lower->oppPtT.pt - lower->edgePtT.pt).normalize();
				OpVector upperDistV = (run.oppPtT.pt - run.edgePtT.pt).normalize();
				//!!! do the vectors need to be normalized first? (probably)
				float lowerX = lowerOppNormal.dot(lowerDistV);
				float upperX = upperOppNormal.dot(upperDistV);
				if (lowerX * upperX < 0)
					addLimit(lower, &run);
			}
			lower = &run;
		}
		curves = &oppCurves;
		std::swap(segment, oSegment);
	} while ((swap = !swap));
	return limits.size() == oldLimits ? SectFound::no : SectFound::add;
}

// finds intersections of opp edge's hull with edge, and stores them in edge's hulls
// returns true if found intersection is true curve curve intersection (or at least, close enough)
void OpCurveCurve::setHullSects(OpEdge& edge, OpEdge& oppEdge, CurveRef curveRef) {
	int ptCount = oppEdge.curve.pointCount();
	LinePts oppPts;
	oppPts.pts[1] = oppEdge.curve.firstPt();
	OpVector threshold = context->threshold();
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
			if (edge.hulls.add(sectPtT, context->threshold(), nullptr, OpNaN, sectType, &oppEdge)) {
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

bool OpCurveCurve::setSnipFromLimits(size_t oldCount) {
	if (oldCount >= limits.size())
		return false;
	FoundLimits limit = limits[oldCount];
	snipEdge = limit.seg;
	snipOpp = limit.opp;
	return true;
}

bool OpCurveCurve::setOverlaps() {
	edgeCurves.clear();
	oppCurves.clear();
	for (auto edgePtr : edgeCurves.c) {
		auto& edge = *edgePtr;
		for (auto oppPtr : oppCurves.c) {
			auto& oppEdge = *oppPtr;
			// !!! used to be close bounds; simple bounds required for testQuads1883885
			if (depth > 1 && !edge.bounds.intersects(oppEdge.bounds))
				continue;
			bool sharesPoint = (edge.ccStart || edge.ccEnd) && (oppEdge.ccStart || oppEdge.ccEnd);
			// if bounds have common edge only and already share point, they don't intersect
			if (sharesPoint) {
				// !!! can one have no area (e.g. horz or vert) and the other not?
				if (edge.bounds.hasArea() && oppEdge.bounds.hasArea() &&
						!edge.bounds.areaOverlaps(oppEdge.bounds))
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

bool OpCurveCurve::splitDownTheMiddle(const OpEdge& edge, CurveRef curveRef, CcCurves& splits) {
	// !!!? while edgeMid is in a deleted bounds, bump edgeMidT
	//      (this isn't necessarily in the intersection hull)
	//      wait until this is necessary to make it work
	// checkSplit(edge.startT, edge.endT, which, edgeMid);
	const OpPtT& edgeMid = edge.center;
	if (OpMath::EqualT(edge.startT, edgeMid.t))
		return false;
	if (OpMath::EqualT(edgeMid.t, edge.endT))
		return false;
	if (edge.centerless)
		return false;
	OP_ASSERT(edge.startT < edgeMid.t);
	OP_ASSERT(edgeMid.t < edge.endT);
	CcCurves& curves = CurveRef::edge == curveRef ? edgeCurves : oppCurves;
	OpEdge* oppEdge = CurveRef::edge == curveRef ? parentOpp : parentEdge;
	void* blockL = context->allocateEdge(context->ccStorage);
	OpEdge* splitLeft = new(blockL) OpEdge(&edge, edgeMid, NewEdge::isLeft  OP_LINE_FILE_PARGS());
	if (!splitLeft->disabled) {
		splitLeft->ccInit(true);
		splitLeft->ccStart = edge.ccStart;
		splitLeft->ccSmall = edge.ccSmall;
		OP_ASSERT(!splitLeft->disabled);
		splits.c.push_back(splitLeft);
		curves.addEdgeRun(splitLeft, oppEdge, EdgeMatch::end);
	}
	void* blockR = context->allocateEdge(context->ccStorage);
	OpEdge* splitRight = new(blockR) OpEdge(&edge, edgeMid, NewEdge::isRight  OP_LINE_FILE_PARGS());
	if (!splitRight->disabled) {
		splitRight->ccInit(true);
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
	OpVector threshold = context->threshold();
	const CcCurves& curves = CurveRef::edge == which ? edgeCurves : oppCurves;
	OpSegment* segment = CurveRef::edge == which ? seg : opp;
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
			if (!splitDownTheMiddle(edge, which, splits))
				return false;
			continue;
		}
		if (2 <= hulls.h.size()) {  // see if hulls are close enough to define an intersection
			hulls.sort(smallTFound);
#if OP_DEBUG
			for (size_t index = 1; index < hulls.h.size(); ++index) {
				if (!hulls.debugSectCandidates((int) index, edge))
					continue;
				OpDebugOut("!!! splitHulls fail:"  OP_DEBUG_CODE(+ context->debugData.testname)
					+ std::string("\n"));
			}
#endif
		}
		OP_DEBUG_VALIDATE_CODE(hulls.debugValidate());
		hulls.add(edge.start(), threshold, &edge.startOpp, edge.startDist, SectType::endHull);
		hulls.add(edge.end(), threshold, &edge.endOpp, edge.endDist, SectType::endHull);
		if (!hulls.nudgeDeleted(edge, *this, which))
			return false;  // soft error; give up on this intersection
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
			void* block = context->allocateEdge(context->ccStorage);
			OpEdge* split = new(block) OpEdge(segment, hullLo.sect.t, hullHi.sect.t  
					OP_LINE_FILE_PARGS());
			OP_DEBUG_CODE(split->debugParentID = edge.id);
			if (split->disabled)
				continue;
			split->ccInit(true);
			if (!OpMath::IsNaN(hullLo.oppPtT.t)) {
				split->startDist = hullLo.oppDist;
				split->startOpp = hullLo.oppPtT;
			}
			if (!OpMath::IsNaN(hullHi.oppPtT.t)) {
				split->endDist = hullHi.oppDist;
				split->endOpp = hullHi.oppPtT;
			}
			split->ccStart = edge.ccStart && edge.start().isNearly(split->start(), threshold);
			if (split->ccStart)
				split->ccSmall = edge.ccSmall;
			split->ccEnd = edge.ccEnd && edge.start().isNearly(split->end(), threshold);
			if (split->ccEnd)
				split->ccLarge = edge.ccLarge;
			if (split->endT < edge.endT) {
				EdgeRun* zeroDistance = addEdgeRun(split, which, EdgeMatch::end);
				if (zeroDistance) {
					if (CurveRef::edge == which)
						recordSect(parentEdge, parentOpp, zeroDistance->edgePtT,
								zeroDistance->oppPtT  OP_LINE_FILE_PARGS());
					else
						recordSect(parentEdge, parentOpp, zeroDistance->oppPtT,
								zeroDistance->edgePtT  OP_LINE_FILE_PARGS());
				}
			}
			splits.c.push_back(split);
		}
		if (splits.lopSided(splitCount, maxSplitBias)) {
			splits.c.resize(splitCount); // this may leave an edge was never used ... not the end of the world
			if (!splitDownTheMiddle(edge, which, splits))  // divide edge in two
				return false;
		}
		if (!splits.c.empty()) {
			OpEdge* edgeLo = splits.c[0];
			for (size_t index = 1; index < splits.c.size(); ++index) {
				OpEdge* edgeHi = splits.c[index];
				if (edgeLo->endT == edgeHi->startT) {
					if (OpMath::IsNaN(edgeHi->startDist)) {
						edgeHi->startDist = edgeLo->endDist;
						edgeHi->startOpp = edgeLo->endOpp;
					} else if (OpMath::IsNaN(edgeLo->endDist)) {
						edgeLo->endDist = edgeHi->startDist;
						edgeLo->endOpp = edgeHi->startOpp;
					}
				}
				edgeLo = edgeHi;
			}
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
	OpVector threshold = context->threshold();
	for (size_t index = 1; index < limits.size(); ++index) {
		const FoundLimits* limit = &limits[index];
		bool soClose = last->seg.isNearly(limit->seg, threshold);
		soClose |= last->opp.isNearly(limit->opp, threshold);
		float distSq = (limit->seg.pt - limit->opp.pt).lengthSquared();
		if (!soClose) {
			OpPtT midPtT = seg->c.ptTAtT(OpMath::Average(last->seg.t, limits[index].seg.t));
			OpPtT midOpp = seg->distance(midPtT, opp);
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
