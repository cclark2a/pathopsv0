// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpWinder.h"
#include <utility>

// !!! just fooling around; fix debug params if serious
// !!! not sure how to adapt this to store two (or more?) different close points
//     maybe save the least and greatest t values?
//     then, could test mid value to see if it is also on the curve or diverges further away...
// !!! wait until test case shows up
				// !!! save man in the middle?
void CcClose::save(OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT) {
	if (!lo.e.edge || lo.e.edge->start.t > edge.start.t)
		lo = CcSects(&edge, edgePtT, &opp, oppPtT  
			OP_LINE_FILE_PARAMS(SectReason::edgeCCHullPair, SectReason::oppCCHullPair));
	if (!hi.e.edge || hi.e.edge->start.t < edge.end.t)
		hi = CcSects(&edge, edgePtT, &opp, oppPtT  
			OP_LINE_FILE_PARAMS(SectReason::edgeCCHullPair, SectReason::oppCCHullPair));
	float dist = (edgePtT.pt - oppPtT.pt).lengthSquared();
	if (bestDist > dist) {
		bestDist = dist;
		best = CcSects(&edge, edgePtT, &opp, oppPtT  
				OP_LINE_FILE_PARAMS(SectReason::edgeCCHullPair, SectReason::oppCCHullPair));
	}
}

void CcCurves::clear() {
	for (auto edge : c) {
		edge->hulls.clear();
		edge->ccOverlaps = false;
	}
}

std::vector<TGap> CcCurves::findGaps() const {
	std::vector<TGap> gaps;
	OpPtT last = c[0]->start;
	for (auto edgePtr : c) {
		OP_ASSERT(edgePtr->start.t >= last.t);  // if not sorted, fix
		if (edgePtr->start.t != last.t)
			gaps.emplace_back(last, edgePtr->start);
		last = edgePtr->end;
	}
	return gaps;
}

void CcCurves::markToDelete(float loT, float hiT) {
	for (auto edgePtr : c) {
		if (edgePtr->start.t >= loT && hiT >= edgePtr->end.t)
			edgePtr->ccOverlaps = false;
	}
}

bool CcCurves::overlaps() const {
	for (auto curve : c)
		if (curve->ccOverlaps)
			return true;
	return false;
}

// snip out the curve 16 units to the side of the intersection point, to prevent another closeby
// intersection from also getting recorded. The units maybe t values, or may be x/y distances.
void CcCurves::snipAndGo(const OpSegment* segment, const OpPtT& ptT) {
	// snip distance must be large enough to differ in x/y and in t
	CutRangeT tRange = segment->c.cutRange(ptT, 0, 1);
	OP_ASSERT(tRange.lo.t < tRange.hi.t);
	// remove part or all of edges that overlap tRange
	snipRange(segment, tRange.lo, tRange.hi);
}

void CcCurves::snipOne(const OpSegment* segment, const OpPtT& loPtT, const OpPtT& hiPtT) {
	OpPtT loCutPt = segment->c.cut(loPtT, -1);
	OpPtT hiCutPt = segment->c.cut(hiPtT, 1);
	snipRange(segment, loCutPt, hiCutPt);
}

void CcCurves::snipRange(const OpSegment* segment, const OpPtT& lo, const OpPtT& hi) {
	CcCurves snips;
	OpContours* contours = segment->contour->contours;
	auto addSnip = [contours](const OpEdge* edge, const OpPtT& start, const OpPtT& end) {
		void* block = contours->allocateEdge(contours->ccStorage);
		OpEdge* newE = new(block) OpEdge(edge, start, end  OP_DEBUG_PARAMS(EDGE_MAKER(snip)));
		newE->ccOverlaps = true;
		return newE;
	};
	for (OpEdge* edge : c) {
		if (edge->start.t >= hi.t || edge->end.t <= lo.t) {
			snips.c.push_back(edge);
			continue;
		}
		if (edge->start.t < lo.t) {
			OpEdge* snipE = addSnip(edge, edge->start, lo);
			snipE->ccStart = edge->ccStart;
			snipE->ccSmall = edge->ccSmall;
			snipE->ccEnd = true;
			snips.c.push_back(snipE);
		}
		if (edge->end.t > hi.t) {
			OpEdge* snipS = addSnip(edge, hi, edge->end);
			snipS->ccStart = true;
			snipS->ccEnd = edge->ccEnd;
			snipS->ccLarge = edge->ccLarge;
			snips.c.push_back(snipS);
		}
	}
	std::swap(snips.c, c);
}


void FoundPtTs::setEnd(const OpSegment* o, const OpCurve& curve, float t) {
	seg = { t ? curve.lastPt() : curve.pts[0], t };
	opp = { o->c.ptTAtT(o->findValidT(0, 1, seg.pt)) };
}

OpCurveCurve::OpCurveCurve(OpSegment* seg, OpSegment* opp)
	: depth(0)
	, rotateFailed(false)
	, sectResult(false)
	, foundGap(false)
{
	OpContours* contours = seg->contour->contours;
#if OP_DEBUG_DUMP
	++debugCcCall;
	debugLocalCcCall = debugCcCall;  // copied so value is visible in debugger
	contours->debugCurveCurve = this;
#endif
	contours->reuse(contours->ccStorage);
	matchRev = seg->matchEnds(opp);
	smallTFound = MatchEnds::start & matchRev.match;
	largeTFound = MatchEnds::end & matchRev.match;
	splitMid = smallTFound || largeTFound;
    seg->edges.clear();
    opp->edges.clear();
    OpPtT segS {seg->c.pts[0], 0 };
    OpPtT segE {seg->c.lastPt(), 1 };
    OpPtT oppS {opp->c.pts[0], 0 };
    OpPtT oppE {opp->c.lastPt(), 1 };
	if (smallTFound) {
		FoundPtTs small = nearbyRun(seg, opp, segS, 1);
		OP_ASSERT(OpMath::IsFinite(small.seg.t) && OpMath::IsFinite(small.opp.t));
		segS = small.seg;
		oppS = small.opp;
	}
	if (largeTFound) {
		FoundPtTs large = nearbyRun(seg, opp, segE, 0);
		OP_ASSERT(OpMath::IsFinite(large.seg.t) && OpMath::IsFinite(large.opp.t));
		segE = large.seg;
		oppE = large.opp;
	}
    // if ends of segments already touch, exclude from made edge
    seg->makeEdge(segS, segE  OP_DEBUG_PARAMS(EDGE_MAKER(segSect)));
    opp->makeEdge(oppS, oppE  OP_DEBUG_PARAMS(EDGE_MAKER(oppSect)));
	OpEdge* edge = &seg->edges.back();
	edge->ccStart = edge->ccSmall = smallTFound;
	edge->ccEnd = edge->ccLarge = largeTFound;
	edgeCurves.c.push_back(edge);
	originalEdge = edge;
	OpEdge* oppEdge = &opp->edges.back();
	oppCurves.c.push_back(oppEdge);
	originalOpp = oppEdge;
	oppEdge->ccStart = oppEdge->ccSmall = matchRev.reversed ? largeTFound : smallTFound;
	oppEdge->ccEnd = oppEdge->ccLarge = matchRev.reversed ? smallTFound : largeTFound;
}

void OpCurveCurve::addIntersection(OpEdge& edge, OpEdge& opp) {
	recordSect(edge, edge.start, opp, opp.start  
			OP_LINE_FILE_PARAMS(SectReason::edgeCurveCurve, SectReason::oppCurveCurve));
	snipEdge = edge.start;
	snipOpp = opp.start;
}

SectFound OpCurveCurve::addSect() {
	if (!ccSects.size())
		return SectFound::no;
	for (CcSects& s : ccSects) {
		OpSegment* eSegment = const_cast<OpSegment*>(s.e.edge->segment);
		OpSegment* oSegment = const_cast<OpSegment*>(s.o.edge->segment);
		OpIntersection* sect = eSegment->addEdgeSect(s.e.ptT  
				OP_LINE_FILE_PARAMS(SectReason::lineCurve, s.e.edge, s.o.edge));
		OpIntersection* oSect = oSegment->addEdgeSect(s.o.ptT  
				OP_LINE_FILE_PARAMS(SectReason::lineCurve, s.e.edge, s.o.edge));
		sect->pair(oSect);
	}
	return SectFound::intersects;
}

void OpCurveCurve::addUnsectable(OpSegment* seg, const OpPtT& edgeStart, const OpPtT& edgeEnd,
		OpSegment* opp, const OpPtT& oppStart, const OpPtT& oppEnd) {  
	int usectID = seg->nextID();
	OpIntersection* segSect1 = seg->addUnsectable(edgeStart, usectID, MatchEnds::start, opp
			OP_LINE_FILE_PARAMS(SectReason::unsectable));
	OpIntersection* segSect2 = seg->addUnsectable(edgeEnd, usectID, MatchEnds::end, opp
			OP_LINE_FILE_PARAMS(SectReason::unsectable));
	MatchEnds oppMatch = MatchEnds::start;
	OpPtT oStart = oppStart;
	OpPtT oEnd = oppEnd;
	bool flipped = oStart.t > oEnd.t;
	if (flipped) {
		std::swap(oStart, oEnd); // flip to fix opp t (now opp pts are flipped vs. edge)
		usectID = -usectID;
		oppMatch = MatchEnds::end;
	}
	OpIntersection* oppSect1 = opp->addUnsectable(oStart, usectID, oppMatch, seg
			OP_LINE_FILE_PARAMS(SectReason::unsectable));
	OpIntersection* oppSect2 = opp->addUnsectable(oEnd, usectID, !oppMatch, seg
			OP_LINE_FILE_PARAMS(SectReason::unsectable));
	segSect1->pair(flipped ? oppSect2 : oppSect1);
	segSect2->pair(flipped ? oppSect1 : oppSect2);
}

// if after breaking runs spacially on both edge and opp into two runs
//  and one run is connected to already found intersections, remove that run
// return true if edges connected to small and large t are marked for removal (not overlapping)
bool OpCurveCurve::checkForGaps() {
	if (!smallTFound && !largeTFound)
		return false;
	OP_ASSERT(edgeCurves.c.size() && oppCurves.c.size());
	std::vector<TGap> edgeGaps = edgeCurves.findGaps();
	if (edgeGaps.size() < smallTFound + largeTFound)  // require 2 gaps if sm && lg
		return false;
	std::vector<TGap> oppGaps = oppCurves.findGaps();
	if (oppGaps.size() < smallTFound + largeTFound)
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
		bool edgeDone = edge.start.isNearly(edge.end);
		for (auto oppPtr : oppCurves.c) {
			auto& opp = *oppPtr;
			if (!opp.ccOverlaps)
				continue;
			// check end condition
			if (edgeDone && (opp.start.isNearly(opp.end))) {
				addIntersection(edge, opp);
				return true;
			} 
			if (ifExactly(edge, edge.start, opp, opp.start)
					|| ifExactly(edge, edge.end, opp, opp.start)
					|| ifExactly(edge, edge.start, opp, opp.end)
					|| ifExactly(edge, edge.end, opp, opp.end))
				return true;
			if (ifNearly(edge, edge.start, opp, opp.start)
					|| ifNearly(edge, edge.end, opp, opp.start)
					|| ifNearly(edge, edge.start, opp, opp.end)
					|| ifNearly(edge, edge.end, opp, opp.end))
				return true;
			if (!splitMid || edge.isLinear() || opp.isLinear()) {
				if (setHullSects(edge, opp, CurveRef::edge))
					return true;
				if (setHullSects(opp, edge, CurveRef::opp))
					return true;
			}
		}
	}
	return false;
}

// Scan through opposite curves and see if check point is inside deleted bounds. If so, use a
// different (but close by if possible) point to split the curve.
bool OpCurveCurve::checkSplit(float loT, float hiT, CurveRef which, OpPtT& checkPtT) const {
	OP_ASSERT(loT <= checkPtT.t && checkPtT.t <= hiT);
	const std::vector<OpEdge*>& oCurves = CurveRef::edge == which ? oppCurves.c : edgeCurves.c;
	const OpEdge* edge = CurveRef::edge == which ? originalEdge : originalOpp;
	const OpCurve& eCurve = edge->segment->c;
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
		const OpEdge* opp = CurveRef::edge == which ? originalOpp : originalEdge;
		OpPtT delLo = opp->start;
		auto checkBounds = [checkPtT](const OpPtT& lo, const OpPtT& hi) {
			OpPointBounds delBounds(lo.pt, hi.pt);
			return delBounds.contains(checkPtT.pt);
		};
		for (OpEdge* oCurve : oCurves) {
			OpPtT delHi = oCurve->start;
			OP_ASSERT(delLo.t <= delHi.t);
			if (delLo.t < delHi.t && checkBounds(delLo, delHi))  // there's a gap between edges
				goto tryAgain;
			delLo = oCurve->end;
			if (!oCurve->ccOverlaps && checkBounds(delHi, delLo))  // check edge without overlap
				goto tryAgain;
		}
		if (delLo.t == opp->end.t || !checkBounds(delLo, opp->end))  // check last list / original
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

// !!! to do : look for 2nd best point if curves come close to intersecting twice
void OpCurveCurve::closest() {
    if (!OpMath::IsFinite(closeEdge.bestDist))
		return;
    // require either pt or t to be within some metric of closeness
    // since this specifies intersection points, some sloppiness is OK
    // as long as the point is (nearly) on each segment
    OpPtT sPtT = closeEdge.best.e.ptT;
    OpPtT oPtT = OpPtT(sPtT.pt, closeEdge.best.o.ptT.t);
	OpSegment* s = const_cast<OpSegment*>(closeEdge.best.e.edge->segment);
	OpSegment* o = const_cast<OpSegment*>(closeEdge.best.o.edge->segment);
	if (s->sects.contains(sPtT, o) || o->sects.contains(oPtT, s))
		return;
	OpIntersection* sect = s->addSegSect(sPtT, o  OP_LINE_FILE_PARAMS(SectReason::soClose));
	OpIntersection* oSect = o->addSegSect(oPtT, s  OP_LINE_FILE_PARAMS(SectReason::soClose));
    sect->pair(oSect);
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
#if OP_DEBUG_DUMP
	int oneOr = 2;
	int theOther = 5;
	bool breakAtDraw = (oneOr == originalEdge->segment->id && theOther == originalOpp->segment->id)
			|| (oneOr == originalOpp->segment->id && theOther == originalEdge->segment->id);
#endif
	OP_ASSERT(1 == edgeCurves.c.size());
	OP_ASSERT(1 == oppCurves.c.size());
//	int splitCount = 0;
	// !!! start here;
	// looks like recursion is getting stuck trimming tiny bits from end instead of dividing in the
	// middle and/or detecting that it ought to look to see if the run is largely unsectable. It
	// should be trying in more than one place to find intersections somehow
	for (depth = 1; depth < maxDepth; ++depth) {
#if OP_DEBUG_DUMP
		constexpr int debugBreakDepth = 1;
		if (breakAtDraw && depth >= debugBreakDepth) {
			if (debugBreakDepth == depth)
				::debug();
			1 == depth ? ::showSegmentEdges() : ::hideSegmentEdges();
			::dmpDepth(depth);
			OpDebugOut("");
		}
#endif
		bool snipEm = false;
		if (!setOverlaps())
			return SectFound::fail;
		if (checkForGaps() || (splitMid && !endsOverlap()))
			splitMid = false;
		if (checkSect())
			snipEm = true;
#if 0  // !!! if required, document test case that needs it
		else if (SectFound::tryClose != setOver)
			tryClose();
#endif
		// if there is more than one crossover, look for unsectable
		if (ccSects.size() > (smallTFound || largeTFound ? 0 : 1)
				&& (SectFound::intersects == findUnsectable() 
				|| !edgeCurves.c.size() || !oppCurves.c.size()))
			return addSect();
		if ((edgeCurves.c.size() >= maxSplits && oppCurves.c.size() >= maxSplits) ||
				edgeCurves.c.size() >= ccMaxSplits * 2 || oppCurves.c.size() >= ccMaxSplits * 2) {
			if (ccSects.size())
				return addSect();
			if (smallTFound || largeTFound)
				return SectFound::no;
			OP_ASSERT(0);  // more code required in this case
		}
#if OP_DEBUG_VERBOSE  // save state prior to split and delete
		debugSaveState();
#endif
		if (!edgeCurves.overlaps() || !oppCurves.overlaps())
			return addSect();
		// If any intersection is found, already found or not, remove piece around 
		// both edge and opp so that remaining edges can be checked for intersection
		if (snipEm) {
			edgeCurves.snipAndGo(originalEdge->segment, snipEdge);
			oppCurves.snipAndGo(originalOpp->segment, snipOpp);
		} else
			splitHulls(CurveRef::edge) || splitHulls(CurveRef::opp);  // only split one if effective
		if (!edgeCurves.c.size() || !oppCurves.c.size())
			return addSect();
	}
	OP_ASSERT(0);  // !!! if this occurs likely more code is needed
	return SectFound::fail;
}

// return true if either small t or large t belong to edge that is still available
bool OpCurveCurve::endsOverlap() const {
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
SectFound OpCurveCurve::findUnsectable() {
	// start with existing found point
	OP_ASSERT(ccSects.size() || smallTFound || largeTFound);
	OpSegment* seg = const_cast<OpSegment*>(originalEdge->segment);
	OpSegment* opp = const_cast<OpSegment*>(originalOpp->segment);
	// find limit of where curve pair are nearly coincident
	std::sort(ccSects.begin(), ccSects.end(), [](const CcSects& a, const CcSects& b) {
			return a.e.ptT.t < b.e.ptT.t; });
	FoundLimits limits;
	if (0 == ccSects[0].e.ptT.t || smallTFound)
		limits.lo.setEnd(opp, seg->c, 0);
	else
		limits.lo = nearbyRun(seg, opp, ccSects[0].e.ptT, 0);
	if (1 == ccSects.back().e.ptT.t || largeTFound)
		limits.hi.setEnd(opp, seg->c, 1);
	else
		limits.hi = nearbyRun(seg, opp, ccSects.back().e.ptT, 1);
	// replace matching existing sects with new range, marking them as unsectable range
	auto sharePoint = [](OpPtT& seg, OpPtT& opp) {
		OpPtT::MeetInTheMiddle(seg, opp);
		seg.t = OpMath::PinNear(seg.t);
		opp.t = OpMath::PinNear(opp.t);
	};
	sharePoint(limits.lo.seg, limits.lo.opp);
	sharePoint(limits.hi.seg, limits.hi.opp);
	// check the middle of the two ends to see if the whole span is plausibly coincident
	OpPoint segMid = (limits.lo.seg.pt + limits.hi.seg.pt) / 2;
	OpPtT midPtT { seg->c.ptTAtT(seg->findValidT(0, 1, segMid)) };
	if (!segMid.isNearly(midPtT.pt)) {
		std::vector<CcSects> copy;
		if (!smallTFound)
			copy.push_back(ccSects[0]);
		if (!largeTFound)
			copy.push_back(ccSects.back());
		std::swap(ccSects, copy);
		return SectFound::intersects;
	}
	addUnsectable(seg, limits.lo.seg, limits.hi.seg, opp, limits.lo.opp, limits.hi.opp);
	ccSects.clear();
	// snip remaining parts of curve pairs as needed
	edgeCurves.snipOne(seg, limits.lo.seg, limits.hi.seg); 
	oppCurves.snipOne(opp, limits.lo.opp, limits.hi.opp); 
	return SectFound::split;
}

void OpCurveCurve::ifCloseSave(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT) {
	if (!edgePtT.pt.soClose(oppPtT.pt))
		return;
	closeEdge.save(edge, edgePtT, opp, oppPtT);
}

bool OpCurveCurve::ifExactly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT) {
	if (edgePtT.pt != oppPtT.pt)
		return false;

	recordSect(edge, edgePtT, opp, oppPtT 
			OP_LINE_FILE_PARAMS(SectReason::edgeCCExact, SectReason::oppCCExact));
	snipEdge = edgePtT;
	snipOpp = oppPtT;
	return true;
}

bool OpCurveCurve::ifNearly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT) {
	if (!edgePtT.pt.isNearly(oppPtT.pt))
		return false;
	recordSect(edge, edgePtT, opp, oppPtT  
			OP_LINE_FILE_PARAMS(SectReason::edgeCCNearly,  SectReason::oppCCNearly));
	snipEdge = edgePtT;
	snipOpp = oppPtT;
	return true;
}

// returns true if edge's line missed opposite edge's curve
// don't check against opposite segment's curve, since it may be offset
// (Later) If edge is linear, but not a true line (e.g., the control points are nearly colinear with
//  the end points) check the hulls rather than just the line. 
bool OpCurveCurve::LineMissed(OpEdge& edge, OpEdge& opp) {
	if (!edge.isLinear())
		return false;
	if (!edge.exactLine)
		return false;
	LinePts edgePts;
	edgePts.pts = { edge.start.pt, edge.end.pt };
	OpRootPts septs = opp.curve.lineIntersect(edgePts);
	return !septs.count;
}

// trim matches back far enough so that is nearly does not trigger on this end
// returns point furthest from input that near by the opposite curve (extent of unsectable run)
// returns extent from base towards end seg t
FoundPtTs OpCurveCurve::nearbyRun(OpSegment* seg, OpSegment* opp, const OpPtT& basePtT, 
		float endSegT) {
	bool searchDown = endSegT < basePtT.t;
	float loSegT = searchDown ? endSegT : basePtT.t;
	float hiSegT = searchDown ? basePtT.t : endSegT;
	float testSegT = OpMath::Average(loSegT, hiSegT);
	float step = OpMath::Average(testSegT, -hiSegT);
	for (;;) {
		OpPtT testSegPtT { seg->c.ptTAtT(testSegT) };
		OpPtT testOppPtT { opp->c.ptTAtT(opp->findValidT(0, 1, testSegPtT.pt)) };
		if (fabsf(step) < OpEpsilon)
			break;
		// e.g., if step up, opp pt is nearby seg pt, searching down: reverse step
		bool nearby = testOppPtT.pt.isNearly(testSegPtT.pt);
		if ((int) ((step > 0) ^ nearby ^ searchDown))
			step = -step;
		testSegT += step;
		step = step / 2;
	}
	// advance until points are not near
	do {
		OpPtT foundSegPtT { seg->c.ptTAtT(testSegT) };
		OpPtT foundOppPtT { opp->c.ptTAtT(opp->findValidT(0, 1, foundSegPtT.pt)) };
		if (!foundSegPtT.pt.isNearly(foundOppPtT.pt) && foundOppPtT.t >= OpEpsilon
				&& foundOppPtT.t <= 1 - OpEpsilon)
			return { foundSegPtT, foundOppPtT };
		testSegT += searchDown ? -OpEpsilon : OpEpsilon;
	} while (0 <= testSegT && testSegT <= 1);
	OP_ASSERT(0);
	return FoundPtTs();  // error (pt t is nan)
}

void OpCurveCurve::recordSect(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT
			OP_LINE_FILE_DEF(SectReason eReason, SectReason oReason)) {
	OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
	OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
	// end points matching has already been recorded, so don't do it again
	if (edgePtT.onEnd() && oppPtT.onEnd())
		return;
	OpPtT ePtT = edgePtT;
	OpPtT oPtT = oppPtT;
	if (const OpIntersection* eClose = eSegment->sects.nearly(ePtT, nullptr); eClose) {
		if (eClose->segment == oSegment)
			return;
		ePtT = eClose->ptT;
	}
	if (const OpIntersection* oClose = oSegment->sects.nearly(oPtT, nullptr); oClose) {
		if (oClose->segment == eSegment)
			return;
		oPtT = oClose->ptT;
	}
	OpPtT::MeetInTheMiddle(ePtT, oPtT);
	if (eSegment->sects.nearly(ePtT, oSegment))
		// even though intersection is already recorded, report here so edges can be trimmed
		return;
	if (oSegment->sects.nearly(oPtT, eSegment))
		return;
	// note that that sects are not added until the end
	if (ccSects.end() != std::find_if(ccSects.begin(), ccSects.end(), 
			[ePtT, oPtT](auto ccSect) {
			return ccSect.e.ptT.isNearly(ePtT) || ccSect.o.ptT.isNearly(oPtT);
			} ))
		return;
	ccSects.emplace_back(&edge, ePtT, &opp, oPtT  OP_LINE_FILE_PARAMS(eReason, oReason));
}

bool OpCurveCurve::rotatedIntersect(OpEdge& edge, OpEdge& opp) {
	LinePts edgePts { edge.start.pt, edge.end.pt };
	OpCurve edgeRotated = edge.setVertical(edgePts);
	rotateFailed |= !edgeRotated.isFinite();
	OpCurve oppRotated = opp.setVertical(edgePts);
	rotateFailed |= !oppRotated.isFinite();
	OpPointBounds eRotBounds;
	eRotBounds.set(edgeRotated);
	OpPointBounds oRotBounds;
	oRotBounds.set(oppRotated);
	if (eRotBounds.intersects(oRotBounds))
		return true;
	// testQuads3760505 needs this
	bool outsetIntersects = eRotBounds.outsetClose().intersects(oRotBounds.outsetClose());
	tryCloseBy |= outsetIntersects;
	return outsetIntersects;
}

bool OpCurveCurve::setHullSects(OpEdge& edge, OpEdge& opp, CurveRef curveRef) {
	int ptCount = edge.curve.pointCount();
	LinePts edgePts;
	edgePts.pts[1] = edge.curve.pts[0];
	for (int index = 1; index <= ptCount; ++index) {
		edgePts.pts[0] = edgePts.pts[1];
		int endHull = index < ptCount ? index : 0;
		edgePts.pts[1] = edge.curve.pts[endHull];
		if (edgePts.pts[0].isNearly(edgePts.pts[1]))
			continue;
		// since curve/curve intersection works by keeping overlapping edge bounds, it should
		// use edge, not segment, to find hull intersections
		OpRootPts septs = opp.curve.lineIntersect(edgePts);
		for (size_t inner = 0; inner < septs.count; ++inner) {
			OpPtT sectPtT = septs.ptTs[inner];
			// set to secttype endhull iff computed point is equal to or nearly an end point
			SectType sectType;
			if ((1 == index || 0 == endHull) && sectPtT.pt.isNearly(edgePts.pts[0])) {
				sectType = SectType::endHull;
				sectPtT.pt = edgePts.pts[0];
			} else if ((ptCount - 1 == index || 0 == endHull) 
					&& sectPtT.pt.isNearly(edgePts.pts[1])) {
				sectType = SectType::endHull;
				sectPtT.pt = edgePts.pts[1];
			} else
				sectType = endHull ? SectType::controlHull : SectType::midHull;
			sectPtT.t = OpMath::Interp(opp.start.t, opp.end.t, sectPtT.t);
			OP_ASSERT(opp.start.t <= sectPtT.t && sectPtT.t <= opp.end.t);
			if (opp.hulls.add(sectPtT, sectType, &edge)) {
				OpPtT edgePtT { edge.segment->c.ptTAtT(edge.segment->findValidT(0, 1, sectPtT.pt))};
				OP_DEBUG_CODE(SectReason debugEdgeReason = SectReason::edgeCtrlMidHull);
				OP_DEBUG_CODE(SectReason debugOppReason = SectReason::oppCtrlMidHull);
				if (CurveRef::opp == curveRef) {
					recordSect(opp, sectPtT, edge, edgePtT
							OP_LINE_FILE_PARAMS(debugOppReason, debugEdgeReason));
					snipEdge = sectPtT;
					snipOpp = edgePtT;
				} else {
					recordSect(edge, edgePtT, opp, sectPtT
							OP_LINE_FILE_PARAMS(debugEdgeReason, debugOppReason));
					snipEdge = edgePtT;
					snipOpp = sectPtT;
				}
				return true;
			}
		}
	}
	return false;
}

bool OpCurveCurve::setOverlaps() {
	tryCloseBy = false;
	edgeCurves.clear();
	oppCurves.clear();
	for (auto edgePtr : edgeCurves.c) {
		auto& edge = *edgePtr;
		for (auto oppPtr : oppCurves.c) {
			auto& opp = *oppPtr;
			// !!! used to be close bounds; simple bounds required for testQuads1883885
			if (depth > 1 && !edge.bounds().intersects(opp.bounds()))
				continue;
			if (!rotatedIntersect(edge, opp))
				continue;
			if (!rotatedIntersect(opp, edge))
				continue;
			if (LineMissed(edge, opp) || LineMissed(opp, edge)) {
				tryCloseBy = true;
				continue;
			}
			opp.ccOverlaps = true;
			edge.ccOverlaps = true;
		}
	}
	if (tryCloseBy)
		tryClose();
	return !rotateFailed;
}

void OpCurveCurve::splitDownTheMiddle(OpContours* contours, OpEdge& edge, const OpPtT& edgeMid, 
		CcCurves* splits) {
	// !!!? while edgeMid is in a deleted bounds, bump edgeMidT
	//      (this isn't necessarily in the intersection hull)
	//      wait until this is necessary to make it work
	// checkSplit(edge.start.t, edge.end.t, which, edgeMid);
	void* blockL = contours->allocateEdge(contours->ccStorage);
	OpEdge* splitLeft = new(blockL) OpEdge(&edge, edgeMid, NewEdge::isLeft  
			OP_DEBUG_PARAMS(EDGE_MAKER(splitLeft)));
	splitLeft->ccOverlaps = true;
	splitLeft->ccStart = edge.ccStart;
	splitLeft->ccSmall = edge.ccSmall;
	splits->c.push_back(splitLeft);
	void* blockR = contours->allocateEdge(contours->ccStorage);
	OpEdge* splitRight = new(blockR) OpEdge(&edge, edgeMid, NewEdge::isRight  
			OP_DEBUG_PARAMS(EDGE_MAKER(splitRight)));
	splitRight->ccOverlaps = true;
	splitRight->ccEnd = edge.ccEnd;
	splitRight->ccLarge = edge.ccLarge;
	splits->c.push_back(splitRight);
}

// If edge has hull points that define split but opp does not, opp may be entirely inside edge hull.
// Modify below to keep opp corresponding to kept edge split to handle this case.
// where should the opp be stored?

// if end hull index == -1, discard both sides of sect (cutout via exact or nearby sect)
// if end hull >= 0, look for sect through curve
bool OpCurveCurve::splitHulls(CurveRef which) {
	CcCurves& curves = CurveRef::edge == which ? edgeCurves : oppCurves;
	CcCurves& oCurves = CurveRef::edge == which ? oppCurves : edgeCurves;
	CcCurves splits;
	OpContours* contours = originalEdge->contours();
	for (auto edgePtr : curves.c) {
		auto& edge = *edgePtr;
		if (!edge.ccOverlaps)
			continue;
		float edgeMidT = OpMath::Average(edge.start.t, edge.end.t);
		OpHulls& hulls = edge.hulls;
		auto closest = [oCurves](OpPoint pt) -> OpPtT {
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
			for (OpEdge* opp : oCurves.c) {
				OpPoint startPt = opp->start.pt;
				if (oLast != startPt)
					bestPt(startPt);
				OpPoint endPt = opp->end.pt;
				bestPt(endPt);
				oLast = endPt;
			}
			return { oBest, oBestDistSq };
		};
		auto ptTCloseTo = [edge](OpPtT oPtPair, const OpPtT& ptT, 
				OpPtT* result) {
			OpVector unitTan = edge.segment->c.tangent(ptT.t);
			OpVector tan = unitTan.setLength(sqrtf(oPtPair.t));
			if (1 == ptT.t)
				tan = -tan;
			OpPoint testPt = ptT.pt + tan;
			if (!edge.ptBounds.contains(testPt))
				return;
			// use unit tan to pass correct axis to find pt t
			Axis axis = fabsf(unitTan.dx) > fabsf(unitTan.dy) ? Axis::vertical : Axis::horizontal; 
			float resultT = edge.segment->findAxisT(axis, edge.start.t, edge.end.t, 
					testPt.choice(axis));
			if (!OpMath::IsNaN(resultT))
				*result = OpPtT(edge.segment->c.ptAtT(resultT), resultT);
		};
		// use the smaller distance to an end of oStart, oEnd, edgeMid
		auto splitPt = [closest, ptTCloseTo, edge](float oMidDist, OpPtT* result) {
			OpPtT oStart = closest(edge.start.pt);
			OpPtT oEnd = closest(edge.end.pt);
			// choose split point near edge end that is closest to opp end
			if (oMidDist <= std::min(oStart.t, oEnd.t))
				return;
			if (oStart.t > oEnd.t)
				ptTCloseTo(oEnd, edge.end, result);
			else
				ptTCloseTo(oStart, edge.start, result);
		};
		if (!hulls.h.size() || splitMid) {
			OpPtT edgeMid = { edge.segment->c.ptAtT(edgeMidT), edgeMidT };
			splitDownTheMiddle(contours, edge, edgeMid, &splits);
			continue;
		}
		if (2 <= hulls.h.size()) {  // see if hulls are close enough to define an intersection
			hulls.sort(smallTFound);
			for (size_t index = 1; index < hulls.h.size(); ++index) {
				if (!hulls.sectCandidates(index, edge))
					continue;
				OP_ASSERT(hulls.h[index - 1].opp);
				OpEdge& oEdge = const_cast<OpEdge&>(*hulls.h[index - 1].opp);
				OpPtT oPtT;
				OpPtT hull1Sect = hulls.h[index - 1].sect;
				oPtT.t = oEdge.segment->findValidT(0, 1, hull1Sect.pt);
				if (OpMath::IsNaN(oPtT.t))
					return false;
				oPtT.pt = oEdge.segment->c.ptAtT(oPtT.t);
				if (!hulls.closeEnough(index - 1, edge, oEdge, &oPtT, &hull1Sect))
					continue;
				OP_DEBUG_CODE(SectReason edgeReason = SectReason::edgeCCHullPair);
				OP_DEBUG_CODE(SectReason oppReason = SectReason::oppCCHullPair);
				if (CurveRef::opp == which)
					recordSect(oEdge, oPtT, edge, hull1Sect
							OP_LINE_FILE_PARAMS(oppReason, edgeReason));
				else 
					recordSect(edge, hull1Sect, oEdge, oPtT  
							OP_LINE_FILE_PARAMS(edgeReason, oppReason));
				curves.snipAndGo(edge.segment, hull1Sect);
				oCurves.snipAndGo(oEdge.segment, oPtT);
				return true;
			}
		}
		int midCount = 0;
		for (const HullSect hull : hulls.h) {
			if (edge.start.t < hull.sect.t && hull.sect.t < edge.end.t)
				++midCount;
		}
		if (!midCount) {
			OpPtT centerPtT = edge.center;
			OpPtT oCtr = closest(centerPtT.pt);
			splitPt(oCtr.t, &centerPtT);
			if (OpMath::IsFinite(centerPtT.t) && edge.start.t != centerPtT.t
					&& edge.end.t != centerPtT.t)
				hulls.add(centerPtT, SectType::center);
		}
		hulls.add(edge.start, SectType::endHull);
		hulls.add(edge.end, SectType::endHull);
		hulls.nudgeDeleted(edge, *this, which);
		size_t splitCount = splits.c.size();
		for (size_t index = 0; index + 1 < hulls.h.size(); ) {
			const HullSect& hullLo = hulls.h[index];
			const HullSect& hullHi = hulls.h[++index];
			if (edge.ccStart && edge.start.soClose(hullHi.sect))
				continue;
			if (edge.ccEnd && edge.end.soClose(hullLo.sect))
				continue;
			void* block = contours->allocateEdge(contours->ccStorage);
			OpEdge* split = new(block) OpEdge(&edge, hullLo.sect.t, 
					hullHi.sect.t  OP_DEBUG_PARAMS(EDGE_MAKER(hull)));
#if OP_DEBUG
			split->debugSplitStart = hullLo.type;
			split->debugSplitEnd = hullHi.type;
#endif
			split->ccOverlaps = true;
			split->ccStart = edge.ccStart && edge.start.soClose(split->start);
			if (split->ccStart)
				split->ccSmall = edge.ccSmall;
			split->ccEnd = edge.ccEnd && edge.start.soClose(split->end);
			if (split->ccEnd)
				split->ccLarge = edge.ccLarge;
			splits.c.push_back(split);
		}
		if (splits.c.size() - splitCount < 2) {
			splits.c.resize(splitCount); // this may leave an edge was never used ... not the end of the world
			splitDownTheMiddle(contours, edge, edge.center, &splits);  // divide edge in two			
		}
	}
	curves.c.swap(splits.c);
	return false;
}

void OpCurveCurve::tryClose() {
	for (auto edge : edgeCurves.c) {
		if (!edge->ccOverlaps)
			continue;
		if (edge->isClose()) {
			for (auto opp : oppCurves.c) {
				if (opp->isClose())
					closeEdge.save(*edge, edge->start, *opp, opp->start);
			}
		}
		if (!edge->ccStart) {
			for (auto opp : oppCurves.c) {
				if (!opp->ccStart)
					ifCloseSave(*edge, edge->start, *opp, opp->start);
				if (!opp->ccEnd)
					ifCloseSave(*edge, edge->start, *opp, opp->end);
			}
		}
		if (!edge->ccEnd) {
			for (auto opp : oppCurves.c) {
				if (!opp->ccStart)
					ifCloseSave(*edge, edge->end, *opp, opp->start);
				if (!opp->ccEnd)
					ifCloseSave(*edge, edge->end, *opp, opp->end);
			}
		}
	}
}
