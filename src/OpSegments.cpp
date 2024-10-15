// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpSegments.h"
#include "OpWinder.h"

static bool compareXBox(const OpSegment* s1, const OpSegment* s2) {
    const OpRect& r1 = s1->ptBounds;
    const OpRect& r2 = s2->ptBounds;
    if (r1.left < r2.left)
        return true;
    if (r1.left > r2.left)
        return false;
    if (r1.left == r2.left && r1.right < r2.right)
        return true;
    if (r1.left == r2.left && r1.right > r2.right)
        return false;
    return s1->id < s2->id;
}

OpSegments::OpSegments(OpContours& contours) {
    inX.clear();
    for (auto contour : contours.contours) {
        for (auto& segment : contour->segments) {
            inX.push_back(&segment);
        }
    }
    std::sort(inX.begin(), inX.end(), compareXBox);
    OP_DEBUG_CODE(debugFailSegID = 0);
    OP_DEBUG_CODE(debugFailOppID = 0);

}

// may need to adjust values in opp if end is nearly equal to seg
void OpSegments::AddEndMatches(OpSegment* seg, OpSegment* opp) {
    OP_DEBUG_CONTEXT();
    auto add = [seg, opp](OpPoint pt, float segT, float oppT   OP_LINE_FILE_DEF()) {
        if (opp->willDisable || seg->willDisable)
            return;
        if (seg->sects.contains(OpPtT { pt, segT }, opp) 
                || opp->sects.contains(OpPtT { pt, oppT }, seg))
            return;
        OpIntersection* sect = seg->addSegSect(OpPtT { pt, segT }, opp  
                OP_LINE_FILE_CALLER());
        OpIntersection* oSect = opp->addSegSect(OpPtT { pt, oppT }, seg 
                OP_LINE_FILE_CALLER());
        sect->pair(oSect);
    };
    auto checkEnds = [add, seg, opp](OpPoint oppPt, float oppT  OP_LINE_FILE_DEF()) {
        OpPtT segPtT;
        OpPtAliases& aliases = seg->contour->contours->aliases;
        if (seg->c.firstPt().isNearly(oppPt, seg->threshold()) || (seg->startMoved 
                && aliases.isSmall(seg->c.firstPt(), oppPt)))
            segPtT = { seg->c.firstPt(), 0 };
        else if (seg->c.lastPt().isNearly(oppPt, seg->threshold()) || (seg->endMoved 
                && aliases.isSmall(seg->c.lastPt(), oppPt)))
            segPtT = { seg->c.lastPt(), 1 };
        if (!OpMath::IsNaN(segPtT.t)) {
            oppPt = seg->mergePoints(segPtT, opp, { oppPt, oppT });
            add(oppPt, segPtT.t, oppT  OP_LINE_FILE_CALLER());
        }
        OP_ASSERT(opp->c.c.data->start != opp->c.c.data->end || opp->willDisable);
        return segPtT.t;
    };
    float startSegT = checkEnds(opp->c.firstPt(), 0  OP_LINE_FILE_PARAMS());
    float endSegT = checkEnds(opp->c.lastPt(), 1  OP_LINE_FILE_PARAMS());
    auto checkOpp = [add, seg, opp](OpPoint segPt, float segT  OP_LINE_FILE_DEF()) {
        float oppT = opp->c.match(0, 1, segPt);
		if (OpMath::IsNaN(oppT))
            return oppT;
        oppT = OpMath::PinNear(oppT);
        if (0 == oppT || 1 == oppT) {
            OpPoint oppEnd = opp->c.end(oppT);
            segPt = seg->mergePoints({ segPt, segT }, opp, { oppEnd, oppT });
        }
        /// !!! may add coincidence between seg and opp which goes undetected (skphealth_com76s)
        add(segPt, segT, oppT  OP_LINE_FILE_CALLER());
        return oppT;
    };
    float startOppT = OpNaN;
    float endOppT = OpNaN;
    if (0 != startSegT && 0 != endSegT) 
        startOppT = checkOpp(seg->c.firstPt(), 0  OP_LINE_FILE_PARAMS());  // see if start pt is on opp curve
    if (1 != startSegT && 1 != endSegT) 
		endOppT = checkOpp(seg->c.lastPt(), 1  OP_LINE_FILE_PARAMS());
    auto checkSeg = [add, seg, opp](OpPoint oppPt, float oppT  OP_LINE_FILE_DEF()) {
        float segT = seg->c.match(0, 1, oppPt);
		if (OpMath::IsNaN(segT))
            return;
        segT = OpMath::PinNear(segT);
        if (0 == segT || 1 == segT) {
            OpPoint segEnd = seg->c.end(segT);
            if (segEnd != oppPt)
                oppPt = opp->mergePoints({ oppPt, oppT }, seg, { segEnd, segT });
        }
        add(oppPt, segT, oppT  OP_LINE_FILE_CALLER());
    };
    if (OpMath::IsNaN(startSegT) && 0 != startOppT && 0 != endOppT)
        checkSeg(opp->c.firstPt(), 0  OP_LINE_FILE_PARAMS());
    if (OpMath::IsNaN(endSegT) && 1 != startOppT && 1 != endOppT)
        checkSeg(opp->c.lastPt(), 1  OP_LINE_FILE_PARAMS());
}

// somewhat different from winder's edge based version, probably for no reason
void OpSegments::AddLineCurveIntersection(OpSegment* opp, OpSegment* seg) {
    OP_DEBUG_CONTEXT();
    OP_ASSERT(opp != seg);
    OP_ASSERT(seg->c.debugIsLine());
    LinePts edgePts { seg->c.firstPt(), seg->c.lastPt() };
    OP_ASSERT(edgePts.pts[0] != edgePts.pts[1]);
    MatchReverse matchRev = opp->matchEnds(seg);
    // if line and curve share end point, pass hint that root finder can call
    // reduced form that assumes one root is zero or one.
    OpRoots septs = opp->c.rayIntersect(edgePts, matchRev.match);
	if (septs.fail == RootFail::rawIntersectFailed) {
		// binary search on opp t-range to find where vert crosses zero
		OpCurve rotated = opp->c.toVertical(edgePts, matchRev.match);
		septs.roots[0] = rotated.tZeroX(0, 1);
		septs.count = 1;
	}
    if (opp->c.isLine() && MatchEnds::both == matchRev.match) {
        seg->moveWinding(opp, matchRev.reversed);
        return;
    }
    if (2 == septs.count && opp->c.isLine()) {
#if OP_NEW_COINCIDENCE
        OpWinder::CoincidentCheck(seg, opp);
#else
        OpWinder::CoincidentCheck({ edgePts.pts[0], 0 }, { edgePts.pts[1], 1 },
                { opp->c.firstPt(), 0}, { opp->c.lastPt(), 1 }, seg, opp );
#endif
        return;
    }
    std::vector<OpPtT> oppPtTs;
    std::vector<OpPtT> edgePtTs;
    size_t segSects = seg->sects.i.size();
    size_t oppSects = opp->sects.i.size();
    for (unsigned index = 0; index < septs.count; ++index) {
        float oppT = septs.get(index);
        if (OpMath::NearlyEndT(oppT))
            continue;
        // if computed point is nearly end, ignore
        OpPoint oppPt = opp->c.ptAtT(oppT);  // !!! redundant if ray intersect is rewritten to return pt
        if (oppPt.isNearly(oppT < .5 ? opp->c.firstPt() : opp->c.lastPt(), seg->threshold()))
            continue;
        if (oppPt.isNearly(edgePts.pts[0], seg->threshold()))
            continue;
        if (oppPt.isNearly(edgePts.pts[1], seg->threshold()))
            continue;
        float edgeT = seg->findValidT(0, 1, oppPt);
        oppPt = seg->c.ptAtT(edgeT);  // use line instead of curve to keep points on line
        OpPtT oppPtT { oppPt, oppT };
        if (OpMath::IsNaN(edgeT))
            continue;
        if (OpMath::NearlyEndT(edgeT))
            continue;
        seg->ptBounds.pin(&oppPtT.pt);  // required by testLine409
        opp->ptBounds.pin(&oppPtT.pt);
        oppPtTs.push_back(oppPtT);
        edgePtTs.emplace_back(oppPtT.pt, edgeT);
        OpPtT& edgePtT = edgePtTs.back();
        for (size_t earlier = 1; earlier < oppPtTs.size(); ++earlier) {
            if (oppPtTs[earlier - 1].t == oppPtT.t)
                continue;
        }
        for (size_t earlier = 1; earlier < edgePtTs.size(); ++earlier) {
            if (edgePtTs[earlier - 1].t == edgePtT.t)
                continue;
        }
        if (seg->sects.contains(edgePtT, opp))
            continue;
        if (opp->sects.contains(oppPtT, seg))
            continue;
            // don't add sects here if coincident or unsectable will be added below --
            // i guess record this and defer until after coin/unsect has been checked
        OpIntersection* sect = seg->addSegBase(edgePtT  
                OP_LINE_FILE_PARAMS(opp));
        OpIntersection* oSect = opp->addSegBase(oppPtT  
                OP_LINE_FILE_PARAMS(seg));
        sect->pair(oSect);
    }
    // if pair share two intersections, and mid t is close, mark intersections as unsectable
    OpIntersection* iStart = nullptr;
    OpIntersection* iEnd = nullptr;
    size_t index = 0;
    while (index < seg->sects.i.size()) {
        OpIntersection* test = seg->sects.i[index];
        if (test->opp->segment == opp) {
            if (iStart) {
                iEnd = test;
                break;
            }
            iStart = test;
        }
        ++index;
    }
    if (iEnd) {
        float midT = OpMath::Average(iStart->ptT.t, iEnd->ptT.t);
        // distance from seg point at midT normal to opp segment
        OpPtT midPtT = seg->c.ptTAtT(midT);
        OpPtT oppPtT = CcCurves::Dist(seg, midPtT, opp);
        float dist = (midPtT.pt - oppPtT.pt).length();
        auto endFromT = [](OpIntersection* one, OpIntersection* two, MatchEnds match) -> MatchEnds {
            return (one->ptT.t < two->ptT.t) == (MatchEnds::start == match) 
                    ? MatchEnds::start : MatchEnds::end;
        };
        if (dist < OpEpsilon) {
            auto removeBetweeners = [](OpSegment* seg, size_t segSects, 
                    OpIntersection* start, OpIntersection* end) {
                if (start->ptT.t > end->ptT.t)
                    std::swap(start, end);
                size_t index = seg->sects.i.size();
                const OpSegment* opp = start->opp->segment;
                OP_ASSERT(opp == end->opp->segment);
                while (index > segSects) {
                    OpIntersection* test = seg->sects.i[--index];
                    if (test->opp->segment == opp 
                            && start->ptT.t < test->ptT.t && test->ptT.t < end->ptT.t)
                        seg->sects.i.erase(seg->sects.i.begin() + index);
                }
            };
            removeBetweeners(seg, segSects, iStart, iEnd);
            removeBetweeners(opp, oppSects, iStart->opp, iEnd->opp);
            std::array<CoinEnd, 4> ends {{{ seg, opp, iStart->ptT, OpVector() }, 
                { seg, opp, iEnd->ptT, OpVector() },
                { opp, seg, iStart->opp->ptT, OpVector() }, 
                { opp, seg, iEnd->opp->ptT, OpVector() }}};
            OpWinder::CoincidentCheck(ends, nullptr, nullptr);
        } else if (dist < OpEpsilon * 8) { // !!! who knows what this const should be?
	        int usectID = seg->nextID();
            seg->addUnsectable(iStart->ptT, usectID, endFromT(iStart, iEnd, MatchEnds::start), opp
                    OP_LINE_FILE_PARAMS());
            seg->addUnsectable(iEnd->ptT, usectID, endFromT(iStart, iEnd, MatchEnds::end), opp
                    OP_LINE_FILE_PARAMS());
            OpIntersection* oStart = iStart->opp;
            OpIntersection* oEnd = iEnd->opp;
	        bool flipped = oStart->ptT.t > oEnd->ptT.t;
            if (flipped)
                usectID = -usectID;
            opp->addUnsectable(oStart->ptT, usectID, endFromT(oStart, oEnd, MatchEnds::start), seg
                    OP_LINE_FILE_PARAMS());
            opp->addUnsectable(oEnd->ptT, usectID, endFromT(oStart, oEnd, MatchEnds::end), seg
                    OP_LINE_FILE_PARAMS());
        }
    }
    return;
}

void OpSegments::FindCoincidences(OpContours* contours) {
    // take care of totally coincident segments
    SegmentIterator segIterator(contours);
    while (OpSegment* seg = segIterator.next()) {
        SegmentIterator oppIterator = segIterator;
        while (OpSegment* opp = oppIterator.next()) {
            if (seg->ptBounds != opp->ptBounds)
                continue;
            MatchReverse mr = seg->matchEnds(opp);
            if (MatchEnds::both != mr.match || seg->c.c.type != opp->c.c.type)
                continue;
                // if control points and weight match, treat as coincident: transfer winding
            if (!seg->contour->contours->callBack(seg->c.c.type).curvesEqualFuncPtr(
                    seg->c.c, opp->c.c ))
                continue;
            seg->winding.move(opp->winding, mr.reversed);
            opp->winding.zero();
            opp->setDisabled(OP_LINE_FILE_NPARAMS());
            if (!seg->winding.visible()) {
                seg->setDisabled(OP_LINE_FILE_NPARAMS());
                break;
            }
        }
    }
}

IntersectResult OpSegments::LineCoincidence(OpSegment* seg, OpSegment* opp) {
    OP_ASSERT(seg->c.debugIsLine());
    OP_ASSERT(!seg->disabled);
    // special case pairs that exactly match start and end
    MatchReverse ends = seg->matchEnds(opp);
    if (MatchEnds::both == ends.match) {
        seg->moveWinding(opp, ends.reversed);
        return IntersectResult::yes;
    }
    OpVector tangent = seg->c.tangent(0);
    if (!tangent.dx || !tangent.dy) {
        OP_ASSERT(tangent.dx || tangent.dy);
        OP_ASSERT(opp->c.debugIsLine());
        OP_ASSERT(!opp->disabled);
        OpVector oTangent = opp->c.tangent(0);
        if (oTangent.dx && oTangent.dy)
            return IntersectResult::no;
        OP_ASSERT(oTangent.dx || oTangent.dy);
        if (!tangent.dot(oTangent))  // if at right angles, skip
            return IntersectResult::no;
        if (!seg->ptBounds.intersects(opp->ptBounds))  // close bounds intersect, ptBounds do not
            return IntersectResult::no;
#if OP_NEW_COINCIDENCE
        return OpWinder::CoincidentCheck(seg, opp);
#else
        seg->makeEdge(OP_LINE_FILE_NPARAMS());
        opp->makeEdge(OP_LINE_FILE_NPARAMS());
        return OpWinder::CoincidentCheck(seg->edges.front(), opp->edges.front());
#endif
    }
    LinePts oppLine = opp->c.linePts();
    OpCurve vertSeg = seg->c.toVertical(oppLine, ends.match);
    if (!vertSeg.isFinite()) {
        seg->contour->contours->setError(PathOpsV0Lib::ContextError::toVertical  
                OP_DEBUG_PARAMS(seg->id));
        return IntersectResult::fail;
    }
    if (!vertSeg.isVertical())
        return IntersectResult::no;
    LinePts segLine = seg->c.linePts();
    OpCurve vertOpp = opp->c.toVertical(segLine, ends.flipped());
    if (!vertOpp.isFinite()) {
        seg->contour->contours->setError(PathOpsV0Lib::ContextError::toVertical  
                OP_DEBUG_PARAMS(opp->id));
        return IntersectResult::fail;
    }
    if (!vertOpp.isVertical())
        return IntersectResult::no;
#if OP_NEW_COINCIDENCE
    LinePts edgePts;
    if (fabsf(vertSeg.firstPt().x - vertOpp.firstPt().x) > OpEpsilon)
        return IntersectResult::no;
    return OpWinder::CoincidentCheck(seg, opp);
#else
    OpPtT oppInSeg[2], segInOpp[2];
    OpPtT* oppInSegPtr = oppInSeg;
    OpPtT* segInOppPtr = segInOpp;
    auto checkBetween = [](OpCurve& vert, OpCurve& base, float test, OpPtT*& inPtr) {
        if (OpMath::Between(vert.firstPt().y, test, vert.lastPt().y)
                && vert.firstPt().y != test && test != vert.lastPt().y) {
            float t = OpMath::XYRatio(vert.firstPt().y, vert.lastPt().y, test);
            *inPtr++ = OpPtT(OpMath::Interp(base.firstPt(), base.lastPt(), t), t);
            return true;
        }
        return false;
    };
    auto checkVert = [checkBetween](OpCurve& vertBase, OpCurve& segCurve, OpCurve& vertO, 
            OpCurve& oppCurve, OpPtT*& oInSegPtr, OpPtT*& sInOppPtr
            OP_DEBUG_PARAMS(OpPtT* oInSeg, OpPtT* sInOpp)) {
        if (checkBetween(vertBase, segCurve, vertO.firstPt().y, oInSegPtr))
            *sInOppPtr++ = OpPtT(oppCurve.firstPt(), 0);
        if (checkBetween(vertBase, segCurve, vertO.lastPt().y, oInSegPtr))
            *sInOppPtr++ = OpPtT(oppCurve.lastPt(), 1);
    };
    OpCurve vertSegBase = seg->c.toVertical(segLine, MatchEnds::start);
    checkVert(vertSegBase, seg->c, vertOpp, opp->c, oppInSegPtr, segInOppPtr
            OP_DEBUG_PARAMS(oppInSeg, segInOpp));
    OpCurve vertOppBase = opp->c.toVertical(oppLine, MatchEnds::start);
    checkVert(vertOppBase, opp->c, vertSeg, seg->c, segInOppPtr, oppInSegPtr
            OP_DEBUG_PARAMS(segInOpp, oppInSeg));
    if (MatchEnds::none != ends.match) {
        if (oppInSegPtr >= &oppInSeg[2])
            return IntersectResult::no;  // triggered by fuzz763_2b
        *oppInSegPtr++ = MatchEnds::start == ends.match 
                ? OpPtT(seg->c.firstPt(), 0) : OpPtT(seg->c.lastPt(), 1);
        if (segInOppPtr >= &segInOpp[2])
            return IntersectResult::no;
        *segInOppPtr++ = MatchEnds::start == ends.flipped() 
                ? OpPtT(opp->c.firstPt(), 0) : OpPtT(opp->c.lastPt(), 1);
    }
    if (oppInSegPtr - oppInSeg != 2)
        return IntersectResult::no;
    if (segInOppPtr - segInOpp != 2)
        return IntersectResult::no;
    if (oppInSeg[0].t > oppInSeg[1].t) {
        std::swap(oppInSeg[0], oppInSeg[1]);
        std::swap(segInOpp[0], segInOpp[1]);
    }
//    OP_ASSERT(oppInSeg[0].pt.isNearly(segInOpp[0].pt));
//    OP_ASSERT(oppInSeg[1].pt.isNearly(segInOpp[1].pt));
    OpPtT::MeetInTheMiddle(oppInSeg[0], segInOpp[0]);
    OpPtT::MeetInTheMiddle(oppInSeg[1], segInOpp[1]);
    OpVector segV = oppInSeg[1].pt - oppInSeg[0].pt;
    XyChoice larger = fabsf(segV.dx) > fabsf(segV.dy) ? XyChoice::inX : XyChoice::inY;
    if (oppInSeg[0].pt == oppInSeg[1].pt || segInOpp[0].pt == segInOpp[1].pt)
        return IntersectResult::no;
    return OpWinder::AddPair(larger, oppInSeg[0], oppInSeg[1], segInOpp[0], segInOpp[1],
	        segInOpp[0].t > segInOpp[1].t, seg, opp);
#endif
}

// note: ends have already been matched for consecutive segments
FoundIntersections OpSegments::findIntersections() {
    OP_DEBUG_CONTEXT();
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (seg->disabled)
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (opp->disabled)
                continue;
            // comparisons below need to be 'nearly' since adjusting opp may make sort incorrect
            // or, exact compare may miss nearly equal seg/opp pairs
            if (seg->closeBounds.right < opp->closeBounds.left)
                break;
            if (!seg->closeBounds.intersects(opp->closeBounds))
                continue;
            // set both to lines if they are linear before using them in t calculations
            (void) seg->c.isLine();
            (void) opp->c.isLine();
            AddEndMatches(seg, opp);
            if (seg->willDisable || opp->willDisable)
                continue;
            if (seg->isSmall()) {
                seg->willDisable = true;
                continue;
            }
            if (opp->isSmall()) {
                opp->willDisable = true;
                continue;
            }
            // for line-curve intersection we can directly intersect
            if (seg->c.isLine()) {
                if (opp->c.isLine()) {
                    IntersectResult lineCoin = LineCoincidence(seg, opp);
                    if (seg->disabled)
                        break;
                    if (IntersectResult::fail == lineCoin) {
                        OP_DEBUG_CODE(debugFailSegID = seg->id);
                        OP_DEBUG_CODE(debugFailOppID = opp->id);
                        return FoundIntersections::fail;
                    }
                    if (IntersectResult::coincident == lineCoin)
                        continue;
                }
                AddLineCurveIntersection(opp, seg);
                continue;
            } else if (opp->c.isLine()) {
                AddLineCurveIntersection(seg, opp);
                continue;
            }
            // if the bounds only share a corner, there's nothing more to do
            bool sharesHorizontal = seg->ptBounds.right == opp->ptBounds.left
                    || seg->ptBounds.left == opp->ptBounds.right;
            bool sharesVertical = seg->ptBounds.bottom == opp->ptBounds.top
                    || seg->ptBounds.top == opp->ptBounds.bottom;
            if (sharesHorizontal && sharesVertical)
                continue;
            // look for curve curve intersections (skip coincidence already found)
            OpCurveCurve cc(seg, opp);
            SectFound ccResult = cc.divideAndConquer();
#if OP_DEBUG_DUMP
            if (cc.dumpBreak(false))
                OP_ASSERT(0);
#endif
            if (true) { // SectFound::fail == ccResult || SectFound::maxOverlaps == ccResult
                        //        || SectFound::noOverlapDeep == ccResult
                // !!! as an experiment, search runs for small opp distances; turn found into limits
                SectFound limitsResult = cc.runsToLimits();
                if (SectFound::add == limitsResult)
                    ccResult = limitsResult;
            }
            if (SectFound::add == ccResult || cc.limits.size())
                cc.findUnsectable();
        }
        if (!seg->sects.i.size())
            seg->disabled = true;
    }
    return FoundIntersections::yes; // !!! if something can fail, return 'fail' (don't return 'no')
}
