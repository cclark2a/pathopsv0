#include "OpContour.h"
#include "OpCurveCurve.h"
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
    for (auto& contour : contours.contours) {
        for (auto& segment : contour.segments) {
            inX.push_back(&segment);
        }
    }
    std::sort(inX.begin(), inX.end(), compareXBox);
}

// !!! I'm bothered that curve / curve calls a different form of this with edges
void OpSegments::AddLineCurveIntersection(OpSegment* opp, OpSegment* seg) {
    OpRoots septs;
    OP_ASSERT(OpType::line == seg->c.type);
    LinePts edgePts { seg->c.pts[0], seg->c.pts[1] };
    septs = opp->c.rayIntersect(edgePts);
    bool reversed;
    MatchEnds common = seg->matchEnds(opp, &reversed, MatchSect::existing);
    if (!septs.count && MatchEnds::none == common)
        return; // IntersectResult::no;
    if (OpType::line == opp->c.type && MatchEnds::both == common) {
        seg->winding.move(opp->winding, seg->contour->contours, seg->c.pts[0] != opp->c.pts[0]);
        opp->winding.zero(ZeroReason::addIntersection);
        return; // IntersectResult::yes;
    }
    if (2 == septs.count && OpType::line == opp->c.type)
        return (void) OpWinder::CoincidentCheck({ seg->c.pts[0], 0 }, { seg->c.pts[1], 1 },
                { opp->c.pts[0], 0}, { opp->c.pts[1], 1 }, seg, opp );
    if ((int) MatchEnds::start & (int) common)
        septs.addEnd(reversed ? 1 : 0);
    if ((int) MatchEnds::end & (int) common)
        septs.addEnd(reversed ? 0 : 1);
    // !!! is sorting needed?
//    if (septs.count > 1)
//        std::sort(&septs.roots[0], &septs.roots[septs.count]);
    MatchEnds existingMatch = seg->matchExisting(opp);
    for (unsigned index = 0; index < septs.count; ++index) {
        OpPtT oppPtT { opp->c.ptAtT(septs.get(index)), septs.get(index) };
    // this fails if pt is outside the ptBounds, easily the case for axis aligned lines : ex. filinmangust14
	//	if (!seg->ptBounds.contains(oppPtT.pt))
	//		continue;
        float edgeT = seg->findPtT(0, 1, oppPtT.pt);
        if (OpMath::IsNaN(edgeT))
            continue;
        // pin point to both bounds, but only if it is on edge
 //       OP_DEBUG_CODE(OpPoint debugPt = oppPtT.pt);
        seg->ptBounds.pin(&oppPtT.pt);
 //       OP_ASSERT(debugPt == oppPtT.pt);	// triggered by test filinmangust14
        opp->ptBounds.pin(&oppPtT.pt);
 //       OP_ASSERT(debugPt == oppPtT.pt);	// triggered by test filinmangust14
        OpPtT edgePtT { oppPtT.pt, edgeT };
        if (MatchEnds::start == existingMatch && (edgePtT.pt == seg->c.pts[0] || 0 == edgeT
                || oppPtT.pt == opp->c.lastPt() || 1 == oppPtT.t))
            continue;
        if (MatchEnds::end == existingMatch && (edgePtT.pt == seg->c.lastPt() || 1 == edgeT
                || oppPtT.pt == opp->c.pts[0] || 0 == oppPtT.t))
            continue;
        OpIntersection* sect = seg->addSegSect(edgePtT  
                OP_DEBUG_PARAMS(SECT_MAKER(segmentLineCurve), SectReason::lineCurve, opp));
        OpIntersection* oSect = opp->addSegSect(oppPtT  
                OP_DEBUG_PARAMS(SECT_MAKER(segmentLineCurveOpp), SectReason::lineCurve, opp));
        sect->pair(oSect);
    }
}

void OpSegments::findCoincidences() {
    // take care of totally coincident segments
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (!seg->winding.visible())
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (!opp->winding.visible())
                continue;
            if (seg->ptBounds != opp->ptBounds)
                continue;
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed, MatchSect::allow);
            if (MatchEnds::both == match && seg->c.type == opp->c.type) {
                // if control points and weight match, treat as coincident: transfer winding
                bool coincident = false;
                switch (seg->c.type) {
                    case OpType::no:
                        OP_ASSERT(0);
                        break;
                    case OpType::line:
                        coincident = true;
                        break;
                    case OpType::quad:
                        coincident = seg->c.pts[1] == opp->c.pts[1];
                        break;
                    case OpType::conic:
                        coincident = seg->c.pts[1] == opp->c.pts[1]
                            && seg->c.weight == opp->c.weight;
                        break;
                    case OpType::cubic:
                        coincident = seg->c.pts[1] == opp->c.pts[1 + (int) reversed]
                            && seg->c.pts[2] == opp->c.pts[2 - (int) reversed];
                        break;
                }
                if (coincident) {
                    seg->winding.move(opp->winding, seg->contour->contours, reversed);
                    opp->winding.zero(ZeroReason::findCoincidences);
                    if (!seg->winding.visible() || !opp->winding.visible())
                        continue;
                }
            }
        }
    }
}

bool OpSegments::lineCoincidence(OpSegment* seg, OpSegment* opp) {
    OP_ASSERT(OpType::line == seg->c.type);
    OP_ASSERT(seg->winding.visible());
    OpVector tangent = seg->c.asLine().tangent();
    if (tangent.dx && tangent.dy)
        return false;
    OP_ASSERT(tangent.dx || tangent.dy);
    OP_ASSERT(OpType::line == opp->c.type);
    OP_ASSERT(opp->winding.visible());
    OpVector oTangent = opp->c.asLine().tangent();
    if (oTangent.dx && oTangent.dy)
        return false;
    OP_ASSERT(oTangent.dx || oTangent.dy);
    if (!tangent.dot(oTangent))  // if at right angles, skip
        return false;
    OP_ASSERT(seg->ptBounds.intersects(opp->ptBounds));
    seg->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(segSect)));
    opp->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(oppSect)));
    (void) OpWinder::CoincidentCheck(seg->edges.front(), opp->edges.front());
    return true;
}

// note: ends have already been matched for consecutive segments
FoundIntersections OpSegments::findIntersections() {
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (!seg->winding.visible())
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (!opp->winding.visible())
                continue;
            if (seg->ptBounds.right < opp->ptBounds.left)
                break;
            if (!seg->ptBounds.intersects(opp->ptBounds))
                continue;
            // for line-curve intersection we can directly intersect
            if (OpType::line == seg->c.type) {
                if (OpType::line == opp->c.type && lineCoincidence(seg, opp))
                    continue;
                AddLineCurveIntersection(opp, seg);
                continue;
            } else if (OpType::line == opp->c.type) {
                AddLineCurveIntersection(seg, opp);
                continue;
            }
            // check if segments share endpoints
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed, MatchSect::existing);
            OpIntersection* oppSect;
            if ((int) MatchEnds::start & (int) match) {
                auto sect = seg->addSegSect(OpPtT{ seg->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_start), 
                        SectReason::sharedEnd, opp));
                if (reversed)
                    oppSect = opp->addSegSect(OpPtT{ opp->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_startOppReversed),
                        SectReason::sharedEnd, seg));
                else
                    oppSect = opp->addSegSect(OpPtT{ opp->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_startOpp),
                        SectReason::sharedEnd, seg));
                sect->pair(oppSect);
            }
            if ((int) MatchEnds::end & (int) match) {
                auto sect = seg->addSegSect(OpPtT{ seg->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_end),
                        SectReason::sharedEnd, opp));
                if (reversed)
                    oppSect = opp->addSegSect(OpPtT{ opp->c.pts[0], 0 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_endOppReversed),
                        SectReason::sharedEnd, seg));
                else
                    oppSect = opp->addSegSect(OpPtT{ opp->c.lastPt(), 1 }
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_endOpp),
                        SectReason::sharedEnd, seg));
                sect->pair(oppSect);
            }
            // look for curve curve intersections (skip coincidence already found)
            seg->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(segSect)));
            opp->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(oppSect)));
            OpCurveCurve cc(&seg->edges.back(), &opp->edges.back());
            SectFound result = cc.divideAndConquer();
            if (SectFound::fail == result)
                OP_DEBUG_FAIL(*seg, FoundIntersections::fail);
        }
    }
    return FoundIntersections::yes; // !!! if something can fail, return 'fail' (don't return 'no')
}
