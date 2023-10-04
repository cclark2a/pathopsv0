// (c) 2023, Cary Clark cclark2@gmail.com
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
    OP_ASSERT(opp != seg);
    OP_ASSERT(OpType::line == seg->c.type);
    LinePts edgePts { seg->c.pts[0], seg->c.pts[1] };
    OpDebugBreakIf(opp, 54, 14 == seg->id);
    OpRoots septs = opp->c.rayIntersect(edgePts);
    bool reversed;
    MatchEnds common = seg->matchEnds(opp, &reversed, nullptr, MatchSect::existing);
    if (!septs.count && MatchEnds::none == common)
        return;
    if (OpType::line == opp->c.type && MatchEnds::both == common) {
        seg->winding.move(opp->winding, seg->contour->contours, seg->c.pts[0] != opp->c.pts[0]);
        if (!seg->winding.visible())
            seg->setDisabled(OP_DEBUG_CODE(ZeroReason::addIntersection));
        opp->winding.zero();
        opp->setDisabled(OP_DEBUG_CODE(ZeroReason::addIntersection));
        return;
    }
    if (2 == septs.count && OpType::line == opp->c.type)
        return (void) OpWinder::CoincidentCheck({ seg->c.pts[0], 0 }, { seg->c.pts[1], 1 },
                { opp->c.pts[0], 0}, { opp->c.pts[1], 1 }, seg, opp );
    if ((int) MatchEnds::start & (int) common)
        septs.addEnd(reversed ? 1 : 0);
    if ((int) MatchEnds::end & (int) common)
        septs.addEnd(reversed ? 0 : 1);
    MatchEnds existingMatch = seg->matchExisting(opp);
    std::vector<OpPtT> oppPtTs;
    std::vector<OpPtT> edgePtTs;
    septs.prioritize01();
    for (unsigned index = 0; index < septs.count; ++index) {
        oppPtTs.emplace_back(opp->c.ptAtT(septs.get(index)), septs.get(index));
        OpPtT& oppPtT = oppPtTs.back();
        float edgeT = seg->findPtT(0, 1, oppPtT.pt);
        if (OpMath::IsNaN(edgeT))
            continue;
        seg->ptBounds.pin(&oppPtT.pt);
        opp->ptBounds.pin(&oppPtT.pt);
        edgePtTs.emplace_back(oppPtT.pt, edgeT);
        OpPtT& edgePtT = edgePtTs.back();
        if ((int) MatchEnds::start & (int) existingMatch && (edgePtT.pt == seg->c.pts[0] || 0 == edgeT
                || oppPtT.pt == opp->c.lastPt() || 1 == oppPtT.t))
            continue;
        if ((int) MatchEnds::end & (int) existingMatch && (edgePtT.pt == seg->c.lastPt() || 1 == edgeT
                || oppPtT.pt == opp->c.pts[0] || 0 == oppPtT.t))
            continue;
        for (size_t earlier = 1; earlier < oppPtTs.size(); ++earlier) {
            if (oppPtTs[earlier - 1].t == oppPtT.t)
                goto duplicate;
        }
        for (size_t earlier = 1; earlier < edgePtTs.size(); ++earlier) {
            if (edgePtTs[earlier - 1].t == edgePtT.t)
                goto duplicate;
        }
        {
            OpIntersection* sect = seg->addSegSect(edgePtT, opp  
                    OP_DEBUG_PARAMS(SECT_MAKER(segmentLineCurve), SectReason::lineCurve));
            OpIntersection* oSect = opp->addSegSect(oppPtT, seg  
                    OP_DEBUG_PARAMS(SECT_MAKER(segmentLineCurveOpp), SectReason::lineCurve));
            sect->pair(oSect);
        }
duplicate: ;
    }
}

void OpSegments::findCoincidences() {
    // take care of totally coincident segments
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (seg->disabled)
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (opp->disabled)
                continue;
            if (seg->ptBounds != opp->ptBounds)
                continue;
            bool reversed;
            MatchEnds match = seg->matchEnds(opp, &reversed, nullptr, MatchSect::allow);
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
                    if (!seg->winding.visible())
                        seg->setDisabled(OP_DEBUG_CODE(ZeroReason::findCoincidences));
                    opp->winding.zero();
                    opp->setDisabled(OP_DEBUG_CODE(ZeroReason::findCoincidences));
                }
            }
        }
    }
}

IntersectResult OpSegments::lineCoincidence(OpSegment* seg, OpSegment* opp) {
    OP_ASSERT(OpType::line == seg->c.type);
    OP_ASSERT(!seg->disabled);
    OpVector tangent = seg->c.asLine().tangent();
    if (tangent.dx && tangent.dy)
        return IntersectResult::no;
    OP_ASSERT(tangent.dx || tangent.dy);
    OP_ASSERT(OpType::line == opp->c.type);
    OP_ASSERT(!opp->disabled);
    OpVector oTangent = opp->c.asLine().tangent();
    if (oTangent.dx && oTangent.dy)
        return IntersectResult::no;
    OP_ASSERT(oTangent.dx || oTangent.dy);
    if (!tangent.dot(oTangent))  // if at right angles, skip
        return IntersectResult::no;
    OP_ASSERT(seg->ptBounds.intersects(opp->ptBounds));
    seg->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(segSect)));
    opp->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(oppSect)));
    return OpWinder::CoincidentCheck(seg->edges.front(), opp->edges.front());
}

// note: ends have already been matched for consecutive segments
FoundIntersections OpSegments::findIntersections() {
    for (auto segIter = inX.begin(); segIter != inX.end(); ++segIter) {
        OpSegment* seg = const_cast<OpSegment*>(*segIter);
        if (seg->disabled)
            continue;
        for (auto oppIter = segIter + 1; oppIter != inX.end(); ++oppIter) {
            OpSegment* opp = const_cast<OpSegment*>(*oppIter);
            if (opp->disabled)
                continue;
            if (seg->ptBounds.right < opp->ptBounds.left)
                break;
            if (!seg->ptBounds.intersects(opp->ptBounds))
                continue;
            // for line-curve intersection we can directly intersect
            if (OpType::line == seg->c.type) {
                if (OpType::line == opp->c.type) {
                    IntersectResult lineCoin = lineCoincidence(seg, opp);
                    if (IntersectResult::fail == lineCoin)
                        return FoundIntersections::fail;
                    if (IntersectResult::yes == lineCoin)
                        continue;
                }
                AddLineCurveIntersection(opp, seg);
                continue;
            } else if (OpType::line == opp->c.type) {
                AddLineCurveIntersection(seg, opp);
                continue;
            }
            // check if segments share endpoints
            bool reversed;
            MatchEnds existing;
            MatchEnds match = seg->matchEnds(opp, &reversed, &existing, MatchSect::existing);
            OpIntersection* oppSect;
            if ((int) MatchEnds::start & (int) match) {
                auto sect = seg->addSegSect(OpPtT{ seg->c.pts[0], 0 }, opp
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_start), 
                        SectReason::sharedEnd));
                if (sect) {
                    if (reversed)
                        oppSect = opp->addSegSect(OpPtT{ opp->c.lastPt(), 1 }, seg
                            OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_startOppReversed),
                            SectReason::sharedEnd));
                    else
                        oppSect = opp->addSegSect(OpPtT{ opp->c.pts[0], 0 }, seg
                            OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_startOpp),
                            SectReason::sharedEnd));
                    OP_ASSERT(oppSect);
                    sect->pair(oppSect);
                }
            }
            if ((int) MatchEnds::end & (int) match) {
                auto sect = seg->addSegSect(OpPtT{ seg->c.lastPt(), 1 }, opp
                        OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_end),
                        SectReason::sharedEnd));
                if (sect) {
                    if (reversed)
                        oppSect = opp->addSegSect(OpPtT{ opp->c.pts[0], 0 }, seg
                            OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_endOppReversed),
                            SectReason::sharedEnd));
                    else
                        oppSect = opp->addSegSect(OpPtT{ opp->c.lastPt(), 1 }, seg
                            OP_DEBUG_PARAMS(SECT_MAKER(findIntersections_endOpp),
                            SectReason::sharedEnd));
                    OP_ASSERT(oppSect);
                    sect->pair(oppSect);
                }
            }
            // if the bounds only share a corner, there's nothing more to do
            bool sharesHorizontal = seg->ptBounds.right == opp->ptBounds.left
                    || seg->ptBounds.left == opp->ptBounds.right;
            bool sharesVertical = seg->ptBounds.bottom == opp->ptBounds.top
                    || seg->ptBounds.top == opp->ptBounds.bottom;
            if (sharesHorizontal && sharesVertical)
                continue;
            // if the bounds share only an edge, and ends match, there's nothing more to do
            if ((sharesHorizontal || sharesVertical) && MatchEnds::none != existing)
                continue;
            // look for curve curve intersections (skip coincidence already found)
            seg->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(segSect)));
            opp->makeEdge(OP_DEBUG_CODE(EDGE_MAKER(oppSect)));
            OpCurveCurve cc(&seg->edges.back(), &opp->edges.back());
            SectFound result = cc.divideAndConquer();
            if (SectFound::fail == result)
                return FoundIntersections::fail;  // triggered by fuzzhang_1
        }
    }
    return FoundIntersections::yes; // !!! if something can fail, return 'fail' (don't return 'no')
}
