// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpIntersection.h"
#include "OpContour.h"
#include "OpSegment.h"

void OpIntersection::betweenPair(OpIntersection* end) {
    for (OpEdge& edge : segment->edges) {
        if (edge.start.t < ptT.t)
            continue;
        OP_ASSERT(ptT.t <= edge.start.t && edge.start.t < end->ptT.t);
        edge.setBetween();
        if (end->ptT.t == edge.end.t)
            break;
    }
}

OpIntersections::OpIntersections()
    : resort(false) {
}

OpIntersection* OpIntersections::add(OpIntersection* sect) {
    i.push_back(sect);
    resort = true;
    return sect;
}

OpIntersection* const * OpIntersections::entry(const OpPtT& ptT, const OpSegment* opp) const {
	for (unsigned index = 0; index < i.size(); ++index) {
        OpIntersection* sect = i[index];
        if (!sect->opp || sect->opp->segment != opp)
            continue;
		if (ptT.pt == sect->ptT.pt || ptT.t == sect->ptT.t)
			return &i[index];
	}
	return nullptr;
}

// count and sort extrema; create an edge for each extrema + 1
void OpIntersections::makeEdges(OpSegment* segment) {
    std::vector<const OpIntersection*> unsectables;
 //   start here (still thinking about it)
    // detect if segment is monotonic in x and y
    // if points are out of order, mark intersections as unsortable, between
    // generated edges should be linear, since control points can't be meaningful
    // !!! recompute points?
    const OpIntersection* last = i.front();
    OP_ASSERT(!resort);
    for (auto sectPtr : i) {
        const OpIntersection& sect = *sectPtr;
        if (sect.ptT.t != last->ptT.t) {
            OP_ASSERT(83 > segment->contour->contours->uniqueID);
            segment->edges.emplace_back(segment, last->ptT, sect.ptT
                    OP_DEBUG_PARAMS(EDGE_MAKER(makeEdges), last, sectPtr));
            OpEdge& newEdge = segment->edges.back();
            if (unsectables.size()) {
                const OpIntersection* unsectable = unsectables.back();
                int usectID = abs(unsectable->unsectID);
                // check if the bounds of the edge intersects the bounds of the pal's intersections
 //               OpPointBounds bounds;
 //               for (OpIntersection* pal : unsectable->opp->segment->sects.i) {
 //                   if (abs(pal->unsectID) == usectID)
 //                       bounds.add(pal->ptT.pt);
 //               }
 //               if (newEdge.ptBounds.intersects(bounds))
                newEdge.unsectableID = usectID;
                newEdge.palSet = true;  // defer setting pal until all edges are made
            }
            if (last->betweenID > 0)
                newEdge.setBetween();
        }
        if (sect.unsectID) {
            auto found = std::find_if(unsectables.begin(), unsectables.end(), 
                    [&sect](const OpIntersection* uT) { return uT->unsectID == sect.unsectID; });
            if (unsectables.end() == found)
                unsectables.push_back(&sect);
            else
                unsectables.erase(found);
        } else { // if we are inside a unsectable span, see if the pal has the other end
            OpIntersection* sectOpp = sect.opp;
            for (auto unsectable : unsectables) {
                 if (unsectable->ptT.t == sect.ptT.t)
                     continue;
                 bool foundStart = false;
                 for (OpIntersection* pal : unsectable->opp->segment->sects.i) {
                    if (pal->unsectID == unsectable->unsectID) {
                        if (foundStart)
                            break;
                        foundStart = true;
                        continue;
                    }
                    if (!foundStart)
                        continue;
                    OpIntersection* palOpp = pal->opp;
                    if (sectOpp->segment != palOpp->segment)
                        continue;
                    // found same segment intersections on both unsectables; mark as between
                    if (palOpp->ptT.t > sectOpp->ptT.t)
                        std::swap(palOpp, sectOpp);
                    int uID = abs(unsectable->unsectID);
                    palOpp->betweenID = uID;
                    sectOpp->betweenID = -uID;
                    // mark the edges, if they've been allocated, as betweeners
                    palOpp->betweenPair(sectOpp);
                    break;
                 }
            }
        }
        if (sect.ptT.t != last->ptT.t)
            last = &sect;
    }
}

const OpIntersection* OpIntersections::nearly(const OpPtT& ptT, OpSegment* oSeg) const {
	for (unsigned index = 0; index < i.size(); ++index) {
        OpIntersection* sect = i[index];
        if (oSeg && (!sect->opp || sect->opp->segment != oSeg))
            continue;
		if (ptT.pt.isNearly(sect->ptT.pt) 
                || (ptT.t - OpEpsilon <= sect->ptT.t && sect->ptT.t <= ptT.t + OpEpsilon))
			return sect;
	}
	return nullptr;
}

std::vector<OpIntersection*> OpIntersections::range(const OpSegment* opp) {
	if (resort)
	    sort();
    OP_DEBUG_CODE(float last = -1);
    std::vector<OpIntersection*> result;
    for (auto sect : i) {
            if (sect->opp && sect->opp->segment == opp) {
                OP_ASSERT(last < sect->ptT.t);
                OP_DEBUG_CODE(last = sect->ptT.t);
                result.push_back(sect);
            }
        }
    return result;
}

void OpIntersections::sort() {
    if (!resort) {
#if OP_DEBUG
        if (i.size()) {
            const OpIntersection* last = i.front();
            for (const auto sectPtr : i) {
                OP_ASSERT(last->ptT.t <= sectPtr->ptT.t);
                last = sectPtr;
            }
        }
#endif
        return;
    }
    // intersections may have multiple different t values with the same pt value
    // fix if detected here
    bool fixTs = false;
    std::sort(i.begin(), i.end(),
        [&fixTs](const OpIntersection* s1, const OpIntersection* s2) {
            if (s1->ptT.t != s2->ptT.t) {
                fixTs |= s1->ptT.pt == s2->ptT.pt;
                return s1->ptT.t < s2->ptT.t;
            }
            if (s1->unsectID || s2->unsectID) {
                int id1 = s1->unsectID * (MatchEnds::end == s1->unsectEnd ? -1 : 1);
                int id2 = s2->unsectID * (MatchEnds::end == s2->unsectEnd ? -1 : 1);
                return id1 < id2;
            }
            int id1 = abs(s1->coincidenceID) * (MatchEnds::end == s1->coinEnd ? -1 : 1);
            int id2 = abs(s2->coincidenceID) * (MatchEnds::end == s2->coinEnd ? -1 : 1);
            return id1 < id2;
        });
    if (!fixTs) {
        resort = false;
        return;
    }
    // should be rare; do an exhaustive search for duplicates
    for (unsigned outer = 1; outer < i.size(); ++outer) {
        OpIntersection* oSect = i[outer - 1];
        for (unsigned inner = outer; inner < i.size(); ++inner) {
            OpIntersection* iSect = i[inner];
            if (oSect->ptT.pt != iSect->ptT.pt)
                continue;
            if (oSect->ptT.t == iSect->ptT.t)
                continue;
            // At this point (probably because of curve/curve intersection)
            //  there are two or more intersections with the same point and different
            //  t values. The choice of which t value to use is entirely arbitrary.
            // Changing the t value may require resorting.
            OP_ASSERT(0 != oSect->ptT.t || 1 != iSect->ptT.t);
            if (0 == oSect->ptT.t)
                iSect->ptT.t = 0;
            else if (1 == iSect->ptT.t)
                oSect->ptT.t = 1;
            else
                oSect->ptT.t = iSect->ptT.t;
        }
    }
    sort();
}

void OpIntersections::windCoincidences(std::vector<OpEdge>& edges  
        OP_DEBUG_PARAMS(OpVector tangent)) {
    OP_ASSERT(!resort);
    std::vector<CoinPair> pairs;
    OpEdge* edge = &edges.front();
    for (auto sectPtr : i) {
        int coinID = sectPtr->coincidenceID;
        if (!coinID)
            continue;
        // start here; 
        if (sectPtr->coincidenceProcessed)
            continue;
        auto pairIter = std::find_if(pairs.begin(), pairs.end(), [coinID](CoinPair& pair) { 
            return coinID == pair.id; 
        });
        if (pairs.end() == pairIter) {  // set up start
            while (edge->start.t != sectPtr->ptT.t) {
                OP_ASSERT(edge->start.t < sectPtr->ptT.t);
                OP_ASSERT(edge < &edges.back());
                ++edge;
            }
            OP_ASSERT(edge->start.t == sectPtr->ptT.t);
            OpSegment* oppSegment = sectPtr->opp->segment;
            OP_ASSERT(tangent.dot(oppSegment->c.asLine().tangent())); 
            auto& oppEdges = oppSegment->edges;
            EdgeMatch match = coinID > 0 ? EdgeMatch::start : EdgeMatch::end;
            OpEdge* oppEdge = &oppEdges.front();
            while (oppEdge->ptT(match) != sectPtr->opp->ptT) {
                ++oppEdge;
                OP_ASSERT(oppEdge <= &oppEdges.back());
            }
            pairs.emplace_back(sectPtr, sectPtr->opp, edge, oppEdge, coinID
                    OP_DEBUG_PARAMS(EdgeMatch::start == match ? &oppEdges.back() : &oppEdges.front()));
        } else {    // set up end
            OP_ASSERT(!pairIter->end);
            pairIter->end = sectPtr;
            pairIter->oEnd = sectPtr->opp;
        }
    }    
    for (auto& coinPair : pairs) {
        coinPair.start->coincidenceProcessed = true;
        coinPair.end->coincidenceProcessed = true;
        coinPair.oStart->coincidenceProcessed = true;
        coinPair.oEnd->coincidenceProcessed = true;
        EdgeMatch match = coinPair.id > 0 ? EdgeMatch::start : EdgeMatch::end;
        OP_ASSERT(coinPair.oppEdge->ptT(match).pt == coinPair.start->ptT.pt);
        OP_ASSERT(coinPair.edge->start == coinPair.start->ptT);
        // In rare cases (e.g. issue1435) coincidence points may not match; one seg has extra.
        // Defer thinking about this if the winding is uniform. Assert to fix this in the future.
        edge = coinPair.edge;
        OpEdge* oppEdge = coinPair.oppEdge;
        OpEdge* edgeBack = edge;  // find final edge corresponding to final sect
        while (edgeBack->end.t < coinPair.end->ptT.t) {
            OP_ASSERT(edgeBack < &edges.back());
            ++edgeBack;
            // if assert, more code to write; add point to opp edge sect list to match 
            if (!(edge->winding == edgeBack->winding)) {  // example: testRect2
                // search intersection list for entry pointing to opp edge at edge end
                OP_DEBUG_CODE(auto& oI = oppEdge->segment->sects.i);
                OP_ASSERT(oI.end() != std::find_if(oI.begin(), oI.end(), 
                        [&edge](auto sect) { return sect->ptT.pt == edge->end.pt; }));
            }
        }
        OP_ASSERT(edgeBack->end == coinPair.end->ptT);
        OpEdge* oppBack = oppEdge;
        while (oppBack->ptT(Opposite(match)).t != coinPair.oEnd->ptT.t) {
            OP_ASSERT(coinPair.id > 0 ? oppBack < coinPair.lastEdge : oppBack > coinPair.lastEdge);
            coinPair.id > 0 ? ++oppBack : --oppBack;
            // more code to write; add point if needed to edge list to match
            if (!(oppEdge->winding == oppBack->winding)) {
                // search opp intersection list for entry pointing to edge at opp edge end
                OP_DEBUG_CODE(OpEdge* oEdge = coinPair.id > 0 ? oppEdge : oppBack);
                OP_ASSERT(i.end() != std::find_if(i.begin(), i.end(), 
                        [&oEdge](auto sect) { return sect->ptT.pt == oEdge->end.pt; }));
            }
        }
        OpContours* contours = edge->contours();
        // for each different winding: 
        int oppBump = coinPair.id < 0 ? -1 : 1;
        // surprisingly difficult to get right ...
        for (;;) {
            OP_DEBUG_CODE(OpWinding edgeWinding = edge->winding);
            OP_DEBUG_CODE(OpWinding oppWinding = oppEdge->winding);
            OP_ASSERT(edge->start.pt 
                    == oppEdge->ptT(coinPair.id > 0 ? EdgeMatch::start : EdgeMatch::end).pt);
            edge->winding.move(oppEdge->winding, contours, coinPair.id < 0);
            OpWinding combinedWinding = edge->winding;
            if (!combinedWinding.visible())
                edge->setDisabled(OP_DEBUG_CODE(ZeroReason::hvCoincidence1));
            else if (edge->disabled)
                edge->reenable();  // un-disable it; was disabled from earlier coincidence
            oppEdge->setDisabledZero(OP_DEBUG_CODE(ZeroReason::hvCoincidence2));
            for (;;) {
                OpPoint oppEnd = oppEdge->ptT(coinPair.id > 0 
                        ? EdgeMatch::end : EdgeMatch::start).pt;
                if (edge->end.pt == oppEnd)
                    break;
                bool oppInEdge = edge->ptBounds.contains(oppEnd);
                OP_DEBUG_CODE(bool edgeInOpp = oppEdge->ptBounds.contains(edge->end.pt));
                OP_ASSERT(oppInEdge != edgeInOpp);
                if (oppInEdge) {
                    oppEdge += oppBump;
                    OP_ASSERT(oppWinding == oppEdge->winding);
                    oppEdge->setDisabledZero(OP_DEBUG_CODE(ZeroReason::hvCoincidence3));
                } else {
                    ++edge;
                    OP_ASSERT(edgeWinding == edge->winding);
                    if (combinedWinding.visible()) {
                        edge->winding = combinedWinding;
                        if (edge->disabled)
                            edge->reenable();
                    } else
                        edge->setDisabledZero(OP_DEBUG_CODE(ZeroReason::hvCoincidence4));
                }
            }
            if (edge == edgeBack) {
                OP_ASSERT(oppEdge == oppBack);
                break;
            }
            ++edge;
            OP_ASSERT(oppEdge != oppBack);
            oppEdge += oppBump;
        }
    }
}

#if OP_DEBUG_VALIDATE
void OpIntersection::debugValidate() const {
    OP_ASSERT(OpMath::Between(0, ptT.t, 1));
    OpPoint pt = segment->c.ptAtT(ptT.t);
    OpMath::DebugCompare(pt, ptT.pt);
    OpPoint oPt = opp->segment->c.ptAtT(opp->ptT.t);
    OpMath::DebugCompare(pt, oPt);
}
#endif

#if OP_DEBUG
void OpIntersection::debugSetID() {
    id = segment->nextID();
}

bool OpIntersections::debugContains(const OpPtT& ptT, const OpSegment* opp) const {
    for (auto sect : i) {
        if ((sect->ptT.pt == ptT.pt || sect->ptT.t == ptT.t) 
                && sect->opp && sect->opp->segment == opp)
            return true;
    }
    return false;
}

OpIntersection* OpIntersections::debugAlreadyContains(const OpPoint& pt, const OpSegment* oppSegment) const {
	for (auto sectPtr : i) {
		const OpIntersection& sect = *sectPtr;
        if (!sect.opp)
            continue;
		if (oppSegment == sect.opp->segment && (pt == sect.ptT.pt || pt == sect.ptT.pt))
			return sectPtr;
	}
	return nullptr;
}
#endif
