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

OpIntersection* OpIntersections::alreadyContains(const OpPtT& ptT, const OpSegment* oppSegment) const {
	for (auto sectPtr : i) {
		const OpIntersection& sect = *sectPtr;
        if (!sect.opp)
            continue;
		if (oppSegment == sect.opp->segment && (ptT.pt == sect.ptT.pt || ptT.t == sect.ptT.t))
			return sectPtr;
	}
	return nullptr;
}

bool OpIntersections::contains(const OpPtT& ptT, const OpSegment* opp) const {
    for (auto sect : i) {
        if (sect->opp->segment != opp)
            continue;
        if (sect->ptT.t == ptT.t || sect->ptT.pt == ptT.pt)
            return true;
    }
    return false;
}

// count and sort extrema; create an edge for each extrema + 1
void OpIntersections::makeEdges(OpSegment* segment) {
    std::vector<const OpIntersection*> unsectables;
 //   start here;
    // detect if segment is monotonic in x and y
    // if points are out of order, mark intersections as unsortable, between
    // generated edges should be linear, since control points can't be meaningful
    // !!! recompute points?
    const OpIntersection* last = i.front();
    OP_ASSERT(!resort);
    for (auto sectPtr : i) {
        const OpIntersection& sect = *sectPtr;
//        if (512 == abs(sect.unsectID)) OP_DEBUG_BREAK();
        if (sect.ptT.t != last->ptT.t) {
            segment->edges.emplace_back(segment, last->ptT, sect.ptT
                    OP_DEBUG_PARAMS(EDGE_MAKER(makeEdges), last, sectPtr));
            OpEdge& newEdge = segment->edges.back();
            if (unsectables.size()) {
                const OpIntersection* unsectable = unsectables.back();
                int unsectableID = abs(unsectable->unsectID);
                // check if the bounds of the edge intersects the bounds of the pal's intersections
  //              OpPointBounds bounds;
 //               for (OpIntersection* pal : unsectable->opp->segment->sects.i) {
 //                   if (abs(pal->unsectID) == unsectableID)
 //                       bounds.add(pal->ptT.pt);
 //               }
 //               if (newEdge.ptBounds.intersects(bounds))
                    newEdge.unsectableID = unsectableID;
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

std::vector<OpIntersection*> OpIntersections::range(const OpSegment* opp) {
	if (resort)
	    sort();
    OP_DEBUG_CODE(float last = -1);
    std::vector<OpIntersection*> result;
    for (auto sect : i) {
            if (sect->opp->segment == opp) {
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
            int id1 = s1->unsectID * (s1->unsectEnd ? -1 : 1);
            int id2 = s2->unsectID * (s2->unsectEnd ? -1 : 1);
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

struct CoinPair {
    CoinPair(OpEdge* o, OpEdge* l, int id)
        : opp(o)
        , last(l)
        , coinID(id) 
        , processed(false) {
    }
    OpEdge* opp;
    OpEdge* last;
    int coinID;
    bool processed;
};

void OpIntersections::windCoincidences(OpVector tangent, std::vector<OpEdge>& edges) {
    OP_ASSERT(!resort);
    std::vector<CoinPair> coinPairs;
    OpEdge* edge = &edges.front();
    for (auto sectPtr : i) {
        if (1 == sectPtr->ptT.t)
            break;
        if (edge->start.t != sectPtr->ptT.t) {
            OP_ASSERT(edge->start.t < sectPtr->ptT.t);
            OP_ASSERT(edge < &edges.back());
            ++edge;
            OP_ASSERT(edge->start.t == sectPtr->ptT.t);
            for (auto& coinPair : coinPairs) {
                if (coinPair.opp == coinPair.last)
                    continue;
                coinPair.coinID > 0 ?  ++coinPair.opp : --coinPair.opp;
                coinPair.processed = false;
            }
        }
        int coinID = sectPtr->coincidenceID;
        if (coinID) {
            auto pairID = [coinID](CoinPair& pair) { 
                return coinID == pair.coinID; 
            };
            auto coinIter = std::find_if(coinPairs.begin(), coinPairs.end(), pairID);
            if (coinPairs.end() != coinIter) {
                coinPairs.erase(coinIter);
            } else {
                OpSegment* oppSegment = sectPtr->opp->segment;
                OP_ASSERT(tangent.dot(oppSegment->c.asLine().tangent())); 
                auto& oppEdges = oppSegment->edges;
                EdgeMatch match = coinID > 0 ? EdgeMatch::start : EdgeMatch::end;
                OpEdge* oppEdge = &oppEdges.front();
                while (oppEdge->ptT(match) != sectPtr->opp->ptT) {
                    ++oppEdge;
                    OP_ASSERT(oppEdge <= &oppEdges.back());
                }
                coinPairs.emplace_back(oppEdge, EdgeMatch::start == match ?
                        &oppEdges.back() : &oppEdges.front(), coinID);
            }
        } else if (coinPairs.empty())
            continue;
        if (edge->disabled)
            continue;
        for (auto& coinPair : coinPairs) {
            if (coinPair.processed)
                continue;
            EdgeMatch match = coinPair.coinID > 0 ? EdgeMatch::start : EdgeMatch::end;
            if (coinPair.opp->ptT(match).pt != sectPtr->ptT.pt)
                continue;
            OP_ASSERT(edge->start.pt == coinPair.opp->ptT(match).pt);
            OP_ASSERT(edge->end.pt == coinPair.opp->ptT(Opposite(match)).pt);
            OpContours* contours = edge->segment->contour->contours;
            edge->winding.move(coinPair.opp->winding, contours, coinPair.coinID < 0);
            if (!edge->winding.visible())
                edge->setDisabled(OP_DEBUG_CODE(ZeroReason::hvCoincidence));
            coinPair.opp->winding.zero();
            coinPair.opp->setDisabled(OP_DEBUG_CODE(ZeroReason::hvCoincidence));
            coinPair.processed = true;
        }
    }
}

#if OP_DEBUG

void OpIntersection::debugSetID() {
    id = segment->nextID();
}

void OpIntersection::debugValidate() const {
    OP_ASSERT(OpMath::Between(0, ptT.t, 1));
    OpPoint pt = segment->c.ptAtT(ptT.t);
    OpMath::DebugCompare(pt, ptT.pt);
    OpPoint oPt = opp->segment->c.ptAtT(opp->ptT.t);
    OpMath::DebugCompare(pt, oPt);
}

bool OpIntersections::debugContains(const OpPtT& ptT, const OpSegment* opp) const {
    for (auto sect : i) {
        if ((sect->ptT.pt == ptT.pt || sect->ptT.t == ptT.t) 
                && sect->opp->segment == opp)
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
