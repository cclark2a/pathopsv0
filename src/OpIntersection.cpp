// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpIntersection.h"
#include "OpContext.h"
#include "OpSegment.h"

void OpIntersection::pair(OpIntersection* o) {
	OP_ASSERT(abs(unsectID) == abs(o->unsectID)); 
	OP_ASSERT(coincidenceID == o->coincidenceID); 
#if 0  // !!! need looser compare to allow for intersection error  testCubics3519581
// !!! ... or, make sects common in merge intersection, and remove need for meet-in-the-middle
	OP_ASSERT(ptT.pt.isNearly(o->ptT.pt, segment->threshold()) 
			|| (!!unsectID && !!o->unsectID) || !opp);
	if (!unsectID && !o->unsectID && ptT.pt != o->ptT.pt 
			/* !!! && ptT.pt.isNearly(o->ptT.pt, segment->threshold()) */ )
		OpPtT::MeetInTheMiddle(ptT, o->ptT);
#endif
	opp = o;
	o->opp = this;
}

void OpIntersection::setCoin(int cid, MatchEnds end, CoinOpp co) {
	coincidenceID = cid;
	coinEnd = end;
	coinOpp = co;
	segment->hasCoin = true;
	segment->sects.hasPairs = true;
	segment->sects.unsorted = true;
}

// returns true if merge end points needs to run again
bool OpIntersection::setMerge(int masterID, OpPoint masterPt, MergeType mergeType) {
	mergeID = masterID;
	ptT.pt = masterPt;
	if (MergeType::midPoint == mergeType) {
		if (opp->unsectID)
			return false;
		if (opp->mergeID == masterID && opp->ptT.pt == masterPt)
			return true;
	}
	if (opp->mergeID && opp->mergeID != masterID)
		opp->segment->mergeMultiple(masterPt, masterID, opp->ptT.pt, opp->mergeID);
	else {
//		OP_ASSERT(MergeType::endPoint != mergeType || !opp->mergeID);
		opp->mergeID = masterID;
		opp->ptT.pt = masterPt;
	}
    if (MergeType::midPoint == mergeType && !opp->segment->disabled)
	    opp->segment->setUnmerged();
	return true;
}

OpRect OpIntersection::setMergeBounds(OpVector threshold) {
	OpRect result { ptT.pt, callerPt };
	if (!unsectID) {
		result.add(opp->ptT.pt);
		result.add(opp->callerPt);
	}
	OpVector wh = result.widthHeight();
	if (wh.dx < threshold.dx) {
		result.left = (result.left + result.right - threshold.dx) / 2;
		result.right = result.left + threshold.dx;
	}
	if (wh.dy < threshold.dy) {
		result.top = (result.top + result.bottom - threshold.dy) / 2;
		result.bottom = result.top + threshold.dy;
	}
	return result;
}

void OpIntersection::setUnsect(int uid, MatchEnds end) {
	unsectID = uid;
	unsectEnd = end;
	segment->hasUnsectable = true;
	segment->sects.hasPairs = true;
	segment->sects.unsorted = true;
}

#if 0
OpIntersection* OpIntersection::coinOtherEnd() {
	auto endSect = std::find_if(segment->sects.i.begin(), segment->sects.i.end(), 
			[this](const OpIntersection* test) { 
			return test->coincidenceID == coincidenceID && test->coinEnd == !coinEnd; });
	OP_ASSERT(segment->sects.i.end() != endSect);
	return *endSect;
};
#endif

OpIntersection* OpIntersections::add(OpIntersection* sect) {
	i.push_back(sect);
	unsorted = true;
	return sect;
}

void OpIntersections::clear() {
    i.clear();
	unsorted = false;
	hasCCSects = false;
	hasPairs = false;
}

// this matches opp with nearby ptT
OpIntersection* OpIntersections::contains(const OpPtT& ptT, const OpSegment* opp) const {
	for (unsigned index = 0; index < i.size(); ++index) {
		OpIntersection* sect = i[index];
		OpIntersection* oppSect = sect->opp;
		if (!oppSect || oppSect->segment != opp)
			continue;
		if (ptT.isNearly(sect->ptT, opp->threshold()))
			return sect;
		if (ptT.pt.isNearly(oppSect->ptT.pt, opp->threshold()))
			return sect;
	}
	return nullptr;
}

// returns point on matching segment within threshold, if any
// treat coincident range as containing pt
CloseBy OpIntersections::containsClose(OpPoint pt, OpVector threshold, const OpSegment* opp) const {
	OpIntersection* nearby = nullptr;
	OpPoint coinStart;
	for (unsigned index = 0; index < i.size(); ++index) {
		OpIntersection* sect = i[index];
		if (!sect->opp || sect->opp->segment != opp)
			continue;
		if (pt.isNearly(sect->ptT.pt, threshold)) {
			if (pt == sect->ptT.pt)
				return { sect, NearBy::equal };
			if (nearby) {
				float nearbyDist = (pt - nearby->ptT.pt).lengthSquared();  // choose closer
				float sectDist = (pt - sect->ptT.pt).lengthSquared();
				if (nearbyDist > sectDist)
					nearby = sect;
			} else
				nearby = sect;
		}
		if (!sect->coincidenceID)
			continue;
		if (MatchEnds::start == sect->coinEnd) {
			coinStart = sect->ptT.pt;
			continue;
		}
		OpPointBounds coinBounds { coinStart, sect->ptT.pt };
		if (coinBounds.nearlyContains(pt, threshold))
			return { sect, NearBy::far };  // return closing coin
	}
	return { nearby, nearby ? NearBy::nearby : NearBy::none };
}

OpIntersection* OpIntersections::coinContains(OpPoint pt, const OpSegment* opp, OpPtT* nearby) const {
	OpIntersection* match = coinContains(pt, opp);
	if (match)
		return match;
	OpVector thresh = opp->threshold();
	// return exact match first
	for (OpIntersection* sect : i) {
		bool sectNearby = sect->ptT.pt.isNearly(pt, thresh);
		if (sectNearby)
			*nearby = sect->ptT;
		if (!sect->opp || sect->opp->segment != opp)
			continue;
		if (sect->coincidenceID)
			return sect;
		if (sectNearby)
			match = sect;
	}
	if (match)
		*nearby = match->ptT;
	return match;
}

OpIntersection* OpIntersections::coinContains(OpPoint pt, const OpSegment* opp) const {
	for (OpIntersection* sect : i) {
		if (!sect->coincidenceID || !sect->opp || sect->opp->segment != opp)
			continue;
		if (sect->ptT.pt == pt)
			return sect;
	}
	return nullptr;
}

void OpIntersections::collectMatchingPts(OpPoint pt, std::vector<OpPoint>& pts) const {
	for (const OpIntersection* sect : i) {
		if (sect->ptT.pt != pt)
			continue;
		OpPoint oppPt = sect->opp->ptT.pt;
		if (pts.end() == std::find(pts.begin(), pts.end(), oppPt))
			pts.push_back(oppPt);
	}
}

#if 0
// this matches sects exactly, and is for pair lookups
OpIntersection* const * OpIntersections::entry(const OpPtT& ptT, const OpSegment* opp) const {
	for (unsigned index = 0; index < i.size(); ++index) {
		OpIntersection* sect = i[index];
		if (!sect->opp || sect->opp->segment != opp)
			continue;
		if (ptT.pt == sect->ptT.pt || ptT.t == sect->ptT.t)  // !!! should this be && ?
			return &i[index];
	}
	return nullptr;
}
#endif

static void stackCoins(std::vector<CoinPal>& coincidences, OpIntersection* sect) {
	int coinID = sect->coincidenceID;
	if (!coinID)
		return;
	if (MatchEnds::start == sect->coinEnd) {
		coincidences.push_back({ sect->opp->segment, coinID /*, Transfer::none */ });
		return;
	}
	OP_ASSERT(MatchEnds::end == sect->coinEnd);
	auto found = std::find_if(coincidences.begin(), coincidences.end(), [coinID]
			(const CoinPal& cPal) { return cPal.coinID == coinID; });
	OP_ASSERT(coincidences.end() != found);
	coincidences.erase(found);
}

// only capture coin if start and end t values enclose test value
static void findStack(std::vector<CoinPal>& coincidences, OpIntersection* opp) {
	float oppT = opp->opp->ptT.t;
	if (OpMath::NearlyEndT(oppT))
		return;
	for (OpIntersection* oppSect : opp->opp->segment->sects.i) {
		if (oppSect->ptT.t > oppT)
			break;
		if (oppSect->ptT.t < oppT || MatchEnds::end == oppSect->coinEnd)
			stackCoins(coincidences, oppSect);
	}
}

// if the edge is inside an unsectable range, record all sects that start that range
void OpIntersections::makeEdges(OpSegment* segment) {
	OP_ASSERT(!unsorted);
	std::vector<OpIntersection*> unsectables;
	auto stackUnsects = [&unsectables](OpIntersection* sect) {
		if (!sect->unsectID)
			return;
		if (MatchEnds::start == sect->unsectEnd) {
			unsectables.push_back(sect);
			return;
		}
		OP_ASSERT(MatchEnds::end == sect->unsectEnd);
		auto found = std::find_if(unsectables.begin(), unsectables.end(), 
				[sect](const OpIntersection* uT) { return uT->unsectID == sect->unsectID; });
		OP_ASSERT(unsectables.end() != found);
		unsectables.erase(found);
	};
	std::vector<CoinPal> coincidences;
	OpIntersection* first = i.front();
	for (OpIntersection* sectPtr : i) {
		if (first->ptT.t != sectPtr->ptT.t) {
			segment->edges.emplace_back(first, sectPtr  OP_LINE_FILE_PARGS());
			OpEdge& newEdge = segment->edges.back();
			newEdge.unsectableStart = !!first->unsectID;
			newEdge.unsectableEnd = !!sectPtr->unsectID;
#if 1   // old code breaks skpagentxsites_com55 / though loops61i works
		// old code did not check if coincident pair are from same coincidence (same coin id)
		// new code: if edge is between a pair of coincident edges, mark it unsortable
			if (first->betweenCoins && sectPtr->betweenCoins 
					&& first->opp->segment != sectPtr->opp->segment) {
				std::vector<CoinPal> firstCoins;
				// !!! optimization: could do cheaper nearly end test on sect ptr first ...
				findStack(firstCoins, first);
				if (!firstCoins.empty()) {
					std::vector<CoinPal> sectCoins;
					findStack(sectCoins, sectPtr);
					if (!sectCoins.empty()) {
						for (CoinPal& coinPal : firstCoins) {
							if (sectCoins.end() != std::find_if(sectCoins.begin(), sectCoins.end(),
									[coinPal](CoinPal& test) {
									return coinPal.coinID == test.coinID; })) {
								newEdge.setUnsortable(Unsortable::betweenCoins);
								break;
							}
						}
					}
				}
			}
#endif
			first = sectPtr;
			if (!unsectables.empty())
				newEdge.unSects = unsectables;
			if (!coincidences.empty())
				newEdge.coinPals = coincidences;
		}
		stackUnsects(sectPtr);
		stackCoins(coincidences, sectPtr);
	}
}

// edge is coincident with an edge in opp, but hasn't been marked as such
// check existing for coincidence with same opp, so coincidence ranges may grow
// !!! optimization: if below lengthens coin run, it could combine/lengthen edges also
int OpIntersections::coinRange(OpEdge& edge, OpSegment* opp, bool reversed) {
	int coinID = 0;
	OpIntersection* edgeStart = nullptr;
	OpIntersection* edgeEnd = nullptr;
	OpIntersection* coinStart = nullptr;
	OpIntersection* coinEnd = nullptr;
	std::vector<int> mergeIDs;
	auto setCoin = [&coinID, reversed](OpIntersection* sect, MatchEnds matchEnd) {
		sect->setCoin(coinID, matchEnd, CoinOpp::no);
		MatchReverse matchReverse { matchEnd, reversed };
		sect->opp->setCoin(coinID, matchReverse.flipped(), CoinOpp::yes);
		return sect;
	};
	int result = 0;
	for (OpIntersection* sect : i) {
		if (sect->opp->segment != opp)
			continue;
		float t = sect->ptT.t;
		if (MatchEnds::start == sect->coinEnd) {
			OP_ASSERT(!coinStart);
			coinStart = sect;
			coinEnd = nullptr;
			int oldCoinID = coinID;
			coinID = sect->coincidenceID;
			if (edgeStart) {
//				OP_ASSERT(edgeStart->coincidenceID);
   				edgeStart->coincidenceID = edgeStart->opp->coincidenceID = coinID;
				OP_ASSERT(t > edge.startT);
				coinStart->zeroCoincidencePair();
				if (oldCoinID) {
					OP_ASSERT(oldCoinID != coinID);
					mergeIDs.push_back(oldCoinID);
				}
			}
		}
		if (MatchEnds::end == sect->coinEnd) {
			OP_ASSERT(coinStart);
			coinStart = nullptr;
			coinEnd = sect;
			if (edgeStart && !edgeEnd)
				coinEnd->zeroCoincidencePair();
			if (edgeEnd) {
				result = 0;
				break;
			}
		}
		if (t == edge.startT) {
			if (coinStart) 
				edgeStart = sect;
			else if (coinEnd) {
				edgeStart = sect;
				coinEnd->zeroCoincidencePair();
			} else {
				coinID = opp->nextID();
				edgeStart = setCoin(sect, MatchEnds::start);
			}
		} else if (t == edge.endT) {
			OP_ASSERT(coinStart || edgeStart);
			if (coinStart) 
				edgeEnd = sect;
			else {
				edgeEnd = setCoin(sect, MatchEnds::end);
				result = coinID;
				break;
			}
		}
		if (coinEnd) {
			coinEnd = nullptr;
			if (!edgeStart)
				coinID = 0;
		}
	}
	OP_ASSERT(!coinStart);
	for (int oldID : mergeIDs) {  // merge a pair of coins together if they now touch
		bool seenOld = false;
		bool seenNew = false;
		for (OpIntersection* sect : i) {
			if (oldID == sect->coincidenceID) {
				sect->zeroCoincidencePair();
				if (!seenNew)
					sect->coincidenceID = coinID;
				seenOld = true;
			} else if (coinID == sect->coincidenceID) {
				if (seenOld)
					sect->zeroCoincidencePair();
				seenNew = true;
			}
		}
	}
	return result;
}

std::vector<OpIntersection*> OpIntersections::unsectables(OpPoint pt) {
	std::vector<OpIntersection*> result;
	for (OpIntersection* sect : i) {
		if (sect->ptT.pt != pt)
			continue;
		if (!sect->unsectID)
			continue;
		result.push_back(sect);
	}
	return result;
}

#if 0
struct SectPreferred {
	SectPreferred(OpIntersection* sect)
		: best(sect)
		, bestOnEnd(sect->ptT.onEnd())
		, collapsed(false) {
		OP_DEBUG_CODE(int safetyValue = 10);  // !!! no idea what this should be
		do {
            PrefFound found = find();
			if (PrefFound::retry != found)  // repeat once if better point was found
				return;
			visited.clear();
			OP_ASSERT(--safetyValue);
		} while (true);
	}

//	PrefFound find();  // make consecutive nearly equal sects the same, and follow each opposite

	std::vector<OpSegment*> visited; // visit each segment with matching sects once
	OpIntersection* best;  // an end point, if one exists; otherwise, an arbitrary sect
	bool bestOnEnd;  // never cleared, only set (once first sect on end is found)
	bool collapsed;
};

PrefFound SectPreferred::find() {
	OpSegment* seg = best->segment;
    if (seg->disabled)
        return PrefFound::disabled;
    OpIntersections& sects = seg->sects;
	visited.push_back(seg);
	bool sawBest = false;
	for (OpIntersection* test : sects.i) {
		if (!test->ptT.isNearly(best->ptT, seg->threshold())) {
			if (sawBest)
				break;
			continue;
		}
		test->mergeProcessed = true;  // skip this when seen by merge near
		if (test == best) {
			sawBest = true;
			if (sects.i.back()->ptT.isNearly(best->ptT, seg->threshold())) {
// small segment may have first and last nearly touching
#if 0
				if (0 == best->ptT.t) {
					seg->setDisabled(OP_LINE_FILE_NPARGS());
					return false;
				}
#endif
				OP_ASSERT(1 == sects.i.back()->ptT.t);
				if (1 != best->ptT.t) {
					best->ptT.t = 1;
					sects.unsorted = true;
				}
			}
		} else if (test->ptT != best->ptT) {
	#if 0
			if (test->ptT.onEnd()) {
				if (bestOnEnd) {
					seg->movePt(test->ptT, best->ptT.pt);  // !!! trace through -- may need rewrite
				} else {
					best = test;
					bestOnEnd = true;
				}
				return false; // best changed; start over
			}
	#endif
			if (test->ptT.pt != best->ptT.pt) {
                PrefFound found = seg->moveSects(test->ptT, best->ptT.pt);
				if (PrefFound::ok != found)
					return found;
			} else if (test->ptT.t != best->ptT.t) {
				test->ptT.t = best->ptT.t;
				sects.unsorted = true;
			}
		}
		if (visited.end() == std::find(visited.begin(), visited.end(), test->opp->segment)) {
			OpIntersection* save = best;
			best = test->opp;
            PrefFound found = find();
			if (PrefFound::disabled == found)
				return found;
			best = save;
		}
	}
	return PrefFound::ok;
}
#endif

// wait until sorting to order pairs; ordering earlier may fail if sects are moved
void OpIntersections::orderPairs() {
	for (size_t index = 0; index + 1 < i.size(); ++index) {
		OpIntersection* sect = i[index];
		int cID = sect->coincidenceID;
		int uID = sect->unsectID;
		if (!cID && !uID)
			continue;
		for (size_t inner = index + 1; inner < i.size(); ++inner) {
			OpIntersection* test = i[inner];
			if (cID && cID == test->coincidenceID) {
				sect->coinEnd = MatchEnds::start;
				test->coinEnd = MatchEnds::end;
			}
			if (uID && uID == test->unsectID) {
				sect->unsectEnd = MatchEnds::start;
				test->unsectEnd = MatchEnds::end;
			}
		}
	}
}

#if 01  // breaks skpagentxsites_com55 (add what test this is required for...
// mark intersections between coincident start and end which do not match same coin opposite
// first pass: mark possible between sects
// second pass: if opposite is also between sects, then neither is
void OpIntersections::markInCoincidence() {
	int coinCount = 0;
	for (OpIntersection* iSect : i) {
		if (MatchEnds::none != iSect->coinEnd)
			coinCount += MatchEnds::start == iSect->coinEnd ? 1 : -1;
		else if (coinCount)
			iSect->opp->betweenCoins = true;
	}
}
#endif

float OpIntersections::matchT(const OpPtT& match, OpPoint destination, MatchEnds matchEnds) const {
	float destT = MatchEnds::start == matchEnds ? 0 : MatchEnds::end == matchEnds ? 1 : match.t;
	if (0 < destT && destT < 1) {
		for (OpIntersection* sect : i) {
			if (sect->ptT.pt == destination) {
				destT = sect->ptT.t;
				break;
			}
		}
	}
	return destT;
}

// note that intersections may not be sorted
SectCleanup OpIntersections::moveSects(const OpPtT& match, OpPoint destination,
		MatchEnds matchEnds) {
	OP_ASSERT(match.pt != destination);  // !!! maybe this is correct if t needs changing ?
// before changing anything, determine if changing match.pt to destination collapses coin or unsect
//   either in this sect pair, or in the opposite sect pair
	std::vector<size_t> remove;
	std::vector<OpIntersection*> moveOpps;
	bool segmentCollapsed = false;
	float destT = matchT(match, destination, matchEnds);
	for (size_t index = 0; index < i.size(); ++index) {  // look for sects to move (match is equal)
		OpIntersection* toMove = i[index];
		if (toMove->collapsed && remove.end() == std::find(remove.begin(), remove.end(), index))
			remove.push_back(index);
		if (toMove->ptT.pt != match.pt)
			continue;
		bool keepToMove = true;
		size_t testIndex = SIZE_MAX;
		OpIntersection* test = nullptr;  // suppress compiler warning
		for (size_t inner = 0; inner < i.size(); ++inner) {  // look for coins, unsects, equal-to's
			test = i[inner];
			if (test->opp->segment != toMove->opp->segment)
				continue;
			if (test->ptT.pt == destination) {
				testIndex = inner;
				if ((0 == destT && 1 == test->ptT.t) || (0 == test->ptT.t && 1 == destT))
					segmentCollapsed = true;  // caller should disable segment
				if (toMove->collapsed || (!test->collapsed 
						&& ((!toMove->coincidenceID && test->coincidenceID)
						|| (!toMove->unsectID && test->unsectID))))
					keepToMove = false;
				break;
			}
		}
		OP_ASSERT(index != testIndex);
		if (testIndex == SIZE_MAX) {
			toMove->ptT = OpPtT(destination, destT);
			unsorted = true;
			if (toMove->opp->ptT.pt != destination)
				moveOpps.push_back(toMove->opp);
			continue;
		}
		OP_ASSERT(test);
		OpIntersection* keep = keepToMove ? toMove : test;
		float toMoveT = toMove->ptT.t;  // copy: to move's t will be adjusted
		for (size_t idx2 = 0; idx2 < i.size(); ++idx2) {
			// if there are two or more sects with same pt/t/seg, remove all but one
			OpIntersection* test2 = i[idx2];
			if (!OpMath::Between(toMoveT, test2->ptT.t, test->ptT.t))
				continue;
			if (test2->opp->segment == toMove->opp->segment && test2 != keep) { 
				if (remove.end() == std::find(remove.begin(), remove.end(), idx2))
					remove.push_back(idx2);
			}
			test2->ptT = OpPtT(destination, destT);
			unsorted = true;
			if (test2->opp->ptT.pt != destination)
				moveOpps.push_back(test2->opp);
		}
	}
	bool cleanup = remove.size();
	std::sort(remove.begin(), remove.end());
	// if sect to move is one end of coin/unsect, and movement causes span to collapse, remove end
	while (remove.size()) {
		size_t index = remove.back();
		OpIntersection* toErase = i[index];
		OP_DEBUG_CODE(toErase->debugErased = true);
		toErase->opp->collapsed = true;
		zeroPairs(toErase);
		OP_ASSERT(toErase->opp->ptT.pt == destination
				|| moveOpps.end() != std::find(moveOpps.begin(), moveOpps.end(), toErase->opp));
//		opp->ptT = OpPtT(destination, opp->segment->sects.matchT(opp->ptT, destination));
		toErase->opp = nullptr;
		i.erase(i.begin() + index);
		remove.pop_back();
	}
	// recurse on matching opposite intersection
	while (moveOpps.size()) {
		OpIntersection* opp = moveOpps.back();
		if (!opp->collapsed && opp->ptT.pt != destination)
			opp->segment->moveSects(opp->ptT, destination);
		moveOpps.pop_back();
	}
	return segmentCollapsed ? SectCleanup::segmentCollapsed : cleanup ? SectCleanup::sectsRemoved 
			: SectCleanup::none;
}

bool OpIntersections::simpleEnd() const {
	OP_ASSERT(!unsorted);
	OP_ASSERT(i.size() > 1);
	OP_ASSERT(i.back()->ptT.t == 1);
	return i[i.size() - 2]->ptT.t != 1;
}

bool OpIntersections::simpleStart() const {
	OP_ASSERT(!unsorted);
	OP_ASSERT(i.size() > 1);
	OP_ASSERT(i.front()->ptT.t == 0);
	return i[1]->ptT.t != 0;
}

void OpIntersections::sort() {
	if (!unsorted) {
#if 01 && OP_DEBUG  // !!! fails with fuzz763_378
		if (i.size()) {
			const OpIntersection* last = i.front();
			auto findEndIndex = [this](const OpIntersection* sect) {
				for (size_t index = 0; index < i.size(); ++index) {
					const OpIntersection* test = i[index];
					if (test->coincidenceID == sect->coincidenceID && MatchEnds::end == test->coinEnd)
						return index;
					if (test->unsectID == sect->unsectID && MatchEnds::end == test->unsectEnd)
						return index;
				}
				OP_ASSERT(0); // match not found
				return (size_t) 0;
			};
			for (const auto s : i) {
				OP_ASSERT(last->ptT.t <= s->ptT.t);
				if (last != s && last->ptT.t == s->ptT.t
					   && (MatchEnds::start == last->unsectEnd || MatchEnds::start == last->coinEnd)
					   && (MatchEnds::start == s->unsectEnd || MatchEnds::start == s->coinEnd)) {
					size_t lastEndIndex = findEndIndex(last);
					size_t sEndIndex = findEndIndex(s);
					OP_ASSERT(sEndIndex < lastEndIndex 
							|| i[lastEndIndex]->ptT.t == i[sEndIndex]->ptT.t);
				}
				last = s;
			}
		}
#endif
		return;
	}
	unsorted = false;
	// moved sects may have put coin, unsect out of order -- reset coin, unsect: start and end
	// order first in t
	std::sort(i.begin(), i.end(), [](const OpIntersection* s1, const OpIntersection* s2) {
		return s1->ptT.t < s2->ptT.t;
	});
	// mark coin, unsect: start, end
	if (hasPairs) {
		orderPairs();
		// then put coincident start before unmarked, and finally end
		// start: put coincident, unmarked, unsectable (if t is equal)
		// end: put unsectable, unmarked, coincident (if t is equal)
		// sort unmarked by segment id
		std::sort(i.begin(), i.end(), [](const OpIntersection* s1, const OpIntersection* s2) {
			if (s1->ptT.t != s2->ptT.t)
				return s1->ptT.t < s2->ptT.t;
			if (MatchEnds::none != s1->unsectEnd || MatchEnds::none != s1->coinEnd
					|| MatchEnds::none != s2->unsectEnd || MatchEnds::none != s2->coinEnd) {
				bool s1start = MatchEnds::start == s1->unsectEnd || MatchEnds::start == s1->coinEnd;
				bool s2end = MatchEnds::end == s2->unsectEnd || MatchEnds::end == s2->coinEnd;
				if (s1start && s2end)
					return false;
				bool s1end = MatchEnds::end == s1->unsectEnd || MatchEnds::end == s1->coinEnd;
				bool s2start = MatchEnds::start == s2->unsectEnd || MatchEnds::start == s2->coinEnd;
				if (s1end && s2start)
					return true;
				if (s1start)
					return MatchEnds::start == s1->coinEnd 
							&& (!s2start || s2->coincidenceID < s1->coincidenceID);
				if (s2end)
					return MatchEnds::end == s2->coinEnd
							&& (!s1end || s1->coincidenceID < s2->coincidenceID);
				if (s1end)
					return MatchEnds::end == s1->unsectEnd
							&& (!s2end || s1->unsectID < s2->unsectID);
				if (s2start)
					return MatchEnds::start == s2->unsectEnd
							&& (!s1start || s2->unsectID < s1->unsectID);
			}
			return s1->opp->segment->id < s2->opp->segment->id;  // compare ptr if id is debug only?
		});
	}
	if (3 >= i.size())
		return;
	// Two or more coincident (or unsectable) pairs with the same t may require further
	// sorting. Order them so that they nest coincidences by finding the other end.
	size_t rangeStart = 0;
	auto processStart = [this, &rangeStart](size_t rangeEnd) {
		size_t toFind = rangeEnd - rangeStart;
		std::vector<OpIntersection*> sorted(toFind);  // reserve sorted pointers copy (zeroed)
		size_t endI = rangeEnd;
		size_t found = 0;
		while (found < toFind && endI < i.size()) {  // look for ends that match found starts
			OpIntersection* end = i[endI++];
			if (MatchEnds::end != end->unsectEnd && MatchEnds::end != end->coinEnd)
				continue;
			size_t startI = rangeStart;
			do {   // for found end, find matching start
				OpIntersection* start = i[startI++];
				OP_ASSERT(MatchEnds::start == start->unsectEnd
						|| MatchEnds::start == start->coinEnd);
				if ((start->unsectID != end->unsectID || !start->unsectID) 
						&& (start->coincidenceID != end->coincidenceID || !start->coincidenceID))
					continue;
				 if (sorted.end() != std::find(sorted.begin(), sorted.end(), start))
					continue;
				++found;
				OP_ASSERT(!sorted[toFind - found]);
				sorted[toFind - found] = start;  // reverse order so ranges nest
				break;
			} while (startI < rangeEnd);
		}
		OP_ASSERT(found == toFind);
		std::copy(sorted.begin(), sorted.end(), i.begin() + rangeStart);
	};
	float t = 0;
	size_t index = 0;
	do {  // iterate through all, gathering groups of equal t start values
		OpIntersection* sect = i[index];
		bool isStart = MatchEnds::start == sect->unsectEnd || MatchEnds::start == sect->coinEnd;
		float nextT = sect->ptT.t;
		if (isStart && t == nextT)
			continue;
		if (rangeStart + 2 <= index)
			processStart(index);
		rangeStart = index + !isStart;
		t = nextT;
	} while (++index < i.size());
	// first pass (above) resorts range of start with same t. second pass (below) resorts end 
	// !!! maybe two passes can share code? not sure
	auto processEnd = [this, &rangeStart](size_t rangeEnd) {  // look for starts that match ends
		size_t toFind = rangeEnd - rangeStart;
		std::vector<OpIntersection*> sorted(toFind);  // reserve sorted pointers copy (zeroed)
		size_t startI = 0;
		size_t found = 0;
		while (found < toFind && startI < rangeStart) {  // look for starts that match found ends
			OpIntersection* start = i[startI++];
			if (MatchEnds::start != start->unsectEnd && MatchEnds::start != start->coinEnd)
				continue;
			size_t endI = rangeStart;
			do {   // for found start, find matching end
				OpIntersection* end = i[endI++];
				OP_ASSERT(MatchEnds::end == end->unsectEnd
						|| MatchEnds::end == end->coinEnd);
				if ((end->unsectID != start->unsectID || !end->unsectID)
						&& (end->coincidenceID != start->coincidenceID || !end->coincidenceID))
					continue;
				if (sorted.end() != std::find(sorted.begin(), sorted.end(), end))
					continue;
				++found;
				OP_ASSERT(!sorted[toFind - found]);
				sorted[toFind - found] = end;  // reverse order so ranges nest
				break;
			} while (endI < rangeEnd);
		}
		OP_ASSERT(found == toFind);
		std::copy(sorted.begin(), sorted.end(), i.begin() + rangeStart);
	};
	t = 0;
	index = 0;
	do {  // iterate through all, gathering groups of equal t end values
		OpIntersection* sect = i[index];
		bool isEnd = MatchEnds::end == sect->unsectEnd || MatchEnds::end == sect->coinEnd;
		float nextT = sect->ptT.t;
		if (isEnd && t == nextT)
			continue;
		if (rangeStart + 2 <= index)
			processEnd(index);
		rangeStart = index + !isEnd;
		t = nextT;
	} while (++index < i.size());
	if (rangeStart + 2 <= index)
		processEnd(index);
}

#if 0
// look for cases where seg A intersects seg B at X, and seg A intersects seg C at X
//   but seg B does not intersect seg C at X
TripleSected OpIntersections::tripleSect() {
	OP_ASSERT(!unsorted);
	OpIntersection* last = nullptr;
	OpPoint lastPt;
	size_t index = 0;
	while (index < i.size() && !i[index]->ptT.t)
		++index;
	while (index < i.size()) {
		OpIntersection* test = i[index];
		if (1 == test->ptT.t)
			break;
		OpPoint testPt = test->ptT.pt;
		if (last && MatchEnds::none == test->coinEnd && MatchEnds::none == last->coinEnd
				&& testPt == lastPt) {
			// there are 3 or more intersections at the same point (3 or more different segments)
			OpSegment* oppSegment = test->opp->segment;
			OP_ASSERT(test->segment->id != oppSegment->id);
			OP_ASSERT(oppSegment->id != last->opp->segment->id);
			OP_ASSERT(oppSegment->id != test->segment->id);
			// there may be multiple intersections with correct segment pair
			// the matching one may be identical, may be close, or may be missing
			// use threshold to distinguish between close and missing
			OpVector threshold = test->segment->threshold();  // !!! may need a scaling factor
			threshold *= 16;  // !!! hardcode for now
			CloseBy close = oppSegment->sects.containsClose(testPt, threshold, last->opp->segment);
			if (NearBy::nearby == close.nearby) {
				OP_ASSERT(close.sect && close.sect->ptT.pt != testPt);
				OP_ASSERT(testPt.isNearly(close.sect->ptT.pt, threshold));
				close.sect->ptT = test->opp->ptT;
				close.sect->opp->ptT = last->opp->ptT;
//				if (oppSegment->sects.outOfOrder())
//					return TripleSected::tryAgain;
			} else if (NearBy::none == close.nearby) {
				OpIntersection* testSect = oppSegment->addSegBase(test->opp->ptT  
						OP_LINE_FILE_PARAMS(last->opp->segment));
				OpIntersection* lastSect = last->opp->segment->addSegBase(last->opp->ptT  
						OP_LINE_FILE_PARAMS(oppSegment));
				testSect->pair(lastSect);
				// return to caller, sort intersections, try again
				// !!! redesign if this has performance implications
				return TripleSected::tryAgain;
			} else {  // nothing to do if the pair's coincidence includes test pt
				// !!! assert on !close.sect means add code; intersection point is missing from B/C
				OP_ASSERT(close.sect);
				OP_ASSERT(NearBy::equal == close.nearby  
						|| (NearBy::far == close.nearby && close.sect->coincidenceID));
			}
		}
		last = test;
		lastPt = testPt;
		++index;
	}
	return TripleSected::done;
}
#endif

void OpIntersections::zeroPairs(OpIntersection* sect) {
	int coinID = sect->coincidenceID;
	int unsectID = sect->unsectID;
	if (!coinID && !unsectID)
		return;
	for (OpIntersection* test : i) {
		if (coinID && test->coincidenceID == coinID)
			test->zeroCoincidencePair();
		if (unsectID && test->unsectID == unsectID)
			test->zeroUnsectPair();
	}
}
