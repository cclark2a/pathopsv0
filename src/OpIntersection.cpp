// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpIntersection.h"
#include "OpContour.h"
#include "OpSegment.h"

void OpIntersection::setCoin(int cid, MatchEnds end) {
	coincidenceID = cid;
	coinEnd = end;
	segment->hasCoin = true;
	segment->sects.unsorted = true;
}

void OpIntersection::setUnsect(int uid, MatchEnds end) {
	unsectID = uid;
	unsectEnd = end;
	segment->hasUnsectable = true;
	segment->sects.unsorted = true;
}

OpIntersection* OpIntersection::coinOtherEnd() {
	auto endSect = std::find_if(segment->sects.i.begin(), segment->sects.i.end(), 
			[this](const OpIntersection* test) { 
			return test->coincidenceID == coincidenceID && test->coinEnd == !coinEnd; });
	OP_ASSERT(segment->sects.i.end() != endSect);
	return *endSect;
};

OpIntersections::OpIntersections()
	: unsorted(false) {
}

OpIntersection* OpIntersections::add(OpIntersection* sect) {
	i.push_back(sect);
	unsorted = true;
	return sect;
}

// this matches opp with nearby ptT
OpIntersection* OpIntersections::contains(const OpPtT& ptT, const OpSegment* opp) {
	for (unsigned index = 0; index < i.size(); ++index) {
		OpIntersection* sect = i[index];
		if (!sect->opp || sect->opp->segment != opp)
			continue;
		if (ptT.isNearly(sect->ptT, opp->threshold()))
			return sect;
	}
	return nullptr;
}

OpIntersection* OpIntersections::coinContains(OpPoint pt, const OpSegment* opp, OpPtT* nearby) {
	OpIntersection* match = nullptr;
	OpVector thresh = opp->threshold();
	for (unsigned index = 0; index < i.size(); ++index) {
		OpIntersection* sect = i[index];
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
	auto stackCoins = [&coincidences](OpIntersection* sect) {
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
	};
	OpIntersection* first = i.front();
	for (OpIntersection* sectPtr : i) {
		if (first->ptT.t != sectPtr->ptT.t) {
			segment->edges.emplace_back(first, sectPtr  OP_LINE_FILE_PARAMS());
			first = sectPtr;
			OpEdge& newEdge = segment->edges.back();
			if (unsectables.size())
				newEdge.unSects = unsectables;
			if (coincidences.size()) {
				newEdge.coinPals = coincidences;
			}
		}
		stackUnsects(sectPtr);
		stackCoins(sectPtr);
	}
}

const OpIntersection* OpIntersections::nearly(const OpPtT& ptT, OpSegment* oSeg) const {
	for (unsigned index = 0; index < i.size(); ++index) {
		OpIntersection* sect = i[index];
		if (oSeg && (!sect->opp || sect->opp->segment != oSeg))
			continue;
		if (ptT.pt.isNearly(sect->ptT.pt, oSeg->threshold()) 
				|| (ptT.t - OpEpsilon <= sect->ptT.t && sect->ptT.t <= ptT.t + OpEpsilon))
			return sect;
	}
	return nullptr;
}

void OpIntersections::range(const OpSegment* opp, std::vector<OpIntersection*>& result) {
	if (unsorted)
		sort();
	OP_DEBUG_CODE(float last = -1);
	for (OpIntersection* sect : i) {
		if (sect->opp && sect->opp->segment == opp) {
			OP_ASSERT(last < sect->ptT.t);
			OP_DEBUG_CODE(last = sect->ptT.t);
			result.push_back(sect);
		}
	}
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

struct SectPreferred {
	SectPreferred(OpIntersection* sect)
		: best(sect)
		, bestOnEnd(sect->ptT.onEnd())
		, collapsed(false) {
		OP_DEBUG_CODE(int safetyValue = 10);  // !!! no idea what this should be
		do {
			if (find())  // repeat once if better point was found
				return;
			visited.clear();
			OP_ASSERT(--safetyValue);
		} while (true);
	}

	bool find();  // make consecutive nearly equal sects the same, and follow each opposite

	std::vector<OpSegment*> visited; // visit each segment with matching sects once
	OpIntersection* best;  // an end point, if one exists; otherwise, an arbitrary sect
	bool bestOnEnd;  // never cleared, only set (once first sect on end is found)
	bool collapsed;
};

bool SectPreferred::find() {
	OpSegment* seg = best->segment;
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
				if (0 == best->ptT.t) {
					seg->setDisabled(OP_LINE_FILE_NPARAMS());
					return false;
				}
				OP_ASSERT(1 == sects.i.back()->ptT.t);
				best->ptT.t = 1;
			}
		} else if (test->ptT != best->ptT) {
			if (test->ptT.onEnd()) {
				if (bestOnEnd) {
					seg->movePt(test->ptT, best->ptT.pt);  // !!! trace through -- may need rewrite
				} else {
					best = test;
					bestOnEnd = true;
				}
				return false; // best changed; start over
			}
			if (test->ptT.pt != best->ptT.pt)
				sects.moveSects(test->ptT, best->ptT.pt);
		}
		if (visited.end() == std::find(visited.begin(), visited.end(), test->opp->segment)) {
			OpIntersection* save = best;
			best = test->opp;
			if (!find())
				return false;
			best = save;
		}
	}
	return true;
}

bool OpIntersections::checkCollapse(OpIntersection* sect) {
	bool result = false;
	int cID = sect->coincidenceID;
	int uID = sect->unsectID;
	OP_ASSERT(cID || uID);  // could have both
	for (size_t index = 0; index < i.size(); ++index) {
		OpIntersection* test = i[index];
		if ((!cID || cID != test->coincidenceID) && (!uID || uID != test->unsectID))
			continue;
		if (test == sect)
			continue;
		if (test->ptT == sect->ptT) {
			if (cID)
				sect->zeroCoincidenceID();
			if (uID)
				sect->zeroUnsectID();
			result = test->collapsed = test->opp->collapsed = true;
		} else {
			OP_ASSERT(test->ptT.pt != sect->ptT.pt);
			OP_ASSERT(test->ptT.t != sect->ptT.t);
		}
	}
	return result;
}

void OpIntersections::eraseCollapsed() {
	// delete collapsed coin or unsect after all sects are checked
	for (size_t index = i.size(); index-- != 0; ) {
		OpIntersection* iSect = i[index];
		if (!iSect->collapsed) 
			continue;
		OpIntersections& oSects = iSect->opp->segment->sects;
		for (size_t oIndex = oSects.i.size(); oIndex-- != 0; ) {
			OpIntersection* oSect = oSects.i[oIndex];
			if (oSect == iSect->opp) {
				oSects.i.erase(oSects.i.begin() + oIndex);
				break;
			}
		}
		i.erase(i.begin() + index);
	}
}

	// intersections may have multiple different t values with the same pt value
	// should be rare; do an exhaustive search for duplicates
void OpIntersections::mergeNear(OpPtAliases& aliases) {
	for (unsigned outer = 1; outer < i.size(); ++outer) {
		OpIntersection* oSect = i[outer - 1];
		if (oSect->mergeProcessed)
			continue;
		unsigned limit = outer;
		bool nearEqual = false;
		do {
			OpIntersection* iSect = i[limit];
			if (!iSect->ptT.isNearly(oSect->ptT, iSect->segment->threshold()))
				break;
			if (iSect->ptT == oSect->ptT)
				continue;
			nearEqual = true;
			if (aliases.contains(iSect->ptT.pt))
				oSect = iSect;
		} while (++limit < i.size());
		if (outer == limit)
			continue;
		if (!nearEqual) {
			outer = limit;
			continue;
		}
		SectPreferred preferred(oSect);
	}
}

void OpIntersections::moveSects(OpPtT match, OpPoint destination) {
	OP_ASSERT(match.pt != destination);
	std::vector<OpIntersection*> moved;
	std::vector<OpIntersection*> collaspes;
	float destT = match.t;
	if (0 < destT && destT < 1) {
		for (OpIntersection* sect : i) {
			if (sect->ptT.pt == destination) {
				destT = sect->ptT.t;
				break;
			}
		}
	}
	for (OpIntersection* sect : i) {
		OpPtT& sPtT = sect->ptT;
		if (sPtT.pt == destination ? sPtT.t == destT : 
				sPtT.pt != match.pt && sPtT.t != match.t && sPtT.t != destT)
			continue;
		if (sPtT.pt != destination) {
			OpIntersection* opp = sect->opp;
			bool oppMatches = sPtT.pt == opp->ptT.pt;
			sPtT.pt = destination;
			if (oppMatches && !opp->moved)
				moved.push_back(sect);
			sect->moved = true;
		}
		if (sPtT.t != destT) {
			sPtT.t = destT;
			unsorted = true;
		}
	}
	for (OpIntersection* sect : moved) {
		OpIntersection* opp = sect->opp;
		if (opp->moved)
			continue;
		if (opp->ptT.onEnd())
			opp->segment->movePt(opp->ptT, destination);
		else
			opp->segment->sects.moveSects(opp->ptT, destination);
	}
	bool collapsed = false;
	for (OpIntersection* sect : moved) {
		if (sect->coincidenceID || sect->unsectID)
			collapsed |= checkCollapse(sect);
	}
	if (collapsed)
		eraseCollapsed();
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
#if OP_DEBUG
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
					OP_ASSERT(sEndIndex < lastEndIndex);
				}
				last = s;
			}
		}
#endif
		return;
	}
	unsorted = false;
	// order first in t, then put coincident start before unmarked, and finally end
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
				return MatchEnds::start == s1->coinEnd;
			if (s2end)
				return MatchEnds::end == s2->coinEnd;
			if (s1end)
				return MatchEnds::end == s1->unsectEnd;
			if (s2start)
				return MatchEnds::start == s2->unsectEnd;
		}
		return s1->opp->segment->id < s2->opp->segment->id;  // compare seg ptr if id is debug only?
	});
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
