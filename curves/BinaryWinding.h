// (c) 2023, Cary Clark cclark2@gmail.com

// new interface idea

#include "PathOps.h"

namespace PathOpsV0Lib {

enum class BinaryOperand : int {
	left,
	right
};

// !!! only used for debugging?
enum class BinaryOperation : int {
    Difference,
    Intersect,
    Union,
    ExclusiveOr,
    ReverseDifference
};

enum class BinaryWindType : int {
    evenOdd,
    windLeft,
    windRight,
    windBoth
};

struct BinaryWinding {
    BinaryWinding() 
        : left(0)
        , right(0) {
    }

    BinaryWinding(int initialL, int initialR)
		: left(initialL)
		, right(initialR) {
	}

    BinaryWinding(Winding w) {
        OP_ASSERT(w.size == sizeof(BinaryWinding));
        std::memcpy(this, w.data, sizeof(BinaryWinding));
    }

	void copyTo(Winding& w) const {
		OP_ASSERT(w.size == sizeof(BinaryWinding));
		std::memcpy(w.data, this, sizeof(BinaryWinding));
	}

    int left;
    int right;
};

struct BinaryOpData {
    BinaryOperation operation;
    BinaryOperand operand;
};

inline Winding binaryEvenOddFunc(Winding winding, Winding toAdd) {
    BinaryWinding sum(winding);
    BinaryWinding addend(toAdd);
    sum.left ^= addend.left;
    sum.right ^= addend.right;
    sum.copyTo(winding);
    return winding;
}

inline Winding binaryWindingAddFunc(Winding winding, Winding toAdd) {
    BinaryWinding sum(winding);
    BinaryWinding addend(toAdd);
    sum.left += addend.left;
    sum.right += addend.right;
    sum.copyTo(winding);
    return winding;
}

inline Winding binaryWindingAddLeftFunc(Winding winding, Winding toAdd) {
    BinaryWinding sum(winding);
    BinaryWinding addend(toAdd);
    sum.left += addend.left;
    sum.right ^= addend.right;
    sum.copyTo(winding);
    return winding;
}

inline Winding binaryWindingAddRightFunc(Winding winding, Winding toAdd) {
    BinaryWinding sum(winding);
    BinaryWinding addend(toAdd);
    sum.left ^= addend.left;
    sum.right += addend.right;
    sum.copyTo(winding);
    return winding;
}

// normal (clockwise from vector direction) points to sum
// if winding is non-zero:
//   if sum equals winding, fill starts
//   if sum is zero, fill ends
struct KeepData {
    KeepData(Winding winding, Winding sumWinding, void (KeepData::*FuncPtr)())
        : bWind(winding)
        , bSum(sumWinding)
        , keep(WindKeep::Discard)
    {
        auto windState = [](int wind, int sum) {
            if (!wind)
                return sum ? WindState::one : WindState::zero;
            if (sum)
                return wind == sum ? WindState::flipOff : WindState::one;
            return WindState::flipOn;
        };
        left = windState(bWind.left, bSum.left);
        right = windState(bWind.right, bSum.right);
	    bool leftFlips = left == WindState::flipOff || left == WindState::flipOn;
		bool rightFlips = right == WindState::flipOff || right == WindState::flipOn;
        if (!leftFlips && !rightFlips)
            return;
        bothFlip = leftFlips && rightFlips;
        (this->*(FuncPtr))();
    }

    // !!! while true, it may be too confusing to associate this table with implementation
    /* table of winding states that the op types use to keep an edge
	left op (first path)	right op (second path)		keep if:
			0					0					---
			0					flipOff				union, rdiff, xor
			0					flipOn				union, rdiff, xor
			0					1					---
		    flipOff				0					union, diff, xor
		    flipOff				flipOff				intersect, union
		    flipOff				flipOn				diff, rdiff
		    flipOff				1					intersect, rdiff, xor
		    flipOn				0					union, diff, xor
		    flipOn				flipOff				diff, rdiff
		    flipOn				flipOn				intersect, union
		    flipOn				1					intersect, rdiff, xor
			1					0					---
			1					flipOff				intersect, diff, xor
			1					flipOn				intersect, diff, xor
			1					1					---
    */

    void Difference() {
		if (bothFlip ? left != right : WindState::one == left || WindState::zero == right)
			keep = bSum.right || !bSum.left ? WindKeep::End : WindKeep::Start;
    }

    void ExclusiveOr() {
		if (!bothFlip)
			keep = !(bool)bSum.left == !(bool)bSum.right ? WindKeep::End : WindKeep::Start;
    }

    void Intersect() {
        if (bothFlip ? left == right : WindState::zero != left && WindState::zero != right)
		   keep = !bSum.left || !bSum.right ? WindKeep::End : WindKeep::Start;
    }

    void ReverseDifference() {
		if (bothFlip ? left != right : WindState::zero == left || WindState::one == right)
			keep = bSum.left || !bSum.right ? WindKeep::End : WindKeep::Start;
    }

    void Union() {
        if (bothFlip ? left == right : WindState::one != left && WindState::one != right)
		    keep = !bSum.left && !bSum.right ? WindKeep::End : WindKeep::Start;
    }

    BinaryWinding bWind;
    BinaryWinding bSum;
    WindState left;
    WindState right;
    WindKeep keep;
    bool bothFlip;
};

inline WindKeep binaryWindingDifferenceFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::Difference).keep;
}

inline WindKeep binaryWindingExclusiveOrFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::ExclusiveOr).keep;
}

inline WindKeep binaryWindingIntersectFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::Intersect).keep;
}

inline WindKeep binaryWindingReverseDifferenceFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::ReverseDifference).keep;
}

inline WindKeep binaryWindingUnionFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::Union).keep;
}

inline Winding binaryWindingSubtractFunc(Winding winding, Winding toSubtract) {
    BinaryWinding difference(winding);
    BinaryWinding subtrahend(toSubtract);
    difference.left -= subtrahend.left;
    difference.right -= subtrahend.right;
    difference.copyTo(winding);
    return winding;
}
    
inline Winding binaryWindingSubtractLeftFunc(Winding winding, Winding toSubtract) {
    BinaryWinding difference(winding);
    BinaryWinding subtrahend(toSubtract);
    difference.left -= subtrahend.left;
    difference.right ^= subtrahend.right;
    difference.copyTo(winding);
    return winding;
}
    
inline Winding binaryWindingSubtractRightFunc(Winding winding, Winding toSubtract) {
    BinaryWinding difference(winding);
    BinaryWinding subtrahend(toSubtract);
    difference.left ^= subtrahend.left;
    difference.right -= subtrahend.right;
    difference.copyTo(winding);
    return winding;
}
    
inline bool binaryWindingVisibleFunc(Winding winding) {
    BinaryWinding test(winding);
    return test.left || test.right;
}

inline void binaryWindingZeroFunc(Winding toZero) {
    BinaryWinding zero;
    zero.copyTo(toZero);
}

#if OP_DEBUG_DUMP
inline void binaryWindingDumpInFunc(const char*& str, Winding winding) {
    BinaryWinding binaryWinding;
    OpDebugRequired(str, "{");
    binaryWinding.left = OpDebugReadSizeT(str);
    binaryWinding.right = OpDebugReadSizeT(str);
    OpDebugRequired(str, "}");
    binaryWinding.copyTo(winding);
}

inline std::string binaryWindingDumpOutFunc(Winding winding) {
    BinaryWinding binary(winding);
    std::string s = "{" + STR(binary.left) + ", " + STR(binary.right) + "}";
    return s;
}

#endif

#if OP_DEBUG_IMAGE
inline std::string binaryWindingImageOutFunc(Winding winding, int index) {
    if (index > 1)
        return "-";
    BinaryWinding binaryWinding(winding);
    std::string s = STR(index ? binaryWinding.right : binaryWinding.left);
    return s;
}

#endif

}
