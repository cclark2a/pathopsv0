// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef BinaryWinding_DEFINED
#define BinaryWinding_DEFINED

#include "PathOps.h"
#include "DebugOps.h"

namespace PathOpsV0Lib {

enum class BinaryOperand : int {
	left,
	right
};

// !!! only used for debugging?
#if 0
enum class BinaryOperation : int {
    Difference,
    Intersect,
    Union,
    ExclusiveOr,
    ReverseDifference
};
#endif

enum class BinaryWindType : int {
    evenOdd,
    windLeft,
    windRight,
    windBoth
};

struct BinaryData {
    BinaryData() 
        : left(0)
        , right(0) {
    }

    BinaryData(int initialL, int initialR)
		: left(initialL)
		, right(initialR) {
	}

    BinaryData(BinaryOperand binaryOperand) {
        left = BinaryOperand::left == binaryOperand;
        right = BinaryOperand::right == binaryOperand;
    }

    BinaryData(Winding w) {
        OP_ASSERT(w.size == sizeof(BinaryData));
        std::memcpy(this, w.data, sizeof(BinaryData));
    }

	void copyTo(Winding& w) const {
		OP_ASSERT(w.size == sizeof(BinaryData));
		std::memcpy(w.data, this, sizeof(BinaryData));
	}

    int left;
    int right;
};

struct BinaryWinding {
    BinaryWinding(Context* context, BinaryOperand );

    Winding winding;
    BinaryData data;
};

#if 0
struct BinaryOpData {
    BinaryOperation operation;
    BinaryOperand operand;
};
#endif

inline void binaryEvenOddFunc(Winding winding, Winding toAdd) {
    BinaryData sum(winding);
    BinaryData addend(toAdd);
    sum.left ^= addend.left;
    sum.right ^= addend.right;
    sum.copyTo(winding);
}

inline void binaryAddFunc(Winding winding, Winding toAdd) {
    BinaryData sum(winding);
    BinaryData addend(toAdd);
    sum.left += addend.left;
    sum.right += addend.right;
    sum.copyTo(winding);
}

inline void binaryAddLeftFunc(Winding winding, Winding toAdd) {
    BinaryData sum(winding);
    BinaryData addend(toAdd);
    sum.left += addend.left;
    sum.right ^= addend.right;
    sum.copyTo(winding);
}

inline void binaryAddRightFunc(Winding winding, Winding toAdd) {
    BinaryData sum(winding);
    BinaryData addend(toAdd);
    sum.left ^= addend.left;
    sum.right += addend.right;
    sum.copyTo(winding);
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

    BinaryData bWind;
    BinaryData bSum;
    WindState left;
    WindState right;
    WindKeep keep;
    bool bothFlip;
};

inline WindKeep binaryDifferenceFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::Difference).keep;
}

inline WindKeep binaryExclusiveOrFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::ExclusiveOr).keep;
}

inline WindKeep binaryIntersectFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::Intersect).keep;
}

inline WindKeep binaryReverseDifferenceFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::ReverseDifference).keep;
}

inline WindKeep binaryUnionFunc(Winding winding, Winding sumWinding) {
    return KeepData(winding, sumWinding, &KeepData::Union).keep;
}

inline void binarySubtractFunc(Winding winding, Winding toSubtract) {
    BinaryData difference(winding);
    BinaryData subtrahend(toSubtract);
    difference.left -= subtrahend.left;
    difference.right -= subtrahend.right;
    difference.copyTo(winding);
}
    
inline void binarySubtractLeftFunc(Winding winding, Winding toSubtract) {
    BinaryData difference(winding);
    BinaryData subtrahend(toSubtract);
    difference.left -= subtrahend.left;
    difference.right ^= subtrahend.right;
    difference.copyTo(winding);
}
    
inline void binarySubtractRightFunc(Winding winding, Winding toSubtract) {
    BinaryData difference(winding);
    BinaryData subtrahend(toSubtract);
    difference.left ^= subtrahend.left;
    difference.right -= subtrahend.right;
    difference.copyTo(winding);
}
    
#if OP_DEBUG
inline bool binaryDebugIsFill(Winding winding) {
    return true;
}
#endif

#if OP_DEBUG_SERIALIZE
inline std::string binaryDumpOutFunc(Winding winding) {
    BinaryData binary(winding);
    std::string s = "{" + STR(binary.left) + ", " + STR(binary.right) + "}";
    return s;
}

#define BINARY_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(binaryEvenOddFunc), \
    OP_TAGGED_FUNCTION(binaryAddFunc), \
    OP_TAGGED_FUNCTION(binaryAddLeftFunc), \
    OP_TAGGED_FUNCTION(binaryAddRightFunc), \
    OP_TAGGED_FUNCTION(binaryDifferenceFunc), \
    OP_TAGGED_FUNCTION(binaryExclusiveOrFunc), \
    OP_TAGGED_FUNCTION(binaryIntersectFunc), \
    OP_TAGGED_FUNCTION(binaryReverseDifferenceFunc), \
    OP_TAGGED_FUNCTION(binaryUnionFunc), \
    OP_TAGGED_FUNCTION(binarySubtractFunc), \
    OP_TAGGED_FUNCTION(binarySubtractLeftFunc), \
    OP_TAGGED_FUNCTION(binarySubtractRightFunc), \
    OP_TAGGED_FUNCTION(binaryDebugIsFill), \
    OP_TAGGED_FUNCTION(binaryDumpOutFunc), \

#endif

#if OP_DEBUG_DUMP

inline void binaryDumpSetFunc(const char*& str, Winding& winding) {
    int left = OpDebugReadSizeT(str);
    int right = OpDebugReadSizeT(str);
    BinaryData binary(left, right);
    binary.copyTo(winding);
}

inline std::vector<std::string> binaryImageNamesFunc() {
    return { "left", "right" };
}

inline uint32_t binaryColorFunc(Winding winding, DebugEdgeType edgeType) {
    BinaryData data(winding);
	if (edgeType.disabled)
		return data.left ? red : darkRed;
	else if (edgeType.inOutput)
		return data.left ? orange : darkOrange;
	else if (edgeType.unsortable)
		return data.left ? purple : darkViolet;
	else if (edgeType.curveCurve) {
		if (edgeType.ccOverlaps)
			return data.left ? orange : darkGreen;
		else
			return data.left ? purple : darkViolet;
	}
    return data.left ? debugBlack : darkGreen;
}

inline WindKeep binaryVisibleFunc(Winding w, Winding s) {
    BinaryData winding(w);
    BinaryData sum(s);
    auto keeper = [](int wValue, int sValue) {  // stolen from unary keep
        if (!wValue || (sValue && sValue != wValue))
            return WindKeep::Discard;
        return sValue ? WindKeep::Start : WindKeep::End;
    };
    WindKeep keepLeft = keeper(winding.left, sum.left);
    WindKeep keepRight = keeper(winding.right, sum.right);
    if (WindKeep::Discard == keepLeft)
        return keepRight;
    if (WindKeep::Discard == keepRight)
        return keepLeft;
    OP_ASSERT(keepLeft == keepRight);  // !!! think about it if this returns start + end
    return keepLeft;
}

#define DUMP_BINARY_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(binaryDumpSetFunc), \
    OP_TAGGED_FUNCTION(binaryImageNamesFunc), \
    OP_TAGGED_FUNCTION(binaryColorFunc), \
    OP_TAGGED_FUNCTION(binaryVisibleFunc), \

#endif

#if OP_DEBUG_SERIALIZE
// !!! this will replace index version
inline std::string binaryImageOutFunc(Winding winding) {
    BinaryData binaryData(winding);
    return STR(binaryData.left) + " " + STR(binaryData.right);
}

#define BINARY_IMAGE_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(binaryImageOutFunc), \

#endif

inline Context* binaryContext(CurveOutput output = nullptr, EmptyCallerPath empty = nullptr) {
    Context* context = CreateContext();
    SetContextCallbacks(context, { output, empty });
//    binaryCallbacks(context);  // !!! still thinking about this
#if OP_DEBUG
	SetDebugContextCallbacks(context, {
        binaryDebugIsFill, nullptr, nullptr
        OP_DEBUG_DUMP_PARAMS(binaryDumpOutFunc, binaryDumpSetFunc, nullptr,
                binaryImageOutFunc, binaryImageNamesFunc, binaryColorFunc, binaryVisibleFunc)
    });
#endif
    return context;
}

inline BinaryWinding::BinaryWinding(Context* context, BinaryOperand binaryOperand) 
    : data(binaryOperand) {
    winding.contour = CreateContour(context, &data, sizeof(data));
}

}

#endif
