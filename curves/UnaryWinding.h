// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef UnaryWinding_DEFINED
#define UnaryWinding_DEFINED

#include "PathOps.h"

namespace PathOpsV0Lib {

struct UnaryWinding {
	
    UnaryWinding()
        : value(0) {
	}

    UnaryWinding(int initial)
		: value(initial) {
	}

    UnaryWinding(Winding w) {
        OP_ASSERT(w.size == sizeof(UnaryWinding));
        std::memcpy(this, w.data, sizeof(UnaryWinding));
    }

	void copyTo(Winding& w) const {
		OP_ASSERT(w.size == sizeof(UnaryWinding));
		std::memcpy(w.data, this, sizeof(UnaryWinding));
	}

    int value;
};

inline void unaryEvenOddFunc(Winding winding, Winding toAdd) {
    UnaryWinding sum(winding);
    UnaryWinding addend(toAdd);
    sum.value ^= addend.value;
    sum.copyTo(winding);
}

inline void unaryWindingAddFunc(Winding winding, Winding toAdd) {
    UnaryWinding sum(winding);
    UnaryWinding addend(toAdd);
    sum.value += addend.value;
    sum.copyTo(winding);
}

// normal (clockwise from vector direction) points to sum
// if winding is non-zero:
//   if sum equals winding, fill starts
//   if sum is zero, fill ends
inline WindKeep unaryWindingKeepFunc(Winding winding, Winding sumWinding) {
    UnaryWinding wind(winding);
    UnaryWinding sum(sumWinding);
    if (!wind.value || (sum.value && sum.value != wind.value))
         return WindKeep::Discard;
    return sum.value ? WindKeep::Start : WindKeep::End;
}

inline void unaryWindingSubtractFunc(Winding winding, Winding toSubtract) {
    UnaryWinding difference(winding);
    UnaryWinding subtrahend(toSubtract);
    difference.value -= subtrahend.value;
    difference.copyTo(winding);
}
    
inline bool unaryWindingVisibleFunc(Winding winding) {
    UnaryWinding test(winding);
    return !!test.value;
}

inline void unaryWindingZeroFunc(Winding toZero) {
    UnaryWinding zero;
    zero.copyTo(toZero);
}

inline void unaryCallbacks(Context* context) {
    SetWindingCallbacks(context, { unaryWindingAddFunc, unaryWindingKeepFunc, 
            unaryWindingVisibleFunc, unaryWindingZeroFunc, unaryWindingSubtractFunc } );
}

#if OP_DEBUG_DUMP
inline std::string unaryWindingDumpOutFunc(Winding winding) {
    UnaryWinding unary(winding);
    std::string s = "{" + STR(unary.value) + "}";
    return s;
}
#endif

#if OP_DEBUG_IMAGE
inline std::string unaryWindingImageOutFunc(Winding winding, int index) {
    if (index > 0)
        return "-";
    UnaryWinding unaryWinding(winding);
    std::string s = STR(unaryWinding.value);
    return s;
}
#endif

}

#endif
