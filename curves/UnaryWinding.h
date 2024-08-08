// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

struct UnaryWinding {
    UnaryWinding()
        : left(0) {
    }

    UnaryWinding(Winding w) {
        OP_ASSERT(w.size == sizeof(UnaryWinding));
        std::memcpy(this, w.data, sizeof(UnaryWinding));
    }

    int left;
};

inline void copyToWinding(Winding& w, UnaryWinding u) {
    OP_ASSERT(w.size == sizeof(UnaryWinding));
    std::memcpy(w.data, &u, sizeof(UnaryWinding));
}

inline Winding unaryEvenOddFunc(Winding winding, Winding toAdd) {
    UnaryWinding sum(winding);
    UnaryWinding addend(toAdd);
    sum.left ^= addend.left;
    copyToWinding(winding, sum);
    return winding;
}

inline Winding unaryWindingAddFunc(Winding winding, Winding toAdd) {
    UnaryWinding sum(winding);
    UnaryWinding addend(toAdd);
    sum.left += addend.left;
    copyToWinding(winding, sum);
    return winding;
}

// normal (clockwise from vector direction) points to sum
// if winding is non-zero:
//   if sum equals winding, fill starts
//   if sum is zero, fill ends
inline WindKeep unaryWindingKeepFunc(Winding winding, Winding sumWinding) {
    UnaryWinding wind(winding);
    UnaryWinding sum(sumWinding);
    if (!wind.left || (sum.left && sum.left != wind.left))
         return WindKeep::Discard;
    return sum.left ? WindKeep::Start : WindKeep::End;
}

inline Winding unaryWindingSubtractFunc(Winding winding, Winding toSubtract) {
    UnaryWinding difference(winding);
    UnaryWinding subtrahend(toSubtract);
    difference.left -= subtrahend.left;
    copyToWinding(winding, difference);
    return winding;
}
    
inline bool unaryWindingVisibleFunc(Winding winding) {
    UnaryWinding test(winding);
    return !!test.left;
}

inline void unaryWindingZeroFunc(Winding toZero) {
    UnaryWinding zero;
    copyToWinding(toZero, zero);
}

#if OP_DEBUG_DUMP
inline void unaryWindingDumpInFunc(const char*& str, Winding winding) {
    UnaryWinding unaryWinding;
    OpDebugRequired(str, "{");
    unaryWinding.left = OpDebugReadSizeT(str);
    OpDebugRequired(str, "}");
    copyToWinding(winding, unaryWinding);
}

inline std::string unaryWindingDumpOutFunc(Winding winding) {
    UnaryWinding unary(winding);
    std::string s = "{" + STR(unary.left) + "}";
    return s;
}
#endif

#if OP_DEBUG_IMAGE
inline std::string unaryWindingImageOutFunc(Winding winding, int index) {
    if (index > 0)
        return "-";
    UnaryWinding unaryWinding(winding);
    std::string s = STR(unaryWinding.left);
    return s;
}
#endif

}
