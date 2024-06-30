// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOpsTypes.h"
#include "OpTypes.h"

namespace PathOpsV0Lib {

inline Winding unaryWindingAddFunc(Winding winding, Winding toAdd) {
    *(int*) winding.data += *(int*) toAdd.data;
    return winding;
}

// normal (clockwise from vector direction) points to sum
// if winding is non-zero:
//   if sum equals winding, fill starts
//   if sum is zero, fill ends
inline WindKeep unaryWindingKeepFunc(Winding winding, Winding sumWinding) {
    int wind = *(int*) winding.data;
    int sum = *(int*) sumWinding.data;
    if (!wind || (sum && sum != wind))
         return WindKeep::Discard;
    return sum ? WindKeep::Start : WindKeep::End;
}

inline Winding unaryWindingSubtractFunc(Winding winding, Winding toSubtract) {
    *(int*) winding.data -= *(int*) toSubtract.data;
    return winding;
}
    
inline bool unaryWindingVisibleFunc(Winding winding) {
    return !!*(int*) winding.data;
}

inline void unaryWindingZeroFunc(Winding toZero) {
    *(int*) toZero.data = 0;
}

#if OP_DEBUG_IMAGE
inline std::string unaryWindingImageOutFunc(Winding winding, int index) {
    if (index > 0)
        return "-";
    std::string s = STR(((int*) winding.data)[0]);
    return s;
}
#endif

#if OP_DEBUG_DUMP
inline void unaryWindingDumpInFunc(const char*& str, Winding winding) {
    OpDebugRequired(str, "{");
    int value = OpDebugReadSizeT(str);
    OpDebugRequired(str, "}");
    *(int*) winding.data = value;
}

inline std::string unaryWindingDumpOutFunc(Winding winding) {
    std::string s = "{" + STR(*(int*) winding.data) + "}";
    return s;
}
#endif

}
