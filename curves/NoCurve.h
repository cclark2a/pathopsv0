// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOps.h"

namespace PathOpsV0Lib {

inline void noEmptyPath(PathOutput ) {
}

// !!! put these in a different header?
inline int maxDepth(Curve , Curve ) {
	return 64;
}

inline int maxSplits(Curve , Curve ) {
	return 8;
}

inline float maxSignSwap(Curve , Curve ) {
	return 131072.f;
}

inline int maxLimbs() {
	return 1100;
}

inline WindKeep noWindKeepFunc(Winding , Winding ) {
    return WindKeep::Discard;
}

inline Winding noWindingOpFunc(Winding , Winding ) {
	Winding dummyWinding { nullptr, 0 };
	return dummyWinding;
}

#if OP_DEBUG
inline uint8_t noDebugBitOper(CallerData , uint8_t , uint8_t ) {
	return 0;
}

inline void noDebugScale(Curve , double , double , double ) {
}
#endif


#if OP_DEBUG_DUMP
inline std::string noDumpName() { 
    return ""; 
}

inline std::string noDumpCurveExtra(Curve , DebugLevel , DebugBase ) {
    return "";
}

inline void noDumpCurveSet(Curve , const char*& ) {
}

inline void noDumpCurveSetExtra(Curve , const char*& str) {
}
#endif

}
