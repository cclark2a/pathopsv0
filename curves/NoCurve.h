// (c) 2023, Cary Clark cclark2@gmail.com

#include "PathOps.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

namespace PathOpsV0Lib {

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

inline int maxLimbs(Context* ) {
	return 500;
}

#if OP_DEBUG
inline uint8_t noDebugBitOper(DebugCallerData , uint8_t , uint8_t ) {
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
