// (c) 2023, Cary Clark cclark2@gmail.com

#include "PathOps.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

namespace PathOpsV0Lib {

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
