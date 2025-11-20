// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef UnaryWinding_DEFINED
#define UnaryWinding_DEFINED

#include "PathOps.h"
#include "DebugOps.h"

namespace PathOpsV0Lib {

struct UnaryData {
    UnaryData()
        : value(0) {
	}

    UnaryData(int initial)
		: value(initial) {
	}

    UnaryData(Winding w) {
        OP_ASSERT(w.size == sizeof(UnaryData));
        std::memcpy(this, w.data, sizeof(UnaryData));
    }

	void copyTo(Winding& w) const {
		OP_ASSERT(w.size == sizeof(UnaryData));
		std::memcpy(w.data, this, sizeof(UnaryData));
	}

    int value;
};

struct UnaryWinding {
    UnaryWinding(Context* context);

    Winding winding;
    UnaryData data;
};

inline void unaryEvenOddFunc(Winding winding, Winding toAdd) {
    UnaryData sum(winding);
    UnaryData addend(toAdd);
    sum.value ^= addend.value;
    sum.copyTo(winding);
}

inline void unaryAddFunc(Winding winding, Winding toAdd) {
    UnaryData sum(winding);
    UnaryData addend(toAdd);
    sum.value += addend.value;
    sum.copyTo(winding);
}

// normal (clockwise from vector direction) points to sum
// if winding is non-zero:
//   if sum equals winding, fill starts
//   if sum is zero, fill ends
inline WindKeep unaryKeepFunc(Winding winding, Winding sumWinding) {
    UnaryData wind(winding);
    UnaryData sum(sumWinding);
    if (!wind.value || (sum.value && sum.value != wind.value))
         return WindKeep::Discard;
    return sum.value ? WindKeep::Start : WindKeep::End;
}

inline void unarySubtractFunc(Winding winding, Winding toSubtract) {
    UnaryData difference(winding);
    UnaryData subtrahend(toSubtract);
    difference.value -= subtrahend.value;
    difference.copyTo(winding);
}
    
inline void unaryCallbacks(Context* context) {
    SetWindingCallbacks(context, { unaryAddFunc, unaryKeepFunc, unarySubtractFunc } );
}

#if OP_DEBUG
inline bool unaryDebugIsFill(Winding winding) {
    return true;
}
#endif

#if OP_DEBUG_DUMP
inline std::string unaryDumpOutFunc(Winding winding) {
    UnaryData unary(winding);
    std::string s = "{" + STR(unary.value) + "}";
    return s;
}

inline void unaryDumpSetFunc(const char*& str, Winding& winding) {
    int left = OpDebugReadSizeT(str);
    UnaryData unaryData(left);
    unaryData.copyTo(winding);
}

#define UNARY_WINDING_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(unaryEvenOddFunc), \
    OP_TAGGED_FUNCTION(unaryAddFunc), \
    OP_TAGGED_FUNCTION(unaryKeepFunc), \
    OP_TAGGED_FUNCTION(unarySubtractFunc), \
    OP_TAGGED_FUNCTION(unaryDebugIsFill), \
    OP_TAGGED_FUNCTION(unaryDumpOutFunc), \
    OP_TAGGED_FUNCTION(unaryDumpSetFunc), \

#endif

#if OP_DEBUG_IMAGE
inline std::string unaryImageOutXFunc(Winding winding) {
    UnaryData unaryData(winding);
    return STR(unaryData.value);
}

// !!! deprecated
inline std::string unaryImageOutFunc(Winding winding, int index) {  // deprecated
    if (index > 0)
        return "-";
    UnaryData unaryData(winding);
    std::string s = STR(unaryData.value);
    return s;
}

inline uint32_t unaryColorFunc(Winding winding, DebugEdgeType edgeType) {
UnaryData unaryData(winding);
	if (edgeType.disabled || !unaryData.value)
		return red;
	if (edgeType.inOutput)
		return orange;
	if (edgeType.unsortable)
		return purple;
	if (edgeType.curveCurve) {
		if (edgeType.ccOverlaps)
			return orange;
		else
			return purple;
	}
    return debugBlack;
}

#define UNARY_IMAGE_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(unaryImageOutXFunc), \
    OP_TAGGED_FUNCTION(unaryImageOutFunc), \
    OP_TAGGED_FUNCTION(unaryColorFunc), \

#endif

inline Context* unaryContext(CurveOutput output = nullptr, EmptyCallerPath empty = nullptr) {
    Context* context = CreateContext();
    SetContextCallbacks(context, { output, empty });
    unaryCallbacks(context);
#if OP_DEBUG
    OpDebugData debugData(false);
    SetDebugData(context, debugData);
	SetDebugContextCallbacks(context, { 
        unaryDebugIsFill
        OP_DEBUG_DUMP_PARAMS(unaryDumpOutFunc, unaryDumpSetFunc)
        OP_DEBUG_IMAGE_PARAMS(unaryImageOutXFunc, unaryImageOutFunc, unaryColorFunc)
    });
#endif
    return context;
}

inline UnaryWinding::UnaryWinding(Context* context) {
    winding.contour = CreateContour(context, &data, sizeof(data));
}

}

#endif
