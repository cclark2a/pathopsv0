// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpOperators_DEFINED
#define OpOperators_DEFINED

enum class OpOperator {
    Subtract,         //!< subtract the op path from the first path
    Intersect,        //!< intersect the two paths
    Union,            //!< union (inclusive-or) the two paths
    ExclusiveOr,      //!< exclusive-or the two paths
    ReverseSubtract,  //!< subtract the first path from the op path
};

constexpr inline int operator+(OpOperator a) {
    return static_cast<int>(a);
}

#endif
