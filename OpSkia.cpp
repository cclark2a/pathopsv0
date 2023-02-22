#include "PathOps.h"

#if PATH_OPS_V0_TARGET == PATH_OPS_V0_FOR_SKIA
#include "include/core/SkPath.h"

bool OpInPath::isInverted() const {
	return skPath->isInverseFillType();
}

bool OpOutPath::setEmpty() {
	skPath->reset();
	return true;
}

bool OpOutPath::setInverted(bool wasInverted) {
    SkPathFillType fillType = wasInverted ? SkPathFillType::kInverseEvenOdd : SkPathFillType::kEvenOdd;
    skPath->setFillType(fillType);
	return true;
}

bool PathSimplify(OpInPath path, OpOutPath result) {
    SkPath empty;
	OpInPath emptyPath(&empty);
    return PathOps(path, emptyPath, OpOperator::Union, result);
}

#endif
