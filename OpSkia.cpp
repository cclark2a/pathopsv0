// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpEdge.h"
#include "PathOps.h"

#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"
#endif

#if !OP_TEST_NEW_INTERFACE
static SkPath* skPath(const void* extRef) {
    return (SkPath*) extRef;
}

bool PathSimplify(OpInPath& path, OpOutPath& result  OP_DEBUG_PARAMS(OpDebugData& debugData)) {
    SkPath empty;
	OpInPath emptyPath(&empty);
    return PathOps(path, emptyPath, OpOperator::Union, result  OP_DEBUG_PARAMS(debugData));
}

bool OpInPath::isInverted() const {
	return skPath(externalReference)->isInverseFillType();
}

void OpOutPath::setEmpty() {
	skPath(externalReference)->reset();
}

void OpOutPath::setInverted(bool wasInverted) {
    SkPathFillType fillType = wasInverted ? SkPathFillType::kInverseEvenOdd : SkPathFillType::kEvenOdd;
    skPath(externalReference)->setFillType(fillType);
}

#if OP_DEBUG_DUMP
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include "include/core/SkStream.h"

std::string OpInPath::debugDump(DebugLevel , DebugBase b) const {
    SkPath* skPath = (SkPath*) externalReference;
    SkDynamicMemoryWStream memoryStream;
    skPath->dump(&memoryStream, DebugBase::hex == b);
    std::string str;
    str.resize(memoryStream.bytesWritten());
    memoryStream.copyTo(str.data());
    str.pop_back();
    return str;
}

void dumpSet(const char*& str, SkPath* path) {
    path->reset();
    OpDebugRequired(str, "path.setFillType(SkPathFillType::k");
    if (OpDebugOptional(str, "Winding"))
        path->setFillType(SkPathFillType::kWinding);
    else if (OpDebugOptional(str, "EvenOdd"))
        path->setFillType(SkPathFillType::kEvenOdd);
    else if (OpDebugOptional(str, "InverseWinding"))
        path->setFillType(SkPathFillType::kInverseWinding);
    else if (OpDebugOptional(str, "InverseEvenOdd"))
        path->setFillType(SkPathFillType::kInverseEvenOdd);
    OpDebugRequired(str, ");");
    while (OpDebugOptional(str, "path.")) {
        size_t xyCount = 0;
        SkPathVerb verb = (SkPathVerb) -1;
        if (OpDebugOptional(str, "moveTo(")) {
            verb = SkPathVerb::kMove;
            xyCount = 2;
        } else if (OpDebugOptional(str, "lineTo(")) {
            verb = SkPathVerb::kLine;
            xyCount = 2;
        } else if (OpDebugOptional(str, "quadTo(")) {
            verb = SkPathVerb::kQuad;
            xyCount = 4;
        } else if (OpDebugOptional(str, "conicTo(")) {
            verb = SkPathVerb::kConic;
            xyCount = 5;
        } else if (OpDebugOptional(str, "cubicTo(")) {
            verb = SkPathVerb::kCubic;
            xyCount = 6;
        } else {
            OpDebugRequired(str, "close();");
            verb = SkPathVerb::kClose;
        }
        std::vector<float> data;
        data.resize(xyCount);
        for (size_t index = 0; index < xyCount; ++index) {
            OpDebugRequired(str, "SkBits2Float(");
            data[index] = OpDebugHexToFloat(str);
        }
        switch (verb) {
            case SkPathVerb::kMove: 
                path->moveTo(data[0], data[1]);
                break;
            case SkPathVerb::kLine: 
                path->lineTo(data[0], data[1]);
                break;
            case SkPathVerb::kQuad: 
                path->quadTo(data[0], data[1], data[2], data[3]);
                break;
            case SkPathVerb::kConic: 
                path->conicTo(data[0], data[1], data[2], data[3], data[4]);
                break;
            case SkPathVerb::kCubic: 
                path->cubicTo(data[0], data[1], data[2], data[3], data[4], data[5]);
                break;
            case SkPathVerb::kClose:
                path->close();
                break;
            default:
                OpDebugExit("missing verb");
        }
        if (!xyCount)
            continue;
        OpDebugRequired(str, ");  //");
        const char* runaway = str + 100;
        while (strncmp("path.", str, 5)) {
            if (++str > runaway)
                OpDebugExit("missing path value");
        }
    }
}

void OpInPath::dumpSet(const char*& str) {
    SkPath* path = (SkPath*) externalReference;
    ::dumpSet(str, path);
}

void OpInPath::dumpResolveAll(OpContours* ) {
}
#endif

const void* OpInPath::MakeExternalReference() {
    return new SkPath();
}

void OpInPath::ReleaseExternalReference(const void* ref) {
    return delete (SkPath*) ref;
}

bool OpOutPath::debugIsEmpty() const {
    return skPath(externalReference)->isEmpty();
}

std::string OpOutPath::debugDump(DebugLevel , DebugBase b) const {
    SkPath* skPath = (SkPath*) externalReference;
    SkDynamicMemoryWStream memoryStream;
    skPath->dump(&memoryStream, DebugBase::hex == b);
    std::string str;
    str.resize(memoryStream.bytesWritten());
    memoryStream.copyTo(str.data());
    str.pop_back();
    return str;
}

void OpOutPath::dumpSet(const char*& str) {
    SkPath* path = (SkPath*) externalReference;
    ::dumpSet(str, path);
}

void OpOutPath::dumpResolveAll(OpContours* ) {
}

void* OpOutPath::MakeExternalReference() {
    return new SkPath();
}

void OpOutPath::ReleaseExternalReference(void* ref) {
    return delete (SkPath*) ref;
}

static SkPoint toSkPoint(OpPoint pt) {
    return SkPoint::Make(pt.x, pt.y);
}

// return false to abort output
bool OpCurve::output(OpOutPath& path, bool firstPt, bool lastPt) {
    SkPath* skpath = skPath(path.externalReference);
    if (firstPt)
        skpath->moveTo(toSkPoint(pts[0]));
    switch (type) {
    case OpType::line: 
        skpath->lineTo(toSkPoint(pts[1])); 
        break;
    case OpType::quad: 
        skpath->quadTo(toSkPoint(pts[1]), toSkPoint(pts[2])); 
        break;
    case OpType::conic: 
        skpath->conicTo(toSkPoint(pts[1]), toSkPoint(pts[2]), weight); 
        break;
    case OpType::cubic: 
        skpath->cubicTo(toSkPoint(pts[1]), toSkPoint(pts[2]), toSkPoint(pts[3])); 
        break;
    default:
        OP_ASSERT(0);
        return false;
    }
    if (lastPt)
        skpath->close();
    return true;
}

bool OpContours::build(OpInPath& path, OpOperand operand) {
    const SkPath& skpath = *skPath(path.externalReference);
    setFillType(operand, SkPathFillType::kEvenOdd == skpath.getFillType()
            || SkPathFillType::kInverseEvenOdd == skpath.getFillType() 
            ? OpFillType::evenOdd : OpFillType::winding);
    OpContour* head = makeContour(operand);
    SkPath::RawIter iter(skpath);
    SkPath::Verb verb;
    do {
        OpPoint pts[4];
        verb = iter.next((SkPoint*) pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            head = addMove(head, operand, pts);
            break;
        case SkPath::kLine_Verb:
            head->addLine(pts);
            break;
        case SkPath::kQuad_Verb:
            if (!head->addQuad(pts))
                return false;
            break;
        case SkPath::kConic_Verb:
            if (!head->addConic(pts, iter.conicWeight()))
                return false;
            break;
        case SkPath::kCubic_Verb:
            if (!head->addCubic(pts))
                return false;
            break;
        case SkPath::kClose_Verb:
            break;
        case SkPath::kDone_Verb:
            break;
        }
    } while (verb != SkPath::kDone_Verb);
    head->addClose();
    return true;
}
#endif
