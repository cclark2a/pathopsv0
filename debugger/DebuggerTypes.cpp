// (c) 2025, Cary Clark cclark2@gmail.com
#include "DebuggerWindow.h"
#include "OpContour.h"
#include "OpEdge.h"
#include "OpIntersection.h"
#include "OpJoiner.h"
#include "OpSegment.h"

OpType::OpType(const OpEdge* e)
    : edge(e)
    , id(e->id)
    , type(IDType::edge) {
}

OpType::OpType(const OpSegment* s)
    : segment(s)
    , id(s->id)
    , type(IDType::segment) {
}

OpType::OpType(const OpContour* c, int index)
    : contour(c)
    , id(c->id)
    , curveIndex(index)
    , type(IDType::contour) {
    OP_ASSERT(c->context);
}

OpType::OpType(const OpIntersection* i, IDType t)
    : intersection(i)
    , id(i->id)
    , type(t) {
}

OpType::OpType(const Distance* d)
    : distance(d)
    , id(d->debugID)
    , type(IDType::distance) {
}

OpType::OpType(const EdgePal* p)
    : pal(p)
    , id(p->unsectID)
    , type(IDType::pal) {
}

OpType::OpType(const OpTree* t)
    : tree(t)
    , id(t->id)
    , type(IDType::tree) {
}

OpType::OpType(const OpLimb* l)
    : limb(l)
    , id(l->id)
    , type(IDType::limb) {
}

#if OP_DEBUG_VALIDATE
void OpType::validate() const {
    OP_ASSERT(IDType::none != type);
    switch (type) {
        case IDType::none:
            OP_ASSERT(0);
            break;
        case IDType::contour:
            OP_ASSERT(contour);
            OP_ASSERT(contour->context);
            OP_ASSERT(curveIndex >= 0);
            break;
        default:
            break;
    }
}
#endif

#if OP_DEBUG
std::string NativeTextCache::debugDump() const {
    std::string s = "\"" + str + "\" ";
    s += "size:" + size.debugDump(defaultLevel, defaultBase) + " ";
    // skip texture
    s += "color:" + debugDumpColor(defaultLevel, color);
    return s;
}

void OpDebugText::dump(DebuggerWindow& picture) const {
    std::string s;
    s += "pt:" + pt.debugDump(defaultLevel, defaultBase) + " ";
    s += "debugLocal:" + debugLocal.debugDump(defaultLevel, defaultBase) + " ";
    s += "cacheIndex:" + STR(cacheIndex) + " ";
    if (vertical) s += "vertical ";
    s += picture.debugTextDump(cacheIndex);
    OpDebugOut(s + "\n");
}
#endif

