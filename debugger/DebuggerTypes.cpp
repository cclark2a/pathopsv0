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
        , idType(IDType::edge) {
    }

OpType::OpType(const OpSegment* s)
        : segment(s)
        , id(s->id)
        , idType(IDType::segment) {
    }

OpType::OpType(const OpContour* c)
        : contour(c)
        , id(c->id)
        , idType(IDType::contour) {
    }

OpType::OpType(const OpIntersection* i, IDType t)
        : intersection(i)
        , id(i->id)
        , idType(t) {
    }

OpType::OpType(const Distance* d)
        : distance(d)
        , id(d->debugID)
        , idType(IDType::distance) {
    }

OpType::OpType(const EdgePal* p)
    : pal(p)
    , id(p->unsectID)
    , idType(IDType::pal) {
}

OpType::OpType(const OpTree* t)
        : tree(t)
        , id(t->id)
        , idType(IDType::tree) {
    }

OpType::OpType(const OpLimb* l)
        : limb(l)
        , id(l->id)
        , idType(IDType::limb) {
    }

#if OP_DEBUG_DUMP
extern DebugBase defaultBase;
extern DebugLevel defaultLevel;

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
