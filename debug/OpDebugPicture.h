// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image

#include "OpDebug.h"

#if OP_DEBUG_IMAGE

#include "PathOpsTypes.h"
#include "OpSegment.h"

// all points are in device coordinates

// device bounds in float is rounded out

struct OpDebugPicture;

struct OpDebugAddPoly {
    OpDebugAddPoly(OpDebugPicture* p)
        : picture(p) {
    }

    void add(const LinePts& );
    void add(const PathOpsV0Lib::Curve& );
    void add(const OpCurve& c) { return add(c.c); }
    void add(const OpCurve* c) { return add(*c); }
    void add(const OpEdge& e);
    void add(const OpEdge* e) { return add(*e); }
    void add(const OpSegment& s);
    void add(const OpSegment* s) { return add(*s); }
    void fill(const OpSegment& s);
    void fill(const OpSegment* s) { fill(*s); }
    void add(const OpRect& );

    OpDebugPicture* picture;
    const OpEdge* edge = nullptr;
    const OpSegment* segment = nullptr;
    const OpContour* contour = nullptr;
    uint32_t color = debugBlack;
    bool addingFill = false;  // true if added is fill, false if added is frame
};

struct OpDebugPoly {
#if OP_DEBUG_DUMP
    void dump() const;
#endif

    PathOpsV0Lib::Curve c;
    PathOpsV0Lib::CurveData cData;  // used by construction lines
    static constexpr float fill_thickness = 0;
    std::vector<OpPoint> local;    // lines used to draw, in local coordinates
    std::vector<OpPoint> device;    // lines used to draw, in device coordinates
    std::vector<size_t> contours;  // index for each device contour
    const OpEdge* edge = nullptr;
    const OpSegment* segment = nullptr;
    const OpContour* contour = nullptr;
    float thickness = 1;    // special value for fill
    uint32_t color = debugBlack;
    float tStart = 0;
    float tEnd = 1;
};

struct OpDebugText {
#if OP_DEBUG_DUMP
    void dump() const;
#endif

    OpPoint pos;    // always device
    std::string str;
    bool vertical;
};

struct OpDPoint {
    double x;
    double y;
};

struct OpDebugPicture {
    void add(OpPoint , OpPoint , OpDebugAddPoly* );
    void add(std::vector<OpPoint>& points );
    void add(const OpCurve& , OpDebugAddPoly* );
    void addDevice(std::vector<OpPoint>& points, OpDebugPoly& );
    void append(OpPoint );
    void clear();
    void setDevice();
    OpPoint toLocal(OpPoint p);
    OpPoint toDevice(OpPoint p);
#if OP_DEBUG_DUMP
    void dump();
#endif

    OpContext* context;
    std::vector<OpDebugPoly> polys;
    std::vector<OpDebugText> texts;
    OpRect focus;  // local coordinates
    OpVector wh; // screen w/h
    OpVector threshold;
    double scale; // factor to go from local to device
};

#endif
