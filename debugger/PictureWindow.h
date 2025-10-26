// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef PictureWindow_DEFINED
#define PictureWindow_DEFINED

#include "DebuggerWindow.h"

enum class DrawGrid {
    none,
    linear,
    log
};

extern const std::vector<std::string> drawGridStrs;

struct PictureWindow : public DebuggerWindow {
    PictureWindow(DebuggerState* state);
    void addGrid();
    void addHulls();
    void addIDs();
    void addIntersections();
    void addLabel(std::string , OpPoint , uint32_t color);
    void addPointLabel(OpPoint , OpType& );
    void addPoints();
    void addTangents();
    void addTs();
    void addWindings();
    void addDevice(std::vector<OpPoint>& points, DebuggerPoly& );
    void addEdgeHulls();
    void addFittedBottom(std::string , float xPos, float right, uint32_t color);
    void addFittedSide(std::string , float yPos, float bottom, uint32_t color);
    void addTangent(DebuggerPoly& );
    void addWinding(DebuggerPoly& );
    void clear();
    void colorPolys();
    bool drawOne(DebuggerPoly& ) override;
    uint32_t edgeColor(const OpEdge& );
    DrawLevel event(const DebuggerEvent& ) override;
    void move(OpVector v);  // v is in screen coordinates
    void pan(OpVector v);  // v is percentage of screen
    void playback(const char*& str ) override;
    std::string record() override;
    void resolvePoints();
    void setDevice();
    OpPoint toLocal(OpPoint p);
    OpPoint toDevice(OpPoint p);
    bool touches(const OpRect& bounds);
    void update();
    void zoom(int factor);
#if OP_DEBUG_DUMP
    void dump();
#endif

    OpVector zoomOffset {0, 0};
    double dummy = 0; // alignment of double following struct of floats is not portable
    double scale = 0; // factor to go from local to device (zero is uninitialized)
    float zoomFactor = 1;
    int zoomer = 0;
//    int debugPrecision = 0;
    int gridIntervals = 8;
    DrawGrid drawGrid = DrawGrid::none;
    bool drawCenters = false;
    bool drawControls = false;
    bool drawEdgeHulls = false;
    bool drawFill = false;
    bool drawHulls = false;
    bool drawIDs = true;
    bool drawPoints = true;
    bool drawTangents = false;
    bool drawTs = false;
    bool drawValues = true;
    bool drawWindings = true;
};

#endif
