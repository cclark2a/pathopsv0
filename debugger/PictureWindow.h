// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef PictureWindow_DEFINED
#define PictureWindow_DEFINED

#include "DebuggerWindow.h"

struct PictureWindow : public DebuggerWindow {
    PictureWindow(DebuggerState* state);
    void addGrid();
    void addHulls();
    void addLabel(std::string , OpPoint , uint32_t color);
    void addLabels();
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
    double dummy;  // alignment of double following struct of floats is not portable
    double scale = 0; // factor to go from local to device (zero is uninitialized)
    float zoomFactor = 1;
    int zoomer = 0;
//    int debugPrecision = 0;
    int gridIntervals = 8;
    bool drawCentersOn = false;
    bool drawControlsOn = false;
    bool drawEdgeHullsOn = false;
    bool drawFillOn = false;
    bool drawGridOn = true;
    bool drawHullsOn = false;
    bool drawIDsOn = true;
    bool drawPointsOn = true;
    bool drawTangentsOn = false;
    bool drawTsOn = false;
    bool drawValuesOn = true;
    bool drawWindingsOn = true;
    bool drawGridLinear = false;
    bool keyboardZoom = false;
};

#endif
