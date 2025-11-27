// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef PictureWindow_DEFINED
#define PictureWindow_DEFINED

#include "DebuggerWindow.h"

enum class DrawGrid {
    none,
    linear,
    log
};

struct PictureWindow;

struct GridLabel : Bumper {
    GridLabel(PictureWindow* w)
        : window(w) {
    }

    DrawGrid drawGrid() {
        OP_ASSERT(0 <= lastIndex && lastIndex <= (int) DrawGrid::log);
        return (DrawGrid) lastIndex;
    }

    std::string labelAt(int index) override;
    int size() const override;

    PictureWindow* window;
};

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
    DrawLevel doWheel(const DebuggerEvent& , int delta) override;
    static std::string DrawGridLabel(int index);
    bool drawOne(DebuggerPoly& ) override;
    uint32_t edgeColor(const OpEdge& );
    DrawLevel event(const DebuggerEvent& ) override;
    void move(OpVector v);  // v is in screen coordinates
    DrawLevel pan(OpVector v);  // v is percentage of screen
    void playback(const char*& str ) override;
    std::string record() override;
    void resolvePoints();
    void setDevice();
    OpPoint toLocal(OpPoint p) const;
    OpPoint toDevice(OpPoint p) const;
    bool touches(const OpRect& bounds) const;
    void update();
    void zoom(int factor);
#if OP_DEBUG_DUMP
    void dump();
#endif
    GridLabel gridLabel;
    OpVector zoomOffset {0, 0};
    double scale = 0; // factor to go from local to device (zero is uninitialized)
    float thresholdMultiplier = 1;
    float zoomFactor = 1;
    int thresholdWheel = 0;
    int zoomer = 0;
    int gridIntervals = 8;
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
