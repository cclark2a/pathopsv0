// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef CompareWindow_DEFINED
#define CompareWindow_DEFINED

#include "DebuggerWindow.h"

struct CompareWindow;
struct DebugRaster;
struct OpDebugSamples;

typedef DrawLevel (*CompareAction)(const DebuggerEvent* , CompareWindow* , 
        const OpDebugSamples& , int row);

// !!! eventually move this into debugging strings in curves or maybe debugger bits txt
struct CompareLabel : Bumper {
    CompareLabel(CompareWindow* w)
        : window(w) {
    }

    std::string labelAt(int index) override;
    int size() const override;

    CompareWindow* window;
};

struct CompareWindow : public DebuggerWindow {
    CompareWindow(DebuggerState* state);
    DrawLevel doType(CompareAction , const DebuggerEvent* );
    SDL_AppResult draw() override;
    static std::string DrawCompareLabel(int index);
    DrawLevel event(const DebuggerEvent& ) override;  // defer to topmost window
    bool readBits();
    void update();  // draw help corresponding to topmost window
    DebugRaster* debugRaster = nullptr;
    CompareLabel leftLabel;
    CompareLabel rightLabel;
    OpRect leftFocus;
    OpRect rightFocus;
    time_t lastTime = 0;
    float scale = OpNaN;
    float xOffset = OpNaN;
    float yOffset = OpNaN;
    int updateAttempts = 0;
    int maxUpdateAttempts = 16;
    int margin = 1;
};

#endif
