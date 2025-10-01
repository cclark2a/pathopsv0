#include "debugger/OpDebugPicture.h"
#include "include/shim/surface.h"
#include "include/core/path_builder.h"
#include "src/raster/raster_canvas.h"

using namespace pentrek;

void Window::pentrek_draw(char* bits, int width, int height, int scan) {
    if (verboseLevel) OpDebugOut("pentrek_draw " + name + "\n");
    auto shim = ShimContext::MakeRaster();
    auto pm = Pixmap::C32(width, height, (Premul32*) bits, scan);
    RasterCanvas canvas(pm);
#if 0
    Paint clrPaint;
    clrPaint.color({1, 1, 1, 1});
    canvas.drawIRect({0, 0, width, height}, clrPaint);
#endif
    int debugCount = 0;
    PictureWindow& picture = debuggerState->pictureWindow;
    for (DebuggerPoly& poly : polys) {
        if (!drawOne(poly))
            continue;
        size_t index = 0;
        for (size_t count : poly.contours) {
            Span<Point> points((Point*) (&poly.device.front() + index), count);
            index += count;
            PathBuilder bu;
            bu.addPoly(points, false);
            auto path = bu.snapshot();
            Paint paint;
            auto component = [poly](int bit) { return ((poly.color >> bit) & 0xFF) / 255.f; };
            paint.color({ component(16), component(8), component(0), component(24) });
            paint.stroke(!!poly.thickness);
            if (poly.thickness)
                paint.width(poly.thickness * 2);
            canvas.drawPath(path, paint);
            ++debugCount;
        }
    }
}
