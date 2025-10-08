#include "OpDebugPicture.h"
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
#else
    memset(bits, 0xff, scan * height);
#endif
    int debugCount = 0;
    Paint paint;
    for (DebuggerPoly& poly : polys) {
        if (!drawOne(poly))
            continue;
        size_t index = 0;
        for (size_t count : poly.contours) {
            Span<Point> points((Point*) (&poly.device.front() + index), count);
            index += count;
            auto component = [poly](int bit) { return ((poly.color >> bit) & 0xFF) / 255.f; };
            paint.color({ component(16), component(8), component(0), component(24) });
            paint.stroke(!!poly.thickness);
            if (poly.thickness)
                paint.width(poly.thickness * 2);
            canvas.drawPoly(points, false, paint);
            ++debugCount;
        }
    }
}
