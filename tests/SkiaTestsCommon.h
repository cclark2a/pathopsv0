// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTestsCommon_DEFINED
#define OpSkiaTestssCommon_DEFINED

#include <string>

namespace skiatest {
    struct Reporter {
        Reporter() {
        }
        Reporter(const char* name, const char* sub) {
            filename = name;
            subname = sub;
        }
        bool allowExtendedTest();
        void bumpTestCount() {}
        bool verbose() { return false; }
        // skia's testSimplifyTrianglesMain appears to be missing the test name
        // construct one out of the name passed to initTests and a running count
        std::string testname;  // not in skia's version
        std::string filename;  // not in skia's version
        std::string subname;  // not in skia's version
    };
}


#endif
