#include <cv_bridge/cv_bridge.h>

namespace draw_cv{
    class Draw{
        public:
            static void circle(cv_bridge::CvImageConstPtr cv_ptr, int x, int y, int r, int g, int b);
    };
}

