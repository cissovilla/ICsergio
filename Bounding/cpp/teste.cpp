#include "./src/stdafx.h"
#include "./src/ap.h"
#include "./src/statistics.h"
#include "./src/alglibmisc.h"
#include "./src/linalg.h"
#include "./src/specialfunctions.h"
#include "./src/alglibinternal.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "./src/interpolation.h"

using namespace alglib;


int main(int argc, char **argv)
{
    //
    // We use cubic spline to do grid differentiation, i.e. having
    // values of f(x)=x^2 sampled at 5 equidistant nodes on [-1,+1]
    // we calculate derivatives of cubic spline at nodes WITHOUT
    // CONSTRUCTION OF SPLINE OBJECT.
    //
    // There are efficient functions spline1dgriddiffcubic() and
    // spline1dgriddiff2cubic() for such calculations.
    //
    // We use default boundary conditions ("parabolically terminated
    // spline") because cubic spline built with such boundary conditions
    // will exactly reproduce any quadratic f(x).
    //
    // Actually, we could use natural conditions, but we feel that
    // spline which exactly reproduces f() will show us more
    // understandable results.
    //
    real_1d_array x = "[-1.0,-0.5,0.0,+0.5,+1.0]";
    real_1d_array y = "[+1.0,0.25,0.0,0.25,+1.0]";
    real_1d_array d1;
    real_1d_array d2;

    //
    // We calculate first derivatives: they must be equal to 2*x
    //
    spline1dgriddiffcubic(x, y, d1);
    printf("%s\n", d1.tostring(3).c_str()); // EXPECTED: [-2.0, -1.0, 0.0, +1.0, +2.0]

    //
    // Now test griddiff2, which returns first AND second derivatives.
    // First derivative is 2*x, second is equal to 2.0
    //
    spline1dgriddiff2cubic(x, y, d1, d2);
    printf("%s\n", d1.tostring(3).c_str()); // EXPECTED: [-2.0, -1.0, 0.0, +1.0, +2.0]
    printf("%s\n", d2.tostring(3).c_str()); // EXPECTED: [ 2.0,  2.0, 2.0,  2.0,  2.0]
    return 0;
}
