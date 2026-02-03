#include "MathUtils.h"
#include <math.h>

namespace MathUtils {

void eulerToRotationMatrix(float roll, float pitch, float yaw, float R[3][3]) {
    float r = degToRad(roll);
    float p = degToRad(pitch);
    float y = degToRad(yaw);

    float sr = sinf(r), cr = cosf(r);  // roll icin sin/cos
    float sp = sinf(p), cp = cosf(p);  // pitch icin sin/cos
    float sy = sinf(y), cy = cosf(y);  // yaw icin sin/cos

    // Rotasyon matrisi elemanlari
    // R = Rz(yaw) * Ry(pitch) * Rx(roll) formulu acilimi
    //
    // Ilk satir: X ekseni donusumu
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    // Ikinci satir: Y ekseni donusumu
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    // Ucuncu satir: Z ekseni donusumu
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}


void rotateVector(const float v[3], const float R[3][3], float out[3]) {
    out[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    out[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    out[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

}
