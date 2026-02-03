#include "GroundLockController.h"//yere paralel tutacak gimbal komutlarını üretir
#include "../utils/MathUtils.h"
#include <math.h>

GroundLockController::GroundLockController()
    : _enabled(false)
{
}

bool GroundLockController::begin() {
    _enabled = true;
    return true;
}

void GroundLockController::computeGroundLockTarget(
    const IMUFusion::Orientation &bodyOri,
    float &targetPan,
    float &targetTilt
)
{
    _groundRef.computeGroundLockAngles(bodyOri, targetPan, targetTilt);
    targetPan = MathUtils::wrapAngle360(targetPan);
    targetTilt = MathUtils::wrapAngle360(targetTilt);
}
