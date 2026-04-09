/*******************************************************************************
 * @file    DeltaKinematics.cpp
 * @brief   Delta Robot Kinematics - Implementation with Singularity Protection
 * @version 2.1 - Targeted Fix for "Thẳng tay" issue at low Z
 * @date    2026-01-28
 ******************************************************************************/

#include "DeltaKinematics.h"

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <math.h>


/*******************************************************************************
 * CONSTRUCTOR
 ******************************************************************************/
DeltaKinematics::DeltaKinematics(float _ArmLength, float _RodLength, float _BassTri, float _PlatformTri)
{
  PlatformTri = _PlatformTri;  // end effector (e)
  BassTri = _BassTri;          // base (f)
  RodLength = _RodLength;      // lower arm length (re)
  ArmLength = _ArmLength;      // upper arm length (rf)
  x = y = z = a = b = c = 0.0;
}


/*******************************************************************************
 * FORWARD KINEMATICS
 ******************************************************************************/
int DeltaKinematics::forward()
{
  return forward(a, b, c);
}

int DeltaKinematics::forward(float thetaA, float thetaB, float thetaC)
{
  x = 0.0;
  y = 0.0;
  z = 0.0;

  float t = (BassTri - PlatformTri) * tan30 / 2.0;
  float dtr = pi / 180.0;

  float cosThetaA = cos(thetaA * dtr); 
  float sinThetaA = sin(thetaA * dtr); 
  float cosThetaB = cos(thetaB * dtr);
  float sinThetaB = sin(thetaB * dtr); 
  float cosThetaC = cos(thetaC * dtr); 
  float sinThetaC = sin(thetaC * dtr);

  float y1 = -(t + ArmLength * cosThetaA); 
  float z1 = -ArmLength * sinThetaA;

  float y2 = (t + ArmLength * cosThetaB) * sin30; 
  float x2 = y2 * tan60; 
  float z2 = -ArmLength * sinThetaB;

  float y3 = (t + ArmLength * cosThetaC) * sin30; 
  float x3 = -y3 * tan60; 
  float z3 = -ArmLength * sinThetaC;
  
  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  float w1 = y1 * y1 + z1 * z1;
  float w2 = x2 * x2 + y2 * y2 + z2 * z2; 
  float w3 = x3 * x3 + y3 * y3 + z3 * z3;
  float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1); 
  float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;
  float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2; 
  float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  float aV = a1 * a1 + a2 * a2 + dnm * dnm;
  float bV = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  float cV = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - RodLength * RodLength);

  float dV = bV * bV - 4.0 * aV * cV;
  if (dV < 0.0)
  {
    return non_existing_povar_error;
  }

  z = -0.5 * (bV + sqrt(dV)) / aV;
  x = (a1 * z + b1) / dnm;
  y = (a2 * z + b2) / dnm;

  return no_error;
}


/*******************************************************************************
 * INVERSE KINEMATICS HELPER - ✅ FIXED FOR "THẲNG TAY" ISSUE
 ******************************************************************************/
int DeltaKinematics::delta_calcAngleYZ(float *Angle, float x0, float y0, float z0)
{
  float y1 = -0.5 * tan30 * BassTri;
  y0 -= 0.5 * tan30 * PlatformTri;

  float aV = (x0 * x0 + y0 * y0 + z0 * z0 + ArmLength * ArmLength - RodLength * RodLength - y1 * y1) / (2.0 * z0);
  float bV = (y1 - y0) / z0;

  float dV = -(aV + bV * y1) * (aV + bV * y1) + ArmLength * (bV * bV * ArmLength + ArmLength);
  if (dV < 0.0)
  {
    return non_existing_povar_error;
  }

  float yj = (y1 - aV * bV - sqrt(dV)) / (bV * bV + 1);
  float zj = aV + bV * yj;

  float angle = atan2(-zj, (y1 - yj)) * 180.0 / pi;
  if (isnan(angle))
    return non_existing_povar_error;
  
  // ═══════════════════════════════════════════════════════════════════════
  // ✅ FIX: Prevent "thẳng tay" (singularity) at low Z
  // ═══════════════════════════════════════════════════════════════════════
  // Delta robot singularity occurs when arms are near ±90°
  // Keep 5° safety buffer: -85° to +85° operating range
  // 
  // WHY THIS FIXES YOUR ISSUE:
  // - At low Z (close to table), robot needs large angles
  // - If angle > 85°, arms are almost straight → loss of control
  // - This check rejects dangerous positions BEFORE moving
  // ═══════════════════════════════════════════════════════════════════════
  
  if (angle < -85.0 || angle > 85.0) {
    return singularity_error;  // Góc nguy hiểm!
  }
  
  *Angle = angle;
  return no_error;
}


/*******************************************************************************
 * INVERSE KINEMATICS
 ******************************************************************************/
int DeltaKinematics::inverse()
{
  return inverse(x, y, z);
}

int DeltaKinematics::inverse(float x0, float y0, float z0)
{
  a = 0.0;
  b = 0.0;
  c = 0.0;

  int error = delta_calcAngleYZ(&a, x0, y0, z0);
  if (error != no_error)
    return error;

  error = delta_calcAngleYZ(&b, x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0);
  if (error != no_error)
    return error;

  error = delta_calcAngleYZ(&c, x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0);
  return error;
}


/*******************************************************************************
 * ✅ NEW: GETTER FUNCTIONS - Fix missing implementations
 ******************************************************************************/
void DeltaKinematics::getAngles(float &thetaA, float &thetaB, float &thetaC)
{
  thetaA = a;
  thetaB = b;
  thetaC = c;
}

void DeltaKinematics::getPosition(float &xOut, float &yOut, float &zOut)
{
  xOut = x;
  yOut = y;
  zOut = z;
}


/*******************************************************************************
 * ✅ NEW: CHECK IF ANGLE IS SAFE
 * Prevents "thẳng tay" by validating angle range
 ******************************************************************************/
bool DeltaKinematics::isSafeAngle(float angle)
{
  // Delta robot safe operating range: -85° to +85°
  // 
  // ĐIỀU CHỈNH NẾU CẦN:
  // - Nếu robot vẫn bị "thẳng tay": giảm xuống ±80°
  // - Nếu workspace quá nhỏ: tăng lên ±88° (nhưng rủi ro hơn!)
  
  return (angle >= -85.0f && angle <= 85.0f);
}


/*******************************************************************************
 * ✅ NEW: VALIDATE POSITION BEFORE MOVE
 * Prevents "lỗi di chuyển do tọa độ"
 ******************************************************************************/
bool DeltaKinematics::isReachable(float x0, float y0, float z0)
{
  // ═══════════════════════════════════════════════════════════════════════
  // QUICK CHECKS (Fast rejection)
  // ═══════════════════════════════════════════════════════════════════════
  
  // 1. Z range check
  // ĐIỀU CHỈNH theo robot thực tế sau khi test!
  if (z0 > -50.0f || z0 < -450.0f) {
    return false;
  }
  
  // 2. Horizontal radius check
  float radius = sqrt(x0 * x0 + y0 * y0);
  
  // Conservative workspace limit
  // ĐIỀU CHỈNH sau khi chạy TEST_WORKSPACE command!
  float maxRadius = 200.0f;
  
  if (radius > maxRadius) {
    return false;
  }
  
  // ═══════════════════════════════════════════════════════════════════════
  // FULL IK TEST (Accurate validation)
  // ═══════════════════════════════════════════════════════════════════════
  
  // Save current state
  float prevA = a, prevB = b, prevC = c;
  float prevX = x, prevY = y, prevZ = z;
  
  // Try inverse kinematics for candidate position
  int result = inverse(x0, y0, z0);
  
  // Cache candidate joint angles before restoring
  float candA = a, candB = b, candC = c;
  
  // Restore original state (không thay đổi object)
  a = prevA; b = prevB; c = prevC;
  x = prevX; y = prevY; z = prevZ;
  
  // Check result
  if (result != no_error) {
    return false;  // IK failed or singularity detected
  }
  
  // Double-check all candidate angles are safe
  return isSafeAngle(candA) && isSafeAngle(candB) && isSafeAngle(candC);
}


/*******************************************************************************
 * END OF FILE
 ******************************************************************************/
