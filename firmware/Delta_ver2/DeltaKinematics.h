/*******************************************************************************
 * @file    DeltaKinematics.h
 * @brief   Delta Robot Kinematics - Fixed for Singularity at Low Z
 * @version 2.1 - Targeted Fix
 * @date    2026-01-28
 * 
 * FIXES:
 * ✅ Angle limit check (prevent "thẳng tay" at low Z)
 * ✅ Position validation (prevent coordinate errors)
 * ✅ Missing getter implementations
 ******************************************************************************/

#ifndef DeltaKinematics_h
#define DeltaKinematics_h

/*******************************************************************************
 * MATHEMATICAL CONSTANTS
 ******************************************************************************/
#define sqrt3   1.7320508075688772935274463415059f
#define pi      3.1415926535897932384626433832795f

#define sin120  (sqrt3 / 2.0f)
#define cos120  (-0.5f)
#define tan60   (sqrt3)
#define sin30   (0.5f)
#define tan30   (1.0f / sqrt3)

/*******************************************************************************
 * ERROR CODES
 ******************************************************************************/
#define no_error                    1   // Success
#define non_existing_povar_error   -2   // Position unreachable
#define singularity_error          -3   // Dangerous angle (near singularity)

/*******************************************************************************
 * DELTAKINEMATICS CLASS
 ******************************************************************************/
class DeltaKinematics
{
    public:
        // Constructor
        DeltaKinematics(float _ArmLength, float _RodLength, float _BassTri, float _PlatformTri);
        
        // Forward Kinematics
        int forward();
        int forward(float thetaA, float thetaB, float thetaC);
        
        // Inverse Kinematics
        int inverse();
        int inverse(float x0, float y0, float z0);
        
        // ✅ NEW: Validation functions
        bool isReachable(float x0, float y0, float z0);
        bool isSafeAngle(float angle);
        
        // ✅ FIXED: Getter functions (now implemented)
        void getAngles(float &thetaA, float &thetaB, float &thetaC);
        void getPosition(float &xOut, float &yOut, float &zOut);

        // Position Variables (public for direct access)
        float x, y, z;
        float a, b, c;

    private:
        int delta_calcAngleYZ(float *Angle, float x0, float y0, float z0);

        float ArmLength;
        float RodLength;
        float BassTri;
        float PlatformTri;
};

#endif
