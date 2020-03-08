/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * A proprietary model for a trapezoidal motion profile.
 */
public class RevoMotionProfile {

    private final double maxVelocity;
    private final double maxAcceleration;

    private double accelEnd;
    private double cruiseEnd;
    private double decelEnd;

    /**
     * Construct a re-usable trapezoidal motion profile.
     * @param maxVelocity Cruising velocity in units distance/units time
     * @param maxAcceleration Cruising velocity in units distance/units time^2
     * @param distance Target distance in units distance
     */
    public RevoMotionProfile(double maxVelocity, double maxAcceleration, double distance) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        double maxAccelDuration = maxVelocity / maxAcceleration;
        double maxAccelDisplacement = maxAcceleration * Math.pow(maxAccelDuration, 2);
        double cruiseDisplacement = distance - maxAccelDisplacement;
        double cruiseDuration = cruiseDisplacement / maxVelocity;

        if (cruiseDisplacement > 0) {
            // In this case, we do reach cruising speed
            accelEnd = maxAccelDuration;
            cruiseEnd = accelEnd + cruiseDuration;
            decelEnd = cruiseEnd + maxAccelDuration;
        } else {
            // We are projected to reach the target distance so quickly that we never reach cruising speed
            double accelDuration = Math.sqrt(distance / maxAcceleration);
            accelEnd = accelDuration;
            cruiseEnd = accelEnd;
            decelEnd = cruiseEnd + accelDuration;
        }
    }

    /**
     * Calculate the current velocity according to the motion profile.
     * @param time Time elapsed since beginning of motion profile execution in units time
     * @return Output velocity in units distance/units time
     */
    public double getVelocity(double time) {
        if (time >= decelEnd) {
            return 0;
        } else if (time >= cruiseEnd) {
            double decelElapsed = time - cruiseEnd;
            return maxVelocity - decelElapsed * maxAcceleration;
        } else if (time >= accelEnd) {
            return maxVelocity;
        } else {
            return time * maxAcceleration;
        }
    }
}
