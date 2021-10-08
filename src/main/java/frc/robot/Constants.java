/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public class Constants {

    //Color Sensor Calibration Constants
    public static final double yellowHueMin = 85.0;
    public static final double yellowHueMax = 105.0;
    public static final double redHueMin = 20.0;
    public static final double redHueMax = 40.0;
    public static final double greenHueMin = 120.0;
    public static final double greenHueMax = 140.0;
    public static final double blueHueMin = 175.0;
    public static final double blueHueMax = 195.0;

    //Limelight Constants 
    public static final double POSITION_TOLERANCE = 5;
    public static final double VELOCITY_TOLERANCE = 5;
    public static final double AUTO_LINE_MAX_AREA = 0;
    public static final double AUTO_LINE_MIN_AREA = 0;
    public static final double CR_CLOSE_MAX_AREA = 0;
    public static final double CR_CLOSE_MIN_AREA = 0;
    public static final double DOWNTOWN_MAX_AREA = 0;
    public static final double DOWNTOWN_MIN_AREA = 0;

    //Encoder Constants
    public static final double kWheelDiameter = 6;
    public static final double kGearing = 14.933333333;
    public static final double kEncoderCPR = 42;
    public static final double kEncoderConstant = 1 / kGearing;
    //public static final double kEncoderToDistanceMetersConversionFactor = 

    //Ramsete Command Constants
    /*
    public static final double ksVolts = 0.159;
    public static final double kvVoltSecondsPerMeter = 3.87;
    public static final double kaVoltSecondsSquaredPerMeter = 0.503;
    */

    //Ramsete Command Constants
    public static final double ksVolts = 0.176;
    public static final double kvVoltSecondsPerMeter = 3.87;
    public static final double kaVoltSecondsSquaredPerMeter = 0.574;

    public static final double kPDriveVel = 0.05;

    //public static final double kTrackwidthMeters = 1.330086493020229;
    public static final double kTrackwidthMeters = Units.inchesToMeters(22.0);

    public static final double kMaxSpeedMetersPerSecond = 1.0; //1.75
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.75; //1.5

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
