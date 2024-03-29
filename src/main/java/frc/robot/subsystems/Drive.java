/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;
import java.util.stream.Collectors;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Drive extends SubsystemBase {

  // Drivetrain Motor controllers
  private CANSparkMax leftMaster;
  private CANSparkMax leftFollowerA;

  private CANSparkMax rightMaster;
  private CANSparkMax rightFollowerA;

  // Drivetrain Motor encoders
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;

  private double encoderPositionMetersConversionFactor = (Constants.kWheelDiameter * Math.PI) / (Constants.kEncoderCPR * Constants.kGearing);

  private boolean pidEnabled = false;

  // Drive Motion Profiling
  private CANPIDController leftPID;
  private CANPIDController rightPID;

  public double p, i, d, f, setpoint;
  private static final double MAX_OUTPUT = 1;
  private static final double MIN_OUTPUT = -1;

  private static final ADIS16470_IMU imu = new ADIS16470_IMU();

  private static final int SMART_MOTION_SLOT = 0;
  public double maxVelocity, minOutputVelocity, maxAccel, allowedErr;

  private DifferentialDrive differentialDrive;

  private Solenoid flashlight;
  private Relay flashlightRelay;

  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;

  private Pose2d pose;

  private final Field2d m_field;

  private SimpleMotorFeedforward feedforward;

  private PIDController leftPIDController;
  private PIDController rightPIDController;

  /**
   * Creates a new Drive.
   */
  public Drive() {
    leftMaster = new CANSparkMax(RobotMap.LEFT_MASTER, CANSparkMax.MotorType.kBrushless);
    leftFollowerA = new CANSparkMax(RobotMap.LEFT_FOLLOWER_A, CANSparkMax.MotorType.kBrushless);

    rightMaster = new CANSparkMax(RobotMap.RIGHT_MASTER, CANSparkMax.MotorType.kBrushless);
    rightFollowerA = new CANSparkMax(RobotMap.RIGHT_FOLLOWER_A, CANSparkMax.MotorType.kBrushless);

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    leftPID = leftMaster.getPIDController();
    rightPID = rightMaster.getPIDController();

    flashlight = new Solenoid(4);
    flashlightRelay = new Relay(0);

    odometry = new DifferentialDriveOdometry(imu.getRotation2d());
    kinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);

    feedforward = new SimpleMotorFeedforward(Constants.ksVolts, 
                                             Constants.kvVoltSecondsPerMeter, 
                                             Constants.kaVoltSecondsSquaredPerMeter);

    leftPIDController = new PIDController(Constants.kPDriveVel, 0, 0);
    rightPIDController = new PIDController(Constants.kPDriveVel, 0, 0);

    configSparkParams();

    differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
  }

  private void configSparkParams() {
    // Following reset is a soft reset and does not persist between power cycles
    leftMaster.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();

    //Config Master Sparks
    leftFollowerA.follow(leftMaster);
    rightFollowerA.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(false);

    //Sets default brake mode
    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    leftFollowerA.setIdleMode(IdleMode.kBrake);
    rightFollowerA.setIdleMode(IdleMode.kBrake);

    // Velocity Regulation PID constants
    //More or less stable
    //p = 0.01425
    //d = 0.000112
    //Slightly less stable
    //p = 0.01625
    //d = 0.000175
    //setpoint = 56;
    //Tuned
    //p = 0.000001;
    //d = 0.000005;
    //f = 0.006;

    //Tuned PID Values
    p = 0.000001;
    i = 0;
    d = 0.000005;
    f = 0.006;
    setpoint = 36;

    setupPIDConstants(leftPID, p, i, d, f);
    setupPIDConstants(rightPID, p, i, d, f);

    /*leftEncoder.setVelocityConversionFactor(0.0315561762079);
    rightEncoder.setVelocityConversionFactor(0.0315561762079);*/

    /*leftEncoder.setPositionConversionFactor(encoderPositionMetersConversionFactor);
    rightEncoder.setPositionConversionFactor(encoderPositionMetersConversionFactor);*/

    // Trapezoidal Motion Profiling Parameters
    maxVelocity = 24;
    minOutputVelocity = 0;
    maxAccel = 6;

    setupProfilingParameters(leftPID, maxVelocity, minOutputVelocity, maxAccel, allowedErr);
    setupProfilingParameters(rightPID, maxVelocity, minOutputVelocity, maxAccel, allowedErr);

    SmartDashboard.putNumber("P Gain", p);
    SmartDashboard.putNumber("I Gain", i);
    SmartDashboard.putNumber("D Gain", d);
    SmartDashboard.putNumber("F Gain", f);
    SmartDashboard.putNumber("Max Velocity", maxVelocity);
    SmartDashboard.putNumber("Min Output Velocity", minOutputVelocity);
    SmartDashboard.putNumber("Max Accel", maxAccel);
    SmartDashboard.putNumber("Allowed Error", allowedErr);
    SmartDashboard.putNumber("Setpoint", setpoint);

    //Save Config Settings
    leftMaster.burnFlash();
    rightMaster.burnFlash();
    leftFollowerA.burnFlash();
    rightFollowerA.burnFlash();
  }

  private void setupPIDConstants(CANPIDController a, double p, double i, double d, double f) {
    a.setP(p);
    a.setI(i);
    a.setD(d);
    a.setFF(f);
    a.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
  }

  private void setupProfilingParameters(CANPIDController a, double maxVelocity, double minOutputVelocity, double maxAccel, double allowedErr) {
    a.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, SMART_MOTION_SLOT);
    a.setSmartMotionMaxVelocity(maxVelocity, SMART_MOTION_SLOT);
    a.setSmartMotionMinOutputVelocity(minOutputVelocity, SMART_MOTION_SLOT);
    a.setSmartMotionMaxAccel(maxAccel, SMART_MOTION_SLOT);
    a.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);
  }

  public void arcadeDrive(double power, double turn) {
    differentialDrive.arcadeDrive(power, turn);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    differentialDrive.feed();
  }

  public void setOutput(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
    differentialDrive.feed();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Right Velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Power", rightMaster.get());
    SmartDashboard.putNumber("Left Power", leftMaster.get());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("Gyro Reading", imu.getRotation2d().getDegrees());

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double f = SmartDashboard.getNumber("F Gain", 0);
    double maxVelocity = SmartDashboard.getNumber("Max Velocity", 0);
    double minOutputVelocity = SmartDashboard.getNumber("Min Output Velocity", 0);
    double maxAccel = SmartDashboard.getNumber("Max Accel", 0);
    double allowedErr = SmartDashboard.getNumber("Allowed Error", 0);

    if ((p != this.p)) {
      leftPID.setP(p);
      rightPID.setP(p);
      this.p = p;
    }
    if ((i != this.i)) {
      leftPID.setI(i);
      rightPID.setI(i);
      this.i = i;
    }
    if ((d != this.d)) {
      leftPID.setD(d);
      rightPID.setD(d);
      this.d = d;
    }
    if((f != this.f)) {
      leftPID.setFF(f);
      rightPID.setFF(f);
      this.f = f;
    }

    if ((maxVelocity != this.maxVelocity)) {
      leftPID.setSmartMotionMaxVelocity(maxVelocity, SMART_MOTION_SLOT);
      rightPID.setSmartMotionMaxVelocity(maxVelocity, SMART_MOTION_SLOT);
      this.maxVelocity = maxVelocity;
    }
    if ((minOutputVelocity != this.minOutputVelocity)) {
      leftPID.setSmartMotionMinOutputVelocity(minOutputVelocity, SMART_MOTION_SLOT);
      rightPID.setSmartMotionMinOutputVelocity(minOutputVelocity, SMART_MOTION_SLOT);
      this.minOutputVelocity = minOutputVelocity;
    }
    if ((maxAccel != this.maxAccel)) {
      leftPID.setSmartMotionMaxAccel(maxAccel, SMART_MOTION_SLOT);
      rightPID.setSmartMotionMaxAccel(maxAccel, SMART_MOTION_SLOT);
      this.maxAccel = maxAccel;
    }
    if(allowedErr != this.allowedErr) {
      leftPID.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);
      leftPID.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);
      this.allowedErr = allowedErr;
    }
  }

  public void pidOn() {
    System.out.println("PID On");
    if (!pidEnabled) {
      pidEnabled = true;
      leftPID.setIAccum(0);
      rightPID.setIAccum(0);
    }
  }

  public void pidOff() {
    System.out.println("PID Off");
    pidEnabled = false;
  }

  public double getImuAngle() {
    return imu.getAngle();
  }
  public void calibrateImu() {
    imu.calibrate();
  }
  public void resetImu() {
    imu.reset();
  }

  private void pidPeriodic() {
    //setpoint = SmartDashboard.getNumber("Setpoint", 0);
    leftPID.setReference(setpoint, ControlType.kVelocity, 0);
    rightPID.setReference(-setpoint, ControlType.kVelocity, 0);
  }

  public void setMotorIdleMode(IdleMode mode) {
    leftMaster.setIdleMode(mode);
    leftFollowerA.setIdleMode(mode);

    rightMaster.setIdleMode(mode);
    rightFollowerA.setIdleMode(mode);
  }

  public CANEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public CANEncoder getRightEncoder() {
    return rightEncoder;
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public PIDController getLeftRamsetePIDController() {
    return leftPIDController;
  }

  public PIDController getRightRamsetePIDController() {
    return rightPIDController;
  }

  public void setSetpoint(double setpointIn) {
    setpoint = setpointIn;
  }

  public boolean getPIDEnabled() {
    return pidEnabled;
  }

  public CANPIDController getLeftController() {
    return leftPID;
  }

  public CANPIDController getRightController() {
    return rightPID;
  }

  public void lightOn() {
    flashlight.set(true);
   // flashlightRelay.set(Value.kForward);
    System.out.println("Turning Relay On");
  }

  public void lightOff() {
    flashlight.set(false);
   // flashlightRelay.set(Value.kReverse);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetImu();
    odometry.resetPosition(pose, imu.getRotation2d());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftEncoder.getVelocity() / 14.933333333 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60, 
      -rightEncoder.getVelocity() / 14.933333333 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60);
  }

  public void plotTrajectory(Trajectory trajectory) {
    m_field.getObject("trajectory").setPoses(
      trajectory.getStates().stream()
        .map(state -> state.poseMeters)
        .collect(Collectors.toList()));
  }
  
  public void plotWaypoints(List<Pose2d> waypoints) {
    m_field.getObject("waypoints").setPoses(waypoints);
  }

  @Override
  public void periodic() {
    if (pidEnabled) {
      pidPeriodic();
    }
    pose = odometry.update(imu.getRotation2d(), 
    leftEncoder.getPosition() / 14.933333333 * 2 * Math.PI * Units.inchesToMeters(3.0),
    -rightEncoder.getPosition() / 14.933333333 * 2 * Math.PI * Units.inchesToMeters(3.0));
    m_field.setRobotPose(pose);
    SmartDashboard.putNumber("Odometry X", pose.getX());
    SmartDashboard.putNumber("Odometry Y", pose.getY());
    SmartDashboard.putNumber("Odometry Rotation", pose.getRotation().getRadians());
  }
}
