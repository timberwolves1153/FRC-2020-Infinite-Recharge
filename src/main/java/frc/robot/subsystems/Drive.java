/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private boolean pidEnabled = false;

  // Drive Motion Profiling
  private CANPIDController leftPID;
  private CANPIDController rightPID;

  public double p, i, d, setpoint;
  private static final double MAX_OUTPUT = 1;
  private static final double MIN_OUTPUT = -1;

  private static final ADIS16470_IMU imu = new ADIS16470_IMU();

  //private static final int SMART_MOTION_SLOT = 0;
  public double maxVelocity, minOutputVelocity, maxAccel;

  private DifferentialDrive differentialDrive;

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

    configSparkParams();

    differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  }

  private void configSparkParams() {
    // Following reset is a soft reset and does not persist between power cycles
    leftMaster.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();

    //Config Master Sparks
    leftFollowerA.follow(leftMaster);
    rightFollowerA.follow(rightMaster);

    //Sets default brake mode
    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    leftFollowerA.setIdleMode(IdleMode.kBrake);
    rightFollowerA.setIdleMode(IdleMode.kBrake);

    // Velocity Regulation PID constants
    p = 0.0427;
    i = 0;
    d = 0;
    setpoint = 24;

    setupPIDConstants(leftPID, p, i, d);
    setupPIDConstants(rightPID, p, i, d);

    leftEncoder.setVelocityConversionFactor(0.0315561762079);
    rightEncoder.setVelocityConversionFactor(0.0315561762079);

    // Trapezoidal Motion Profiling Parameters
    /*maxVelocity = 24;
    minOutputVelocity = 0;
    maxAccel = 6;

    setupProfilingParameters(leftPID, maxVelocity, minOutputVelocity, maxAccel);
    setupProfilingParameters(rightPID, maxVelocity, minOutputVelocity, maxAccel);*/

    SmartDashboard.putNumber("P Gain", p);
    SmartDashboard.putNumber("I Gain", i);
    SmartDashboard.putNumber("D Gain", d);
    SmartDashboard.putNumber("Max Velocity", maxVelocity);
    SmartDashboard.putNumber("Min Output Velocity", minOutputVelocity);
    SmartDashboard.putNumber("Max Accel", maxAccel);
    SmartDashboard.putNumber("Setpoint", setpoint);

    //Save Config Settings
    leftMaster.burnFlash();
    rightMaster.burnFlash();
    leftFollowerA.burnFlash();
    rightFollowerA.burnFlash();
  }

  private void setupPIDConstants(CANPIDController a, double p, double i, double d) {
    a.setP(p);
    a.setI(i);
    a.setD(d);
    a.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
  }

  /*private void setupProfilingParameters(CANPIDController a, double maxVelocity, double minOutputVelocity, double maxAccel) {
    a.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, SMART_MOTION_SLOT);
    a.setSmartMotionMaxVelocity(maxVelocity, SMART_MOTION_SLOT);
    a.setSmartMotionMinOutputVelocity(minOutputVelocity, SMART_MOTION_SLOT);
    a.setSmartMotionMaxAccel(maxAccel, SMART_MOTION_SLOT);
  }*/

  public void arcadeDrive(double power, double turn) {
    differentialDrive.arcadeDrive(power, turn);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Right Velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Power", rightMaster.get());
    SmartDashboard.putNumber("Left Power", leftMaster.get());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    /*double maxVelocity = SmartDashboard.getNumber("Max Velocity", 0);
    double minOutputVelocity = SmartDashboard.getNumber("Min Output Velocity", 0);
    double maxAccel = SmartDashboard.getNumber("Max Accel", 0);*/

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

    /*if ((maxVelocity != this.maxVelocity)) {
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
    }*/
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
    setpoint = SmartDashboard.getNumber("Setpoint", 0);
    leftPID.setReference(setpoint, ControlType.kVelocity);
    rightPID.setReference(-setpoint, ControlType.kVelocity);
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

  @Override
  public void periodic() {
    if (pidEnabled) {
      pidPeriodic();
    }
  }
}
