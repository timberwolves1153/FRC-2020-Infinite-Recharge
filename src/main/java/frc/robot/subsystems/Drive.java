/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
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

    configMasterSparks();
    configMotionProfiling();

    differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  }

  private void configMasterSparks() {
    leftFollowerA.follow(leftMaster);
    rightFollowerA.follow(rightMaster);
  }

  private void configMotionProfiling() {
    // Following reset is a soft reset and does not persist between power cycles
    leftMaster.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();

    // Velocity Regulation PID constants
    p = 0.0000036902;
    i = 0.0001314766;
    d = 0;
    setpoint = 0;

    setupPIDConstants(leftPID, p, i, d);
    setupPIDConstants(rightPID, p, i, d);

    // Trapezoidal Motion Profiling Parameters
    /*setupProfilingParameters(leftPID, maxVelocity, minOutputVelocity, maxAccel);
    setupProfilingParameters(rightPID, maxVelocity, minOutputVelocity, maxAccel);*/

    SmartDashboard.putNumber("P Gain", p);
    SmartDashboard.putNumber("I Gain", i);
    SmartDashboard.putNumber("D Gain", d);
    SmartDashboard.putNumber("Max Velocity", maxVelocity);
    SmartDashboard.putNumber("Min Output Velocity", minOutputVelocity);
    SmartDashboard.putNumber("Max Accel", maxAccel);
    SmartDashboard.putNumber("Setpoint", setpoint);
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
    if (!pidEnabled) {
      pidEnabled = true;
      leftPID.setIAccum(0);
      rightPID.setIAccum(0);
    }
  }

  public void pidOff() {
    pidEnabled = false;
  }

  private void pidPeriodic() {
    setpoint = SmartDashboard.getNumber("Setpoint", 0);
    leftPID.setReference(setpoint, ControlType.kVelocity);
    rightPID.setReference(setpoint, ControlType.kVelocity);
  }

  public void setMotorIdleMode(IdleMode mode) {
    leftMaster.setIdleMode(mode);
    leftFollowerA.setIdleMode(mode);

    rightMaster.setIdleMode(mode);
    rightFollowerA.setIdleMode(mode);
  }

  @Override
  public void periodic() {
    if (pidEnabled) {
      pidPeriodic();
    }
  }
}