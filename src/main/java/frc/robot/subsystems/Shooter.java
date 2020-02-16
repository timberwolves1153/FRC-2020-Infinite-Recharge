/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  
  private CANSparkMax motorA;
  private CANSparkMax motorB;
  private CANSparkMax accelerator;

  private CANEncoder shooterEncoder;

  private CANPIDController shooterPID;

  private boolean pidEnabled = false;

  // Common shooting positions
  @SuppressWarnings("unused") private static final int SHOOTER_POSITION_AUTO = 0;
  @SuppressWarnings("unused") private static final int SHOOTER_POSITION_CR_CLOSE = 1;
  @SuppressWarnings("unused") private static final int SHOOTER_POSITION_DOWNTOWN = 2;

  // Tuned shooter PID values for common shooting positions
  private static final double[] SHOOTER_P = {.0005, .0005, .0005};
  private static final double[] SHOOTER_F = {(.67/3400), (.75/3900), (.82/4500)};
  private static final double[] SHOOTER_SETPOINT = {3400, 4100, 4500};

  private double p, i, d, f, setpoint;

  private static final double MAX_OUTPUT = 1;
  private static final double MIN_OUTPUT = -1;

  public Shooter() {
    motorA = new CANSparkMax(RobotMap.SHOOTER_MOTOR_A, CANSparkMax.MotorType.kBrushless);
    motorB = new CANSparkMax(RobotMap.SHOOTER_MOTOR_B, CANSparkMax.MotorType.kBrushless);
    accelerator = new CANSparkMax(RobotMap.ACCELERATOR, CANSparkMax.MotorType.kBrushless);

    shooterEncoder = motorA.getEncoder();

    shooterPID = motorA.getPIDController();

    configSparkParams();
  }

  private void configSparkParams() {
    motorA.restoreFactoryDefaults();
    motorB.restoreFactoryDefaults();
    accelerator.restoreFactoryDefaults();

    motorA.setIdleMode(IdleMode.kCoast);
    motorB.setIdleMode(IdleMode.kCoast);
    accelerator.setIdleMode(IdleMode.kCoast);

    motorB.follow(motorA, true);

    // TODO: Clean this up: Pass in desired shooter location once and hotswap values based on controller input
    p = SHOOTER_P[SHOOTER_POSITION_DOWNTOWN];
    i = 0;
    d = 0;
    f = SHOOTER_F[SHOOTER_POSITION_DOWNTOWN];
    setpoint = SHOOTER_SETPOINT[SHOOTER_POSITION_DOWNTOWN];

    // Setup PID constants
    shooterPID.setP(p);
    shooterPID.setI(i);
    shooterPID.setD(d);
    shooterPID.setFF(f);
    shooterPID.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

    SmartDashboard.putNumber("Shooter P Gain", p);
    SmartDashboard.putNumber("Shooter I Gain", i);
    SmartDashboard.putNumber("Shooter D Gain", d);
    SmartDashboard.putNumber("Shooter F Gain", f);
    SmartDashboard.putNumber("Shooter Setpoint", setpoint);

    motorA.burnFlash();
    motorB.burnFlash();
    accelerator.burnFlash();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Power", motorA.get());

    double p = SmartDashboard.getNumber("Shooter P Gain", 0);
    double i = SmartDashboard.getNumber("Shooter I Gain", 0);
    double d = SmartDashboard.getNumber("Shooter D Gain", 0);
    double f = SmartDashboard.getNumber("Shooter F Gain", 0);

    // If PID constants have changed due to input on ShuffleBoard, inform the PID controller
    // of the changes
    if ((p != this.p)) {
      shooterPID.setP(p);
      this.p = p;
    }
    if ((i != this.i)) {
      shooterPID.setI(i);
      this.i = i;
    }
    if ((d != this.d)) {
      shooterPID.setD(d);
      this.d = d;
    }
    if((f != this.f)) {
      shooterPID.setFF(f);
      this.f = f;
    }
  }

  public void pidOn() {
    if (!pidEnabled) {
      pidEnabled = true;
      shooterPID.setIAccum(0);
    }
  }

  public void pidOff() {
    pidEnabled = false;
    motorA.set(0);
  }

  private void pidPeriodic() {
    setpoint = SmartDashboard.getNumber("Shooter Setpoint", 0);
    shooterPID.setReference(setpoint, ControlType.kVelocity);
  }

  public void setSpeed(double speed) {
    motorA.set(speed);
   // accelerator.set(0.5);
  }

  public void stop(){
    motorA.set(0);
    accelerator.set(0);
  }

  public void setAcceleratorSpeed(double speed) {
    accelerator.set(speed);
  }

  @Override
  public void periodic() {
    if (pidEnabled) {
      pidPeriodic();
    }
  }
}
