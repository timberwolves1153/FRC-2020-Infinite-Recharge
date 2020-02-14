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

    p = 0.05;
    i = 0;
    d = 0;
    f = 0.00018510638;
    setpoint = 0;

    setupPIDConstants(shooterPID, p, i, d, f);

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

  private void setupPIDConstants(CANPIDController a, double p, double i, double d, double f) {
    a.setP(p);
    a.setI(i);
    a.setD(d);
    a.setFF(f);
    a.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
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
    //setpoint = SmartDashboard.getNumber("Shooter Setpoint", 0);
    shooterPID.setReference(4700, ControlType.kVelocity);
    System.out.println("Setting setpoint");
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
