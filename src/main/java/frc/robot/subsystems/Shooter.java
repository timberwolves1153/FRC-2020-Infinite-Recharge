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
  public enum ShooterPosition {
    AUTO_LINE(0), CR_CLOSE(1), DOWNTOWN(2), INVALID(3);

    private int value;
    
    private ShooterPosition(int value) {
      this.value = value;
    }

    public int getPosition() {
      return value;
    }

    public static ShooterPosition fromInt(int value) {
      for(ShooterPosition position : values()) {
        if(position.value == value) {
          return position;
        }
      }
      return INVALID;
    }

    public static int getHighestValue() {
      int highestVal = 0;
      for(ShooterPosition position : values()) {
        if(position.value > highestVal) {
          highestVal = position.value;
        }
      }
      return highestVal - 1;
    }
  }

  public enum Direction {
    kForwards(1), kBackwards(-1);

    private int direction;

    private Direction(int direction) {
      this.direction = direction;
    }

    public int getDirection() {
      return direction;
    }
  }
  
  private CANSparkMax motorA;
  private CANSparkMax motorB;
  private CANSparkMax feeder;

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

  private ShooterPosition defaultPosition = ShooterPosition.CR_CLOSE;

  private static final double MAX_OUTPUT = 1;
  private static final double MIN_OUTPUT = -1;

  public Shooter() {
    motorA = new CANSparkMax(RobotMap.SHOOTER_MOTOR_A, CANSparkMax.MotorType.kBrushless);
    motorB = new CANSparkMax(RobotMap.SHOOTER_MOTOR_B, CANSparkMax.MotorType.kBrushless);
    feeder = new CANSparkMax(RobotMap.FEEDER, CANSparkMax.MotorType.kBrushless);

    shooterEncoder = motorA.getEncoder();

    shooterPID = motorA.getPIDController();

    configSparkParams();
  }

  private void configSparkParams() {
    motorA.restoreFactoryDefaults();
    motorB.restoreFactoryDefaults();
    feeder.restoreFactoryDefaults();

    motorA.setIdleMode(IdleMode.kCoast);
    motorB.setIdleMode(IdleMode.kCoast);
    feeder.setIdleMode(IdleMode.kCoast);

    motorB.follow(motorA, true);

    p = SHOOTER_P[defaultPosition.getPosition()];
    i = 0;
    d = 0;
    f = SHOOTER_F[defaultPosition.getPosition()];
    setpoint = SHOOTER_SETPOINT[defaultPosition.getPosition()];

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
    feeder.burnFlash();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Power", motorA.get());
    SmartDashboard.putNumber("Shooter Position", defaultPosition.getPosition());
  }

  public void updateGains(double p, double f) {
    shooterPID.setP(p);
    shooterPID.setFF(f);
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
   // feeder.set(0.5);
  }

  public void stop(){
    motorA.set(0);
    feeder.set(0);
  }

  public void setFeederSpeed(double speed) {
    feeder.set(speed);
  }

  public void setGainPreset(ShooterPosition shooterPosition) {
    p = SHOOTER_P[shooterPosition.getPosition()];
    f = SHOOTER_F[shooterPosition.getPosition()];
    setpoint = SHOOTER_SETPOINT[shooterPosition.getPosition()];
    SmartDashboard.putNumber("Shooter P Gain", p);
    SmartDashboard.putNumber("Shooter F Gain", f);
    SmartDashboard.putNumber("Shooter Setpoint", setpoint);
    updateGains(p, f);
  }

  public void resetGainPreset() {
    p = SHOOTER_P[defaultPosition.getPosition()];
    f = SHOOTER_F[defaultPosition.getPosition()];
    setpoint = SHOOTER_SETPOINT[defaultPosition.getPosition()];
    SmartDashboard.putNumber("Shooter P Gain", p);
    SmartDashboard.putNumber("Shooter F Gain", f);
    SmartDashboard.putNumber("Shooter Setpoint", setpoint);
    updateGains(p, f);
  }

  public void cycleGainPreset(Direction direction) {
    int highestVal = ShooterPosition.getHighestValue();
    int nextPosition = defaultPosition.getPosition() + direction.getDirection();
    if(nextPosition < 0) {
      defaultPosition = ShooterPosition.fromInt(highestVal);
    } else if(nextPosition > highestVal) {
      defaultPosition = ShooterPosition.fromInt(0);
    } else {
      defaultPosition = ShooterPosition.fromInt(nextPosition);
    }
    setGainPreset(defaultPosition);
  }

  @Override
  public void periodic() {
    if (pidEnabled) {
      pidPeriodic();
    }
  }
}
