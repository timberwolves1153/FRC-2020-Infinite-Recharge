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

  /**
   * Outputs and reads extra telemetry from Shuffleboard for effortless testing;
   * this option should be set to `false` during competitions as important values
   * are then read from defaults set in code
   */
  private static final boolean TEST = true;

  // Tuned shooter PID values for common shooting positions
  private static final double[] SHOOTER_P = {.0005, .0005, .0005};
  private static final double[] SHOOTER_F = {(.67/3400), (.75/3900), (.82/4500)};
  private static final double[] SHOOTER_SETPOINT = {3400, 3700, 4500};
  //Setpoint Values: 3400, 4100, 4500

  private double p, i, d, f, setpoint;

  private ShooterPosition defaultPosition = ShooterPosition.CR_CLOSE;
  private ShooterPosition selectedPosition;

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

  /**
   * Sets all config parameters for motor controllers
   * 
   * Motor controller parameters are set in code and then saved to the controller
   * flash by design so that if a motor controller misses initial configuration,
   * it will default to the last-saved values
   */
  private void configSparkParams() {
    motorA.restoreFactoryDefaults();
    motorB.restoreFactoryDefaults();
    feeder.restoreFactoryDefaults();

    motorA.setIdleMode(IdleMode.kCoast);
    motorB.setIdleMode(IdleMode.kCoast);
    feeder.setIdleMode(IdleMode.kCoast);

    motorB.follow(motorA, true);

    resetGainPreset();
    shooterPID.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

    motorA.burnFlash();
    motorB.burnFlash();
    feeder.burnFlash();
  }

  /**
   * Handles all Shooter-related SmartDashboard read/write updates
   */
  public void updateDashboard() {
    if (TEST) {
      SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
      SmartDashboard.putNumber("Shooter Power", motorA.get());
      SmartDashboard.putNumber("Shooter Position", defaultPosition.getPosition());

      double p = SmartDashboard.getNumber("Shooter P", this.p);
      double i = SmartDashboard.getNumber("Shooter I", this.i);
      double d = SmartDashboard.getNumber("Shooter D", this.d);
      double f = SmartDashboard.getNumber("Shooter F", this.f);
      double setpoint = SmartDashboard.getNumber("Shooter Setpoint", this.setpoint);

      // If there are any changes from Shuffleboard, update the PID Controller
      if (this.p != p || this.i != i || this.d != d || this.f != f || this.setpoint != setpoint) {
        setPIDGains(p, i, d, f, setpoint);
      }
    }
  }

  /**
   * Sets all tuning parameters for the rotation-regulating PID
   * @param p P gain
   * @param i I gain
   * @param d D gain
   * @param f Feed-forward gain
   * @param setpoint RPM setpoint
   */
  public void setPIDGains(double p, double i, double d, double f, double setpoint) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.f = f;
    this.setpoint = setpoint;

    shooterPID.setP(p);
    shooterPID.setI(i);
    shooterPID.setD(d);
    shooterPID.setFF(f);

    if (TEST) {
      SmartDashboard.putNumber("Shooter P Gain", p);
      SmartDashboard.putNumber("Shooter I Gain", i);
      SmartDashboard.putNumber("Shooter D Gain", d);
      SmartDashboard.putNumber("Shooter F Gain", f);
      SmartDashboard.putNumber("Shooter Setpoint", setpoint);
    }
  }

  /**
   * Sets a flag that will signal the PID to run periodically
   */
  public void pidOn() {
    if (!pidEnabled) {
      pidEnabled = true;
      shooterPID.setIAccum(0);
    }
  }

  /**
   * Sets a flag that will prevent the PID from running periodically
   */
  public void pidOff() {
    pidEnabled = false;
    motorA.set(0);
  }

  /**
   * Runs all periodic logic for the PID controller, called periodically while
   * the PID is enabled
   */
  private void pidPeriodic() {
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

  /**
   * Convenience wrapper that sets PID parameters according to pre-stored values that
   * correspond with the given shooter position
   * @param shooterPosition Indicates the shooter position for which PID values will be updated
   */
  public void setGainPreset(ShooterPosition shooterPosition) {
    int pos = shooterPosition.getPosition();
    selectedPosition = shooterPosition;

    // I and D values are not pulled from an array since these values are always zero
    setPIDGains(SHOOTER_P[pos], 0, 0, SHOOTER_F[pos], SHOOTER_SETPOINT[pos]);
  }

  /**
   * Convenience method that sets PID parameters according to pre-stored default values
   */
  public void resetGainPreset() {
    setGainPreset(defaultPosition);
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

  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  public double getSetPoint() {
    return SHOOTER_SETPOINT[selectedPosition.getPosition()];
  }

  public boolean isAtSetpoint() {
    return shooterEncoder.getVelocity() >= getSetPoint(); 
  }

  public ShooterPosition getSelectedPosition() {
    return selectedPosition;
  }

  @Override
  public void periodic() {
    if (pidEnabled) {
      pidPeriodic();
    }
  }
}
