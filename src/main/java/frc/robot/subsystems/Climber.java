/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private VictorSPX winchA;
  private VictorSPX winchB;

  private Solenoid armPistonA = new Solenoid(RobotMap.ARM_PISTON_A);
  private Solenoid armPistonB = new Solenoid(RobotMap.ARM_PISTON_B);

  /**
   * Creates a new Climber.
   */
  public Climber() {
    winchA = new VictorSPX(RobotMap.WINCH_A);
    winchB = new VictorSPX(RobotMap.WINCH_B);
    configureMaster();
  }

  private void configureMaster() {
    winchB.follow(winchA);
    winchB.setInverted(true);
  }

  public void climb() {
    winchA.set(ControlMode.PercentOutput, -.5);
  }

  public void retract() {
    winchA.set(ControlMode.PercentOutput, .5);
  }

  public void stop() {
    winchA.set(ControlMode.PercentOutput, 0);
  }

  public void setPower(double power) {
    winchA.set(ControlMode.PercentOutput, power);
  }

  public void armUp() {
    armPistonA.set(true);
    armPistonB.set(true);
  }

  public void armDown() {
    armPistonA.set(false);
    armPistonB.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
