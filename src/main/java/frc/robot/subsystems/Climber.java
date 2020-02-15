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
  private VictorSPX winch;
  private VictorSPX hook;

  private Solenoid armPistonA = new Solenoid(RobotMap.ARM_PISTON_A);
  private Solenoid armPistonB = new Solenoid(RobotMap.ARM_PISTON_B);

  /**
   * Creates a new Climber.
   */
  public Climber() {
    winch = new VictorSPX(RobotMap.WINCH);
    hook = new VictorSPX(RobotMap.HOOK);
  }

  public void climb() {
    winch.set(ControlMode.PercentOutput, 1);
  }

  public void retract() {
    winch.set(ControlMode.PercentOutput, -1);
  }

  public void stop() {
    winch.set(ControlMode.PercentOutput, 0);
  }

  public void hookEnable() {
    hook.set(ControlMode.PercentOutput, 1);
  }
  
  public void hookDisable() {
    hook.set(ControlMode.PercentOutput, 0);
  }

  public void hookRetract() {
    hook.set(ControlMode.PercentOutput, -1);
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
