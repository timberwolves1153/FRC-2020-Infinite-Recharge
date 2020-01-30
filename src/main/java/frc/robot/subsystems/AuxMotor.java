/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class AuxMotor extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX motor;

  public AuxMotor() {
    motor = new TalonSRX(RobotMap.AUX_MOTOR);
  }

  public void fullForward() {
    motor.set(ControlMode.PercentOutput, 1);
  }

  public void fullBackward() {
    motor.set(ControlMode.PercentOutput, -1);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }
}
