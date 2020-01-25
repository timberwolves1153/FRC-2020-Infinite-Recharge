/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  
  private CANSparkMax motorA;
  private CANSparkMax motorB;

  public Shooter() {
    motorA = new CANSparkMax(RobotMap.SHOOTER_MOTOR_A, CANSparkMax.MotorType.kBrushless);
    motorB = new CANSparkMax(RobotMap.SHOOTER_MOTOR_B, CANSparkMax.MotorType.kBrushless);

    configMasterSparks();
  }

  private void configMasterSparks() {
    motorB.follow(motorA);
  }

  public void setSpeed(double speed) {
    motorA.set(speed);
  }

  public void stop(){
    motorA.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}