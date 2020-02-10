/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  
  private CANSparkMax motorA;
  private CANSparkMax motorB;
  private CANSparkMax accelerator;

  public Shooter() {
    motorA = new CANSparkMax(RobotMap.SHOOTER_MOTOR_A, CANSparkMax.MotorType.kBrushless);
    motorB = new CANSparkMax(RobotMap.SHOOTER_MOTOR_B, CANSparkMax.MotorType.kBrushless);
    accelerator = new CANSparkMax(RobotMap.ACCELERATOR, CANSparkMax.MotorType.kBrushless);

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

    motorA.burnFlash();
    motorB.burnFlash();
    accelerator.burnFlash();
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
    // This method will be called once per scheduler run
  }
}
