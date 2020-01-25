/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {

  private TalonSRX vIndexA;
  private TalonSRX vIndexB;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    vIndexA = new TalonSRX(RobotMap.V_INDEX_MOTOR_A);
    vIndexB = new TalonSRX(RobotMap.V_INDEX_MOTOR_B);

    configMaster();
  }

  private void configMaster() {
    vIndexB.follow(vIndexA);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
