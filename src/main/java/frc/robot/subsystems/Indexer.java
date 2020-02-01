/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {

  private TalonSRX collectRoller;

  private TalonSRX vIndexA;
  private TalonSRX vIndexB;

  private boolean vIndexState = false;
  private double vIndexSpeed = 0.5;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    collectRoller = new TalonSRX(RobotMap.COLLECT_ROLLER_MOTOR);
    vIndexA = new TalonSRX(RobotMap.V_INDEX_MOTOR_A);
    vIndexB = new TalonSRX(RobotMap.V_INDEX_MOTOR_B);

    configMaster();

    SmartDashboard.putNumber("V Indexer Speed", vIndexSpeed);
  }

  public void updateDashboard() {
    double vIndexSpeed = SmartDashboard.getNumber("V Indexer Speed", 0);
    if (vIndexSpeed != this.vIndexSpeed) {
      if (vIndexState) {
        vIndexA.set(ControlMode.PercentOutput, vIndexSpeed);
      }
      this.vIndexSpeed = vIndexSpeed;
    }
  }

  private void configMaster() {
    vIndexB.follow(vIndexA);
  }

  public void startVIndexer() {
    vIndexState = true;
    vIndexA.set(ControlMode.PercentOutput, vIndexSpeed);
  }

  public void stopVIndexer() {
    vIndexState = false;
    vIndexA.set(ControlMode.PercentOutput, 0);
  }

  public void collect() {
    collectRoller.set(ControlMode.PercentOutput, 1);
  }

  public void dispense() {
    collectRoller.set(ControlMode.PercentOutput, -1);
  }

  public void stop() {
    collectRoller.set(ControlMode.PercentOutput, 0);
  }
}
