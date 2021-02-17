/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {

  private VictorSPX insideCollectRoller;
  private VictorSPX outsideCollectRoller;

  private VictorSPX vIndexA;
  private VictorSPX vIndexB;

  private boolean vIndexState = false;
  private double vIndexSpeed = 0.5;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    insideCollectRoller = new VictorSPX(RobotMap.INSIDE_COLLECT_ROLLER_MOTOR);
    outsideCollectRoller = new VictorSPX(RobotMap.OUTSIDE_COLLECT_ROLLER_MOTOR);
    vIndexA = new VictorSPX(RobotMap.V_INDEX_MOTOR_A);
    vIndexB = new VictorSPX(RobotMap.V_INDEX_MOTOR_B);

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
    vIndexB.setInverted(true);
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
    insideCollectRoller.set(ControlMode.PercentOutput, -0.8);
    outsideCollectRoller.set(ControlMode.PercentOutput, 0.8);
  }

  public void dispense() {
    insideCollectRoller.set(ControlMode.PercentOutput, 1);
    outsideCollectRoller.set(ControlMode.PercentOutput, -1);
  }

  public void kick() {
    insideCollectRoller.set(ControlMode.PercentOutput, -.8);
  }

  public void stop() {
    insideCollectRoller.set(ControlMode.PercentOutput, 0);
    outsideCollectRoller.set(ControlMode.PercentOutput, 0);
  }
}
