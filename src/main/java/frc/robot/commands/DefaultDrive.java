/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;

public class DefaultDrive extends CommandBase {

  private Drive drive;
  private DoubleSupplier power;
  private DoubleSupplier turn;

  private boolean mLastToggleState = false;

  /**
   * Creates a new DefaultDrive.
   */
  public DefaultDrive(Drive drive, DoubleSupplier power, DoubleSupplier turn) {
    this.drive = drive;
    this.power = power;
    this.turn = turn;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean bothPressed = Robot.m_robotContainer.getDriveStick().getRawButtonPressed(6);
    if(bothPressed && !mLastToggleState) {
      Robot.m_robotContainer.teleOpDriveSide = Robot.m_robotContainer.teleOpDriveSide > 0 ? -1 : 1;
    }
    mLastToggleState = bothPressed;
    drive.arcadeDrive(Robot.m_robotContainer.teleOpDriveSide * power.getAsDouble(), -turn.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
