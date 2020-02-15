/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;

public class DriveForEncoderWithCollector extends CommandBase {
  private Drive drive;
  private Indexer collector;

  private double encoderTicks;
  private double initialEncoderPosition;
  private int driveDirection;
  /**
   * Creates a new DriveForEncoder.
   */
  public DriveForEncoderWithCollector(double encoderTicks, int driveDirection, Drive drive, Indexer indexer) {
    addRequirements(drive);
    this.drive = drive;
    this.collector = indexer;
    this.encoderTicks = encoderTicks;
    this.driveDirection = driveDirection;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialEncoderPosition = drive.getLeftEncoder().getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(driveDirection * 0.8, 0);
    collector.collect();
    collector.outsideCollect();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    collector.stop();
    collector.outsideStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drive.getLeftEncoder().getPosition() - initialEncoderPosition) >= encoderTicks;
  }
}
