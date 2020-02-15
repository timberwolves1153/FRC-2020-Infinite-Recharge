/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveForEncoder extends CommandBase {
  private Drive drive;
  private double power;
  private double encoderTicks;
  private int driveDirection;
  private double initialEncoderCount;
  /**
   * Creates a new DriveForDistance.
   */
  public DriveForEncoder(Drive drive, double power, int driveDirection, double encoderTicks) {
    addRequirements(drive);
    this.drive = drive;
    this.power = power;
    this.encoderTicks = encoderTicks;
    this.driveDirection = driveDirection;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialEncoderCount = drive.getLeftEncoder().getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(driveDirection * power, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drive.getLeftEncoder().getPosition() - initialEncoderCount) >= encoderTicks;
  }
}
