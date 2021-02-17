/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.util.RevoMotionProfile;

public class RevoMotionProfileCommand extends CommandBase {
  private Drive drive;

  private double startTime;
  private double currentVelocity;

  private int iteration = 0;

  private RevoMotionProfile motionProfile;
  /**
   * Creates a new RevoMotionProfileCommand.
   */
  public RevoMotionProfileCommand(double maxVelocity, double maxAcceleration, double distance, Drive drive) {
    addRequirements(drive);
    this.drive = drive;
    motionProfile = new RevoMotionProfile(maxVelocity, maxAcceleration, distance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.pidOn();
    startTime = System.currentTimeMillis() / 1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    iteration++;
    currentVelocity = motionProfile.getVelocity((System.currentTimeMillis() / 1000) - startTime);
    drive.setSetpoint(currentVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.pidOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (iteration != 0 && currentVelocity == 0);
  }
}
