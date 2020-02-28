/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.Drive;

/**
 * Drives a set distance using a motion profile.
 */
public class MotionProfileCommand extends TrapezoidProfileCommand {

  /**
   * Cruising (maximum) speed, denoted in inches per second
   */
  private static final double kMaxSpeed = 48;

  /**
   * Rate of change in speed, denoted in inches per second squared
   */
  private static final double kMaxAcceleration = 24;

  private Drive drive;

  /**
   * Creates a new MotionProfileCommand command.
   *
   * @param inches The distance to drive.
   * @param drive  The drive subsystem to use.
   */
  public MotionProfileCommand(double inches, Drive drive) {
    super(new TrapezoidProfile(
        new TrapezoidProfile.Constraints(kMaxSpeed,
          kMaxAcceleration),
        // End at desired position in inches; implicitly starts at 0
        new TrapezoidProfile.State(inches, 0)),
        // Pipe the profile state to the drive
        setpointState -> drive.setSetpoint(setpointState),
        // Require the drive
        drive);

    this.drive = drive;

    // Reset drive encoders since we're starting at 0
    drive.resetEncoders();
  }

  @Override
  public void initialize() {
    super.initialize();
    drive.pidOn();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drive.pidOff();
  }
}