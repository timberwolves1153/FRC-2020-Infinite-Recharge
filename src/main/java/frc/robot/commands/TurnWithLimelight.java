/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LimelightVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnWithLimelight extends PIDCommand {
  private boolean canFinishCommand = false;
  private int counter = 1;
  private double initialGyroAngle;

  private Drive drive;
  private LimelightVision vision;
  /**
   * Creates a new TurnWithLimelight.
   */
  public TurnWithLimelight(Drive drive, LimelightVision vision) {
    super(
        // The controller that the command will use
        new LimelightVision().getController(),
        // This should return the measurement
        vision::getTargetX,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        //output -> drive.arcadeDrive(0, output * 0.25), 
        output -> System.out.println(output),
        drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(5, 5);
    getController().enableContinuousInput(-29.8, 29.8);
    initialGyroAngle = drive.getImuAngle();
    this.drive = drive;
    this.vision = vision;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter % 20 == 0) {
      canFinishCommand = true;
      System.out.println(canFinishCommand);
    } else {
      canFinishCommand = false;
    }
    counter++;
    return canFinishCommand && getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    vision.setPipeline(0);
    drive.arcadeDrive(0, 0);
    Robot.m_robotContainer.lastLimelightTurnAngleDifference = drive.getImuAngle() - initialGyroAngle;
  }
}
