/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TurnForAngleCommand extends CommandBase {
  private double degreesToTurn;
  private double speedTurn;
  private double turnSide;
  
  private Drive drive;

  private double startingDegrees;

  public TurnForAngleCommand(double degreesToTurn, double speedTurn, Drive drive) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(drive);
    this.degreesToTurn = degreesToTurn;
    this.drive = drive;
    this.speedTurn = speedTurn;
    speedTurn = 0.6;
    startingDegrees = drive.getImuAngle();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    turnSide = Math.copySign(1.0, degreesToTurn);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    drive.arcadeDrive(0, turnSide * speedTurn);
  }

  public boolean isWithinValue() {
    boolean isFinished = false;
    if((drive.getImuAngle() - startingDegrees) < (degreesToTurn - 30)) {
      isFinished = false;
    } else if ((drive.getImuAngle() - startingDegrees) >= (degreesToTurn - 30)) {
      isFinished = true;
      System.out.println("Ending Command");
    }
    return isFinished;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return isWithinValue();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }
}
