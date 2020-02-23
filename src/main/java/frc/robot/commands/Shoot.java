/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPosition;

public class Shoot extends CommandBase {
  private Shooter shooter;
  private ShooterPosition shooterPosition;
  private long startTime;
  /**
   * Creates a new Shoot.
   */
  public Shoot(Shooter shooter, ShooterPosition shooterPosition) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.shooterPosition = shooterPosition;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setGainPreset(shooterPosition);
    shooter.pidOn();
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.pidOff();
    shooter.resetGainPreset();
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > 3;
  }
}
