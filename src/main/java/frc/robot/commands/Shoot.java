/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPosition;

public class Shoot extends CommandBase {
  private Shooter shooter;
  private LimelightVision vision;
  private Indexer indexer;

  private ShooterPosition shooterPosition;
  private long startTime;
  private boolean isAuto;
  /**
   * Creates a new Shoot.
   */
  public Shoot(Shooter shooter, Indexer indexer, LimelightVision vision, ShooterPosition shooterPosition, boolean isAuto) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.vision = vision;
    this.indexer = indexer;
    this.shooterPosition = shooterPosition;
    this.isAuto = isAuto;
  }
  public Shoot(Shooter shooter, Indexer indexer, LimelightVision vision, boolean isAuto) {
    this(shooter, indexer, vision, shooter.getSelectedPosition(), isAuto);
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
    /*ShooterPosition shooterPosition = this.shooterPosition;
    if(vision.isAreaWithin(Constants.AUTO_LINE_MIN_AREA, Constants.AUTO_LINE_MAX_AREA)) {
      shooterPosition = ShooterPosition.AUTO_LINE;
    } else if (vision.isAreaWithin(Constants.CR_CLOSE_MIN_AREA, Constants.CR_CLOSE_MAX_AREA)) {
      shooterPosition = ShooterPosition.CR_CLOSE;
    } else if (vision.isAreaWithin(Constants.DOWNTOWN_MIN_AREA, Constants.DOWNTOWN_MAX_AREA)) {
      shooterPosition = ShooterPosition.DOWNTOWN;
    }
    if(shooterPosition != this.shooterPosition) {
      shooter.setGainPreset(shooterPosition);
      this.shooterPosition = shooterPosition;
    }*/
    if(shooter.isAtSetpoint()) {
      shooter.setFeederSpeed(-1);
      indexer.kick();
      indexer.startVIndexer();
    } else {
      shooter.setFeederSpeed(0);
      indexer.stop();
      indexer.stopVIndexer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.pidOff();
    shooter.resetGainPreset();
    shooter.setSpeed(0);
    shooter.setFeederSpeed(0);
    indexer.stop();
    indexer.stopVIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isAuto) {
      return (System.currentTimeMillis() - startTime) > 5000;
    } else {
      return false;
    }
  }
}
