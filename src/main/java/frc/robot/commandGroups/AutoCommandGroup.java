/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveForEncoder;
import frc.robot.commands.DriveForEncoderWithCollector;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnForAngleCommand;
import frc.robot.commands.TurnWithLimelight;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandGroup.
   */
  public AutoCommandGroup(Drive drive, LimelightVision vision, Shooter shooter, Indexer indexer, RobotContainer robotContainer) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    /*addCommands(new TurnWithLimelight(drive, vision));
    addCommands(new Shoot(shooter, vision, ShooterPosition.AUTO_LINE, true));
    addCommands(new TurnForAngleCommand(-robotContainer.lastLimelightTurnAngleDifference, 0.5, drive));
    addCommands(new DriveForEncoderWithCollector(10, 1, drive, indexer));
    addCommands(new TurnWithLimelight(drive, vision));
    addCommands(new Shoot(shooter, vision, ShooterPosition.CR_CLOSE, true));*/
    addCommands(new Shoot(shooter, indexer, vision, ShooterPosition.AUTO_LINE, true));
    //addCommands(new TurnForAngleCommand(17, 0.5, drive));
    addCommands(new DriveForEncoderWithCollector(66, 1, drive, indexer));
    //addCommands(Robot.m_robotContainer.generateRamseteCommand(Robot.m_robotContainer.getAutoLineToControlTrajectory()));
  }
}
