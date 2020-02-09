/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Drive drive;
    private Shooter shooter;
    private Indexer indexer;
    private ColorSensor colorSensor;
    private Climber climber;

    private XboxController driver;
    private XboxController operator;

    private JoystickButton opX;
    private JoystickButton opY;
    private JoystickButton opA;
    private JoystickButton opB;
    private JoystickButton opBumpLeft;
    private JoystickButton opBumpRight;
    private JoystickButton opStart;
    private JoystickButton opBack;

    public int teleOpDriveSide;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = new Drive();
    shooter = new Shooter();
    indexer = new Indexer();
    colorSensor = new ColorSensor();
    climber = new Climber();

    driver = new XboxController(0);
    operator = new XboxController(1);

    opX = new JoystickButton(operator, XboxController.Button.kX.value);
    opY = new JoystickButton(operator, XboxController.Button.kY.value);
    opA = new JoystickButton(operator, XboxController.Button.kA.value);
    opB = new JoystickButton(operator, XboxController.Button.kB.value);
    opBumpLeft = new JoystickButton(operator, XboxController.Button.kBumperLeft.value);
    opBumpRight = new JoystickButton(operator, XboxController.Button.kBumperRight.value);
    opStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    opBack = new JoystickButton(operator, XboxController.Button.kBack.value);

    teleOpDriveSide = -1;

    // Configure the button bindings
    configureButtonBindings();

    drive.setDefaultCommand(new DefaultDrive(drive,
        () -> driver.getRawAxis(1),
        () -> driver.getRawAxis(4)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    opBumpLeft.whenPressed(new InstantCommand(indexer::collect, indexer));
    opBumpLeft.whenReleased(new InstantCommand(indexer::stop, indexer));
    opBumpRight.whenPressed(new InstantCommand(indexer::dispense, indexer));
    opBumpRight.whenReleased(new InstantCommand(indexer::stop, indexer));
    
    opX.whenPressed(new InstantCommand(indexer::startVIndexer, indexer));
    opX.whenReleased(new InstantCommand(indexer::stopVIndexer, indexer));

    opY.whenPressed(new InstantCommand(() -> shooter.setSpeed(1), shooter));
    opY.whenReleased(new InstantCommand(() -> shooter.setSpeed(0), shooter));

    opA.whenPressed(new InstantCommand(indexer::outsideCollect, indexer));
    opA.whenReleased(new InstantCommand(indexer::outsideStop, indexer));

    opB.whenPressed(new InstantCommand(indexer::outsideDispense, indexer));
    opB.whenReleased(new InstantCommand(indexer::outsideStop, indexer));

    opStart.whenPressed(new InstantCommand(climber::climb, climber));
    opStart.whenReleased(new InstantCommand(climber::stop, climber));

    opBack.whenPressed(new InstantCommand(climber::hookEnable, climber));
    opBack.whenReleased(new InstantCommand(climber::hookDisable, climber));
  }

  public void updateDashboard() {
    drive.updateDashboard();
    colorSensor.updateShuffleboard();
    indexer.updateDashboard();
  }

  public XboxController getDriveStick() {
    return driver;
  }

  public XboxController getOpStick() {
    return operator;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    throw new UnsupportedOperationException("No auto routine");
  }
}
