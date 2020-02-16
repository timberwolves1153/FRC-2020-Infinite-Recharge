/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandGroups.AutoCommandGroup;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveForEncoder;
import frc.robot.commands.TurnWithLimelight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightVision;
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
    private LimelightVision vision;

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
    private JoystickButton opLeftJoystickButton;
    private JoystickButton opRightJoystickButton;

    private SendableChooser<Command> chooseAutoCommand = new SendableChooser<>();
    private AutoCommandGroup autoCommandGroup;

    public int teleOpDriveSide;

    public double lastLimelightTurnAngleDifference = 0;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = new Drive();
    shooter = new Shooter();
    indexer = new Indexer();
    colorSensor = new ColorSensor();
    climber = new Climber();
    vision = new LimelightVision();

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
    opLeftJoystickButton = new JoystickButton(operator, XboxController.Button.kStickLeft.value);
    opRightJoystickButton = new JoystickButton(operator, XboxController.Button.kStickRight.value);

    autoCommandGroup = new AutoCommandGroup(drive, vision, shooter, indexer, this);

    teleOpDriveSide = -1;

    chooseAutoCommand.setDefaultOption("Drive off Auto Line", new DriveForEncoder(drive, 0.6, -1, -60));
    chooseAutoCommand.addOption("Limelight Vision Command", new TurnWithLimelight(drive, vision));
    chooseAutoCommand.addOption("Auto Command Group", autoCommandGroup);
    SmartDashboard.putData("Auto Selector", chooseAutoCommand);

    configureButtonBindings();

    //LiveWindow.disableAllTelemetry();

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
    
    opB.whenPressed(new InstantCommand(indexer::startVIndexer, indexer));
    opB.whenPressed(new InstantCommand(indexer::kick, indexer));
    opB.whenPressed(new InstantCommand(() -> shooter.setAcceleratorSpeed(-1), shooter));
    opB.whenReleased(new InstantCommand(indexer::stopVIndexer, indexer));
    opB.whenReleased(new InstantCommand(indexer::stop, indexer));
    opB.whenReleased(new InstantCommand(() -> shooter.setAcceleratorSpeed(0), shooter));
    
    // Auto: 0.67, Close CP: 0.74, Downtown: 0.82
    /*opY.whenPressed(new InstantCommand(() -> shooter.setSpeed(.82), shooter));
    opY.whenReleased(new InstantCommand(() -> shooter.setSpeed(0), shooter));*/
    
    opY.whenPressed(shooter::pidOn, shooter);
    opY.whenReleased(shooter::pidOff, shooter);
    
    opA.whenPressed(new InstantCommand(indexer::startVIndexer, indexer));
    opA.whenReleased(new InstantCommand(indexer::stopVIndexer, indexer));

    /*
    opStart.whenPressed(new InstantCommand(() -> shooter.setAcceleratorSpeed(-1), shooter));
    opStart.whenReleased(new InstantCommand(() -> shooter.setAcceleratorSpeed(0), shooter));
    */
    opStart.whenPressed(new InstantCommand(climber::retract, climber));
    opStart.whenReleased(new InstantCommand(climber::stop, climber));

    opBack.whenPressed(new InstantCommand(climber::climb, climber));
    opBack.whenReleased(new InstantCommand(climber::stop, climber));

    opLeftJoystickButton.whenPressed(new InstantCommand(climber::hookEnable, climber));
    opLeftJoystickButton.whenReleased(new InstantCommand(climber::hookDisable, climber));

    opX.whenPressed(new InstantCommand(climber::armUp, climber));
    opX.whenReleased(new InstantCommand(climber::armDown, climber));

    opRightJoystickButton.whenPressed(new InstantCommand(climber::hookRetract, climber));
    opRightJoystickButton.whenReleased(new InstantCommand(climber::hookDisable, climber));


    
    //opBack.whenHeld(new TurnWithLimelight(drive, vision));
    
  }

  public void updateDashboard() {
    drive.updateDashboard();
    //colorSensor.updateDashboard();
    indexer.updateDashboard();
    shooter.updateDashboard();
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
    return chooseAutoCommand.getSelected();
  }
}
