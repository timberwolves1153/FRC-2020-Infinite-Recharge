/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commandGroups.AutoCommandGroup;
import frc.robot.commands.AlignWithFlashlight;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveForEncoder;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.RunDrivePID;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnWithLimelight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPosition;

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
    //private POVButton opPOVUp;
    //private POVButton opPOVDown;

    private JoystickButton drStart;
    private JoystickButton drX;
    private JoystickButton drBumpLeft;
    private JoystickButton drBumpRight;

    private SendableChooser<Command> chooseAutoCommand = new SendableChooser<>();
    private AutoCommandGroup autoCommandGroup;

    private RunDrivePID runDrivePID;
    private MotionProfileCommand profileCommand;
    private TurnWithLimelight turnWithLimelight;
    private Shoot shoot;
    private AlignWithFlashlight alignRight;
    private AlignWithFlashlight alignLeft;

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

    drStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    drX = new JoystickButton(driver, XboxController.Button.kX.value);
    drBumpLeft = new JoystickButton(driver, XboxController.Button.kBumperLeft.value);
    drBumpRight = new JoystickButton(driver, XboxController.Button.kBumperRight.value);

    //Initialize commands that require repetition
    autoCommandGroup = new AutoCommandGroup(drive, vision, shooter, indexer, this);
    runDrivePID = new RunDrivePID(drive);
    profileCommand = new MotionProfileCommand(240, drive);
    turnWithLimelight = new TurnWithLimelight(drive, vision);
    shoot = new Shoot(shooter, indexer, vision, false);
    alignRight = new AlignWithFlashlight(0.5, drive);
    alignLeft = new AlignWithFlashlight(-0.5, drive);

    teleOpDriveSide = -1;

    chooseAutoCommand.setDefaultOption("Auto Command Group", autoCommandGroup);
    chooseAutoCommand.addOption("Drive off Auto Line", new DriveForEncoder(drive, 0.6, 1, 25));
    chooseAutoCommand.addOption("Limelight Vision Command", new TurnWithLimelight(drive, vision));
    chooseAutoCommand.addOption("Ramsete Command", generateRamseteCommand());
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
    opB.whenPressed(new InstantCommand(() -> shooter.setFeederSpeed(-1), shooter));
    opB.whenReleased(new InstantCommand(indexer::stopVIndexer, indexer));
    opB.whenReleased(new InstantCommand(indexer::stop, indexer));
    opB.whenReleased(new InstantCommand(() -> shooter.setFeederSpeed(0), shooter));
    
    // Auto: 0.67, Close CP: 0.74, Downtown: 0.82
    /*opY.whenPressed(new InstantCommand(() -> shooter.setSpeed(.82), shooter));
    opY.whenReleased(new InstantCommand(() -> shooter.setSpeed(0), shooter));*/
    
    //opY.whenPressed(shoot);
    //opY.whenReleased(() -> CommandScheduler.getInstance().cancel(shoot));

    opY.whenPressed(new InstantCommand(shooter::pidOn, shooter));
    opY.whenReleased(new InstantCommand(shooter::pidOff, shooter));
    
    opA.whenPressed(new InstantCommand(indexer::startVIndexer, indexer));
    opA.whenReleased(new InstantCommand(indexer::stopVIndexer, indexer));
    
    opStart.whenPressed(new InstantCommand(climber::retract, climber));
    opStart.whenReleased(new InstantCommand(climber::stop, climber));

    opBack.whenPressed(new InstantCommand(climber::climb, climber));
    opBack.whenReleased(new InstantCommand(climber::stop, climber));

    opX.whenPressed(new InstantCommand(climber::toggle, climber));
    //opX.whenReleased(new InstantCommand(climber::armDown, climber));

    /*opLeftJoystickButton.whenPressed(() -> shooter.cycleGainPreset(Direction.kForwards), shooter);
    opRightJoystickButton.whenPressed(() -> shooter.cycleGainPreset(Direction.kBackwards), shooter);*/
    opLeftJoystickButton.whenPressed(() -> shooter.setGainPreset(ShooterPosition.AUTO_LINE));
    opRightJoystickButton.whenPressed(() -> shooter.setGainPreset(ShooterPosition.CR_CLOSE));

    drStart.whenPressed(turnWithLimelight);
    drStart.whenReleased(() -> CommandScheduler.getInstance().cancel(turnWithLimelight));

    /*drX.whenPressed(runDrivePID);
    drX.whenReleased(() -> CommandScheduler.getInstance().cancel(runDrivePID));*/
    drX.whenPressed(profileCommand);
    drX.whenReleased(() -> profileCommand.cancel());

    drBumpLeft.whenPressed(new InstantCommand(drive::lightOn, drive));
    drBumpLeft.whenReleased(new InstantCommand(drive::lightOff, drive));
  }

  public void updateDashboard() {
    drive.updateDashboard();
    //colorSensor.updateDashboard();
    indexer.updateDashboard();
    shooter.updateDashboard();
  }

  /**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were 
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts, 
                                       Constants.kvVoltSecondsPerMeter, 
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        /*new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(0.5, 0.5),
          new Translation2d(1.0, 0.5),
          new Translation2d(1.5, 0),
          new Translation2d(1.0, -0.5),
          new Translation2d(0.5, -0.5)
        ),
        new Pose2d(0, 0, new Rotation2d(Math.PI)),*/
        /*List.of(
          new Pose2d(0, 0, new Rotation2d(0)),
          new Pose2d(0.75, 0, new Rotation2d(Math.PI / 2))
          ),*/
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
            new Translation2d(0.25, 0.25)
          ),
          new Pose2d(0, 0, new Rotation2d(0)),
        config);
      /*String trajectoryJSON = "paths/small.wpilib.json";
      Trajectory tempTrajectory = new Trajectory();
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      try {
        tempTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
      Transform2d transform = m_drivetrain.getPose().minus(tempTrajectory.getInitialPose());
      Trajectory exampleTrajectory = tempTrajectory.transformBy(transform);*/

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        drive::tankDriveVolts,
        drive);

    drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> drive.resetOdometry(exampleTrajectory.getInitialPose()), drive)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> drive.tankDriveVolts(0, 0), drive));
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
    //return chooseAutoCommand.getSelected();
    return generateRamseteCommand();
  }
}
