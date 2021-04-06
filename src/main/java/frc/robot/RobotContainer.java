/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandGroups.AutoCommandGroup;
import frc.robot.commands.AlignWithFlashlight;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveForEncoder;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.RunDrivePID;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnWithLimelight;
import frc.robot.subsystems.ChameleonVision;
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
    private LimelightVision limelightVision;
    private ChameleonVision chameleonVision;

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

    private SendableChooser<Supplier<Command>> chooseAutoCommand = new SendableChooser<>();
    private SendableChooser<Trajectory> trajectoryChooser = new SendableChooser<>();

    private AutoCommandGroup autoCommandGroup;
    private SequentialCommandGroup bouncePathCommandGroup;

    private RunDrivePID runDrivePID;
    private MotionProfileCommand profileCommand;
    private TurnWithLimelight turnWithLimelight;
    private Shoot shoot;
    private AlignWithFlashlight alignRight;
    private AlignWithFlashlight alignLeft;

    private Trajectory slalomTrajectory;
    private Trajectory bounceTrajectory1;
    private Trajectory bounceTrajectory2;
    private Trajectory bounceTrajectory3;
    private Trajectory bounceTrajectory4;
    private Trajectory barrelTrajectory;
    private Trajectory redGalacticTrajectoryA;
    private Trajectory redGalacticTrajectoryB;
    private Trajectory blueGalacticTrajectoryA;
    private Trajectory blueGalacticTrajectoryB;

    private double dxy;

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
    limelightVision = new LimelightVision();
    chameleonVision = new ChameleonVision();

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
    autoCommandGroup = new AutoCommandGroup(drive, limelightVision, shooter, indexer, this);
    runDrivePID = new RunDrivePID(drive);
    profileCommand = new MotionProfileCommand(240, drive);
    turnWithLimelight = new TurnWithLimelight(drive, limelightVision);
    shoot = new Shoot(shooter, indexer, limelightVision, false);
    alignRight = new AlignWithFlashlight(0.5, drive);
    alignLeft = new AlignWithFlashlight(-0.5, drive);

    teleOpDriveSide = -1;

    dxy = Units.inchesToMeters(30);

    configureButtonBindings();
    generateTrajectories();
    //LiveWindow.disableAllTelemetry();

    bouncePathCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> drive.resetOdometry(bounceTrajectory1.getInitialPose())),
      generateRamseteCommand(bounceTrajectory1),
      new InstantCommand(() -> drive.resetOdometry(bounceTrajectory2.getInitialPose())),
      generateRamseteCommand(bounceTrajectory2),
      new InstantCommand(() -> drive.resetOdometry(bounceTrajectory3.getInitialPose())),
      generateRamseteCommand(bounceTrajectory3),
      new InstantCommand(() -> drive.resetOdometry(bounceTrajectory4.getInitialPose())),
      generateRamseteCommand(bounceTrajectory4)
    );

    trajectoryChooser.setDefaultOption("Slalom", slalomTrajectory);
    trajectoryChooser.addOption("Bounce 1", bounceTrajectory1);
    trajectoryChooser.addOption("Bounce 2", bounceTrajectory2);
    trajectoryChooser.addOption("Bounce 3", bounceTrajectory3);
    trajectoryChooser.addOption("Bounce 4", bounceTrajectory4);
    trajectoryChooser.addOption("Barrel", barrelTrajectory);
    trajectoryChooser.addOption("Red Galactic A", redGalacticTrajectoryA);
    trajectoryChooser.addOption("Red Galactic B", redGalacticTrajectoryB);
    trajectoryChooser.addOption("Blue Galactic A", blueGalacticTrajectoryA);
    trajectoryChooser.addOption("Blue Galactic B", blueGalacticTrajectoryB);

    chooseAutoCommand.setDefaultOption("Auto Command Group", () -> autoCommandGroup);
    chooseAutoCommand.addOption("Drive off Auto Line", () -> new DriveForEncoder(drive, 0.6, 1, 25));
    chooseAutoCommand.addOption("Limelight Vision Command", () -> new TurnWithLimelight(drive, limelightVision));
    chooseAutoCommand.addOption("Ramsete Command From Trajectory", () -> generateRamseteCommand(trajectoryChooser.getSelected()));
    chooseAutoCommand.addOption("Bounce Path Command Group", () -> bouncePathCommandGroup);

    SmartDashboard.putData("Trajectory Selector", trajectoryChooser);
    SmartDashboard.putData("Auto Selector", chooseAutoCommand);

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

  private void generateTrajectories() {
    slalomTrajectory = trajectoryForPath(
      List.of(new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(2.25*dxy, -1.5*dxy, new Rotation2d(-Math.PI / 6)),
        new Pose2d(4*dxy, -2*dxy, new Rotation2d()),
        new Pose2d(6.25*dxy, -2*dxy, new Rotation2d()),
        new Pose2d(8*dxy, 0, new Rotation2d(Math.PI / 6)),
        new Pose2d(10*dxy, -0.5*dxy, new Rotation2d(-Math.PI / 2)),
        new Pose2d(8.5*dxy, -1.8*dxy, new Rotation2d(-Math.PI)),
        new Pose2d(7.5*dxy, -dxy, new Rotation2d((-4*Math.PI) / 3)),
        new Pose2d(5*dxy, 0, new Rotation2d(Math.PI)),
        new Pose2d(2.75*dxy, 0, new Rotation2d(Math.PI)),
        new Pose2d(1.5*dxy, -1.5*dxy, new Rotation2d((5*Math.PI) / 4)),
        new Pose2d(-dxy, -2.25*dxy, new Rotation2d(Math.PI))
      ), 
      false);
    
    bounceTrajectory1 = trajectoryForPath(
      List.of(new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(1.25*dxy, -2*dxy, new Rotation2d(-Math.PI / 2))
      ), 
      false);
    
    bounceTrajectory2 = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(1.75*dxy, -0.5*dxy, new Rotation2d(5*Math.PI / 6)),
        new Pose2d(3.8*dxy, -1.75*dxy, new Rotation2d(Math.PI / 2)),
        new Pose2d(0, -3*dxy, new Rotation2d())
      ), 
      true);

    bounceTrajectory3 = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(2.5*dxy, 0, new Rotation2d()),
        new Pose2d(3.5*dxy, -dxy, new Rotation2d(-Math.PI / 2)),
        new Pose2d(3*dxy, -2.75*dxy, new Rotation2d(Math.PI)),
        new Pose2d(0, -2.75*dxy, new Rotation2d(Math.PI))
      ), 
      false);

    bounceTrajectory4 = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(1.25*dxy, -2*dxy, new Rotation2d(Math.PI / 2))
      ), 
      true);

    barrelTrajectory = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d()),
        new Pose2d(3*dxy, 0, new Rotation2d()),
        new Pose2d(4.25*dxy, dxy, new Rotation2d(Math.PI / 2)),
        new Pose2d(3.25*dxy, 1.5*dxy, new Rotation2d(Math.PI)),
        new Pose2d(2.75*dxy, dxy, new Rotation2d((3*Math.PI) / 2)),
        new Pose2d(6.5*dxy, 0, new Rotation2d(-Math.PI / 6)),
        new Pose2d(7*dxy, -dxy, new Rotation2d(-Math.PI / 2)),
        new Pose2d(6.5*dxy, -1.75*dxy, new Rotation2d(Math.PI)),
        new Pose2d(5.75*dxy, -0.75*dxy, new Rotation2d((-5*Math.PI) / 3)),
        new Pose2d(9*dxy, dxy, new Rotation2d(-Math.PI / 6)),
        new Pose2d(9*dxy, 0, new Rotation2d(Math.PI)),
        new Pose2d(-0.5*dxy, -0.25*dxy, new Rotation2d(Math.PI))
      ), 
      false);

    redGalacticTrajectoryA = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(3*dxy, 0, new Rotation2d(7*Math.PI / 6)),
        new Pose2d(5*dxy, 0.5*dxy, new Rotation2d((-5*Math.PI) / 4)),
        new Pose2d(6*dxy, -2*dxy, new Rotation2d(Math.PI)),
        new Pose2d(11*dxy, 0, new Rotation2d(Math.PI))
      ), 
      true);

    redGalacticTrajectoryB = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(3*dxy, -0.75*dxy, new Rotation2d((7*Math.PI) / 6)),
        new Pose2d(5*dxy, 0.5*dxy, new Rotation2d((-7*Math.PI) / 6)),
        new Pose2d(7*dxy, -dxy, new Rotation2d((7*Math.PI) / 6)),
        new Pose2d(11*dxy, 0, new Rotation2d(Math.PI))
      ), 
      true);

    blueGalacticTrajectoryA = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(3*dxy, 0, new Rotation2d(Math.PI)),
        new Pose2d(6*dxy, 1.5*dxy, new Rotation2d((-7*Math.PI) / 6)),
        new Pose2d(7*dxy, -dxy, new Rotation2d((7*Math.PI) / 6)),
        new Pose2d(9*dxy, -0.5*dxy, new Rotation2d(Math.PI)),
        new Pose2d(11*dxy, -0.5*dxy, new Rotation2d(Math.PI))
      ), 
      true);

    blueGalacticTrajectoryB = trajectoryForPath(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(3*dxy, 0, new Rotation2d(Math.PI)),
        new Pose2d(6*dxy, 0.5*dxy, new Rotation2d((-7*Math.PI) / 6)),
        new Pose2d(8*dxy, -dxy, new Rotation2d((7*Math.PI) / 6)),
        new Pose2d(10*dxy, dxy, new Rotation2d(Math.PI)),
        new Pose2d(11*dxy, dxy, new Rotation2d(Math.PI))
      ), 
      true);
  }

  public void updateDashboard() {
    drive.updateDashboard();
    //colorSensor.updateDashboard();
    indexer.updateDashboard();
    shooter.updateDashboard();
    chameleonVision.updateDashboard();
  }

  private Trajectory trajectoryForPath(List<Pose2d> path, boolean reversed) {
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(drive.getKinematics())
            .setReversed(reversed);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      path,
      config);

      return trajectory;
  }

  public Command generateRamseteCommand(Trajectory trajectory) {
    drive.plotTrajectory(trajectory);
    
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      drive::getPose, 
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
      drive.getFeedforward(), 
      drive.getKinematics(),
      drive::getWheelSpeeds, 
      drive.getLeftRamsetePIDController(), 
      drive.getRightRamsetePIDController(), 
      drive::tankDriveVolts, 
      drive
      );

  drive.resetOdometry(trajectory.getInitialPose());
  return ramseteCommand.andThen(() -> drive.setOutput(0, 0));
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
    return chooseAutoCommand.getSelected().get();
  }
}
