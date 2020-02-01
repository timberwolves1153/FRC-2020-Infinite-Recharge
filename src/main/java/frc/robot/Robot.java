/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj.command.Command;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Command autoCommand;

  public static OI oi;
  public static Drive drive;
  //public static Shooter shooter;
  public static Indexer indexer;

  public static ColorSensor colorSensor;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    drive = new Drive();
    //shooter = new Shooter();
    indexer = new Indexer();
    colorSensor = new ColorSensor();
    oi = new OI();
  }

  private void updateDashboard(){
    drive.updateDashboard();
    colorSensor.updateShuffleboard();
    indexer.updateDashboard();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }

    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
  }

  private boolean driveToggle = true;
  private boolean drXPast = false;

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    updateDashboard();

    // Check button state against previous button state
    //boolean drXCurrent = oi.drX.get();
    // TODO: Change button address, see conflict in OI.java for Shooter control
    boolean drXCurrent = false;
    if (!drXPast && drXCurrent) {
      driveToggle = !driveToggle;

      if (!driveToggle) {
        drive.pidOn();
      } else {
        drive.pidOff();
      }
      
      drXPast = drXCurrent;
    }

    if (driveToggle) {
      double power = oi.getDriverStick().getRawAxis(OI.JOYSTICK_LEFT_Y);
      double turn = oi.getDriverStick().getRawAxis(OI.JOYSTICK_RIGHT_X);
      drive.arcadeDrive(power, -turn);
    }

    /*if (oi.getDriverStick().getRawButton(5)) {
      indexer.collect();
    } else if (oi.getDriverStick().getRawButton(6)) {
      indexer.disperse();
    } else {
      indexer.stop();
    }*/
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
  }
}
