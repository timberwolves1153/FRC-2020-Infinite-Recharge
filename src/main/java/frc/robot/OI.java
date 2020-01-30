/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {

    // Joystick Positions
    private static final int DRIVER_JOYSTICK = 0;
    private static final int OPERATOR_STICK = 1;

    // Axis IDs
    public static final int JOYSTICK_LEFT_Y = 1;
	public static final int JOYSTICK_RIGHT_X = 4;
	public static final int JOYSTICK_RIGHT_Y = 5;
	public static final int JOYSTICK_TRIGGER_LEFT = 2;
	public static final int JOYSTICK_TRIGGER_RIGHT = 3;
    
    private Joystick opStick = new Joystick(OPERATOR_STICK);
    private Joystick driverStick = new Joystick(DRIVER_JOYSTICK);

    private JoystickButton drA = new JoystickButton(driverStick, 1);
    private JoystickButton drB = new JoystickButton(driverStick, 2);
    public JoystickButton drX = new JoystickButton(driverStick, 3);
    public JoystickButton drY = new JoystickButton(driverStick, 4);

    public OI() {
        // Register all button-command associations here
        drA.whenActive(new InstantCommand(Robot.auxMotor::fullForward)); // Per docs, whenActive() is functionally idential to whenPressed()
        drA.whenInactive(new InstantCommand(Robot.auxMotor::stop)); // whenInactive() is functionally idential to whenReleased()
        drB.whenActive(new InstantCommand(Robot.auxMotor::fullBackward));
        drB.whenInactive(new InstantCommand(Robot.auxMotor::stop));
        
        /*drX.whenActive(new VIndexStartCommand(1));
        drX.whenInactive(new VIndexStopCommand());
        drY.whenActive(new VIndexStartCommand(0.5));
        drY.whenInactive(new VIndexStopCommand());*/
    }

    public Joystick getOpStick() {
        return opStick;
    }

    public Joystick getDriverStick() {
        return driverStick;
    }
}
