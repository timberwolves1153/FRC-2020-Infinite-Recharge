/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.AuxMotorBackward;
import frc.robot.commands.AuxMotorForward;
import frc.robot.commands.AuxMotorStop;

public class OI {

    private static final int DRIVER_JOYSTICK = 0;
    private static final int OPERATOR_STICK = 1;

    public static final int JOYSTICK_LEFT_Y = 1;
	public static final int JOYSTICK_RIGHT_X = 4;
	public static final int JOYSTICK_RIGHT_Y = 5;
	public static final int JOYSTICK_TRIGGER_LEFT = 2;
	public static final int JOYSTICK_TRIGGER_RIGHT = 3;
    
    private Joystick opStick = new Joystick(OPERATOR_STICK);
    private Joystick driverStick = new Joystick(DRIVER_JOYSTICK);

    private Button opA = new JoystickButton(opStick, 1);
    private Button opB = new JoystickButton(opStick, 2);

    public OI() {
        // Register all button-command associations here
        opA.whenPressed(new AuxMotorForward());
        opA.whenReleased(new AuxMotorStop());
        opB.whenPressed(new AuxMotorBackward());
        opB.whenReleased(new AuxMotorStop());
    }

    public Joystick getOpStick() {
        return opStick;
    }

    public Joystick getDriverStick() {
        return driverStick;
    }
}
