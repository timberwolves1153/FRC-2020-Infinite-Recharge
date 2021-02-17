/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * Proprietary subclass of CANPIDController that provides convenience features
 * including the ability to tune PID values from Shuffleboard
 */
public class RebelCANPIDController extends CANPIDController implements Sendable {

    private static int instances;

    // Container for setpoint to keep track of setpoint changes
    private double setpoint;

    public RebelCANPIDController(CANSparkMax device) {
        super(device);

        instances++;
        SendableRegistry.addLW(this, "CANPIDController", instances);
    }

    /**
     * Get the established setpoint.
     * 
     * @return target setpoint
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * Set the target setpoint sent to the Spark MAX controllers via `setReference()`
     * 
     * @param setpoint
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public CANError setReference(double value, ControlType ctrl, int pidSlot,
                               double arbFeedforward) {
        if (value != setpoint) setSetpoint(value);
        return super.setReference(setpoint, ctrl, pidSlot, arbFeedforward);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CANPIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
