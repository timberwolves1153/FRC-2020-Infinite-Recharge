// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChameleonVision extends SubsystemBase {
  private NetworkTable table;

  private Target target;

  private Supplier<Double> tx;
  private Supplier<Double> ty;
  private Supplier<Double> ta;
  private Supplier<Double> tBoundingWidth;
  private Supplier<Double> tBoundingHeight;

  public ChameleonVision() {
    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("Microsoft Webcam");

    tx = () -> table.getEntry("targetYaw").getDouble(0.0);
    ty = () -> table.getEntry("targetPitch").getDouble(0.0);
    ta = () -> table.getEntry("targetArea").getDouble(0.0);
    tBoundingWidth = () -> table.getEntry("targetBoundingWidth").getDouble(0.0);
    tBoundingHeight = () -> table.getEntry("targetBoundingHeight").getDouble(0.0);

    target = new Target(tx.get(), ty.get(), ta.get(), tBoundingWidth.get(), tBoundingHeight.get());
  }

  public void updateChameleonData() {
    boolean targetExists = table.getEntry("isValid").getBoolean(false);

    if(!targetExists) {
      target.updateValues(0.0, 0.0, 0.0, 0.0, 0.0);
      return;
    }

    target.updateValues(tx.get(), ty.get(), ta.get(), tBoundingWidth.get(), tBoundingHeight.get());
  }

  public void updateDashboard() {
    updateChameleonData();

    SmartDashboard.putBoolean("Has Target", table.getEntry("isValid").getBoolean(false));

    SmartDashboard.putNumber("Target X", target.x);
    SmartDashboard.putNumber("Target Y", target.y);
    SmartDashboard.putNumber("Target A", target.a);
    SmartDashboard.putNumber("Target Bounding Area", target.boundingArea);
  }

  public void flushNetworkTables() {
    NetworkTableInstance.getDefault().flush();
  }

  public void setDriverMode(boolean isDriverMode) {
    table.getEntry("driverMode").setBoolean(isDriverMode);
  }

  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  @Override
  public void periodic() {}

  public class Target {
    public double x;
    public double y;
    public double a;
    public double boundingArea;

    /**
     * Creates a new Target object representing a visible target in Chameleon Vision
     * @param x - The value of the horizontal delta from the center of the viewport(the yaw)
     * @param y - The value of the vertical delta from the center o the viewpower(the pitch)
     * @param a - The area of the target from the NetworkTables
     * @param boundingWidth - The width of the red square of the selected target
     * @param boundingHeight - The height of the red square of the selected target
     */
    public Target(double x, double y, double a, double boundingWidth, double boundingHeight) {
      updateValues(x, y, a, boundingWidth, boundingHeight);
    }

    /**
     * Updates the values of the Target object
     * @param x - The value of the horizontal delta from the center of the viewport(the yaw)
     * @param y - The value of the vertical delta from the center o the viewpower(the pitch)
     * @param a - The area of the target from the NetworkTables
     * @param boundingWidth - The width of the red square of the selected target
     * @param boundingHeight - The height of the red square of the selected target
     */
    public void updateValues(double x, double y, double a, double boundingWidth, double boundingHeight) {
      this.x = x;
      this.y = y;
      this.a = a;
      boundingArea = boundingWidth * boundingHeight;
    }
  }
}
