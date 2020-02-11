/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase {
  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;

  private Color detectedColor;
  private int IR;
  private double proximity;
  private double hue;

  public String colorName;

  /**
   * Creates a new ColorSensor.
   */
  public ColorSensor() {
    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
    updateColorData();
  }

  public void updateColorData() {
    detectedColor = colorSensor.getColor();
    IR = colorSensor.getIR();
    proximity = colorSensor.getProximity();
    hue = RGBtoHue(detectedColor.red, detectedColor.green, detectedColor.blue);
    colorName = findColorName(hue);
  }

  public Color getColor() {
    return detectedColor;
  }

  public int getIR(){
    return IR;
  }
  public double getProximity() {
    return proximity;
  }

  public double getHue() {
    return hue;
  }
  
  public String getColorName() {
    return colorName;
  }

  public double RGBtoHue(double red, double green, double blue) {
    double hue;
    double min, max, delta;
    min = Math.min(Math.min(red, green), blue);
    max = Math.max(Math.max(red, green), blue);

    delta = max - min;

    if(red == max){
      hue = (green - blue) / delta;
    } else if(green == max) {
      hue = 2 + (blue - red) / delta;
    } else {
      hue = 4 + (red - green) / delta;
    }

    hue *= 60;
    if(hue < 0){
      hue += 360;
    }
    return hue;
  }

  public String findColorName(double hue){
    String color = "";
    if(hue >= Constants.yellowHueMin && hue <= Constants.yellowHueMax) {
      color = "Yellow";
    } else if(hue >= Constants.greenHueMin && hue <= Constants.greenHueMax) {
      color = "Green";
    } else if(hue >= Constants.redHueMin && hue <= Constants.redHueMax) {
      color = "Red";
    } else if(hue >= Constants.blueHueMin && hue <= Constants.blueHueMax) {
      color = "Blue";
    } else {
      color = "";
    }
    return color;
  }

  public void updateDashboard() {
    updateColorData();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Hue", hue);
    SmartDashboard.putString("Color", colorName);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
