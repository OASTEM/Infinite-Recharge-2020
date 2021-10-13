/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class ControlPanelMan extends SubsystemBase {
  /**
   * Creates a new ControlPanelMan.
   */

  private VictorSPX cpMan = new VictorSPX(13);
  // VictorSPX 13

  private I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch m_colorMatcher = new ColorMatch();

  //Our Wheel
  private final Color kBlueTarget = ColorMatch.makeColor(0.185, 0.420, 0.395); //.143,.427,.429
  private final Color kGreenTarget = ColorMatch.makeColor(0.180, 0.620, 0.200); //.197,.561,.240
  private final Color kRedTarget = ColorMatch.makeColor(0.627, 0.310, 0.063); //.561, .232, .114
  private final Color kYellowTarget = ColorMatch.makeColor(0.386, 0.516, 0.088);//.361, .524, .113

  /*//Field Wheel
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429); //.143,.427,.429
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240); //.197,.561,.240
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114); //.561, .232, .114
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);//.361, .524, .113*/

  private final String[] colorList = {"Y", "R", "G", "B"};

  private String colorString;

  public ControlPanelMan() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget); 
    colorString = "";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Color detectedColor = m_colorSensor.getColor();
    m_colorMatcher.setConfidenceThreshold(.050);
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    //m_colorMatcher.m
    /*System.out.println("Red: " + detectedColor.red);
    System.out.println("Green: " + detectedColor.green);
    System.out.println("Blue: " + detectedColor.blue);
    */
    if (match.color == kBlueTarget) {
      colorString = "B";
    } else if (match.color == kRedTarget) {
      colorString = "R";
    } else if (match.color == kGreenTarget) {
      colorString = "G";
    } else if (match.color == kYellowTarget) {
      colorString = "Y";
    } else {
      colorString = "Unknown";
    }
    //System.out.println(colorString);
  }
  
  public void run(double power) {
    cpMan.set(ControlMode.PercentOutput, power);
  }

  public String getColor() {
    int colorInt = 0;

    for (int i = 0; i < 4; i++) {
      if (colorString.equals(colorList[i])) {
        colorInt = i;
      }
    }
    return colorList[(colorInt+2)%4];
  }
}
