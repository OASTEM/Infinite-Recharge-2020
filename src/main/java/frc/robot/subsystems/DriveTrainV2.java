/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainV2 extends SubsystemBase {
  /**
   * Creates a new DriveTrainV2.
   */

  private CANSparkMax frontLeft = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax backLeft = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax backRight = new CANSparkMax(4, MotorType.kBrushless);

  private CANEncoder leftEncoder = backLeft.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
  private CANEncoder rightEncoder = backRight.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);

  public DriveTrainV2() {
    frontLeft.follow(backLeft);
    frontRight.follow(backRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivePercentOutput(0.3, 0.3);
  }

  public void drivePercentOutput(double leftPower, double rightPower) {
    backLeft.set(leftPower);
    backRight.set(rightPower);
  }
}
