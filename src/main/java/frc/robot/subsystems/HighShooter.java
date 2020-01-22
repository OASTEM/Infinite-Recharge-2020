/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HighShooter extends SubsystemBase {
  /**
   * Creates a new HighShooter.
   */

  private TalonSRX motor1;
  private TalonSRX motor2;

  public HighShooter() {
    motor1 = new TalonSRX(20);
    motor2 = new TalonSRX(21);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double power) {
    motor1.set(ControlMode.PercentOutput, power);
    motor2.set(ControlMode.PercentOutput, power);
  }

  public void stop() {
    motor1.set(ControlMode.PercentOutput, 0);
    motor2.set(ControlMode.PercentOutput, 0);
  }
}
