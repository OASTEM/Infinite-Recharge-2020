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

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LowShooter extends SubsystemBase {
  /**
   * Creates a new LowShooter.
   */
  private TalonSRX motor1;
  private TalonSRX motor2; 
  private Spark motor3; 
  private Spark motor4;

  public LowShooter() {
    motor1 = new TalonSRX(11);
    motor2 = new TalonSRX(12);
    motor3 = new Spark(13);
    motor4 = new Spark(14);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double power) {
    motor1.set(ControlMode.PercentOutput, power);
    motor2.set(ControlMode.PercentOutput, power);
    motor3.set(power);
    motor4.set(power);
  }

  public void stop() {
    motor1.set(ControlMode.PercentOutput, 0);
    motor2.set(ControlMode.PercentOutput, 0);
    motor3.set(0);
    motor4.set(0);
  }

  
}
