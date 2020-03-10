/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LowDumper extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  VictorSPX motor1; 
  VictorSPX motor2;
  
  public LowDumper() {
    motor1 = new VictorSPX(9);
    motor2 = new VictorSPX(7);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake() {
    motor1.set(ControlMode.PercentOutput, -1);
    motor2.set(ControlMode.PercentOutput, 1);
  }

  public void outtake() {
    motor1.set(ControlMode.PercentOutput, 1);
    motor2.set(ControlMode.PercentOutput, -1);
  }

  public void stop() {
    motor1.set(ControlMode.PercentOutput, 0);
    motor2.set(ControlMode.PercentOutput, 0);
  }
}
