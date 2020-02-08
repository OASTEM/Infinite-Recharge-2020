/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  /**
   * Creates a new NavX.
   */
  private AHRS navX;

  public NavX() {
    navX = new AHRS(Port.kMXP, (byte)50);
    //refer to rio for xyz not navx
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAccelX() {
    return navX.getRawAccelX();
  }

  public double getAccelY() {
    return navX.getRawAccelY();
  }

  public double getAccelZ() {
    return navX.getRawAccelZ();
  }

  public double getGyroY() {
    return navX.getRawGyroY();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  public void reset() {
    navX.reset();
  }
}



