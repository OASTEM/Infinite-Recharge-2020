/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

/**
 * Add your docs here.
 */
public class SelfTest {

    public SelfTest() {

    }

    public double getOutputCurrent(TalonSRX talonSRX) {
        return talonSRX.getStatorCurrent();
    }

    public double getOutputCurrent(CANSparkMax sparkMax) {
        return sparkMax.getOutputCurrent();
    }

    public double getEncoderCount() {
        return RobotContainer.climber.getLeftPosition();
    }
}
