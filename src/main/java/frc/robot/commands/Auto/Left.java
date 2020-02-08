/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BringClimberUp;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.GamepadDrive;
import frc.robot.commands.GoToGoalColor;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Left extends SequentialCommandGroup {
  /**
   * Creates a new Left.
   */
  public Left() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DriveDistance(1000), new TurnAngle(90), new Shoot());
  }
}
