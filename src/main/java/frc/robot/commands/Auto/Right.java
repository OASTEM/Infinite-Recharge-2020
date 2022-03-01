/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.OuttakeBalls;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.TurnAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Right extends SequentialCommandGroup {
  /**
   * Creates a new Right.
   */
  public Right() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DriveDistance(5, "B", 0.2), new TurnAngle(-23), new DriveDistance(132, "B", 0.5), new TurnAngle(15), new DriveDistance(25, "B", 0.5), new OuttakeBalls(.8, true));
  }
}
