package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.drivetrain.DriveStraight;
import frc.robot.commands.autonomous.drivetrain.TurnAngle;
import frc.robot.commands.gripperarm.AutoPositionArm;
import frc.robot.commands.gripperarm.HomeForearm;
import frc.robot.limelight.Limelight;
import frc.robot.commands.InstantCommands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GripperArm;

public class DunkyCone extends SequentialCommandGroup {

    public DunkyCone(DriveTrain driveTrain, GripperArm gripperArm, Limelight limelight) {
        InstantCommands instantCommands = new InstantCommands(gripperArm, driveTrain);

        addCommands(
                new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.INSERT_ELEM_TOP).withTimeout(10.0),
                instantCommands.openGripper(),
                Commands.waitSeconds(0.25),
                new DriveStraight(-0.1, driveTrain).withTimeout(5.0),
                new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.PLACE_ELEM_TOP).withTimeout(10.0),
                new DriveStraight(-0.3, driveTrain).withTimeout(5.0)
            );
    }
}
