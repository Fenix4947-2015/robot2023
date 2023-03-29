package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.InstantCommands;
import frc.robot.commands.autonomous.drivetrain.DriveStraight;
import frc.robot.commands.gripperarm.AutoPositionArm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GripperArm;

public class DepositConeMiddle extends SequentialCommandGroup {

    public DepositConeMiddle(DriveTrain driveTrain, GripperArm gripperArm) {
        InstantCommands instantCommands = new InstantCommands(gripperArm, driveTrain);

        addCommands(
                new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.PLACE_ELEM_MID).withTimeout(10.0),
                //instantCommands.extendKickstand(),
                Commands.waitSeconds(0.5),
                new DriveStraight(1.1, driveTrain).withTimeout(5.0),
                Commands.waitSeconds(0.5),
                instantCommands.openGripper(),
                Commands.waitSeconds(0.5)
                //new DriveStraight(-1.0, driveTrain).withTimeout(5.0)
        );
    }
}
