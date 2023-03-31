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

public class DepositConeHighAndMore extends SequentialCommandGroup {

    public DepositConeHighAndMore(DriveTrain driveTrain, GripperArm gripperArm, Limelight limelight) {
        InstantCommands instantCommands = new InstantCommands(gripperArm, driveTrain);

        addCommands(
                new HomeForearm(gripperArm),
                instantCommands.shiftHigh(),
                new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.PLACE_ELEM_TOP).withTimeout(10.0),
                instantCommands.extendKickstand(),
                Commands.waitSeconds(0.25),
                new DriveStraight(0.825, driveTrain).withTimeout(5.0),
                new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.INSERT_ELEM_TOP).withTimeout(10.0),
                instantCommands.openGripper(),
                Commands.waitSeconds(0.25),
                new DriveStraight(-0.1, driveTrain).withTimeout(5.0),
                new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.PLACE_ELEM_TOP).withTimeout(10.0),
                new DriveStraight(-0.9, driveTrain).withTimeout(5.0),
                new ParallelCommandGroup(
                    new DriveStraight(-3.0, driveTrain).withTimeout(5.0),
                    new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.TRAVEL).withTimeout(10.0)),
                instantCommands.retractKickstand(),
                new TurnAngle(180., driveTrain).withTimeout(5.0),
                new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.PICK_ELEM_FLOOR).withTimeout(5.0),
                new AutoAim(AutoAim.AUTOAIM_PICK_PIPELINE, driveTrain, limelight).withTimeout(5.0),
                instantCommands.shiftHigh(),
                instantCommands.closeGripper(),
                Commands.waitSeconds(0.5),
                new AutoPositionArm(gripperArm, AutoPositionArm.ArmPosition.TRAVEL).withTimeout(5.0),
                new TurnAngle(-180., driveTrain).withTimeout(5.0),
                new DriveStraight(2.5, driveTrain).withTimeout(5.0)
                );
    }
}
