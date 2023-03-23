// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private static final HardwareConstants CLONE_HARDWARE_CONSTANTS = new CloneHardwareConstants();
    private static final HardwareConstants ROBOT_HARDWARE_CONSTANTS = new RobotHardwareConstants();

    public static HardwareConstants currentHardwareConstants() {
        return ROBOT_HARDWARE_CONSTANTS;
    }

    public interface HardwareConstants {
        int getLeftLeaderDeviceId();
        int getLeftFollowerDeviceId();
        int getRightLeaderDeviceId();
        int getRightFollowerDeviceId();
        int getForeArmLeaderId();
        int getForeArmFollowerId();

        int getShifterSolenoidChannelId();
        int getLockElbowSolenoidChannel();
        int getVerticalArmStage1SolenoidChannel();
        int getVerticalArmStage2SolenoidChannel();
        int getKickstandSolenoidChannel();
        int getGripperSolenoidChannel();

        int getArmEncoderChannel1();
        int getArmEncoderChannel2();
        int getArmLimitSwitchDigitalInput();

        int getSpareTalonDeviceNumber();

        PneumaticsModuleType getPneumaticsModuleType();

        double getDriveSlewRate();
        double getReverseEncoder();
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kHelperControllerPort = 1;

        public static final double kDeadband = 0.15;
    }

    public static class CloneHardwareConstants implements HardwareConstants {

        @Override
        public int getLeftLeaderDeviceId() {
            return 41;
        }

        @Override
        public int getLeftFollowerDeviceId() {
            return 31;
        }

        @Override
        public int getRightLeaderDeviceId() {
            return 21;
        }

        @Override
        public int getRightFollowerDeviceId() {
            return 29;
        }

        @Override
        public int getShifterSolenoidChannelId() {
            return 3;
        }

        @Override
        public int getForeArmLeaderId() {
            return 42;
        }

        @Override
        public int getForeArmFollowerId() {
            return 43;
        }

        @Override
        public int getLockElbowSolenoidChannel() {
            return 0;
        }

        @Override
        public int getVerticalArmStage1SolenoidChannel() {
            return 1;
        }

        @Override
        public int getVerticalArmStage2SolenoidChannel() {
            return 2;
        }

        @Override
        public int getKickstandSolenoidChannel() {
            return 7;
        }

        @Override
        public int getGripperSolenoidChannel() {
            return 6;
        }

        @Override
        public int getSpareTalonDeviceNumber() {
            return 9;
        }

        @Override
        public int getArmEncoderChannel1() {
            return 0;
        }

        @Override
        public int getArmEncoderChannel2() {
            return 1;
        }

        @Override
        public int getArmLimitSwitchDigitalInput() {
            return 7;
        }

        @Override 
        public PneumaticsModuleType getPneumaticsModuleType() {
            return PneumaticsModuleType.CTREPCM;
        }

        @Override
        public double getDriveSlewRate() {
            return 2.5;
        }

        @Override
        public double getReverseEncoder() {
            return 1.0;
        }

    }

    public static class RobotHardwareConstants implements HardwareConstants {

        @Override
        public int getLeftLeaderDeviceId() {
            return 23;
        }

        @Override
        public int getLeftFollowerDeviceId() {
            return 24;
        }

        @Override
        public int getRightLeaderDeviceId() {
            return 27;
        }

        @Override
        public int getRightFollowerDeviceId() {
            return 28;
        }

        @Override
        public int getShifterSolenoidChannelId() {
            return 3;
        }

        @Override
        public int getForeArmLeaderId() {
            return 21;
        }

        @Override
        public int getForeArmFollowerId() {
            return 22;
        }

        @Override
        public int getLockElbowSolenoidChannel() {
            return 0;
        }

        @Override
        public int getVerticalArmStage1SolenoidChannel() {
            return 1;
        }

        @Override
        public int getVerticalArmStage2SolenoidChannel() {
            return 2;
        }

        @Override
        public int getKickstandSolenoidChannel() {
            return 7;
        }

        @Override
        public int getGripperSolenoidChannel() {
            return 6;
        }

        @Override
        public int getSpareTalonDeviceNumber() {
            return 19;
        }

        @Override
        public int getArmEncoderChannel1() {
            return 0;
        }

        @Override
        public int getArmEncoderChannel2() {
            return 1;
        }

        @Override
        public int getArmLimitSwitchDigitalInput() {
            return 9;
        }

        @Override 
        public PneumaticsModuleType getPneumaticsModuleType() {
            return PneumaticsModuleType.REVPH;
        }

        @Override
        public double getDriveSlewRate() {
            return 1.5;
        }

        @Override
        public double getReverseEncoder() {
            return -1.0;
        }

    }

    public static class DriveTrainConstants {

        public static final double kTurnP = 0.025;
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0;
        public static final double kTurnFF = 0.2;
        public static final double kTurnToleranceDegrees = 5;
        public static final double kTurnRateToleranceDegreesPerSec = 10; // degrees per second
        // Constants obtained from SysId
        public static final double ksLow = 0.14344;
        public static final double kvLow = 1.4602;
        public static final double kaLow = 0.77896;
        public static final double kpLow = 50.149;
        public static final double kdLow = 8.0681;
        public static final SimpleMotorFeedforward m_feedFwdLow = new SimpleMotorFeedforward(ksLow, kvLow, kaLow);
        private static final double kGearRatioLow = 10.86;
        private static final double kGearRatioHigh = 6.0;
        private static final double kWheelDiameterMeters = Units.inchesToMeters(6.0);
        public static final double kEncoderPositionConversionFactorLow = (Math.PI * kWheelDiameterMeters) / kGearRatioLow;          // in meters
        public static final double kEncoderVelocityConversionFactorLow = (Math.PI * kWheelDiameterMeters) / (kGearRatioLow * 60.0); // in meters per second
        public static final double kEncoderPositionConversionFactorHigh = (Math.PI * kWheelDiameterMeters) / kGearRatioHigh;          // in meters
        public static final double kEncoderVelocityConversionFactorHigh = (Math.PI * kWheelDiameterMeters) / (kGearRatioHigh * 60.0); // in meters per second
    }

}
