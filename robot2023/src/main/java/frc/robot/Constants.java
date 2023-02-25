// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kHelperControllerPort = 1;

    public static final double kDeadband = 0.15;
  }

  public static class DriveTrainConstants {
    public static final int kLeftLeaderDeviceId = 41;
    public static final int kLeftFollowerDeviceId = 31;

    public static final int kRightLeaderDeviceId = 21;
    public static final int kRightFollowerDeviceId = 29;

    public static final int kShifterSolenoidChannelId = 3;

    private static final double kGearRatioLow = 10.86;
    private static final double kGearRatioHigh = 6.0;
    private static final double kWheelDiameterMeters = Units.inchesToMeters(6.0);

    public static final double kEncoderPositionConversionFactorLow = (Math.PI * kWheelDiameterMeters) / kGearRatioLow;          // in meters
    public static final double kEncoderVelocityConversionFactorLow = (Math.PI * kWheelDiameterMeters) / (kGearRatioLow * 60.0); // in meters per second
    public static final double kEncoderPositionConversionFactorHigh = (Math.PI * kWheelDiameterMeters) / kGearRatioHigh;          // in meters
    public static final double kEncoderVelocityConversionFactorHigh = (Math.PI * kWheelDiameterMeters) / (kGearRatioHigh * 60.0); // in meters per second

    public static final int kSpareTalonDeviceNumber = 9;

    public static final double kTurnP = 10.0;
    public static final double kTurnI = 8.0;
    public static final double kTurnD = 0;
    public static final double kTurnToleranceDegrees = 5;
    public static final double kTurnRateToleranceDegreesPerSec = 10; // degrees per second

    // Constants obtained from SysId
    public static final double ksLow = 0.14344;
    public static final double kvLow = 1.4602;
    public static final double kaLow = 0.77896;
    public static final double kpLow = 50.149;
    public static final double kdLow = 8.0681;

    public static final SimpleMotorFeedforward m_feedFwdLow = new SimpleMotorFeedforward(ksLow, kvLow, kaLow);
  }

}
