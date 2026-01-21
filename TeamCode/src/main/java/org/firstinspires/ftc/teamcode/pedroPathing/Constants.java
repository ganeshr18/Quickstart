package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.3)
            .forwardZeroPowerAcceleration(-54.10327003325917)
            .lateralZeroPowerAcceleration(-114.13318332577153)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.01, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.000001,0.6,0.0))
            .centripetalScaling(0.005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("front_left_drive")
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.21365043145533)
            .yVelocity(64.20739257044593);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("front_left_drive")
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.006233887325408998)
            .strafeTicksToInches(0.006500178560342686)
            .turnTicksToInches(0.0166)

            .robotWidth(17.87)
            .robotLength(17);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .build();
    }
}
