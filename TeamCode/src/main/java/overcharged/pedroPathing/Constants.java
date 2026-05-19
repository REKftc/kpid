package overcharged.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.00)
            .forwardZeroPowerAcceleration(-31.366051036171647)
            .lateralZeroPowerAcceleration(-73.44266717215542)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.00036)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.35, 0.07, 0.3, 0.1))
            .headingPIDFCoefficients(new PIDFCoefficients(1.65, 0.003, 0.09, 0.09))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0.000, 0.0001, 0.6, 0))
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.2, 0, 0.02, 0.025)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients( 0.8, 0.00001, 0.01, 0.01))
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.005, 0, 0, 0.6, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorName("driveLF")
            .leftRearMotorName("driveLB")
            .rightFrontMotorName("driveRF")
            .rightRearMotorName("driveRB")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.47396057609498)
            .yVelocity(56.537629375307574);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(91.034/25.4)
            .strafePodX(-69.26/25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            50,
            0.8,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}