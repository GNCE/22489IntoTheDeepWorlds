package pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants_0_7 {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;


        FollowerConstants.mass = 15.5;

        FollowerConstants.xMovement = 80;
        FollowerConstants.yMovement = 55.00711797139106;

        FollowerConstants.forwardZeroPowerAcceleration = -31.920673072184087;
        FollowerConstants.lateralZeroPowerAcceleration = -70.91520272340031;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(2.2,0,0.267,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.38,0,0.025,0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(3,0,0.25,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.headingPIDFSwitch = 0.075;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1.2,0,0.15,0);

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.04,0,0.0005,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.026,0,0.00045,0.6,0);

        FollowerConstants.zeroPowerAccelerationMultiplier = 1;
        FollowerConstants.centripetalScaling = 0.0003;

        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.98;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.25;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.nominalVoltage = 12.54;

        FollowerConstants.useVoltageCompensationInAuto = false;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
