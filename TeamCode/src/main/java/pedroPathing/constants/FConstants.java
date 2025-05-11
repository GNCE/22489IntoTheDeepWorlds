package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
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

        FollowerConstants.xMovement = 73.40592699663702;
        FollowerConstants.yMovement = 55.60059530562092;

        FollowerConstants.forwardZeroPowerAcceleration = -30.17792406024835;
        FollowerConstants.lateralZeroPowerAcceleration = -69.10862875630924;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(20,0,0.001,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.0001,0,0.00005,0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(3.4,0,0.4,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1.5,0,0.1,0);

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.035,0,0.0022,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.02,0,0.00004,0.6,0);

        FollowerConstants.zeroPowerAccelerationMultiplier = 3.5;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.25;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.nominalVoltage = 12.54;

        FollowerConstants.useVoltageCompensationInAuto = false;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
