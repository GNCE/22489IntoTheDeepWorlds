package config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import config.core.utils.SubsystemCore;

public class DriveSubsys extends SubsystemCore {
    DcMotorEx leftRear, leftFront, rightRear, rightFront;

    @Override
    public void init(){
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
