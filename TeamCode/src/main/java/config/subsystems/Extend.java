package config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import config.core.utils.SubsystemCore;

public class Extend extends SubsystemCore {
    public DcMotorEx IntakeExtend;

    @Override
    public void init() {
        IntakeExtend = hardwareMap.get(DcMotorEx.class, "horizExtend");
        IntakeExtend.setDirection(DcMotor.Direction.REVERSE);
        IntakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void telemetry(){
        tel.addData("Intake Extension Pos:", IntakeExtend.getCurrentPosition());
        tel.addData("Intake Velocity:", IntakeExtend.getVelocity());
        tel.addData();
    }
}