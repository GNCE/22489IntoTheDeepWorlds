package config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import config.core.utils.SubsystemCore;

public class Extend extends SubsystemCore {
    public DcMotorEx IntakeExtend;

    private PIDController controller, visionPID, hangPID;
    public static double p = 0.02, i = 0, d = 0.00053;
    public static double vp = 0.03, vi = 0, vd = 0.00027;
    public static double hp = 0.03, hi=0, hd = 0.00027, hf = -0.0001;
    public int target = 0;

    public enum PIDStates{
        MANUAL, NORMAL, VISION, HANG;
    }

    private PIDStates currentPIDState;

    @Override
    public void init() {
        IntakeExtend = hardwareMap.get(DcMotorEx.class, "horizExtend");
        IntakeExtend.setDirection(DcMotor.Direction.REVERSE);
        IntakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void holdLift(){
        switch(currentPIDState){
            case NORMAL:
                IntakeExtend.setPower();
        }
    }

    public void telemetry(){
        tel.addData("Intake Extension Pos", IntakeExtend.getCurrentPosition());
        tel.addData("Intake Extension Velocity", IntakeExtend.getVelocity());
        tel.addData("Intake Extension Target", target);
    }
}