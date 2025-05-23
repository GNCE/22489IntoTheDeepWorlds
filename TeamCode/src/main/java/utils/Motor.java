package utils;

import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Motor {
    private final DcMotorEx motor;
    private double powerCacheThreshold;
    private int encoderOffset;
    private double curPower;
    public Motor(DcMotorEx motor){
        this.motor = motor;
        this.powerCacheThreshold = 0;
        encoderOffset = 0;
    }
    public Motor(DcMotorEx motor, double powerCacheThreshold){
        this.motor = motor;
        this.powerCacheThreshold = powerCacheThreshold;
        encoderOffset = 0;
    }

    public int getPosition(){
        return this.motor.getCurrentPosition() - encoderOffset;
    }
    public double getVelocity(){ return this.motor.getVelocity(); }
    public double getCurrent(){ return this.motor.getCurrent(CurrentUnit.AMPS); }
    public void resetEncoder(){
        encoderOffset = this.motor.getCurrentPosition();
    }
    public void setPower(double power){
        power = MathUtils.clamp(power, -1, 1);
        if(Math.abs(curPower - power) > powerCacheThreshold || (power == 0 && curPower != 0)) this.motor.setPower(power);
        curPower = power;
    }
    public void setDirection(DcMotorSimple.Direction dir){
        this.motor.setDirection(dir);
    }
    public void setPowerCacheThreshold(double powerCacheThreshold){
        this.powerCacheThreshold = powerCacheThreshold;
    }
    public double getPower(){
        return curPower;
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb){
        this.motor.setZeroPowerBehavior(zpb);
    }
    public void setMode(DcMotor.RunMode rm){
        this.motor.setMode(rm);
    }
    public void stopAndResetEncoder(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderOffset = 0;
    }
}
