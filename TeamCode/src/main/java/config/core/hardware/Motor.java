package config.core.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Motor {
    private DcMotorEx motor;
    private int encoderOffset;
    public Motor(DcMotorEx motor){
        this.motor = motor;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void resetEncoder(){
        encoderOffset = motor.getCurrentPosition();
    }

    public void stopAndResetEncoder(){

    }
}
