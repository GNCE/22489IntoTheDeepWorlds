package config.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import config.core.utils.Alliance;
import config.core.utils.Opmode;
import config.core.utils.SubsystemCore;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import config.subsystems.Extend;
import config.subsystems.HangServoSubsys;
import config.subsystems.Outtake;
import config.subsystems.Lift;

public class Robot extends com.arcrobotics.ftclib.command.Robot {
    private HardwareMap h;
    private Telemetry tel;
    private static Alliance a = Alliance.RED;
    private Opmode op = Opmode.TELEOP;
    private Outtake o;
    private Lift l;
    private Extend e;
    private HangServoSubsys hs;
    private Gamepad g1;
    private Gamepad g2;
    private Follower f;

    private void initSubsys(){
        SubsystemCore.setGlobalParameters(this.h, this.tel);
        this.o = new Outtake();
        this.l = new Lift();
        this.e = new Extend();
        this.hs = new HangServoSubsys();
    }

    public Robot(HardwareMap h, Telemetry tel, Opmode op, Gamepad g1, Gamepad g2, Pose startPose, boolean robotCentric){
        this.h = h;
        this.tel = tel;
        this.g1 = g1;
        this.g2 = g2;
        this.op = op;

        f = new Follower(this.h, FConstants.class, LConstants.class);
        f.setStartingPose(startPose);
    }

    public Alliance getA() {
        return a;
    }

    public void setA(Alliance a) {
        Robot.a = a;
    }
}
