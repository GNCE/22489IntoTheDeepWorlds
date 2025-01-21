import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name = "outakey")
public class outtaketesty extends OpMode {
    private Outtake outtake;

    public void init() {
        outtake = new Outtake(hardwareMap);
    }
    public void start(){


    }

    public void loop() {
        if (gamepad1.y){
            outtake.pivotToScoreorpickupSpecFront();
        }else if (gamepad1.b){
            outtake.pivotToPickupBack();
        }else if (gamepad1.x){
            outtake.pivotToScoreSampandBackSpec();
        }else if (gamepad1.a){
            outtake.pivotToTransfer();
        }
        if (gamepad1.dpad_left){
            outtake.openClaw();
        } else {
            outtake.closeClaw();
        }
        outtake.updatePivPosition();
        telemetry.update();
    }
}
