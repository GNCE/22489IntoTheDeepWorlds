import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
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
            outtake.pivotToFront();
        }else if (gamepad1.b){
            outtake.pivotToPickupBack();
        }else if (gamepad1.x){
            outtake.pivotToScoreSamp();
        }else if (gamepad1.a){
            outtake.pivotToTransfer();
        }
        if (gamepad1.dpad_left){
            outtake.setClaw(true);
        } else {
            outtake.setClaw(false);
        }
        outtake.loop();
        telemetry.update();
    }
}
