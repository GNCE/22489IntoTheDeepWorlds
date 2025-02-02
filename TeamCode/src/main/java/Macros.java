import com.qualcomm.robotcore.util.ElapsedTime;

public class Macros {
    private Intake intake;
    private OuttakeLift outtakeLift;
    private Outtake outtake;


    private enum TransferState {
        NOTHING,
        READY_OUTTAKE_LIFT,
        READY_OUTTAKE_ARM,
        OPEN_CLAW,
        INTAKE_RETRACT,

    };

    private ElapsedTime transferTime;
    private TransferState transferState = TransferState.NOTHING;
    public void initTransfer(){
        transferState = TransferState.READY_OUTTAKE_LIFT;
    }
    private void setTransferState(TransferState newState){
        transferState = newState;
        transferTime.reset();
    }
    public void transfer(){
        switch (transferState){
            case READY_OUTTAKE_LIFT:
                if(outtakeLift.getCurrentPosition() < 300) outtakeLift.LiftTarget(300);
                if(!outtakeLift.isBusy()){
                    outtake.pivotToTransfer();
                    setTransferState(TransferState.READY_OUTTAKE_ARM);
                }
                break;
            case READY_OUTTAKE_ARM:
                if(transferTime.time() > 2){ // Time it takes for arm to pivot
                    outtake.setClaw(true);
                    setTransferState(TransferState.OPEN_CLAW);
                }
                break;
            case OPEN_CLAW:
                if(transferTime.time() > 1){
                    intake.ManualRetract();
                    intake.flipToTransfer();
                    setTransferState(TransferState.NOTHING);
                }
                break;
            default:
                break;
        }
    }
}
