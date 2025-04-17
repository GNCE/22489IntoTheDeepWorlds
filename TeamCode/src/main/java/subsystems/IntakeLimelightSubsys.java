package subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLimelightSubsys extends SubsysCore {
    public static Limelight3A ll;
    public static Servo light;
    public static UnifiedTelemetry tel = new UnifiedTelemetry();
    public static LLResult llResult;
    private static double tx, ty, ta, angle;
    private static double[] pythonOutput;
    private static int pipelineNumber=4;
    private boolean prevllon = false, llon = false;
    public boolean isDataFresh(){ return llResult != null && llResult.getStaleness() < 100; }

    public boolean isRunning(){ return llon; }
    public void turnOn(){ llon = true; }
    public void turnOff(){ llon = false; }
    public boolean isResultValid(){
        if(!llon) return false;
        return isDataFresh() && (llResult.isValid() || getTa() > 6);
    }


    public double getTa() { return ta; }
    public double getTx(){ return tx; }
    public double getTy(){ return ty; }
    public double getAngle(){ return angle; }
    public void setPipelineNumber(int pipelineNum){
        if(pipelineNum > 6 || pipelineNum < 4) return;
        pipelineNumber = pipelineNum;
    }
    public void captureSnapshot(String key){
        ll.captureSnapshot(key);
    }

    public int getPipelineNumber(){ return pipelineNumber; }

    @Override
    public void init(){
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        light = hardwareMap.get(Servo.class, "light");
        ll.setPollRateHz(100);
        light.setPosition(0);
        turnOff();
    }

    @Override
    public void loop(){
        if(prevllon != llon){
            if(llon) ll.start();
            else ll.stop();
        }

        if(llon){
            light.setPosition(1);
            ll.pipelineSwitch(pipelineNumber);
            llResult = ll.getLatestResult();
            if(llResult != null){
                tx = llResult.getTx();
                ty = llResult.getTy();
                ta = llResult.getTa();

                pythonOutput = llResult.getPythonOutput();
                if(pythonOutput != null){
                    if(pythonOutput.length > 0) angle = pythonOutput[0];
                }
            }
        } else {
            light.setPosition(0);
        }

        prevllon = llon;

//        tel.addLine("Limelight Data");
//        tel.addData("Connected?", ll.isConnected());
//        tel.addData("Running", isRunning());
//        if(isRunning()){
//            tel.addData("Pipeline:", getPipelineNumber());
//            tel.addData("Data Valid", isResultValid());
//            tel.addData("Data Fresh", isDataFresh());
//            tel.addData("Detected X", getTx());
//            tel.addData("Detected Y", getTy());
//            tel.addData("Detected Area", getTa());
//            tel.addData("Detected Angle", getAngle());
//        }
    }
}