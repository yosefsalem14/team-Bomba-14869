
package org.firstinspires.ftc.teamcode.utilities;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;


public class vision {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "Ae31TOj/////AAABmU6qla4qL0GxgNm3Sj+5YWQXbuPYfw459YaMe+A/w13jJTx4ZE0ArONrBmezY2yfFQB4wsBQzmBif8AFghBNyvrIFTCjzGUePnyNDwghP5UZt1ed/yUkkK3PmGpxHQ8VLJvLogJ/2/MvG6g+H9mrvIWSnurJSnkNbr96s4umRABLLkhBSCd/mN2HwbUkXL9xNpipvdeNqWIrpH4Mt04aa81wgFQBxhCbqFZYwEfM+VnGnttvCIx1tcI1YI0ktylCCL3lwXPPCNNdbs89xrJamIMB7zrKNEJ/S08mKCCDKJO2W2o2yK3JzbdWNXh//fsEm5N+OfZKgBjGz8IM082RLQT7RZPVhKGJyl6D2AniDyVF";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;
    private HardwareMap hw;
    private int runs;
    public vision(HardwareMap hw){
        this.runs = 0;
        this.hw = hw;
    }

    public void init() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
    }

    public int sign(double num){
        if(num>0){
            return 1;
        }else if(num<0){
            return -1;
        }
        return 0;
    }
    public int getIter(){
        return this.runs;
    }
    public void activate(){
        if (tfod != null) {
            tfod.activate();
        }
    }
    public DetectedObject getPos() {
        DetectedObject gold = new DetectedObject(-1,0);
        DetectedObject silver1=new DetectedObject(-1,0);
        DetectedObject silver2=new DetectedObject(-1,0);
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2 || updatedRecognitions.size() == 3) {
                    this.runs++;
                    for (int x = 0; x < updatedRecognitions.size(); x++) {
                        Recognition recognition = updatedRecognitions.get(x);
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            gold = new DetectedObject((int) recognition.getLeft()
                                    ,this.calcAngle(recognition,AngleUnit.DEGREES));

                        } else if (silver1.getPos() == -1) {
                            silver1 = new DetectedObject((int) recognition.getLeft()
                                    ,this.calcAngle(recognition,AngleUnit.DEGREES));
                        } else {
                            silver2 = new DetectedObject((int) recognition.getLeft()
                                    ,this.calcAngle(recognition,AngleUnit.DEGREES));
                        }
                    }
                    int goldMineralX = gold.getPos();
                    int silverMineral1X = silver1.getPos();
                    int silverMineral2X = silver2.getPos();
                    if (updatedRecognitions.size() == 2) {
                        if (goldMineralX == -1) {
                            gold.setID(ObjectPositions.RIGHT);//the positions are only for identifications
                            //if you only see 2 and the golden is not in view,
                            // turn to teh silver piece that's on the far left
                            gold.setAngle(-sign(silver1.getAngle())*25.0);
                        } else if (goldMineralX < silverMineral1X) {
                            gold.setID(ObjectPositions.CEMTER);
                        }else {
                            gold.setID(ObjectPositions.LEFT);
                        }
                    } else if (updatedRecognitions.size() == 3) {
                        if(goldMineralX!=-1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                gold.setID(ObjectPositions.LEFT);
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                gold.setID(ObjectPositions.CEMTER);

                            }else{
                                gold.setID(ObjectPositions.RIGHT);
                            }
                    }else{
                            gold.setID(ObjectPositions.UNKNOWN);
                        }
                }
            }
        }
    }
        return gold;
    }
    public void shutdown(){
        if(tfod!=null){
            tfod.shutdown();
        }
    }
    public double calcAngle(Recognition R,AngleUnit angleUnit) {
        float focalLength = vuforia.getCameraCalibration().getFocalLength().getData()[0];
        double adjacentSideLength = focalLength;

        double oppositeSideLength = ((R.getTop()+R.getBottom()) * 0.5f) - (0.5f * R.getImageHeight());

        double tangent = oppositeSideLength / adjacentSideLength;
        double angle = angleUnit.fromRadians(Math.atan(tangent));
        return angle;
    }





    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = this.hw.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", this.hw.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();//TODO tfodMonitorViewId;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
