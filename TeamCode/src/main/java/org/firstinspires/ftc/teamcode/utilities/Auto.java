package org.firstinspires.ftc.teamcode.utilities;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import java.util.ArrayList;
///////////////////      ^_^      //////////////
public abstract class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    private PID motorDist;
    private PID motorTurn;
    private Orientation angles;
    private BNO055IMU imu;
    private vision objectDetector;
    final int ACCURACY = 10;
    /* get the IMU sensor

     */
    private void getIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
    }


    /* reset all the PID's used in the Auto class
     */
    private void reset(){
        motorTurn.reset();
        motorDist.reset();
    }


    /*
     initialize all the variables(including PIDs)
     */
    private void initialize(){
        objectDetector = new vision(hardwareMap);
        angles=new Orientation();
        objectDetector.init();
        getIMU();
        //TODO: Tune the PIDs!
        motorDist = new PID(0.0004995,0.0,0.008877);
        motorTurn = new PID(0.0158878789,0.0,0.00777);
        //motorTurn = new PID(0.01849998542908,0.0,0.000598858);
    }

    /*
            ***VISION FUNCTIONS***
     */
    private void act(){
        objectDetector.activate();
    }
    private ArrayList<DetectedObject> getDetections(double timeOut){
        ArrayList<DetectedObject> objects = new ArrayList<>();
        //it goes to the last read when it fails
        int i = 0;
        runtime.reset();
        while(i<this.ACCURACY&&opModeIsActive()&&runtime.seconds()<=timeOut){
            DetectedObject nextPos = objectDetector.getPos();
            objects.add(nextPos);
            i++;
        }
        return objects;
    }
    private void shut(){
            objectDetector.shutdown();
    }
    private DetectedObject getMatchingId(ArrayList<DetectedObject> detections,int ID){
        for(int i =0;i<detections.size();i++){
            int currentValue = detections.get(i).getID().getIntVal();
            if(currentValue == ID+1){
                return detections.get(i);
            }
        }
        return new DetectedObject(0,0);
    }

    ///////////////////////////////////////////////////////

    /*
        ******NOTE******
        * Both turn and move take an array of commands,
        * the functions will execute these commands one at the same time
        * this is done to give the programmer more flexibility,
        *
        * THIS IS WRONG, the functions only execute the first command,
        * might redesign it and only run the loop through commands that have
        * encoders as it's hard-coded right now and doesn't work properly
        * with multiple encoders,
        * ALSO NODE:
        * when a command's direction
     */


    /*
        IMU_TURN function:
        robot will work with a PID until it reaches the desired angle
        the robot will automatically stop moving if the command times out
        (if the timer count exceeds timeout)
     */
    private void turn(Commands[] comms,double angle,double timeout) throws InterruptedException{
        if (opModeIsActive()) {
            reset();
            //rest run time
            runtime.reset();


            //wait until the moto1rs reach their goal
            //or until the time runs out
            boolean canRun;
            canRun = Math.abs(Math.abs(angle)-Math.abs(angles.firstAngle))>=10;
            while (opModeIsActive() &&canRun&&
                        runtime.seconds() < timeout) {
                    angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    angle = angle%360;
                    angles.firstAngle = angles.firstAngle%360;

                    double dist = (angle)-angles.firstAngle;

                    double D = Math.abs(angle) - Math.abs(angles.firstAngle);
                    if(Math.abs(dist)>180) {
                        dist =  -dist;
                    }
                    double power = motorTurn.getPower(-dist);
                    for(Commands command : comms){command.updatePower(power);}
                    //work until all the motors stop or until the time runs out
                telemetry.addData("angle",angles.firstAngle);
                    telemetry.addData("Target angle",angle);
                    telemetry.addData("dist", dist);
                    telemetry.update();
                    canRun = Math.abs(D)>=10;
             }
             for(Commands command : comms){
                command.stop();
             }
             Thread.sleep(100);
            telemetry.addData("status","finished");
            telemetry.update();
        } }

    private void turn(Commands[] comms, double Power, double angle,double timeout) throws InterruptedException{
        if (opModeIsActive()) {
            reset();
            //rest run time
            runtime.reset();


            //wait until the moto1rs reach their goal
            //or until the time runs out
            boolean canRun;
            canRun = Math.abs(Math.abs(angle)-Math.abs(angles.firstAngle))>=10;
            while (opModeIsActive() &&canRun&&
                    runtime.seconds() < timeout) {
                angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = angle%360;
                angles.firstAngle = angles.firstAngle%360;

                double dist = (angle)-angles.firstAngle;

                double D = Math.abs(angle) - Math.abs(angles.firstAngle);
                if(Math.abs(dist)>180) {
                    dist =  -dist;
                }
                double power = motorTurn.getPower(-dist);
                for(Commands command : comms){command.updatePower(power,Power);}
                //work until all the motors stop or until the time runs out
                telemetry.addData("angle",angles.firstAngle);
                telemetry.addData("Target angle",angle);
                telemetry.addData("dist", dist);
                telemetry.update();
                canRun = Math.abs(D)>=10;
            }
            for(Commands command : comms){
                command.stop();
            }
            telemetry.addData("status","finished");
            telemetry.update();
        } }


    /*
        ENCODER_DRIVE(MOVE) function:
        robot will keep moving until it reachess the desired distance
        the function will automatically stop when the timer count exceeds
        the timeout variable

     */
    private void move(Commands[] comms,double distance,double timeout)throws InterruptedException {
        if (opModeIsActive()) {
            reset();
            for(Commands command : comms) {
                command.init(distance);
            }

            //rest run time
            runtime.reset();



            //Cait until the motors reach their goal
            //or until the time runs out
            boolean canRun = true;
                for(Commands command:comms) {
                while (opModeIsActive() && canRun&&
                        runtime.seconds() < timeout) {
                            double dist = command.getDist();
                            double power = motorDist.getPower(dist);
                            for(Commands C : comms) {
                                C.updatePower(power);
                            }
                            //work until all the motors stop or until the time runs out
                            telemetry.addData("dist", dist);
                            telemetry.update();
                            canRun = command.canMove();
                        }
                    }


            for(Commands command : comms){
                command.stop();
            }
            }
        }

    private void move(Commands[] comms, double Power, double distance,double timeout)throws InterruptedException {
        if (opModeIsActive()) {
            reset();
            for(Commands command : comms) {
                command.init(distance);
            }

            //rest run time
            runtime.reset();



            //Cait until the motors reach their goal
            //or until the time runs out
            boolean canRun = true;
            for(Commands command:comms) {
                while (opModeIsActive() && canRun&&
                        runtime.seconds() < timeout) {
                    double dist = command.getDist();
                    double power = motorDist.getPower(dist);
                    for(Commands C : comms) {
                        C.updatePower(power,Power);
                    }
                    //work until all the motors stop or until the time runs out
                    telemetry.addData("dist", dist);
                    telemetry.update();
                    canRun = command.canMove();
                }
            }


            for(Commands command : comms){
                command.stop();
            }
            Thread.sleep(100);
            telemetry.addData("status","finished");
            telemetry.update();
        }
    }

    /*
        this is used in the move() function to check
        whether the one of the commands in the command array reached it's desired
        goal, it uses a bit of recursion to go through every command
        and check, all the commands in the Commands array will abort if
        it reaches the goal, this is done because we're only using 2
        encoders, the encoders are wired so each possible Commands array
        has at least one encoder.
     */
    //////////THIS WILL ONLY GET USED WITH MORE THAN 1 ENCODER//////
    private boolean isBusy(Commands comms[],int i){
        // checks if all the motors are busy
        if(i<comms.length)
            return comms[i].canMove() && isBusy(comms,i+1);
        return true;
    }


    //////////SERVO COMMANDS///////////
    /*
        opens the marker servos and throws the marker!

     */
    private void throwMarker(){
        //opens marker Servo to trow the marker into the depot
        if(robot.markerServo!=null) {
            robot.markerServo.setPosition(1);
        }else{
            telemetry.addData("Error","Marker Servo isn't connected");
        }
    }

    /*
        open the latching servos
     */
    private void openServos(){
        for(int i =0;i<this.robot.latches.length;i++){
            if(i%2 == 0) {
                this.robot.latches[i].setPosition(0);
            }else{
                this.robot.latches[i].setPosition(1);
            }
        }
    }

    /*
        close the latching servos
     */
    private void closeServos(){
        for(int i =0;i<this.robot.latches.length;i++){
            if(i%2 == 0) {
                this.robot.latches[i].setPosition(1);
            }else{
                this.robot.latches[i].setPosition(0);
            }
        }
    }

    /*
        this will be used by the programmer to call the movement type
        that he/she wants, this is done so the programmer doesn't have
        to memorise functions, more functions could potentially get added later

     */


    /////////////EXECUTE COMMANDS/////////////
    /*
        this will always be called in the start of every autonomous, it will initialize
        and get everything ready.
     */

    /**
     * initializes the robot and gets everything ready for the autonomous stage
     * @param robot the robot that the autonomous will work with
     */
    public void initAuto(Robot robot){
        this.robot = robot;
        robot.init(hardwareMap);
        robot.init2018Auto();
        initialize();
    }



    /**
     * returns the angle of the golden cube, stops running when the timer reaches
     * the timeout!
     * @param timeOut timer runs out when it reaches this
     * @return the angle
     */
    public double getGoldPosition(double timeOut){
        this.act();
        ArrayList<DetectedObject> detections = this.getDetections(timeOut);
        int[] counters = new int[3];
        double GOLD_POS=0;
        int maxCounter = 0;
        int index = 0;
        for(int i = 0;i<detections.size();i++){
            int pos = detections.get(i).getID().getIntVal();
            counters[pos-1]++;
        }
        for(int i = 0;i<counters.length;i++){
            if(counters[i]>maxCounter){
                maxCounter = counters[i];
                index = i;
            }
        }
        DetectedObject matching = getMatchingId(detections,index);
        GOLD_POS = matching.getAngle();
        this.shut();
        return GOLD_POS;
    }

    /**
     * takes an action and performs it
     * @param action the type of the action
     */
    public void execute(AutoDrivetype action){
        switch(action){
            case OPEN_SERVOS:
                this.openServos();
                break;
            case CLOSE_SERVOS:
                this.closeServos();
                break;
            case THROW_MARKER:
                this.throwMarker();
                break;
            default:
                telemetry.addData("error","unknown command");
                break;
        }

    }

    /**
     * takes actions and performs them
     * @param movement the type of the action
     * @param goal the stop condition of the action
     * @param timeOut the action stops when the timer reaches this
     * @throws InterruptedException
     */
    public void execute(AutoDrivetype movement, double goal, double timeOut)throws InterruptedException{
        switch(movement){
            case ENCODER_MOVE:
                this.move(this.robot.move,goal,timeOut);
                break;
            case IMU_TURN:
                this.turn(this.robot.turn,goal,timeOut);
                break;
            case ENCODER_STRAFE:
                this.move(this.robot.strafe,goal,timeOut);
                break;
            case ARM_MOVE:
                this.move(this.robot.armMove,goal,timeOut);
                break;
            default:
                telemetry.addData("error","unknown command");
                break;
        }

    }

    /**
     *
     * takes actions and performs them
     * @param movement the type of the action
     * @param power limit the motor power to this
     * @param goal the stop condition of the action
     * @param timeOut the action stops when the timer reaches this
     * @throws InterruptedException
     */
    public void execute(AutoDrivetype movement, double power, double goal, double timeOut)throws InterruptedException{
        switch(movement){
            case ENCODER_MOVE:
                    this.move(this.robot.move, Math.abs(power), goal, timeOut);
                break;
            case IMU_TURN:
                this.turn(this.robot.turn, Math.abs(power), goal, timeOut);
                break;
            case ENCODER_STRAFE:
                this.move(this.robot.strafe, Math.abs(power), goal, timeOut);
                break;
            case ARM_MOVE:
                this.move(this.robot.armMove, Math.abs(power), goal, timeOut);
                break;
            default:
                telemetry.addData("error","unknown command");
                break;
        }

    }
}
