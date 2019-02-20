package org.firstinspires.ftc.teamcode.utilities;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

///////////////////      ^_^      //////////////
public abstract class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    private PID movePID;
    private PID turnPID;
    private PID armPID;
    private PID latchPID;
    private Orientation angles;
    private BNO055IMU imu;
    private vision objectDetector;
    private int currentState = 0;

    private int currentState1 = 0;
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
        movePID.reset();
        turnPID.reset();
        armPID.reset();
        latchPID.reset();
    }


    /*
     initialize all the variables(including PIDs)
     */
    private void tunePIDControllers(){
        movePID = new PID(0.0004995,0.0,0.008877);
        turnPID = new PID(0.0158878789,0.0,0.00777);
        armPID = new PID(0.01654,0.0,0.0123);
        latchPID = new PID(0.0865436,0.0,0.0546);
    }
    private void initializeWithIMU(){
        objectDetector = new vision(hardwareMap);
        angles=new Orientation();
        objectDetector.init();
        getIMU();
        tunePIDControllers();
    }
    private void initializeWithoutIMU(){
        angles=new Orientation();
        tunePIDControllers();
    }
    /*
            ***VISION FUNCTIONS***
     */
    private void act(){
        objectDetector.activate();
        CameraDevice.getInstance().setFlashTorchMode(true);
    }
    private ArrayList<Integer> getDetections(double timeOut){
      ArrayList<Integer> objects = new ArrayList<>();
        //it goes to the last read when it fails
        int i = 0;
        runtime.reset();
        while(i<this.ACCURACY&&opModeIsActive()&&runtime.seconds()<=timeOut) {
            int nextPos = objectDetector.getPos();
            if (nextPos!=-1) {
                objects.add(nextPos);
                i++;
            }
        }
        return objects;
    }
    /**
     * returns the angle of the golden cube, stops running when the timer reaches
     * the timeout!
     * @param timeOut timer runs out when it reaches this
     * @return the angle
     */
    public double getGoldPosition(double timeOut){
        this.act();
        ArrayList<Integer> detections = this.getDetections(timeOut);
        double[] angles = new double[3];
        if(detections.size() == 0){
            return -1;
        }
        for(int pos : detections){
            angles[pos+1]++;
        }
        this.shut();
        return getMaxIndex(angles)-1;
    }
    private void shut(){
        CameraDevice.getInstance().setFlashTorchMode(false);
        objectDetector.shutdown();
    }
    public int getMaxIndex(double[] array){
        double max=0 ;
        int index=-1;
        for(int i =0;i<array.length;i++){
            if(array[i]>max){
                max = array[i];
                index = i;
            }
        }
        return index;

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
    private void turn(Commands[] comms,double angle,double timeout,PID pid)  {
        double dist;
        double power;
        boolean canRun;
        if (opModeIsActive()) {
            reset();
            //rest run time
            runtime.reset();
            //wait until the moto1rs reach their goal
            //or until the time runs out
            do{
                angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = angle%360;
                dist = (angle)-angles.firstAngle;
                if(Math.abs(dist)>180) {
                    dist =  -dist;
                }
                power = pid.getPower(-dist);
                for(Commands command : comms){command.updatePower(power);}
                //work until all the motors stop or until the time runs out
                telemetry.addData("angle",angles.firstAngle);
                telemetry.addData("Target angle",angle);
                telemetry.addData("dist", dist);
                telemetry.update();
                canRun = (Math.abs(dist)>5);
            }
            while (opModeIsActive() &&canRun&&
                        runtime.seconds() < timeout) ;
             for(Commands command : comms) {
                 command.stop();
             }
            telemetry.addData("status","finished");
            telemetry.update();
        }
    }

    private void turn(Commands[] comms, double powerLimit,
                      double angle, double timeout, PID pid)   {
        double dist;
        double power;
        boolean canRun;
        if (opModeIsActive()) {
            reset();
            //rest run time
            runtime.reset();
            //wait until the moto1rs reach their goal
            //or until the time runs out
            do {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = angle % 360;
                dist = (angle) - angles.firstAngle;
                if (Math.abs(dist) > 180) {
                    dist = -dist;
                }
                power = pid.getPower(-dist);
                for (Commands command : comms) {
                    command.updatePower(power, powerLimit);
                }
                //work until all the motors stop or until the time runs out
                telemetry.addData("angle", angles.firstAngle);
                telemetry.addData("Target angle", angle);
                telemetry.addData("dist", dist);
                telemetry.update();
                canRun = (Math.abs(dist) > 5);
            }
            while (opModeIsActive() && canRun &&
                    runtime.seconds() < timeout);
            for (Commands command : comms) {
                command.stop();
            }
            telemetry.addData("status", "finished");
            telemetry.update();
        }
    }


    /*
        ENCODER_DRIVE(MOVE) function:
        robot will keep moving until it reachess the desired distance
        the function will automatically stop when the timer count exceeds
        the timeout variable

     */
    private void move(Commands[] comms, double distance,
                      double timeout, PID pid)  {
        double dist;
        boolean canRun;
        double power;
        if (opModeIsActive()) {
            reset();
            for(Commands command : comms) {
                command.init(distance);
            }
            runtime.reset();
            do{
            //rest run time
            //Wait until the motors reach their goal
            //or until the time runs out
            dist = comms[0].getDist();
            power = pid.getPower(dist);
            for(Commands C : comms) {
                C.updatePower(power);
            }
            //work until all the motors stop or until the time runs out
            telemetry.addData("dist", dist);
            telemetry.update();
            canRun = comms[0].canMove()&&(Math.abs(power)>0.1);
            }
            while (opModeIsActive() && canRun&&
                   runtime.seconds() < timeout);

            for(Commands command : comms){
                command.stop();
            }
        }
    }

    private void move(Commands[] comms, double powerLimit,
                      double distance, double timeout, PID pid){
        double dist;
        boolean canRun;
        double power;
        if (opModeIsActive()) {
            reset();
            for(Commands command : comms) {
                command.init(distance);
            }
            do{
                //rest run time
                runtime.reset();
                //Wait until the motors reach their goal
                //or until the time runs out
                dist = comms[0].getDist();
                if(pid == null){
                    power = 1;
                }else {
                    power = pid.getPower(dist);
                }
                for(Commands C : comms) {
                    C.updatePower(power,powerLimit);
                }
                //work until all the motors stop or until the time runs out
                telemetry.addData("dist", dist);
                telemetry.update();
                canRun = comms[0].canMove()&&(power>0.1);
            }
            while (opModeIsActive() && canRun&&
                    runtime.seconds() < timeout);

            for(Commands command : comms){
                command.stop();
            }
        }
    }


    private void controlledMove(Commands[] comms, double powerLimit,
                      double distance, double timeout, PID pid){
        double dist;
        double power;
        for(Commands command : comms) {
            command.init(distance,timeout);
        }
        do {
            if(!(gamepad2.dpad_left||gamepad2.dpad_right||gamepad2.dpad_up)){
                break;
            }
            dist = comms[0].getDist();
            power = pid.getPower(dist);
            for (Commands C : comms) {
                C.updatePower(power, powerLimit);
            }
            //work until all the motors stop or until the time runs out
            telemetry.addData("dist", dist);
            telemetry.update();

        }
        while(comms[0].canMove());

            reset();
            for (Commands command : comms) {
                command.stop();
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
    // just don't use it it's not configured correctly this needs more work
    @Deprecated
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
        initializeWithIMU();
    }
    public void initControlled(Robot robot){
        this.robot = robot;
        robot.init(hardwareMap);
        robot.init2018Auto();
        initializeWithoutIMU();
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
    public void execute(AutoDrivetype movement,
                        double goal, double timeOut){
        switch(movement){
            case ENCODER_MOVE:
                this.move(this.robot.move, -goal
                        , timeOut, movePID);
                break;
            case IMU_TURN:
                this.turn(this.robot.turn,
                        goal, timeOut, turnPID);
                break;
            case ARM_MOVE:
                this.move(this.robot.armMove,
                        goal, timeOut, armPID);
                break;
            case LATCH_MOVE:
                this.move(this.robot.latchMove, goal,
                        timeOut, latchPID);
                break;
            case INTAKE_MOVE:
                this.move(this.robot.intakeMove, goal,
                        timeOut, null);
                break;
            default:
                telemetry.addData("error","unknown command");
                break;
        }

    }
    {


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
    public void execute(AutoDrivetype movement, double power,
                        double goal, double timeOut) {
        switch(movement){
            case ENCODER_MOVE:
                this.move(this.robot.move, power, -goal
                        , timeOut, movePID);
                break;
            case IMU_TURN:
                this.turn(this.robot.turn, power,
                        -goal, timeOut, turnPID);
                break;
            case ARM_MOVE:
                this.move(this.robot.armMove,power,
                        goal, timeOut, armPID);
                break;
            case LATCH_MOVE:
                this.move(this.robot.latchMove, goal,power,
                        timeOut, latchPID);
                break;
            case INTAKE_MOVE:
                this.move(this.robot.intakeMove, goal,power,
                        timeOut, null);
                break;
            default:
                telemetry.addData("error","unknown command");
                break;
        }

    }
    public void goToPos(int state,AutoDrivetype type,double pos){
        switch (type) {
            case ARMS:
                if(currentState!=state) {
                    currentState=state;
                    controlledMove(this.robot.armMove, 1,
                            pos * currentState, 20, armPID);
                }
                break;
            case LATCH:
                if(currentState1!=state) {
                currentState1=state;
                controlledMove(this.robot.latchMove, 1,
                   -pos * currentState1, 20, latchPID);
                }
                break;
        }
    }
}

