package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.VuforiaStuff;

import static java.lang.Thread.sleep;

public class  RobotMap {

    public RobotMap robot;
    private ElapsedTime runtime = new ElapsedTime();
    public ColorSensor skyStone;
    public ColorSensor line;
    public DistanceSensor distanceSensor;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor lift = null;
    public DcMotor sonny2 = null;
    public DcMotor extendy = null;
    public Servo leftclaw = null;
    public Servo rightclaw = null;
    public Servo lefttail = null;
    public Servo righttail = null;
    public Servo arm = null;
    //public Servo tape = null;
    public Servo capLatch = null;
    public CRServo tape = null;
    public DistanceSensor leftSensor;
    public DistanceSensor rightSensor;
    //public DistanceSensor smallEyes;
    private Orientation angles;
    private Acceleration acceleration;
    public BNO055IMU imu;
    public float Gerror;
    public float deltaError;
    public float Derror;
    public float currentTime;
    public float currentError;
    public float preError;
    float deltaTime = 0;
    float preTime = 0;
    float PDout = 0;
    //1120 counts per rotation
    double COUNTS_PER_INCH = 64.81;
    double SCALE_FACTOR = 255;

    public VuforiaStuff vuforiaStuff;

    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "AU0W0n3/////AAABmR8XzF3guUvdioVZmVSPQWgSHZEGEsa/li/PrDXTJOzpNa7++SSQmj96GVwRjrH8zk+oQ9P5Efp4sz80tiIS/rmHGtedHonNHn0He1tZoedzk5W24EiWOBo52Wr3tucP2cfP9zucxfOnKahG1cGKDhEV65GPv8YyxbVHYmqID0TEG3GooDJDUSdQqlDVLfVcf+EkFn1CHthzF3xMk7vMdttHHdXHSaYIWdTcx3ZM3HRNUkaBCkkXEKcnlS/SS5MQvlxTeeKVGjhKyWqOQG8JssMTrOrmXlmq5JJyziqjRE0LHvetRx2PpxEpEb2SnCk0UeSJV6U/IccfX/tGyZaTTJfm/XMddlMj9YJ1YpZLvPCk ";


    //HardwareMap hwMap =  null;


    /* Constructor */
    public RobotMap() {

    }

    public void init(HardwareMap ahwmap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = ahwmap.get(DcMotor.class, "left_front");
        rightFront = ahwmap.get(DcMotor.class, "right_front");
        leftBack = ahwmap.get(DcMotor.class, "left_back");
        rightBack = ahwmap.get(DcMotor.class, "right_back");
        lift = ahwmap.get(DcMotor.class, "lift");
        extendy = ahwmap.get(DcMotor.class, "extendy");
        imu = ahwmap.get(BNO055IMU.class, "imu");
        capLatch = ahwmap.get(Servo.class, "capLatch");
        sonny2 = ahwmap.get(DcMotor.class, "Deltaakamentors");


        leftclaw = ahwmap.get(Servo.class, "left_claw");
        rightclaw = ahwmap.get(Servo.class, "right_claw");
        lefttail = ahwmap.get(Servo.class, "left_tail");
        //tape = ahwmap.get(Servo.class, "tape");

        righttail = ahwmap.get(Servo.class, "right_tail");
        arm = ahwmap.get(Servo.class, "arm");
        tape = ahwmap.get(CRServo.class, "tape");

        skyStone = ahwmap.get(ColorSensor.class,"skystone");//this was Gran's fault, saved by Yung Nigil//
        distanceSensor = ahwmap.get(DistanceSensor.class,"skystone");
        line = ahwmap.get(ColorSensor.class, "line");
        //smallEyes = ahwmap.get(DistanceSensor.class, "smallEyes");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

*/
        int cameraMonitorViewId = ahwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwmap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }
    public void resetEncoders(){
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public synchronized void drive(double power, long time) throws InterruptedException {
        setMotor_br(power);
        setMotor_bl(power);
        setMotor_fl(power);
        setMotor_fr(power);
        sleep(time);
        setMotor_br(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_fr(0);
        sleep(500);
    }

    public synchronized void turn(double power, long time) throws InterruptedException {
        setMotor_bl(-power);
        setMotor_fl(-power);
        setMotor_br(power);
        setMotor_fr(power);
        sleep(time);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
        setMotor_fr(0);
    }

    public synchronized void strafeRightForwardTime(double power, long time) throws InterruptedException {
        setMotor_fr(-power+.2);
        setMotor_bl(-power+.2);
        setMotor_fl(power+.2);
        setMotor_br(power+.2);
        sleep(time);
        setMotor_fr(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
    }
    public synchronized void strafeLeftForwardTime(double power, long time) throws InterruptedException {
        setMotor_fr(power + .2);
        setMotor_bl(power + .2);
        setMotor_fl(-power + .2);
        setMotor_br(-power + .2);
        sleep(time);
        setMotor_fr(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
    }
    public synchronized void strafeLeftTime(double power, long time) throws InterruptedException {
        setMotor_fr(power);
        setMotor_bl(power);
        setMotor_fl(-power);
        setMotor_br(-power);
        sleep(time);
        setMotor_fr(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
        sleep(250);

    }
    public synchronized void strafeRightTime(double power, long time) throws InterruptedException {
        setMotor_fr(-power);
        setMotor_bl(-power);
        setMotor_fl(power);
        setMotor_br(power);
        sleep(time);
        setMotor_fr(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
        sleep(250);
    }

    public synchronized void strafeLeft() throws InterruptedException {
        setMotor_fr(1);
        setMotor_bl(1);
        setMotor_fl(-1);
        setMotor_br(-1);


    }
    public synchronized void strafeRight() throws InterruptedException {
        setMotor_fr(-1);
        setMotor_bl(-1);
        setMotor_fl(1);
        setMotor_br(1);

    }

    public synchronized void stop() {
        setMotor_br(0);
        setMotor_fl(0);
        setMotor_bl(0);
        setMotor_fr(0);
    }
    public synchronized void leftArc(double power, long time) throws InterruptedException {
        setMotor_fl(power);
        setMotor_bl(power);
        sleep(time);
        setMotor_fl(0);
        setMotor_bl(0);
        sleep(500);
    }
    public synchronized void rightArc(double power, long time) throws InterruptedException {
        setMotor_fr(power);
        setMotor_br(power);
        sleep(time);
        setMotor_fl(0);
        setMotor_bl(0);
        sleep(500);
    }
    public synchronized void bigExtend(double power, long time) throws InterruptedException {
        setExtendy(power);
        sleep(time);
        setExtendy(0);
        sleep(500);
    }
    public synchronized void skizzorLift(double power, long time) throws InterruptedException {
        setLift(power);
        setsonny2(power);
        sleep(time);
        setLift(0);
        setsonny2(0);
        sleep(500);
    }



    public synchronized void setLift(double power){ lift.setPower(power);}
    public synchronized void setsonny2(double power) {sonny2.setPower(power);}
    public synchronized void setExtendy(double power){
        extendy.setPower(power);
    }


    public synchronized int getPos(){
        return rightFront.getCurrentPosition();

    }

    public synchronized void notPinch(){
        leftclaw.setPosition(.01);
        rightclaw.setPosition(0.61);
    }
    public synchronized void pinch(){
        leftclaw.setPosition(.5);
        rightclaw.setPosition(.12);
    }


    public synchronized void setMotor_fl(double power) {
        double convertedPower = (power);

        leftFront.setPower(convertedPower);
    }

    public synchronized void setMotor_bl(double power) {
        double convertedPower = (power);

        leftBack.setPower(convertedPower);
    }

    public synchronized void setMotor_fr(double power) {
        double convertedPower = (power);

        rightFront.setPower(convertedPower);
    }

    public synchronized void setMotor_br(double power) {
        double convertedPower = (power);

        rightBack.setPower(convertedPower);
    }
    public synchronized void closeTail() {
       lefttail.setPosition(1);
       righttail.setPosition(.5);
    }
    public synchronized void openTail() {
        lefttail.setPosition(.5);
        righttail.setPosition(1);
    }
    /*public boolean Open(){
        boolean Open;
        if (robot.smallEyes.getDistance(DistanceUnit.CM) < .1) {
            Open = true;
        }
        else {
            Open = false;
        }
        return Open;
    }*/

    public float[] hsv(){
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((int) (skyStone.red() * SCALE_FACTOR),
                (int) (skyStone.green() * SCALE_FACTOR),
                (int) (skyStone.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues;
    }
    public float linered(){
        return line.red();
    }
    public float lineblue         (){
        return line.blue();
    }

    public void mechanumDrive(float forward, float strafe, float rotation){
        if(Math.abs(forward)< .2){
            forward = 0;
        }

        if(Math.abs(strafe)< .2){
            strafe = 0;
        }

        if(Math.abs(rotation)< .2){
            rotation = 0;
        }
        leftBack.setPower(forward + strafe - rotation);
        leftFront.setPower(forward - strafe - rotation);
        rightBack.setPower(forward - strafe + rotation);
        rightFront.setPower(forward + strafe + rotation);
    }
    public void imuINIT() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;//new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public float getHeading() {
        //PLZ dont touch *touch*
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double[] getAcceleration() {

        acceleration = imu.getAcceleration();
        double[] accel = new double[]{acceleration.xAccel, acceleration.yAccel, acceleration.zAccel};

        return accel;
    }

    public void gyroturn(float degrees) throws InterruptedException {

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        float Kp = (float) 0.03;
        float Kd = (float) 0.0001;

        while (true) {
            deltaTime = System.nanoTime() - preTime;
            Gerror = getHeading() - degrees;
            PDout = (Kp * Gerror) + (Kd * (Gerror / deltaTime));
            //PDout = Kp * Gerror;
            leftBack.setPower(PDout);
            leftFront.setPower(PDout);
            rightBack.setPower(-PDout);
            rightFront.setPower(-PDout);
            //preTime = currentTime;
            if (Math.abs(Gerror) <= 5) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
                break;
            }
        }
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }

    public void gyroStrafe(double power, double target,long time) throws InterruptedException {

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        float Kp = (float) 0.03;
        float Kd = (float) 0.0001;

        double out = 0;

        ElapsedTime strafeTime = new ElapsedTime();

        while (true) {
            out = Kp * (getHeading()-target);
            leftBack.setPower(-power + out);
            leftFront.setPower(power + out);
            rightBack.setPower(power - out);
            rightFront.setPower(-power - out);
            //preTime = currentTime;
            if (strafeTime.milliseconds() > time) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
                break;
            }
        }
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }
    public void gyroDrive(double power, double target, long time) throws InterruptedException {

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //float Kp = (float) 0.015;
        float Kp = (float) 0.03;
        float Kd = (float) 0.0001;

        double out = 0;

        ElapsedTime driveTime = new ElapsedTime();

        while (true) {
            out = Kp * (getHeading()-target);
            leftBack.setPower(power + out);
            leftFront.setPower(power + out);
            rightBack.setPower(power - out);
            rightFront.setPower(power - out);
            //preTime = currentTime;
            if (driveTime.milliseconds() > time) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
                break;
            }
        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }


    public void encoderDrive(double speed,


                                     double inches) throws InterruptedException {


        int newLeftBackTarget;


        int newrightBackTarget;


        int newLeftFrontTarget;


        int newRightFrontTarget;


        boolean rightAhead = false;


        boolean leftAhead = false;




        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newLeftBackTarget = (int) (inches * COUNTS_PER_INCH);


        newrightBackTarget = (int) (inches * COUNTS_PER_INCH);


        newLeftFrontTarget = (int) (inches * COUNTS_PER_INCH);


        newRightFrontTarget = (int) (inches * COUNTS_PER_INCH);


        leftBack.setTargetPosition(newLeftBackTarget);


        rightBack.setTargetPosition(newrightBackTarget);


        leftFront.setTargetPosition(newLeftFrontTarget);


        rightFront.setTargetPosition(newRightFrontTarget);


        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftBack.setPower(speed *1.5);


        rightBack.setPower(speed*1.5);


        leftFront.setPower(speed*1.5);


        rightFront.setPower(speed*1.5);

        while (rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()){

            int x = 0;
            x ++;
        }

        leftBack.setPower(0);


        rightBack.setPower(0);


        leftFront.setPower(0);


        rightFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

        }
    public void encoderGyroDrive(double speed, double target,


                             double inches) throws InterruptedException {


        int newLeftBackTarget;


        int newrightBackTarget;


        int newLeftFrontTarget;


        int newRightFrontTarget;


        boolean rightAhead = false;


        boolean leftAhead = false;

        double out = 0;

        float Kp = (float) 0.03;


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newLeftBackTarget = (int) (inches * COUNTS_PER_INCH);


        newrightBackTarget = (int) (inches * COUNTS_PER_INCH);


        newLeftFrontTarget = (int) (inches * COUNTS_PER_INCH);


        newRightFrontTarget = (int) (inches * COUNTS_PER_INCH);


        leftBack.setTargetPosition(newLeftBackTarget);


        rightBack.setTargetPosition(newrightBackTarget);


        leftFront.setTargetPosition(newLeftFrontTarget);


        rightFront.setTargetPosition(newRightFrontTarget);


        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftBack.setPower(speed);


        rightBack.setPower(speed);


        leftFront.setPower(speed);


        rightFront.setPower(speed);

        while (rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()){


            out = Kp * (getHeading()-target);
            leftBack.setPower(speed + out);
            leftFront.setPower(speed + out);
            rightBack.setPower(speed - out);
            rightFront.setPower(speed - out);

        }

        leftBack.setPower(0);


        rightBack.setPower(0);


        leftFront.setPower(0);


        rightFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

    }
    }

