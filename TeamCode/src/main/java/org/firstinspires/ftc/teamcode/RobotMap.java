package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class  RobotMap {

    public RobotMap robot;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor lift = null;
    public Servo leftclaw = null;
    public Servo rightclaw = null;
    public Servo lefttail = null;
    public Servo righttail = null;
    public DistanceSensor leftSensor;
    public DistanceSensor rightSensor;
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
        imu = ahwmap.get(BNO055IMU.class, "imu");

        leftclaw = ahwmap.get(Servo.class, "left_claw");
        rightclaw = ahwmap.get(Servo.class, "right_claw");
        lefttail = ahwmap.get(Servo.class, "left_tail");
        righttail = ahwmap.get(Servo.class, "right_tail");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

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

    public synchronized void setLift(double power){
        lift.setPower(power);
    }

    public synchronized int getPos(){
        return rightFront.getCurrentPosition();

    }

    public synchronized void pinch(){
        leftclaw.setPosition(1);
        rightclaw.setPosition(0);
    }
    public synchronized void notPinch(){
        leftclaw.setPosition(.6);
        rightclaw.setPosition(.4);
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
    public synchronized void closeTail(){
       lefttail.setPosition(1);
       righttail.setPosition(.5);
    }
    public synchronized void openTail(){
        lefttail.setPosition(.5);
        righttail.setPosition(1);
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

    public void Gyroturn(float degrees) {

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        float Kp = (float) 0.012;
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


/*        imu = ahwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        sensorRange = ahwmap.get(DistanceSensor.class, "sensor_range");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;*/


    }


    public void encoderDrive(double speed,


                                     double inches) {


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


        leftBack.setPower(speed);


        rightBack.setPower(speed);


        leftFront.setPower(speed);


        rightFront.setPower(speed);

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


        }
    }

