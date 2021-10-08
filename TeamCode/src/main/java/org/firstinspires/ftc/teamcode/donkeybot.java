package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class donkeybot {

    public DcMotor frontLeft;
    public DcMotor rearLeft;
    public DcMotor rearRight;
    public DcMotor frontRight;

    private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU imu;
    Orientation angles;

    static final double     SCALE_FACTOR = 75.0/75.0; //  if drive speed = .2 or .3 use 75.0/75.0;  .5 is 75.0/76.0 .4 is 75.0/75.5 if drive_speed = .1, use 1.0; if drive_speed = .3, use 75.0/77.0 note that .3 has hard time braking
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (SCALE_FACTOR * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;

    private CRServo servo;


   // public ColorSensor sensorColor;
    public DigitalChannel sensorTouch;


    HardwareMap hwMap = null;
    private ElapsedTime period =new ElapsedTime();

    public donkeybot (){}


    public void init(HardwareMap ahwMap, LinearOpMode opmode){

        hwMap=ahwMap;

        frontLeft = hwMap.dcMotor.get("frontLeft");
        rearLeft = hwMap.dcMotor.get("rearLeft");
        frontRight = hwMap.dcMotor.get("frontRight");
        rearRight = hwMap.dcMotor.get("rearRight");

        servo = hwMap.crservo.get("servo");

        //sensorColor = hwMap.get(ColorSensor.class,"colorSensor");
        sensorTouch = hwMap.get(DigitalChannel.class,"touchSensor");



        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }



    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, LinearOpMode opmode) throws InterruptedException {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Send telemetry message to signify robot waiting;
        opmode.telemetry.addData("Status", "Resetting Encoders");    //
        opmode.telemetry.update();

        initRunWithEncoder();

        // Send telemetry message to indicate successful Encoder reset
        opmode.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                rearLeft.getCurrentPosition(),
                rearRight.getCurrentPosition());
        opmode.telemetry.update();

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
            newFrontRightTarget = (frontRight.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH));
            newRearLeftTarget = -(rearLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
            newRearRightTarget = -(rearRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            rearLeft.setTargetPosition(newRearLeftTarget);
            rearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();

            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            rearRight.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy())) {

                // Display it for the driver.
                opmode.telemetry.addData("Path1", "Running to %7d :%7d : %d : %d", newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                opmode.telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        rearLeft.getCurrentPosition(),
                        rearRight.getCurrentPosition());
                opmode.telemetry.update();

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            initRunWithoutEncoder();
            // Turn off RUN_TO_POSITION

        }
    }

        public void initRunWithEncoder()
        {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            rearRight.setDirection(DcMotor.Direction.FORWARD);
            rearLeft.setDirection(DcMotor.Direction.FORWARD);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            rearLeft.setDirection(DcMotor.Direction.REVERSE);
        }

        public void initRunWithoutEncoder ()
        {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            rearRight.setDirection(DcMotor.Direction.REVERSE);
            rearLeft.setDirection(DcMotor.Direction.FORWARD);
        }

    public void driveForwardTime(double speed,long time) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        rearLeft.setPower(speed);
        rearRight.setPower(speed);
        Thread.sleep(time);
        stopDriving();
    }



    public void driveForward(double speed)  {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        rearLeft.setPower(speed);
        rearRight.setPower(speed);
    }

    public void driveUntilTouch(double speed, LinearOpMode opmode) throws InterruptedException
    {
        driveForward(speed);
        while (sensorTouch.getState()==true&&!opmode.isStopRequested())
        {
            if (sensorTouch.getState() == true) {
                opmode.telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                opmode.telemetry.addData("Digital Touch", "Is Pressed");
            }
            opmode.telemetry.update();
        }
        stopDriving();
    }


   /* public void driveUntilColor(double speed, double colorValue, LinearOpMode opmode) throws InterruptedException {
        float hsvValues[]={300F,300F,300F};
        Color.RGBToHSV(sensorColor.red()*255, sensorColor.green()*255,sensorColor.blue()*255,hsvValues);

        driveForward(speed);
//&&hsvValues[0]>(colorValue-5.0)
        while (hsvValues[0]<(colorValue)&&!opmode.isStopRequested())
        {
            Color.RGBToHSV(sensorColor.red()*255, sensorColor.green()*255,sensorColor.blue()*255,hsvValues);
        }


        stopDriving();
    }
*/
    public void stopDriving()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }


    public void turnLeftTime(double speed,long time) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        rearLeft.setPower(-speed);
        rearRight.setPower(speed);
        Thread.sleep(time);
        stopDriving();


    }

    public void turnLeftAngle(double speed,int angleReading, LinearOpMode opmode) throws InterruptedException {

        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Thread.sleep(500);
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        rearLeft.setPower(-speed);
        rearRight.setPower(speed);
        while (angles.firstAngle < angleReading && !opmode.isStopRequested()){
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            opmode.telemetry.addData("Heading",angles.firstAngle);
            opmode.telemetry.update();
        }
        stopDriving();
    }




}
