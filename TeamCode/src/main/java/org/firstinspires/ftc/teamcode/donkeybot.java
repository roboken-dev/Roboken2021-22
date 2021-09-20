package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class donkeybot {

    public DcMotor frontLeft;
    public DcMotor rearLeft;
    public DcMotor rearRight;
    public DcMotor frontRight;

    BNO055IMU imu;
    Orientation angles;

    private CRServo servo;


    public ColorSensor sensorColor;
    public DigitalChannel sensorTouch;


    HardwareMap hwMap = null;

    public donkeybot (){}


    public void init(HardwareMap ahwMap, LinearOpMode opmode){

        hwMap=ahwMap;

        frontLeft = hwMap.dcMotor.get("frontLeft");
        rearLeft = hwMap.dcMotor.get("rearLeft");
        frontRight = hwMap.dcMotor.get("frontRight");
        rearRight = hwMap.dcMotor.get("rearRight");

        servo = hwMap.crservo.get("servo");

        sensorColor = hwMap.get(ColorSensor.class,"colorSensor");
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


    public void driveForwardTime(double speed,long time) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        rearLeft.setPower(speed);
        rearRight.setPower(speed);
        Thread.sleep(time);
        stopDriving();
    }


    public void driveForward(double speed) throws InterruptedException {
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


    public void driveUntilColor(double speed, double colorValue, LinearOpMode opmode) throws InterruptedException {
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

    public void stopDriving(){
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
