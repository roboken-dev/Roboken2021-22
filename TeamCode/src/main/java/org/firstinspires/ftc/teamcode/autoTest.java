package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="auto")

public class autoTest extends LinearOpMode{
    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private DcMotor frontRight;


    BNO055IMU imu;
    Orientation angles;

    private CRServo servo;
    @Override


    public void runOpMode()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        servo = hardwareMap.crservo.get("servo");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


waitForStart();

turnLeftAngle(0.1,90);







    }

    public void driveForward(double speed,long time){
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        rearLeft.setPower(speed);
        rearRight.setPower(speed);
        sleep(time);
        stopDriving();


    }


    public void stopDriving(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }


    public void turnLeftTime(double speed,long time){
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        rearLeft.setPower(-speed);
        rearRight.setPower(speed);
        sleep(time);
        stopDriving();


    }
    public void turnLeftAngle(double speed,int angleReading){
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        sleep(500);
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        rearLeft.setPower(-speed);
        rearRight.setPower(speed);
        while (angles.firstAngle < angleReading && !isStopRequested()){
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading",angles.firstAngle);
            telemetry.update();
        }
        stopDriving();


    }



}



