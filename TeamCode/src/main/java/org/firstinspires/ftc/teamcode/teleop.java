package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
@TeleOp(name="teleop")

public class teleop extends LinearOpMode{
    donkeybot robot = new donkeybot();

    float hsvValues[]={120F,300F,300F};


    public void runOpMode()
    {
        robot.init(hardwareMap,this);
        waitForStart();

        while (opModeIsActive()){

           robot.rearLeft.setPower(gamepad1.right_stick_y);
          robot.frontLeft.setPower(gamepad1.right_stick_y);
           robot.frontRight.setPower(gamepad1.left_stick_y);
           robot.rearRight.setPower(gamepad1.left_stick_y);
/*
           // Color.RGBToHSV(robot.sensorColor.red()*255, robot.sensorColor.green()*255,robot.sensorColor.blue()*255,hsvValues);

                if (robot.sensorTouch.getState() == true) {
                    telemetry.addData("Digital Touch", "Is Not Pressed");
                 } else {
                    telemetry.addData("Digital Touch", "Is Pressed");
                }
                telemetry.update();



            telemetry.addData("hue",hsvValues[0]);
            telemetry.update();
*/
        }


    }

}