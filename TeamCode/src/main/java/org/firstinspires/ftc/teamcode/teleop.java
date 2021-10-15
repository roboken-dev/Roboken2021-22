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
    double SpeedControl = 1.0;

    public void runOpMode()
    {
        robot.init(hardwareMap,this);
        waitForStart();



        while (opModeIsActive()){

            robot.rearLeft.setPower(gamepad1.right_stick_y*SpeedControl);
            robot.frontLeft.setPower(gamepad1.right_stick_y*SpeedControl);
            robot.frontRight.setPower(gamepad1.left_stick_y*SpeedControl);
            robot.rearRight.setPower(gamepad1.left_stick_y*SpeedControl);

            robot.arm.setPower(gamepad2.right_stick_y*0.2);

            if(gamepad1.dpad_up) {
                SpeedControl = 1.0;
            }
            if(gamepad1.dpad_right){
                SpeedControl=0.6;
            }
            if(gamepad1.dpad_left){
                SpeedControl=0.6;
            }
            if(gamepad1.dpad_down){
                SpeedControl=0.3;
            }


            if (gamepad2.a)
            {
                robot.claw.setPosition(0.8);
            }
            if(gamepad2.b)
            {
                robot.claw.setPosition(0.2);
            }

}}}