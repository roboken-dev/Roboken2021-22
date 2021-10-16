package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DuckParkStorageRed" +
        "" +
        "")
public class DuckParkStorageRed extends LinearOpMode {

    donkeybot robot = new donkeybot();


    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap,this);

        waitForStart();

        robot.encoderDrive(0.3, 16, 16, 8, this);

        robot.turnLeftAngle(0.4,90,this);
        robot.encoderDrive(0.3,8,8,6,this);

        robot.spin.setPower(-1);
        sleep(4000);
        robot.spin.setPower(0);

        robot.encoderDrive(0.3,-22, -22, 11, this);
    }



}
