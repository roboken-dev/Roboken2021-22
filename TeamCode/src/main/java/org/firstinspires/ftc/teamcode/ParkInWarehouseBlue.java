package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ParkInWarehouseBlue")
public class ParkInWarehouseBlue extends LinearOpMode {
    donkeybot robot = new donkeybot();

        public void runOpMode() throws InterruptedException
        {
            robot.init(hardwareMap,this);

            waitForStart();

            robot.encoderDrive(0.2,36,36,10,this);
            robot.turnLeftAngle(0.2,90,this);
            robot.encoderDrive(0.8,50,50,20,this);


}}
