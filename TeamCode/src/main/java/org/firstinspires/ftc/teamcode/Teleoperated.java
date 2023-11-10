package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name="name") // todo: change name
public class Teleoperated extends LinearOpMode {

    HardwareMap hwMap;

    DriveTrain driveTrain = new DriveTrain();

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain.initialize(hwMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveTrain.set(new Vector(0, 0), 0, true);
        }

    }
}
