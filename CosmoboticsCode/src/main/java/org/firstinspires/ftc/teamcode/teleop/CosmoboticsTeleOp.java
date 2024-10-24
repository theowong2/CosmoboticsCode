package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;

@Config
@TeleOp
public class CosmoboticsTeleOp extends OpMode {
    Drive drive;
    Transport transport;
    Misc misc;
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        transport = new Transport(hardwareMap);
        misc = new Misc(hardwareMap);
    }

    @Override
    public void loop() {
        drive.update(gamepad1);
        transport.update(gamepad1, gamepad2);
        misc.update(gamepad2);
        telemetry.update();
    }
}
