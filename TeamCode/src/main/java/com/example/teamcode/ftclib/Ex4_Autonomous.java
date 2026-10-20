package com.example.teamcode.ftclib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.example.teamcode.ftclib.commands.KickCommand;
import com.example.teamcode.ftclib.commands.LowerLegCommand;
import com.example.teamcode.ftclib.subsystems.DriveSubsystem;
import com.example.teamcode.ftclib.subsystems.KickerSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * 예제 4: 자율주행(Autonomous)을 커맨드로 짜기
 *
 * [배우는 것]
 *   - 같은 서브시스템, 같은 커맨드를 TeleOp과 Autonomous에서 그대로 재사용
 *   - withTimeout(ms): 끝나지 않는 커맨드(RunCommand)를 정해진 시간만 실행
 *   - WaitUntilCommand: 조건이 true가 될 때까지 기다림
 *
 * [순서] START 후
 *   1) 앞으로 1초  2) 멈추고 0.5초 대기  3) 킥  4) 다리 내리기  5) 오른쪽으로 1초  6) 멈춤
 *
 * 처음 돌릴 땐 로봇 주변을 비우고, SPEED를 낮게 해서 확인할 것.
 */
@Autonomous(name = "FTCLib 4 - Autonomous", group = "FTCLib")
public class Ex4_Autonomous extends CommandOpMode {

    static final double SPEED = 0.4;

    @Override
    public void initialize() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap);

        SequentialCommandGroup auto = new SequentialCommandGroup(
                // initialize()는 START 전에 불려서, 그냥 두면 타이머가 INIT 때부터 돌아감.
                // START를 누를 때까지 먼저 기다림.
                new WaitUntilCommand(this::isStarted),

                new RunCommand(() -> drive.drive(0, 1, 0, SPEED), drive).withTimeout(1000),
                new InstantCommand(drive::stop, drive),
                new WaitCommand(500),

                new KickCommand(kicker, 1.0, 90.0, 0.5),
                new LowerLegCommand(kicker, 60.0, 4.0),

                new RunCommand(() -> drive.drive(1, 0, 0, SPEED), drive).withTimeout(1000),
                new InstantCommand(drive::stop, drive)
        );

        schedule(auto);

        telemetry.addLine("Ready. Press START to run the auto.");
        telemetry.update();
    }
}
