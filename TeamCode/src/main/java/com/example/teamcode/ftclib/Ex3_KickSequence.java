package com.example.teamcode.ftclib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.example.teamcode.ftclib.commands.KickCommand;
import com.example.teamcode.ftclib.commands.LowerLegCommand;
import com.example.teamcode.ftclib.subsystems.DriveSubsystem;
import com.example.teamcode.ftclib.subsystems.KickerSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * 예제 3: 직접 만든 커맨드 + 순서대로 실행(SequentialCommandGroup)
 *
 * [배우는 것]
 *   - KickCommand, LowerLegCommand: CommandBase를 상속해서 직접 만든 커맨드
 *   - SequentialCommandGroup: 커맨드를 순서대로 하나씩 실행
 *   - WaitCommand: 기다리기만 하는 커맨드 → 쿨다운으로 사용
 *   - 서브시스템 두 개(주행, 킥)를 동시에 따로 돌리기
 *
 * [조작]
 *   왼쪽 스틱 = 이동, 오른쪽 스틱 좌우 = 회전
 *   X = 킥 → 천천히 내림 → 1.5초 쿨다운
 *
 * mecanum_drive.java에서는 KickState enum과 switch문으로 직접 짰던 순서를
 * 여기서는 SequentialCommandGroup 하나로 표현함.
 * 그룹이 도는 중(쿨다운 포함)에 X를 또 누르면 무시됨: 이미 실행 중인 커맨드는 다시 시작 안 함.
 * (차징 없이 바로 최대 파워로 차는 단순 버전)
 */
@TeleOp(name = "FTCLib 3 - Kick Sequence", group = "FTCLib")
public class Ex3_KickSequence extends CommandOpMode {

    static final double SPEED = 0.5;

    static final double KICK_POWER       = 1.0;
    static final double KICK_ANGLE_DEG   = 90.0;
    static final double KICK_MAX_SEC     = 0.5;
    static final double RETURN_DEG_S     = 60.0;
    static final double RETURN_MAX_SEC   = 4.0;
    static final long   KICK_COOLDOWN_MS = 1500;

    private KickerSubsystem kicker;

    @Override
    public void initialize() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        kicker = new KickerSubsystem(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1);

        drive.setDefaultCommand(new RunCommand(
                () -> drive.drive(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), SPEED),
                drive));

        // 킥 한 번 = 이 세 개를 순서대로
        SequentialCommandGroup kickSequence = new SequentialCommandGroup(
                new KickCommand(kicker, KICK_POWER, KICK_ANGLE_DEG, KICK_MAX_SEC),
                new LowerLegCommand(kicker, RETURN_DEG_S, RETURN_MAX_SEC),
                new WaitCommand(KICK_COOLDOWN_MS)
        );

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(kickSequence);

        telemetry.addLine("Ready. X = kick.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("kicker command",
                kicker.getCurrentCommand() == null ? "none" : kicker.getCurrentCommand().getName());
        telemetry.addData("leg angle", "%.0f deg", kicker.getAngleDeg());
        telemetry.update();
    }
}
