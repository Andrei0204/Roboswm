package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static com.pedropathing.ivy.commands.Commands.branch;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.function.BooleanSupplier;

public class IntakeSystem {
    public final Intake intake;
    public final Transfer transfer;
    public final Tavalug tavalug;

    private double intakestate = 0;
    private final Command applyMode;

    public IntakeSystem(HardwareMap hardwareMap) {
        this(new Intake(hardwareMap), new Transfer(hardwareMap), new Tavalug(hardwareMap));
    }

    public IntakeSystem(Intake intake, Transfer transfer, Tavalug tavalug) {
        this.intake = intake;
        this.transfer = transfer;
        this.tavalug = tavalug;

        LinkedHashMap<BooleanSupplier, Command> cases = new LinkedHashMap<>();
        cases.put(() -> intakestate == 0, mode(0, 0, Tavalug.Mode.COLLECT_ALL));
        cases.put(() -> intakestate == 1, mode(1, 1, Tavalug.Mode.COLLECT_ALL));
        applyMode = branch(cases);
    }

    private Command mode(double intakePower, double transferPower, Tavalug.Mode tav) {
        return instant(() -> {
            intake.set(intakePower);
            transfer.set(transferPower);
            tavalug.setMode(tav);
        });
    }

    public void setIntakeOn(double intakestate) {
        this.intakestate = intakestate;
    }

    public double getIntakeOn() {
        return intakestate;
    }

    /** Selects a mode (does not touch hardware until update() runs). */
    public Command setIn(double intakeOn) {
        return instant(() -> setIntakeOn(intakeOn));
    }

    /** Applies the currently selected mode to the hardware. */
    public Command update() {
        return applyMode;
    }

    /** Select a mode and apply it. */
    public CommandBuilder run(double intakestate) {
        return sequential(setIn(intakestate), update());
    }
}
