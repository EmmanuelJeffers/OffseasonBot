package frc.robot.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public enum SuperstructureSate {
    START(SuperstructureSateData.builder().build()),
    AUTO_START(SuperstructureSateData.builder().build()),
    CHARACTERIZATION(SuperstructureSateData.builder().build()),
    SAFTEY(SuperstructureSateData.builder().build()),
    STOW(SuperstructureSateData.builder().build()),
    CORAL_INTAKE(SuperstructureSateData.builder().build()),
    GOODBYE_CORAL(SuperstructureSateData.builder().build()),
    L1_CORAL(SuperstructureSateData.builder().build()),
    L2_CORAL(SuperstructureSateData.builder().build()),
    L3_CORAL(SuperstructureSateData.builder().build()),
    L4_CORAL(SuperstructureSateData.builder().build()),
    L1_CORAL_EJECT(SuperstructureSateData.builder().build()),
    L2_CORAL_EJECT(SuperstructureSateData.builder().build()),
    L3_CORAL_EJECT(SuperstructureSateData.builder().build()),
    L4_CORAL_EJECT(SuperstructureSateData.builder().build()),
    ALGAE_STOW(SuperstructureSateData.builder().build()),
    ALGAE_L2_INTAKE(SuperstructureSateData.builder().build()),
    ALGAE_L3_INTAKE(SuperstructureSateData.builder().build()),
    PRE_THROW(SuperstructureSateData.builder().build()),
    THROW(SuperstructureSateData.builder().build()),
    TOSS(SuperstructureSateData.builder().build()),
    PRE_PROCESS(SuperstructureSateData.builder().build()),
    PROCESS(SuperstructureSateData.builder().build());

    private final SuperstructureSateData value;
}
