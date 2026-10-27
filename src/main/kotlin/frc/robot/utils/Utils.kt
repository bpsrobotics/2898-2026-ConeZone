package frc.robot.utils

import edu.wpi.first.wpilibj.DriverStation
import kotlin.jvm.optionals.getOrNull

fun DriverStation.Alliance.opposite(): DriverStation.Alliance =
    when (this) {
        DriverStation.Alliance.Red -> DriverStation.Alliance.Blue
        DriverStation.Alliance.Blue -> DriverStation.Alliance.Red
    }

/** [DriverStation.getAlliance] except with a standard fallback */
fun getAlliance(): DriverStation.Alliance =
    DriverStation.getAlliance().getOrNull() ?: DriverStation.Alliance.Red
