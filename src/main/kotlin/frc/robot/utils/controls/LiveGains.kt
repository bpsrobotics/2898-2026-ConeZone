package frc.robot.utils.controls

/** Marks source defaults that saveElevatorGains can refresh from Sendable NetworkTables values. */
@Target(AnnotationTarget.PROPERTY)
@Retention(AnnotationRetention.SOURCE)
annotation class LiveGains(val topic: String, val properties: String)
