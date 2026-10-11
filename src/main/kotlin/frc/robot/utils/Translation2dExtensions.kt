package frc.robot.utils

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle

/**
 * Direction of the vector pointing from [other] to this translation.
 *
 * Preserves the semantics of the removed `Vector2.angleTo`.
 */
fun Translation2d.angleTo(other: Translation2d): Angle =
    Units.Radians.of((this - other).angle.radians)

/** Unit vector pointing in the same direction as this translation. */
val Translation2d.unit: Translation2d
    get() = div(norm)

/**
 * Builds a translation from polar coordinates.
 *
 * Mirrors the removed `Vector2.new(angle, magnitude)` factory.
 */
fun polarTranslation(magnitude: Double, angle: Angle): Translation2d =
    Translation2d(magnitude, angle.rotation2d)
