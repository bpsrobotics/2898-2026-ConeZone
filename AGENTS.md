Welcome to FRC Team 2898, The Flying Hedgehogs' robot code.

- Language: We mainly use Kotlin because of its null safety advantages over Java, extension functions, full
  cross-compatibility with Java, and easy singletons with the `object` keyword. We don't use C++ because memory
  management is harder and it won't/doesn't have coroutine support in WPILib 2027.
- Utilities: We keep all our utilities in-repo, without submodules. We used to use Beaverlib, but push permissions to
  that repository were inconsistent. If WPILib has a utility for it, use that, else make your own.
- Units: Use WPILib Units (edu.wpi.first.units) for **every** constant. Values should stay as Measure instances for as
  long as possible. We have extension functions on Number in [
  `utils/Units.kt`](./src/main/kotlin/frc/robot/utils/Units.kt).
- Put all "constants" in NetworkTables so they can be easily tested/reconfigured without deploying.
- Style: Use Kotlinlang styles (run ktfmt) and channel YAGNI, DRY and KISS principles.
- Testing: Run `./gradlew build` to check your changes and nothing more unless the user requests otherwise.