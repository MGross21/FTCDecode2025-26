package pioneer.hardware.cache

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * A drop-in replacement for DcMotorEx that intercepts method calls to add caching for power and velocity.
 */
class CachedMotorEx(
    hardwareMap: HardwareMap,
    motorName: String,
) : DcMotorEx by hardwareMap.get(DcMotorEx::class.java, motorName) {
    // Cached values for power and velocity
    private var cachedPower: Double = 0.0
    private var cachedVelocity: Double = 0.0

    /**
     * Sets the motor power, updating the cache only if the value changes.
     */
    override fun setPower(power: Double) {
        if (power != cachedPower) {
            cachedPower = power
            (this as DcMotorEx).setPower(power)
        }
    }

    /**
     * Gets the motor velocity, updating the cache if the value changes.
     */
    override fun getVelocity(): Double {
        val currentVelocity = (this as DcMotorEx).getVelocity()
        if (currentVelocity != cachedVelocity) {
            cachedVelocity = currentVelocity
        }
        return cachedVelocity
    }

    /**
     * Clears the cached values, forcing the next set/get to update.
     */
    fun clearCache() {
        cachedPower = 0.0
        cachedVelocity = 0.0
    }
}
