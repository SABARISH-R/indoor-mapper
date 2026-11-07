package com.arroute.nav.core

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Conventions (shared by the plan frame and the ARCore session frame):
 *  - Right-handed, y up, horizontal coordinates are (x, z). Units are meters.
 *  - Heading `yaw` is an angle in radians with forward = (cos yaw, sin yaw) in (x, z).
 *    Positive yaw turns from +x toward +z, which is a right turn when the plan is drawn with z down.
 *  UNVERIFIED: that the editor's plan frame is right-handed with the same axis order. A mirrored
 *  plan would flip every left/right result. Check this on the phone (see README, check 1).
 */
data class Vec2(val x: Double, val z: Double) {
    operator fun plus(o: Vec2) = Vec2(x + o.x, z + o.z)
    operator fun minus(o: Vec2) = Vec2(x - o.x, z - o.z)
    operator fun times(k: Double) = Vec2(x * k, z * k)
    fun length() = hypot(x, z)
    fun dot(o: Vec2) = x * o.x + z * o.z
}

data class Vec3(val x: Double, val y: Double, val z: Double)

fun normalizeAngle(a: Double): Double = atan2(sin(a), cos(a))
fun headingOf(fx: Double, fz: Double): Double = atan2(fz, fx)
fun rotate(v: Vec2, a: Double) = Vec2(v.x * cos(a) - v.z * sin(a), v.x * sin(a) + v.z * cos(a))

data class PlanPose(val position: Vec2, val yaw: Double, val floorId: String, val mapVersion: String)

/** Rigid 2D transform (rotation then translation). Never scales: AR meters stay meters. */
class Rigid2D(val rotation: Double, val translation: Vec2) {
    fun apply(p: Vec2): Vec2 = rotate(p, rotation) + translation
    fun applyYaw(yaw: Double): Double = normalizeAngle(yaw + rotation)

    companion object {
        /** Session-to-plan transform from one marker seen in the session and surveyed in the plan. */
        fun fromMarker(sessionPos: Vec2, sessionRefYaw: Double, planPos: Vec2, planRefYaw: Double): Rigid2D {
            val r = normalizeAngle(planRefYaw - sessionRefYaw)
            return Rigid2D(r, planPos - rotate(sessionPos, r))
        }
    }
}
