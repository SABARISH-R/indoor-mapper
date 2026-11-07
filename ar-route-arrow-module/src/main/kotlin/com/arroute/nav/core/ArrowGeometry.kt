package com.arroute.nav.core

import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/** Flat arrow lying on the finished floor, in the AR SESSION frame (feed these to your OpenGL renderer). */
object ArrowGeometry {
    private val LOCAL = listOf(0.9 to 0.0, -0.3 to 0.45, -0.05 to 0.0, -0.3 to -0.45) // (forward, right), meters

    fun vertices(
        camX: Double, camZ: Double, fwdX: Double, fwdZ: Double,
        floorY: Double, relativeBearing: Double, distanceAhead: Double = 2.4,
    ): List<Vec3> {
        val n = hypot(fwdX, fwdZ)
        require(n > 1e-6) { "Camera looks straight up or down: no horizontal forward direction." }
        val fx = fwdX / n; val fz = fwdZ / n
        val cb = cos(relativeBearing); val sb = sin(relativeBearing)
        val ux = fx * cb - fz * sb; val uz = fx * sb + fz * cb   // arrow forward axis
        val vx = -uz; val vz = ux                                  // arrow right axis
        val bx = camX + fx * distanceAhead; val bz = camZ + fz * distanceAhead
        return LOCAL.map { (u, v) -> Vec3(bx + ux * u + vx * v, floorY, bz + uz * u + vz * v) }
    }
}
