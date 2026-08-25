package com.unidroids.robot_hmi.ui.navigation

import androidx.navigation3.runtime.NavKey as BaseNavKey
import kotlinx.serialization.Serializable

@Serializable
sealed interface NavKey : BaseNavKey {
    @Serializable
    data object Dashboard : NavKey

    @Serializable
    data object Scanner : NavKey
}
