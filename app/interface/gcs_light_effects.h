/*
 * Crazyflie GCS-controlled light effect evaluation module
 *
 * This file is part of the Skybrush compatibility layer for the Crazyflie firmware.
 *
 * Copyright 2021-2022 CollMot Robotics Ltd.
 *
 * This app is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This app is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __GCS_LIGHT_EFFECTS_H__
#define __GCS_LIGHT_EFFECTS_H__

#include <stdbool.h>
#include <stdint.h>

/** Light effect types that the user can trigger from the GCS */
typedef enum {
    GCS_LIGHT_EFFECT_OFF = 0,
    GCS_LIGHT_EFFECT_SOLID = 1
} gcs_light_effect_type_t;

/* Public functions */

/**
 * Initializes the GCS light effects module.
 */
void gcsLightEffectsInit(void);

/**
 * Tests whether the GCS light effects module is ready to be used.
 */
bool gcsLightEffectsTest(void);

/**
 * Evaluates the LED light color dictated by the current GCS light effect
 * at the current timestamp.
 *
 * \param  color      pointer to a memory location where the evaluated RGB
 *         color should be written (3 bytes)
 */
void gcsLightEffectEvaluate(uint8_t* color);

/**
 * Clears the current GCS light effect, returning the LED ring to its normal
 * operation mode.
 */
void gcsLightEffectDisable();

/**
 * Triggers a light effect.
 * 
 * \param  effect the effect to trigger
 * \param  color  pointer to a memory location where the desired color of the
 *                light effect is written in RGB notation; NULL to use the
 *                color of the last active effect
 * \return  zero if the light effect was triggered successfully, an error code
 *          otherwise
 */
int gcsLightEffectTrigger(gcs_light_effect_type_t effect, const uint8_t* color);

/**
 * Returns whether there is an activated light effect that should override the
 * color of the LED ring.
 */
bool areGcsLightEffectsActive(void);

#endif /* __GCS_LIGHT_EFFECTS_H__ */
