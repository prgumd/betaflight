/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RX_MSP

#include "common/utils.h"

#include "drivers/io.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "rx/msp.h"

#include "fc/rc_controls.h"

static uint16_t mspFrame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static bool rxMspFrameDone = false;

static uint16_t rxMspReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return mspFrame[chan];
}

/*
 * Called from MSP command handler - mspFcProcessCommand
 */
void rxMspFrameReceive(uint16_t *frame, int channelCount)
{
    for (int i = 0; i < channelCount; i++) {
        mspFrame[i] = frame[i];
    }

    // Any channels not provided will be reset to zero
    for (int i = channelCount; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        mspFrame[i] = 0;
    }

    rxMspFrameDone = true;
}

static uint8_t rxMspFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (!rxMspFrameDone) {
        return RX_FRAME_PENDING;
    }

    rxMspFrameDone = false;
    return RX_FRAME_COMPLETE;
}

void rxMspInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT;
    rxRuntimeConfig->rxRefreshRate = 20000;

    rxRuntimeConfig->rcReadRawFn = rxMspReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxMspFrameStatus;
}

void rxMspChannelsReset()
{
    // Also clear MSP channels to make sure old data doesn't sneak in
    mspFrame[ROLL]  = 1500;
    mspFrame[PITCH]  = 1500;

    // It was found through testing that THROTTLE was actually YAW and YAW was THROTTLE
    // It appears that items in these arrays aren't in the same order as in rx.c
    mspFrame[THROTTLE]  = 1500;
    mspFrame[YAW] = 1000;
    for(uint8_t channel = AUX1; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++)
    {
        mspFrame[channel] = 1000;
    }
}

#endif
