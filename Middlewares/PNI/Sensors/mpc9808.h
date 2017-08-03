/**
* @file         mpc9808.h
*
* @brief        PNI mpc9808 functions
*
* @date         02/03/2017
*
* @copyright    (C) 2017 PNI Corp
*
*               THIS SOFTWARE IS PROVIDED BY PNI SENSOR CORPORATION "AS IS" AND
*               ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
*               TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
*               PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PNI SENSOR
*               CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*               SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
*               NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*               LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*               HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*               CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
*               OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
*               EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*               DISCLOSURE TO THIRD PARTIES OR REPRODUCTION IN ANY FORM
*               WHATSOEVER, WITHOUT PRIOR WRITTEN CONSENT, IS STRICTLY
*               FORBIDDEN.
*
*/

#ifndef PNI_MPC9808_H
#define PNI_MPC9808_H

#include "main.h"
#include "i2c.h"

#define MPC9808_ADDRESS 0x30

#define MPC9808_I2C_HANDLER                   hi2c1

// MPC9808 registers
#define MPC9808_REG_TEMPERATURE               0x05    // 00000101 Device ID/Revision Register
#define MPC9808_REG_DEVICE_ID                 0x07    // 00000111 Device ID/Revision Register

// ERRORS
#define MPC9808_OK                            0
#define MPC9808_ERROR                         1


uint8_t Mpc9808_ready();
uint8_t Mpc9808_GetIdRev(uint8_t* id, uint8_t* revision);
uint8_t Mpc9808_GetTemperature(int8_t* temperature);

#endif
