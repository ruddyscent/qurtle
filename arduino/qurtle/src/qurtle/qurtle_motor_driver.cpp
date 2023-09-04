/*******************************************************************************
 * Copyright 2023 Kyungwon Chun
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "../../include/qurtle/qurtle_motor_driver.h"

// Limit values (XM430-W210-T and XM430-W350-T)
// MAX RPM is 77 when DXL is powered 12.0V
// 77 / 0.229 (RPM) = 336.24454...
const uint16_t LIMIT_X_MAX_VELOCITY = 337;
// V = r * w = r     *        (RPM             * 0.10472)
//           = 0.033 * (0.229 * Goal_Velocity) * 0.10472
// Goal_Velocity = V * 1263.632956882
const float VELOCITY_CONSTANT_VALUE = 1263.632956882;

/* DYNAMIXEL Information for controlling motors and  */
const uint8_t DXL_MOTOR_ID_FRONT_LEFT = 1;   // ID of left motor
const uint8_t DXL_MOTOR_ID_FRONT_RIGHT = 2;  // ID of right motor
const uint8_t DXL_MOTOR_ID_REAR_LEFT = 3;    // ID of left motor
const uint8_t DXL_MOTOR_ID_REAR_RIGHT = 4;   // ID of right motor
const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000;  // baurd rate of Dynamixel
const int OPENCR_DXL_DIR_PIN = 84;           // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);

QurtleMotorDriver::QurtleMotorDriver()
    : front_left_wheel_id_(DXL_MOTOR_ID_FRONT_LEFT),
      front_right_wheel_id_(DXL_MOTOR_ID_FRONT_RIGHT),
      rear_left_wheel_id_(DXL_MOTOR_ID_REAR_LEFT),
      rear_right_wheel_id_(DXL_MOTOR_ID_REAR_RIGHT),
      torque_(false)
{
}

QurtleMotorDriver::~QurtleMotorDriver()
{
    close();
    digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool QurtleMotorDriver::init(void)
{
    pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
    drv_dxl_init();

    dxl.begin(DXL_PORT_BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

    sync_write_param.id_count = 4;
    sync_write_param.xel[FRONT_LEFT].id = front_left_wheel_id_;
    sync_write_param.xel[FRONT_RIGHT].id = front_right_wheel_id_;
    sync_write_param.xel[REAR_LEFT].id = rear_left_wheel_id_;
    sync_write_param.xel[REAR_RIGHT].id = rear_right_wheel_id_;

    sync_read_param.addr = 132;
    sync_read_param.length = 4;
    sync_read_param.id_count = 4;
    sync_read_param.xel[FRONT_LEFT].id = front_left_wheel_id_;
    sync_read_param.xel[FRONT_RIGHT].id = front_right_wheel_id_;
    sync_read_param.xel[REAR_LEFT].id = rear_left_wheel_id_;
    sync_read_param.xel[REAR_RIGHT].id = rear_right_wheel_id_;

    // Enable Dynamixel Torque
    set_torque(true);

    return true;
}

Dynamixel2Arduino &QurtleMotorDriver::getDxl()
{
    return dxl;
}

bool QurtleMotorDriver::is_connected()
{
    return (
        dxl.ping(DXL_MOTOR_ID_FRONT_LEFT) == true &&
        dxl.ping(DXL_MOTOR_ID_FRONT_RIGHT) == true &&
        dxl.ping(DXL_MOTOR_ID_REAR_LEFT) == true &&
        dxl.ping(DXL_MOTOR_ID_REAR_RIGHT) == true);
}

bool QurtleMotorDriver::set_torque(bool onoff)
{
    bool ret = false;

    sync_write_param.addr = 64;
    sync_write_param.length = 1;
    sync_write_param.xel[FRONT_LEFT].data[0] = onoff;
    sync_write_param.xel[FRONT_RIGHT].data[0] = onoff;
    sync_write_param.xel[REAR_LEFT].data[0] = onoff;
    sync_write_param.xel[REAR_RIGHT].data[0] = onoff;

    if (dxl.syncWrite(sync_write_param) == true)
    {
        ret = true;
        torque_ = onoff;
    }

    return ret;
}

bool QurtleMotorDriver::get_torque()
{
    if (
        dxl.readControlTableItem(TORQUE_ENABLE, front_left_wheel_id_) == true &&
        dxl.readControlTableItem(TORQUE_ENABLE, front_right_wheel_id_) == true &&
        dxl.readControlTableItem(TORQUE_ENABLE, rear_left_wheel_id_) == true &&
        dxl.readControlTableItem(TORQUE_ENABLE, rear_right_wheel_id_) == true)
    {
        torque_ = true;
    }
    else
    {
        torque_ = false;
    }

    return torque_;
}

void QurtleMotorDriver::close(void)
{
    // Disable Dynamixel Torque
    set_torque(false);
}

bool QurtleMotorDriver::read_present_position(
    int32_t &front_left_value, int32_t &front_right_value,
    int32_t &rear_left_value, int32_t &rear_right_value)
{
    bool ret = false;

    sync_read_param.addr = 132;
    sync_read_param.length = 4;

    if (dxl.syncRead(sync_read_param, read_result))
    {
        memcpy(&front_left_value, read_result.xel[FRONT_LEFT].data, read_result.xel[FRONT_LEFT].length);
        memcpy(&front_right_value, read_result.xel[FRONT_RIGHT].data, read_result.xel[FRONT_RIGHT].length);
        memcpy(&rear_left_value, read_result.xel[REAR_LEFT].data, read_result.xel[REAR_LEFT].length);
        memcpy(&rear_right_value, read_result.xel[REAR_RIGHT].data, read_result.xel[REAR_RIGHT].length);
        ret = true;
    }

    return ret;
}

bool QurtleMotorDriver::read_present_velocity(
    int32_t &front_left_value, int32_t &front_right_value,
    int32_t &rear_left_value, int32_t &rear_right_value)
{
    bool ret = false;

    sync_read_param.addr = 128;
    sync_read_param.length = 4;

    if (dxl.syncRead(sync_read_param, read_result))
    {
        memcpy(&front_left_value, read_result.xel[FRONT_LEFT].data, read_result.xel[FRONT_LEFT].length);
        memcpy(&front_right_value, read_result.xel[FRONT_RIGHT].data, read_result.xel[FRONT_RIGHT].length);
        memcpy(&rear_left_value, read_result.xel[REAR_LEFT].data, read_result.xel[REAR_LEFT].length);
        memcpy(&rear_right_value, read_result.xel[REAR_RIGHT].data, read_result.xel[REAR_RIGHT].length);
        ret = true;
    }

    return ret;
}

bool QurtleMotorDriver::read_present_current(
    int16_t &front_left_value, int16_t &front_right_value,
    int16_t &rear_left_value, int16_t &rear_right_value)
{
    bool ret = false;

    sync_read_param.addr = 126;
    sync_read_param.length = 2;

    if (dxl.syncRead(sync_read_param, read_result))
    {
        memcpy(&front_left_value, read_result.xel[FRONT_LEFT].data, read_result.xel[FRONT_LEFT].length);
        memcpy(&front_right_value, read_result.xel[FRONT_RIGHT].data, read_result.xel[FRONT_RIGHT].length);
        memcpy(&rear_left_value, read_result.xel[REAR_LEFT].data, read_result.xel[REAR_LEFT].length);
        memcpy(&rear_right_value, read_result.xel[REAR_RIGHT].data, read_result.xel[REAR_RIGHT].length);
        ret = true;
    }

    return ret;
}

bool QurtleMotorDriver::read_profile_acceleration(
    uint32_t &front_left_value, uint32_t &front_right_value,
    uint32_t &rear_left_value, uint32_t &rear_right_value)
{
    bool ret = false;

    sync_read_param.addr = 108;
    sync_read_param.length = 4;

    if (dxl.syncRead(sync_read_param, read_result))
    {
        memcpy(&front_left_value, read_result.xel[FRONT_LEFT].data, read_result.xel[FRONT_LEFT].length);
        memcpy(&front_right_value, read_result.xel[FRONT_RIGHT].data, read_result.xel[FRONT_RIGHT].length);
        memcpy(&rear_left_value, read_result.xel[REAR_LEFT].data, read_result.xel[REAR_LEFT].length);
        memcpy(&rear_right_value, read_result.xel[REAR_RIGHT].data, read_result.xel[REAR_RIGHT].length);
        ret = true;
    }

    return ret;
}

bool QurtleMotorDriver::write_velocity(
    int32_t front_left_value, int32_t front_right_value,
    int32_t rear_left_value, int32_t rear_right_value)
{
    bool ret = false;

    sync_write_param.addr = 104;
    sync_write_param.length = 4;
    memcpy(sync_write_param.xel[FRONT_LEFT].data, &front_left_value, sync_write_param.length);
    memcpy(sync_write_param.xel[FRONT_RIGHT].data, &front_right_value, sync_write_param.length);
    memcpy(sync_write_param.xel[REAR_LEFT].data, &rear_left_value, sync_write_param.length);
    memcpy(sync_write_param.xel[REAR_RIGHT].data, &rear_right_value, sync_write_param.length);

    if (dxl.syncWrite(sync_write_param))
    {
        ret = true;
    }

    return ret;
}

bool QurtleMotorDriver::write_profile_acceleration(
    uint32_t front_left_value, uint32_t front_right_value,
    uint32_t rear_left_value, uint32_t rear_right_value)
{
    bool ret = false;

    sync_write_param.addr = 108;
    sync_write_param.length = 4;
    memcpy(sync_write_param.xel[FRONT_LEFT].data, &front_left_value, sync_write_param.length);
    memcpy(sync_write_param.xel[FRONT_RIGHT].data, &front_right_value, sync_write_param.length);
    memcpy(sync_write_param.xel[REAR_LEFT].data, &rear_left_value, sync_write_param.length);
    memcpy(sync_write_param.xel[REAR_RIGHT].data, &rear_right_value, sync_write_param.length);

    if (dxl.syncWrite(sync_write_param))
    {
        ret = true;
    }

    return ret;
}

bool QurtleMotorDriver::control_motors(const float wheel_separation, float linear_value, float angular_value)
{
    bool dxl_comm_result = false;

    float wheel_velocity[MortorLocation::MOTOR_NUM_MAX];
    float lin_vel = linear_value;
    float ang_vel = angular_value;

    wheel_velocity[FRONT_LEFT] = lin_vel - (ang_vel * wheel_separation / 2);
    wheel_velocity[REAR_LEFT] = lin_vel - (ang_vel * wheel_separation / 2);
    wheel_velocity[FRONT_RIGHT] = lin_vel + (ang_vel * wheel_separation / 2);
    wheel_velocity[REAR_RIGHT] = lin_vel + (ang_vel * wheel_separation / 2);

    wheel_velocity[FRONT_LEFT] = constrain(wheel_velocity[FRONT_LEFT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
    wheel_velocity[REAR_LEFT] = constrain(wheel_velocity[REAR_LEFT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
    wheel_velocity[FRONT_RIGHT] = constrain(wheel_velocity[FRONT_RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
    wheel_velocity[REAR_RIGHT] = constrain(wheel_velocity[REAR_RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

    dxl_comm_result = write_velocity(
        (int32_t)wheel_velocity[FRONT_LEFT], (int32_t)wheel_velocity[FRONT_RIGHT],
        (int32_t)wheel_velocity[REAR_LEFT], (int32_t)wheel_velocity[REAR_RIGHT]);
    if (dxl_comm_result == false)
        return false;

    return true;
}
