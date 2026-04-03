
/*!
 *  @file Adafruit_LSM6DSOX.cpp
 *  Adafruit LSM6DSOX 6-DoF Accelerometer and Gyroscope library
 *
 *  Bryan Siepert for Adafruit Industries
 * 	BSD (see license.txt)
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_LSM6DSOX.h"

/*!
 *    @brief  Instantiates a new LSM6DSOX class
 */
Adafruit_LSM6DSOX::Adafruit_LSM6DSOX(void) {}

bool Adafruit_LSM6DSOX::_init(int32_t sensor_id) {
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_WHOAMI);

  // make sure we're talking to the right chip
  if (chip_id.read() != LSM6DSOX_CHIP_ID) {
    return false;
  }
  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;

  reset();

  // Block Data Update
  // this prevents MSB/LSB data registers from being updated until both are read
  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL3_C);
  Adafruit_BusIO_RegisterBits bdu = Adafruit_BusIO_RegisterBits(&ctrl3, 1, 6);
  bdu.write(true);

  // Disable I3C
  Adafruit_BusIO_Register ctrl_9 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL9_XL);
  Adafruit_BusIO_RegisterBits i3c_disable_bit =
      Adafruit_BusIO_RegisterBits(&ctrl_9, 1, 1);

  i3c_disable_bit.write(true);

  // call base class _init()
  Adafruit_LSM6DS::_init(sensor_id);

  return true;
}

/**************************************************************************/
/*!
    @brief Disables and enables the SPI master bus pulllups.
    @param disable_pullups true to **disable** the I2C pullups, false to enable.
*/
void Adafruit_LSM6DSOX::disableSPIMasterPullups(bool disable_pullups) {

  Adafruit_BusIO_Register pin_config = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_PIN_CTRL);

  Adafruit_BusIO_RegisterBits disable_ois_pu =
      Adafruit_BusIO_RegisterBits(&pin_config, 1, 7);

  disable_ois_pu.write(disable_pullups);
}

/**************************************************************************/
/*!
    @brief Enables and disables the I2C master bus pulllups.
    @param enable_pullups true to enable the I2C pullups, false to disable.
*/
void Adafruit_LSM6DSOX::enableI2CMasterPullups(bool enable_pullups) {

  Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_FUNC_CFG_ACCESS);
  Adafruit_BusIO_RegisterBits master_cfg_enable_bit =
      Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 6);

  Adafruit_BusIO_Register master_config = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_MASTER_CONFIG);
  Adafruit_BusIO_RegisterBits i2c_master_pu_en =
      Adafruit_BusIO_RegisterBits(&master_config, 1, 3);

  master_cfg_enable_bit.write(true);
  i2c_master_pu_en.write(enable_pullups);
  master_cfg_enable_bit.write(false);
}

void Adafruit_LSM6DSOX::enableTilt()
{
  Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_FUNC_CFG_ACCESS);
  Adafruit_BusIO_RegisterBits shub_access_bit =
      Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 6);
  Adafruit_BusIO_RegisterBits func_cfg_bit =
      Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 7);
  Adafruit_BusIO_Register emb_fun_en = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_EMB_FUNC_EN_A);
  Adafruit_BusIO_RegisterBits tilt_en =
      Adafruit_BusIO_RegisterBits(&emb_fun_en, 1, 4);
    
  shub_access_bit.write(true);
  func_cfg_bit.write(true);
  if(!tilt_en.write(true))
  {
    Serial.println("Failed to enable tilt!!!");
  }
  shub_access_bit.write(false);
  func_cfg_bit.write(false);
}

bool Adafruit_LSM6DSOX::tilt()
{
  Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_FUNC_CFG_ACCESS);
  Adafruit_BusIO_RegisterBits shub_access_bit =
      Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 6);
  Adafruit_BusIO_RegisterBits func_cfg_bit =
      Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 7);
  Adafruit_BusIO_Register emb_fun_status = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_EMB_FUNC_STATUS);
  Adafruit_BusIO_RegisterBits tilt =
      Adafruit_BusIO_RegisterBits(&emb_fun_status, 1, 4);
  shub_access_bit.write(true);
  func_cfg_bit.write(true);
  bool ret = tilt.read();
  shub_access_bit.write(false);
  func_cfg_bit.write(false);

  return ret;
}

void attachTiltInt(int int_no)
{
    Adafruit_BusIO_Register mdcfg;
    if(int_no == 1)
    {
    mdcfg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_MD1_CFG);
    }
    else if(int_no == 2)
    {
    mdcfg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_MD2_CFG); 
    }

  Adafruit_BusIO_RegisterBits emb_int = Adafruit_BusIO_RegisterBits(&mdcfg, 1, 1);
  emb_int.write(true);

  Adafruit_BusIO_Register emb_fuc_int = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, EMB_FUNC_INIT_A);
  Adafruit_BusIO_RegisterBits tilt =
      Adafruit_BusIO_RegisterBits(&emb_fuc_int, 1, 4);
    tilt.write(true);
}
