uint8_t Decode(float* value, uint8_t header)
{
  uint8_t tmp_pointer = Decode_pointer;
  uint8_t tmp_header[2] = {0};
  uint8_t tmp_data_pointer[3] = {0};
  uint8_t OK = 0;
  for (int8_t i = 0; i < 2 ; i++)
  {
    tmp_header[i] = RX_buffer[tmp_pointer];
    tmp_pointer++;
    if (tmp_pointer >= RX_buffer_size)tmp_pointer = 0;
  }
  if ((tmp_header[0] == header && tmp_header[1] == header))
  {
    tmp_data_pointer[0] = (uint8_t)RX_buffer[tmp_pointer + 0];
    tmp_data_pointer[1] = (uint8_t)RX_buffer[tmp_pointer + 1];
    tmp_data_pointer[2] = (uint8_t)RX_buffer[tmp_pointer + 2];
    if ((uint8_t)tmp_data_pointer[0] + (uint8_t)tmp_data_pointer[1] == (uint8_t)tmp_data_pointer[2])
    {
      uint16_t tmp = (uint16_t)tmp_data_pointer[0] << 8 | (uint16_t)tmp_data_pointer[1];
      * value = (float)(tmp) * 0.1f;
      OK = 1;
    }
  }
  return OK;
}
uint8_t Decode_press(uint16_t* value, uint8_t header)
{
  uint8_t tmp_pointer = Decode_pointer;
  uint8_t tmp_header[2] = {0};
  uint8_t tmp_data_pointer[3] = {0};
  uint8_t OK = 0;
  for (int8_t i = 0; i < 2 ; i++)
  {
    tmp_header[i] = RX_buffer[tmp_pointer];
    tmp_pointer++;
    if (tmp_pointer >= RX_buffer_size)tmp_pointer = 0;
  }
  if ((tmp_header[0] == header && tmp_header[1] == header))
  {
    tmp_data_pointer[0] = (uint8_t)RX_buffer[tmp_pointer + 0];
    tmp_data_pointer[1] = (uint8_t)RX_buffer[tmp_pointer + 1];
    tmp_data_pointer[2] = (uint8_t)RX_buffer[tmp_pointer + 2];
    if ( ((uint8_t)tmp_data_pointer[0] + (uint8_t)tmp_data_pointer[1]) == (uint8_t)tmp_data_pointer[2])
    {
      uint16_t tmp = (uint16_t)tmp_data_pointer[0] << 8 | (uint16_t)tmp_data_pointer[1];
      * value = (uint16_t)(tmp);
      OK = 1;
    }
  }
  return OK;
}

//////////////////////////////calVoltage////////////////////////////////
float calculate_initial_capacity_percentage(float voltage)
{
  float capacity = 0;
  if (voltage > 4.2) {
    capacity = 100;
  } else if (voltage < 3.2 ) {
    capacity = 0;
  } else if (voltage > 4 ) {
    capacity = 80 * voltage - 236;
  } else if (voltage > 3.67 ) {
    capacity = 212.53 * voltage - 765.29;
  } else {
    capacity = 29.787234 * voltage - 95.319149 ;
  }
  return capacity;
}


//////////////////////////////SentValue////////////////////////////////
void Sent_value(uint8_t header, float* tmp) {
  uint16_t data_buffer = (float) * tmp * 10.f;
  uint8_t sant_tmp[2] = {0};
  sant_tmp[0] = (uint16_t)data_buffer >> 8;
  sant_tmp[1] = (uint16_t)data_buffer >> 0;
  Serial.write(header);
  Serial.write(header);
  Serial.write(sant_tmp[0]);
  Serial.write(sant_tmp[1]);
  Serial.write(sant_tmp[0] + sant_tmp[1]);
  Serial.println();
}

//////////////////////////////UART////////////////////////////////


void read_uart() {
  while (Decode_pointer != RX_pointer) {
    if ((is_data_OK & _volume_OK) == 0) {
      if (Decode(&_volume, 0xff) && _volume > 0)
      // if (Decode(&_volume, 0xff))
        is_data_OK |= _volume_OK;
    }

    if ((is_data_OK & _lidStatus_OK) == 0) {
      if (Decode_press(&_lidStatus, 0xfe))
        is_data_OK |= _lidStatus_OK;
    }

    if ((is_data_OK & _temp_OK) == 0) {
      if (Decode(&_temp, 0xfd) && _temp > 0)
        is_data_OK |= _temp_OK;
    }

    if ((is_data_OK & _humid_OK) == 0) {
      if (Decode(&_humid, 0xfc) && _humid > 0)
        is_data_OK |= _humid_OK;
    }

    if ((is_data_OK & _flameStatus_OK) == 0) {
      if (Decode_press(&_flameStatus, 0xfb))
        is_data_OK |= _flameStatus_OK;
    }

    if ((is_data_OK & _soundStatus_OK) == 0) {
      if (Decode(&_soundStatus, 0xfa) && _soundStatus > 0)
      // if (Decode(&_soundStatus, 0xfa))
        is_data_OK |= _soundStatus_OK;
    }

    if ((is_data_OK & _carbon_OK) == 0) {
      if (Decode_press(&_carbon, 0xf9) && _carbon > 0)
      // if (Decode_press(&_carbon, 0xf9))
        is_data_OK |= _carbon_OK;
    }

    if ((is_data_OK & _methane_OK) == 0) {
      if (Decode_press(&_methane, 0xf8) && _methane > 0)
      // if (Decode_press(&_methane, 0xf8))
        is_data_OK |= _methane_OK;
    }

    if ((is_data_OK & _light_OK) == 0) {
      if (Decode_press(&_light, 0xf7) && _light > 0)
      // if (Decode_press(&_light, 0xf7))
        is_data_OK |= _light_OK;
    }

    if ((is_data_OK & _pitch_OK) == 0) {
      if (Decode(&_pitch, 0xf6))
        is_data_OK |= _pitch_OK;
    }

    if ((is_data_OK & _roll_OK) == 0) {
      if (Decode(&_roll, 0xf5))
        is_data_OK |= _roll_OK;
    }

    if ((is_data_OK & _press_OK) == 0) {
      if (Decode_press(&_press, 0xf4) && _press > 0)
      // if (Decode_press(&_press, 0xf4))
        is_data_OK |= _press_OK;
    }

    if ((is_data_OK & _batt_OK) == 0) {
      // if (Decode(&V_batt, 0xf3) && V_batt > 0 && V_batt < 5)
      if (Decode(&V_batt, 0xf3) && V_batt >= 0 && V_batt <= 101)
        is_data_OK |= _batt_OK;
    }
    // Serial.println(is_data_OK, BIN);
    Decode_pointer++;
    if (Decode_pointer >= RX_buffer_size) {
      Decode_pointer = 0;
    }
  }
}


void serialEvent() {
  while (Serial.available()) {
    RX_buffer[RX_pointer] = Serial.read();
    RX_pointer++;
    if (RX_pointer >= RX_buffer_size)RX_pointer = 0;
  }
}





//#define         accuracyMQ4                     (0)   //for linearcurves
#define         accuracyMQ4                   (1)   //for nonlinearcurves, un comment this line and comment the above line if calculations

/****************** MQ4ResistanceCalculation ****************************************
  Input:   raw_adc - raw value read from adc, which represents the voltage
  Output:  the calculated sensor resistance
  Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQ4ResistanceCalculation(uint16_t raw_adc)
{
  return ( ((float)1 * (5 - (float)raw_adc*0.001f) /((float)raw_adc*0.001f)));
}

/*****************************  MQ4Read *********************************************
  Input:   mq_pin - analog channel
  Output:  Rs of the sensor
  Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQ4Read(uint16_t Raw)
{
  return MQ4ResistanceCalculation(Raw);
}

/*****************************  MQ4GetGasPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
  Output:  ppm of the target gas
  Remarks: This function uses different equations representing curves of each gas to
         calculate the ppm (parts per million) of the target gas.
************************************************************************************/
int MQ4GetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( accuracyMQ4 == 0 ) {
    if ( gas_id == 3 )  return (pow(10, ((-2.199 * (log10(rs_ro_ratio))) + 2.766 )));
  } else if ( accuracyMQ4 == 1 ) {
    if ( gas_id == 3 ) return (pow(10, ((-2.199 * (log10(rs_ro_ratio))) + 2.766 )));
  }
  return 0;
}





//#define         accuracyMQ9                     (0)   //for linearcurves
#define         accuracyMQ9                   (1)   //for nonlinearcurves, un comment this line and comment the above line if calculations


/****************** MQ9ResistanceCalculation ****************************************
  Input:   raw_adc - raw value read from adc, which represents the voltage
  Output:  the calculated sensor resistance
  Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQ9ResistanceCalculation(uint16_t raw_adc)
{
  return ( ((float)1 * (5 - (float)raw_adc*0.001f) / ((float)raw_adc*0.001f)));
}


/*****************************  MQ9Read *********************************************
  Input:   mq_pin - analog channel
  Output:  Rs of the sensor
  Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQ9Read(uint16_t Raw)
{
  return MQ9ResistanceCalculation(Raw);
}

/*****************************  MQ9GetGasPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
  Output:  ppm of the target gas
  Remarks: This function uses different equations representing curves of each gas to
         calculate the ppm (parts per million) of the target gas.
************************************************************************************/
int MQ9GetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( accuracyMQ9 == 0 ) {
    if ( gas_id == 2 )  return (pow(10, ((-2.849 * (log10(rs_ro_ratio))) + 2.997)));
  } else if ( accuracyMQ9 == 1 ) {
    if ( gas_id == 2 ) return  (pow(10, ((-2.849 * (log10(rs_ro_ratio))) + 2.997)));
  }
  return 0;
}
