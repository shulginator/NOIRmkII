void ee_float_write(int addr, float val) // запись в ЕЕПРОМ
{
  byte *x = (byte *)&val;
  for (byte i = 0; i < 4; i++)
  {
    myEEPROM.write(i + addr, x[i]);
  }
}

float ee_float_read(int addr) // чтение из ЕЕПРОМ
{
  byte x[3];
  for (byte i = 0; i < 4; i++) x[i] = myEEPROM.read(i + addr);
  float *y = (float *)&x;
  return y[0];
}

/////////////////////////////////
//--Функция для мониторинга данных
void monitoring () {
  if (millis() - timing[2] > 50)
  {
/*
    Serial.println("$");
    Serial.println(rateF+rateS);
    Serial.println(";");

*/
    timing[2] = millis();
  }
}