/////////////
//-PWM OUTPUT
void pwmout() {
  if (wavebank) {
    switch (waveform) {
      case 1: //--Пила
        waveval = saw[i];
        break;
      case 2: //--Обратная пила
        waveval = saw[i];
        waveval = 1000 - waveval;
        break;
      case 3: //--Квадрат
        waveval = square[i];
        waveval *= 1000;
        break;
      case 4: //--Треугольник
        waveval = triangle[i];
        break;
      case 5: //--Синус
        waveval = sine[i];
        break;
      case 6: //--Полуволна
        waveval = lumps[i];
        waveval = 1000 - waveval;
        break;
      case 7: //--Обратная полуволна
        waveval = lumps[i];
        break;
      case 8: //--S&H
        waveval = squarewave1[0];
        waveval *= sh_rand;
        break;
    }
  }

  if (!wavebank) {
    switch (waveform) {
      case 1:
        {
          waveval = squarewave1[p];
          waveval *= 1000;
          break;
        }
      case 2:
        {
          waveval = squarewave2[p];
          waveval *= 1000;
          break;
        }
      case 3:
        {
          waveval = squarewave3[p];
          waveval *= 1000;
          break;
        }
      case 4:
        {
          waveval = squarewave4[p];
          waveval *= 1000;
          break;
        }
      case 5: // повтор
        {
          waveval = squarewave5[p];
          waveval *= 1000;
          break;
        }
      case 6:
        {
          waveval = squarewave6[p];
          waveval *= 1000;
          break;
        }
      case 7:
        {
          waveval = squarewave7[p];
          waveval *= 1000;
          break;
        }
      case 8:
        {
          waveval = squarewave1[0];
          waveval *= pattern_rand;
          break;
        }
    }
  }

  waveval3 = firstKalmanFilter.updateEstimate(waveval);

  if (wavebank) {
    if (waveform == 1 || waveform == 2 || waveform == 3 || waveform == 8) {
      waveval = waveval3;
    }
  }
  if (!wavebank) {
    waveval = waveval3;
  }

  //--Добавление ручки гублины и инверсия второй волны
  mode_ramp = mode_move.update();
  waveval_inv = 1000 - waveval_inv;
  waveval_inv = abs(mode_ramp - waveval);
  waveval_inv *= depth;
  if (depthramptime > 0) {
    waveval_inv *= depthrampval;
  }
  waveval_inv = 1000 - waveval_inv;

  wavevalf = 1000 - waveval;
  wavevalf *= depth;
  if (depthramptime > 0) {
    wavevalf *= depthrampval;
  }
  wavevalf = 1000 - wavevalf;

  pwmWrite(PB9, waveval_inv);
  pwmWrite(PB8, wavevalf);
  /*
    //Если смена режима залагала
    if (mode && mode_move.isFinished() == 1 && mode_ramp > 0) {
    mode_move.go(1000, 25, SINUSOIDAL_INOUT, ONCEFORWARD);
    }
    if (!mode && mode_move.isFinished() == 1 && mode_ramp < 1000) {
    mode_move.go(0, 25, SINUSOIDAL_INOUT, ONCEFORWARD);
    }
  */
}