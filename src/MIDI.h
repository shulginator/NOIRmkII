void handleControlChange(byte channel, byte number, byte value) {
  if (channel == midi_channel) {
    if (number == 102) { //байпас с ретриггером
      if (value > 63) {
        bpstate = 1;
        mode_sw = 1;
        i = 0;
        j = 0;
        if (depthramptime > 0) {
          depthrampval = 0;
          depthrampup = 1;
          depthramptimer = millis();
        }
      }
      if (value <= 63) {
        bpstate = 0;
        mode_sw = 0;
      }
    }

    if (number == 101) { //байпас
      if (value > 63) {
        bpstate = 1;
        mode_sw = 1;
      }
      if (value <= 63) {
        bpstate = 0;
        mode_sw = 0;
      }
    }

    if (number == 105) { //банк волн
      if (value > 63) {
        wavebank = 0;
      }
      if (value <= 63) {
        wavebank = 1;
      }
    }

    if (number == 100) { //ретриггер
      i = 0;
      j = 0;
    }

    if (number == 103) { //режим
      if (value > 63) {
        mode = 1;
      }
      if (value <= 63) {
        mode = 0;
      }
    }
    if (number == 104) { //генератор волн
      pattern_rand_midi = 1;
      pattern_rand = value;
      pattern_rand = map(pattern_rand, 0, 127, 0, 1000);
    }

    if (number == 50) { //multiplier
      int mult_midi_value = value;
      if (mult_follow_midi) {
        if (mult_midi_value < 15) {
          m = 0.25;
        }
        else if (mult_midi_value >= 15 && mult_midi_value < 31) {
          m = 0.5;
        }
        else if (mult_midi_value >= 31 && mult_midi_value < 47) {
          m = 1;
        }
        else if (mult_midi_value >= 47 && mult_midi_value < 63) {
          m = 1.5;
        }
        else if (mult_midi_value >= 63 && mult_midi_value < 79) {
          m = 2;
        }
        else if (mult_midi_value >= 79 && mult_midi_value < 95) {
          m = 3;
        }
        else if (mult_midi_value >= 95 && mult_midi_value < 111) {
          m = 4;
        }
        else if (mult_midi_value >= 111 && mult_midi_value <= 127) {
          m = 6;
        }
      }
      mult_follow_midi = 1;
      mult_follow_pot = mult_follow_pres = 0;
    }

    if (number == 51) { //n
      int n_midi_value = value;
      if (n_midi_value < 15) {
        n = 0;
      }
      else if (n_midi_value >= 15 && n_midi_value < 31) {
        n = 1;
      }
      else if (n_midi_value >= 31 && n_midi_value < 47) {
        n = 2;
      }
      else if (n_midi_value >= 47 && n_midi_value < 63) {
        n = 3;
      }
      else if (n_midi_value >= 63 && n_midi_value < 79) {
        n = 4;
      }
      else if (n_midi_value >= 79 && n_midi_value < 95) {
        n = 5;
      }
      else if (n_midi_value >= 95 && n_midi_value < 111) {
        n = 10;
      }
      else if (n_midi_value >= 111 && n_midi_value <= 127) {
        n = 20;
      }
    }

    if (number == 52) { //waveform
      int wave_midi_value = value;
      if (wave_follow_midi) {
        if (wave_midi_value < 15) {
          waveform = 1;
        }
        else if (wave_midi_value >= 15 && wave_midi_value < 31) {
          waveform = 2;
        }
        else if (wave_midi_value >= 31 && wave_midi_value < 47) {
          waveform = 3;
        }
        else if (wave_midi_value >= 47 && wave_midi_value < 63) {
          waveform = 4;
        }
        else if (wave_midi_value >= 63 && wave_midi_value < 79) {
          waveform = 5;
        }
        else if (wave_midi_value >= 79 && wave_midi_value < 95) {
          waveform = 6;
        }
        else if (wave_midi_value >= 95 && wave_midi_value < 111) {
          waveform = 7;
        }
        else if (wave_midi_value >= 111 && wave_midi_value <= 127) {
          waveform = 8;
        }
      }
      wave_follow_midi = 1;
      wave_follow_pot = wave_follow_pres = 0;
    }

    //Symmetry
    if (number == 53) {
      symm_follow_midi = 1;
      symm_follow_pot = symm_follow_pres = symm_follow_exp = 0;
      float symm_midi_value = value + 1;
      symm_midi_value = map(symm_midi_value, 128, 1, (128 - 10), 10);
      symm_midi_value /= 128;
      if (symm_follow_midi) {
        symmetry = symm_midi_value;
      }
    }

    //Depth
    if (number == 54) {
      depth_follow_midi = 1;
      depth_follow_pot = depth_follow_exp = depth_follow_pres = 0;
      float depth_midi_value = value;
      depth_midi_value /= 127;
      if (depth_follow_midi) {
        depth = depth_midi_value;
      }
    }

    //Randomizer
    if (number == 55) {
      float randomizer_midi_value = value;
      if (randomizer_midi_value == 0) {
        randomizer_depth = 0;
      }
      else {
        randomizer_depth = randomizer_midi_value / 127.0f;
      }
    }

    //Ramp-time
    if (number == 56) {
      int ramptime_midi_value = value;
      if (ramptime_midi_value == 0) {
        ramptime = 0;
      }
      else {
        ramptime = map(ramptime_midi_value, 0, 127, 0, 500);
      }
    }

    //Depth ramp-time
    if (number == 57) {
      int depth_ramptime_midi_value = value;
      depthramptime = map(depth_ramptime_midi_value, 0, 127, 0, 500);
    }
  }

  //midi channel set
  if (number == 90) {
    midi_channel = channel;
    myEEPROM.write(0, midi_channel);
    blinking = 2;
  }
}

int clk, duration, delta, attempt;
unsigned long inHigh, lastinHigh;

void handleClock() {
  clk++;
  if (clk == 4) {
    inHigh = millis();
    duration = inHigh - lastinHigh;
    lastinHigh = inHigh;
    clk = 0;
    attempt++;
    delta += duration;
  }
  if (attempt == 6) {
    midi_tempo = delta;
    if (!rate_follow_midi) {
      rate_follow_midi = 1;
      rate_follow_pot = rate_follow_tap = rate_follow_exp = rate_follow_pres = 0;
    }
    attempt = 0;
    delta = 0;
    Serial.print("midi_tempo: ");
    Serial.println(midi_tempo);
  }
}

void handleProgramChange(byte channel, byte number) {
  if (channel == midi_channel) {
    if (number <= 15) {
      if (!rampflag) {
        preset_recall(number + 1);
      }
      if (rampflag) {
        preset_save(number + 1);
      }
    }
  }
}