#ifdef USE_BASE

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;

void handleEncL() {
  bool inA = digitalRead(LEFT_ENC_PIN_A);

  if (inA == HIGH) {
    if (digitalRead(LEFT_ENC_PIN_B) != inA) {
      left_enc_pos--;
    } else {
      left_enc_pos++;
    }
  }
}


void handleEncR() {
  bool inA = digitalRead(RIGHT_ENC_PIN_A);

  if (inA == HIGH) {
    if (digitalRead(RIGHT_ENC_PIN_B) != inA) {
      right_enc_pos--;
    } else {
      right_enc_pos++;
    }
  }
}


/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return left_enc_pos;
  else return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    left_enc_pos = 0L;
    return;
  } else {
    right_enc_pos = 0L;
    return;
  }
}


/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
