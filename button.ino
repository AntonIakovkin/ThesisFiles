
int getButton() {
  int ret = 0;

  currentButton = debounce(lastButton, pinButton);
  if (lastButton == 1 && currentButton == 0) {
    millisButton = millis();
    millisInterval = millis();
    countClick++;
  }

  else if (lastButton == 0 && currentButton == 1) {
    if (millis() - millisButton > intervalLong && countClick == 1) {
      ret = 9;
      countClick = 0;
    }
    else
    {
      millisInterval = millis();
    }
  }
  lastButton  = currentButton;
  if (millis() - millisInterval > intervalInterval && lastButton == 1 ) {
    if (countClick == 1) {
      countClick = 0;
      ret = 1;
    }
    else if (countClick == 2) {
      countClick = 0;
      ret = 2;
    }
    else if (countClick == 3) {
      countClick = 0;
      ret = 3;
    }
    else if (countClick > 3) {
      countClick = 0;
    }
  }
  return ret;
}

int debounce(int last, int pin1)
{
  int current = digitalRead(pin1);
  if (last != current)
  {
    delay(5);
    current = digitalRead(pin1);
  }
  return current;
}
