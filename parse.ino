void parse_string(String inputString) {
  int length1 = inputString.length();
  int s0, s1, s2, s3, s4;

  // 0
  for (int i = 1; i < length1; i++) {
    if (inputString.charAt(i) == ';') {
      s0 = i; break;
    }
  }
  params[0] = inputString.substring(1, s0).toInt();
  // 1
  for (int i = s0 + 1; i < length1; i++) {
    if (inputString.charAt(i) == ';') {
      s1 = i; break;
    }
  }
  params[1] = inputString.substring(s0 + 1, s1).toInt();
  // 2
  for (int i = s1 + 1; i < length1; i++) {
    if (inputString.charAt(i) == ';') {
      s2 = i; break;
    }
  }
  params[2] = inputString.substring(s1 + 1, s2).toInt();
  // 3
  for (int i = s2 + 1; i < length1; i++) {
    if (inputString.charAt(i) == ';') {
      s3 = i; break;
    }
  }
  params[3] = inputString.substring(s2 + 1, s3).toInt();
  // 4
  for (int i = s3 + 1; i < length1; i++) {
    if (inputString.charAt(i) == ';') {
      s4 = i; break;
    }
  }
  params[4] = inputString.substring(s3 + 1, s4).toInt();

}


void run_from_get() {

  switch (params[0]) {
    case 1:
      if (params[1] == 1)
        sendData = true;
      else
        sendData = false;
      break;
    case 2: //
      tekLeds = params[1];
      break;
    case 3:  //
      intervalsendBME = params[1];
      intervalsendIMU = params[2];
      intervalsendGPS = params[3];
      intervalsendSYS = params[4];
      break;
    default:
      break;
  }
}
