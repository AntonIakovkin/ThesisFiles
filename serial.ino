void serialEvent() {
    boolean flag1=false;
 
    while (Serial.available() && flag1==false) {
        // get the new byte:
        char inChar = (char)Serial.read();
        if (inChar == '$') {
            stringComplete = true;
            firstByte = false;
            flag1=true;
        }
        else if(inChar == '*') {
            firstByte = true;
            inputString="*";
        }
        else if(firstByte == true) {
            inputString += inChar;
        }
    }
}
