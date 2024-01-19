void mp3play (){
  soundcmd = receiveFromESP32Data.soundcmd;

#ifdef MP3DFPlayer
    String prefix = "00";
    String postfix = ".mp3";
    String PlayFileName = prefix + soundcmd;
    int myPlayFileName = PlayFileName.toInt(); // we need to convert back to integer for the function requires number
//    FileName = "/T00" + soundcmd + ".mp3";
    if (soundcmd != 0) {
      if (soundcmd == 9) soundcmd = randomsound;
      myDFPlayer.pause();  //pause the mp3
      myDFPlayer.playMp3Folder(myPlayFileName); //play specific mp3 in SD:/MP3/0004.mp3; File Name(0~65535)
      soundcmd = 0;
    }
  
#endif

#ifdef MP3Sparkfun
  if (mp3.isConnected()) {  // check if the MP3 Trigger is connected via i2c
    if (soundcmd != 0) {
      if (soundcmd == 9) soundcmd = randomsound;
      if (mp3.isPlaying() == true) mp3.stop();
      mp3.playTrack(soundcmd);
      soundcmd = 0;
    }
  }
#endif

#ifdef MP3VS105
    String prefix = "/T00";
    String postfix = ".mp3";
    String PlayFileName = prefix + soundcmd + postfix;
//    FileName = "/T00" + soundcmd + ".mp3";
    if (soundcmd != 0) {
      if (soundcmd == 9) soundcmd = randomsound;
      musicPlayer.stopPlaying();
      musicPlayer.playFullFile(PlayFileName);
      soundcmd = 0;
    }
  
#endif

}


#ifdef MP3VS105
void printDirectory(File dir, int numTabs) { /// File listing helper
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}
#endif
