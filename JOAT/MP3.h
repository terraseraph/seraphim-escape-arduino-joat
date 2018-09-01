#include <arduino.h>
#include <SoftwareSerial.h>


///////////////////////////////
//  header starts here
//////////////////////////////
#ifndef MP3_H     
#define MP3_H

#define MP3_BAUD_RATE 9600
#define MP3_START_CMD 0x7E
#define MP3_VERS 0xFF
#define MP3_END_CMD 0xEF
#define MP3_TX 5
#define MP3_RX 6

void MP3_Init();      
void MP3_Reset();

void MP3_NextSong();
void MP3_PrevSong();
void MP3_PlayWithIndex(uint8_t idx);
void MP3_VolumeUp();
void MP3_VolumeDown();
void MP3_SetVolume(uint8_t vol);
void MP3_SingleCyclePlay(uint8_t songIdx);
void MP3_SleepMode();
void MP3_Wakeup();
void MP3_Play();
void MP3_Pause();
void MP3_PlayWithFolderFilename(uint8_t folderId, uint8_t fileId);
void MP3_Stop();
void MP3_CyclePlayFolder(uint8_t folderId);
void MP3_ShufflePlay();
void MP3_GroupPlay(int cnt, uint8_t[]);
void MP3_PlayWithVolumeIndex(uint8_t vol, uint8_t songIdx);

void MP3_sendCommand(uint8_t cmd, uint8_t dataLen, uint8_t data[]);

#endif // end MP3_H
