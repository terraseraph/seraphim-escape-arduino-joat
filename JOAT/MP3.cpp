#include "MP3.h"

SoftwareSerial mp3Serial(MP3_RX,MP3_TX);
uint8_t mp3Command[15];
uint8_t mp3Data[10];

void MP3_Init()
{
    mp3Serial.begin(MP3_BAUD_RATE);
    MP3_Reset();
}

void MP3_Reset()
{
    memset((void*) mp3Data, 0x00, sizeof(mp3Data));
    MP3_sendCommand(0x0C, 2, mp3Data);     // reset;
    mp3Data[1] = 0x02;
    MP3_sendCommand(0x09, 2, mp3Data);     // select device;
}
void MP3_NextSong()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x01, 2, mp3Data);     // next song;
}
void MP3_PrevSong()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x02, 2, mp3Data);     // prev song;
}
void MP3_PlayWithIndex(uint8_t idx)
{
    mp3Data[0] = 0x00;
    mp3Data[1] = idx;
    MP3_sendCommand(0x03, 2, mp3Data);     // play indexed song;
}
void MP3_VolumeUp()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x04, 2, mp3Data);     // volume up;
}
void VolumeDown()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x05, 2, mp3Data);     // volume down;
}
void MP3_SetVolume(uint8_t vol)
{
    mp3Data[0] = 0x00;
    if (vol < 0)  mp3Data[1] = 0;
    else if (vol > 30)  mp3Data[1] = 30;
    else mp3Data[1] = vol;
    MP3_sendCommand(0x06, 2, mp3Data);     // set volumne
}
void MP3_SingleCyclePlay(uint8_t songIdx)
{
    mp3Data[0] = 0x00;
    mp3Data[1] = songIdx;
    MP3_sendCommand(0x08, 2, mp3Data);     // single cycle play song;
}
void MP3_SleepMode()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x0A, 2, mp3Data);     // sleep;
}
void MP3_Wakeup()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x0B, 2, mp3Data);     // wake up;
}
void MP3_Play()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x0D, 2, mp3Data);     // play;
}
void MP3_Pause()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x0E, 2, mp3Data);     // Pause;
}
void MP3_PlayWithFolderFilename(uint8_t folderId, uint8_t fileId)
{
    mp3Data[0] = folderId;
    mp3Data[1] = fileId;
    MP3_sendCommand(0x0F, 2, mp3Data);     // Play Folder filename;
}
void MP3_Stop()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x16, 2, mp3Data);     // stop;
}
void MP3_CyclePlayFolder(uint8_t folderId)
{
    mp3Data[0] = 0x00;
    mp3Data[1] = folderId;
    MP3_sendCommand(0x17, 2, mp3Data);     // cycle play folder;
}
void MP3_ShufflePlay()
{
    mp3Data[0] = 0x00;
    mp3Data[1] = 0x00;
    MP3_sendCommand(0x18, 2, mp3Data);     // shuffle play;
}
void MP3_GroupPlay(int cnt, uint8_t data[])
{
  // not working
    MP3_sendCommand(0x21, cnt, data);
}
void MP3_PlayWithVolumeIndex(uint8_t vol, uint8_t songIdx)
{
  // not working
    mp3Data[0] = vol;
    mp3Data[1] = songIdx;
    MP3_sendCommand(0x22, 2, mp3Data);     // play with volume;
}
void MP3_sendCommand(uint8_t cmd, uint8_t dataLen, uint8_t data[])
{ 
    uint8_t dataIdx = 5;
    mp3Command[0] = MP3_START_CMD;
    mp3Command[1] = MP3_VERS;
    mp3Command[3] = cmd;
    mp3Command[4] = 0x00;      // feedback always 0;
    for (int i = 0; i < dataLen; i++)
    {
      mp3Command[dataIdx] = data[i];
      dataIdx++;
    }
    mp3Command[2] = dataIdx-1;    // size of data
    mp3Command[dataIdx] = MP3_END_CMD; 
    dataIdx++;

    for (int i = 0; i < dataIdx; i++)
    {
      mp3Serial.write(mp3Command[i]);
    }
}

