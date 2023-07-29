/*
   MIDI play on PSG Modoki

*/

#include <MIDI.h>
#include <Wire.h>

#define MAX_CH 12

// freq table in 4MHz master clock

uint16_t psgtone[] = {
  15289, 14431, 13621, 12856, 12135, 11454, 10811, 10204, 9631, 9091, 8581, 8099, 7645, 7215, 6810, 6428, 6067, 5727, 5405, 5102, 4816, 4545, 4290, 4050, 3822, 3608, 3405, 3214, 3034, 2863, 2703, 2551, 2408, 2273, 2145, 2025, 1911, 1804, 1703, 1607, 1517, 1432, 1351, 1276, 1204, 1136, 1073, 1012, 956, 902, 851, 804, 758, 716, 676, 638, 602, 568, 536, 506, 478, 451, 426, 402, 379, 358, 338, 319, 301, 284, 268, 253, 239, 225, 213, 201, 190, 179, 169, 159, 150, 142, 134, 127, 119, 113, 106, 100, 95, 89, 84, 80, 75, 71, 67, 63, 60, 56, 53, 50, 47, 45, 42, 40, 38, 36, 34, 32, 30, 28, 27, 25, 24, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 13, 12, 11, 11, 10
};

MIDI_CREATE_DEFAULT_INSTANCE();

uint8_t psg_mixer_state[MAX_CH], psg_inuse[MAX_CH], psg_inuse_ch[MAX_CH], psg_inuse_tone[MAX_CH];

void PSGWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x10);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void noteOn(int ch, int note) {
  uint8_t freq_hi, freq_lo;
  uint8_t psgno, psgch;

  freq_hi = psgtone[note] >> 8;
  freq_lo = psgtone[note] & 0xff;

  psgno = ch / 3;
  psgch = ch % 3;

  PSGWrite(psgno * 16 + psgch * 2, freq_lo);
  PSGWrite(psgno * 16 + psgch * 2 + 1, freq_hi);

  psg_mixer_state[psgno] &= ~(1 << psgch);

  PSGWrite(psgno * 16 + 7, psg_mixer_state[psgno]);

}

void noteOff(int ch) {
  uint8_t psgno, psgch;

  psgno = ch / 3;
  psgch = ch % 3;

  psg_mixer_state[psgno] |= (1 << psgch);

  PSGWrite(psgno * 16 + 7, psg_mixer_state[psgno]);
}

void setVolume(int ch, int volume) {
  uint8_t psgno, psgch;

  psgno = ch / 3;
  psgch = ch % 3;

  PSGWrite(psgno * 16 + psgch + 8, (volume & 0xf));
}

void setup() {

  Wire.begin(4, 5);
  Wire.setClock(400000);

  PSGWrite(0xf, 1); // 4MHz
  PSGWrite(0x1f, 1);
  PSGWrite(0x2f, 1);
  PSGWrite(0x3f, 1);
  PSGWrite(0x7, 0xf8);
  PSGWrite(0x17, 0xf8);
  PSGWrite(0x27, 0xf8);
  PSGWrite(0x37, 0xf8);

  for (int i = 0; i < MAX_CH; i++) {
    psg_mixer_state[i] = 0xff;
    psg_inuse[i] = 0;
    setVolume(i, 15);
  }


  MIDI.begin(MIDI_CHANNEL_OMNI);

  Serial.begin(115200);

}

void loop()
{

  int vol, miditone;
  midi::DataByte data1;

  if (MIDI.read())
  {
    midi::Channel ch = MIDI.getChannel();
    switch (MIDI.getType())     // Get the type of the message we caught
    {
      case midi::NoteOn:
        if (MIDI.getData2() == 0) {
          miditone = MIDI.getData1();
          for (int i = 0; i < MAX_CH; i++) {
            if ((psg_inuse[i] == 1) && (psg_inuse_ch[i] == ch) && (psg_inuse_tone[i] == miditone)) {
              noteOff( i);
              psg_inuse[i] = 0;
              break;
            }
          }
        } else {
          if (ch != 10) { //
            miditone = MIDI.getData1();
            for (int i = 0; i < MAX_CH; i++) {
              if (psg_inuse[i] == 0) {
                noteOn( i , miditone);
                psg_inuse[i] = 1;
                psg_inuse_ch[i] = ch;
                psg_inuse_tone[i] = miditone;
                break;
                //              } else if (psg_inuse_ch[i] == ch) {
                //                noteOn( i , MIDI.getData1());
                //                psg_inuse[i] = 1;
                //                psg_inuse_ch[i] = ch;
                //                break;
                //              }
              }
            }
          }
        }

        break;
      case midi::NoteOff:
        /*
              switch (ch)
              {
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                  noteOff((ch - 1));
                  break;
                default:
                  break;
              }
        */
        miditone = MIDI.getData1();
        for (int i = 0; i < MAX_CH; i++) {
          if ((psg_inuse[i] == 1) && (psg_inuse_ch[i] == ch) && (psg_inuse_tone[i] == miditone)) {
            noteOff( i);
            psg_inuse[i] = 0;
            break;
          }
        }
        break;

      case midi::ProgramChange:
        for (int i = 0; i < MAX_CH; i++) {
          if ((psg_inuse[i] == 1) && (psg_inuse_ch[i] == ch)) {
            noteOff(i);
            psg_inuse[i] = 0;
            setVolume(i,15);
          }
        }
        break;


      case midi::ControlChange:
        data1 = MIDI.getData1();
        switch (data1)
        {
          case midi::ExpressionController:  //エクスプレッション
            //         int vol = map(MIDI.getData2(), 0, 0x7f, 0, 0x0f);
            vol = MIDI.getData2();
            vol >>= 3;
            switch (ch)
            {
              case 1:
              case 2:
              case 3:
              case 4:
              case 5:
              case 6:
              case 7:
              case 8:
              case 9:
                //           setVolume(ch - 1, vol);
                break;
              default:
                break;
            }
            break;
          case midi::ResetAllControllers:
            PSGWrite(0xf, 1); // 4MHz
            PSGWrite(0x1f, 1);
            PSGWrite(0x2f, 1);
            PSGWrite(0x3f, 1);
            PSGWrite(0x7, 0xf8);
            PSGWrite(0x17, 0xf8);
            PSGWrite(0x27, 0xf8);
            PSGWrite(0x37, 0xf8);

            for (int i = 0; i < MAX_CH; i++) {
              psg_mixer_state[i] = 0xff;
              psg_inuse[i] = 0;
              setVolume(i, 15);
            }
            break;
          default:
            break;
        }
      default:
        break;
    }
  }
}
