#include <MP.h>

#include "FFT.h"
#include "IIR.h"

/*-----------------------------------------------------------------*/
/*
 * FFT parameters
 */
/* Select FFT length */

//#define FFT_LEN 32
//#define FFT_LEN 64
//#define FFT_LEN 128
//#define FFT_LEN 256
//#define FFT_LEN 512
#define FFT_LEN 1024
// #define FFT_LEN 2048
// #define FFT_LEN 4096

/* Number of channels*/
#define MAX_CHANNEL_NUM 1
//#define MAX_CHANNEL_NUM 2
//#define MAX_CHANNEL_NUM 4

/* Parameters */
const int   g_channel = MAX_CHANNEL_NUM; /* Number of channels */
const int   g_cutoff  = 1000; /* Cutoff frequency */
const float g_Q       = sqrt(0.5); /* Q Value */
const int   g_sample  = 1024; /* Number of channels */
// const int   g_fs      = 48000; /* Sampling Rate */

const int   g_max_shift = 30; /* Pitch shift value */

const int   g_result_size = 4; /* Result buffer size */

FFTClass<MAX_CHANNEL_NUM, FFT_LEN> FFT;
IIRClass LPF;

arm_rfft_fast_instance_f32 iS;
arm_biquad_cascade_df2T_instance_f32 bS;

/* Allocate the larger heap size than default */
USER_HEAP_SIZE(64 * 1024);

/* MultiCore definitions */
struct Request {
  void *buffer;
  int  sample;
  int  pitch_shift;
};

struct Result {
  void *buffer;
  int  sample;
};

void setup()
{
  int ret = 0;

  /* Initialize MP library */
  ret = MP.begin();
  if (ret < 0) {
    errorLoop(2);
  }

  /* receive with non-blocking */
  MP.RecvTimeout(100000);

  /* begin FFT */
  FFT.begin(WindowRectangle,MAX_CHANNEL_NUM,0);

  /* begin LPF */
  if(!LPF.begin(TYPE_LPF, g_channel, g_cutoff, g_Q, g_sample)) {
    int err = LPF.getErrorCause();
    printf("error! %d\n", err);
    errorLoop(abs(err));
  }

  /* begin Filter */
  // if(!HPF.begin(TYPE_HPF, g_channel, g_cutoff, g_Q, g_sample, IIRClass::Interleave, g_fs)) {
  //   int err = HPF.getErrorCause();
  //   printf("error! %d\n", err);
  //   errorLoop(abs(err));
  // }

  arm_rfft_1024_fast_init_f32(&iS);
}

void loop()
{
  int      ret;
  int8_t   sndid = 10; /* user-defined msgid */
  int8_t   rcvid;
  Request  *request;
  static Result result[g_result_size];

  static float pTmp[(FFT_LEN + g_max_shift) * 2];
  static float pDst[FFT_LEN]; //FFTの結果が入る変数
  static q15_t pLpfTmp[FFT_LEN]; //ローパスフィルタの結果が入る一時変数
  static q15_t pLpfDst[FFT_LEN*2]; // /* for stereo */ pLpfTmpをステレオデータにした時に入る変数
  // static q15_t pHpfTmp[FFT_LEN]; //ハイパスフィルタの結果が入る一時変数
  // static q15_t pHpfDst[FFT_LEN*2]; // pHpfTmpをステレオデータにした時に入る変数
  static q15_t pDstQ15[FFT_LEN]; // DstをQ15に変換するための変数
 
  static q15_t pOut[g_result_size][FFT_LEN*2]; //最終的にメインコアに渡す変数
  static int pos = 0;
  static int pitch_shift = 0;

  /* Receive PCM captured buffer from MainCore */
  ret = MP.Recv(&rcvid, &request);
  if (ret >= 0) {
      FFT.put((q15_t*)request->buffer,request->sample);
      pitch_shift = request->pitch_shift;
  }

  if ((pitch_shift < -g_max_shift) || (pitch_shift > g_max_shift)) {
    puts("Shift value error.");
    errorLoop(10);
  }

  while (!FFT.empty(0)) {
    for (int i = 0; i < g_channel; i++) {
      if (pitch_shift > 0) {
        memset(pTmp, 0, pitch_shift * 2);
        FFT.get_raw(&pTmp[(pitch_shift) * 2], i);
        arm_rfft_fast_f32(&iS, &pTmp[0], pDst, 1);
      } else {
        FFT.get_raw(&pTmp[0],i);
        memset(pTmp+(FFT_LEN), 0, abs(pitch_shift) * 2);
        arm_rfft_fast_f32(&iS, &pTmp[abs(pitch_shift) * 2], pDst, 1);
      }
      
      if (i == 0) {
        // arm_float_to_q15(pDst, pLpfTmp, FFT_LEN); // FFTの結果をローパスフィルタの一時変数に代入している（タイプ変換かねて）
        // LPF.put(pLpfTmp, FFT_LEN); // 実際にローパスフィルタをかけている
        // int cnt = LPF.get(pLpfDst, 0); // pLpfDstでローパスフィルタの結果を受け取っている（戻り値はローパスフィルタの結果の長さ）
        // printf("cnt=%d\n", cnt);
        // for(int j = 0; j < cnt; j++) {
        //   pOut[pos][j * 2]     = pLpfDst[j];
        //   pOut[pos][j * 2 + 1] = 0;
        // }
        // result[pos].buffer = (void*)MP.Virt2Phys(&pOut[pos][0]);
        // result[pos].sample = cnt;

        // ret = MP.Send(sndid, &result[pos],0);
        // pos = (pos + 1) % g_result_size;
        // if (ret < 0) {
        //   errorLoop(11);
        // }


        // arm_float_to_q15(pDst, pDstQ15, FFT_LEN);
        // HPF.put(pHpfTmp, FFT_LEN); // 実際にローパスフィルタをかけている
        // int cnt = HPF.get(pHpfDst, 0);
        // printf("cnt=%d\n", cnt);
        // for(int j = 0; j < cnt; j++) {
        //   pOut[pos][j * 2]     = pHpfDst[j];
        //   pOut[pos][j * 2 + 1] = 0;
        // }
        // result[pos].buffer = (void*)MP.Virt2Phys(&pOut[pos][0]);
        // result[pos].sample = cnt;

        arm_float_to_q15(pDst, pDstQ15, FFT_LEN);
        for(int j = 0; j < FFT_LEN; j++) {
          pOut[pos][j * 2]     = pDstQ15[j];
          pOut[pos][j * 2 + 1] = 0;
        }
        result[pos].buffer = (void*)MP.Virt2Phys(&pOut[pos][0]);
        result[pos].sample = FFT_LEN;

        ret = MP.Send(sndid, &result[pos],0);
        pos = (pos + 1) % g_result_size;
        if (ret < 0) {
          errorLoop(11);
        }
      }
    }
  }
}

void errorLoop(int num)
{
  int i;

  while (1) {
    for (i = 0; i < num; i++) {
      ledOn(LED0);
      delay(300);
      ledOff(LED0);
      delay(300);
    }
    delay(1000);
  }
}
