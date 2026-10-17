#include "../firmware/common/resampler.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* 48k -> 44.1k: feed 1 kHz and 15 kHz sines in random chunk sizes, measure SNR vs ideal */
static resampler_t R;
int main(void){
  const int N=48000*2; int16_t *in=malloc(N*4), *out=malloc(N*4);
  double f[2]={1000,15000};
  for(int i=0;i<N;i++){in[2*i]=(int16_t)(20000*sin(2*M_PI*f[0]*i/48000)); in[2*i+1]=(int16_t)(20000*sin(2*M_PI*f[1]*i/48000));}
  rs_init(&R,48000,44100); int ip=0,op=0; srand(1);
  while(ip<N){ int chunk=1+rand()%300; if(ip+chunk>N)chunk=N-ip; int used; int m=rs_process(&R,in+2*ip,chunk,&used,out+2*op,4096); ip+=used; op+=m; if(used<chunk){ /* out full not possible */ } }
  /* expected: out[j] = sin at input time t_j = j*48000/44100 - delay; find delay by fit */
  double best=1e30,bd=0;
  for(double d=-20; d<20; d+=0.001){ double e=0; for(int j=2000;j<2400;j++){ double t=j*48000.0/44100-d; double ex=20000*sin(2*M_PI*f[0]*t/48000); e+=(out[2*j]-ex)*(out[2*j]-ex);} if(e<best){best=e;bd=d;} }
  for(int c=0;c<2;c++){ double se=0,ss=0; for(int j=2000;j<op-100;j++){ double t=j*48000.0/44100-bd; double ex=20000*sin(2*M_PI*f[c]*t/48000); double v=out[2*j+c]; se+=(v-ex)*(v-ex); ss+=ex*ex;} printf("%5.0f Hz: SNR %.1f dB\n",f[c],10*log10(ss/se)); }
  printf("delay %.3f input samples, produced %d of expected %d\n",bd,op,(int)(N*44100.0/48000));
  /* aliasing check: 23 kHz input should be strongly attenuated after 44.1k output */
  rs_init(&R,48000,44100); for(int i=0;i<N;i++){in[2*i]=in[2*i+1]=(int16_t)(20000*sin(2*M_PI*23000.0*i/48000));}
  int used; int m=rs_process(&R,in,N,&used,out,N); double s=0; for(int j=1000;j<m-100;j++) s+=out[2*j]*(double)out[2*j];
  printf("23 kHz (above new Nyquist) leakage: %.1f dBFS\n",10*log10(s/(m-1100)/(32768.0*32768/2)));
  return 0;}
