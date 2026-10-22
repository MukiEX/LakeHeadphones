/* usage: usbout_sim <bins dir> <lowlat 0|1> <old feed 0|1> <clock ppm>
 * Whole-chain timing model of the RP2350-USB-A build: PC -> device FIFO with
 * FIFO-count feedback -> pump (250 us) -> pipeline (real code) -> DSP on a busy
 * core 0 -> output feed (core 1 loop) -> host FIFO -> synchronous device that
 * takes 48 frames per SOF. Counts silence spliced in by the feed (old) and
 * silent packets sent because the host FIFO ran dry. */
#include "../firmware/common/dh_pipeline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static lhp_engine E; static dhp_t P; static uint8_t blob[1 << 16];
#define NCH 8
#define DEV_CAP 294            /* 6 x 7.1 packets */
#define HOST_CAP 512
int main(int argc, char **argv) {
  if (argc < 5) { fprintf(stderr, "usage: usbout_sim <bins> <lowlat> <oldfeed> <ppm>\n"); return 2; }
  const int ll = atoi(argv[2]), oldfeed = atoi(argv[3]); const double ppm = atof(argv[4]);
  char fn[512]; snprintf(fn, sizeof fn, "%s/lhp_filter_dh1_small_48000.bin", argv[1]);
  FILE *f = fopen(fn, "rb"); size_t bl = fread(blob, 1, sizeof blob, f); fclose(f);
  dhp_filter_t F = {48000, 1, blob, (uint32_t)bl};
  dhp_init(&P, &E, &F, 1, 48000, 1); dhp_set_channels(&P, 8); P.lowlat = ll;
  static int16_t frame[64 * NCH], tmp[64 * 2]; memset(frame, 0, sizeof frame);
  double dev_lvl = 0, fb = 48, avg = DEV_CAP / 2, pc_acc = 0;
  int host_lvl = 0; long silent_pkts = 0, spliced = 0, pkts = 0;
  const int US = 10; long t_end = 60L * 1000000;
  long busy_until = 0, hidden_until = -1; uint32_t hidden_w = 0, shown_w = 0;
  double next_sof = 1000, next_pc = 1000;
  srand(1);
  for (long t = 0; t < t_end; t += US) {
    if (t >= next_pc) {                       /* PC packet */
      next_pc += 1000; pc_acc += fb; int n = (int)pc_acc; pc_acc -= n;
      if (dev_lvl + n <= DEV_CAP) dev_lvl += n;
      avg = (avg * 63 + dev_lvl) / 64;        /* TinyUSB FIFO_COUNT feedback */
      const double thr = DEV_CAP / 2;
      fb = 48 + (thr - avg) / thr; if (fb > 49) fb = 49; if (fb < 47) fb = 47;
    }
    if (t % 250 == 0) {                       /* pump (USB timer IRQ) */
      for (;;) {
        uint32_t want = dhp_usb_frames_wanted_guarded(&P, (uint32_t)dev_lvl, DEV_CAP * 3 / 4, DEV_CAP / 2);
        if (want > (uint32_t)dev_lvl) want = (uint32_t)dev_lvl;
        if (want > 64) want = 64;
        if (!want) break;
        dhp_usb_write(&P, frame, want); dev_lvl -= want;
      }
    }
    if (hidden_until >= 0 && t >= hidden_until) { P.out_w = hidden_w; hidden_until = -1; }
    if (t >= busy_until && hidden_until < 0) {   /* core 0 main loop: DSP */
      uint32_t w0 = P.out_w;
      if (dhp_dsp_poll(&P)) {
        long cost = ll ? 250 : 12000 + rand() % 6000;   /* 11-18 ms per room block */
        hidden_w = P.out_w; P.out_w = w0; hidden_until = t + cost; busy_until = t + cost;
      } else if (rand() % 20000 == 0) busy_until = t + 3000;   /* settings page request */
    }
    (void)shown_w;
    /* core 1 loop: feed */
    const int target = 48 * 4;
    for (int g = 0; g < 8 && host_lvl < target; g++) {
      int n = target - host_lvl; if (n > 64) n = 64;
      uint32_t avail = dhp_out_available(&P);
      if (!oldfeed) { if ((uint32_t)n > avail) n = (int)avail; if (!n) break; }
      else if ((uint32_t)n > avail) spliced += n - (int)avail;
      dhp_read_stereo(&P, tmp, (uint32_t)n); host_lvl += n;
    }
    if (t >= next_sof) {                       /* device SOF (our clock) */
      next_sof += 1000.0 * (1 - ppm * 1e-6); pkts++;
      if (host_lvl >= 48) host_lvl -= 48; else { silent_pkts++; if (silent_pkts < 12 || silent_pkts % 10 == 0) fprintf(stderr, "  silent at %.3f s (out avail %u, in %u)\n", t / 1e6, dhp_out_available(&P), P.in_w - P.in_r); }
    }
  }
  printf("%s, %s feed, clock %+.0f ppm: %ld packets, %ld silent packets, %ld frames of spliced silence, pipeline underruns %lu, in-overruns %lu\n",
         ll ? "low latency" : "rooms", oldfeed ? "old" : "new", ppm, pkts, silent_pkts, spliced,
         (unsigned long)P.out_underruns, (unsigned long)P.in_overruns);
}
