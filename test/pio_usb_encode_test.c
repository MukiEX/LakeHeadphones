#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
uint16_t pio_usb_ll_encode_tx_data(uint8_t const *buffer, uint16_t buffer_len, uint8_t *encoded_data);
/* reference: symbols list (J=3,K=1,SE0=0,COMP=2) from NRZI + bit stuffing, then SE0, COMP, pad K to multiple of 4 */
static int ref(const uint8_t *b, int n, uint8_t *sym) {
  int k = 0, state = 1 /* J idle: first 0 -> J? match original */, ones = 0;
  for (int i = 0; i < n; i++) for (int bit = 0; bit < 8; bit++) {
    int v = (b[i] >> bit) & 1;
    if (v) { sym[k++] = state ? 1 : 3; ones++; }
    else { if (state) { sym[k++] = 3; state = 0; } else { sym[k++] = 1; state = 1; } ones = 0; }
    if (ones == 6) { if (state) { sym[k++] = 3; state = 0; } else { sym[k++] = 1; state = 1; } ones = 0; }
  }
  sym[k++] = 0; sym[k++] = 2;
  do sym[k++] = 1; while (k & 3);
  return k;
}
int main(void) {
  static uint8_t buf[404], enc[2000], sym[5000];
  srand(3); int bad = 0;
  int sizes[] = {2, 12, 64, 68, 100, 196, 200, 292, 392, 404};
  for (int t = 0; t < 10; t++) for (int rep = 0; rep < 200; rep++) {
    int n = sizes[t];
    for (int i = 0; i < n; i++) buf[i] = rep == 0 ? 0xFF : rep == 1 ? 0x00 : (uint8_t)rand();
    memset(enc, 0xAA, sizeof enc);
    int len = pio_usb_ll_encode_tx_data(buf, n, enc);
    int k = ref(buf, n, sym);
    if (len != k / 4) { printf("n=%d len %d expected %d\n", n, len, k / 4); bad++; continue; }
    for (int i = 0; i < k; i++) {
      int s = (enc[i / 4] >> (6 - 2 * (i % 4))) & 3;
      if (s != sym[i]) { printf("n=%d mismatch at symbol %d\n", n, i); bad++; break; }
    }
  }
  int n = 404; for (int i = 0; i < n; i++) buf[i] = 0xFF;
  int len = pio_usb_ll_encode_tx_data(buf, n, enc);
  printf("worst case (404 bytes of 0xFF): %d encoded bytes (pool slot %d)\n", len, (404 + 4) * 2 * 7 / 6 + 4);
  printf(bad ? "FAIL (%d)\n" : "PASS: encoder matches NRZI+bit-stuffing reference for 2..404-byte packets\n", bad);
  return bad;
}
