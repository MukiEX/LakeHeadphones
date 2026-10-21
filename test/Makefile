# Host tests. BINS = directory from: extract_filters.py DolbyHph.dll --bins <dir> --rates all
BINS ?= ../bins
CFLAGS = -O2 -Wall -Wextra -I../engine
all: pipeline_test stall_test lowlat_test resampler_test lhp_wav surround_test surround71_test
pipeline_test: pipeline_test.c ../firmware/common/dh_pipeline.c ../engine/lakehp.c
	$(CC) $(CFLAGS) -o $@ $^ -lm
stall_test: stall_test.c ../firmware/common/dh_pipeline.c ../engine/lakehp.c
	$(CC) $(CFLAGS) -o $@ $^ -lm
resampler_test: resampler_test.c ../firmware/common/resampler.c
	$(CC) $(CFLAGS) -o $@ $^ -lm
lhp_wav: ../tools/lhp_wav.c ../engine/lakehp.c
	$(CC) $(CFLAGS) -o $@ $^ -lm
lowlat_test: lowlat_test.c ../firmware/common/dh_pipeline.c ../engine/lakehp.c
	$(CC) $(CFLAGS) -Wno-misleading-indentation -o $@ $^ -lm
check: all
	./pipeline_test $(BINS) && ./stall_test $(BINS) && ./lowlat_test $(BINS) && ./resampler_test && ./surround_test $(BINS) && ./surround71_test $(BINS)
clean:
	rm -f surround_test surround71_test pipeline_test stall_test lowlat_test resampler_test lhp_wav host_run host_run_half
.PHONY: all check clean

surround_test: surround_test.c ../firmware/common/dh_pipeline.c ../engine/lakehp.c
	$(CC) -O2 -Wall -I../engine -I../firmware/common -o $@ $^ -lm

surround71_test: surround71_test.c ../firmware/common/dh_pipeline.c ../engine/lakehp.c
	$(CC) -O2 -Wall -I../engine -I../firmware/common -o $@ $^ -lm

# Pico-PIO-USB's NRZI/bit-stuffing encoder with the isochronous patch (packets
# up to 404 bytes). Needs: python3 tools/vendor_usbhost_libs.py
PIOUSB = ../firmware/rp2350_usbhost/lib/Pico-PIO-USB/src
pio_usb_encode_test: pio_usb_encode_test.c $(PIOUSB)/pio_usb.c
	{ printf '#include <stdint.h>\n#define __no_inline_not_in_flash_func(f) f\nenum {\n'; \
	  sed -n '/PIO_USB_TX_ENCODED_DATA_SE0 = 0/,/^};/p' $(PIOUSB)/pio_usb_ll.h; \
	  sed -n '/^uint16_t __no_inline_not_in_flash_func(pio_usb_ll_encode_tx_data)(/,/^}/p' $(PIOUSB)/pio_usb.c; } > pio_usb_encode.c
	$(CC) $(CFLAGS) -o $@ pio_usb_encode_test.c pio_usb_encode.c

