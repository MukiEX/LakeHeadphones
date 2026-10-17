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
