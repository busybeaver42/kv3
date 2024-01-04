//============================================================================
// Name        : kv3audio.cpp
// Author      : Joerg Angermayer
// Version     : 1.0.0
// Copyright   : MIT
// Description : C++, Ansi-style
//============================================================================
/**
 * @file  kv3audio.cpp
 * @brief Implementation of the class functions to access the kinect azure microphone audio data.
 * @brief created on:  09.02.2023;  last update: 24.06.2023
 * @author Jörg Angermayer\n
 * @copyright Licensed under MIT
 */
#include "kv3audio.h"

	// global data
	// info: https://www.alsa-project.org/alsa-doc/alsa-lib/group___p_c_m.html#ga4c2c7bd26cf221268d59dc3bbeb9c048 
	// info: https://vovkos.github.io/doxyrest/samples/alsa/group_PCM.html
	// info: https://wiki.multimedia.cx/index.php/PCM#Channels_And_Interleaving
	static snd_pcm_sframes_t (*readi_func)(snd_pcm_t *handle, void *buffer, snd_pcm_uframes_t size);// Read interleaved frames from a PCM
	static snd_pcm_sframes_t (*writei_func)(snd_pcm_t *handle, const void *buffer, snd_pcm_uframes_t size);// Read non interleaved frames to a PCM.
	static snd_pcm_sframes_t (*readn_func)(snd_pcm_t *handle, void **bufs, snd_pcm_uframes_t size);
	static snd_pcm_sframes_t (*writen_func)(snd_pcm_t *handle, void **bufs, snd_pcm_uframes_t size);

	char *kv3audio::command = NULL;
	snd_pcm_t *kv3audio::handle = NULL;
	std::string kv3audio::strDevice = std::string(DEFAULT_DEVICE);
	snd_pcm_format_t kv3audio::format = DEFAULT_FORMAT;
	unsigned int kv3audio::rate = DEFAULT_RATE;
	unsigned int kv3audio::channels = DEFAULT_CHANNELS;
	tFrameBuffer kv3audio::audioFrameBuffer[];
	bool kv3audio::recordMicroToFile = 0;
	int kv3audio::timelimit = 0;
	int kv3audio::quiet_mode = 0;
	int kv3audio::file_type = FORMAT_DEFAULT;
	int kv3audio::open_mode = 0;
	int kv3audio::capture_stop = 0;
	snd_pcm_stream_t kv3audio::stream = SND_PCM_STREAM_PLAYBACK;
	int kv3audio::interleaved = 1;
	int kv3audio::nonblock = 0;
	u_char *kv3audio::audiobuf = NULL;
	snd_pcm_uframes_t kv3audio::periodeBitSize = 0;

	unsigned kv3audio::period_time = 0;
	unsigned kv3audio::buffer_time = 0;
	snd_pcm_uframes_t kv3audio::period_frames = 0;
	snd_pcm_uframes_t kv3audio::buffer_frames = 0;
	int kv3audio::avail_min = -1;
	int kv3audio::start_delay = 0;
	int kv3audio::stop_delay = 0;
	int kv3audio::verbose = 0;
	int kv3audio::vumeter = VUMETER_NONE;
	int kv3audio::buffer_pos = 0;
	size_t kv3audio::bits_per_sample;
	size_t kv3audio::bits_per_frame;
	size_t kv3audio::chunk_bytes;
	snd_output_t *kv3audio::log;
	int kv3audio::fd = -1;
	off64_t kv3audio::pbrec_count = LLONG_MAX;
	off64_t kv3audio::fdcount;
	int kv3audio::vocmajor = 0;
	int kv3audio::vocminor = 0;

	int kv3audio::frameByteSize = 0;
	int kv3audio::periodeByteSize = 0;
	int kv3audio::sampleByteSize = 0;
	double kv3audio::audioBufferCh01[];

	char *kv3audio::filename = "/var/www/ramdev/kv3.wav";

kv3audio::kv3audio() {}
kv3audio::~kv3audio() {}

/* calculate the data count to read from/to dsp */
off64_t kv3audio::calc_count(void)
{
	off64_t count;
	count = snd_pcm_format_size(hwparams.format, hwparams.rate * hwparams.channels);
/*
	if (timelimit == 0) {
		count = pbrec_count;
	} else {
		count = snd_pcm_format_size(hwparams.format, hwparams.rate * hwparams.channels);
		count *= (off64_t)timelimit;
	}

	return count < pbrec_count ? count : pbrec_count;
	*/
	//count = pbrec_count;
return count;
}

 /**
  * set pcm parameter to read audio signal
  *
  */
void kv3audio::set_params(void)
{
	snd_pcm_hw_params_t *params;
	snd_pcm_sw_params_t *swparams;
	snd_pcm_uframes_t buffer_size;
	int err;
	size_t n;
	unsigned int rate;
	snd_pcm_uframes_t start_threshold, stop_threshold;
	snd_pcm_hw_params_alloca(&params);
	snd_pcm_sw_params_alloca(&swparams);
	err = snd_pcm_hw_params_any(handle, params);
	if (err < 0) {
		error(_("Broken configuration for this PCM: no configurations available"));
		exit(EXIT_FAILURE);
	}
	else if (interleaved)
		err = snd_pcm_hw_params_set_access(handle, params,
						   SND_PCM_ACCESS_RW_INTERLEAVED);
	else
		err = snd_pcm_hw_params_set_access(handle, params,
						   SND_PCM_ACCESS_RW_NONINTERLEAVED);
	if (err < 0) {
		error(_("Access type not available"));
		exit(EXIT_FAILURE);
	}
	err = snd_pcm_hw_params_set_format(handle, params, hwparams.format);
	if (err < 0) {
		error(_("Sample format non available"));
		exit(EXIT_FAILURE);
	}
	err = snd_pcm_hw_params_set_channels(handle, params, hwparams.channels);
	if (err < 0) {
		error(_("Channels count non available"));
		exit(EXIT_FAILURE);
	}

#if 0
	err = snd_pcm_hw_params_set_periods_min(handle, params, 2);
	assert(err >= 0);
#endif
	rate = hwparams.rate;
	err = snd_pcm_hw_params_set_rate_near(handle, params, &hwparams.rate, 0);
	assert(err >= 0);
	if ((float)rate * 1.05 < hwparams.rate || (float)rate * 0.95 > hwparams.rate) {
		if (!quiet_mode) {
			char plugex[64];
			const char *pcmname = snd_pcm_name(handle);
			fprintf(stderr, _("Warning: rate is not accurate (requested = %iHz, got = %iHz)\n"), rate, hwparams.rate);
			if (! pcmname || strchr(snd_pcm_name(handle), ':'))
				*plugex = 0;
			else
				snprintf(plugex, sizeof(plugex), "(-Dplug:%s)",
					 snd_pcm_name(handle));
			fprintf(stderr, _("         please, try the plug plugin %s\n"),
				plugex);
		}
	}
	rate = hwparams.rate;
	if (buffer_time == 0 && buffer_frames == 0) {
		err = snd_pcm_hw_params_get_buffer_time_max(params,
							    &buffer_time, 0);
		assert(err >= 0);
		if (buffer_time > 500000)
			buffer_time = 500000;
	}
	if (period_time == 0 && period_frames == 0) {
		if (buffer_time > 0)
			period_time = buffer_time / 4;
		else
			period_frames = buffer_frames / 4;
	}
	if (period_time > 0)
		err = snd_pcm_hw_params_set_period_time_near(handle, params,
							     &period_time, 0);
	else
		err = snd_pcm_hw_params_set_period_size_near(handle, params,
							     &period_frames, 0);
	assert(err >= 0);
	if (buffer_time > 0) {
		err = snd_pcm_hw_params_set_buffer_time_near(handle, params,
							     &buffer_time, 0);
	} else {
		err = snd_pcm_hw_params_set_buffer_size_near(handle, params,
							     &buffer_frames);
	}
	assert(err >= 0);
	err = snd_pcm_hw_params(handle, params);
	if (err < 0) {
		error(_("Unable to install hw params:"));
		snd_pcm_hw_params_dump(params, log);
		exit(EXIT_FAILURE);
	}
	snd_pcm_hw_params_get_period_size(params, &periodeBitSize, 0);
	snd_pcm_hw_params_get_buffer_size(params, &buffer_size);
	if (periodeBitSize == buffer_size) {
		error(_("Can't use period equal to buffer size (%lu == %lu)"),
		      periodeBitSize, buffer_size);
		exit(EXIT_FAILURE);
	}
	snd_pcm_sw_params_current(handle, swparams);
	if (avail_min < 0)
		n = periodeBitSize;
	else
		n = (double) rate * avail_min / 1000000;
	err = snd_pcm_sw_params_set_avail_min(handle, swparams, n);

	/* round up to closest transfer boundary */
	n = buffer_size;
	if (start_delay <= 0) {
		start_threshold = n + (double) rate * start_delay / 1000000;
	} else
		start_threshold = (double) rate * start_delay / 1000000;
	if (start_threshold < 1)
		start_threshold = 1;
	if (start_threshold > n)
		start_threshold = n;
	err = snd_pcm_sw_params_set_start_threshold(handle, swparams, start_threshold);
	assert(err >= 0);
	if (stop_delay <= 0) 
		stop_threshold = buffer_size + (double) rate * stop_delay / 1000000;
	else
		stop_threshold = (double) rate * stop_delay / 1000000;
	err = snd_pcm_sw_params_set_stop_threshold(handle, swparams, stop_threshold);
	assert(err >= 0);

	if (snd_pcm_sw_params(handle, swparams) < 0) {
		error(_("unable to install sw params:"));
		snd_pcm_sw_params_dump(swparams, log);
		exit(EXIT_FAILURE);
	}

	if (verbose)
		snd_pcm_dump(handle, log);

	bits_per_sample = snd_pcm_format_physical_width(hwparams.format); // 16 bit = 2 byte
	sampleByteSize = bits_per_sample/8;
	//std::cout << " --- bits_per_sample: " << bits_per_sample << std::endl;  16
	//std::cout << " --- hwparams.channels;: " << hwparams.channels << std::endl; 7
	bits_per_frame = bits_per_sample * hwparams.channels; // 112 bit = 14 byte = 7 sound stream lines or 7 samples , 7samples = 1 Frame
	//std::cout << " --- bits_per_frame: " << bits_per_frame << std::endl; // bits_per_frame = 14 bytes
	// 1 periode = 8 frames  = 112 bytes
	// 1 buffer =  16 periode = 112 bytes * 16 = 1792 bytes

	//periodeBitSize = periode: 5512 bits  //5512 times are 7 channels inside the chunk_Byte = audiobuf  5512 words for each channel
	chunk_bytes = periodeBitSize * bits_per_frame / 8; // periodeBitSize: 5512 byte   chunk_bytes = 77168
	//std::cout << "chunk_bytes:" << chunk_bytes << std::endl;

	audiobuf = (u_char *)realloc(audiobuf, chunk_bytes);
	if (audiobuf == NULL) {
		error(_("not enough memory"));
		exit(EXIT_FAILURE);
	}

	// fprintf(stderr, "real periodeBitSize = %i, frags = %i, total = %i\n", periodeBitSize, setup.buf.block.frags, setup.buf.block.frags * periodeBitSize);

	/* stereo VU-meter isn't always available... */
	if (vumeter == VUMETER_STEREO) {
		if (hwparams.channels != 2 || !interleaved || verbose > 2)
			vumeter = VUMETER_MONO;
	}

	buffer_frames = buffer_size;	/* for position test */
}

 /**
  * capture sound file
  * input:\n
  * \param	char *name, char *namebuf, size_t namelen, int filecount  
  * output:\n
  * \param	int filecount
  * \return	void
  *
  */
int kv3audio::new_capture_file(char *name, char *namebuf, size_t namelen, int filecount)
{
	/* get a copy of the original filename */
	char *s;
	char buf[PATH_MAX+1];

	strncpy(buf, name, sizeof(buf));

	/* separate extension from filename */
	s = buf + strlen(buf);
	while (s > buf && *s != '.' && *s != '/')
		--s;
	if (*s == '.')
		*s++ = 0;
	else if (*s == '/')
		s = buf + strlen(buf);

	/* upon first jump to this if block rename the first file */
	if (filecount == 1) {
		if (*s)
			snprintf(namebuf, namelen, "%s-01.%s", buf, s);
		else
			snprintf(namebuf, namelen, "%s-01", buf);
		remove(namebuf);
		rename(name, namebuf);
		filecount = 2;
	}

	/* name of the current file */
	if (*s)
		snprintf(namebuf, namelen, "%s-%02i.%s", buf, filecount, s);
	else
		snprintf(namebuf, namelen, "%s-%02i", buf, filecount);

	return filecount;
}

/* I/O error handler */
void kv3audio::xrun(void)
{
	snd_pcm_status_t *status;
	int res;
	
	snd_pcm_status_alloca(&status);
	if ((res = snd_pcm_status(handle, status))<0) {
		error(_("status error: %s"), snd_strerror(res));
		exit(EXIT_FAILURE);
	}
	if (snd_pcm_status_get_state(status) == SND_PCM_STATE_XRUN) {
		struct timeval now, diff, tstamp;
		gettimeofday(&now, 0);
		snd_pcm_status_get_trigger_tstamp(status, &tstamp);
		timersub(&now, &tstamp, &diff);
		fprintf(stderr, _("%s!!! (at least %.3f ms long)\n"),
			stream == SND_PCM_STREAM_PLAYBACK ? _("underrun") : _("overrun"),
			diff.tv_sec * 1000 + diff.tv_usec / 1000.0);
		if (verbose) {
			fprintf(stderr, _("Status:\n"));
			snd_pcm_status_dump(status, log);
		}
		if ((res = snd_pcm_prepare(handle))<0) {
			error(_("xrun: prepare error: %s"), snd_strerror(res));
			exit(EXIT_FAILURE);
		}
		return;		/* ok, data should be accepted again */
	} if (snd_pcm_status_get_state(status) == SND_PCM_STATE_DRAINING) {
		if (verbose) {
			fprintf(stderr, _("Status(DRAINING):\n"));
			snd_pcm_status_dump(status, log);
		}
		if (stream == SND_PCM_STREAM_CAPTURE) {
			fprintf(stderr, _("capture stream format change? attempting recover...\n"));
			if ((res = snd_pcm_prepare(handle))<0) {
				error(_("xrun(DRAINING): prepare error: %s"), snd_strerror(res));
				exit(EXIT_FAILURE);
			}
			return;
		}
	}
	if (verbose) {
		fprintf(stderr, _("Status(R/W):\n"));
		snd_pcm_status_dump(status, log);
	}
	error(_("read/write error, state = %s"), snd_pcm_state_name(snd_pcm_status_get_state(status)));
	exit(EXIT_FAILURE);
}


/* I/O suspend handler */
void kv3audio::suspend(void)
{
	int res;

	if (!quiet_mode)
		fprintf(stderr, _("Suspended. Trying resume. ")); fflush(stderr);
	while ((res = snd_pcm_resume(handle)) == -EAGAIN)
		sleep(1);	/* wait until suspend flag is released */
	if (res < 0) {
		if (!quiet_mode)
			fprintf(stderr, _("Failed. Restarting stream. ")); fflush(stderr);
		if ((res = snd_pcm_prepare(handle)) < 0) {
			error(_("suspend: prepare error: %s"), snd_strerror(res));
			exit(EXIT_FAILURE);
		}
	}
	if (!quiet_mode)
		fprintf(stderr, _("Done.\n"));
}

ssize_t kv3audio::pcm_read(u_char *data, size_t rcount)
{
	ssize_t r;
	size_t result = 0;
	size_t count = rcount;

	if (count != periodeBitSize) {
		count = periodeBitSize;
	}

	while (count > 0) {
		/*
		readi_func()
		 Parameters
		 pcm	PCM handle
		 buffer	frames containing buffer
		 size	frames to be read 
		*/
		r = readi_func(handle, data, count);
		if (r == -EAGAIN || (r >= 0 && (size_t)r < count)) {
			snd_pcm_wait(handle, 1000);
		} else if (r == -EPIPE) {
			xrun();
		} else if (r == -ESTRPIPE) {
			suspend();
		} else if (r < 0) {
			error(_("read error: %s"), snd_strerror(r));
			exit(EXIT_FAILURE);
		}
		if (r > 0) {
			if (vumeter)
				compute_max_peak(data, r * hwparams.channels);
			result += r;
			count -= r;
			data += r * bits_per_frame / 8;
		}
	}
	return rcount;
}

void kv3audio::print_vu_meter_mono(int perc, int maxperc)
{
	const int bar_length = 50;
	char line[80];
	int val;

	for (val = 0; val <= perc * bar_length / 100 && val < bar_length; val++)
		line[val] = '#';
	for (; val <= maxperc * bar_length / 100 && val < bar_length; val++)
		line[val] = ' ';
	line[val] = '+';
	for (++val; val <= bar_length; val++)
		line[val] = ' ';
	if (maxperc > 99)
		sprintf(line + val, "| MAX");
	else
		sprintf(line + val, "| %02i%%", maxperc);
	fputs(line, stdout);
	if (perc > 100)
		printf(_(" !clip  "));
}

void kv3audio::print_vu_meter_stereo(int *perc, int *maxperc)
{
	const int bar_length = 35;
	char line[80];
	int c;

	memset(line, ' ', sizeof(line) - 1);
	line[bar_length + 3] = '|';

	for (c = 0; c < 2; c++) {
		int p = perc[c] * bar_length / 100;
		char tmp[4];
		if (p > bar_length)
			p = bar_length;
		if (c)
			memset(line + bar_length + 6 + 1, '#', p);
		else
			memset(line + bar_length - p - 1, '#', p);
		p = maxperc[c] * bar_length / 100;
		if (p > bar_length)
			p = bar_length;
		if (c)
			line[bar_length + 6 + 1 + p] = '+';
		else
			line[bar_length - p - 1] = '+';
		if (maxperc[c] > 99)
			sprintf(tmp, "MAX");
		else
			sprintf(tmp, "%02d%%", maxperc[c]);
		if (c)
			memcpy(line + bar_length + 3 + 1, tmp, 3);
		else
			memcpy(line + bar_length, tmp, 3);
	}
	line[bar_length * 2 + 6 + 2] = 0;
	fputs(line, stdout);
}


void kv3audio::print_vu_meter(signed int *perc, signed int *maxperc)
{
	if (vumeter == VUMETER_STEREO)
		print_vu_meter_stereo(perc, maxperc);
	else
		print_vu_meter_mono(*perc, *maxperc);
}

/* peak handler */
void kv3audio::compute_max_peak(u_char *data, size_t count)
{
	signed int val, max, perc[2], max_peak[2];
	static	int	run = 0;
	size_t ocount = count;
	int	format_little_endian = snd_pcm_format_little_endian(hwparams.format);	
	int ichans, c;

	if (vumeter == VUMETER_STEREO)
		ichans = 2;
	else
		ichans = 1;

	memset(max_peak, 0, sizeof(max_peak));

	
	switch (bits_per_sample) {
	case 8: {
		signed char *valp = (signed char *)data;
		signed char mask = snd_pcm_format_silence(hwparams.format);
		c = 0;
		while (count-- > 0) {
			val = *valp++ ^ mask;
			val = abs(val);
			if (max_peak[c] < val)
				max_peak[c] = val;
			if (vumeter == VUMETER_STEREO)
				c = !c;
		}
		break;
	}
	case 16: {
		signed short *valp = (signed short *)data;
		signed short mask = snd_pcm_format_silence_16(hwparams.format);
		signed short sval;

		count /= 2;
		c = 0;
		while (count-- > 0) {
			if (format_little_endian)
				sval = __le16_to_cpu(*valp);
			else
				sval = __be16_to_cpu(*valp);
			sval = abs(sval) ^ mask;
			if (max_peak[c] < sval)
				max_peak[c] = sval;
			valp++;
			if (vumeter == VUMETER_STEREO)
				c = !c;
		}
		break;
	}
	case 24: {
		unsigned char *valp = data;
		signed int mask = snd_pcm_format_silence_32(hwparams.format);

		count /= 3;
		c = 0;
		while (count-- > 0) {
			if (format_little_endian) {
				val = valp[0] | (valp[1]<<8) | (valp[2]<<16);
			} else {
				val = (valp[0]<<16) | (valp[1]<<8) | valp[2];
			}
			/* Correct signed bit in 32-bit value */
			if (val & (1<<(bits_per_sample-1))) {
				val |= 0xff<<24;	/* Negate upper bits too */
			}
			val = abs(val) ^ mask;
			if (max_peak[c] < val)
				max_peak[c] = val;
			valp += 3;
			if (vumeter == VUMETER_STEREO)
				c = !c;
		}
		break;
	}
	case 32: {
		signed int *valp = (signed int *)data;
		signed int mask = snd_pcm_format_silence_32(hwparams.format);

		count /= 4;
		c = 0;
		while (count-- > 0) {
			if (format_little_endian)
				val = __le32_to_cpu(*valp);
			else
				val = __be32_to_cpu(*valp);
			val = abs(val) ^ mask;
			if (max_peak[c] < val)
				max_peak[c] = val;
			valp++;
			if (vumeter == VUMETER_STEREO)
				c = !c;
		}
		break;
	}
	default:
		if (run == 0) {
			fprintf(stderr, _("Unsupported bit size %d.\n"), (int)bits_per_sample);
			run = 1;
		}
		return;
	}
	max = 1 << (bits_per_sample-1);
	if (max <= 0)
		max = 0x7fffffff;

	for (c = 0; c < ichans; c++) {
		if (bits_per_sample > 16)
			perc[c] = max_peak[c] / (max / 100);
		else
			perc[c] = max_peak[c] * 100 / max;
	}

	if (interleaved && verbose <= 2) {
		static int maxperc[2];
		static time_t t=0;
		const time_t tt=time(NULL);
		if(tt>t) {
			t=tt;
			maxperc[0] = 0;
			maxperc[1] = 0;
		}
		for (c = 0; c < ichans; c++)
			if (perc[c] > maxperc[c])
				maxperc[c] = perc[c];

		putchar('\r');
		print_vu_meter(perc, maxperc);
		fflush(stdout);
	}
	else if(verbose==3) {
		printf(_("Max peak (%li samples): 0x%08x "), (long)ocount, max_peak[0]);
		for (val = 0; val < 20; val++)
			if (val <= perc[0] / 5)
				putchar('#');
			else
				putchar(' ');
		printf(" %i%%\n", perc[0]);
		fflush(stdout);
	}
}

void kv3audio::captureStep(){
	int tostdout=0;		/* boolean which describes output stream */
	int filecount=0;	/* number of files written */
	char namebuf[PATH_MAX+1];
	off64_t count, rest;		/* number of bytes to capture */

	/* get number of bytes to capture */
	count = kv3audio::calc_count();
	//cout << " number of bytes to capture --- count: " << (int)count << endl;
	if (count == 0)
		count = LLONG_MAX;
	/* WAVE-file should be even (I'm not sure), but wasting one byte
	   isn't a problem (this can only be in 8 bit mono) */
	if (count < LLONG_MAX)
		count += count % 2;
	else
		count -= count % 2;

	/* setup sound hardware */
	set_params();

	/////////////
	int counterCh = 0;
		rest = count;
		//rest = 77168; //capture one periode in byte
		cout << " -------------- count: " << count << endl;
		periodeByteSize = chunk_bytes;

		/* capture */
		//cout << " -------------- run ... capture: " << endl;
		fdcount = 0;
		while (rest > 0) {
			size_t c = (rest <= (off64_t)chunk_bytes) ?
				(size_t)rest : chunk_bytes;
			size_t f = c * 8 / bits_per_frame; //bits_per_sample = 2 Byte
			frameByteSize = (int)f;//for visu

			//cout << " -------------- frame in byte:" << f << endl;
			ssize_t pcmReadSize = pcm_read(audiobuf, f);
			if (pcmReadSize != f){
				cout << " BREAK --- void kv3audio::captureStep(): pcmReadSize != (int)f " << endl;
				break;
			}
			///grabb data
			tByte4 convByte4;
			tByte2Word convtoword;
			
			int k = 0;
			int i = 0;
			int imax = kv3audio::frameByteSize;	
					
			if(counterCh > 0){
				while(i < imax){
					convtoword.bytes.msb_0 = audiobuf[i];
					convtoword.bytes.msb_1 = audiobuf[i+1];
					i = i+(2*6);
					/*
					convByte4.bytes.msb_0 = audiobuf[i];
					convByte4.bytes.msb_1 = audiobuf[i+1];
					convByte4.bytes.msb_2 = audiobuf[i+2];
					convByte4.bytes.msb_3 = audiobuf[i+3];
					*/
					//audioFrameBuffer[k].ch0 = convByte4.val;
					audioFrameBuffer[k].ch0 = convtoword.val;

					k++;
				}
			}
			cout << " ------ counterCh: " << counterCh << endl;
			counterCh++;
				
				
			///grabb data
			/*
				tByte2Word convtoword;
				
				int i = 0;
				int imax = kv3audio::frameByteSize;				
				while(i < imax){
					int j = 0;
					int jmax = kv3audio::channels ;	
					while(j<jmax){
						//if((int)audiobuf[j] != 0){cout << (int)audiobuf[j] << endl;}						
							convtoword.bytes.msb_0 = (int)audiobuf[j];
							if(j == 0){
							//int16_t fval = (int16_t)((fval & audiobuf[j]) << 8);
							int16_t fval1 = (int16_t)(audiobuf[j]);
							int16_t fval2 = (int16_t)(audiobuf[j+1]);
							int16_t fval = 0;
							
							if(fval2 == 255){
								fval = fval1 * (-1);
							}
							if((fval2 > 0)&&(fval2<255)){
								//cout << "--- Hit: " << fval2 << endl;
							}
							if(fval2 == 0){
								fval = fval1;
							}
							audioFrameBuffer[i].ch0 = fval;
							//cout << " fval: " << fval << " fval1: " << fval1 << " fval2: " << fval2 << endl;
						//	fval = (int16_t)fval & (int16_t)(audiobuf[j+1]);

							//int valSign = ((audiobuf[j] & 0x80) >> 7);
							//cout << " valSign: " << valSign << " fval: " << fval << " audiobuf[j]: " << (double)audiobuf[j]  << " (audiobuf[j] << 8): " << (double)(audiobuf[j] << 8) << endl;							
								}
								if(j == 1){
									audioFrameBuffer[i].ch1 = (int)convtoword.val;
								}
								if(j == 2){
									audioFrameBuffer[i].ch2 = (int)convtoword.val;
								}
								if(j == 3){
									audioFrameBuffer[i].ch3 = (int)convtoword.val;
								}
								if(j == 4){
									audioFrameBuffer[i].ch4 = (int)convtoword.val;
								}
								if(j == 5){
									audioFrameBuffer[i].ch5 = (int)convtoword.val;
								}
								if(j == 6){
									audioFrameBuffer[i].ch6 = (int)convtoword.val;
								}	
								if(convtoword.val != 0){
									//cout << " i: " << i << " j: " << j << " convtoword.val: " << (int)convtoword.val << endl;	
								}		
								
								const int bitDepth = 16;
								double divisor = pow(2,bitDepth-1);
								//kv3audio::audioBufferCh01[i]  = (audiobuf[i*2+1]*256 + (audiobuf[i*2] & 0xFF))/divisor; // little endian - signed
								//kv3audio::audioBufferCh01[i]  = (audiobuf[i*2+1]*256 + (audiobuf[i*2] - 0x80))/divisor; // little endian - unsigned
								//cout << "kv3audio::audioBufferCh01[i]: " << kv3audio::audioBufferCh01[i] << endl;
								

							//}
						//cout << " i: " << i << " j: " << j << endl;
					j++;
					}
					
				i = i + 14;
				}
*/
			count -= c;
			rest -= c;
			fdcount += c;
			//cout << "rest = " << rest << endl;
		}
}


void kv3audio::capture(char *orig_name)
{
	int tostdout=0;		/* boolean which describes output stream */
	int filecount=0;	/* number of files written */
	char *name = orig_name;	/* current filename */
	char namebuf[PATH_MAX+1];
	off64_t count, rest;		/* number of bytes to capture */

	/* get number of bytes to capture */
	count = kv3audio::calc_count();
	if (count == 0)
		count = LLONG_MAX;
	/* WAVE-file should be even (I'm not sure), but wasting one byte
	   isn't a problem (this can only be in 8 bit mono) */
	if (count < LLONG_MAX)
		count += count % 2;
	else
		count -= count % 2;

    printf("arecord: Recording audio to: %s\n", name);
	/* setup sound hardware */
	set_params();

	/* write to stdout? */
	if (!name || !strcmp(name, "-")) {
		fd = fileno(stdout);
		name = "stdout";
		tostdout=1;
		if (count > fmt_rec_table[file_type].max_filesize)
			count = fmt_rec_table[file_type].max_filesize;
	}

	do {
		/* open a file to write */
		if(!tostdout) {
			/* upon the second file we start the numbering scheme */
			if (filecount) {
				filecount = new_capture_file(orig_name, namebuf,
							     sizeof(namebuf),
							     filecount);
				name = namebuf;
			}

			/* open a new file */
			remove(name);
			if ((fd = open64(name, O_WRONLY | O_CREAT, 0644)) == -1) {
				perror(name);
				exit(EXIT_FAILURE);
			}
			filecount++;
		}

		rest = count;
		if (rest > fmt_rec_table[file_type].max_filesize)
			rest = fmt_rec_table[file_type].max_filesize;

		/* setup sample header */
		if (fmt_rec_table[file_type].start)
			fmt_rec_table[file_type].start(fd, rest);

		/* capture */
		fdcount = 0;
		while (rest > 0 && capture_stop == 0) {
			size_t c = (rest <= (off64_t)chunk_bytes) ?
				(size_t)rest : chunk_bytes;
			size_t f = c * 8 / bits_per_frame;
			if (pcm_read(audiobuf, f) != f)  //read full audio buffer = Frame ?
				break;
			
			if (write(fd, audiobuf, c) != c) {
				perror(name);
				exit(EXIT_FAILURE);
			}
			
			count -= c;
			rest -= c;
			fdcount += c;
		}

		/* finish sample container */
		if (fmt_rec_table[file_type].end && !tostdout) {
			fmt_rec_table[file_type].end(fd);
			fd = -1;
		}

		/* repeat the loop when format is raw without timelimit or
		 * requested counts of data are recorded
		 */
	} while ( ((file_type == FORMAT_RAW && !timelimit) || count > 0) &&
        capture_stop == 0);
    printf("arecord: Stopping capturing audio.\n");
}

////////////////////////
//wav format

// write a WAVE-header
void kv3audio::begin_wave(int fd, size_t cnt)
{
	WaveHeader h;
	WaveFmtBody f;
	WaveChunkHeader cf, cd;
	int bits;
	u_int tmp;
	u_short tmp2;

	// WAVE cannot handle greater than 32bit (signed?) int 
	if (cnt == (size_t)-2)
		cnt = 0x7fffff00;

	bits = 8;
	switch ((unsigned long) hwparams.format) {
	case SND_PCM_FORMAT_U8:
		bits = 8;
		break;
	case SND_PCM_FORMAT_S16_LE:
		bits = 16;
		break;
	case SND_PCM_FORMAT_S32_LE:
        case SND_PCM_FORMAT_FLOAT_LE:
		bits = 32;
		break;
	case SND_PCM_FORMAT_S24_LE:
	case SND_PCM_FORMAT_S24_3LE:
		bits = 24;
		break;
	default:
		error(_("Wave doesn't support %s format..."), snd_pcm_format_name(hwparams.format));
		exit(EXIT_FAILURE);
	}
	h.magic = WAV_RIFF;
	tmp = cnt + sizeof(WaveHeader) + sizeof(WaveChunkHeader) + sizeof(WaveFmtBody) + sizeof(WaveChunkHeader) - 8;
	h.length = LE_INT(tmp);
	h.type = WAV_WAVE;

	cf.type = WAV_FMT;
	cf.length = LE_INT(16);

        if (hwparams.format == SND_PCM_FORMAT_FLOAT_LE)
                f.format = LE_SHORT(WAV_FMT_IEEE_FLOAT);
        else
                f.format = LE_SHORT(WAV_FMT_PCM);
	f.channels = LE_SHORT(hwparams.channels);
	f.sample_fq = LE_INT(hwparams.rate);
#if 0
	tmp2 = (samplesize == 8) ? 1 : 2;
	f.byte_p_spl = LE_SHORT(tmp2);
	tmp = dsp_speed * hwparams.channels * (u_int) tmp2;
#else
	tmp2 = hwparams.channels * snd_pcm_format_physical_width(hwparams.format) / 8;
	f.byte_p_spl = LE_SHORT(tmp2);
	tmp = (u_int) tmp2 * hwparams.rate;
#endif
	f.byte_p_sec = LE_INT(tmp);
	f.bit_p_spl = LE_SHORT(bits);

	cd.type = WAV_DATA;
	cd.length = LE_INT(cnt);

	if (write(fd, &h, sizeof(WaveHeader)) != sizeof(WaveHeader) ||
	    write(fd, &cf, sizeof(WaveChunkHeader)) != sizeof(WaveChunkHeader) ||
	    write(fd, &f, sizeof(WaveFmtBody)) != sizeof(WaveFmtBody) ||
	    write(fd, &cd, sizeof(WaveChunkHeader)) != sizeof(WaveChunkHeader)) {
		error(_("write error"));
		exit(EXIT_FAILURE);
	}
}

//close output
void kv3audio::end_wave(int fd)
{				
	WaveChunkHeader cd;
	off64_t length_seek;
	off64_t filelen;
	u_int rifflen;

	length_seek = sizeof(WaveHeader) +
		      sizeof(WaveChunkHeader) +
		      sizeof(WaveFmtBody);
	cd.type = WAV_DATA;
	cd.length = fdcount > 0x7fffffff ? LE_INT(0x7fffffff) : LE_INT(fdcount);
	filelen = fdcount + 2*sizeof(WaveChunkHeader) + sizeof(WaveFmtBody) + 4;
	rifflen = filelen > 0x7fffffff ? LE_INT(0x7fffffff) : LE_INT(filelen);
	if (lseek64(fd, 4, SEEK_SET) == 4)
		write(fd, &rifflen, 4);
	if (lseek64(fd, length_seek, SEEK_SET) == length_seek)
		write(fd, &cd, sizeof(WaveChunkHeader));
	if (fd != 1)
		close(fd);
}

////////////////////////////////
//int kv3audio::run(char *filename)
int kv3audio::run()
{
    //char *filename = "kv3test.wav";
    capture_stop = 0;
	//char *pcm_name = "default";
	//kv3audio::strDevice
	
	int tmp, err;
	snd_pcm_info_t *info;

	snd_pcm_info_alloca(&info);

	err = snd_output_stdio_attach(&log, stderr, 0);
	assert(err >= 0);

	file_type = FORMAT_DEFAULT;
    stream = SND_PCM_STREAM_CAPTURE;
    file_type = FORMAT_WAVE;
    command = "arecord";
    start_delay = 1;

	periodeBitSize = -1;
    rhwparams.format = file_type == FORMAT_AU ? SND_PCM_FORMAT_S16_BE : SND_PCM_FORMAT_S16_LE;
    rhwparams.rate = kv3audio::rate;
    rhwparams.channels = kv3audio::channels;

	file_type = FORMAT_WAVE;
	err = snd_pcm_open(&handle, kv3audio::strDevice.c_str(), stream, open_mode);
	
	if (err < 0) {
		error(_("audio open error: %s"), snd_strerror(err));
		return 1;
	}

	if ((err = snd_pcm_info(handle, info)) < 0) {
		error(_("info error: %s"), snd_strerror(err));
		return 1;
	}

	if (nonblock) {
		err = snd_pcm_nonblock(handle, 1);
		if (err < 0) {
			error(_("nonblock setting error: %s"), snd_strerror(err));
			return 1;
		}
	}

	periodeBitSize = 1024;
	hwparams = rhwparams;

	audiobuf = (u_char *)malloc(1024);
	if (audiobuf == NULL) {
		error(_("not enough memory"));
		return 1;
	}

    writei_func = snd_pcm_writei;
	readi_func = snd_pcm_readi;
	writen_func = snd_pcm_writen;
	readn_func = snd_pcm_readn;

	if(kv3audio::recordMicroToFile == 1){
		capture(filename);
	}else{
		captureStep();
	}

    if (fmt_rec_table[file_type].end) {
        fmt_rec_table[file_type].end(fd);
        fd = -1;
    }
	stream = (snd_pcm_stream_t)-1;
	if (fd > 1) {
		close(fd);
		fd = -1;
	}
	if (handle) {
		snd_pcm_close(handle);
		handle = NULL;
	}
	//snd_pcm_close(handle);
	//free(audiobuf);
	//snd_output_close(log);
	//snd_config_update_free_global();
	return EXIT_SUCCESS;
}

void kv3audio::getCurrentAudioDevice(std::string &audioDevice){
	audioDevice = kv3audio::strDevice;
}
void kv3audio::getCurrentAudioFormat(unsigned int &audioFormat){
	audioFormat = (unsigned int)file_type;
}
void kv3audio::getCurrentAudioRate(unsigned int &rate){
	rate = kv3audio::rate;
	//44100 48000 88200 96000 192000 352800        2822400 5644800 11289600 22579200
	// kinect max: 384000Hz                 384000
	//https://en.wikipedia.org/wiki/Sampling_(signal_processing)
}
void kv3audio::getCurrentAudioChannels(unsigned int &channels){
	channels = kv3audio::channels;
}
void kv3audio::setAudioDevice(std::string audioDevice){
	kv3audio::strDevice = audioDevice;
}
void kv3audio::setAudioRate(const unsigned int rate){
	kv3audio::rate = rate;
}
void kv3audio::setChannels(const unsigned int channels){
	kv3audio::channels = channels;
}

void kv3audio::setRecordMicroToFile( bool bValue, char *path ){
	kv3audio::recordMicroToFile = bValue;
	kv3audio::filename = path;
}
int kv3audio::getRecordMicroToFile(){
	int retval = 0;
	retval = kv3audio::recordMicroToFile;
	return(retval);
}

void kv3audio::getFrameBuffer(int index, int channel, int &out){
	tFrameBuffer audioFrameBuffer[MAX_FRAMEBUFFER];

	if(channel == 0){
		out = audioFrameBuffer[index].ch0;
	}
	if(channel == 1){
		out = audioFrameBuffer[index].ch1;
	}
	if(channel == 2){
		out = audioFrameBuffer[index].ch2;
	}
	if(channel == 3){
		out = audioFrameBuffer[index].ch3;
	}
	if(channel == 4){
		out = audioFrameBuffer[index].ch4;
	}
	if(channel == 5){
		out = audioFrameBuffer[index].ch5;
	}
	if(channel == 6){
		out = audioFrameBuffer[index].ch6;
	}			
}

int kv3audio::getFrameByteSize(){return(kv3audio::frameByteSize);}
int kv3audio::getPeriodeByteSize(){return(kv3audio::periodeByteSize);}
int kv3audio::getSampleByteSize(){return(kv3audio::sampleByteSize);}