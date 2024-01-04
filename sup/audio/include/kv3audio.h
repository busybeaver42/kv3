/*
 * kv3audio.h
 *
 *  Created on: 24.02.2023
 *      Author: joerg angermayer
 *     Licence: MIT
 */
/**
* @file kv3audio.h
* @brief Implementation of the class functions to access the kinect azure microphone audio data.
* @author Jörg Angermayer\n
* @copyright Licensed under MIT
*
* Projekt : kv3 framework\n
*
* @date 09.02.2023 – first implementation
* @date 24.06.2023 – last update
*
*
* @version 1.0
*************************************************************/
#ifndef KV3_AUDIO_H_
#define KV3_AUDIO_H_

#include <string>
#include <iostream>
#include <stdio.h>
#include <malloc.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <time.h>
#include <locale.h>
#include <alsa/asoundlib.h>
#include <assert.h>
#include <sys/poll.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/signal.h>
#include <asm/byteorder.h>
#include <libintl.h>
#include <endian.h>
#include <byteswap.h>
#include <math.h>

using namespace std;

/* Definitions for Microsoft WAVE format */

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define COMPOSE_ID(a,b,c,d)	((a) | ((b)<<8) | ((c)<<16) | ((d)<<24))
#define LE_SHORT(v)		(v)
#define LE_INT(v)		(v)
#define BE_SHORT(v)		bswap_16(v)
#define BE_INT(v)		bswap_32(v)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define COMPOSE_ID(a,b,c,d)	((d) | ((c)<<8) | ((b)<<16) | ((a)<<24))
#define LE_SHORT(v)		bswap_16(v)
#define LE_INT(v)		bswap_32(v)
#define BE_SHORT(v)		(v)
#define BE_INT(v)		(v)
#else
#error "Wrong endian"
#endif

#define DEFAULT_DEVICE      "plughw:2,0" //"default"   "plughw:2,0" // plughw:card,device
#define DEFAULT_FORMAT		SND_PCM_FORMAT_S16_LE //SND_PCM_FORMAT_U8
#define DEFAULT_RATE        16000//48000
//44100 48000 88200 96000 192000 352800        2822400 5644800 11289600 22579200
// kinect max: 384000Hz                 384000
//https://en.wikipedia.org/wiki/Sampling_(signal_processing)
#define DEFAULT_CHANNELS    7
#define MAX_FRAMEBUFFER 7000 //5512

#define FORMAT_DEFAULT		-1
#define FORMAT_RAW		0
#define FORMAT_VOC		1
#define FORMAT_WAVE		2
#define FORMAT_AU		3

#define WAV_RIFF		COMPOSE_ID('R','I','F','F')
#define WAV_WAVE		COMPOSE_ID('W','A','V','E')
#define WAV_FMT			COMPOSE_ID('f','m','t',' ')
#define WAV_DATA		COMPOSE_ID('d','a','t','a')

/* WAVE fmt block constants from Microsoft mmreg.h header */
#define WAV_FMT_PCM             0x0001
#define WAV_FMT_IEEE_FLOAT      0x0003

/* it's in chunks like .voc and AMIGA iff, but my source say there
   are in only in this combination, so I combined them in one header;
   it works on all WAVE-file I have
 */
typedef struct {
	u_int magic;		/* 'RIFF' */
	u_int length;		/* filelen */
	u_int type;		    /* 'WAVE' */
} WaveHeader;

typedef struct {
	u_short format;		/* see WAV_FMT_* */
	u_short channels;
	u_int sample_fq;	/* frequence of sample */
	u_int byte_p_sec;
	u_short byte_p_spl;	/* samplesize; 1 or 2 bytes */
	u_short bit_p_spl;	/* 8, 12 or 16 bit */
} WaveFmtBody;

typedef struct {
	u_int type;		/* 'data' */
	u_int length;   /* samplecount */
} WaveChunkHeader;


typedef struct {
	int32_t ch0;
	int32_t ch1;
	int32_t ch2;
	int32_t ch3;
	int32_t ch4;
	int32_t ch5;
	int32_t ch6;
	/*
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;
	int16_t ch6;
	*/
}tFrameBuffer;




typedef union {
	int16_t val;
   struct {
		int8_t msb_0;
		int8_t msb_1;
   } bytes;
} tByte2Word;

typedef union {
	int32_t val;
   struct {
		u_char msb_0;
		u_char msb_1;
		u_char msb_2;
		u_char msb_3;
   } bytes;
} tByte4;



#define _(msgid) gettext (msgid)
#define gettext_noop(msgid) msgid
#define N_(msgid) gettext_noop (msgid)

#ifndef LLONG_MAX
#define LLONG_MAX    9223372036854775807LL
#endif

	static struct {
		snd_pcm_format_t format;
		unsigned int channels;
		unsigned int rate;
	} hwparams, rhwparams;

	enum {
		VUMETER_NONE,
		VUMETER_MONO,
		VUMETER_STEREO
	};

	enum {
		OPT_VERSION = 1,
		OPT_PERIOD_SIZE,
		OPT_BUFFER_SIZE,
		OPT_DISABLE_RESAMPLE,
		OPT_DISABLE_CHANNELS,
		OPT_DISABLE_FORMAT,
		OPT_DISABLE_SOFTVOL,
		OPT_TEST_POSITION
	};


class kv3audio {
	public:
		kv3audio();
		~kv3audio();
		static tFrameBuffer audioFrameBuffer[MAX_FRAMEBUFFER]; //5512
	private:
		static double audioBufferCh01[MAX_FRAMEBUFFER];
		static char *command;
		static snd_pcm_t *handle;
		static std::string strDevice;
		static snd_pcm_format_t format;
		static unsigned int channels;
		static unsigned int rate;

		static int frameByteSize;
		static int periodeByteSize;
		static int sampleByteSize;
		
		static bool recordMicroToFile;
		static int timelimit;
		static int quiet_mode;
		static int file_type;
		static int open_mode;
		static int capture_stop;
		static snd_pcm_stream_t stream;
		static int interleaved;
		static int nonblock;
		static u_char *audiobuf;
		static snd_pcm_uframes_t periodeBitSize;
		static unsigned period_time;
		static unsigned buffer_time;
		static snd_pcm_uframes_t period_frames;
		static snd_pcm_uframes_t buffer_frames;
		static int avail_min;
		static int start_delay;
		static int stop_delay;
		static int verbose;
		static int vumeter;
		static int buffer_pos;
		static size_t bits_per_sample;
		static size_t bits_per_frame;
		static size_t chunk_bytes;
		static snd_output_t *log;
		static int fd;
		static off64_t pbrec_count;
		static off64_t fdcount;
		static int vocmajor, vocminor;
		static char *filename;


		/* needed prototypes */

		//main:  run("kv3test.wav"); 

		//capture
			static off64_t calc_count(void);
			static void set_params(void);
			static int new_capture_file(char *name, char *namebuf, size_t namelen, int filecount);			
			//pcmread
				static void xrun(void);
				static void suspend(void);
			static ssize_t pcm_read(u_char *data, size_t rcount);				
				//compute_max_peak
						static void print_vu_meter_mono(int perc, int maxperc);
						static void print_vu_meter_stereo(int *perc, int *maxperc);
					static void print_vu_meter(signed int *perc, signed int *maxperc);
				static void compute_max_peak(u_char *data, size_t count);
		static void capture(char *filename);
		static void captureStep();

		public:
		static void begin_wave(int fd, size_t count);
		static void end_wave(int fd);
		int run();
		//int run(char *filename);

		//set / get
		void getCurrentAudioDevice(std::string &audioDevice);
		void getCurrentAudioFormat(unsigned int &audioFormat);
		void getCurrentAudioRate(unsigned int &rate);
		void getCurrentAudioChannels(unsigned int &channels);
		void setAudioRate(const unsigned int rate);
		void setChannels(const unsigned int channels);
		void setAudioDevice(std::string audioDevice);
		void getFrameBuffer(int index, int channel, int &out);
		void setRecordMicroToFile( bool bValue, char *path );
		int getRecordMicroToFile();

		int getFrameByteSize();
		int getPeriodeByteSize();
		int getSampleByteSize();

};


#ifndef fmt_capture
#define fmt_capture
	struct fmt_capture {
		void (*start) (int fd, size_t count);
		void (*end) (int fd);
		char *what;
		long long max_filesize;
	} fmt_rec_table[] = {
		{	NULL,		NULL,		N_("raw data"),		LLONG_MAX },
		{	NULL,	NULL,	N_("VOC"),		16000000LL },
		{	kv3audio::begin_wave,	kv3audio::end_wave,	N_("WAVE"),		2147483648LL },
		{	NULL,	NULL,		N_("Sparc Audio"),	LLONG_MAX }
	};
#endif



	//define the error message
	#if __GNUC__ > 2 || (__GNUC__ == 2 && __GNUC_MINOR__ >= 95)
	#define error(...) do {\
		fprintf(stderr, "%s: %s:%d: ", command, __FUNCTION__, __LINE__); \
		fprintf(stderr, __VA_ARGS__); \
		putc('\n', stderr); \
	} while (0)
	#else
	#define error(args...) do {\
		fprintf(stderr, "%s: %s:%d: ", command, __FUNCTION__, __LINE__); \
		fprintf(stderr, ##args); \
		putc('\n', stderr); \
	} while (0)
	#endif

#endif /* KV3_AUDIO_H_ */