#include "audioMeasurement.h"

//STATIC 
	char *audioMeasurement::device = "plughw:2,0";//"plughw:1,0";// "plughw:2,0" //"default";
	short audioMeasurement::buffer[8*1024];
	int audioMeasurement::buffer_size = sizeof(buffer)>>1;
    snd_pcm_t *audioMeasurement::handle_capture;
    snd_pcm_sframes_t audioMeasurement::frames;
    double audioMeasurement::k = 0.45255;
	double audioMeasurement::Pvalue = 0;
	double audioMeasurement::peak = 0;    
    int grayValue = 40;
    int audioMeasurement::counterpValueBuffer = 0;
    int audioMeasurement::pValueBuffer[];
    #ifdef USE_CV
    cv::Mat audioMeasurement::audioMeasurementImg(MAX_PVALUE_Y_BUFFER, MAX_PVALUE_X_BUFFER, CV_8UC3,Scalar(grayValue, grayValue, grayValue));
    #endif	
//CONST
    double offsetPvalue = 24.00;

audioMeasurement::audioMeasurement(){
    init();
}
audioMeasurement::~audioMeasurement(){
    snd_pcm_close(handle_capture);
}

float audioMeasurement::getPvalue(){return((float)audioMeasurement::Pvalue);}
float audioMeasurement::getPvalueMax(){return((float)audioMeasurement::peak);}
int audioMeasurement::getIntPvalue(){return((int)round(audioMeasurement::Pvalue));}
int audioMeasurement::getIntPvalueMax(){return((int)round(audioMeasurement::peak));}
#ifdef USE_CV
cv::Mat audioMeasurement::getMeasurementImage(){return(audioMeasurement::audioMeasurementImg);}
#endif

 /**
  * init / open pcm audio channel
  * \return	void
  *
  */
void audioMeasurement::init(){
    err = snd_pcm_open(&handle_capture, device, SND_PCM_STREAM_CAPTURE, 0);
    if(err < 0) {
        printf("Capture open error: %s\n", snd_strerror(err));
        exit(EXIT_FAILURE);
    }

    err = snd_pcm_set_params(handle_capture,
                             SND_PCM_FORMAT_S16_LE,
                             SND_PCM_ACCESS_RW_INTERLEAVED,
                             1,
                             48000,
                             1,
                             500000);

    if(err < 0) {
        printf("Capture open error: %s\n", snd_strerror(err));
        exit(EXIT_FAILURE);
    }
}

#ifdef USE_CV
 /**
  * update measurement image
  * \return	void
  *
  */
void audioMeasurement::updateImage(){
    const int thickness = 1;
    int x, y;
    int maxy = MAX_PVALUE_Y_BUFFER;
    int i=0;
    int imax=MAX_PVALUE_X_BUFFER;
    audioMeasurementImg.setTo(cv::Scalar(grayValue,grayValue,grayValue));
    while(i<imax){
        x = i;
        y = maxy-pValueBuffer[i];
        Point startPoint(x, maxy);
        Point endPoint(x, y);
        line(audioMeasurementImg, startPoint, endPoint, Scalar(0, 200, 0), thickness, LINE_8);
    i++;
    }

    const float fontScale = 0.5;
    std::stringstream strStreamdB;
    strStreamdB << std::fixed << std::setprecision(2) << audioMeasurement::Pvalue;
    string strdB = "dB: " + strStreamdB.str();
    cv::putText(audioMeasurementImg, 
        strdB, //text
        cv::Point(imax/2-26, 26),
        cv::FONT_HERSHEY_SIMPLEX,
        fontScale,
        CV_RGB(118, 185, 200),
        1);
}
#endif

 /**
  * update audio data
  * \return	void
  *
  */
void audioMeasurement::update(){
    frames = snd_pcm_readi(handle_capture, buffer, buffer_size);

    if(frames < 0) {
        // Try to recover
        frames = snd_pcm_recover(handle_capture, frames, 0);
        if(frames < 0) {
            //printf("snd_pcm_readi failed: %s\n", snd_strerror(err));
            return;
        }
        if(frames > 0 && frames < (long)buffer_size) {
            //printf("Short read (expected %li, read %li)\n", (long)buffer_size, frames);
            return;
        }
    }

    // Successfully read, calculate dB and update peak value
    Pvalue = rms(buffer) * k;
    if(Pvalue > peak){peak = Pvalue;}

    int i=MAX_PVALUE_X_BUFFER-1;
   // int imax=MAX_PVALUE_X_BUFFER-1;
    while(i>=0){
        pValueBuffer[i+1] = pValueBuffer[i];
    i--;
    }
    pValueBuffer[0] = (int)round(Pvalue);
    ///std::cout << "pValueBuffer[0]: " << pValueBuffer[0] << std::endl;
    #ifdef USE_CV
    updateImage();   
    #endif
}

 /**
  * run audio data
  * \return	void
  *
  */

void audioMeasurement::run(){
        frames = snd_pcm_readi(handle_capture, buffer, buffer_size);

        if(frames < 0) {
            // Try to recover
            frames = snd_pcm_recover(handle_capture, frames, 0);
            if(frames < 0) {
                printf("snd_pcm_readi failed: %s\n", snd_strerror(err));
            }
        }

        if(frames > 0 && frames < (long)buffer_size) {
            printf("Short read (expected %li, read %li)\n", (long)buffer_size, frames);
        }
        
        // Successfully read, calculate dB and update peak value
        Pvalue = rms(buffer) * k;
        Pvalue = Pvalue - offsetPvalue;
        if(Pvalue > peak){peak = Pvalue;}
        show(Pvalue, peak);
}

 /**
  * rms from square sum of the buffer
  * input:\n
  * \param	short *buffer  
  * \return	double result
  *
  */
double audioMeasurement::rms(short *buffer)
{
    int i;
    long int square_sum = 0.0;
    for(i=0; i<buffer_size; i++)
        square_sum += (buffer[i] * buffer[i]);

    double result = sqrt(square_sum/buffer_size);
    return result;
}

 /**
  * show pValue
  * input:\n
  * \param	int Pvalue, int peak
  * \return	void
  *
  */
void audioMeasurement::show(int Pvalue, int peak)
{
    int j;
    int dB;
    int dBpeak;

    for(j=0; j<13; j++)
        printf("\b");
    fflush(stdout);

    if(Pvalue > 0)
    {
        dB = (int)20 * log10(Pvalue);
        printf("dB=%d,", dB);
    }
    else
        printf("dB=--,");

    if(peak > 0)
    {
        dBpeak = (int)20 * log10(peak);
        printf("Peak=%d", dBpeak);
    }
    else
        printf("Peak=--", dB, peak);

    fflush(stdout);
}