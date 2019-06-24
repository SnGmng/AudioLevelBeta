/* Copyright (C) 2014 Rainmeter Project Developers
*
* This Source Code Form is subject to the terms of the GNU General Public
* License; either version 2 of the License, or (at your option) any later
* version. If a copy of the GPL was not distributed with this file, You can
* obtain one at <https://www.gnu.org/licenses/gpl-2.0.html>. */

#include <Windows.h>
#include <cstdio>
#include <AudioClient.h>
#include <AudioPolicy.h>
#include <MMDeviceApi.h>
#include <FunctionDiscoveryKeys_devpkey.h>
#include <VersionHelpers.h>

#include <cmath>
#include <cassert>
#include <vector>

#include <thread>
#include <avrt.h>
#pragma comment(lib, "Avrt.lib")

#include "../../API/RainmeterAPI.h"

#include "kiss_fft130/kiss_fftr.h"

// Overview: Audio level measurement from the Window Core Audio API
// See: http://msdn.microsoft.com/en-us/library/windows/desktop/dd370800%28v=vs.85%29.aspx

// Sample skin:
/*
[mAudio_Raw]
Measure=Plugin
Plugin=AudioLevel.dll
Port=Output

[mAudio_RMS_L]
Measure=Plugin
Plugin=AudioLevel.dll
Parent=mAudio_Raw
Type=RMS
Channel=L

[mAudio_RMS_R]
Measure=Plugin
Plugin=AudioLevel.dll
Parent=mAudio_Raw
Type=RMS
Channel=R
*/

#define WINDOWS_BUG_WORKAROUND	1
#define TWOPI					(2 * 3.14159265358979323846)
#define EXIT_ON_ERROR(hres)		if (FAILED(hres)) { goto Exit; }
#define SAFE_RELEASE(p)			if ((p) != NULL) { (p)->Release(); (p) = NULL; }
#define CLAMP01(x)				max(0.0, min(1.0, (x)))
#define MSG_UPDATE				L"!UpdateMeasure Audio"

struct Measure
{
	enum Port
	{
		PORT_OUTPUT,
		PORT_INPUT,
	};

	enum Channel
	{
		CHANNEL_FL,
		CHANNEL_FR,
		CHANNEL_C,
		CHANNEL_LFE,
		CHANNEL_BL,
		CHANNEL_BR,
		CHANNEL_SL,
		CHANNEL_SR,
		CHANNEL_SUM,
		MAX_CHANNELS
	};

	enum Type
	{
		TYPE_RMS,
		TYPE_PEAK,
		TYPE_FFT,
		TYPE_WAVE,
		TYPE_BAND,
		TYPE_WAVEBAND,
		TYPE_FFTFREQ,
		TYPE_BANDFREQ,
		TYPE_FORMAT,
		TYPE_DEV_STATUS,
		TYPE_DEV_NAME,
		TYPE_DEV_ID,
		TYPE_DEV_LIST,
		TYPE_BUFFERSTATUS,
		// ... //
		NUM_TYPES
	};

	enum Format
	{
		FMT_INVALID,
		FMT_PCM_S16,
		FMT_PCM_F32,
		// ... //
		NUM_FORMATS
	};

	struct BandInfo
	{
		float freq;
		float x;
	};

	Port					m_port;						// port specifier (parsed from options)
	Channel					m_channel;					// channel specifier (parsed from options)
	Type					m_type;						// data type specifier (parsed from options)
	Format					m_format;					// format specifier (detected in init)
	int						m_envRMS[2];				// RMS attack/decay times in ms (parsed from options)
	int						m_envPeak[2];				// peak attack/decay times in ms (parsed from options)
	int						m_envFFT[2];				// FFT attack/decay times in ms (parsed from options)
	int						m_fftSize;					// size of FFT (parsed from options)
	int						m_fftBufferSize;			// size of FFT with zero-padding (parsed from options)
	int						m_fftIdx;					// FFT index to retrieve (parsed from options)
	int						m_waveIdx;					// WAVE index to retrieve (parsed from options)
	int						m_nBands;					// number of frequency bands (parsed from options)
	int						m_bandIdx;					// band index to retrieve (parsed from options)
	int                     m_smoothing;                // smoothing level (parsed from options)
	int						m_waveSize;					// size of WAVE (parsed from options)
	int						m_ringBufferSize;			// size of the ring buffer for FFT and WAVE
	UINT32					m_nFramesNext;				// number of frames obtained on the UpdateParent call
	UINT32					m_nSilentFrames;			// number of silent frames, used to calculate when to stop updating
	double					m_gainRMS;					// RMS gain (parsed from options)
	double					m_gainPeak;					// peak gain (parsed from options)
	double					m_freqMin;					// min freq for band measurement
	double					m_freqMax;					// max freq for band measurement
	double					m_sensitivity;				// dB range for FFT/Band return values (parsed from options)
	Measure*				m_parent;					// parent measure, if any
	void*					m_skin;						// skin pointer
	void*					m_rm;						// rainmeter pointer
	LPCWSTR					m_rmName;					// measure name
	IMMDeviceEnumerator*	m_enum;						// audio endpoint enumerator
	IMMDevice*				m_dev;						// audio endpoint device
	WAVEFORMATEX			m_wfxR;						// audio format request info
	WAVEFORMATEX*			m_wfx;						// audio format info
	IAudioClient*			m_clAudio;					// audio client instance
	IAudioCaptureClient*	m_clCapture;				// capture client instance
	IAudioClient*			m_clBugAudio;				// audio client for loopback events
#if (WINDOWS_BUG_WORKAROUND)
	IAudioRenderClient*		m_clBugRender;				// render client for dummy silent channel
#endif
	HANDLE					m_hReadyEvent;				// buffer-event handle to receive notifications
	HANDLE					m_hStopEvent;				// skin closed handle to receive notifications
	HANDLE					m_hTask;					// Multimedia Class Scheduler Service task
	WCHAR					m_reqID[64];				// requested device ID (parsed from options)
	WCHAR					m_devName[64];				// device friendly name (detected in init)
	float					m_kRMS[2];					// RMS attack/decay filter constants
	float					m_kPeak[2];					// peak attack/decay filter constants
	float					m_kFFT[2];					// FFT attack/decay filter constants
	float*					m_bufChunk;					// buffer for latest data chunk copy
	float					m_rms[MAX_CHANNELS];		// current RMS levels
	float					m_peak[MAX_CHANNELS];		// current peak levels
	kiss_fftr_cfg			m_fftCfg;					// FFT states for each channel
	float*					m_ringBuffer;				// ring buffer for audio data
	float*					m_fftOut;					// buffer for FFT output
	float*					m_fftKWdw;					// window function coefficients
	float*					m_ringBufOut;				// buffer for audio data from the ring buffer
	kiss_fft_cpx*			m_fftTmpOut;				// temp FFT processing buffer
	int						m_ringBufW;					// write index for input ring buffers
	float*					m_bandFreq;					// buffer of band max frequencies
	float*					m_bandOut;					// buffer of band values
	float*					m_bandTmpOut;               // temp buffer of band values
	float*					m_waveBandOut;				// buffer of wave values
	float*					m_waveOut;					// temp buffer of wave values
	float*					m_waveBandTmpOut;			// 2nd temp buffer of wave values
	float					m_df;						// delta freqency between two bins
	float					m_dw;						// delta waveform values between two bands
	float					m_fftScalar;				// FFT scalar
	float					m_bandScalar;				// band scalar
	float					m_waveScalar;				// wave scalar
	float					m_smoothingScalar;			// smoothing scalar

	Measure() :
		m_port(PORT_OUTPUT),
		m_channel(CHANNEL_SUM),
		m_type(TYPE_RMS),
		m_format(FMT_INVALID),
		m_fftSize(0),
		m_fftBufferSize(0),
		m_fftIdx(-1),
		m_fftScalar(0),
		m_nBands(0),
		m_bandIdx(-1),
		m_bandScalar(0),
		m_df(0),
		m_dw(0),
		m_smoothing(0),
		m_smoothingScalar(0),
		m_waveSize(0),
		m_waveIdx(0),
		m_waveScalar(0),
		m_ringBufferSize(0),
		m_nFramesNext(0),
		m_nSilentFrames(0),
		m_gainRMS(1.0),
		m_gainPeak(1.0),
		m_freqMin(20.0),
		m_freqMax(20000.0),
		m_sensitivity(0.0),
		m_parent(NULL),
		m_skin(NULL),
		m_rm(NULL),
		m_rmName(NULL),
		m_enum(NULL),
		m_dev(NULL),
		m_wfxR({ 0 }),
		m_wfx(NULL),
		m_clAudio(NULL),
		m_clCapture(NULL),
		m_clBugAudio(NULL),
#if (WINDOWS_BUG_WORKAROUND)
		m_clBugRender(NULL),
#endif
		m_hReadyEvent(NULL),
		m_hStopEvent(NULL),
		m_hTask(NULL),
		m_fftKWdw(NULL),
		m_ringBufOut(NULL),
		m_fftTmpOut(NULL),
		m_ringBufW(0),
		m_bandFreq(NULL)
	{
		m_envRMS[0] = 300;
		m_envRMS[1] = 300;
		m_envPeak[0] = 50;
		m_envPeak[1] = 2500;
		m_envFFT[0] = 300;
		m_envFFT[1] = 300;
		m_reqID[0] = '\0';
		m_devName[0] = '\0';
		m_kRMS[0] = 0.0f;
		m_kRMS[1] = 0.0f;
		m_kPeak[0] = 0.0f;
		m_kPeak[1] = 0.0f;
		m_kFFT[0] = 0.0f;
		m_kFFT[1] = 0.0f;
		m_fftCfg = NULL;
		m_bufChunk = NULL;
		m_ringBuffer = NULL;
		m_fftOut = NULL;
		m_bandOut = NULL;
		m_bandTmpOut = NULL;
		m_waveBandOut = NULL;
		m_waveOut = NULL;
		m_waveBandTmpOut = NULL;

		for (int iChan = 0; iChan < MAX_CHANNELS; ++iChan)
		{
			m_rms[iChan] = 0.0;
			m_peak[iChan] = 0.0;
		}
	}

	HRESULT DeviceInit();
	void DeviceRelease();
	HRESULT UpdateParent();

	void DoCaptureLoop();
};

float pcmScalar = 1.0f / 0x7fff;

const CLSID CLSID_MMDeviceEnumerator = __uuidof(MMDeviceEnumerator);
const IID IID_IMMDeviceEnumerator = __uuidof(IMMDeviceEnumerator);
const IID IID_IAudioClient = __uuidof(IAudioClient);
//const IID IID_IAudioClient3 = __uuidof(IAudioClient3);
const IID IID_IAudioCaptureClient = __uuidof(IAudioCaptureClient);
const IID IID_IAudioRenderClient = __uuidof(IAudioRenderClient);

std::vector<Measure*> s_parents;

void Measure::DoCaptureLoop() 
{
	// register thread with MMCSS
	DWORD nTaskIndex = 0;
	m_hTask = AvSetMmThreadCharacteristics(L"Pro Audio", &nTaskIndex);
	if (!(m_hTask && AvSetMmThreadPriority(m_hTask, AVRT_PRIORITY_CRITICAL)))
	{
		DWORD dwErr = GetLastError();
		RmLog(LOG_WARNING, L"Failed to start multimedia task.");
		return;
	}

	HANDLE waitArray[2] = { m_hReadyEvent, m_hStopEvent };

	while (1)
	{
		if (WAIT_OBJECT_0 != WaitForMultipleObjects(ARRAYSIZE(waitArray), waitArray, FALSE, INFINITE))
			return;

		// update parent seperated from measure update function
		HRESULT hr = UpdateParent();

		switch (hr)
		{
		case S_OK:
			// everything is fine, update measures
			RmExecute(m_skin, MSG_UPDATE);
			break;
		case S_FALSE:
			// silence detected, no need to update
			break;
		case AUDCLNT_E_BUFFER_ERROR:
		case AUDCLNT_E_DEVICE_INVALIDATED:
		case AUDCLNT_E_SERVICE_NOT_RUNNING:
			// error detected, release device
			DeviceRelease();
			break;
		}
	}
}

/**
* Create and initialize a measure instance.  Creates WASAPI loopback
* device if not a child measure.
*
* @param[out]	data			Pointer address in which to return measure instance.
* @param[in]	rm				Rainmeter context.
*/
PLUGIN_EXPORT void Initialize(void** data, void* rm)
{
	Measure* m = new Measure;
	m->m_skin = RmGetSkin(rm);
	m->m_rmName = RmGetMeasureName(rm);
	*data = m;

	// parse parent specifier, if appropriate
	LPCWSTR parentName = RmReadString(rm, L"Parent", L"");
	if (*parentName)
	{
		// match parent using measure name and skin handle
		std::vector<Measure*>::const_iterator iter = s_parents.begin();
		for (; iter != s_parents.end(); ++iter)
		{
			if (_wcsicmp((*iter)->m_rmName, parentName) == 0 &&
				(*iter)->m_skin == m->m_skin &&
				!(*iter)->m_parent)
			{
				m->m_parent = (*iter);

				return;
			}
		}

		RmLogF(rm, LOG_ERROR, L"Couldn't find Parent measure '%s'.", parentName);
	}

	// this is a parent measure - add it to the global list
	s_parents.push_back(m);

	// parse port specifier
	LPCWSTR port = RmReadString(rm, L"Port", L"");
	if (port && *port)
	{
		if (_wcsicmp(port, L"Output") == 0)
		{
			m->m_port = Measure::PORT_OUTPUT;
		}
		else if (_wcsicmp(port, L"Input") == 0)
		{
			m->m_port = Measure::PORT_INPUT;
		}
		else
		{
			RmLogF(rm, LOG_ERROR, L"Invalid Port '%s', must be one of: Output or Input.", port);
		}
	}

	// create the enumerator
	EXIT_ON_ERROR(CoCreateInstance(CLSID_MMDeviceEnumerator, NULL, CLSCTX_ALL, IID_IMMDeviceEnumerator, (void**)& m->m_enum));

	// parse requested device ID (optional)
	LPCWSTR reqID = RmReadString(rm, L"ID", L"");
	if (reqID)
	{
		_snwprintf_s(m->m_reqID, _TRUNCATE, L"%s", reqID);
	}

	// if a specific ID was requested, search for that one, otherwise get the default
	if (*m->m_reqID)
	{
		HRESULT hr = m->m_enum->GetDevice(m->m_reqID, &m->m_dev);
		if (hr != S_OK)
		{
			WCHAR msg[256];
			_snwprintf_s(msg, _TRUNCATE, L"Audio %s device '%s' not found (error 0x%08x).",
				m->m_port == Measure::PORT_OUTPUT ? L"output" : L"input", m->m_reqID, hr);

			RmLog(LOG_WARNING, msg);
		}
	}
	else
	{
		EXIT_ON_ERROR(m->m_enum->GetDefaultAudioEndpoint(m->m_port == Measure::PORT_OUTPUT ? eRender : eCapture, eConsole, &m->m_dev));
	}

	// init the device (if it fails, log debug message and quit)
	if (m->DeviceInit() == S_OK)
	{
		// create separate thread with event-driven update loop
		std::thread thread(&Measure::DoCaptureLoop, m);
		thread.detach();

		return;
	}

Exit:

	SAFE_RELEASE(m->m_enum);
}


/**
* Destroy the measure instance.
*
* @param[in]	data			Measure instance pointer.
*/
PLUGIN_EXPORT void Finalize(void* data)
{
	Measure* m = (Measure*)data;

	SetEvent(m->m_hStopEvent);

	m->DeviceRelease();
	SAFE_RELEASE(m->m_enum);

	if (!m->m_parent)
	{
		std::vector<Measure*>::iterator iter = std::find(s_parents.begin(), s_parents.end(), m);
		s_parents.erase(iter);
	}

	delete m;
}


/**
* (Re-)parse parameters from .ini file.
*
* @param[in]	data			Measure instance pointer.
* @param[in]	rm				Rainmeter context.
* @param[out]	maxValue		?
*/
PLUGIN_EXPORT void Reload(void* data, void* rm, double* maxValue)
{
	static const LPCWSTR s_typeName[Measure::NUM_TYPES] =
	{
		L"RMS",								// TYPE_RMS
		L"Peak",							// TYPE_PEAK
		L"FFT",								// TYPE_FFT
		L"Wave",							// TYPE_WAVE
		L"Band",							// TYPE_BAND
		L"WaveBand",						// TYPE_WAVEBAND
		L"FFTFreq",							// TYPE_FFTFREQ
		L"BandFreq",						// TYPE_BANDFREQ
		L"Format",							// TYPE_FORMAT
		L"DeviceStatus",					// TYPE_DEV_STATUS
		L"DeviceName",						// TYPE_DEV_NAME
		L"DeviceID",						// TYPE_DEV_ID
		L"DeviceList",						// TYPE_DEV_LIST
		L"BufferStatus"						// TYPE_BUFFERSTATUS
	};

	static const LPCWSTR s_chanName[Measure::MAX_CHANNELS][3] =
	{
		{ L"L",		L"FL",		L"0", },	// CHANNEL_FL
		{ L"R",		L"FR",		L"1", },	// CHANNEL_FR
		{ L"C",		L"",		L"2", },	// CHANNEL_C
		{ L"LFE",	L"Sub",		L"3", },	// CHANNEL_LFE
		{ L"BL",	L"",		L"4", },	// CHANNEL_BL
		{ L"BR",	L"",		L"5", },	// CHANNEL_BR
		{ L"SL",	L"",		L"6", },	// CHANNEL_SL
		{ L"SR",	L"",		L"7", },	// CHANNEL_SR
		{ L"Sum",	L"Avg",		L"", },		// CHANNEL_SUM
	};

	Measure* m = (Measure*)data;

	// parse data type
	LPCWSTR type = RmReadString(rm, L"Type", L"");
	if (*type)
	{
		int iType;
		for (iType = 0; iType < Measure::NUM_TYPES; ++iType)
		{
			if (_wcsicmp(type, s_typeName[iType]) == 0)
			{
				m->m_type = (Measure::Type)iType;
				break;
			}
		}

		if (!(iType < Measure::NUM_TYPES))
		{
			WCHAR msg[512];
			WCHAR* d = msg;
			d += _snwprintf_s(d, (sizeof(msg) + (UINT32)msg - (UINT32)d) / sizeof(WCHAR), _TRUNCATE,
				L"Invalid Type '%s', must be one of:", type);

			for (int i = 0; i < Measure::NUM_TYPES; ++i)
			{
				d += _snwprintf_s(d, (sizeof(msg) + (UINT32)msg - (UINT32)d) / sizeof(WCHAR), _TRUNCATE,
					L"%s%s%s", i ? L", " : L" ", i == (Measure::NUM_TYPES - 1) ? L"or " : L"", s_typeName[i]);
			}

			d += _snwprintf_s(d, (sizeof(msg) + (UINT32)msg - (UINT32)d) / sizeof(WCHAR), _TRUNCATE, L".");
			RmLogF(rm, LOG_ERROR, msg);
		}
	}

	// parse channel specifier
	LPCWSTR channel = RmReadString(rm, L"Channel", L"");
	if (*channel)
	{
		bool found = false;
		for (int iChan = 0; iChan <= Measure::CHANNEL_SUM && !found; ++iChan)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (_wcsicmp(channel, s_chanName[iChan][j]) == 0)
				{
					m->m_channel = (Measure::Channel)iChan;
					found = true;
					break;
				}
			}
		}

		if (!found)
		{
			WCHAR msg[512];
			WCHAR* d = msg;
			d += _snwprintf_s(d, (sizeof(msg) + (UINT32)msg - (UINT32)d) / sizeof(WCHAR), _TRUNCATE,
				L"Invalid Channel '%s', must be an integer between 0 and %d, or one of:", channel, Measure::MAX_CHANNELS - 2);

			for (int i = 0; i <= Measure::CHANNEL_SUM; ++i)
			{
				d += _snwprintf_s(d, (sizeof(msg) + (UINT32)msg - (UINT32)d) / sizeof(WCHAR), _TRUNCATE,
					L"%s%s%s", i ? L", " : L" ", i == Measure::CHANNEL_SUM ? L"or " : L"", s_chanName[i][0]);
			}

			d += _snwprintf_s(d, (sizeof(msg) + (UINT32)msg - (UINT32)d) / sizeof(WCHAR), _TRUNCATE, L".");
			RmLogF(rm, LOG_ERROR, msg);
		}
	}

	// parse envelope, fft and band values on parents only
	if (!m->m_parent)
	{
		int fftSize = RmReadInt(rm, L"FFTSize", m->m_fftSize);
		int fftBufferSize = max(m->m_fftSize, RmReadInt(rm, L"FFTBufferSize", m->m_fftBufferSize));
		int nBands = RmReadInt(rm, L"Bands", m->m_nBands);
		int smoothing = max(0, RmReadInt(rm, L"Smoothing", m->m_smoothing));
		int freqMin = max(0.0, RmReadDouble(rm, L"FreqMin", m->m_freqMin));
		int freqMax = max(0.0, RmReadDouble(rm, L"FreqMax", m->m_freqMax));
		int waveSize = RmReadInt(rm, L"WAVESize", m->m_waveSize);

		// if one of these values changed, reinitialize
		if (m->m_fftSize != fftSize ||
			m->m_fftBufferSize != fftBufferSize ||
			m->m_freqMin != freqMin ||
			m->m_freqMax != freqMax ||
			m->m_waveSize != waveSize ||
			m->m_nBands != nBands ||
			m->m_smoothing != smoothing)
		{
			// initialize FFT data
			if (m->m_fftSize < 0 || m->m_fftSize & 1)
			{
				RmLogF(rm, LOG_ERROR, L"Invalid FFTSize %ld: must be an even integer >= 0. (powers of 2 work best)", fftSize);
				fftSize = 0;
			}
			m->m_fftSize = fftSize;
			m->m_fftBufferSize = fftBufferSize;

			// initialize WAVE data
			if (m->m_waveSize < 0 || m->m_waveSize & 1)
			{
				RmLogF(rm, LOG_ERROR, L"Invalid WAVESize %ld: must be an even integer >= 0.", waveSize);
				waveSize = 0;
			}
			m->m_waveSize = waveSize;

			m->m_ringBufferSize = max(fftSize, waveSize);

			// initialize frequency bands
			if (nBands < 0)
			{
				RmLogF(rm, LOG_ERROR, L"Invalid Bands %ld: must be an integer >= 0.", nBands);
				nBands = 0;
			}
			m->m_nBands = nBands;

			// initialize smoothing
			m->m_smoothing = smoothing;

			m->m_freqMin = freqMin;
			m->m_freqMax = freqMax;

			// setup ring buffer
			if (m->m_ringBufferSize)
			{
				m->m_ringBuffer = (float*)calloc(m->m_ringBufferSize * sizeof(float), 1);
				m->m_ringBufOut = (float*)calloc(max(m->m_ringBufferSize, m->m_fftBufferSize) * sizeof(float), 1);
			}

			// setup FFT buffers
			if (m->m_fftSize)
			{
				m->m_fftKWdw = (float*)calloc(m->m_fftSize * sizeof(float), 1);

				m->m_fftCfg = kiss_fftr_alloc(m->m_fftBufferSize, 0, NULL, NULL);
				m->m_fftTmpOut = (kiss_fft_cpx*)calloc(m->m_fftBufferSize * sizeof(kiss_fft_cpx), 1);

				m->m_fftOut = (float*)calloc(m->m_fftBufferSize * sizeof(float), 1);

				m->m_fftScalar = (float)(1.0 / sqrt(m->m_fftSize));

				// zero-padding - https://jackschaedler.github.io/circles-sines-signals/zeropadding.html
				for (int iBin = 0; iBin < m->m_fftBufferSize; ++iBin) m->m_ringBufOut[iBin] = 0.0;

				// calculate window function coefficients (http://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window)
				for (unsigned int iBin = 1; iBin < m->m_fftSize; ++iBin)
					m->m_fftKWdw[iBin] = (float)(0.5 * (1.0 - cos(TWOPI * iBin / (m->m_fftSize + 1))));		// periodic version for FFT/spectral analysis
				m->m_fftKWdw[0] = 0.0;
			}

			// setup WAVE buffers
			if (m->m_waveSize)
			{
				m->m_dw = (float)m->m_waveSize / m->m_nBands;
				m->m_waveScalar = (float)(1.0 / m->m_dw);
				m->m_waveBandOut = (float*)calloc(m->m_nBands * sizeof(float), 1);
				m->m_waveBandTmpOut = (float*)calloc(m->m_nBands * sizeof(float), 1);
				m->m_waveOut = (float*)calloc(m->m_waveSize * sizeof(float), 1);
			}

			// calculate band frequencies and allocate band output buffers
			if (m->m_nBands)
			{
				m->m_bandFreq = (float*)malloc(m->m_nBands * sizeof(float));
				const double step = pow(2.0, (log(m->m_freqMax / m->m_freqMin) / m->m_nBands) / log(2.0));
				m->m_bandFreq[0] = (float)(m->m_freqMin * step);

				m->m_df = (float)m->m_wfx->nSamplesPerSec / m->m_fftBufferSize;
				m->m_bandScalar = 2.0f / (float)m->m_wfx->nSamplesPerSec;
				m->m_smoothingScalar = 1.0f / ((float)m->m_smoothing * 2.0f + 1.0f);

				for (int iBand = 1; iBand < m->m_nBands; ++iBand)
				{
					m->m_bandFreq[iBand] = (float)(m->m_bandFreq[iBand - 1] * step);
				}

				m->m_bandTmpOut = (float*)calloc(m->m_nBands * sizeof(float), 1);
				m->m_bandOut = (float*)calloc(m->m_nBands * sizeof(float), 1);
			}
		}

		// (re)parse envelope values
		m->m_envRMS[0] = max(0, RmReadInt(rm, L"RMSAttack", m->m_envRMS[0]));
		m->m_envRMS[1] = max(0, RmReadInt(rm, L"RMSDecay", m->m_envRMS[1]));
		m->m_envPeak[0] = max(0, RmReadInt(rm, L"PeakAttack", m->m_envPeak[0]));
		m->m_envPeak[1] = max(0, RmReadInt(rm, L"PeakDecay", m->m_envPeak[1]));
		m->m_envFFT[0] = max(0, RmReadInt(rm, L"FFTAttack", m->m_envFFT[0]));
		m->m_envFFT[1] = max(0, RmReadInt(rm, L"FFTDecay", m->m_envFFT[1]));

		// (re)parse gain constants
		m->m_gainRMS = max(0.0, RmReadDouble(rm, L"RMSGain", m->m_gainRMS));
		m->m_gainPeak = max(0.0, RmReadDouble(rm, L"PeakGain", m->m_gainPeak));

		m->m_sensitivity = 10 * log10(m->m_fftSize);	// default dynamic range/noise floor
		m->m_sensitivity = 10 / max(1.0, RmReadDouble(rm, L"Sensitivity", m->m_sensitivity));

		// regenerate filter constants
		if (m->m_wfx)
		{
			const double freq = m->m_wfx->nSamplesPerSec;
			m->m_kRMS[0] = (float)exp(log10(0.01) / (freq * (double)m->m_envRMS[0] * 0.001));
			m->m_kRMS[1] = (float)exp(log10(0.01) / (freq * (double)m->m_envRMS[1] * 0.001));
			m->m_kPeak[0] = (float)exp(log10(0.01) / (freq * (double)m->m_envPeak[0] * 0.001));
			m->m_kPeak[1] = (float)exp(log10(0.01) / (freq * (double)m->m_envPeak[1] * 0.001));

			if (m->m_fftSize)
			{
				m->m_kFFT[0] = (float)exp(log10(0.01) / (freq * 0.001 * (double)m->m_envFFT[0] * 0.001));
				m->m_kFFT[1] = (float)exp(log10(0.01) / (freq * 0.001 * (double)m->m_envFFT[1] * 0.001));
			}
		}
	}

	// parse FFT index request
	m->m_fftIdx = max(0, RmReadInt(rm, L"FFTIdx", m->m_fftIdx));
	m->m_fftIdx = m->m_parent ?
		min(m->m_parent->m_fftBufferSize / 2, m->m_fftIdx) :
		min(m->m_fftBufferSize / 2, m->m_fftIdx);

	// parse WAVE index request
	m->m_waveIdx = max(0, RmReadInt(rm, L"WaveIdx", m->m_waveIdx));
	m->m_waveIdx = m->m_parent ?
		min(m->m_parent->m_waveSize, m->m_waveIdx) :
		min(m->m_waveSize, m->m_waveIdx);

	// parse band index request
	m->m_bandIdx = max(0, RmReadInt(rm, L"BandIdx", m->m_bandIdx));
	m->m_bandIdx = m->m_parent ?
		min(m->m_parent->m_nBands, m->m_bandIdx) :
		min(m->m_nBands, m->m_bandIdx);
}


/**
* Update the measure.
*
* @param[in]	data			Measure instance pointer.
* @return		Latest value - typically an audio level between 0.0 and 1.0.
*/
PLUGIN_EXPORT double Update(void* data)
{
	Measure* m = (Measure*)data;
	Measure* parent = m->m_parent ? m->m_parent : m;

	switch (m->m_type)
	{
	case Measure::TYPE_BAND:
		if (parent->m_clCapture && parent->m_nBands && m->m_bandIdx < parent->m_nBands)
		{
			return parent->m_bandOut[m->m_bandIdx];
		}
		break;
	case Measure::TYPE_WAVEBAND:
		if (parent->m_clCapture && parent->m_nBands && parent->m_waveSize && m->m_bandIdx < parent->m_nBands)
		{
			return parent->m_waveBandOut[m->m_bandIdx];
		}
		break;
	case Measure::TYPE_FFT:
		if (parent->m_clCapture && parent->m_fftBufferSize && m->m_fftIdx < parent->m_fftBufferSize)
		{
			return max(0, parent->m_sensitivity * log10(CLAMP01(parent->m_fftOut[m->m_fftIdx])) + 1.0);
		}
		break;
	case Measure::TYPE_FFTFREQ:
		if (parent->m_clCapture && parent->m_fftBufferSize && m->m_fftIdx <= (parent->m_fftBufferSize * 0.5))
		{
			return (float)m->m_fftIdx * m->m_df;
		}
		break;

	case Measure::TYPE_BANDFREQ:
		if (parent->m_clCapture && parent->m_nBands && m->m_bandIdx < parent->m_nBands)
		{
			return parent->m_bandFreq[m->m_bandIdx];
		}
		break;
	case Measure::TYPE_WAVE:
		if (parent->m_clCapture && parent->m_waveSize && m->m_waveIdx < parent->m_waveSize)
		{
			return parent->m_waveOut[m->m_waveIdx];
		}
		break;
	case Measure::TYPE_RMS:
		if (parent->m_clCapture && parent->m_rms)
		{
			return CLAMP01(sqrt(parent->m_rms[m->m_channel]) * parent->m_gainRMS);
		}
		break;
	case Measure::TYPE_PEAK:
		if (parent->m_clCapture && parent->m_peak)
		{
			return CLAMP01(parent->m_peak[m->m_channel] * parent->m_gainPeak);
		}
		break;
	case Measure::TYPE_DEV_STATUS:
		if (parent->m_dev)
		{
			DWORD state;
			if (parent->m_dev->GetState(&state) == S_OK && state == DEVICE_STATE_ACTIVE)
			{
				return 1.0;
			}
		}
		break;
	case Measure::TYPE_BUFFERSTATUS:
		if (parent->m_nFramesNext > 0)
		{
			return parent->m_nFramesNext;
		}
		break;
	}

	return 0.0;
}


/**
* Indicates that the application working directory will not be reset by the plugin.
*/
PLUGIN_EXPORT void OverrideDirectory()
{
}


/**
* Get a string value from the measure.
*
* @param[in]	data			Measure instance pointer.
* @return		String value - must be copied out by the caller.
*/
PLUGIN_EXPORT LPCWSTR GetString(void* data)
{
	Measure* m = (Measure*)data;
	Measure* parent = m->m_parent ? m->m_parent : m;

	static WCHAR buffer[4096];
	const WCHAR* s_fmtName[Measure::NUM_FORMATS] =
	{
		L"<invalid>",	// FMT_INVALID
		L"PCM 16b",		// FMT_PCM_S16
		L"PCM 32b",		// FMT_PCM_F32
	};

	buffer[0] = '\0';

	switch (m->m_type)
	{
	default:
		// return NULL for any numeric values, so Rainmeter can auto-convert them.
		return NULL;

	case Measure::TYPE_FORMAT:
		if (parent->m_wfx)
		{
			_snwprintf_s(buffer, _TRUNCATE, L"%dHz %s %dch", parent->m_wfx->nSamplesPerSec,
				s_fmtName[parent->m_format], parent->m_wfx->nChannels);
		}
		break;

	case Measure::TYPE_DEV_NAME:
		return parent->m_devName;

	case Measure::TYPE_DEV_ID:
		if (parent->m_dev)
		{
			LPWSTR pwszID = NULL;
			if (parent->m_dev->GetId(&pwszID) == S_OK)
			{
				_snwprintf_s(buffer, _TRUNCATE, L"%s", pwszID);
				CoTaskMemFree(pwszID);
			}
		}
		break;

	case Measure::TYPE_DEV_LIST:
		if (parent->m_enum)
		{
			IMMDeviceCollection* collection = NULL;
			if (parent->m_enum->EnumAudioEndpoints(parent->m_port == Measure::PORT_OUTPUT ? eRender : eCapture,
				DEVICE_STATE_ACTIVE | DEVICE_STATE_UNPLUGGED, &collection) == S_OK)
			{
				WCHAR* d = &buffer[0];
				UINT nDevices;
				collection->GetCount(&nDevices);

				for (ULONG iDevice = 0; iDevice < nDevices; ++iDevice)
				{
					IMMDevice* device = NULL;
					IPropertyStore* props = NULL;
					if (collection->Item(iDevice, &device) == S_OK && device->OpenPropertyStore(STGM_READ, &props) == S_OK)
					{
						LPWSTR id = NULL;
						PROPVARIANT	varName;
						PropVariantInit(&varName);

						if (device->GetId(&id) == S_OK && props->GetValue(PKEY_Device_FriendlyName, &varName) == S_OK)
						{
							d += _snwprintf_s(d, (sizeof(buffer) + (UINT32)buffer - (UINT32)d) / sizeof(WCHAR), _TRUNCATE,
								L"%s%s: %s", iDevice > 0 ? L"\n" : L"", id, varName.pwszVal);
						}

						if (id) CoTaskMemFree(id);

						PropVariantClear(&varName);
					}

					SAFE_RELEASE(props);
					SAFE_RELEASE(device);
				}
			}

			SAFE_RELEASE(collection);
		}
		break;
	}

	return buffer;
}

HRESULT Measure::UpdateParent()
{
	BYTE* buffer;
	UINT32 nFrames, m_nFramesNext;
	DWORD  flags;

	HRESULT hr = m_clCapture->GetNextPacketSize(&m_nFramesNext);
	if (hr == S_OK)
	{
		if (m_nFramesNext <= 0) return S_FALSE;
		
		while (m_clCapture->GetBuffer(&buffer, &nFrames, &flags, NULL, NULL) == S_OK)
		{
			// if not F32, convert to F32
			if (m_format == Measure::FMT_PCM_F32)
			{
				memcpy(&m_bufChunk[0], &buffer[0], nFrames * m_wfx->nBlockAlign);
			}
			else if (m_format == Measure::FMT_PCM_S16)
			{
				INT16* buf = (INT16*)buffer;
				for (int iPcm = 0; iPcm < nFrames * m_wfx->nChannels; ++iPcm)
				{
					m_bufChunk[iPcm] = (float)buf[iPcm] * pcmScalar;
				}
			}

			// release buffer immediately to resume capture
			m_clCapture->ReleaseBuffer(nFrames);

			// test for discontinuity or silence
			if (flags & AUDCLNT_BUFFERFLAGS_SILENT) 
			{
				// is the ring buffer filled with silence? then stop updating
				if (m_nSilentFrames > m_ringBufferSize) 
				{
					return S_FALSE;
				}
				else 
				{
					m_nSilentFrames += nFrames;
				}
			}
			else if (flags & AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY)
			{
				// not sure what to do with those frames... ignore them? use them? treat as silent frames?
				// for now, ignore.
				continue;
			}
			else 
			{
				// audio data is not silent, reset silent frames counter
				m_nSilentFrames = 0;
			}

			if (m_type == Measure::TYPE_RMS || m_type == Measure::TYPE_PEAK)
			{
				// measure RMS and peak levels
				// loops unrolled for float, 16b and mono, stereo
				for (int iFrame = 0; iFrame < nFrames * m_wfx->nChannels;)
				{
					for (int iChan = 0; iChan < m_wfx->nChannels; ++iChan)
					{
						float x = (float)m_bufChunk[iFrame++];
						float sqrX = x * x;
						float absX = abs(x);
						m_rms[iChan] = sqrX + m_kRMS[(sqrX < m_rms[iChan])] * (m_rms[iChan] - sqrX);
						m_peak[iChan] = absX + m_kPeak[(absX < m_peak[iChan])] * (m_peak[iChan] - absX);
					}
				}

				// rms and peak values for sum channel
				m_rms[Measure::CHANNEL_SUM] = m_wfx->nChannels >= 2
					? (m_rms[Measure::CHANNEL_FL] + m_rms[Measure::CHANNEL_FR]) * 0.5f
					: m_rms[Measure::CHANNEL_FL];

				//RmLogF(m_rm, LOG_NOTICE, L"RMS CALCULATED, Channel %d: %f", 0, m_rms[0]);
			}

			// store data in ring buffers, and demux streams
			if (m_ringBufferSize)
			{
				for (int iFrame = 0; iFrame < nFrames * m_wfx->nChannels;)
				{
					for (int iChan = 0; iChan < m_wfx->nChannels; ++iChan)
					{
						if (m_channel == Measure::CHANNEL_SUM)
						{
							if (iChan == Measure::CHANNEL_FL)
							{
								// cannot increment before evaluation
								const float L = m_bufChunk[iFrame++];

								// stereo to mono: (L + R) / 2
								m_ringBuffer[m_ringBufW] = 0.5 * (L + m_bufChunk[iFrame++]);
							}
						}
						else if (iChan == m_channel)
						{
							m_ringBuffer[m_ringBufW] = m_bufChunk[iFrame++];
						}
						else { ++iFrame; }	// move along the raw data buffer
					}
					m_ringBufW = (m_ringBufW + 1) % m_ringBufferSize;	// move along the data-to-process buffer
				}
			}
		}

		// process FFTs
		if (m_ringBufferSize)
		{
			// copy from the circular ring buffer to temp space
			memcpy(&m_ringBufOut[0], &m_ringBuffer[m_ringBufW], (m_ringBufferSize - m_ringBufW) * sizeof(float));
			memcpy(&m_ringBufOut[m_ringBufferSize - m_ringBufW], &m_ringBuffer[0], m_ringBufW * sizeof(float));

			if (m_waveSize)
			{
				// copy waveform into wave output buffer
				memcpy(&m_waveOut[0], &m_ringBufOut[m_ringBufferSize - m_waveSize], m_waveSize * sizeof(float));
			}

			if (m_fftSize)
			{
				// apply the windowing function
				for (int iBin = m_ringBufferSize - m_fftSize; iBin < m_fftSize; ++iBin)
					m_ringBufOut[iBin] *= m_fftKWdw[iBin];

				kiss_fftr(m_fftCfg, &m_ringBufOut[m_ringBufferSize - m_fftSize], m_fftTmpOut);

				for (int iBin = 0; iBin < m_fftBufferSize; ++iBin)
				{
					// old and new values
					float x0 = m_fftOut[iBin];
					const float x1 = (m_fftTmpOut[iBin].r * m_fftTmpOut[iBin].r + m_fftTmpOut[iBin].i * m_fftTmpOut[iBin].i) * m_fftScalar;

					x0 = x1 + m_kFFT[(x1 < x0)] * (x0 - x1);		// attack/decay filter
					m_fftOut[iBin] = x0;
				}
			}
		}

		if (m_nBands)
		{
			// integrate waveform into lin-scale frequency bands
			if (m_waveSize)
			{
				memset(m_waveBandOut, 0, m_nBands * sizeof(float));
				int iBin = 0;
				int iBand = 0;
				float w0 = 0.0f;

				while (iBin <= m_waveSize && iBand < m_nBands)
				{
					const float wLin1 = iBin;
					const float bLin1 = m_dw * (iBand + 1);
					float& y = m_waveBandTmpOut[iBand];

					if (wLin1 < bLin1)
					{
						y += (wLin1 - w0) * (m_waveOut[iBin] + 0.5);
						w0 = wLin1;
						iBin += 1;
					}
					else
					{
						y += (bLin1 - w0) * (m_waveOut[iBin] + 0.5);
						y *= m_waveScalar;
						w0 = bLin1;
						iBand += 1;
					}
				}

				// smoothing
				for (iBand = 0; iBand < m_nBands; iBand++)
				{
					float x = 0;
					for (int s = -m_smoothing; s <= m_smoothing; s++)
					{
						x += m_waveBandTmpOut[(iBand + s <= 0) || (iBand + s >= m_nBands) ? iBand : iBand + s];
					}
					m_waveBandOut[iBand] = x * m_smoothingScalar;
				}
			}

			// integrate FFT results into log-scale frequency bands
			if (m_fftSize)
			{
				memset(m_bandTmpOut, 0, m_nBands * sizeof(float));
				int iBin = (int)((m_freqMin / m_df));
				int iBand = 0;
				float f0 = m_freqMin;

				while (iBin <= (m_fftBufferSize * 0.5) && iBand < m_nBands)
				{
					const float fLin1 = ((float)iBin) * m_df;
					const float fLog1 = m_bandFreq[iBand];
					float& y = m_bandTmpOut[iBand];

					if (fLin1 <= fLog1)
					{
						y += (fLin1 - f0) * m_fftOut[iBin] * m_bandScalar;
						f0 = fLin1;
						iBin += 1;
					}
					else
					{
						y += (fLog1 - f0) * m_fftOut[iBin] * m_bandScalar;
						f0 = fLog1;
						iBand += 1;
					}
				}

				// sensivity
				for (iBand = 0; iBand < m_nBands; iBand++)
				{
					m_bandTmpOut[iBand] = max(0, m_sensitivity * log10(CLAMP01(m_bandTmpOut[iBand])) + 1.0);
				}

				// smoothing
				for (iBand = 0; iBand < m_nBands; iBand++)
				{
					float x = 0;
					for (int s = -m_smoothing; s <= m_smoothing; s++)
					{
						x += m_bandTmpOut[(iBand + s <= 0) || (iBand + s >= m_nBands) ? iBand : iBand + s];
					}
					m_bandOut[iBand] = x * m_smoothingScalar;
				}
			}
		}
	}

	return hr;
}


/**
* Try to initialize the default device for the specified port.
*
* @return		Result value, S_OK on success.
*/
HRESULT	Measure::DeviceInit()
{
	HRESULT hr;

	// get the device handle
	assert(m_enum && m_dev);

	// store device name
	IPropertyStore* props = NULL;
	if (m_dev->OpenPropertyStore(STGM_READ, &props) == S_OK)
	{
		PROPVARIANT	varName;
		PropVariantInit(&varName);

		if (props->GetValue(PKEY_Device_FriendlyName, &varName) == S_OK)
		{
			_snwprintf_s(m_devName, _TRUNCATE, L"%s", varName.pwszVal);
		}

		PropVariantClear(&varName);
	}

	SAFE_RELEASE(props);

	// get an extra audio client for loopback events
	hr = m_dev->Activate(IID_IAudioClient, CLSCTX_ALL, NULL, (void**)&m_clBugAudio);
	if (hr != S_OK)
	{
		RmLog(LOG_WARNING, L"Failed to create audio client for loopback events.");
	}

	// get the main audio client
	//if (m_dev->Activate(IID_IAudioClient3, CLSCTX_ALL, NULL, (void**)&m_clAudio) != S_OK)
	//{
	if (m_dev->Activate(IID_IAudioClient, CLSCTX_ALL, NULL, (void**)&m_clAudio) != S_OK)
	{
		RmLog(LOG_WARNING, L"Failed to create audio client.");
		goto Exit;
	}
	//}

	// parse audio format - Note: not all formats are supported.
	hr = m_clAudio->GetMixFormat(&m_wfx);
	EXIT_ON_ERROR(hr);

	m_wfxR.nChannels = m_wfx->nChannels;
	m_wfxR.nSamplesPerSec = m_wfx->nSamplesPerSec;
	m_wfxR.cbSize = 0;

	CoTaskMemFree(m_wfx);

	m_wfxR.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
	m_wfxR.wBitsPerSample = 32;
	m_wfxR.nBlockAlign = m_wfxR.nChannels * m_wfxR.wBitsPerSample / 8;
	m_wfxR.nAvgBytesPerSec = m_wfxR.nSamplesPerSec * m_wfxR.nBlockAlign;

	if (m_clAudio->IsFormatSupported(AUDCLNT_SHAREMODE_SHARED, &m_wfxR, &m_wfx) != AUDCLNT_E_UNSUPPORTED_FORMAT)
	{
		m_format = FMT_PCM_F32;
	}
	else
	{
		m_wfxR.wFormatTag = WAVE_FORMAT_PCM;
		m_wfxR.wBitsPerSample = 16;
		m_wfxR.nBlockAlign = m_wfxR.nChannels * m_wfxR.wBitsPerSample / 8;
		m_wfxR.nAvgBytesPerSec = m_wfxR.nSamplesPerSec * m_wfxR.nBlockAlign;

		if (m_clAudio->IsFormatSupported(AUDCLNT_SHAREMODE_SHARED, &m_wfxR, &m_wfx) != AUDCLNT_E_UNSUPPORTED_FORMAT)
		{
			m_format = FMT_PCM_S16;
		}
		else
		{
			// try a standard format
			m_wfxR.nChannels = 2;
			m_wfxR.nSamplesPerSec = 48000;
			m_wfxR.nBlockAlign = m_wfxR.nChannels * m_wfxR.wBitsPerSample / 8;
			m_wfxR.nAvgBytesPerSec = m_wfxR.nSamplesPerSec * m_wfxR.nBlockAlign;

			if (m_clAudio->IsFormatSupported(AUDCLNT_SHAREMODE_SHARED, &m_wfxR, &m_wfx) != AUDCLNT_E_UNSUPPORTED_FORMAT)
			{
				m_format = FMT_PCM_S16;
			}
			else
			{
				RmLog(LOG_WARNING, L"Invalid sample format.  Only PCM 16b integer or PCM 32b float are supported.");
				goto Exit;
			}
		}
	}
	if (!m_wfx) { m_wfx = &m_wfxR; }

	hr = m_clBugAudio->Initialize(
		AUDCLNT_SHAREMODE_SHARED,
		AUDCLNT_STREAMFLAGS_EVENTCALLBACK,		// "Each time the client receives an event for the render stream, it must signal the capture client to run"
		0,
		0,
		m_wfx,
		NULL);
	if (hr != S_OK)
	{
		RmLog(LOG_WARNING, L"Failed to initialize audio client for loopback events.");
	}
	EXIT_ON_ERROR(hr);

	m_hReadyEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (m_hReadyEvent == NULL)
	{
		RmLog(LOG_WARNING, L"Failed to create buffer-event handle.");
		hr = E_FAIL;
		goto Exit;
	}

	m_hStopEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

	hr = m_clBugAudio->SetEventHandle(m_hReadyEvent);
	EXIT_ON_ERROR(hr);

#if (WINDOWS_BUG_WORKAROUND)
	// ---------------------------------------------------------------------------------------
	// Windows bug workaround: create a silent render client before initializing loopback mode
	// see: http://social.msdn.microsoft.com/Forums/windowsdesktop/en-US/c7ba0a04-46ce-43ff-ad15-ce8932c00171/loopback-recording-causes-digital-stuttering?forum=windowspro-audiodevelopment
	if (m_port == PORT_OUTPUT)
	{
		hr = m_clBugAudio->GetService(IID_IAudioRenderClient, (void**)&m_clBugRender);
		EXIT_ON_ERROR(hr);

		UINT32 nFrames;
		hr = m_clBugAudio->GetBufferSize(&nFrames);
		EXIT_ON_ERROR(hr);

		BYTE* buffer;
		hr = m_clBugRender->GetBuffer(nFrames, &buffer);
		EXIT_ON_ERROR(hr);

		hr = m_clBugRender->ReleaseBuffer(nFrames, AUDCLNT_BUFFERFLAGS_SILENT);
		EXIT_ON_ERROR(hr);
	}
	// ---------------------------------------------------------------------------------------
#endif

	hr = m_clBugAudio->Start();
	if (hr != S_OK)
	{
		RmLog(LOG_WARNING, L"Failed to start the stream for loopback events.");
	}
	EXIT_ON_ERROR(hr);

	// initialize the audio client

	/* void* ptr = NULL;
	if (m_clAudio->QueryInterface(IID_IAudioClient3, (void**)&ptr) == S_OK)
	{
	AudioClientProperties props = { 0 };
	props.cbSize = sizeof(AudioClientProperties);
	props.bIsOffload = FALSE;
	props.eCategory = AudioCategory_Other;
	//props.Options = AUDCLNT_STREAMOPTIONS_RAW | AUDCLNT_STREAMOPTIONS_MATCH_FORMAT;

	if (((IAudioClient3*)m_clAudio)->SetClientProperties(&props) != S_OK)
	{
	RmLog(LOG_WARNING, L"Failed to set audio client properties.");
	goto Exit;
	}

	UINT32 defFrames, funFrames, minFrames, maxFrames;
	hr = ((IAudioClient3*)m_clAudio)->GetSharedModeEnginePeriod(m_wfx, &defFrames, &funFrames, &minFrames, &maxFrames);
	EXIT_ON_ERROR(hr);

	// 0x88890021 AUDCLNT_E_INVALID_STREAM_FLAG - Loopback not supported?
	hr = ((IAudioClient3*)m_clAudio)->InitializeSharedAudioStream((m_port == PORT_OUTPUT ? AUDCLNT_STREAMFLAGS_LOOPBACK : 0)
	, minFrames, m_wfx, NULL);
	if (hr != S_OK)
	{
	RmLog(LOG_WARNING, L"Failed to initialize audio client (3).");
	goto Exit;
	}
	} else */

	if (m_clAudio->Initialize(
		AUDCLNT_SHAREMODE_SHARED,
		(m_port == PORT_OUTPUT ? AUDCLNT_STREAMFLAGS_LOOPBACK : 0),
		0,
		0,
		m_wfx,
		NULL) != S_OK)
	{
		RmLog(LOG_WARNING, L"Failed to initialize loopback audio client.");
		goto Exit;
	}

	// initialize the audio capture client
	hr = m_clAudio->GetService(IID_IAudioCaptureClient, (void**)&m_clCapture);
	if (hr != S_OK)
	{
		RmLog(LOG_WARNING, L"Failed to create audio capture client.");
	}
	EXIT_ON_ERROR(hr);

	// start the stream
	hr = m_clAudio->Start();
	if (hr != S_OK)
	{
		RmLog(LOG_WARNING, L"Failed to start the stream.");
	}
	EXIT_ON_ERROR(hr);

	// allocate buffer for latest data chunk copy
	UINT32 nMaxFrames;
	hr = m_clAudio->GetBufferSize(&nMaxFrames);
	if (hr != S_OK)
	{
		RmLog(LOG_WARNING, L"Failed to determine max buffer size.");
	}
	EXIT_ON_ERROR(hr);

	m_bufChunk = (float*)calloc(nMaxFrames * m_wfx->nBlockAlign * sizeof(float), 1);

	return S_OK;

Exit:
	DeviceRelease();
	RmLogF(m_rm, LOG_ERROR, L"AudioLevel: Failed with HRESULT  %d", (int)hr);
	return hr;
}


/**
* Release handles to audio resources.  (except the enumerator)
*/
void Measure::DeviceRelease()
{

	RmLog(LOG_DEBUG, L"Releasing dummy stream audio device.");
	if (m_clBugAudio)
	{
		m_clBugAudio->Stop();
	}
#if (WINDOWS_BUG_WORKAROUND)
	SAFE_RELEASE(m_clBugRender);
#endif
	SAFE_RELEASE(m_clBugAudio);

	RmLog(LOG_DEBUG, L"Releasing audio device.");

	if (m_clAudio)
	{
		m_clAudio->Stop();
	}

	SAFE_RELEASE(m_clCapture);
	SAFE_RELEASE(m_clAudio);
	SAFE_RELEASE(m_dev);

	if (m_hReadyEvent != NULL) { CloseHandle(m_hReadyEvent); }
	if (m_hStopEvent != NULL) { CloseHandle(m_hStopEvent); }
	//if (m_hTask != NULL) { AvRevertMmThreadCharacteristics(m_hTask); }

	if (m_fftCfg) kiss_fftr_free(m_fftCfg);
	m_fftCfg = NULL;

	if (m_bufChunk) free(m_bufChunk);
	m_bufChunk = NULL;

	if (m_ringBuffer) free(m_ringBuffer);
	m_ringBuffer = NULL;

	if (m_fftOut) free(m_fftOut);
	m_fftOut = NULL;

	if (m_bandOut) free(m_bandOut);
	m_bandOut = NULL;

	if (m_bandTmpOut) free(m_bandTmpOut);
	m_bandTmpOut = NULL;

	if (m_waveBandOut) free(m_waveBandOut);
	m_waveBandOut = NULL;

	if (m_waveOut) free(m_waveOut);
	m_waveOut = NULL;

	if (m_waveBandTmpOut) free(m_waveBandTmpOut);
	m_waveBandTmpOut = NULL;

	for (int iChan = 0; iChan < Measure::MAX_CHANNELS; ++iChan)
	{
		m_rms[iChan] = 0.0;
		m_peak[iChan] = 0.0;
	}

	if (m_bandFreq)
	{
		free(m_bandFreq);
		m_bandFreq = NULL;
	}

	if (m_fftTmpOut)
	{
		free(m_fftTmpOut);
		free(m_ringBufOut);
		free(m_fftKWdw);
		m_fftTmpOut = NULL;
		m_ringBufOut = NULL;
		m_fftKWdw = NULL;
		kiss_fft_cleanup();
	}

	m_devName[0] = '\0';
	m_format = FMT_INVALID;
}