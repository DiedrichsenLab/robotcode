#include "AudioRecorder.h"
#include <fstream>

#pragma comment(lib, "winmm.lib")

AudioRecorder::AudioRecorder() {}
AudioRecorder::~AudioRecorder() { stop(); }

bool AudioRecorder::start(const std::string& wavPath, int sr, int ch, int bps) {
    if (recording) return true;
    outPath = wavPath;
    pcm.clear();

    fmt.wFormatTag = WAVE_FORMAT_PCM;
    fmt.nChannels = (WORD)ch;
    fmt.nSamplesPerSec = sr;
    fmt.wBitsPerSample = (WORD)bps;
    fmt.nBlockAlign = (fmt.nChannels * fmt.wBitsPerSample) / 8;
    fmt.nAvgBytesPerSec = fmt.nSamplesPerSec * fmt.nBlockAlign;
    fmt.cbSize = 0;

    if (waveInOpen(&hWaveIn, WAVE_MAPPER, &fmt, (DWORD_PTR)&AudioRecorder::waveInProc, (DWORD_PTR)this, CALLBACK_FUNCTION) != MMSYSERR_NOERROR)
        return false;

    for (int i = 0; i < NBUF; ++i) {
        buffers[i].resize(BUF_BYTES);
        headers[i].lpData = (LPSTR)buffers[i].data();
        headers[i].dwBufferLength = BUF_BYTES;
        headers[i].dwFlags = 0;
        headers[i].dwLoops = 0;
        waveInPrepareHeader(hWaveIn, &headers[i], sizeof(WAVEHDR));
        waveInAddBuffer(hWaveIn, &headers[i], sizeof(WAVEHDR));
    }

    if (waveInStart(hWaveIn) != MMSYSERR_NOERROR) return false;
    recording = true;
    return true;
}

void AudioRecorder::stop() {
    if (!hWaveIn) return;
    recording = false;
    waveInStop(hWaveIn);
    waveInReset(hWaveIn);

    for (int i = 0; i < NBUF; ++i)
        waveInUnprepareHeader(hWaveIn, &headers[i], sizeof(WAVEHDR));

    waveInClose(hWaveIn);
    hWaveIn = nullptr;
    writeWavFile();
}

void CALLBACK AudioRecorder::waveInProc(HWAVEIN, UINT msg, DWORD_PTR inst, DWORD_PTR p1, DWORD_PTR) {
    if (msg != WIM_DATA) return;
    auto* self = reinterpret_cast<AudioRecorder*>(inst);
    self->onBuffer(reinterpret_cast<WAVEHDR*>(p1));
}

void AudioRecorder::onBuffer(WAVEHDR* hdr) {
    if (!recording || !hdr || hdr->dwBytesRecorded == 0) return;
    {
        std::lock_guard<std::mutex> lock(mtx);
        pcm.insert(pcm.end(), (unsigned char*)hdr->lpData, (unsigned char*)hdr->lpData + hdr->dwBytesRecorded);
    }
    if (hWaveIn) waveInAddBuffer(hWaveIn, hdr, sizeof(WAVEHDR));
}

void AudioRecorder::writeWavFile() {
    std::lock_guard<std::mutex> lock(mtx);
    if (outPath.empty() || pcm.empty()) return;

    std::ofstream out(outPath, std::ios::binary);
    if (!out) return;

    uint32_t dataSize = (uint32_t)pcm.size();
    uint32_t riffSize = 36 + dataSize;

    out.write("RIFF", 4); out.write((char*)&riffSize, 4); out.write("WAVE", 4);
    out.write("fmt ", 4);
    uint32_t fmtSize = 16; out.write((char*)&fmtSize, 4);
    out.write((char*)&fmt.wFormatTag, 2);
    out.write((char*)&fmt.nChannels, 2);
    out.write((char*)&fmt.nSamplesPerSec, 4);
    out.write((char*)&fmt.nAvgBytesPerSec, 4);
    out.write((char*)&fmt.nBlockAlign, 2);
    out.write((char*)&fmt.wBitsPerSample, 2);
    out.write("data", 4); out.write((char*)&dataSize, 4);
    out.write((char*)pcm.data(), dataSize);
}