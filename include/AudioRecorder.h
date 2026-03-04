#include <windows.h>
#include <mmsystem.h>
#include <string>
#include <vector>
#include <mutex>

class AudioRecorder {
public:
    AudioRecorder();
    ~AudioRecorder();

    bool start(const std::string& wavPath, int sampleRate = 500, int channels = 1, int bitsPerSample = 16);
    void stop();
    bool isRecording() const { return recording; }

private:
    static void CALLBACK waveInProc(HWAVEIN hwi, UINT msg, DWORD_PTR inst, DWORD_PTR p1, DWORD_PTR p2);
    void onBuffer(WAVEHDR* hdr);
    void writeWavFile();

    HWAVEIN hWaveIn = nullptr;
    WAVEFORMATEX fmt{};
    std::string outPath;
    std::vector<unsigned char> pcm;
    std::mutex mtx;
    bool recording = false;

    static constexpr int NBUF = 4;
    static constexpr int BUF_BYTES = 8192;
    WAVEHDR headers[NBUF]{};
    std::vector<unsigned char> buffers[NBUF];
};