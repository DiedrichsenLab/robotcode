#ifndef SYLLABLE_RECEIVER_H
#define SYLLABLE_RECEIVER_H

#include <winsock2.h>
#include <windows.h>
#include <thread>
#include <atomic>
#include <cstdint>

// Wire format (little-endian, one event per UDP datagram, 8 bytes):
//   uint8_t  magic      0xAB sanity byte
//   uint8_t  type       1 = onset (initiated), 0 = offset (terminated)
//   uint8_t  syllable   spoken digit 1-5 (matches cue characters)
//   uint8_t  reserved   0
//   uint32_t devTimeMs  device's own clock (logged only)
#pragma pack(push, 1)
struct SyllableMsg {
    uint8_t  magic;
    uint8_t  type;
    uint8_t  syllable;
    uint8_t  reserved;
    uint32_t devTimeMs;
};
#pragma pack(pop)

#define SYLLABLE_MSG_MAGIC 0xAB

// Event handed to the control loop. arrivalTime is stamped on our clock
// (gTimer[1], same as finger pressTime) at the moment the packet is received.
struct SyllableEvent {
    int          type;        // 1 = onset, 0 = offset
    int          syllable;    // digit 1-5
    double       arrivalTime; // gTimer[1] at receive
    unsigned int devTimeMs;   // device timestamp (logged only)
};

// Asynchronous UDP receiver for per-syllable speech events.
// A background thread blocks on recvfrom and pushes parsed events into a
// single-producer/single-consumer lock-free ring buffer. The control loop
// (running inside the S626 interrupt) drains it via poll() without locking.
class SyllableReceiver {
public:
    SyllableReceiver();
    ~SyllableReceiver();

    bool start(unsigned short port = 5005);
    void stop();
    bool isRunning() const { return running; }

    // Non-blocking dequeue. Returns false if no event is available.
    bool poll(SyllableEvent& out);
    // Drop all queued events (call at trial start).
    void flush();

private:
    void recvLoop();

    static constexpr int CAP = 256; // ring capacity (power of two)
    SyllableEvent buffer[CAP];
    std::atomic<unsigned int> head{ 0 }; // consumer index
    std::atomic<unsigned int> tail{ 0 }; // producer index

    SOCKET sock = INVALID_SOCKET;
    std::thread worker;
    std::atomic<bool> running{ false };
    bool wsaStarted = false;
    unsigned short boundPort = 0;
};

#endif
