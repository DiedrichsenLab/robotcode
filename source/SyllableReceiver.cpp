#include "SyllableReceiver.h"
#include "Timer626.h"
#include <ws2tcpip.h>
#include <iostream>

#pragma comment(lib, "ws2_32.lib")

// Defined in sfs1.cpp; same trial clock used for finger pressTime.
extern Timer gTimer;

SyllableReceiver::SyllableReceiver() {}
SyllableReceiver::~SyllableReceiver() { stop(); }

bool SyllableReceiver::start(unsigned short port) {
    if (running) return true;

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "SyllableReceiver: WSAStartup failed\n";
        return false;
    }
    wsaStarted = true;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "SyllableReceiver: socket() failed (" << WSAGetLastError() << ")\n";
        WSACleanup();
        wsaStarted = false;
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "SyllableReceiver: bind() failed on port " << port
                  << " (" << WSAGetLastError() << ")\n";
        closesocket(sock);
        sock = INVALID_SOCKET;
        WSACleanup();
        wsaStarted = false;
        return false;
    }

    boundPort = port;
    head.store(0);
    tail.store(0);
    running = true;
    worker = std::thread(&SyllableReceiver::recvLoop, this);
    return true;
}

void SyllableReceiver::stop() {
    if (!running && sock == INVALID_SOCKET) return;

    running = false;

    // Closing the socket unblocks the recvfrom call in the worker thread.
    if (sock != INVALID_SOCKET) {
        closesocket(sock);
        sock = INVALID_SOCKET;
    }

    if (worker.joinable()) worker.join();

    if (wsaStarted) {
        WSACleanup();
        wsaStarted = false;
    }
}

void SyllableReceiver::recvLoop() {
    SyllableMsg msg;
    while (running) {
        sockaddr_in from{};
        int fromLen = sizeof(from);
        int n = recvfrom(sock, reinterpret_cast<char*>(&msg), sizeof(msg), 0,
                         reinterpret_cast<sockaddr*>(&from), &fromLen);
        if (n == SOCKET_ERROR) {
            if (!running) break; // socket closed during shutdown
            continue;            // transient error: keep listening
        }
        if (n != (int)sizeof(SyllableMsg) || msg.magic != SYLLABLE_MSG_MAGIC) {
            continue; // malformed packet
        }

        SyllableEvent ev;
        ev.type = msg.type;
        ev.syllable = msg.syllable;
        ev.arrivalTime = gTimer[1];
        ev.devTimeMs = msg.devTimeMs;

        // Push onto SPSC ring (producer side). Drop if full.
        unsigned int t = tail.load(std::memory_order_relaxed);
        unsigned int next = (t + 1) % CAP;
        if (next == head.load(std::memory_order_acquire)) {
            continue; // ring full: drop oldest-newest policy = drop new
        }
        buffer[t] = ev;
        tail.store(next, std::memory_order_release);
    }
}

bool SyllableReceiver::poll(SyllableEvent& out) {
    unsigned int h = head.load(std::memory_order_relaxed);
    if (h == tail.load(std::memory_order_acquire)) {
        return false; // empty
    }
    out = buffer[h];
    head.store((h + 1) % CAP, std::memory_order_release);
    return true;
}

void SyllableReceiver::flush() {
    head.store(tail.load(std::memory_order_acquire), std::memory_order_release);
}
