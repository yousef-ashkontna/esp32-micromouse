#pragma once
class BluetoothSerial {
public:
    bool begin(const char *) { return true; }
    bool hasClient() { return false; }
    void println(const char *) {}
};
