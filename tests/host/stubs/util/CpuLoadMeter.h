#pragma once
// CPU timing is hardware-dependent and outside these audio correctness tests.
namespace daisy {
class CpuLoadMeter {
  public:
    void Init(float, int) {}
    void OnBlockStart() {}
    void OnBlockEnd() {}
    float GetAvgCpuLoad() const { return 0.0f; }
    float GetMaxCpuLoad() const { return 0.0f; }
};
}
