#ifndef POLYHEDRUS_FILTER_CASCADE
#define POLYHEDRUS_FILTER_CASCADE

#include <cmath>
#include <unordered_map>
#include <array>
#include "FilterInternalMode.h"

namespace Polyhedrus
{
	class FilterCascade
	{
	private:
		static const int CVtoAlphaSize = 10500;
        static std::unordered_map<int, std::array<float, CVtoAlphaSize>> CVtoAlphas;
		static void ComputeCVtoAlpha(int samplerate);
	public:
		static inline float GetCvFreq(float cv)
		{
			// Voltage is 1V/OCt, C0 = 16.3516Hz
			// 10.3V = Max = 20614.33hz
			float freq = (float)(440.0 * std::pow(2, (cv * 12 - 69.0 + 12) / 12));
			return freq;
		}

	public:
		int Oversample = 2;

		float Drive;
		float Cutoff;
		float Resonance;
		float CutoffMod;
		float ResonanceMod;
		float DriveMod;

		float c0, c1, c2, c3, c4;

	private:
        std::vector<float> buffer;
		float gain;
		float totalResonance;
		float oversampledInput;

		float p;
		float x;
		float a;
		float b;
		float c;
		float d;
		float feedback;

		float fsinv;
		int samplerate;
		int modulationUpdateRate;
		float gInv;
		float mx;

	public:
		FilterCascade();
        ~FilterCascade();
        FilterCascade(FilterCascade&&) noexcept = default;
        FilterCascade& operator=(FilterCascade&&) noexcept = default;

		void Initialize(int samplerate, int bufferSize, int modulationUpdateRate);
		void Process(float* input, int len);
		inline float* GetOutput() { return buffer.data(); }
		void SetMode(InternalFilterMode mode);

	private:
		float ProcessSample(float input);
		void Update();
	};
}

#endif

