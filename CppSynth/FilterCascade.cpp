#include "FilterCascade.h"
#include "AudioLib/ValueTables.h"
#include "AudioLib/Utils.h"
#include <cmath>

Leiftur::FilterCascade::FilterCascade()
{
	Drive = 0.0;
	Cutoff = 1.0;
	Resonance = 0.0;
	ResonanceMod = 0.0;
	CutoffMod = 0.0;
	DriveMod = 0.0;
	buffer = 0;
	driveTotal = 0.0;
}

Leiftur::FilterCascade::~FilterCascade()
{
	delete buffer;
}

void Leiftur::FilterCascade::Initialize(int samplerate, int bufferSize, int modulationUpdateRate)
{
	buffer = new float[bufferSize];
	this->modulationUpdateRate = modulationUpdateRate;
	this->samplerate = samplerate;
	fsinv = 1.0 / (Oversample * samplerate);

	Cutoff = 1;
	VD = 1;
	updateCounter = 0;
	oversampledInput = 0;
	Update();
}

void Leiftur::FilterCascade::Process(float * input, int len)
{
	float gInv = sqrt(1.0 / gain);

	for (size_t i = 0; i < len; i++)
	{
		if (updateCounter <= 0)
		{
			Update();
			gInv = sqrt(1.0 / gain);
			updateCounter = modulationUpdateRate;
		}

		float value = ProcessSample(input[i]) * gInv;
		buffer[i] = value;
		updateCounter--;
	}
}

float * Leiftur::FilterCascade::GetOutput()
{
	return buffer;
}

float Leiftur::FilterCascade::ProcessSample(float input)
{
	input *= gain;
	float mx = 1.0 / Oversample;
	float sum = 0.0;

	for (int i = 0; i < Oversample; i++)
	{
		float in = mx * (i * input + (Oversample - i) * oversampledInput); // linear interpolation
		in = AudioLib::Utils::TanhPoly(in);

		float fb = totalResonance * 5 * (feedback - 0.5 * in);
		float val = in - fb;
		x = val;

		// 4 cascaded low pass stages
		a = (1 - p) * val + p * a;
		val = a;
		b = (1 - p) * val + p * b;
		val = b;
		c = (1 - p) * val + p * c;
		val = c;
		d = (1 - p) * val + p * d;
		val = d;

		feedback = AudioLib::Utils::TanhPoly(val);
	}

	oversampledInput = input;
	auto sample = (VX * x + VA * a + VB * b + VC * c + VD * d) * (1 - totalResonance * 0.5);
	return sample;
}

void Leiftur::FilterCascade::Update()
{
	driveTotal = Drive + DriveMod;
	driveTotal = AudioLib::Utils::Limit(driveTotal, 0.0, 1.0);

	gain = (0.1 + 1.0 * driveTotal * driveTotal);

	totalResonance = Resonance + ResonanceMod;
	totalResonance = AudioLib::Utils::Limit(totalResonance, 0.0, 0.999);
	
	auto value = Cutoff + CutoffMod;
	value = AudioLib::Utils::Limit(value, 0.0, 1.0);

	auto cutoff = 10 + AudioLib::ValueTables::Get(value, AudioLib::ValueTables::Response3Dec) * 21000;
	// Todo: get a proper lokup table to tune the filter
	p = (1 - 2 * cutoff * fsinv) * (1 - 2 * cutoff * fsinv);
}
