#include "FilterCascadeZero.h"
#include "AudioLib/ValueTables.h"
#include "AudioLib/Utils.h"
#include "AudioLib/MathDefs.h"

namespace Leiftur
{
	float FilterCascadeZero::CVtoAlpha[CVtoAlphaSize];

	void FilterCascadeZero::ComputeCVtoAlpha(int samplerate)
	{
		double fsInv = 1.0 / samplerate;
		for (int i = 0; i < CVtoAlphaSize; i++)
		{
			double freq = 440.0 * std::pow(2, (i*0.01 - 69.0 + 12) / 12);
			CVtoAlpha[i] = (float)((1.0f - 2.0f * freq * fsInv) * (1.0f - 2.0f * freq * fsInv));
		}
	}

	FilterCascadeZero::FilterCascadeZero()
	{
		Drive = 0.0;
		Cutoff = 1.0;
		Resonance = 0.0;
		ResonanceMod = 0.0;
		CutoffMod = 0.0;
		DriveMod = 0.0;
		buffer = 0;
		driveTotal = 0.0;

		k = 0;
		uScaler = 0;
		g = 0;
		g2 = 0;
		g3 = 0;
		g4 = 0;
		gainCompensation = 0;
	}

	FilterCascadeZero::~FilterCascadeZero()
	{
		delete buffer;
	}

	void FilterCascadeZero::Initialize(int samplerate, int bufferSize, int modulationUpdateRate)
	{
		ComputeCVtoAlpha(samplerate);
		buffer = new float[bufferSize];
		this->bufferSize = bufferSize;
		this->modulationUpdateRate = modulationUpdateRate;
		this->samplerate = samplerate;
		fsinv = 1.0f / (Oversample * samplerate);
		cvToFreq.Initialize((float)samplerate);

		Cutoff = 1;
		updateCounter = 0;
		Update();
	}

	void FilterCascadeZero::Process(float* input, int len)
	{
		for (int i = 0; i < len; i++)
		{
			if (updateCounter <= 0)
			{
				Update();
				updateCounter = modulationUpdateRate;
			}

			float value = ProcessSample(input[i]) * gainInv * gainCompensation;
			buffer[i] = value;
			updateCounter--;
		}
	}

	float* FilterCascadeZero::GetOutput()
	{
		return buffer;
	}

	float FilterCascadeZero::ProcessSample(float x)
	{
		// this filter is pretty much directly from the book:
		// https://www.native-instruments.com/fileadmin/ni_media/downloads/pdf/VAFilterDesign_1.1.1.pdf
		// See equations 4.3 and figure 3.18
		// this does not use a root finding algorithm to apply the nonlinearities, rather they are the "quick and dirty approach"

		x *= gain;
		float output = 0.0;

		for (int i = 0; i < Oversample; i++)
		{
			//float G = g4;
			float S = g3 * lp1.z1State + g2 * lp2.z1State + g * lp3.z1State + lp4.z1State;

			//float value = (x - k * S) / (1 + k * G);
			float value = (x - k * S) * uScaler;
			
			// 4 cascaded low pass stages
			value = lp1.Process(value);
			value = AudioLib::Utils::TanhLookup(value);
			value = lp2.Process(value);
			value = AudioLib::Utils::TanhLookup(value);
			value = lp3.Process(value);
			value = AudioLib::Utils::TanhLookup(value);
			value = lp4.Process(value);
			value = AudioLib::Utils::TanhLookup(value);
			output = value;
		}

		return output;
	}

	void FilterCascadeZero::Update()
	{
		driveTotal = Drive + DriveMod;
		driveTotal = AudioLib::Utils::Limit(driveTotal, 0.0f, 1.0f);

		gain = (0.2f + 1.8f * driveTotal * driveTotal);
		gainInv = gain < 1.0 ? std::sqrt(1.0f / gain) : 1.0f;

		// Voltage is 1V/OCt, C0 = 16.3516Hz
		float voltage = 10.3f * Cutoff + CutoffMod;
		voltage = AudioLib::Utils::Limit(voltage, 0.0f, 10.3f);
		float fc = cvToFreq.GetFreqWarped(voltage);

		totalResonance = Resonance + ResonanceMod;
		totalResonance = AudioLib::Utils::Limit(totalResonance, 0.0f, 0.999f);
		k = totalResonance * 4.2f;

		gainCompensation = 1.5f / (1 - totalResonance + 0.5f);
		g = fc * M_PI * fsinv;
		g2 = g * g;
		g3 = g2 * g;
		g4 = g2 * g2;
		uScaler = 1.0f / (1 + k * g4);
		lp1.SetG(g);
		lp2.SetG(g);
		lp3.SetG(g);
		lp4.SetG(g);
	}

}
