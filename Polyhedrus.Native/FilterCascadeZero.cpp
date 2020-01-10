#include "Default.h"
#include "FilterCascadeZero.h"
#include "AudioLib/ValueTables.h"
#include "AudioLib/Utils.h"
#include "AudioLib/MathDefs.h"

namespace Polyhedrus
{
	FilterCascadeZero::FilterCascadeZero()
	{
		Drive = 0.0;
		Cutoff = 20000.0;
		Resonance = 0.0;
		ResonanceMod = 0.0;
		CutoffMod = 0.0;
		DriveMod = 0.0;

		k = 0;
		uScaler = 0;
		g = 0;
		g2 = 0;
		g3 = 0;
		g4 = 0;
		gainCompensation = 0;
	}

	FilterCascadeZero::~FilterCascadeZero()
	{}

	void FilterCascadeZero::Initialize(int samplerate, int bufferSize, int modulationUpdateRate)
	{
        if (bufferSize > buffer.size())
            buffer.resize(bufferSize, 0);
        else
            buffer.assign(bufferSize, 0);
        
        for(auto* lp : { &lp1, &lp2, &lp3, &lp4 }) {
            lp->y = 0.f;
            lp->z1State = 0.f;
        }
        
		this->bufferSize = bufferSize;
		this->modulationUpdateRate = modulationUpdateRate;
		this->samplerate = samplerate;
		fsinv = 1.0f / samplerate;

		Cutoff = 20000;
		Update();
	}

	void FilterCascadeZero::Process(float* input, int len)
	{
		Update();
        
        jassert(len <= buffer.size());

		for (int i = 0; i < len; i++)
		{
			float x = input[i] * gain;
			ProcessSample(x);
			float value = lp4.y * gainInv * gainCompensation;
			buffer[i] = value;
		}
	}

	__inline_always void FilterCascadeZero::ProcessSample(float x)
	{
		// this filter is pretty much directly from the book:
		// https://www.native-instruments.com/fileadmin/ni_media/downloads/pdf/VAFilterDesign_1.1.1.pdf
		// See equations 4.3 and figure 3.18
		// this does not use a root finding algorithm to apply the nonlinearities, rather they are the "quick and dirty approach"

		//float G = g4;
		float S = g3 * lp1.z1State + g2 * lp2.z1State + g * lp3.z1State + lp4.z1State;

		//float value = (x - k * S) / (1 + k * G);
		float value = (x - k * S) * uScaler;
			
		// 4 cascaded low pass stages
		value = lp1.Process(value);
		value = AudioLib::Utils::CubicNonlin(value);
		value = lp2.Process(value);
		value = AudioLib::Utils::CubicNonlin(value);
		value = lp3.Process(value);
		value = AudioLib::Utils::CubicNonlin(value);
		value = lp4.Process(value);
		value = AudioLib::Utils::CubicNonlin(value);
	}

	void FilterCascadeZero::Update()
	{
		float driveTotal = Drive + DriveMod;
		driveTotal = AudioLib::Utils::Limit(driveTotal, 0.0f, 1.0f);

		gain = (0.2f + 1.8f * driveTotal * driveTotal);
		gainInv = gain < 1.0 ? std::sqrt(1.0f / gain) : 1.0f;
		gainInv *= 2; // gain fudge;

        // We're providing the Cutoff frequency in Hz
        float fc = AudioLib::Utils::Limit(Cutoff, 0.0f, 20000.0f);

		totalResonance = Resonance + ResonanceMod;
		totalResonance = AudioLib::Utils::Limit(totalResonance, 0.0f, 1.0f);
		totalResonance = (float)AudioLib::ValueTables::Get(totalResonance, AudioLib::ValueTables::Response2Oct) * 0.999f;
		k = totalResonance * 4.2f;

		gainCompensation = 1.5f / (1 - totalResonance + 0.5f);
		g = fc * (float)(M_PI * fsinv);
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
