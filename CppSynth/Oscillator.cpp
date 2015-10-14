#include "Oscillator.h"
#include "AudioLib/Utils.h"
#include <iostream>

namespace Leiftur
{
	Oscillator::Oscillator()
	{
		Note = 60;
		updateCounter = 0;
		iterator = 0;
		SetWavetable(0);
	}

	void Oscillator::Initialize(int samplerate)
	{
		this->samplerate = samplerate;
	}

	void Oscillator::SetWavetable(int table)
	{
		wavetable = Wavetable::Wavetables[0];
		shiftValue = (32 - (int)(log2(wavetable->SampleSize) + 0.01)); // how many bits of the iterator are used to address the table
	}

	void Oscillator::Update()
	{
		auto pitch = Note + PitchBend * 2;
		auto partialIndex = Wavetable::WavetableIndex[(int)pitch];
		waveMix = WaveIndex - (int)WaveIndex;
		
		bool useNextWave = (WaveIndex < wavetable->Count - 1);
		waveA = wavetable->GetTable(partialIndex, WaveIndex * wavetable->Count);
		waveB = wavetable->GetTable(partialIndex, WaveIndex * wavetable->Count + useNextWave);

		float freq = AudioLib::Utils::Note2Freq(pitch);
		float samplesPerCycle = samplerate / freq;
		increment = (1.0 / samplesPerCycle) * UINT32_MAX;
	}

	void Oscillator::GetSamples(float * buffer, int count)
	{
		for (int i = 0; i < count; i++)
		{
			if (updateCounter == 0)
			{
				Update();
				updateCounter = MOD_UPDATE_RATE;
			}

			buffer[i] = waveA[iterator >> shiftValue] * (1 - waveMix) + waveB[iterator >> shiftValue] * waveMix;
			iterator += increment;

			updateCounter--;
		}
	}
}
