#ifndef LEIFTUR_ENVELOPE
#define LEIFTUR_ENVELOPE

#include "Parameters.h"

namespace Leiftur
{
	class Envelope
	{
	public:
		static const int SectionAttack = 0;
		static const int SectionHold = 1;
		static const int SectionDecay = 2;
		static const int SectionSustain = 3;
		static const int SectionRelease = 4;
		static const int SectionPostRelease = 5;
		static const float MaxTimeSeconds; // defined in body

		bool Gate;
		float Output;

	private:
		int samplerate;
		int section;
		float iterator;
		float attackInc;
		float holdInc;
		float decayInc;
		float sustain;
		float releaseInc;

	public:
		Envelope();
		~Envelope();
		void Initialize(int samplerate);
		void SetParameter(EnvParameters parameter, double value);
		float Process(int samples);
		void Reset();

	};
}

#endif