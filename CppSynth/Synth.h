#ifndef LEIFTUR_SYNTH
#define LEIFTUR_SYNTH

#include <thread>

#include "Default.h"
#include "Voice.h"
#include "Osc/UdpTranceiver.h"

namespace Leiftur
{
	const int MaxVoiceCount = 1;

	class Synth
	{
	public:
		Voice Voices[MaxVoiceCount];
		int Samplerate;

	private:
		volatile bool isClosing;
		UdpTranceiver* udpTranceiver;
		std::thread messageListenerThread;

	public:
		Synth();
		~Synth();

		void Initialize(int samplerate, int udpListenPort, int udpSendPort);

		void SetParameter(int parameter, double value);
		void ProcessMidi(uint8_t* message);
		void ProcessAudio(float** buffer, int bufferSize);

	private:

		void MessageListener();

		void NoteOn(char note, char velocity);
		void NoteOff(char note);
		void MidiCC(uint8_t byte1, uint8_t byte2);
		void MidiProgram(uint8_t program);
		void PitchWheel(int pitchbend);
	};
}

#endif
