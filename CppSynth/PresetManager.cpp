#include "PresetManager.h"
#include "Parameters.h"
#include "Synth.h"
#include "Lfo.h"
#include "ModMatrix.h"
#include "ModSourceDest.h"

namespace Leiftur
{
	PresetManager::PresetManager()
	{
	}

	void PresetManager::Initialize(std::string baseDirectory)
	{
	}

	Preset PresetManager::GetDefaultPreset()
	{
		Preset preset;
		preset.BankName = "Default";
		preset.Filename = "";
		preset.Metadata["Author"] = "Default";
		preset.PresetName = "Init Preset";
		preset.ValuesLoaded = true;

		auto setOsc = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Cent)] = 0;
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Octave)] = 0;
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Pan)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Phase)] = 1.0;
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Semi)] = 0;
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Shape)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Slop)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Volume)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)OscParameters::Waveform)] = 0;
		};

		auto setMixer = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)MixerParameters::Am12)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)MixerParameters::Am23)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)MixerParameters::Color)] = 0;
			preset.Values[Synth::PackParameter(module, (int)MixerParameters::Fm12)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)MixerParameters::Fm13)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)MixerParameters::Fm23)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)MixerParameters::Noise)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)MixerParameters::Output)] = 1.0;
		};

		auto setCharacter = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)CharacterParameters::Bottom)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)CharacterParameters::Clip)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)CharacterParameters::Decimate)] = 0;
			preset.Values[Synth::PackParameter(module, (int)CharacterParameters::Reduce)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)CharacterParameters::Top)] = 0.0;
		};

		auto setFilterHp = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)FilterHpParameters::Cutoff)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)FilterHpParameters::Env)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)FilterHpParameters::Keytrack)] = 0;
			preset.Values[Synth::PackParameter(module, (int)FilterHpParameters::Resonance)] = 0.0;
		};

		auto setFilterMain = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)FilterMainParameters::Drive)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)FilterMainParameters::Cutoff)] = 1.0;
			preset.Values[Synth::PackParameter(module, (int)FilterMainParameters::Env)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)FilterMainParameters::Keytrack)] = 0;
			preset.Values[Synth::PackParameter(module, (int)FilterMainParameters::Resonance)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)FilterMainParameters::Type)] = 0;
		};

		auto setEnv = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)EnvParameters::Attack)] = 0.1;
			preset.Values[Synth::PackParameter(module, (int)EnvParameters::Decay)] = 0.5;
			preset.Values[Synth::PackParameter(module, (int)EnvParameters::Exponent)] = 0;
			preset.Values[Synth::PackParameter(module, (int)EnvParameters::Hold)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)EnvParameters::Release)] = 0.2;
			preset.Values[Synth::PackParameter(module, (int)EnvParameters::Retrigger)] = 0;
			preset.Values[Synth::PackParameter(module, (int)EnvParameters::Sustain)] = 0.8;
			preset.Values[Synth::PackParameter(module, (int)EnvParameters::Velocity)] = 0.0;
		};

		auto setLfo = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)LfoParameters::Attack)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)LfoParameters::Decay)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)LfoParameters::Freq)] = 0.5;
			preset.Values[Synth::PackParameter(module, (int)LfoParameters::Phase)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)LfoParameters::Release)] = 0;
			preset.Values[Synth::PackParameter(module, (int)LfoParameters::Shape)] = (int)LfoShape::Triangle;
			preset.Values[Synth::PackParameter(module, (int)LfoParameters::Sustain)] = 1.0;
			preset.Values[Synth::PackParameter(module, (int)LfoParameters::Sync)] = 0.0;
		};

		auto setArp = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)ArpParameters::Bpm)] = 120.0;
			preset.Values[Synth::PackParameter(module, (int)ArpParameters::Divide)] = 16.0;
			preset.Values[Synth::PackParameter(module, (int)ArpParameters::Gate)] = 0.99;
			preset.Values[Synth::PackParameter(module, (int)ArpParameters::NotePtn)] = 0;
			preset.Values[Synth::PackParameter(module, (int)ArpParameters::OctavePtn)] = 0;
			preset.Values[Synth::PackParameter(module, (int)ArpParameters::Range)] = 3;
			preset.Values[Synth::PackParameter(module, (int)ArpParameters::Sync)] = 0;
		};

		auto setVoices = [&](Module module)
		{
			preset.Values[Synth::PackParameter(module, (int)VoiceParameters::Bend)] = 2.0;
			preset.Values[Synth::PackParameter(module, (int)VoiceParameters::Detune)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)VoiceParameters::Glide)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)VoiceParameters::Master)] = 1.0;
			preset.Values[Synth::PackParameter(module, (int)VoiceParameters::Polyphony)] = 6;
			preset.Values[Synth::PackParameter(module, (int)VoiceParameters::Spread)] = 0.0;
			preset.Values[Synth::PackParameter(module, (int)VoiceParameters::Unison)] = 1;
			preset.Values[Synth::PackParameter(module, (int)VoiceParameters::VoiceMode)] = (int)VoiceMode::PolyRoundRobin;
		};

		auto setModMatrix = [&](Module module)
		{
			for (int i = 0; i < ModMatrix::MatrixCount; i++)
			{
				preset.Values[Synth::PackParameter(module, 10 * (1 + i) + 0)] = (int)ModSource::Off; // Source
				preset.Values[Synth::PackParameter(module, 10 * (1 + i) + 1)] = (int)ModDest::Off; // Dest
				preset.Values[Synth::PackParameter(module, 10 * (1 + i) + 2)] = (int)ModDest::Off; // Via
				preset.Values[Synth::PackParameter(module, 10 * (1 + i) + 3)] = 0.0; // Amount
				preset.Values[Synth::PackParameter(module, 10 * (1 + i) + 4)] = 0.0; // Via Amount
			}
		};

		setOsc(Module::Osc1);
		setOsc(Module::Osc2);
		setOsc(Module::Osc3);
		setMixer(Module::Mixer);
		setCharacter(Module::Character);
		setFilterHp(Module::FilterHp);
		setFilterMain(Module::FilterMain);
		setEnv(Module::EnvAmp);
		setEnv(Module::EnvFilter);
		setEnv(Module::EnvMod);
		setLfo(Module::Lfo1);
		setLfo(Module::Lfo2);
		setArp(Module::Arp);
		setVoices(Module::Voices);
		setModMatrix(Module::ModMatrix);

		preset.Values[Synth::PackParameter(Module::Osc1, (int)OscParameters::Volume)] = 1.0;

		return preset;
	}

	void PresetManager::LoadPresetValues(Preset* preset)
	{
	}

	void PresetManager::SavePreset(Preset* preset)
	{
	}
}
