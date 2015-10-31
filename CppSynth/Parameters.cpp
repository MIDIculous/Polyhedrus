#include <string>
#include <sstream>
#include <vector>
#include "Parameters.h"
#include "Utils.h"

namespace Leiftur
{
	Module Parameters::GetModule(std::string moduleString)
	{
		if (moduleString == "Control") return Module::Control;
		if (moduleString == "Osc1") return Module::Osc1;
		if (moduleString == "Osc2") return Module::Osc2;
		if (moduleString == "Osc3") return Module::Osc3;
		if (moduleString == "Mixer") return Module::Mixer;
		if (moduleString == "Character") return Module::Character;
		if (moduleString == "FilterHp") return Module::FilterHp;
		if (moduleString == "FilterMain") return Module::FilterMain;
		if (moduleString == "Drive") return Module::Drive;
		if (moduleString == "EnvAmp") return Module::EnvAmp;
		if (moduleString == "EnvFilter") return Module::EnvFilter;
		if (moduleString == "Mod1") return Module::Mod1;
		if (moduleString == "Mod2") return Module::Mod2;
		if (moduleString == "Mod3") return Module::Mod3;
		if (moduleString == "Arp") return Module::Arp;
		if (moduleString == "Voices") return Module::Voices;
		if (moduleString == "Chorus") return Module::Chorus;
		if (moduleString == "Delay") return Module::Delay;
		if (moduleString == "Macros") return Module::Macros;
		if (moduleString == "ModMatrix") return Module::ModMatrix;
	}

	int Parameters::GetParameter(std::string parameterString, Module module)
	{
		if (module == Module::Osc1 || module == Module::Osc2 || module == Module::Osc3)
		{
			if (parameterString == "Octave") return (int)OscParameters::Octave;
			if (parameterString == "Semi") return (int)OscParameters::Semi;
			if (parameterString == "Cent") return (int)OscParameters::Cent;
			if (parameterString == "Pan") return (int)OscParameters::Pan;
			if (parameterString == "Volume") return (int)OscParameters::Volume;
			if (parameterString == "Slop") return (int)OscParameters::Slop;
			if (parameterString == "Phase") return (int)OscParameters::Phase;
			if (parameterString == "Shape") return (int)OscParameters::Shape;
			if (parameterString == "Waveform") return (int)OscParameters::Waveform;
			if (parameterString == "Routing") return (int)OscParameters::Routing;
			
		}
		else if (module == Module::Mixer)
		{
			if (parameterString == "Am12") return (int)MixerParameters::Am12;
			if (parameterString == "Am23") return (int)MixerParameters::Am23;
			if (parameterString == "Fm12") return (int)MixerParameters::Fm12;
			if (parameterString == "Fm13") return (int)MixerParameters::Fm13;
			if (parameterString == "Fm23") return (int)MixerParameters::Fm23;
			if (parameterString == "Noise") return (int)MixerParameters::Noise;
			if (parameterString == "Color") return (int)MixerParameters::Color;
			if (parameterString == "Output") return (int)MixerParameters::Output;
		}
		else if (module == Module::Character)
		{
			if (parameterString == "Bottom") return (int)CharacterParameters::Bottom;
			if (parameterString == "Clip") return (int)CharacterParameters::Clip;
			if (parameterString == "Decimate") return (int)CharacterParameters::Decimate;
			if (parameterString == "Reduce") return (int)CharacterParameters::Reduce;
			if (parameterString == "Top") return (int)CharacterParameters::Top;
		}
		else if (module == Module::FilterHp)
		{
			if (parameterString == "Cutoff") return (int)FilterHpParameters::Cutoff;
			if (parameterString == "Resonance") return (int)FilterHpParameters::Resonance;
			if (parameterString == "Keytrack") return (int)FilterHpParameters::Keytrack;
			if (parameterString == "Env") return (int)FilterHpParameters::Env;
		}
		else if (module == Module::FilterMain)
		{
			if (parameterString == "Drive") return (int)FilterMainParameters::Drive;
			if (parameterString == "Cutoff") return (int)FilterMainParameters::Cutoff;
			if (parameterString == "Resonance") return (int)FilterMainParameters::Resonance;
			if (parameterString == "Keytrack") return (int)FilterMainParameters::Keytrack;
			if (parameterString == "Env") return (int)FilterMainParameters::Env;
			if (parameterString == "Type") return (int)FilterMainParameters::Type;
		}
		else if (module == Module::Drive)
		{
			if (parameterString == "Bias") return (int)DriveParameters::Bias;
			if (parameterString == "Gain") return (int)DriveParameters::Gain;
			if (parameterString == "Mellow") return (int)DriveParameters::Mellow;
			if (parameterString == "Post") return (int)DriveParameters::Post;
			if (parameterString == "Type") return (int)DriveParameters::Type;
		}
		else if (module == Module::EnvAmp || module == Module::EnvFilter)
		{
			if (parameterString == "Attack") return (int)EnvParameters::Attack;
			if (parameterString == "Hold") return (int)EnvParameters::Hold;
			if (parameterString == "Decay") return (int)EnvParameters::Decay;
			if (parameterString == "Sustain") return (int)EnvParameters::Sustain;
			if (parameterString == "Release") return (int)EnvParameters::Release;
			if (parameterString == "Velocity") return (int)EnvParameters::Velocity;
			if (parameterString == "AttackCurve") return (int)EnvParameters::AttackCurve;
			if (parameterString == "DecayCurve") return (int)EnvParameters::DecayCurve;
			if (parameterString == "ReleaseCurve") return (int)EnvParameters::ReleaseCurve;
			if (parameterString == "Retrigger") return (int)EnvParameters::Retrigger;
		}
		else if (module == Module::Mod1 || module == Module::Mod2 || module == Module::Mod3)
		{
			if (parameterString == "Attack") return (int)ModParameters::Attack;
			if (parameterString == "AttackCurve") return (int)ModParameters::AttackCurve;
			if (parameterString == "Decay") return (int)ModParameters::Decay;
			if (parameterString == "DecayCurve") return (int)ModParameters::DecayCurve;
			if (parameterString == "Delay") return (int)ModParameters::Delay;
			if (parameterString == "Freq") return (int)ModParameters::Freq;
			if (parameterString == "Hold") return (int)ModParameters::Hold;
			if (parameterString == "Phase") return (int)ModParameters::Phase;
			if (parameterString == "Release") return (int)ModParameters::Release;
			if (parameterString == "ReleaseCurve") return (int)ModParameters::ReleaseCurve;
			if (parameterString == "Retrigger") return (int)ModParameters::Retrigger;
			if (parameterString == "Shape") return (int)ModParameters::Shape;
			if (parameterString == "Slew") return (int)ModParameters::Slew;
			if (parameterString == "Steps") return (int)ModParameters::Steps;
			if (parameterString == "Sustain") return (int)ModParameters::Sustain;
			if (parameterString == "Sync") return (int)ModParameters::Sync;
		}
		else if (module == Module::Arp)
		{
			if (parameterString == "Range") return (int)ArpParameters::Range;
			if (parameterString == "NotePtn") return (int)ArpParameters::NotePtn;
			if (parameterString == "OctavePtn") return (int)ArpParameters::OctavePtn;
			if (parameterString == "Gate") return (int)ArpParameters::Gate;
			if (parameterString == "Divide") return (int)ArpParameters::Divide;
			if (parameterString == "Bpm") return (int)ArpParameters::Bpm;
			if (parameterString == "Sync") return (int)ArpParameters::Sync;
		}
		else if (module == Module::Voices)
		{
			if (parameterString == "Detune") return (int)VoiceParameters::Detune;
			if (parameterString == "Spread") return (int)VoiceParameters::Spread;
			if (parameterString == "Glide") return (int)VoiceParameters::Glide;
			if (parameterString == "Bend") return (int)VoiceParameters::Bend;
			if (parameterString == "Master") return (int)VoiceParameters::Master;
			if (parameterString == "Polyphony") return (int)VoiceParameters::Polyphony;
			if (parameterString == "Unison") return (int)VoiceParameters::Unison;
			if (parameterString == "VoiceMode") return (int)VoiceParameters::VoiceMode;
		}
		else if (module == Module::Chorus)
		{
			if (parameterString == "Depth1") return (int)ChorusParameters::Depth1;
			if (parameterString == "Depth2") return (int)ChorusParameters::Depth2;
			if (parameterString == "Enable1") return (int)ChorusParameters::Enable1;
			if (parameterString == "Enable2") return (int)ChorusParameters::Enable2;
			if (parameterString == "Quality") return (int)ChorusParameters::Quality;
			if (parameterString == "Rate1") return (int)ChorusParameters::Rate1;
			if (parameterString == "Rate2") return (int)ChorusParameters::Rate2;
			if (parameterString == "Wet") return (int)ChorusParameters::Wet;
			if (parameterString == "Width") return (int)ChorusParameters::Width;
		}
		else if (module == Module::Delay)
		{
			if (parameterString == "DelayL") return (int)DelayParameters::DelayL;
			if (parameterString == "DelayR") return (int)DelayParameters::DelayR;
			if (parameterString == "FeedbackL") return (int)DelayParameters::FeedbackL;
			if (parameterString == "FeedbackR") return (int)DelayParameters::FeedbackR;
			if (parameterString == "Lowpass") return (int)DelayParameters::Lowpass;
			if (parameterString == "Highpass") return (int)DelayParameters::Highpass;
			if (parameterString == "Saturate") return (int)DelayParameters::Saturate;
			if (parameterString == "Crossfeed") return (int)DelayParameters::Crossfeed;
			if (parameterString == "Diffuse") return (int)DelayParameters::Diffuse;
			if (parameterString == "Wet") return (int)DelayParameters::Wet;
			if (parameterString == "Sync") return (int)DelayParameters::Sync;
		}
		else if (module == Module::Macros)
		{
			if (parameterString == "Macro1") return (int)MacroParameters::Macro1;
			if (parameterString == "Macro2") return (int)MacroParameters::Macro2;
			if (parameterString == "Macro3") return (int)MacroParameters::Macro3;
			if (parameterString == "Macro4") return (int)MacroParameters::Macro4;
			if (parameterString == "Macro5") return (int)MacroParameters::Macro5;
			if (parameterString == "Macro6") return (int)MacroParameters::Macro6;
			if (parameterString == "Macro7") return (int)MacroParameters::Macro7;
			if (parameterString == "Macro8") return (int)MacroParameters::Macro8;
		}
		else if (module == Module::ModMatrix)
		{
			if (parameterString == "Source1") return (int)ModMatrixParameters::Source1;
			if (parameterString == "Dest1") return (int)ModMatrixParameters::Dest1;
			if (parameterString == "Via1") return (int)ModMatrixParameters::Via1;
			if (parameterString == "Amount1") return (int)ModMatrixParameters::Amount1;
			if (parameterString == "ViaAmount1") return (int)ModMatrixParameters::ViaAmount1;
			if (parameterString == "Source2") return (int)ModMatrixParameters::Source2;
			if (parameterString == "Dest2") return (int)ModMatrixParameters::Dest2;
			if (parameterString == "Via2") return (int)ModMatrixParameters::Via2;
			if (parameterString == "Amount2") return (int)ModMatrixParameters::Amount2;
			if (parameterString == "ViaAmount2") return (int)ModMatrixParameters::ViaAmount2;
			if (parameterString == "Source3") return (int)ModMatrixParameters::Source3;
			if (parameterString == "Dest3") return (int)ModMatrixParameters::Dest3;
			if (parameterString == "Via3") return (int)ModMatrixParameters::Via3;
			if (parameterString == "Amount3") return (int)ModMatrixParameters::Amount3;
			if (parameterString == "ViaAmount3") return (int)ModMatrixParameters::ViaAmount3;
			if (parameterString == "Source4") return (int)ModMatrixParameters::Source4;
			if (parameterString == "Dest4") return (int)ModMatrixParameters::Dest4;
			if (parameterString == "Via4") return (int)ModMatrixParameters::Via4;
			if (parameterString == "Amount4") return (int)ModMatrixParameters::Amount4;
			if (parameterString == "ViaAmount4") return (int)ModMatrixParameters::ViaAmount4;
			if (parameterString == "Source5") return (int)ModMatrixParameters::Source5;
			if (parameterString == "Dest5") return (int)ModMatrixParameters::Dest5;
			if (parameterString == "Via5") return (int)ModMatrixParameters::Via5;
			if (parameterString == "Amount5") return (int)ModMatrixParameters::Amount5;
			if (parameterString == "ViaAmount5") return (int)ModMatrixParameters::ViaAmount5;
			if (parameterString == "Source6") return (int)ModMatrixParameters::Source6;
			if (parameterString == "Dest6") return (int)ModMatrixParameters::Dest6;
			if (parameterString == "Via6") return (int)ModMatrixParameters::Via6;
			if (parameterString == "Amount6") return (int)ModMatrixParameters::Amount6;
			if (parameterString == "ViaAmount6") return (int)ModMatrixParameters::ViaAmount6;
			if (parameterString == "Source7") return (int)ModMatrixParameters::Source7;
			if (parameterString == "Dest7") return (int)ModMatrixParameters::Dest7;
			if (parameterString == "Via7") return (int)ModMatrixParameters::Via7;
			if (parameterString == "Amount7") return (int)ModMatrixParameters::Amount7;
			if (parameterString == "ViaAmount7") return (int)ModMatrixParameters::ViaAmount7;
			if (parameterString == "Source8") return (int)ModMatrixParameters::Source8;
			if (parameterString == "Dest8") return (int)ModMatrixParameters::Dest8;
			if (parameterString == "Via8") return (int)ModMatrixParameters::Via8;
			if (parameterString == "Amount8") return (int)ModMatrixParameters::Amount8;
			if (parameterString == "ViaAmount8") return (int)ModMatrixParameters::ViaAmount8;
			if (parameterString == "Source9") return (int)ModMatrixParameters::Source9;
			if (parameterString == "Dest9") return (int)ModMatrixParameters::Dest9;
			if (parameterString == "Via9") return (int)ModMatrixParameters::Via9;
			if (parameterString == "Amount9") return (int)ModMatrixParameters::Amount9;
			if (parameterString == "ViaAmount9") return (int)ModMatrixParameters::ViaAmount9;
			if (parameterString == "Source10") return (int)ModMatrixParameters::Source10;
			if (parameterString == "Dest10") return (int)ModMatrixParameters::Dest10;
			if (parameterString == "Via10") return (int)ModMatrixParameters::Via10;
			if (parameterString == "Amount10") return (int)ModMatrixParameters::Amount10;
			if (parameterString == "ViaAmount10") return (int)ModMatrixParameters::ViaAmount10;
			if (parameterString == "Source11") return (int)ModMatrixParameters::Source11;
			if (parameterString == "Dest11") return (int)ModMatrixParameters::Dest11;
			if (parameterString == "Via11") return (int)ModMatrixParameters::Via11;
			if (parameterString == "Amount11") return (int)ModMatrixParameters::Amount11;
			if (parameterString == "ViaAmount11") return (int)ModMatrixParameters::ViaAmount11;
			if (parameterString == "Source12") return (int)ModMatrixParameters::Source12;
			if (parameterString == "Dest12") return (int)ModMatrixParameters::Dest12;
			if (parameterString == "Via12") return (int)ModMatrixParameters::Via12;
			if (parameterString == "Amount12") return (int)ModMatrixParameters::Amount12;
			if (parameterString == "ViaAmount12") return (int)ModMatrixParameters::ViaAmount12;
			if (parameterString == "Source13") return (int)ModMatrixParameters::Source13;
			if (parameterString == "Dest13") return (int)ModMatrixParameters::Dest13;
			if (parameterString == "Via13") return (int)ModMatrixParameters::Via13;
			if (parameterString == "Amount13") return (int)ModMatrixParameters::Amount13;
			if (parameterString == "ViaAmount13") return (int)ModMatrixParameters::ViaAmount13;
			if (parameterString == "Source14") return (int)ModMatrixParameters::Source14;
			if (parameterString == "Dest14") return (int)ModMatrixParameters::Dest14;
			if (parameterString == "Via14") return (int)ModMatrixParameters::Via14;
			if (parameterString == "Amount14") return (int)ModMatrixParameters::Amount14;
			if (parameterString == "ViaAmount14") return (int)ModMatrixParameters::ViaAmount14;
			if (parameterString == "Source15") return (int)ModMatrixParameters::Source15;
			if (parameterString == "Dest15") return (int)ModMatrixParameters::Dest15;
			if (parameterString == "Via15") return (int)ModMatrixParameters::Via15;
			if (parameterString == "Amount15") return (int)ModMatrixParameters::Amount15;
			if (parameterString == "ViaAmount15") return (int)ModMatrixParameters::ViaAmount15;
			if (parameterString == "Source16") return (int)ModMatrixParameters::Source16;
			if (parameterString == "Dest16") return (int)ModMatrixParameters::Dest16;
			if (parameterString == "Via16") return (int)ModMatrixParameters::Via16;
			if (parameterString == "Amount16") return (int)ModMatrixParameters::Amount16;
			if (parameterString == "ViaAmount16") return (int)ModMatrixParameters::ViaAmount16;
		}

		return 0;
	}

	void Parameters::ParseAddress(std::string address, Module* module, int* parameter)
	{
		address = std::string(&address[1]);

		auto parts = SplitString(address, '/');
		if (parts.size() != 2)
			return;

		auto moduleString = parts[0];
		auto parameterString = parts[1];

		*module = GetModule(moduleString);
		*parameter = GetParameter(parameterString, *module);
	}
}
