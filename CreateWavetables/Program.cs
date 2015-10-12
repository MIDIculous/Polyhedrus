﻿using OxyPlot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using LowProfile.Visuals;
using LowProfile.Core.Extensions;
using LowProfile.Fourier.Double;
using System.IO;
using System.Threading;
using System.Globalization;
using System.Drawing;

namespace CreateWavetables
{
	class Program
	{
		[STAThread]
		static void Main(string[] args)
		{
            Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

			//ReadKomplexerWaves();
			ReadSq80Waves();
            return;
			var pulseWavetable = Pwm.CreateTable(2048, 64);

			var converted = ConvertTable(pulseWavetable, ArduinoPartials.NoteToPartials);
			converted.TableName = "Pwm";
			converted.Normalize();
			
			converted.WriteCppFile(TableInfo.DataMode.Float, @"C:\Src\_Tree\Audio\Leiftur\CppSynth\Wavetables\Pwm.cpp");
		}

        private static void ReadKomplexerWaves()
        {
            var file = @"E:\KOMPLEXER-Wavetables\kROM0.wt";
            var data = File.ReadAllBytes(file);
			
			int w = 0;
			foreach (var bytes in data.Chunk(64 * 4).Skip(15))
			{
				var partials = AudioLib.BufferConverter.ToFloat(bytes.ToArray()).Select(x => (double)(x)).ToArray();
				var wave1 = MakeWave(partials);
				var pm = new PlotModel();
				pm.AddLine(wave1);
				pm.Show();
			}
		}

		private static void ReadSq80Waves()
		{
			var file = @"E:\WaveData\esq1wavlo.bin";
			var data = File.ReadAllBytes(file);

			var waves = AudioLib.BufferConverter.ToSbyte(data).Select(x => (double)x / 128.0).ToArray();
			var pm = new PlotModel();
			pm.AddLine(waves);
			//pm.Show();
			var stereo = new[] { waves, waves };
			var dat = AudioLib.WaveFiles.WriteWaveFile(stereo, AudioLib.WaveFiles.WaveFormat.IEEEFloat32, 48000);
			File.WriteAllBytes(@"e:\sample.wav", dat);
		}

        private static double[] MakeWave(double[] partials)
        {
            var wave = new double[2048];
            for (int n = 0; n < partials.Length; n++)
            {
                var g = partials[n];

                for (int i = 0; i < wave.Length; i++)
                {
                    wave[i] += Math.Sin((n + 1) * (i / (double)wave.Length * 2 * Math.PI + Math.PI * 1.0)) * g;
                }
            }

            return wave;
        }

        private static TableInfo ConvertTable(float[][] wavetable, Dictionary<int, int> noteToPartials)
		{
			var partials = noteToPartials.Select(x => x.Value).Distinct().OrderByDescending(x => x).ToList();
			var transform = new Transform(wavetable.First().Length);

			var output = new TableInfo();
			output.MidiToTable = new int[128];
			output.RenderedTable = new float[partials.Count][][];
						
			Action<int, Complex[]> LimitPartials = (max, data) =>
			{
				for (int i = 0; i < data.Length; i++)
				{
					if (i > max && (data.Length - i) > max)
					{
						data[i].Real = 0;
						data[i].Imag = 0;
					}
				}
			};

			for (int partialIndex = 0; partialIndex < partials.Count; partialIndex++)
			{
				var partial = partials[partialIndex];
				noteToPartials.Where(x => x.Value == partial).Select(x => x.Key).Foreach(x => output.MidiToTable[x] = partialIndex);
				output.RenderedTable[partialIndex] = new float[wavetable.Length][];

				for (int tableIndex = 0; tableIndex < wavetable.Length; tableIndex++)
				{
					var table = wavetable[tableIndex];
					var input = table.Select(x => new Complex(x, 0)).ToArray();
					var fft = new Complex[input.Length];
					var ifft = new Complex[input.Length];
					transform.FFT(input, fft);
					LimitPartials(partial, fft);

					transform.IFFT(fft, ifft);
					var signal = ifft.Select(x => (float)x.Real).ToArray();

					output.RenderedTable[partialIndex][tableIndex] = signal;
				}
            }

			return output;
		}

		class TableInfo
		{
			public string TableName { get; set; }
			public int[] MidiToTable { get; set; }

			/// <summary>
			/// Indexed by Partials,TableIndex,Sample
			/// </summary>
			public float[][][] RenderedTable { get; set; }

			public void Normalize()
			{
				var max = RenderedTable.SelectMany(x => x.SelectMany(y => y)).Max(x => Math.Abs(x));
				var scale = (float)(1.0 / max);

				foreach (var partial in RenderedTable)
				{
					foreach(var table in partial)
					{
						for (int i = 0; i < table.Length; i++)
						{
							table[i] = scale * table[i];
						}
					}
				}
			}

			public enum DataMode
			{
				Bytes,
				Float
			}

			public void WriteCppFile(DataMode mode, string outputCppFile)
			{ 
				var partials = RenderedTable.Length;
				var tableCount = RenderedTable[0].Length;
				var tableSize = RenderedTable[0][0].Length;
				var dataType = mode == DataMode.Bytes ? "char" : "float";

                var cppFile = outputCppFile;
				if (!cppFile.EndsWith(".cpp")) cppFile += ".cpp";
				var hppFile = cppFile.Substring(0, cppFile.Length - 3) + "h";
				var hppFilename = Path.GetFileName(hppFile);
								
				// Write header
				var hppFileData = new StringBuilder();
				hppFileData.AppendLine($"const int {TableName}WavetablePartials = {partials};");
                hppFileData.AppendLine($"const int {TableName}WavetableCount = {tableCount};");
                hppFileData.AppendLine($"const int {TableName}WavetableSize = {tableSize};");
                hppFileData.AppendLine("");
				hppFileData.AppendLine($"extern {dataType} {TableName}WavetableData[{TableName}WavetablePartials][{TableName}WavetableCount][{TableName}WavetableSize];");
				hppFileData.AppendLine("");
				File.WriteAllText(hppFile, hppFileData.ToString());

				Func<float, string> format = number =>
				{
					if (mode == DataMode.Bytes)
					{
						var value = (int)(number * 127.999);
						return value.ToString();
					}
					else
					{
						return number.ToString();
					}
				};

				var data = new StringBuilder();
				foreach (var partial in RenderedTable)
				{
					data.AppendLine("\t{");
					foreach (var table in partial)
					{
						var tableData = string.Join(", ", table.Select(format));
						data.AppendLine("\t\t{ " + tableData + " },");
					}
					data.AppendLine("\t},");
				}

				// Write main
				var cppFileData = new StringBuilder();
				cppFileData.AppendLine($"#include \"{hppFilename}\"");
                cppFileData.AppendLine("");
				cppFileData.AppendLine($"{dataType} {TableName}WavetableData[{TableName}WavetablePartials][{TableName}WavetableCount][{TableName}WavetableSize] =");
				cppFileData.AppendLine("{");
				cppFileData.AppendLine(data.ToString());
				cppFileData.AppendLine("};");
				cppFileData.AppendLine("");
				File.WriteAllText(cppFile, cppFileData.ToString());
			}
		}
	}
}
