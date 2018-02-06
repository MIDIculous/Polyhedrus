#ifndef AUDIOLIB_LCGRANDOM
#define AUDIOLIB_LCGRANDOM

#include <stdint.h>
#include "Sse.h"
#include "Utils.h"

namespace AudioLib
{
	class LcgRandom
	{
	private:
		uint64_t x;
		uint64_t a;
		uint64_t c;
		//uint64_t m;

		double doubleInv;
		float floatUintInv;
		float floatIntInv;

	public:
		inline LcgRandom(uint64_t seed = 0)
		{
			x = seed;
			//m = 1 << 32;
			a = 22695477;
			c = 1;

			doubleInv = 1.0 / (double)UINT32_MAX;
			floatUintInv = 1.0 / (float)UINT32_MAX;
			floatIntInv = 1.0 / (float)INT32_MAX;
		}

		inline void SetSeed(uint64_t seed)
		{
			x = seed;
		}

		inline uint32_t NextUInt()
		{
			uint64_t axc = a * x + c;
			//x = axc % m;
			x = axc & 0xFFFFFFFF;
			return (uint32_t)x;
		}

		inline int32_t NextInt()
		{
			int64_t axc = a * x + c;
			//x = axc % m;
			x = axc & 0x7FFFFFFF;
			return (int32_t)x;
		}

		inline double NextDouble()
		{
			auto n = NextUInt();
			return n * doubleInv;
		}

		inline float NextFloat()
		{
			auto n = NextInt();
			return n * floatIntInv;
		}

		inline void GetFloats(float* buffer, int len)
		{
			for (int i = 0; i < len; i++)
				buffer[i] = NextFloat();
		}

		inline void GetFloatsBipolar(float* buffer, int len)
		{
			for (int i = 0; i < len; i++)
				buffer[i] = NextFloat() * 2 - 1;
		}
	};
}

#endif