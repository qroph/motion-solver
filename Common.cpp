/*
Copyright (c) 2014 Mika Rantanen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <random>
#include <functional>

#include "Common.h"

namespace Random
{

unsigned int _seed;
std::mt19937 _engine;
std::uniform_real_distribution<double> _uniformDist;
std::normal_distribution<double> _normalDist;
std::function<double()> _getUniform;
std::function<double()> _getNormal;

void setSeed(unsigned int seed)
{
    _seed = seed;
    _engine.seed(seed);

    _getUniform = std::bind(_uniformDist, _engine);
    _getNormal = std::bind(_normalDist, _engine);
}

void initialize()
{
    setSeed(0);
}

unsigned int getSeed()
{
    return _seed;
}

double getNormal()
{
    return _getNormal();
}

double getUniform()
{
    return _getUniform();
}

double getHalton(const unsigned int index, const unsigned int prime)
{
    const float oneOverPrime = 1.0f / prime;

    float p = oneOverPrime;
    float result = 0.0f;
    for (unsigned int k = index + 1; k > 0;  p *= oneOverPrime)
    {
        const unsigned int kDiv = k / prime;
        const unsigned int kMod = k - (kDiv * prime); // Faster than %

        result += kMod * p;

        k = kDiv;
    }

    return result;
}

}
