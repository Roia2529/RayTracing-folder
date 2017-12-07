// cyCodeBase by Cem Yuksel
// [www.cemyuksel.com]
//-------------------------------------------------------------------------------
///
/// \file		cyIrradianceMap.h 
/// \author		Cem Yuksel
/// \version	0.3
/// \date		November 16, 2015
///
/// \brief irradiance map class.
///
///
/// @copydoc cyIrradianceMap
///
/// A simple class to store irradiance values for rendering using Monte Carlo
/// sampling for indirect illumination.
///
//-------------------------------------------------------------------------------

#ifndef _CY_IRRADIANCE_MAP_H_INCLUDED_
#define _CY_IRRADIANCE_MAP_H_INCLUDED_

//-------------------------------------------------------------------------------

#include <mutex>

//-------------------------------------------------------------------------------

/// Irradiance map class
template <class T> class cyIrradianceMap
{
public:
	cyIrradianceMap() : data(NULL), width(0), height(0) {}
	virtual ~cyIrradianceMap() { if ( data ) delete [] data; }

	/// Initialize once by providing the image with and height.
	/// The maxSubdiv parameter determines how many computation points will be generated.
	/// The minSubdiv parameter determines the minimum computation resolution.
	/// When minSubdiv is negative, a computation point is generated with 2^(-minSubdiv) pixels apart.
	/// When minSubdiv is positive, 2^(minSubdiv) computation points are generated per pixel.
	/// The total number of computation points is 2^(maxSubdiv) per pixel.
	/// If maxSubdiv is negative, width and height parameters must be an even multiple of 2^(-maxSubdiv);
	/// otherwise, returns false.
	bool Initialize(unsigned int _width, unsigned int _height, int _minSubdiv=-5, int _maxSubdiv=0)
	{
		if ( _maxSubdiv < 0 ) {
			unsigned int mask = (1<<(-_maxSubdiv))-1;
			if ( (_width & mask) > 0 || (_height & mask) > 0 ) return false;
		}
		if ( _maxSubdiv < _minSubdiv ) return false;
		if ( data ) delete [] data;
		width  = _width;
		height = _height;
		minSubdiv = _minSubdiv;
		maxSubdiv = _maxSubdiv;
		widthSampleCount = (width >> -maxSubdiv)+1; 
		heightSampleCount = (height >> -maxSubdiv)+1; 
		unsigned int n = widthSampleCount * heightSampleCount;
		data = new T[n];
		currentSubdiv = minSubdiv;
		currentX = 0;
		currentY = 0;
		currentSkipX = 1<<(-currentSubdiv+maxSubdiv);
		currentSkipY = currentSkipX;
		currentPhase = 0;
		return true;
	}

	/// Computes the next data point. If the data point can be estimated using previously
	/// computed points, it is estimated. Otherwise, ComputePoint() is called.
	/// Returns false if no more computation is needed.
	bool ComputeNextPoint(int threadID=0)
	{
		Lock();
		int subdiv = currentSubdiv;
		int phase = currentPhase;
		unsigned int x = currentX;
		unsigned int y = currentY;
		currentX += currentSkipX;
		if ( currentX >= widthSampleCount ) {
			currentY += currentSkipY;
			if ( currentY >= heightSampleCount ) {
				if ( currentPhase == 0 ) currentSubdiv++;
				currentPhase++;
				if ( currentPhase > 2 ) {
					currentSubdiv++;
					currentPhase = 1;
				}
				if ( currentPhase == 1 ) {
					currentSkipX = 1<<(-currentSubdiv+maxSubdiv+1);
					currentSkipY = currentSkipX;
					currentX = currentSkipX/2;
					currentY = currentSkipY/2;
				} else { // currentPhase == 2
					currentSkipY /= 2;
					currentX = currentSkipX/2;
					currentY = 0;
				}
			} else {
				switch ( currentPhase ) {
				case 0:
					currentX = 0;
					break;
				case 1:
					currentX = currentSkipX / 2;
					break;
				case 2:
					currentX = (1-((currentY/currentSkipY)&1)) * (currentSkipX/2);
					break;
				}
			}
		}
		Unlock();
		if ( subdiv > maxSubdiv ) return false;

		unsigned int i = y*widthSampleCount + x;
		unsigned int halfSkip = currentSkipX/2;
		unsigned int x2 = x + halfSkip;
		unsigned int y2 = y + halfSkip;
		
		bool estimate = false;
		unsigned int i0, i1, i2, i3;

		if ( phase == 1 ) {
			if ( x2 < widthSampleCount && y2 < heightSampleCount ) {
				estimate = true;
				unsigned int x1 = x - halfSkip;
				unsigned int y1 = y - halfSkip;
				i0 = y1*widthSampleCount + x1;
				i1 = y1*widthSampleCount + x2;
				i2 = y2*widthSampleCount + x1;
				i3 = y2*widthSampleCount + x2;
			}
		} else if ( phase == 2 ) {
			if ( x > 0 && y > 0 && x2 < widthSampleCount && y2 < heightSampleCount ) {
				estimate = true;
				unsigned int x1 = x - halfSkip;
				unsigned int y1 = y - halfSkip;
				i0 = y1*widthSampleCount + x;
				i1 = y *widthSampleCount + x1;
				i2 = y *widthSampleCount + x2;
				i3 = y2*widthSampleCount + x;
			}
		}

		if ( estimate ) {
			T avrg;
			if ( Estimate( avrg, data[i0], data[i1], data[i2], data[i3] ) ) {
				data[i] = avrg;
				return true;
			}
		}

		float sskip = (maxSubdiv<0) ? (1<<-maxSubdiv) : (1.0f/(1<<maxSubdiv));
		float sx = float(x) * sskip;
		float sy = float(y) * sskip;

		ComputePoint( data[i], sx, sy, threadID );
		return true;
	}

	/// Evaluates the value at a given image position by interpolating
	/// the values of the computation points.
	/// Use this method after the computation is done.
	/// The given val should include information that is necessary
	/// to determine is the interpolation in the Filter call is good enough.
	T Sample(float x, float y) const
	{
		float iskip = (maxSubdiv<0) ? (1.0f/(1<<-maxSubdiv)) : (1<<maxSubdiv);
		float xx = x*iskip;
		float yy = y*iskip;
		unsigned int ix = (unsigned int)xx;
		unsigned int iy = (unsigned int)yy;
		float fx = xx - (float)ix;
		float fy = yy - (float)iy;
		unsigned int ix2 = ix+1;
		unsigned int iy2 = iy+1;
		if ( ix >= widthSampleCount ) ix=ix2=widthSampleCount-1;
		else if ( ix2 >= widthSampleCount ) ix2=widthSampleCount-1;
		if ( iy >= heightSampleCount ) iy=iy2=heightSampleCount-1;
		else if ( iy2 >= heightSampleCount ) iy2=heightSampleCount-1;
		unsigned int i0 = iy *widthSampleCount + ix;
		unsigned int i1 = iy *widthSampleCount + ix2;
		unsigned int i2 = iy2*widthSampleCount + ix;
		unsigned int i3 = iy2*widthSampleCount + ix2;
		T val;
		BilinearFilter( val, data[i0], data[i1], data[i2], data[i3], fx, fy );
		return val;
	}

protected:

	/// Enters the critical section.
	void Lock() { iterator_mutex.lock(); }

	/// Leaves the critical section.
	void Unlock() { iterator_mutex.unlock(); }

	/// Computes the given screen position.
	virtual void ComputePoint( T &data, float x, float y, int threadID )=0;

	/// Computes the average of the four given data values.
	/// If the average is close to all four data values, returns true.
	/// Otherwise, returns false
	virtual bool Estimate( T &avrg, const T &data0, const T &data1, const T &data2, const T &data3 ) const
	{
		Average( avrg, data0, data1, data2, data3 );
		if (	IsSimilar( avrg, data0 ) &&
				IsSimilar( avrg, data1 ) &&
				IsSimilar( avrg, data2 ) &&
				IsSimilar( avrg, data3 ) ) return true;
		return false;
	}

	/// Returns the average of the given four data values.
	virtual void Average( T &avrg, const T &data0, const T &data1, const T &data2, const T &data3 ) const=0;

	/// Returns if the given two data values are similar.
	virtual bool IsSimilar( const T &data0, const T &data1 ) const=0;

	/// Returns the bilinear interpolation of the given four points.
	virtual void BilinearFilter( T &outVal, const T &dataX0Y0, const T &dataX1Y0, const T &dataX0Y1, const T &dataX1Y1, float fx, float fy ) const
	{
		T vy0, vy1;
		LinearFilter( vy0, dataX0Y0, dataX1Y0, fx );
		LinearFilter( vy1, dataX0Y1, dataX1Y1, fx );
		LinearFilter( outVal, vy0, vy1, fy );
	}

	/// Returns the linear interpolation of the given two points.
	virtual void LinearFilter( T &outVal, const T &data0, const T &data1, float f ) const=0;

private:
	T *data;
	unsigned int width, height;
	int minSubdiv, maxSubdiv;
	unsigned int widthSampleCount, heightSampleCount;
	int currentSubdiv, currentPhase;
	unsigned int currentX, currentY, currentSkipX, currentSkipY;
	std::mutex iterator_mutex;
};

//-------------------------------------------------------------------------------

/// Irradiance map for a single floating point value per computation.
/// Uses a threshold value to determine if the interpolation is good enough.
class cyIrradianceMapFloat : public cyIrradianceMap<float>
{
public:
	cyIrradianceMapFloat(float _threshold=1.0e30f) : threshold(_threshold) {}
	void SetThreshold(float t) { threshold=t; }
protected:
	virtual void Average( float &avrg, const float &data0, const float &data1, const float &data2, const float &data3 ) const
	{
		avrg = ( data0 + data1 + data2 + data3 ) * 0.25f;
	}
	virtual bool IsSimilar( const float &data0, const float &data1 ) const { return fabs(data0-data1)<threshold; }
	virtual void LinearFilter( float &outVal, const float &data0, const float &data1, float f ) const
	{
		outVal = data0*(1-f) + data1*f;
	}
private:
	float threshold;
};

//-------------------------------------------------------------------------------

/// Irradiance map for a single color value per computation.
/// Uses a threshold value to determine if the interpolation is good enough.
class cyIrradianceMapColor : public cyIrradianceMap<cyColor>
{
public:
	cyIrradianceMapColor(float _threshold=1.0e30f) { SetThreshold(_threshold); }
	cyIrradianceMapColor(cyColor _threshold) : threshold(_threshold) {}
	void SetThreshold(float t) { threshold.Set(t,t,t); }
	void SetThreshold(const cyColor &t) { threshold=t; }
protected:
	virtual void Average( cyColor &avrg, const cyColor &data0, const cyColor &data1, const cyColor &data2, const cyColor &data3 ) const
	{
		avrg = ( data0 + data1 + data2 + data3 ) * 0.25f;
	}
	virtual bool IsSimilar( const cyColor &data0, const cyColor &data1 ) const
	{
		cyColor dif = data0-data1;
		return fabs(dif.r)<threshold.r && fabs(dif.g)<threshold.g && fabs(dif.b)<threshold.b;
	}
	virtual void LinearFilter( cyColor &outVal, const cyColor &data0, const cyColor &data1, float f ) const
	{
		outVal = data0*(1-f) + data1*f;
	}
private:
	cyColor threshold;
};

//-------------------------------------------------------------------------------

/// Irradiance map for with a color and a z-depth value per computation.
/// Uses a color and a z-depth threshold value to determine if the interpolation is good enough.
class cyIrradianceMapColorZ : public cyIrradianceMap<cyColorA>
{
public:
	cyIrradianceMapColorZ(float _thresholdColor=1.0e30f, float _thresholdZ=1.0e30f) : thresholdZ(_thresholdZ) { SetColorThreshold(_thresholdColor); }
	cyIrradianceMapColorZ(cyColor _thresholdColor, float _thresholdZ=1.0e30f) : thresholdColor(_thresholdColor), thresholdZ(_thresholdZ) {}
	void SetColorThreshold(float t) { thresholdColor.Set(t,t,t); }
	void SetColorThreshold(const cyColor &t) { thresholdColor=t; }
	void SetZThreshold(float t) { thresholdZ=t; }
protected:
	virtual void Average( cyColorA &avrg, const cyColorA &data0, const cyColorA &data1, const cyColorA &data2, const cyColorA &data3 ) const
	{
		avrg = ( data0 + data1 + data2 + data3 ) * 0.25f;
	}
	virtual bool IsSimilar( const cyColorA &data0, const cyColorA &data1 ) const
	{
		cyColorA dif = data0-data1;
		return fabs(dif.r)<thresholdColor.r && fabs(dif.g)<thresholdColor.g && fabs(dif.b)<thresholdColor.b && fabs(dif.a)<thresholdZ;
	}
	virtual void LinearFilter( cyColorA &outVal, const cyColorA &data0, const cyColorA &data1, float f ) const
	{
		outVal = data0*(1-f) + data1*f;
	}
private:
	cyColor thresholdColor;
	float thresholdZ;
};

//-------------------------------------------------------------------------------

/// Structure that keep a color, a z-depth, and a normal value.
/// Used in cyIrradianceMapColorZNormal.
struct cyColorZNormal
{
	cyColor   c;
	float     z;
	cyPoint3f N;
	cyColorZNormal operator + ( const cyColorZNormal &czn ) const { cyColorZNormal r; r.c=c+czn.c; r.z=z+czn.z; r.N=N+czn.N; return r; }
	cyColorZNormal operator - ( const cyColorZNormal &czn ) const { cyColorZNormal r; r.c=c-czn.c; r.z=z-czn.z; r.N=N-czn.N; return r; }
	cyColorZNormal operator * ( float f ) const { cyColorZNormal r; r.c=c*f; r.z=z*f; r.N=N*f; return r; }
};

/// Irradiance map for with a color, a z-depth, and a normal value per computation.
/// Uses a color, a z-depth, and a normal threshold value to determine if the interpolation is good enough.
class cyIrradianceMapColorZNormal : public cyIrradianceMap<cyColorZNormal>
{
public:
	cyIrradianceMapColorZNormal(float _thresholdColor=1.0e30f, float _thresholdZ=1.0e30f, float _thresholdN=0.7f) : thresholdZ(_thresholdZ), thresholdN(_thresholdN) { SetColorThreshold(_thresholdColor); }
	cyIrradianceMapColorZNormal(cyColor _thresholdColor, float _thresholdZ=1.0e30f, float _thresholdN=0.7f) : thresholdColor(_thresholdColor), thresholdZ(_thresholdZ), thresholdN(_thresholdN) {}
	void SetColorThreshold(float t) { thresholdColor.Set(t,t,t); }
	void SetColorThreshold(const cyColor &t) { thresholdColor=t; }
	void SetZThreshold(float t) { thresholdZ=t; }
	void SetNThreshold(float t) { thresholdN=t; }
protected:
	virtual void Average( cyColorZNormal &avrg, const cyColorZNormal &data0, const cyColorZNormal &data1, const cyColorZNormal &data2, const cyColorZNormal &data3 ) const
	{
		avrg = ( data0 + data1 + data2 + data3 ) * 0.25f;
	}
	virtual bool IsSimilar( const cyColorZNormal &data0, const cyColorZNormal &data1 ) const
	{
		cyColorZNormal dif = data0-data1;
		return fabs(dif.c.r)<thresholdColor.r && fabs(dif.c.g)<thresholdColor.g && fabs(dif.c.b)<thresholdColor.b && fabs(dif.z)<thresholdZ && (data0.N%data1.N)>thresholdN;
	}
	virtual void LinearFilter( cyColorZNormal &outVal, const cyColorZNormal &data0, const cyColorZNormal &data1, float f ) const
	{
		outVal = data0*(1-f) + data1*f;
	}
private:
	cyColor thresholdColor;
	float thresholdZ;
	float thresholdN;
};

//-------------------------------------------------------------------------------

#endif
