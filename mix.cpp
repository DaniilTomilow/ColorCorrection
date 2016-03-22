#include <iostream>
#include <opencv2/opencv.hpp>

#define PI 3.14159265


using namespace std;
using namespace cv;

/**
* RGB to HSL
*/
void RGBtoHSL(int rgb[], int hsl[])
{
	double var_R = rgb[0] / 255.0;
	double var_G = rgb[1] / 255.0;
	double var_B = rgb[2] / 255.0;

	double minV = min(var_R, var_G);
	double var_Min = min(minV, var_B);

	double maxV = max(var_R, var_G);
	double var_Max = max(maxV, var_B);

	double del_Max; // Delta RGB value

	del_Max = var_Max - var_Min;

	double H = 0.0, S, L;

	L = (var_Max + var_Min) / 2.0;

	if (del_Max == 0)
	{
		H = 0;
		S = 0;
	} // gray

	else
	{ // Chroma
		if (L < 0.5)
			S = del_Max / (var_Max + var_Min);
		else
			S = del_Max / (2 - var_Max - var_Min);

		double del_R = (((var_Max - var_R) / 6.0) + (del_Max / 2.0)) / del_Max;
		double del_G = (((var_Max - var_G) / 6.0) + (del_Max / 2.0)) / del_Max;
		double del_B = (((var_Max - var_B) / 6.0) + (del_Max / 2.0)) / del_Max;

		if (var_R == var_Max)
			H = del_B - del_G;
		else if (var_G == var_Max)
			H = (1 / 3.0) + del_R - del_B;
		else if (var_B == var_Max) H = (2 / 3.0) + del_G - del_R;
		if (H < 0) H += 1;
		if (H > 1) H -= 1;

	}

	hsl[0] = (int)(360 * H);
	hsl[1] = (int)(S * 100);
	hsl[2] = (int)(L * 100);

	cout << "RGBtoHSL: " << "H: " << hsl[0] << "; S: " << hsl[1] << "; L: " << hsl[2] << endl;
}

// https://de.wikipedia.org/wiki/HSV-Farbraum
void HSVToRGB(int hsl[], int rgb[]) {
	int h = hsl[0] / 60;
	double f = hsl[0] / 60.0 - h;

	double S = hsl[1] / 100.0;
	double V = hsl[2] / 100.0;

	double p = V * (1 - S);
	double q = V * (1 - S * f);
	double t = V * (1 - S * (1 - f));

	double mRGB[3];
	if (S == 0) {
		rgb[0] = V * 255;
		rgb[1] = V * 255;
		rgb[2] = V * 255;
		return;
	}


	switch (h) {
	case 0:
	case 6:
		mRGB[0] = V;
		mRGB[1] = t;
		mRGB[2] = p;
		break;
	case 1:
		mRGB[0] = q;
		mRGB[1] = V;
		mRGB[2] = p;
		break;
	case 2:
		mRGB[0] = p;
		mRGB[1] = V;
		mRGB[2] = t;
		break;
	case 3:
		mRGB[0] = p;
		mRGB[1] = q;
		mRGB[2] = V;
		break;
	case 4:
		mRGB[0] = t;
		mRGB[1] = p;
		mRGB[2] = V;
		break;
	case 5:
		mRGB[0] = V;
		mRGB[1] = p;
		mRGB[2] = q;
		break;
	}

	rgb[0] = mRGB[0] * 255;
	rgb[1] = mRGB[1] * 255;
	rgb[2] = mRGB[2] * 255;
}

void HSLToRGB(int hsl[], int rgb[]) {
	double v;
	double r, g, b;

	double l = hsl[2] / 100.0;
	double sl = hsl[1] / 100.0;
	double h = hsl[0] / 360.0;

	r = l;   // default to gray
	g = l;
	b = l;
	v = (l <= 0.5) ? (l * (1.0 + sl)) : (l + sl - l * sl);
	if (v > 0)
	{
		double m;
		double sv;
		int sextant;
		double fract, vsf, mid1, mid2;

		m = l + l - v;
		sv = (v - m) / v;
		h *= 6.0;
		sextant = (int)h;
		fract = h - sextant;
		vsf = v * sv * fract;
		mid1 = m + vsf;
		mid2 = v - vsf;
		switch (sextant)
		{
		case 0:
			r = v;
			g = mid1;
			b = m;
			break;
		case 1:
			r = mid2;
			g = v;
			b = m;
			break;
		case 2:
			r = m;
			g = v;
			b = mid1;
			break;
		case 3:
			r = m;
			g = mid2;
			b = v;
			break;
		case 4:
			r = mid1;
			g = m;
			b = v;
			break;
		case 5:
			r = v;
			g = m;
			b = mid2;
			break;
		}
	}

	rgb[0] = r * 255;
	rgb[1] = g * 255;
	rgb[2] = b * 255;
}

/// Aditive color mix
void mix(int rgb1[], int rgb2[], int result[]) {
	int hsl1[3];
	int hsl2[3];

	// Convert to HSL color space
	RGBtoHSL(rgb1, hsl1);
	RGBtoHSL(rgb2, hsl2);

	int r[2];
	r[0] = (hsl1[0] + hsl2[0]) / 2;
	r[1] = ((hsl1[0] + hsl2[0] + 360) / 2) % 360;

	int h, s, l;

	if (min(abs(hsl2[0] - r[0]), abs(hsl1[0] - r[0])) < min(abs(hsl1[0] - r[1]), abs(hsl2[0] - r[1])))
		h = r[0];
	else
		h = r[1];

	// Saturation
	if (hsl1[2] == 0) {
		h = hsl2[0];
	}
	else if (hsl2[2] == 0) {
		h = hsl1[0];
	}

	// Same hue or no light
	if (hsl1[0] == hsl2[0] || hsl1[2] == 0 || hsl2[2] == 0) {
		h = hsl1[2] == 0 ? hsl2[0] : hsl1[0];
		s = (hsl1[1] + hsl2[1]) / 2;
	}
	// Oppposite color
	else if (abs(hsl1[0] - hsl2[0]) == 180)
	{
		h = 180;
		s = abs(hsl1[1] - hsl2[1]);
	}
	else {
		// gamma angle
		int y = abs(hsl1[0] - hsl2[0]);
		y = y > 180 ? 360 - y : y;

		// Hypethenuse / Newton Color Circle
		// get height if triangle
		double c = sqrt(hsl1[1] * hsl1[1] + hsl2[1] * hsl2[1] - 2 * hsl1[1] * hsl2[1] * cos(y * PI / 180));
		//double g = sqrt(hsl1[1] * hsl1[1] + hsl2[1] * hsl2[1]);
		s = hsl1[1] * hsl2[1] * sin(y * PI / 180) / c;

		//s *= 2; // hmmm
	}

	// Avg light
	//l = (hsl1[2] + hsl2[2]) / 2;
	l = min(hsl1[2], hsl2[2]);

	int hsl[] = { h,s,l };

	cout << "MIX: " << h << " S:" << s << " L:" << l << endl;;
	HSLToRGB(hsl, result);
}

void MixImage(Mat* src, Mat* overlay, const Point& location) {

	for (int y = max(location.y, 0); y < src->rows; ++y)
	{
		int fY = y - location.y;

		if (fY >= overlay->rows)
			break;

		for (int x = max(location.x, 0); x < src->cols; ++x)
		{
			int fX = x - location.x;

			if (fX >= overlay->cols)
				break;

			for (int c = 0; c < 3; ++c)
			{
				// Get pixel
				unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
				unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];

				//src->data[y * src->step + src->channels() * x + c] = mix(srcPx, overlayPx);
				// Subtractive Color Mixing
				//src->data[y * src->step + src->channels() * x + c] = 255 - ((255 - srcPx) + (255 - overlayPx));

				//src->data[y * src->step + src->channels() * x + c] = min(srcPx + overlayPx, 255);

				// Simulate Lighting
				//src->data[y * src->step + src->channels() * x + c] = sqrt((overlayPx * overlayPx)) + ((srcPx * srcPx));
			}
		}

	}
}

// http://answers.opencv.org/question/73016/how-to-overlay-an-png-image-with-alpha-channel-to-another-png/
void overlayImage(Mat* src, Mat* overlay, const Point& location)
{
	for (int y = max(location.y, 0); y < src->rows; ++y)
	{
		int fY = y - location.y;

		if (fY >= overlay->rows)
			break;

		for (int x = max(location.x, 0); x < src->cols; ++x)
		{
			int fX = x - location.x;

			if (fX >= overlay->cols)
				break;

			double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

			for (int c = 0; opacity > 0 && c < src->channels(); ++c)
			{
				unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
				unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
				src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
			}
		}
	}
}

void testMix() {

	// Mask with alpha channel
	/*
	Mat surface = mask.clone();
	imshow("surface", surface);
	Mat tmp, alpha;
	cvtColor(mask, tmp, CV_BGRA2GRAY);
	threshold(tmp, alpha, 100, 255, THRESH_BINARY);

	Mat rgb[4];
	split(mask, rgb);

	Mat white(screenHeight, screenWidth, CV_8UC4, Scalar(255, 255, 255, 1));
	Mat dst(screenHeight, screenWidth, CV_8UC4, Scalar(255, 255, 255, 1));

	Mat rgba[4] = { rgb[0],rgb[1],rgb[2], alpha };
	merge(rgba, 4, dst);

	overlayImage(&white, &dst, Point());
	//MixImage(&surface, &white, Point());

	*/

	// http://www.enchantedlearning.com/art/Colormixing.shtml
	// http://stackoverflow.com/questions/6130621/algorithm-for-finding-the-color-between-two-others-in-the-colorspace-of-painte
	//http://lodev.org/cgtutor/color.html
	// Red + Green = Yellow
	int rgb1[] = { 255, 0, 0 };
	int rgb2[] = { 0, 255, 0 };
	int result[3];
	mix(rgb1, rgb2, result);
	cout << "Yellow: R: " << result[0] << " G: " << result[1] << " B: " << result[2] << endl << endl;

	// Red + Blue = Magenta
	int rgb3[] = { 255, 0, 0 };
	int rgb4[] = { 0, 0, 255 };
	mix(rgb3, rgb4, result);
	cout << "Magenta: R: " << result[0] << " G: " << result[1] << " B: " << result[2] << endl << endl;

	//	Blue + Green = Cyan
	int rgb5[] = { 0, 0, 255 };
	int rgb6[] = { 0, 255, 0 };
	mix(rgb5, rgb6, result);
	cout << "Cyan: R: " << result[0] << " G: " << result[1] << " B: " << result[2] << endl << endl;

	//	White + Black = Gray
	int rgb7[] = { 255, 255, 255 };
	int rgb8[] = { 0, 0, 0 };
	mix(rgb7, rgb8, result);
	cout << "Gray: R: " << result[0] << " G: " << result[1] << " B: " << result[2] << endl << endl;

	//	White + Black = Gray
	int rgb9[] = { 255 , 75, 75 };
	int rgb10[] = { 50, 70, 170 };
	mix(rgb9, rgb10, result);
	cout << "Dark Red: R: " << result[0] << " G: " << result[1] << " B: " << result[2] << endl << endl;
}