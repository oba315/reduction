#include "image.h"
#include "bmpIO.h"

//----------------------------------------------------------------------------
Image::Image(int w, int h)
{
	resize(w, h);
}
//----------------------------------------------------------------------------
Image::Image(char *filename)
{
	load(filename);
}
//----------------------------------------------------------------------------
Image::~Image()
{
}
//----------------------------------------------------------------------------
void Image::resize(int w, int h)
{
	width = w;
	height = h;
	if(w*h > 0) pixel.resize(width*height);
}
//----------------------------------------------------------------------------
int Image::getWidth()
{
	return(width);
}
//----------------------------------------------------------------------------
int Image::getHeight()
{
	return(height);
}
//----------------------------------------------------------------------------
Colorub Image::getPixel(int i, int j)
{
	return(pixel[width*j+i]);
}
Colorub Image::getPixel(float i, float j) {
	if (0<=i&&i<=1 && 0<=j&&j<= 1) {
		int u = int(width * i);
		int v = int(height * j);
		return(pixel[width * v + u]);
	}
	else std::cout << "ERROR(image.cpp) : imageのピクセルに対する不正なアクセスです．" << "u:" << i << " v:" << j << "\nRayの正規化を確認して下さい．"<< std::endl;
}
//----------------------------------------------------------------------------
void Image::setPixel(int i, int j, Colorub c)
{
	pixel[width*j+i] = c;
}
//----------------------------------------------------------------------------
bool Image::save(char *filename)
{
	vector<unsigned char> tmp_pixel(width*height*3);
	int k=0;
	for(int i=0; i<width*height; i++) {
		tmp_pixel[k + 0] = pixel[i].b;
		tmp_pixel[k + 1] = pixel[i].g;
		tmp_pixel[k + 2] = pixel[i].r;
		k += 3;
	}
	return(writeBmpImage(filename, width, height, 3, &(tmp_pixel[0])));
}
#include <igl/png/writePNG.h>

bool Image::save_png(char* filename) {
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(height, width);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(height, width);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(height, width);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(height, width);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			R(i, j) = pixel[j * width + i].r;
			G(i, j) = pixel[j * width + i].g;
			B(i, j) = pixel[j * width + i].b;
			A(i, j) = 255;
		}
	}
	igl::png::writePNG(R, G, B, A, filename);
	return 1;
}
//----------------------------------------------------------------------------
bool Image::load(char *filename)
{
	unsigned char* tmp_pixel = NULL;
	int w, h, nc;
	bool flag;
	if((flag = readBmpImage(filename, &w, &h, &nc, &tmp_pixel))) {
		resize(w, h);
		int k = 0;
		for(int i=0; i<width*height; i++) {
			pixel[i].b = tmp_pixel[k + 0];
			pixel[i].g = tmp_pixel[k + 1];
			pixel[i].r = tmp_pixel[k + 2];
			k += 3;
		}
		delete [] tmp_pixel;
	}

	return(flag);
}
bool Image::load(std::string filename) {
	char* temppath = new char[filename.size() + 1];
	std::char_traits<char>::copy(temppath, filename.c_str(), filename.size() + 1);
	load(temppath) == false;
}
//----------------------------------------------------------------------------
void Image::setType(string name) {
	type = name;
}
string Image::getType() {
	return(type);
}
