#ifndef IMAGE_H
#define IMAGE_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std;

class Colorub {
public:
	unsigned char r, g, b;
	Colorub() {
		this->r = 0;
		this->g = 0;
		this->b = 0;
	}
	Colorub(int r, int g, int b) {
		this->r = r;
		this->g = g;
		this->b = b;
	}
	/* ‘ã“ü */
	Colorub& operator = (const Colorub& cc)
	{
		r = cc.r;
		g = cc.g;
		b = cc.b;
		return *this;
	}
	/* float‚Æ‚ÌŠ|‚¯ŽZ‚ð’è‹` */
	Colorub operator * (float p) {
		return Colorub{ (unsigned char)(this->r * p),
						(unsigned char)(this->g * p),
						(unsigned char)(this->b * p) };
	}
};

class Image {
private:
	int			    width, height;
	vector<Colorub>	pixel;
	string          type = "notype";	//‰æ‘œ‚ÌŽí—Þ

public:
	Image(int w = 0, int h = 0);
	Image(char *filename);
	~Image();

	void		resize(int w, int h);
	int			getWidth();
	int			getHeight();
	Colorub		getPixel(int i, int j);
	Colorub		getPixel(float i, float j);
	void		setPixel(int i, int j, Colorub c);

	/* IO */
	bool		load(char *filename);
	bool        load(std::string filename);
	bool		save(char *filename);
	
	void		setType(string name);
	string		getType();
};


#endif
