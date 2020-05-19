#include <windows.h>
#include <stdio.h>
#include <vector>
#pragma warning(disable: 4996)

//----------------------------------------------------------------------------
typedef struct {
    unsigned char      bfType[2];
    unsigned long int  bfSize;
    unsigned short int bfReserved1;
    unsigned short int bfReserved2;
    unsigned long int  bfOffBits;
} BMPFILEHEADER;

typedef struct {
    unsigned long int  biSize;
    unsigned long int  biWidth;
    unsigned long int  biHeight;
    unsigned short int biPlanes;
    unsigned short int biBitCount;
    unsigned long int  biCompression;
    unsigned long int  biSizeImage;
    unsigned long int  biXPelsPerMeter;
    unsigned long int  biYPelsPerMeter;
    unsigned long int  biClrUsed;
    unsigned long int  biClrImportant;
} BMPINFOHEADER;

BMPFILEHEADER bmFileHeader;
BMPINFOHEADER bmInfoHeader;

#define NUM_PLANE 3

#define M_PI 3.1415926

//----------------------------------------------------------------------------
//�r�b�g�}�b�v�t�@�C���̓ǂݍ���
bool readBmpImage(char *filename, int *width, int *height, int *nchannel, unsigned char **pixel) {

    FILE *fp;

    /* Open the bmp file in binay mode */
    if((fp = fopen(filename, "rb")) == NULL) {
        fprintf(stderr, "can't open the file %s.\n",filename);
        return(false);
    }
    
    /* Read bmp file header */
    fread(bmFileHeader.bfType, sizeof(char), 2, fp);
    fread(&bmFileHeader.bfSize, sizeof(unsigned long int), 1, fp);
    fread(&bmFileHeader.bfReserved1, sizeof(unsigned short int), 1, fp);
    fread(&bmFileHeader.bfReserved2, sizeof(unsigned short int), 1, fp);
    fread(&bmFileHeader.bfOffBits, sizeof(unsigned long int), 1, fp);

	/* Read bmp information header */
    fread(&bmInfoHeader.biSize, sizeof(unsigned long int), 1, fp);
    fread(&bmInfoHeader.biWidth, sizeof(unsigned long int), 1, fp);
    fread(&bmInfoHeader.biHeight, sizeof(unsigned long int), 1, fp);
    fread(&bmInfoHeader.biPlanes, sizeof(unsigned short int), 1, fp);
    fread(&bmInfoHeader.biBitCount, sizeof(unsigned short int), 1, fp);		//�F�[�x
    fread(&bmInfoHeader.biCompression, sizeof(unsigned long int), 1, fp);
    fread(&bmInfoHeader.biSizeImage, sizeof(unsigned long int), 1, fp);
    fread(&bmInfoHeader.biXPelsPerMeter, sizeof(unsigned long int), 1, fp);
    fread(&bmInfoHeader.biYPelsPerMeter, sizeof(unsigned long int), 1, fp);
    fread(&bmInfoHeader.biClrUsed, sizeof(unsigned long int), 1, fp);
    fread(&bmInfoHeader.biClrImportant, sizeof(unsigned long int), 1, fp);

    *width = (int)bmInfoHeader.biWidth;
    *height = (int)bmInfoHeader.biHeight;
	*nchannel = NUM_PLANE;

	// ���݁A�摜�T�C�Y�ɐ�������B
	if (*width % 8 != 0 || *height % 8 != 0) {
		fprintf(stderr, "Image size is not 8X.\n");
		return(false);
	}

	int bytes = (*width) * (*height) * sizeof(unsigned char) * NUM_PLANE;
	if (*pixel == NULL) {
		if ((*pixel = (unsigned char*)malloc(bytes)) == NULL) {
			fprintf(stderr, "can't malloc pixel.\n");
			return(false);
		}
	}

	if (bmInfoHeader.biBitCount == 24) {
		/* �t�@�C������s�N�Z���f�[�^����C��(bytes�o�C�g��)�ǂ݂��݂܂� */
		fread(*pixel, sizeof(unsigned char) * NUM_PLANE, int(*width) * int(*height), fp);
	}
	// �F�[�x:32 : �A���t�@���܂�BMP
	else if (bmInfoHeader.biBitCount == 32) {
		/* �A���t�@���͍폜����܂��D */
		std::vector<std::vector<unsigned char>> pixel_temp(int(*width) * int(*height), std::vector<unsigned char>(4));
		fread(&pixel_temp[0], sizeof(unsigned char) * 4, int(*width) * int(*height), fp);
		for (int i = 0; i < (*width) * (*height); i++) {
			for (int j = 0; j < 3; j++) {
				*pixel[j] = pixel_temp[i][j];
			}
		}
	}
	else {
		fprintf(stderr, "color_depth is inappropriate.\n");
		return(false);
	}
    

	return(true);
}
//----------------------------------------------------------------------------
//�r�b�g�}�b�v�t�@�C���̏����o��
bool writeBmpImage(char* filename, int width, int height, int nchannel, unsigned char *pixel)
{
	// �o�̓t�@�C�����J��
	FILE *fout;
	int unit, widthbytes, sizeimage, h;
	char* outbits;
	char* srcbits;
	BITMAPFILEHEADER bf;
	BITMAPINFOHEADER bmih;

	if(nchannel != NUM_PLANE) {
		printf("nChannel != 3\n");
		return(false);
	}

	fout = fopen(filename, "wb");

	// �T�C�Y���v�Z
	unit = NUM_PLANE; //bpp >> 3; // �s�N�Z������̃o�C�g��
	widthbytes = ((unit * width) + 3) & ~3; // 4�o�C�g���E�ɍ��킹��
	sizeimage = widthbytes * height;

	// �t�@�C���w�b�_���o��
	bf.bfType    = 0x4d42;
	bf.bfOffBits = sizeof(bf) + sizeof(BITMAPINFOHEADER);
	bf.bfSize    = bf.bfOffBits + sizeimage;
	bf.bfReserved1 = 0;
	bf.bfReserved2 = 0;
	fwrite((char*)&bf, sizeof(bf), 1, fout);

	// �r�b�g�}�b�v�w�b�_���o��
	bmih.biSize          = sizeof(bmih); 
	bmih.biWidth         = width;
	bmih.biHeight        = height;
	bmih.biPlanes        = 1;
	bmih.biBitCount      = NUM_PLANE*8;
	bmih.biCompression   = BI_RGB; 
	bmih.biSizeImage     = sizeimage;
	bmih.biXPelsPerMeter = 0; 
	bmih.biYPelsPerMeter = 0; 
	bmih.biClrUsed       = 0;  
	bmih.biClrImportant  = 0; 
	fwrite((char*)&bmih, sizeof(bmih), 1, fout);

	// �f�[�^���o��
	outbits = (char *)malloc(sizeof(char)*widthbytes); // 1�s���̃��������m��
	srcbits = (char*)pixel;
	for (h = 0; h < height; h++) {
		memcpy(outbits, srcbits, unit * width);
		fwrite(outbits, sizeof(char), widthbytes, fout);
		srcbits += unit * width; 
	}
	free(outbits);

	// �t�@�C�������
	fclose(fout);

	return(true);
}
