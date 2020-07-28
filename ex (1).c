#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <dirent.h>
#include <pthread.h> //-lp
#include <sys/mman.h>
#include <math.h> //-lm
#include <sys/ioctl.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "fbtobmp.h"

#define MAXBUF 512

#define ACCELATION(x)   ((((9.8*2*2)/256)/256)*x)
#define MAGNETIC(x)      (0.2*x)

#define LCD_WIDTH 1024
#define LCD_HEIGHT 600
#define LCD_BIT 16

#define EVENT_DEVICE    "/dev/input/event1"
#define EVENT_TYPE      EV_ABS
#define EVENT_CODE_X    53
#define EVENT_CODE_Y    54

int OFFSET(int x, int y);
unsigned short makepixel2(unsigned short red, unsigned short green, unsigned short blue);
void predraw(char *bitmap_name, int xpos, int ypos, int size_x, int size_y);

int fd, rd;
float mag_x = 0, mag_y = 0, mag_z = 0;
float acc_x = 0, acc_y = 0, acc_z = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;
int a = 0;

struct input_event ev;
int size = 0;
int tsFd, clcdFd, ev_size, keyFd = -1;
char clcd_buf[33];
int xpos1 = 0;
int ypos1 = 0;
int on = 1;
int isPressed = 0;
int inputFlag = 1;
int isShow = -1;
int isTouch = 0;
int isStart = -1;
int isReset = 0;
int isSocket = 0;
unsigned short *fbbuffer;

int Startpoint = 0;
int Arrivepoint = 0;
int player_x = 0;
int player_y = 0;
int arrive_x=0;
int arrive_y=0;

float x_v = 0, y_v = 0;
float x_d = 0, y_d = 0;
float x_gacc = 0.0, y_gacc = 0.0;
float x_acc = 0.0, y_acc = 0.0;
float alpha = (float)0.8;
int count = 0;
float sum_x_d = 0, sum_y_d = 0;

float pi = 3.141592;
int inputImage[60][60];
int arriveImage[60][60];
int OutputImage[60][60];
int rad = 0;

unsigned short pixel;

pthread_t p_thread[3];

#define BI_RGB 0
#define BI_RLE4 1
#define BI_RLE8 2
#define BI_BITFIELD 3

struct palette_s
{
	unsigned char blue;
	unsigned char green;
	unsigned char red;
	unsigned char filter;
};

struct bmphandle_s
{
	/* bmp header contents */
	int filesize;
	int reserved;
	int dataoffset;
	int headersize;
	int width;
	int height;
	short nplanes;
	short bpp;
	int compression;
	int bitmapsize;
	int hres;
	int vres;
	int ncolors;
	int importantcolors;

	/* palette, pixel data, getpixel function pointer */
	int npalette;
	int bytes_per_line;
	struct palette_s *palette;
	unsigned char *data;
	unsigned char *encodeddata;
	struct bgrpixel(*getpixel)(bmphandle_t, int, int);
	unsigned bsize_blue, bsize_green, bsize_red;
	unsigned boffset_blue, boffset_green, boffset_red;
};

static struct bgrpixel getpixel_32bpp(bmphandle_t bh, int x, int y)
{
	struct bgrpixel ret;
	unsigned *pdata;
	unsigned *mask = (unsigned *)(bh->palette);
	int offset = (bh->height - y)*bh->bytes_per_line + (x << 2);

	pdata = (unsigned *)(bh->data + offset);

	ret.b = ((*pdata & mask[2]) >> bh->boffset_blue) << (8 - bh->bsize_blue);
	ret.g = ((*pdata & mask[1]) >> bh->boffset_green) << (8 - bh->bsize_green);
	ret.r = ((*pdata & mask[0]) >> bh->boffset_red) << (8 - bh->bsize_red);

	return ret;
}

static struct bgrpixel getpixel_24bpp(bmphandle_t bh, int x, int y)
{
	struct bgrpixel ret;
	unsigned char *pdata;
	int offset = (bh->height - y)*bh->bytes_per_line + x * 3;

	pdata = bh->data + offset;
	ret.b = *pdata;
	ret.g = *(pdata + 1);
	ret.r = *(pdata + 2);

	return ret;
}

static struct bgrpixel getpixel_16bpp(bmphandle_t bh, int x, int y)
{
	/* BI_RGB case */
	struct bgrpixel ret;
	unsigned short *pdata;
	unsigned *mask = (unsigned *)(bh->palette);
	int offset = (bh->height - y)*bh->bytes_per_line + (x << 1);

	pdata = (unsigned short *)(bh->data + offset);
	ret.b = ((*pdata & mask[2]) >> bh->boffset_blue) << (8 - bh->bsize_blue);
	ret.g = ((*pdata & mask[1]) >> bh->boffset_green) << (8 - bh->bsize_green);
	ret.r = ((*pdata & mask[0]) >> bh->boffset_red) << (8 - bh->bsize_red);

	return ret;
}

static struct bgrpixel getpixel_8bpp(bmphandle_t bh, int x, int y)
{
	struct bgrpixel ret;
	unsigned char *pdata;
	int offset = (bh->height - y)*bh->bytes_per_line + x;
	int pixel8;

	pdata = bh->data + offset;
	pixel8 = *pdata;
	/* palette lookup */
	ret.b = bh->palette[pixel8].blue;
	ret.g = bh->palette[pixel8].green;
	ret.r = bh->palette[pixel8].red;

	return ret;
}

static struct bgrpixel getpixel_4bpp(bmphandle_t bh, int x, int y)
{
	struct bgrpixel ret;
	unsigned char *pdata;
	int offset = (bh->height - y)*bh->bytes_per_line + (x >> 1);
	int pixel4;
	int boffset;

	pdata = bh->data + offset;
	boffset = (x & 0x01) << 2;

	pixel4 = (*pdata & (0xF0U >> boffset)) >> (4 - boffset);

	/* palette lookup */
	ret.b = bh->palette[pixel4].blue;
	ret.g = bh->palette[pixel4].green;
	ret.r = bh->palette[pixel4].red;

	return ret;
}

static struct bgrpixel getpixel_1bpp(bmphandle_t bh, int x, int y)
{
	struct bgrpixel ret;
	unsigned char *pdata;
	int offset = (bh->height - y)*bh->bytes_per_line + (x / 8);
	int pixel1;
	int boffset;

	pdata = bh->data + offset;
	boffset = x % 8;

	pixel1 = (*pdata & (0x80U >> boffset)) >> (7 - boffset);

	/* palette lookup */
	ret.b = bh->palette[pixel1].blue;
	ret.g = bh->palette[pixel1].green;
	ret.r = bh->palette[pixel1].red;

	return ret;
}

static int bmp_readheader(FILE *fp, bmphandle_t bh)
{
	int size;
	int remnant;
	unsigned char ID[2];

	/* check ID */
	ID[0] = fgetc(fp);
	ID[1] = fgetc(fp);
	if (ID[0] != 'B' || ID[1] != 'M')
		return -1;
	/* Does not support other IDs such as IC */

	/* reading header */
	size = fread(&bh->filesize, 1, 52, fp);
	if (size != 52)
		return -1;

	bh->npalette = (bh->dataoffset - 54) >> 2;
	/* dword boundary on line end */
	size = bh->width*bh->bpp / 8;
	remnant = size % 4;
	if (remnant == 0)
		bh->bytes_per_line = size;
	else
		bh->bytes_per_line = size + (4 - remnant);

	return 0;
}

static int bmp_readpalette(FILE *fp, bmphandle_t bh)
{
	int size;
	/* alread fp point where palette is */
	size = fread(bh->palette, sizeof(struct palette_s), bh->npalette, fp);
	if (size != bh->npalette)
		return -1;

	return 0;
}

static void rle8_decoding(bmphandle_t bh)
{
	int i, j;
	unsigned char *pdata = bh->data;
	unsigned char *pend = pdata + bh->width*bh->height;
	unsigned char *pedata = bh->encodeddata;
	unsigned char c;

	while (1)
	{
		if (pdata >= pend)
			break;
		c = *pedata++;
		if (c == 0) /* escape */
		{
			c = *pedata++;
			if (c == 0) /* end of line */
			{
				i = pdata - bh->data;
				i %= bh->width;
				for (; i < bh->width; i++)
					pdata++;
			}
			else if (c == 1) /* end of bitmap */
			{
				return;
			}
			else if (c == 2) /* delta */
			{
				j = *pedata++; /* right */
				i = *pedata++; /* down */
				i = j + i*bh->width;
				while (0 < i--)
					pdata++;
			}
			else /* absolute mode */
			{
				c = *pedata++;
				while (0 < c--)
					*pdata++ = *pedata++;
				/* word boundary */
				while ((unsigned)pedata & 0x01)
					pedata++;
			}
		}
		else
		{
			while (c--)
				*pdata++ = *pedata;
			pedata++;
		}
	}
}

static int bmp_readdata(FILE *fp, bmphandle_t bh)
{
	fseek(fp, bh->dataoffset, SEEK_SET);

	if (bh->compression == BI_RGB || bh->compression == BI_BITFIELD)
	{
		bh->data = (unsigned char *)malloc(bh->bitmapsize);
		fread(bh->data, 1, bh->bitmapsize, fp);
	}
	else
	{
		bh->encodeddata = (unsigned char *)malloc(bh->bitmapsize);
		bh->data = (unsigned char *)malloc(bh->width*bh->height*bh->bpp / 8);
		memset(bh->encodeddata, 0, bh->bitmapsize);
		fread(bh->encodeddata, 1, bh->bitmapsize, fp);
		if (bh->compression == BI_RLE4)
			return -1; /* rle4_decoding is not supported ! */
		else
			rle8_decoding(bh);
	}

	return 0;
}

static void calculate_boffset(bmphandle_t bh)
{
	int i;
	unsigned *mask = (unsigned *)(bh->palette);
	unsigned temp;

	/* red */
	temp = mask[0];
	for (i = 0; i < 32; i++)
	{
		if (temp & 0x01)
			break;
		temp >>= 1;
	}
	bh->boffset_red = i;
	for (i = 0; i < 32; i++)
	{
		if (temp & 0x800000UL)
			break;
		temp <<= 1;
	}
	bh->bsize_red = 32 - i;

	/* green */
	temp = mask[1];
	for (i = 0; i < 32; i++)
	{
		if (temp & 0x01)
			break;
		temp >>= 1;
	}
	bh->boffset_green = i;
	for (i = 0; i < 32; i++)
	{
		if (temp & 0x800000UL)
			break;
		temp <<= 1;
	}
	bh->bsize_green = 32 - i;

	/* blue */
	temp = mask[2];
	for (i = 0; i < 32; i++)
	{
		if (temp & 0x01)
			break;
		temp >>= 1;
	}
	bh->boffset_blue = i;
	for (i = 0; i < 32; i++)
	{
		if (temp & 0x800000UL)
			break;
		temp <<= 1;
	}
	bh->bsize_blue = 32 - i;
}

bmphandle_t bmp_open(const char *filename)
{
	bmphandle_t bh;
	FILE *fp;

	fp = fopen(filename, "r+b");
	if (fp == NULL)
		return NULL;

	bh = (bmphandle_t)malloc(sizeof(*bh));
	memset(bh, 0, sizeof(*bh));   /* I don't like calloc */
	if (bmp_readheader(fp, bh))
		goto error;

	if (bh->npalette != 0)
	{
		bh->palette = (struct palette_s *)malloc(sizeof(struct palette_s)*bh->npalette);
		memset(bh->palette, 0, sizeof(struct palette_s)*bh->npalette);
		if (bmp_readpalette(fp, bh))
			goto error;
	}

	if (bmp_readdata(fp, bh))
		goto error;

	switch (bh->bpp)
	{
	case 1:
		bh->getpixel = getpixel_1bpp;
		break;
	case 4:
		bh->getpixel = getpixel_4bpp;
		break;
	case 8:
		bh->getpixel = getpixel_8bpp;
		break;
	case 16:
		bh->getpixel = getpixel_16bpp;
		if (bh->compression == BI_RGB)
		{
			unsigned *mask;
			if (bh->palette != NULL) /* something wrong */
				goto error;
			mask = (unsigned *)malloc(sizeof(unsigned) * 3);
			mask[2] = 0x001F; /* blue mask */
			mask[1] = 0x03E0; /* green mask */
			mask[0] = 0x7C00; /* red mask */
			bh->palette = (struct palette_s *)mask;
			bh->boffset_blue = 0;
			bh->boffset_green = 5;
			bh->boffset_red = 10;
			bh->bsize_blue = 5;
			bh->bsize_green = 5;
			bh->bsize_red = 5;
		}
		else   /* BI_BITFIELD */
		{
			if (bh->palette == NULL) /* something wrong */
				goto error;
			calculate_boffset(bh);
		}
		break;
	case 24:
		bh->getpixel = getpixel_24bpp;
		break;
	case 32:
		bh->getpixel = getpixel_32bpp;
		if (bh->compression == BI_RGB)
		{
			unsigned *mask;
			if (bh->palette != NULL) /* something wrong */
				goto error;

			mask = (unsigned *)malloc(sizeof(unsigned) * 3);
			mask[2] = 0x000000FF; /* blue mask */
			mask[1] = 0x0000FF00; /* green mask */
			mask[0] = 0x00FF0000; /* red mask */
			bh->palette = (struct palette_s *)mask;
			bh->boffset_blue = 0;
			bh->boffset_green = 8;
			bh->boffset_red = 16;
			bh->bsize_blue = 8;
			bh->bsize_green = 8;
			bh->bsize_red = 8;
		}
		else /* BI_BITFILED */
		{
			if (bh->palette == NULL) /* something wrong */
				goto error;
			calculate_boffset(bh);
		}
		break;
	default:
		goto error;
	}

	fclose(fp);
	return bh;
error:
	bmp_close(bh);
	fclose(fp);
	return NULL;
}

void bmp_close(bmphandle_t bh)
{
	if (bh->palette)
		free(bh->palette);
	if (bh->encodeddata)
		free(bh->encodeddata);
	if (bh->data)
		free(bh->data);

	free(bh);
}

int bmp_width(bmphandle_t bh)
{
	return bh->width;
}

int bmp_height(bmphandle_t bh)
{
	return bh->height;
}

struct bgrpixel bmp_getpixel(bmphandle_t bh, int x, int y)
{
	return bh->getpixel(bh, x, y);
}

static unsigned short makepixel(struct fb_var_screeninfo *pfbvar, ubyte r, ubyte g, ubyte b)
{
	unsigned short rnew, gnew, bnew;

	rnew = r >> (8 - pfbvar->red.length);
	gnew = g >> (8 - pfbvar->green.length);
	bnew = b >> (8 - pfbvar->blue.length);

	return (unsigned short)((rnew << pfbvar->red.offset) | (gnew << pfbvar->green.offset) | (bnew << pfbvar->blue.offset));
}

int draw(char *bitmap_name, int xpos, int ypos, int size_x, int size_y)
{
	int fbfd;
	unsigned short *pfbmap;
	unsigned short *buffer;
	struct fb_var_screeninfo fbvar;
	int i, j;
	struct bgrpixel pixel;
	bmphandle_t bh;

	bh = bmp_open(bitmap_name);
	if (bh == NULL)
	{
		fprintf(stderr, "Cannot open bmp file(%s)\n", bitmap_name);
		fprintf(stderr, "File may be not bmp. Or file may be not supported by this program\n");
		exit(0);
	}

	fbfd = open(FBDEVFILE, O_RDWR);
	if (fbfd < 0)
	{
		perror("fbdev open");
		exit(1);
	}

	if (ioctl(fbfd, FBIOGET_VSCREENINFO, &fbvar) < 0)
	{
		perror("fbdev ioctl");
		exit(1);
	}

	if (fbvar.bits_per_pixel != 16)
	{
		fprintf(stderr, "bpp is not 16\n");
		exit(1);
	}

	pfbmap = (unsigned short *)mmap(0, fbvar.xres*fbvar.yres * 16 / 8, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);

	if ((unsigned)pfbmap == (unsigned)-1)
	{
		perror("fbdev mmap");
		exit(1);
	}

	/* main loop */
	for (i = 0; i < MIN(bmp_height(bh), fbvar.yres); i++)
	{
		for (j = 0; j < MIN(bmp_width(bh), fbvar.xres); j++)
		{
			pixel = bmp_getpixel(bh, j, i);
			*(pfbmap + ((i / size_x)*fbvar.xres + ypos * 1024) + ((j / size_y) + xpos)) = makepixel(&fbvar, pixel.r, pixel.g, pixel.b); //i과 j값을 나누면 이미지 사이즈가 줄어들고, 더하면 위치가 정해진다. j는 x 좌표, i는 y좌표
		}
	}
	/* clean up */
	bmp_close(bh);
	munmap(pfbmap, fbvar.xres*fbvar.yres * 16 / 8);
	close(fbfd);

	return 0;
}

void *t_readtouch(void *data)
{
	size = *((int*)data);
	while (1) {
		ev_size = sizeof(ev);
		size = read(tsFd, &ev, ev_size);

		if (size < ev_size) {
			printf("Error size when reading\n");
			close(tsFd);
			exit(0);
		}

		if (ev.type == 1 && isPressed == 0) {   //Press
			isPressed = 1;
		}
		else if (ev.type == 1 && isPressed == 1) {   //Release
			isPressed = 0;
			inputFlag = 0;
		}
		else if (ev.type == 3) {   //
			if (ev.code == EVENT_CODE_X) {
				xpos1 = ev.value;
			}
			else if (ev.code == EVENT_CODE_Y) {
				ypos1 = ev.value;
				size = 0;
			}
		}
		
		if(inputFlag == 0 && ypos1 > 500)
		{
			inputFlag = 1;

			if(xpos1>=0&&xpos1<256)
			{
				Startpoint = 1;
			}
			else if(xpos1>=256&&xpos1<512)
			{
				Arrivepoint = 1;
			}
			else if(xpos1>=512&&xpos1<786)
			{
				isStart = 1;
			}
			else if(xpos1>=786&&xpos1<1024)
			{
				isReset = 1;
				arrive_x = 0;
				arrive_y = 0;
				player_x = 0;
				player_y = 0;
				isStart = 0;  
			//socket thread force quit in main() in while()
			}
		}

		if(Startpoint == 1 && inputFlag == 0)
		{ inputFlag =1;
			Startpoint = 0;
			player_x = xpos1-30;
			player_y = ypos1-30;
		}
		
		if(Arrivepoint == 1 && inputFlag == 0)
		{ inputFlag =1;
			Arrivepoint = 0;
			arrive_x = xpos1-30;
			arrive_y = ypos1-30;
		}
	}
}

void rotate(int InputImage[60][60], double Degree, int Width, int Height)
{
	int x, y;
	int orig_x, orig_y;
	int pixel_var;
	double radian = Degree*pi / 180.0; // (1)
	double cc = cos(radian), ss = sin(-radian);
	double xcenter = (double)Width / 2.0, ycenter = (double)Height / 2.0; // (2)

	for (y = 0; y < Height; y++)
	{
		for (x = 0; x < Width; x++)
		{
			orig_x = (int)(xcenter + ((double)y - ycenter)*ss + ((double)x - xcenter)*cc);
			orig_y = (int)(ycenter + ((double)y - ycenter)*cc - ((double)x - xcenter)*ss);
			pixel = 0; // (3)

			if ((orig_y >= 0 && orig_y < Height) && (orig_x >= 0 && orig_x < Width)) // (4)
				pixel_var = InputImage[orig_y][orig_x]; // (5)
			OutputImage[y][x] = pixel_var; // (6)
		} // x-loop
	} // y-loop

}

void p_copy(int r) //player copy
{
	int i;
	int j;
	pixel = makepixel2(31, 0, 0);


	for (i = 0; i<2 * r; i++)
		for (j = 0; j<2 * r; j++)
			if (r*r>abs((i - r))*abs((i - r)) + abs((j - r))*abs((j - r)))
				inputImage[i][j] = pixel;
}

void Arrive_copy(int r)
{
	int i;
	int j;
	pixel = makepixel2(0, 62, 0);


	for (i = 0; i<2 * r; i++)
		for (j = 0; j<2 * r; j++)
			if (r*r>abs((i - r))*abs((i - r)) + abs((j - r))*abs((j - r)))
				arriveImage[i][j] = pixel;
}

void player(unsigned short *fbdata, int r, int x, int y)
{
	int i;
	int j;
	int offset;

	offset = y * 1024 + x;
	for (i = 0; i<2 * r; i++)
		for (j = 0; j<2 * r; j++)
			if (r*r>abs((i - r))*abs((i - r)) + abs((j - r))*abs((j - r)))
				*(fbdata + offset + i + j * 1024) = inputImage[i][j];
}

void Destination(unsigned short *fbdata, int r, int x, int y)
{
	int i;
	int j;
	int offset;

	offset = y * 1024 + x;
	for (i = 0; i<2 * r; i++)
		for (j = 0; j<2 * r; j++)
			if (r*r>abs((i - r))*abs((i - r)) + abs((j - r))*abs((j - r)))
				*(fbdata + offset + i + j * 1024) = arriveImage[i][j];
}

void show(unsigned short *fbdata) {
	int i = 0;
	if(Startpoint == 1)
		predraw("test1_1.bmp", 0, 0, 1, 1);
	else if(Arrivepoint == 1)
		predraw("test1_2.bmp", 0, 0, 1, 1);
	else if(isStart == 1)
		predraw("test1_3.bmp", 0, 0, 1, 1);
	else if(isReset == 1)
		{
			draw("test1_4.bmp", 0, 0, 1, 1);
			sleep(1);
			isReset = 0;
		}
	else
		predraw("test1.bmp", 0, 0, 1, 1);
	rotate(inputImage, rad, 60, 60);
	if(player_x != 0 && player_y != 0)
		player(fbbuffer, 30, player_x, player_y);
	if(arrive_x != 0 && arrive_y != 0)
		Destination(fbbuffer, 30, arrive_x, arrive_y);
	for (i = 0; i<614400; i++)
		*(fbdata + i) = *(fbbuffer + i);
}

void *t_sensor(void *data) {
	char buffer[MAXBUF];
	struct sockaddr_in server_addr, client_addr;
	char temp[20];
	int server_fd, client_fd;
	char *token;
	int portNum = 7777;
	//server_fd, client_fd : 각 소켓 번호
	int len, msg_size;
	int xtemp,ytemp;
	
	printf("Port Num is %d \n",portNum);

	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{// 소켓 생성
		printf("Server : Can't open stream socket\n");
		exit(0);
	}
	memset(&server_addr, 0x00, sizeof(server_addr));
	//server_Addr 을 NULL로 초기화
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(portNum);
	//server_addr 셋팅

	if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <0)
	{//bind() 호출
		printf("Server : Can't bind local address.\n");
		exit(0);
	}
	if (listen(server_fd, 5) < 0)
	{//소켓을 수동 대기모드로 설정
		printf("Server : Can't listening connect.\n");
		exit(0);
	}
	memset(buffer, 0x00, sizeof(buffer));

	len = sizeof(client_addr);
	client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &len);
	if (client_fd < 0)
	{
		printf("Server: accept failed.\n");
		exit(0);
	}
	
	int iswalk = 0;
	inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
	printf("Server : %s client connected.\n", temp);
	while (1)
	{
		if(isStart==0){
			isSocket=0;
			break;
		}
		msg_size = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
		if (msg_size == -1) {
			printf("Receive error..\n");
			exit(0);
		}

		if (!strncmp(buffer, "Exit", 4)) break;

		token = strtok(buffer, "#");
		mag_x = atof(token);
		token = strtok('\0', "#");
		mag_y = atof(token);
		token = strtok('\0', "#");
		mag_z = atof(token);

		token = strtok('\0', "#");
		acc_x = atof(token);
		token = strtok('\0', "#");
		acc_y = atof(token);
		token = strtok('\0', "#");
		acc_z = atof(token);

		rad = ((int)((atan2(mag_y,mag_x)*180/pi)+270)%360);

		if(sqrt(pow(acc_x,2)+pow(acc_y,2)+pow(acc_z,2)) > 11.5 )
			iswalk = 1;

		if(sqrt(pow(acc_x,2)+pow(acc_y,2)+pow(acc_z,2))<=11.5 && iswalk == 1)
		{	
			iswalk = 0;

			xtemp =player_x;
			xtemp += (int)(1024/13*sin(((float)(rad-45))/180*pi));
			if((xtemp) >1&&(xtemp)<963)
				player_x = xtemp;
			else if(xtemp<=1)
				player_x = 1;
			else if(xtemp>=963)
				player_x = 963;

			ytemp =player_y;
			ytemp -= (int)(500/11.5*cos(((float)(rad-45))/180*pi));
			if((ytemp) > 1 && (ytemp) < 440)
				player_y = ytemp;
			else if (ytemp<=1)
				player_y = 1;
			else if (ytemp>=440)
				player_y = 440;

			
		}
	}
	close(client_fd);
	printf("Server : %s client closed.\n", temp);

	close(server_fd);
}

void predraw(char *bitmap_name, int xpos, int ypos, int size_x, int size_y)
{
	int fbfd;
	unsigned short *pfbmap;
	struct fb_var_screeninfo fbvar;
	int i, j;
	struct bgrpixel pixel;
	bmphandle_t bh;

	bh = bmp_open(bitmap_name);
	if (bh == NULL)
	{
		fprintf(stderr, "Cannot open bmp file(%s)\n", bitmap_name);
		fprintf(stderr, "File may be not bmp. Or file may be not supported by this program\n");
	}

	fbfd = open(FBDEVFILE, O_RDWR);
	if (fbfd < 0)
	{
		perror("fbdev open");
		exit(1);
	}

	if (ioctl(fbfd, FBIOGET_VSCREENINFO, &fbvar) < 0)
	{
		perror("fbdev ioctl");
		exit(1);
	}

	if (fbvar.bits_per_pixel != 16)
	{
		fprintf(stderr, "bpp is not 16\n");
		exit(1);
	}


	if ((unsigned)pfbmap == (unsigned)-1)
	{
		perror("fbdev mmap");
		exit(1);
	}


	for (i = 0; i < MIN(bmp_height(bh), fbvar.yres); i++)
	{
		for (j = 0; j < MIN(bmp_width(bh), fbvar.xres); j++)
		{
			pixel = bmp_getpixel(bh, j, i);
			*(fbbuffer + ((i / size_x)*fbvar.xres + ypos * 1024) + ((j / size_y) + xpos)) = makepixel(&fbvar, pixel.r, pixel.g, pixel.b);
		}
	}
	/* clean up */
	bmp_close(bh);
	close(fbfd);
}

int main()
{
	int fbfd;
	int offset;
	int keyFd = -1;

	int i, j;

	unsigned short *fbdata;
	printf("(Example Code)\nProgram Start!\n");

	// Device File Open
	if ((fbfd = open("/dev/fb0", O_RDWR)) < 0)
	{
		printf("Frame Buffer Device File Open Fail.\n");
		exit(1);
	}
	tsFd = open(EVENT_DEVICE, O_RDONLY);
	if (tsFd == -1) {
		printf("%s is not a vaild device\n", EVENT_DEVICE);
		return EXIT_FAILURE;
	}
	if ((keyFd = open("/dev/fpga_push_switch", O_RDONLY)) < 0) {
		printf("Device open error : /dev/fpga_push_switch\n");
		exit(-1);
	}
	fbbuffer = (unsigned short *)malloc(sizeof(unsigned short) * 614400);
	// initialization

	draw("test1.bmp", 0, 0, 1, 1);
	fbdata = (unsigned short *)mmap(0, LCD_WIDTH*LCD_HEIGHT*LCD_BIT / 8, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);

	p_copy(30);

	int thr_id;

	thr_id = pthread_create(&p_thread[0], NULL, t_readtouch, (void*)&size);
	if (thr_id < 0)
	{
		perror("thread0 create error : \n");
		exit(0);
	}

	

	while (1) {
		if(isStart==1 && isSocket==0){
			isSocket =1;
			thr_id = pthread_create(&p_thread[2], NULL, t_sensor, (void*)&i);   
			if (thr_id < 0)
			{
				perror("thread2 create error : \n");
				exit(0);
			}
			
		}
		/*if(isStart==0){
			isSocket =0;
			pthread_cancel(p_thread[2]); //thread force kill
		}*/

		show(fbdata);

		if(!(player_x==0&&player_y==0) && !(arrive_x==0&&arrive_y==0) && sqrt(pow(arrive_x - player_x,2)+pow(arrive_y - player_y,2)) < 60 )
			printf("ok\n");
	}

	// Memory Return
	munmap(fbdata, LCD_WIDTH*LCD_HEIGHT*LCD_BIT / 8);

	// Device File Close
	close(fbfd); // Frame buffer

	return 0;
}

int OFFSET(int x, int y)
{
	return y*LCD_WIDTH*(LCD_BIT / 8) + x*(LCD_BIT / 8);
}

unsigned short makepixel2(unsigned short red, unsigned short green, unsigned short blue)
{
	return ((red << 11) | (green << 5) | (blue));
}
