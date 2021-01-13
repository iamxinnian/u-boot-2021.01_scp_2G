// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2011 Samsung Electronics
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>

#define BL2_SIZE		(16*1024)
#define IMG_SIZE		(14*1024)
#define FILE_PERM		(S_IRUSR | S_IWUSR | S_IRGRP \
				| S_IWGRP | S_IROTH | S_IWOTH)
/*
* Requirement:
* IROM code reads first 14K bytes from boot device.
* It then calculates the checksum of 14K-4 bytes and compare with data at
* 14K-4 offset.
*
* This function takes two filenames:
* IN  "u-boot-spl.bin" and
* OUT "$(BOARD)-spl.bin as filenames.
* It reads the "u-boot-spl.bin" in 16K buffer.
* It calculates checksum of 14K-4 Bytes and stores at 14K-4 offset in buffer.
* It writes the buffer to "$(BOARD)-spl.bin" file.
*/

int checksumf(char *ifile,char *ofile)
{
	int i, len;
	unsigned char *buffer=NULL;
	int ifd, ofd;
	unsigned int checksum = 0, count;

	if(ifile==NULL || ofile==NULL)
	{
		fprintf(stderr, "input file name or ouput file name is null!\n");
		exit(EXIT_FAILURE);
	}
	ifd = open(ifile, O_RDONLY);
	if (ifd < 0) {
		fprintf(stderr, "Can't open %s: %s\n",
			ifile, strerror(errno));
		exit(EXIT_FAILURE);
	}

	ofd = open(ofile, O_WRONLY | O_CREAT | O_TRUNC, FILE_PERM);
	if (ofd < 0) {
		fprintf(stderr, "Can't open %s: %s\n",
			ofile, strerror(errno));
		if (ifd)
			close(ifd);
		exit(EXIT_FAILURE);
	}

	len = lseek(ifd, 0, SEEK_END);
	lseek(ifd, 0, SEEK_SET);

	count = (len < (IMG_SIZE - 4))
		? len : (IMG_SIZE - 4);
	buffer=(unsigned char *)calloc(IMG_SIZE,sizeof(unsigned char));
	if(buffer==NULL)
	{
		fprintf(stderr, "calloc buffer error: %s\n", strerror(errno));
		if (ifd)
			close(ifd);
		if (ofd)
			close(ofd);
		exit(EXIT_FAILURE);
	}
	if (read(ifd, buffer, count) != count) {
		fprintf(stderr, "Can't read %s: %s\n",
			ifile, strerror(errno));

		if (ifd)
			close(ifd);
		if (ofd)
			close(ofd);
		free(buffer);
		exit(EXIT_FAILURE);
	}

	for (i = 0; i < IMG_SIZE - 4; i++)
		checksum += buffer[i];
	*(unsigned int *)(buffer+IMG_SIZE-4) = checksum;
	if(write(ofd,buffer,IMG_SIZE)!=IMG_SIZE)
	{
		fprintf(stderr, "Can't write %s: %s\n",
			ofile, strerror(errno));

		if (ifd)
			close(ifd);
		if (ofd)
			close(ofd);
		free(buffer);
		exit(EXIT_FAILURE);
	}
	free(buffer);
	if (ifd)
		close(ifd);
	if (ofd)
		close(ofd);

	return EXIT_SUCCESS;
}

int add_padding(char *ifile,char *ofile)
{

	unsigned char *buffer=NULL;
	int ifd, ofd;

	if(ifile==NULL || ofile==NULL)
	{
		fprintf(stderr, "input file name or ouput file name is null!\n");
		exit(EXIT_FAILURE);
	}
	ifd = open(ifile, O_RDONLY);
	if (ifd < 0) {
		fprintf(stderr, "Can't open %s: %s\n",
			ifile, strerror(errno));
		exit(EXIT_FAILURE);
	}

	ofd = open(ofile, O_WRONLY | O_CREAT | O_TRUNC, FILE_PERM);
	if (ofd < 0) {
		fprintf(stderr, "Can't open %s: %s\n",
			ofile, strerror(errno));
		if (ifd)
			close(ifd);
		exit(EXIT_FAILURE);
	}
	buffer=(unsigned char *)calloc(BL2_SIZE,sizeof(char));
	if(buffer==NULL)
	{
		fprintf(stderr, "calloc buffer error: %s\n", strerror(errno));
		if (ifd)
			close(ifd);
		if (ofd)
			close(ofd);
		exit(EXIT_FAILURE);
	}
	if (read(ifd, buffer, IMG_SIZE+256) != IMG_SIZE+256) {
		fprintf(stderr, "Can't read %s: %s\n",
			ifile, strerror(errno));

		if (ifd)
			close(ifd);
		if (ofd)
			close(ofd);
		free(buffer);
		exit(EXIT_FAILURE);
	}
	
	if(write(ofd,buffer,BL2_SIZE)!=BL2_SIZE)
	{
		fprintf(stderr, "Can't write %s: %s\n",
			ofile, strerror(errno));

		if (ifd)
			close(ifd);
		if (ofd)
			close(ofd);
		free(buffer);
		exit(EXIT_FAILURE);
	}
	free(buffer);
	if (ifd)
		close(ifd);
	if (ofd)
		close(ofd);
	return EXIT_SUCCESS;
}

int main(int argc,char **argv)
{
	char *pathback=NULL;
	char *pathwork=NULL;
	char *tmpf1="/tmp/tmpf18358483589235.bin";
	char *tmpf2="/tmp/tmpf23534543565476.bin";
	if (argc != 3) {
		printf(" %d Wrong number of arguments\n", argc);
		exit(EXIT_FAILURE);
	}
	//做校验和
	if(checksumf(argv[1],tmpf1)!=0)
	{
		unlink(tmpf1);
		printf("checksum is failed!\n");
		exit(EXIT_FAILURE);
	}
	//做签名
	pathback=(char *)calloc(2048,sizeof(char));
	pathwork=(char *)calloc(150,sizeof(char));
	if(pathback==NULL || pathwork==NULL)
	{
		fprintf(stderr, "calloc path error: %s\n", strerror(errno));
		if(pathback!=NULL)
			free(pathback);
		if(pathwork!=NULL)
			free(pathwork);
		exit(EXIT_FAILURE);
	}
	if(getcwd(pathback,2047)==NULL)
	{
		printf("getcwd is failed!\n");
		goto erro;
	}
	if(chdir("../CodeSign4SecureBoot_SCP/")==-1)
	{
		printf("chdir is failed!\n");
		goto erro;
	}
	//./codesigner_v21 -v2.1 led_b.bin BL2.bin.signed.4412 Exynos4412_V21.prv -STAGE2	
	sprintf(pathwork,"./codesigner_v21 -v2.1 %s %s Exynos4412_V21.prv -STAGE2",tmpf1,tmpf2);
	if(system(pathwork)==-1)
	{
		unlink(tmpf1);
		printf("codesigner is failed!:%s\n",strerror(errno));
		goto erro;
	}
	if(chdir(pathback)==-1)
	{
		printf("chdir is failed!\n");
		goto erro;
	}
	free(pathback);
	free(pathwork);
	if(unlink(tmpf1)==-1)
	{
		fprintf(stderr, "unlink tmpf1 error: %s\n", strerror(errno));
	}
	//做补充到16K
	if(add_padding(tmpf2,argv[2])!=0)
	{
		unlink(tmpf2);
		printf("add_pading is failed!\n");
		exit(EXIT_FAILURE);
	}	
	if(unlink(tmpf2)==-1)
	{
		fprintf(stderr, "unlink tmpf2 error: %s\n", strerror(errno));
	}
	return EXIT_SUCCESS;
erro:
	free(pathback);
	free(pathwork);
	exit(EXIT_FAILURE);
}
